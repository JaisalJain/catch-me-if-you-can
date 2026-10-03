"""M6: the full loop as one explicit state machine (plain Python).

    ros2 run catch_tasks run_demo [--seed 1] [--out log/m6/run_seed1.json]

    DETECT -> PICK -> LOAD -> PREPOSITION -> THROW -> CATCH -> COLOR -> IN_HAND -> PLACE -> VERIFY -> DETECT ...
    until A's pallet is empty (all cubes sorted on B's side) -> DONE

v2 (default): A throws a random entry of the throw library (catch_throw, seeded) with release noise (--noise,
default medium); B catches it vision-only from its ready pose (/reactive_catch: catch_tracker estimates, never A's
plan; --mode pinch|cup). --v1 runs the v1 loop (configured throw, B pre-positions from A's plan).

* DETECT: overhead camera A (fresh frame); the cube to throw is chosen at random (seeded) among those detected.
* PICK / LOAD: A picks it (MoveIt), turns palm-up, loosens so the cube drops onto the palm insert, re-grips and
  moves to the stroke start.
* PREPOSITION: /catch goal with the throw-plan preview; B goes to its cup pose and reports "ready" before A throws.
* THROW: /throw (plan published latched on /throw_plan before the stroke). CATCH: B closes on palm contact.
* COLOR: B's wrist camera decides the color (A's color is logged as a cross-check only).
* IN_HAND: wrist-camera estimate of the cube pose in B's hand. PLACE: corrected top-down place in that color's slot.
* VERIFY: overhead camera B sees a cube of that color in that slot.
* Failure handling: a dropped / missed cube is logged; the scene is reset for that cube (sim teleport back to its
  slot on A's pallet), both arms go home, and the loop continues (at most max_attempts_per_cube per cube).

Decisions use only the robots' own sensing (cameras, joint states, palm force). Ground truth is never read here.
"""
import argparse
import json
import os
import random
import threading
import time
from enum import Enum

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from catch_interfaces.action import Catch, ReactiveCatch, Throw
from catch_interfaces.msg import CubeDetectionArray
from catch_interfaces.srv import GetColor, GetInHandPose, GetThrowPlan, SpawnCube
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from catch_kinematics.transforms import make_tf, rot_x
from catch_sim.scene_config import slot_position
from catch_sim.sim_clock import offload_sim_clock
from catch_throw.throw_node import build_plan

from .moveit_client import _wait
from .skills import Skills


class S(Enum):
    DETECT = "detect"
    PICK = "pick"
    LOAD = "load"
    PREPOSITION = "preposition"
    THROW = "throw"
    COLOR = "color"
    IN_HAND = "in_hand"
    PLACE = "place"
    VERIFY = "verify"
    RECOVER = "recover"
    DONE = "done"


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def quat_yaw(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class Demo:
    def __init__(self, seed, fault_throw=0, v1=False, noise="medium", library="pinch", mode="pinch"):
        self.node = rclpy.create_node("run_demo", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        ex = MultiThreadedExecutor()
        ex.add_node(self.node)
        offload_sim_clock(self.node)          # /clock off the MultiThreadedExecutor (catch_sim.sim_clock)
        threading.Thread(target=ex.spin, daemon=True).start()
        share = get_package_share_directory("catch_tasks")
        self.cfg = yaml.safe_load(open(os.path.join(share, "config", "demo.yaml")))
        self.seed = self.cfg["seed"] if seed is None else seed
        # string seed (hashed): small integer seeds give near-identical first draws in Python's Mersenne Twister
        # (seeds 1, 2, 3 all picked blue, green, red)
        self.rng = random.Random(f"catch-demo-{self.seed}")
        self.sk = Skills(self.node)
        self.mi = self.sk.mi
        self.poses = yaml.safe_load(open(get_package_share_directory("catch_description") + "/config/poses.yaml"))
        self.plan, *_ = build_plan()
        self.release = yaml.safe_load(open(get_package_share_directory("catch_throw") + "/config/throw.yaml"))["throw"]["release"]
        self.det = {}
        for cam in ("overhead_a", "overhead_b"):
            self.node.create_subscription(CubeDetectionArray, f"/perception/{cam}/cubes",
                                          lambda m, c=cam: self.det.__setitem__(c, m), 1)
        self.throw = ActionClient(self.node, Throw, "/throw")
        self.catch = ActionClient(self.node, Catch, "/catch")
        # v2 (default): random library throws with release noise, vision-only reactive catch (B never sees A's plan)
        self.v1, self.noise, self.library, self.catch_mode = v1, noise, library, mode
        self.rcatch = ActionClient(self.node, ReactiveCatch, "/reactive_catch")
        self.entry_rng = np.random.default_rng(random.Random(f"catch-v2-{self.seed}").getrandbits(32))   # reproducible
        self.pv = None              # selected library throw (v2)
        self.preview = self.node.create_client(GetThrowPlan, "/throw/preview")
        self.get_color = self.node.create_client(GetColor, "/perception/get_color")
        self.in_hand = self.node.create_client(GetInHandPose, "/perception/in_hand_pose")
        self.spawn = self.node.create_client(SpawnCube, "/scene/spawn_cube")
        # fault injection (off by default): on throw number `fault_throw` the throw node loosens the grip late
        # (t_loosen = fault_t_loosen) -> slow release, the cube misses B's palm. Exercises the recovery path.
        self.fault_throw = fault_throw
        self.set_throw_param = self.node.create_client(SetParameters, "/throw_node/set_parameters")
        self.events = []
        self.t_start = None
        # per-cube context
        self.target = None          # detection chosen on A's pallet
        self.a_color = ""           # A's color for the cube (cross-check only)
        self.b_color = ""           # B's own color reading
        self.T_inhand = None
        self.throws = {}            # A color -> throws so far
        self.attempts = {}          # A color -> pick attempts so far (bounded by max_attempts_per_cube)
        self.phase = None

    # ------------------------------------------------------------------ utils
    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, dt):
        t = self.now() + dt
        while self.now() < t:
            time.sleep(0.005)

    def log(self, state, msg, **kw):
        t = self.now() - (self.t_start or self.now())
        ev = dict(t=round(t, 2), t_sim=round(self.now(), 3), state=state.value, msg=msg, **kw)
        self.events.append(ev)
        self.node.get_logger().info(f"[{t:7.2f} s] {state.value:>11}: {msg}")

    def fresh_detection(self, cam, min_conf=None):
        t_req = self.now() + 0.15                     # a frame rendered after the arms stopped
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.cfg["detect_timeout"]:
            d = self.det.get(cam)
            if d is not None and stamp(d.header) >= t_req:
                mc = self.cfg["detect_min_confidence"] if min_conf is None else min_conf
                return [x for x in d.detections if x.confidence >= mc]
            time.sleep(0.02)
        return None

    def home(self, robots="ab"):
        for r in robots:
            if np.max(np.abs(self.mi.joints(r) - self.poses[r]["home"])) > 0.01:
                self.mi.move_joints(r, self.poses[r]["home"])

    # ------------------------------------------------------------------ states
    def s_detect(self):
        self.home("a")
        dets = self.fresh_detection("overhead_a")
        if dets is None:
            self.log(S.DETECT, "no fresh detection from overhead_a")
            return S.DETECT
        pallet = self.sk.scene["pallets"]["a"]["center"]
        on_a = [d for d in dets if abs(d.pose.position.x - pallet[0]) < 0.2 and abs(d.pose.position.y - pallet[1]) < 0.1]
        remaining = [d for d in on_a if self.attempts.get(d.color, 0) < self.cfg["max_attempts_per_cube"]]
        if not remaining:
            self.log(S.DETECT, f"A's pallet: {sorted(d.color for d in on_a) or 'empty'} -> nothing left to throw")
            return S.DONE
        remaining.sort(key=lambda d: d.color)        # deterministic order before the seeded choice
        self.target = self.rng.choice(remaining)
        self.a_color = self.target.color
        p = self.target.pose.position
        self.log(S.DETECT, f"{len(on_a)} cube(s) on A's pallet {sorted(d.color for d in on_a)}; random pick: "
                           f"{self.a_color} at ({p.x:.3f}, {p.y:.3f})", a_color=self.a_color)
        return S.PICK

    def s_pick(self):
        d = self.target
        self.attempts[self.a_color] = self.attempts.get(self.a_color, 0) + 1
        pos = np.array([d.pose.position.x, d.pose.position.y, d.pose.position.z])
        for k in range(self.cfg["max_pick_tries"]):
            r = self.sk.pick("a", pos, quat_yaw(d.pose.orientation), cube_id=self.a_color)
            self.log(S.PICK, r.message, ok=r.ok)
            if r.ok:
                return S.LOAD
            self.sk._grip("a", 0.0)
            self.home("a")
        return S.RECOVER

    def s_load(self):
        sk, c = self.sk, self.a_color
        ok, msg = self.mi.move_joints("a", self.poses["a"]["palm_up"])
        if not ok:
            self.log(S.LOAD, f"palm_up failed: {msg}")
            return S.RECOVER
        sk.detach_cube("a", c)
        sk._grip("a", self.release["loosen_to"])       # cube drops onto the palm insert
        self.sleep_sim(self.cfg["reseat_settle"])
        sk._grip("a", sk.cfg["gripper_close"])
        held, k = sk.in_hand("a")
        if not held:
            self.log(S.LOAD, f"cube lost while re-seating (knuckle {k:.3f})")
            return S.RECOVER
        sk.attach_cube("a", c, offset_z=0.0)
        q_start = self.plan.q_start
        if not self.v1:
            from catch_throw.library import Library
            entry = Library(self.library).draw(self.entry_rng)
            self.pv = _wait(self.preview.call_async(GetThrowPlan.Request(library=self.library, entry=entry)), 10)
            if self.pv is None or not self.pv.success:
                self.log(S.LOAD, f"throw library entry {entry} not available")
                return S.RECOVER
            q_start = np.array(self.pv.plan.stroke_start_q)
            tgt = self.pv.plan.catch_point
            self.log(S.LOAD, f"random throw: {self.library} library entry {entry} (target ({tgt.x:.2f}, {tgt.y:.2f}, {tgt.z:.2f}), "
                             f"flight {self.pv.plan.flight_time:.3f} s), release noise {self.noise}", entry=entry)
        ok, msg = self.mi.move_joints("a", q_start)
        self.log(S.LOAD, "cube seated on the palm, A at the stroke start" if ok else f"stroke start failed: {msg}")
        return S.PREPOSITION if ok else S.RECOVER

    def s_preposition(self):
        if not self.v1:
            self.rcatch.wait_for_server(timeout_sec=10)
            self.phase = None
            self.catch_handle = _wait(self.rcatch.send_goal_async(
                ReactiveCatch.Goal(mode=self.catch_mode, timeout=8.0),
                feedback_callback=lambda f: setattr(self, "phase", f.feedback.phase)), 10)
            t0 = time.monotonic()
            while self.phase != "ready" and time.monotonic() - t0 < 60:
                time.sleep(0.01)
            if self.phase != "ready":
                r = _wait(self.catch_handle.get_result_async(), 5)
                self.log(S.PREPOSITION, f"B not ready: {r.result.message if r else 'timeout'}")
                return S.RECOVER
            self.log(S.PREPOSITION, "B at its ready pose (fingers at the 56 mm pre-gap), watching the cameras only")
            return S.THROW
        pv = _wait(self.preview.call_async(GetThrowPlan.Request()), 10)
        self.catch.wait_for_server(timeout_sec=10)
        self.phase = None
        self.catch_handle = _wait(self.catch.send_goal_async(
            Catch.Goal(plan=pv.plan, trigger="force"),
            feedback_callback=lambda f: setattr(self, "phase", f.feedback.phase)), 10)
        t0 = time.monotonic()
        while self.phase != "ready" and time.monotonic() - t0 < 60:
            time.sleep(0.01)
        if self.phase != "ready":
            r = _wait(self.catch_handle.get_result_async(), 5)
            self.log(S.PREPOSITION, f"B not ready: {r.result.message if r else 'timeout'}")
            return S.RECOVER
        self.log(S.PREPOSITION, "B in its cup pose, fingers open, armed for the live throw plan")
        return S.THROW

    def _t_loosen(self, value):
        p = Parameter(name="t_loosen", value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=float(value)))
        _wait(self.set_throw_param.call_async(SetParameters.Request(parameters=[p])), 5)

    def s_throw(self):
        self.throws[self.a_color] = self.throws.get(self.a_color, 0) + 1
        n_throw = sum(self.throws.values())
        inject = self.fault_throw and n_throw == self.fault_throw
        if inject:
            self._t_loosen(self.cfg["fault_t_loosen"])
            self.log(S.THROW, f"FAULT INJECTED (test of the recovery path): throw #{n_throw} loosens the grip at "
                              f"t = {self.cfg['fault_t_loosen']} s instead of {self.release['t_loosen']} s", injected=True)
        if self.v1:
            goal = Throw.Goal(cube_id=f"cube_{self.a_color}")
        else:
            goal = Throw.Goal(cube_id=f"cube_{self.a_color}", library=self.library, entry=self.pv.plan.library_entry,
                              noise=self.noise, noise_seed=int(self.entry_rng.integers(2 ** 31)))
        th = _wait(self.throw.send_goal_async(goal), 10)
        cr = _wait(self.catch_handle.get_result_async(), 20)
        tr = _wait(th.get_result_async(), 20)
        if inject:
            self._t_loosen(self.release["t_loosen"])
        self.sk.detach_cube("a", self.a_color)
        self.log(S.THROW, f"throw #{self.throws[self.a_color]} of the {self.a_color} cube: "
                          f"{tr.result.message if tr else 'timeout'}")
        caught = bool(cr and cr.result.success)
        extra = {}
        if not self.v1 and cr:
            extra = dict(trigger=cr.result.trigger, aborted=cr.result.aborted, n_replans=cr.result.n_replans)
        self.log(S.THROW, f"catch: {cr.result.message if cr else 'timeout'}", caught=caught, **extra)
        self.sk._grip("a", 0.0)
        self.home("a")              # sequential: one move_group goal at a time
        return S.COLOR if caught else S.RECOVER

    def s_color(self):
        r = _wait(self.get_color.call_async(GetColor.Request(camera="")), 10)
        if not (r and r.success):
            self.log(S.COLOR, f"no color in B's hand ({r.message if r else 'timeout'}): treat as dropped")
            return S.RECOVER
        self.b_color = r.color
        agree = "agrees with" if r.color == self.a_color else "DISAGREES with"
        self.log(S.COLOR, f"B sees {r.color} ({r.confidence:.3f}); {agree} A's {self.a_color} (cross-check only)",
                 b_color=r.color)
        return S.IN_HAND

    def s_in_hand(self):
        r = _wait(self.in_hand.call_async(GetInHandPose.Request(color=self.b_color)), 10)
        if r and r.success:
            self.T_inhand = make_tf(rot_x(r.angle), [r.pose.position.x, r.pose.position.y, r.pose.position.z])
            self.log(S.IN_HAND, r.message)
        else:
            self.T_inhand = make_tf(p=[0.0, 0.0, 0.0])
            self.log(S.IN_HAND, f"no estimate ({r.message if r else 'timeout'}): place assuming the nominal pose")
        return S.PLACE

    def s_place(self):
        held, k = self.sk.in_hand("b")
        if not held:
            self.log(S.PLACE, f"cube no longer in B's hand (knuckle {k:.3f})")
            return S.RECOVER
        self.sk.attach_cube("b", self.b_color, offset_z=float(self.T_inhand[2, 3]))
        r = self.sk.place_held("b", self.b_color, "b", self.T_inhand, cube_id=self.b_color, via=self.poses["b"]["home"])
        self.log(S.PLACE, f"{self.b_color} slot on B's pallet: {r.message}", ok=r.ok)
        self.home("b")
        return S.VERIFY

    def s_verify(self):
        slot = slot_position(self.sk.scene, "b", self.b_color)
        seen = []
        for k in range(self.cfg["verify_frames"]):           # a few fresh frames before declaring a failure
            dets = self.fresh_detection("overhead_b", self.cfg["verify_min_confidence"]) or []
            seen = [(d, np.hypot(d.pose.position.x - slot[0], d.pose.position.y - slot[1]))
                    for d in dets if d.color == self.b_color]
            hit = [(d, r) for d, r in seen if r < self.cfg["verify_tolerance"]]
            if hit:
                d = hit[0][0]
                self.log(S.VERIFY, f"overhead B: {self.b_color} cube in its slot (offset "
                                   f"{1000 * (d.pose.position.x - slot[0]):+.1f}, {1000 * (d.pose.position.y - slot[1]):+.1f} mm, "
                                   f"confidence {d.confidence:.2f})", sorted=self.b_color)
                return S.DETECT
        cand = ", ".join(f"({d.pose.position.x:.3f}, {d.pose.position.y:.3f}) conf {d.confidence:.2f}" for d, _ in seen)
        self.log(S.VERIFY, f"overhead B does not see the {self.b_color} cube in its slot "
                           f"({self.cfg['verify_frames']} frames; {self.b_color} detections: {cand or 'none'})")
        return S.RECOVER

    def s_recover(self):
        """Drop / failure: log it, reset the scene for this cube (sim: back to its slot on A's pallet), arms home."""
        c = self.a_color
        self.log(S.RECOVER, f"failure with the {c} cube: resetting it to A's pallet and continuing", dropped=c)
        self.sk.detach_cube("a", c)
        self.sk.detach_cube("b", c)
        self.sk._grip("b", 0.0)
        self.sk._grip("a", 0.0)
        self.home("ab")
        _wait(self.spawn.call_async(SpawnCube.Request(color=c, where="home")), 10)
        self.sleep_sim(0.5)
        return S.DETECT

    # ------------------------------------------------------------------ loop
    def run(self):
        assert self.mi.wait_ready(60), "move_group not ready"
        self.t_start = self.now()
        self.log(S.DETECT, f"start (seed {self.seed})")
        self.sk._grip("b", 0.0)
        self.home("ab")
        handlers = {S.DETECT: self.s_detect, S.PICK: self.s_pick, S.LOAD: self.s_load,
                    S.PREPOSITION: self.s_preposition, S.THROW: self.s_throw, S.COLOR: self.s_color,
                    S.IN_HAND: self.s_in_hand, S.PLACE: self.s_place, S.VERIFY: self.s_verify,
                    S.RECOVER: self.s_recover}
        state = S.DETECT
        while state != S.DONE:
            state = handlers[state]()
        sorted_ = [e["sorted"] for e in self.events if "sorted" in e]
        drops = [e["dropped"] for e in self.events if "dropped" in e]
        summary = dict(seed=self.seed, version="v1" if self.v1 else "v2", noise=None if self.v1 else self.noise,
                       catch_mode="cup (plan)" if self.v1 else self.catch_mode,
                       duration_s=round(self.now() - self.t_start, 1), sorted=sorted_,
                       throws=sum(self.throws.values()), throws_per_cube=self.throws, attempts=self.attempts,
                       recoveries=drops,
                       complete=sorted(sorted_) == ["blue", "green", "red"])
        self.log(S.DONE, json.dumps(summary))
        return summary


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--fault-throw", type=int, default=0, help="inject a late grip loosen on this throw (0 = off)")
    ap.add_argument("--v1", action="store_true", help="v1 loop: the configured throw, B pre-positions from A's plan")
    ap.add_argument("--noise", default="medium", help="v2 release noise: none | low | medium | high")
    ap.add_argument("--mode", default="pinch", choices=["pinch", "cup"], help="v2 catch mode")
    ap.add_argument("--out", default="")
    args, _ = ap.parse_known_args()
    rclpy.init()
    lib = "cup" if args.mode == "cup" else "pinch"
    d = Demo(args.seed, args.fault_throw, v1=args.v1, noise=args.noise, library=lib, mode=args.mode)
    summary = d.run()
    print("RUN", json.dumps(summary), flush=True)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        json.dump(dict(summary=summary, events=d.events), open(args.out, "w"), indent=1)
    rclpy.shutdown()
