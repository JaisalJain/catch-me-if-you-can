"""Robot A throw executor: action server /throw (catch_interfaces/Throw).

On a goal it (1) generates the toss trajectory from catch_throw/config/throw.yaml (catch_throw.generator),
(2) checks that arm A is at the stroke start (the caller moves it there with MoveIt), (3) publishes the
ThrowPlan (latched) with the predicted release time in sim time, (4) sends the time-parameterised joint
trajectory directly to arm_a_controller with a future start stamp and (5) schedules the gripper loosen at
t_loosen after the stroke start as a time-stamped trajectory on gripper_a_controller (a JTC), so the loosen
instant is executed by the control loop in sim time. The cube is released by the arm's deceleration, not by
the fingers.
"""
import os
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, Vector3
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from trajectory_msgs.msg import JointTrajectoryPoint

from catch_ballistics import velocity_for_launch_angle
from catch_interfaces.action import Throw
from catch_interfaces.msg import ThrowPlan
from catch_interfaces.srv import GetThrowPlan
from catch_kinematics import UR5e
from catch_kinematics.transforms import make_tf, rot_z
from catch_sim import gripper_model
from catch_sim.joint_state_cache import JointStateCache
from catch_sim.sim_clock import offload_sim_clock

from .generator import generate_throw
from .library import Library, load_v2, sample_noise
from .profile import ProfileParams

JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def load_yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def to_time(t):
    sec = int(np.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def to_dur(t):
    sec = int(np.floor(t))
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def make_robot(scene=None):
    scene = scene or load_yaml("catch_description", "config/scene.yaml")
    r = scene["robots"]["a"]
    base = [r["base_xy"][0], r["base_xy"][1], scene["table"]["top_z"] + r.get("pedestal_height", 0.0)]
    tcp = scene["gripper"]["mount_offset"] + scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2
    return UR5e(make_tf(rot_z(r["base_yaw"]), base), make_tf(p=[0, 0, tcp]))


def build_plan(cfg=None, scene=None):
    """Plan the configured (v1) throw (pure computation, also used by tests and the evaluation tool)."""
    cfg = cfg or load_yaml("catch_throw", "config/throw.yaml")["throw"]
    robot = make_robot(scene)
    p, c = np.array(cfg["release_point"]), np.array(cfg["catch_point"])
    v, T = velocity_for_launch_angle(p, c, np.radians(cfg["launch_angle_deg"]))
    plan = generate_throw(robot, p, v, cfg["q_release_seed"], ProfileParams(**cfg["profile"]),
                          finger_axis=cfg["finger_axis"], pitch_rate=cfg["pitch_rate"], dt=cfg["sample_dt"])
    return plan, T, c, cfg


def load_feedforward(n_samples):
    """Feed-forward torque table from tools/throw_feedforward.py; None (with reason) if missing or stale."""
    import hashlib
    share = get_package_share_directory("catch_throw")
    path = os.path.join(share, "config", "throw_ff.yaml")
    if not os.path.exists(path):
        return None, "no throw_ff.yaml"
    ff = yaml.safe_load(open(path))
    cfg = yaml.safe_load(open(os.path.join(share, "config", "throw.yaml")))["throw"]
    cfg.pop("release", None)            # gripper timing does not change the trajectory
    h = hashlib.sha256(yaml.safe_dump(cfg, sort_keys=True).encode())
    for f in (os.path.join(get_package_share_directory("catch_description"), "config", "scene.yaml"),
              os.path.join(get_package_share_directory("catch_description"), "config", "physics.yaml")):
        h.update(open(f, "rb").read())
    if ff["inputs_hash"] != h.hexdigest()[:16]:
        return None, "throw_ff.yaml is stale (re-run tools/throw_feedforward.py)"
    if ff["n"] != n_samples:
        return None, f"throw_ff.yaml has {ff['n']} samples, plan has {n_samples}"
    return np.array(ff["tau"]), "ok"


def plan_msg(plan, flight_time, catch_point, cube_id="", t0=None, now=0.0, library="", entry=-1):
    """ThrowPlan message; with t0=None the absolute times are left zero (preview)."""
    msg = ThrowPlan()
    msg.library, msg.library_entry = library, int(entry)
    msg.stroke_start_q = [float(x) for x in plan.q_start]
    msg.header.stamp = to_time(now)
    msg.header.frame_id = "world"
    msg.cube_id = cube_id
    msg.release_position = Point(x=float(plan.p_release[0]), y=float(plan.p_release[1]), z=float(plan.p_release[2]))
    msg.release_velocity = Vector3(x=float(plan.v_release[0]), y=float(plan.v_release[1]), z=float(plan.v_release[2]))
    msg.release_spin = Vector3(x=float(plan.spin[0]), y=float(plan.spin[1]), z=float(plan.spin[2]))
    if t0 is not None:
        msg.release_time = to_time(t0 + plan.t_release)
        msg.stroke_start_time = to_time(t0)
    msg.flight_time = float(flight_time)
    msg.catch_point = Point(x=float(catch_point[0]), y=float(catch_point[1]), z=float(catch_point[2]))
    return msg


class ThrowNode(Node):
    def __init__(self):
        super().__init__("throw_node")
        self.declare_parameter("start_lead", 0.5)          # s of sim time between commit and stroke start
        self.declare_parameter("start_tolerance", 0.02)    # rad, arm must be this close to the stroke start
        self.declare_parameter("blend_acc", 6.0)           # rad/s^2, blend from the nominal to a perturbed stroke start
        self.declare_parameter("use_feedforward", True)
        cb = ReentrantCallbackGroup()
        self.robot = make_robot()
        self.plan, self.flight_time, self.catch_point, self.cfg = build_plan()
        rel = self.cfg.get("release", {})
        self.declare_parameter("t_loosen", float(rel.get("t_loosen", 0.12)))   # overridable for studies
        self.t_loosen = self.get_parameter("t_loosen").value
        self.loosen_to = float(rel.get("loosen_to", 0.30))
        self.hold_cmd = float(rel.get("hold", 0.70))           # grip command while holding (skills gripper_close)
        self.grip_model = gripper_model.load_profile(robot="a")  # 2F-85 speed + command latency (config/gripper.yaml)
        self.ff, why = load_feedforward(len(self.plan.t))
        if self.ff is None:
            self.get_logger().warn(f"no torque feed-forward: {why}")
        # v2: throw library + release noise (config/v2.yaml)
        self.v2 = load_v2()
        self.declare_parameter("seed", int(self.v2["library"]["seed"]))
        self.rng = np.random.default_rng(self.get_parameter("seed").value)
        self.libs, self.lib_plans, self.selected = {}, {}, {}
        self.js = JointStateCache(self)       # latest /joint_states off this node's executor
        self.plan_pub = self.create_publisher(ThrowPlan, "/throw_plan",
                                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.jtc = ActionClient(self, FollowJointTrajectory, "/arm_a_controller/follow_joint_trajectory",
                                callback_group=cb)
        self.grip = ActionClient(self, FollowJointTrajectory, "/gripper_a_controller/follow_joint_trajectory",
                                 callback_group=cb)
        ActionServer(self, Throw, "/throw", self.execute, callback_group=cb)
        self.create_service(GetThrowPlan, "/throw/preview", self.on_preview, callback_group=cb)
        m = self.plan.metrics
        self.get_logger().info(
            f"v1 throw plan: release {np.round(self.plan.p_release, 3).tolist()} v {np.round(self.plan.v_release, 3).tolist()} "
            f"(|v| {np.linalg.norm(self.plan.v_release):.3f} m/s), t_release {self.plan.t_release:.3f} s into a "
            f"{m['duration']:.3f} s stroke, peak |qd|/limit {m['vel_ratio_max']:.2f}; gripper model "
            f"{self.grip_model['name']} ({1000 * self.grip_model['gap_speed']:.0f} mm/s, {1000 * self.grip_model['latency']:.0f} ms)")

    # ------------------------------------------------------------------ library
    def library(self, name):
        if name not in self.libs:
            self.libs[name] = Library(name)
            self.get_logger().info(f"throw library {name}: {len(self.libs[name])} entries")
        return self.libs[name]

    def library_plan(self, name, idx):
        """Nominal plan of a library entry (cached) or (None, reason) if its hash is stale."""
        lib = self.library(name)
        e = lib.entries[idx]
        if (name, idx) not in self.lib_plans:
            ok, h = lib.check(e)
            if not ok:
                return None, e, f"library {name} entry {idx} is stale (hash {h} != {e['hash']}): re-run tools/build_throw_library.py"
            self.lib_plans[(name, idx)] = lib.plan(self.robot, e)
        return self.lib_plans[(name, idx)], e, "ok"

    def on_preview(self, req, res):
        if not req.library:
            res.success = True
            res.plan = plan_msg(self.plan, self.flight_time, self.catch_point, now=self.now())
            return res
        lib = self.library(req.library)
        idx = int(req.entry) if req.entry >= 0 else lib.draw(self.rng)
        plan, e, why = self.library_plan(req.library, idx)
        if plan is None:
            self.get_logger().error(why)
            res.success = False
            return res
        self.selected[req.library] = idx
        res.success = True
        res.plan = plan_msg(plan, e["t_target"], e["target"], now=self.now(), library=req.library, entry=idx)
        return res

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _wait(self, fut, timeout):
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.002)
        return fut.result() if fut.done() else None

    def execute(self, gh):
        res = Throw.Result()
        g = gh.request
        noise = g.noise or "none"
        res.noise = noise
        if not g.library:                                  # the configured v1 throw
            nominal, ex_plan, ff = self.plan, self.plan, self.ff
            flight, catch, entry = self.flight_time, self.catch_point, -1
            t_loosen = self.get_parameter("t_loosen").value
            v_cmd, dt_off = self.plan.v_release, 0.0
        else:
            entry = int(g.entry) if g.entry >= 0 else self.selected.get(g.library, -1)
            if entry < 0:
                gh.abort()
                res.success, res.message = False, f"no {g.library} entry selected (call /throw/preview first)"
                return res
            nominal, e, why = self.library_plan(g.library, entry)
            if nominal is None:
                gh.abort()
                res.success, res.message = False, why
                return res
            lib = self.library(g.library)
            flight, catch = e["t_target"], e["target"]
            if noise not in self.v2["noise"]:
                gh.abort()
                res.success, res.message = False, f"unknown noise level {noise}"
                return res
            v_cmd, dt_off = sample_noise(self.v2["noise"][noise], nominal.v_release, np.random.default_rng(g.noise_seed))
            ex_plan = nominal if noise == "none" else lib.plan(self.robot, e, v_cmd)
            ff = lib.feedforward(e, len(ex_plan.t))
            t_loosen = ex_plan.t_release - self.v2["library"]["loosen_before_separation"]
        res.release_velocity_commanded = Vector3(x=float(v_cmd[0]), y=float(v_cmd[1]), z=float(v_cmd[2]))
        res.release_time_offset = float(dt_off)
        q = np.array(self.js.positions([f"a_{j}" for j in JN]))
        # A waits at the NOMINAL stroke start; a perturbed stroke starts up to ~0.08 rad away (high noise) and is
        # blended in below
        err = float(np.max(np.abs(q - nominal.q_start)))
        tol = self.get_parameter("start_tolerance").value
        if not np.isfinite(err) or err > tol:
            gh.abort()
            res.success, res.message = False, f"arm A not at the stroke start (|dq| {err:.3f} rad)"
            return res
        lead = self.get_parameter("start_lead").value
        t_pub = self.now() + lead            # the published (nominal) stroke start
        t0 = t_pub + dt_off                  # the executed one (release timing noise)
        msg = plan_msg(nominal, flight, catch, g.cube_id, t_pub, self.now(), library=g.library, entry=entry)
        self.plan_pub.publish(msg)
        p = ex_plan
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = to_time(t0)
        goal.trajectory.joint_names = [f"a_{j}" for j in JN]
        use_ff = ff is not None and self.get_parameter("use_feedforward").value
        for i in range(len(p.t)):
            pt = JointTrajectoryPoint(positions=p.q[i].tolist(), velocities=p.qd[i].tolist(),
                                      accelerations=p.qdd[i].tolist(), time_from_start=to_dur(p.t[i]))
            if use_ff:
                pt.effort = ff[i].tolist()
            goal.trajectory.points.append(pt)
        last = goal.trajectory.points[-1]
        last.velocities, last.accelerations = [0.0] * 6, [0.0] * 6
        end = JointTrajectoryPoint(positions=p.q[-1].tolist(), velocities=[0.0] * 6, accelerations=[0.0] * 6,
                                   time_from_start=to_dur(p.t[-1] + 0.3))
        if use_ff:
            last.effort, end.effort = [0.0] * 6, [0.0] * 6
        goal.trajectory.points.append(end)
        t_blend = 0.0
        if ex_plan is not nominal:
            # the perturbed stroke starts a few mm from the nominal start where A waits: blend in (rest to rest,
            # cubic: peak acceleration 6 d / T^2 <= blend_acc, at least 0.15 s)
            d = float(np.max(np.abs(ex_plan.q_start - q)))
            t_blend = max(0.15, float(np.sqrt(6.0 * d / self.get_parameter("blend_acc").value)))
            pre = JointTrajectoryPoint(positions=q.tolist(), velocities=[0.0] * 6, accelerations=[0.0] * 6,
                                       time_from_start=to_dur(0.0))
            for pt in goal.trajectory.points:
                t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9 + t_blend
                pt.time_from_start = to_dur(t)
            goal.trajectory.points.insert(0, pre)
            goal.trajectory.header.stamp = to_time(t0 - t_blend)
            if use_ff:
                pre.effort = [0.0] * 6
        gh.publish_feedback(Throw.Feedback(phase="committed"))
        h = self._wait(self.jtc.send_goal_async(goal), 5.0)
        t_acc = self.now()
        if h is None or not h.accepted:
            gh.abort()
            res.success, res.message = False, "JTC rejected the throw trajectory"
            return res
        if t_acc > t0 - t_blend:
            self.get_logger().error(f"JTC accepted the throw {t_acc - t0:.3f} s after its start stamp (increase start_lead)")
        # grip loosen during the acceleration phase: a time-stamped trajectory for gripper_a_controller (a JTC),
        # executed in sim time; 2F-85 model: the command reaches the fingers one latency later and the gap opens at
        # the model speed from where the fingers are (stalled on the cube)
        k_now = self.js.position("a_robotiq_85_left_knuckle_joint")
        gm = self.grip_model
        ramp = gripper_model.ramp_points(self.hold_cmd, k_now, self.loosen_to, gm["gap_speed"], gm["latency"])
        gg = FollowJointTrajectory.Goal()
        gg.trajectory.header.stamp = to_time(t0)
        gg.trajectory.joint_names = ["a_robotiq_85_left_knuckle_joint"]
        gg.trajectory.points = [JointTrajectoryPoint(positions=[self.hold_cmd], time_from_start=to_dur(0.0))]
        gg.trajectory.points += [JointTrajectoryPoint(positions=[q_], time_from_start=to_dur(t_loosen + t_))
                                 for t_, q_ in ramp[1:]]
        hg = self._wait(self.grip.send_goal_async(gg), 5.0)
        t_gacc = self.now()
        g_ok = hg is not None and hg.accepted and t_gacc < t0 + t_loosen
        if not g_ok:
            self.get_logger().error(f"gripper loosen trajectory not accepted in time (accepted={hg is not None and hg.accepted}, "
                                    f"{t_gacc - t0:+.3f} s vs stroke start)")
        gh.publish_feedback(Throw.Feedback(phase=f"gripper loosen scheduled at t0+{t_loosen:.3f}"))
        r = self._wait(h.get_result_async(), p.t[-1] + lead + 5.0)
        ok = r is not None and r.result.error_code == 0 and g_ok
        res.success = bool(ok)
        src = f"library {g.library} entry {entry}" if g.library else "v1 throw"
        res.message = (f"stroke done ({src}, noise {noise}: |v| {np.linalg.norm(v_cmd):.4f} vs {np.linalg.norm(nominal.v_release):.4f} m/s, "
                       f"start {1000 * dt_off:+.1f} ms; feed-forward {'on' if use_ff else 'off'}); loosen to {self.loosen_to} "
                       f"commanded at t0+{t_loosen:.3f} s ({gm['name']} gripper model)"
                       if ok else f"throw failed: arm JTC {r.result.error_code if r else 'timeout'} "
                                  f"{r.result.error_string if r else ''}, gripper scheduled in time: {g_ok}")
        res.plan = msg
        if ok:
            gh.succeed()
        else:
            gh.abort()
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    n = ThrowNode()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    offload_sim_clock(n)          # /clock off the MultiThreadedExecutor (catch_sim.sim_clock)
    ex.spin()
