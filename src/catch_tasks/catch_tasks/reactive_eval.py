"""v2 step 6: live benchmark of the reactive catch (one cell per run), judged with ground truth.

    ros2 run catch_tasks reactive_eval --mode pinch --source vision --noise medium --throws 60 --seed 1 \
        [--label cell_name] [--out docs/v2/phase2/bench/cell.json] [--rec-dir log/p2/bench/cell] [--rec-fail-only]

Per throw: reset the scene, A selects a random entry of the library (pinch: the full A-feasible set B can catch;
cup: square-arrival throws), spawns the cube on A's palm-up gripper and moves to that entry's stroke start; B gets a
ReactiveCatch goal (vision: /reactive_catch, plan-informed baseline: /reactive_catch_plan), goes to the ready pose
with the 56 mm pre-gap and reports "ready"; A throws with the requested release noise. Ground truth 1 s after the
catch: cube centre < 25 mm from B's TCP (between the pads), fingers closed on it (knuckle 0.30-0.60), at rest, off
the table. Failure classes: abort (B found no feasible intercept), missed intercept (the cube never came within
30 mm of the TCP), late close / slide-off (it came close but was not held: fingers closed after it bounced out of
the gripper / it slid out after being gripped), fingertip (held, but 25-50 mm out), collision (B's capsule model
clearance < 0 along the recorded motion — never expected, the planner checks every plan).
The camera rate, gripper profile / latency and MJCF variant of a cell are properties of the running sim (set by the
benchmark script, tools/v2_live/bench.sh); the cell json records them from the environment.
"""
import argparse
import json
import math
import os
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient

from catch_interfaces.action import ReactiveCatch, Throw
from catch_interfaces.srv import GetThrowPlan, ResetScene
from catch_kinematics.collision import CapsuleModel
from catch_kinematics.transforms import inv_tf, make_tf

from .moveit_client import _wait
from .recorder import Recorder
from .throw_eval import COLORS
from .tracker_eval import TrackerEval


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 1.0
    p = k / n
    den = 1 + z * z / n
    c = (p + z * z / (2 * n)) / den
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / den
    return round(c - h, 4), round(c + h, 4)


class ReactiveEval(TrackerEval):
    def __init__(self, args):
        super().__init__(args)
        n = self.node
        self.recorder = Recorder(n)
        name = "/reactive_catch" if args.source == "vision" else "/reactive_catch_plan"
        self.catch = ActionClient(n, ReactiveCatch, name)
        self.rb = self.sk._robot_model("b")
        self.caps = CapsuleModel()
        self.phase = None

    def tcp_b(self):
        return self.rb.fk(self.sk.mi.joints("b"))

    def one(self, i):
        a = self.args
        color = COLORS[i % 3]
        self.color = color
        row = dict(i=i, color=color, mode=a.mode, source=a.source, noise=a.noise)
        sk = self.sk
        _wait(self.reset.call_async(ResetScene.Request()), 10)
        lib = "cup" if a.mode == "cup" else "pinch"
        pv = _wait(self.preview.call_async(GetThrowPlan.Request(library=lib, entry=-1)), 10)
        if pv is None or not pv.success:
            row.update(ok=False, held=False, stage="setup", why="preview failed")
            return row
        row["entry"] = int(pv.plan.library_entry)
        # B: open fingers (it may still hold the last cube) before anything moves
        sk._grip("b", 0.0)
        ok, msg = self.load_library(color, pv)
        if not ok:
            row.update(ok=False, held=False, stage="setup", why=msg)
            return row
        self.catch.wait_for_server(timeout_sec=10)
        self.phase = None
        ch = _wait(self.catch.send_goal_async(ReactiveCatch.Goal(mode=a.mode, timeout=8.0),
                                              feedback_callback=lambda f: setattr(self, "phase", f.feedback.phase)), 10)
        t0 = time.monotonic()
        while self.phase != "ready" and time.monotonic() - t0 < 60:
            time.sleep(0.01)
        if self.phase != "ready":
            r = _wait(ch.get_result_async(), 5)
            row.update(ok=False, held=False, stage="setup", why=f"B not ready: {r.result.message if r else 'timeout'}")
            return row
        self.recorder.start()
        seed = int(a.seed) * 1000 + i
        th = _wait(self.throw.send_goal_async(Throw.Goal(cube_id=f"cube_{color}", library=lib, entry=row["entry"],
                                                         noise=a.noise, noise_seed=seed)), 10)
        tr = _wait(th.get_result_async(), 20)
        cr = _wait(ch.get_result_async(), 25)
        self.sleep_sim(1.0)
        held, T, v = self.gt_held(color)
        self.recorder.stop()
        arr = self.recorder.arrays()
        res = cr.result if cr else None
        row.update(throw_ok=bool(tr and tr.result.success),
                   noise_dv=(np.array([tr.result.release_velocity_commanded.x, tr.result.release_velocity_commanded.y,
                                       tr.result.release_velocity_commanded.z]) -
                             np.array([tr.result.plan.release_velocity.x, tr.result.plan.release_velocity.y,
                                       tr.result.plan.release_velocity.z])).round(4).tolist() if tr else None,
                   noise_dt=float(tr.result.release_time_offset) if tr else None,
                   catch_msg=res.message if res else "catch timeout", node_success=bool(res and res.success),
                   trigger=res.trigger if res else "", aborted=bool(res and res.aborted),
                   abort_reason=res.abort_reason if res else "", t_commit=res.t_commit if res else None,
                   n_replans=res.n_replans if res else None, replan_rate_hz=res.replan_rate_hz if res else None,
                   max_limit_ratio=res.max_limit_ratio if res else None, plan_min_clearance=res.min_clearance if res else None,
                   plan_ms=res.plan_ms_median if res else None, knuckle=res.knuckle if res else None,
                   t_close_minus_intercept_ms=round(1000 * (res.t_close_cmd - res.t_intercept), 1) if res and res.t_close_cmd else None,
                   t_contact_minus_intercept_ms=round(1000 * (res.t_contact - res.t_intercept), 1) if res and res.t_contact else None,
                   held=held, rel_mm=(1000 * T[:3, 3]).round(1).tolist())
        row.update(self.post(arr, color, row))
        row["outcome"] = self.classify(row)
        if a.rec_dir and (not a.rec_fail_only or not held):
            os.makedirs(a.rec_dir, exist_ok=True)
            self.recorder.save(os.path.join(a.rec_dir, f"trial_{i:03d}.npz"), meta=json.dumps(row))
        sk._grip("a", 0.0)
        sk.mi.move_joints("a", self.poses["a"]["home"])
        row["ok"] = held
        return row

    def gt_held(self, color):
        p, R, v = self.recorder.cube(color)
        T = inv_tf(self.tcp_b()) @ make_tf(R, p)
        k = self.sk.mi.gripper_position("b")
        held = bool(np.linalg.norm(T[:3, 3]) < 0.025 and np.linalg.norm(v) < 0.05 and 0.30 <= k <= 0.60
                    and p[2] > self.sk.scene["table"]["top_z"] + 0.05)
        return held, T, v

    def post(self, arr, color, row):
        """Closest approach cube-TCP and the capsule clearance of B along the recorded motion."""
        out = dict(min_cube_tcp_mm=None, min_clearance_mm=None)
        if not len(arr["cubes"]) or not len(arr["t_js"]):
            return out
        k = COLORS.index(color)
        tc, pc = arr["cubes"][:, 0], arr["cubes"][:, 1 + 7 * k:4 + 7 * k]
        names = list(arr["names"])
        jb = [names.index(f"b_{j}") for j in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                               "wrist_2_joint", "wrist_3_joint")]
        ja = [names.index(f"a_{j}") for j in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint",
                                               "wrist_2_joint", "wrist_3_joint")]
        best = np.inf
        for t, p in zip(tc[::2], pc[::2]):
            j = min(np.searchsorted(arr["t_js"], t), len(arr["t_js"]) - 1)
            best = min(best, np.linalg.norm(self.rb.fk(arr["q"][j, jb])[:3, 3] - p))
        Qb = arr["q"][::25, jb]
        ra = self.sk._robot_model("a")
        A0, A1 = self.caps.segments(ra, arr["q"][-1, ja])
        clr = self.caps.clearance(self.rb, Qb, env=True, self_check=True, extra_capsules=(A0[0], A1[0], self.caps.r))
        out.update(min_cube_tcp_mm=round(1000 * float(best), 1), min_clearance_mm=round(1000 * float(clr), 1))
        return out

    def classify(self, r):
        if r["held"]:
            return "held"
        if r.get("aborted"):
            return "abort"
        if r.get("min_clearance_mm") is not None and r["min_clearance_mm"] < 0:
            return "collision"
        if r.get("min_cube_tcp_mm") is None or r["min_cube_tcp_mm"] > 30:
            return "missed intercept"
        rel = np.linalg.norm(r["rel_mm"]) if r.get("rel_mm") else 1e9
        if 25 <= rel < 50 and r.get("knuckle") and 0.30 <= r["knuckle"] <= 0.60:
            return "fingertip"
        if r.get("knuckle") is not None and 0.30 <= r["knuckle"] <= 0.60:
            return "slide-off"          # the fingers closed on it, then it went
        return "late close"             # came close, fingers never closed on it


def summarize(rows, args):
    n = len(rows)
    valid = [r for r in rows if r.get("stage") != "setup"]
    k = sum(1 for r in valid if r["held"])
    from collections import Counter
    out = dict(n=n, n_valid=len(valid), held=k, rate=round(k / max(len(valid), 1), 4), ci95=wilson(k, len(valid)),
               outcomes=dict(Counter(r.get("outcome", r.get("stage")) for r in rows)),
               triggers=dict(Counter(r.get("trigger") for r in valid)))
    v = [r for r in valid if r.get("t_commit") is not None and not r.get("aborted")]
    if v:
        out.update(t_commit_ms=round(1000 * float(np.median([r["t_commit"] for r in v])), 1),
                   replan_rate_hz=round(float(np.median([r["replan_rate_hz"] for r in v])), 1),
                   plan_ms=round(float(np.median([r["plan_ms"] for r in v])), 2),
                   max_limit_ratio=round(float(max(r["max_limit_ratio"] for r in v)), 3),
                   min_plan_clearance_mm=round(1000 * float(min(r["plan_min_clearance"] for r in v)), 1))
    out["cell"] = dict(label=args.label, mode=args.mode, source=args.source, noise=args.noise, seed=args.seed,
                       gripper_profile=os.environ.get("CATCH_GRIPPER_PROFILE", "realistic"),
                       gripper_latency=os.environ.get("CATCH_GRIPPER_LATENCY", "profile default"),
                       camera_rate=os.environ.get("CATCH_CAMERA_RATE", "125"),
                       mujoco_model=os.environ.get("CATCH_MJCF", "default"))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--throws", type=int, default=60)
    ap.add_argument("--mode", default="pinch", choices=["pinch", "cup"])
    ap.add_argument("--source", default="vision", choices=["vision", "plan"])
    ap.add_argument("--noise", default="medium")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--label", default="")
    ap.add_argument("--out", default="")
    ap.add_argument("--rec-dir", default="")
    ap.add_argument("--rec-fail-only", action="store_true")
    ap.add_argument("--library", default="pinch")
    ap.add_argument("--from-pallet", action="store_true")
    ap.add_argument("--calibrate", action="store_true")
    args, _ = ap.parse_known_args()
    rclpy.init()
    e = ReactiveEval(args)
    rows = []
    for i in range(args.throws):
        r = e.one(i)
        rows.append(r)
        print("TRIAL", json.dumps(r), flush=True)
        if args.out and (i % 5 == 4 or i == args.throws - 1):
            os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
            json.dump(dict(args=vars(args), trials=rows, summary=summarize(rows, args)), open(args.out, "w"))
    s = summarize(rows, args)
    print("SUMMARY", json.dumps(s), flush=True)
    if args.out:
        json.dump(dict(args=vars(args), trials=rows, summary=s), open(args.out, "w"))
    e.sk._grip("b", 0.0)
    rclpy.shutdown()
