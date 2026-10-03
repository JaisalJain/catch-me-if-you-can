"""v2 step 3: live flight-tracking evaluation against ground truth.

    ros2 run catch_tasks tracker_eval [--throws 50] [--library pinch] [--seed 3] [--calibrate] [--out x.json]

Per throw: reset, cube spawned into A's palm-up gripper, A selects a random library entry (/throw/preview, seeded),
moves to its stroke start and throws (no noise); B stays at home (not catching). Recorded: every /tracker/cube
message (render stamp, receive / publish time of the C++ tracker, fit, triangulated fix) and the cube's ground
truth (FreeJointStatePublisher, 200 Hz). Reported:
  * end-to-end latency = t_published - render stamp (sim time; the sim runs at RTF 1.0), and its parts;
  * achieved camera rate (stamp spacing);
  * per-frame fix error vs ground truth (bias / std per axis) -> --calibrate prints the bias for tracker.yaml;
  * prediction error vs time since release: the position predicted (from each estimate) for the instant the cube
    passes closest to B's ready TCP, minus the ground truth there (the intercept B would choose).
"""
import argparse
import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient

from catch_interfaces.action import Throw
from catch_interfaces.msg import CubeTrack
from catch_interfaces.srv import GetThrowPlan, ResetScene, SpawnCube
from mujoco_ros2_control_msgs.msg import FreeJointStateArray

from .moveit_client import _wait
from .throw_eval import COLORS, Eval

READY_TCP = np.array([0.30, 0.25, 1.10])


def st(t):
    return t.sec + t.nanosec * 1e-9


class TrackerEval(Eval):
    def __init__(self, args):
        super().__init__(args, record=False)
        n = self.node
        self.preview = n.create_client(GetThrowPlan, "/throw/preview")
        self.trk, self.gt, self.rec = [], [], False
        self.lk = threading.Lock()
        from catch_throw.library import Library
        self.lib = Library(args.library)
        self.n_entries = len(self.lib)
        self.entry_rng = np.random.default_rng(args.seed)       # entries chosen here (reproducible eval sets)
        n.create_subscription(CubeTrack, "/tracker/cube", self.on_track, 200)
        n.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", self.on_gt, 200)

    def on_track(self, m):
        if self.rec and m.color == self.color:
            with self.lk:
                self.trk.append(m)

    def on_gt(self, m):
        if not self.rec:
            return
        for f in m.free_joints:
            if f.name == f"cube_{self.color}":
                p = f.pose.pose.position
                with self.lk:
                    self.gt.append((st(m.header.stamp), p.x, p.y, p.z))

    def load_library(self, color, pv):
        sk = self.sk
        ok, msg = sk.mi.move_joints("a", self.poses["a"]["palm_up"])
        if not ok:
            return False, f"palm_up: {msg}"
        sk._grip("a", 0.0)
        _wait(self.spawn.call_async(SpawnCube.Request(color=color, where="gripper_a")), 10)
        sk._grip("a", sk.cfg["gripper_close"])
        held, k = sk.in_hand("a")
        if not held:
            return False, f"not held at palm_up (knuckle {k:.3f})"
        sk.attach_cube("a", color, offset_z=0.0)
        ok, msg = sk.mi.move_joints("a", np.array(pv.plan.stroke_start_q))
        return ok, "loaded" if ok else f"to stroke start: {msg}"

    def one(self, i):
        a = self.args
        color = COLORS[i % 3]
        self.color = color
        _wait(self.reset.call_async(ResetScene.Request()), 10)
        entry = self.lib.draw(self.entry_rng)
        pv = _wait(self.preview.call_async(GetThrowPlan.Request(library=a.library, entry=entry)), 10)
        if pv is None or not pv.success:
            return dict(i=i, ok=False, why="preview failed")
        ok, msg = self.load_library(color, pv)
        if not ok:
            return dict(i=i, ok=False, why=msg, entry=pv.plan.library_entry)
        with self.lk:
            self.trk, self.gt = [], []
        self.rec = True
        h = _wait(self.throw.send_goal_async(Throw.Goal(cube_id=f"cube_{color}", library=a.library, entry=pv.plan.library_entry,
                                                        noise="none")), 10)
        r = _wait(h.get_result_async(), 20)
        self.sleep_sim(1.0)
        self.rec = False
        self.sk.detach_cube("a", color)
        self.sk._grip("a", 0.0)
        self.sk.mi.move_joints("a", self.poses["a"]["home"])
        if r is None or not r.result.success:
            return dict(i=i, ok=False, why=r.result.message if r else "throw timeout", entry=pv.plan.library_entry)
        t_rel = st(r.result.plan.release_time)
        return self.analyse(i, color, pv.plan.library_entry, t_rel)

    def analyse(self, i, color, entry, t_rel):
        with self.lk:
            trk, gt = list(self.trk), np.array(self.gt)
        row = dict(i=i, color=color, entry=int(entry), ok=bool(len(trk) and len(gt)), n_msgs=len(trk))
        if not row["ok"]:
            row["why"] = "no tracker output"
            return row
        tg, pg = gt[:, 0], gt[:, 1:]
        fly = [m for m in trk if m.in_flight]
        stamps = np.array([st(m.header.stamp) for m in fly])
        lat = np.array([st(m.t_published) - st(m.header.stamp) for m in fly])
        rcv = np.array([st(m.t_received) - st(m.header.stamp) for m in fly])
        proc = np.array([st(m.t_published) - st(m.t_received) for m in fly])
        gt_at = lambda t: np.array([np.interp(t, tg, pg[:, k]) for k in range(3)])
        fix_err = np.array([[m.measured.x, m.measured.y, m.measured.z] - gt_at(st(m.header.stamp)) for m in trk])
        # reference intercept: GT closest approach to B's ready TCP on the way down
        down = np.where(np.gradient(pg[:, 2], tg) < 0)[0]
        k = down[np.argmin(np.linalg.norm(pg[down] - READY_TCP, axis=1))] if len(down) else int(np.argmin(np.linalg.norm(pg - READY_TCP, axis=1)))
        T_ref, p_ref = tg[k], pg[k]
        pred = []
        for m in fly:
            s = st(m.header.stamp)
            if s >= T_ref:
                break
            tau = T_ref - s
            p = np.array([m.position.x, m.position.y, m.position.z]) + np.array([m.velocity.x, m.velocity.y, m.velocity.z]) * tau
            p[2] -= 0.5 * 9.81 * tau * tau
            e = np.linalg.norm(p - p_ref)
            cov = np.array(m.intercept_covariance).reshape(3, 3)
            pred.append((s - t_rel, st(m.t_published) - t_rel, float(e), float(2.7955 * np.sqrt(max(np.trace(cov), 0) / 3))))
        row.update(t_release=t_rel, T_ref_after_release=float(T_ref - t_rel), p_ref=p_ref.tolist(),
                   t_first_after_release=float(st(fly[0].t_first) - t_rel) if fly else None,
                   n_fly=len(fly), frame_dt_ms=(1000 * np.diff(stamps)).round(2).tolist(),
                   latency_ms=(1000 * lat).round(3).tolist(), receive_ms=(1000 * rcv).round(3).tolist(),
                   process_ms=(1000 * proc).round(3).tolist(), fix_err_mm=(1000 * fix_err).round(2).tolist(),
                   pred=[[round(a, 4), round(b, 4), round(1000 * c, 2), round(1000 * d, 2)] for a, b, c, d in pred])
        return row


def summarize(rows):
    ok = [r for r in rows if r.get("ok") and r.get("n_fly")]
    if not ok:
        return dict(n=len(rows), n_ok=0)
    cat = lambda k: np.concatenate([np.array(r[k], float) for r in ok if len(r[k])])
    lat, rcv, proc, dt = cat("latency_ms"), cat("receive_ms"), cat("process_ms"), cat("frame_dt_ms")
    fe = np.vstack([np.array(r["fix_err_mm"]) for r in ok])
    out = dict(n=len(rows), n_ok=len(ok),
               latency_ms=dict(p50=float(np.median(lat)), p95=float(np.percentile(lat, 95)), max=float(lat.max())),
               receive_ms=dict(p50=float(np.median(rcv)), p95=float(np.percentile(rcv, 95))),
               process_ms=dict(p50=float(np.median(proc)), p95=float(np.percentile(proc, 95))),
               frame_dt_ms=dict(median=float(np.median(dt)), mean=float(dt.mean()), rate_hz=float(1000 / dt.mean())),
               fix_bias_mm=fe.mean(0).round(3).tolist(), fix_std_mm=fe.std(0).round(3).tolist(),
               t_first_after_release_ms=float(1000 * np.median([r["t_first_after_release"] for r in ok])))
    # prediction error vs time since release (estimate available = t_published), 95th percentile per bin
    bins = [0.10, 0.125, 0.15, 0.175, 0.20, 0.25, 0.30]
    pe = {}
    for b in bins:
        errs, covs = [], []
        for r in ok:
            avail = [p for p in r["pred"] if p[1] <= b]
            if avail:
                errs.append(avail[-1][2])
                covs.append(avail[-1][3])
        if len(errs) >= max(5, len(ok) // 2):
            pe[str(b)] = dict(n=len(errs), e95_mm=float(np.percentile(errs, 95)), median_mm=float(np.median(errs)),
                              reported_e95_median_mm=float(np.median(covs)))
    out["pred_err_vs_time"] = pe
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--throws", type=int, default=50)
    ap.add_argument("--library", default="pinch")
    ap.add_argument("--seed", type=int, default=1, help="entry choice (calibration and evaluation use different seeds)")
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--out", default="")
    ap.add_argument("--from-pallet", action="store_true")
    args, _ = ap.parse_known_args()
    rclpy.init()
    e = TrackerEval(args)
    e.sk.mi.move_joints("b", e.poses["b"]["home"])
    rows = []
    for i in range(args.throws):
        r = e.one(i)
        rows.append(r)
        print("THROW", json.dumps({k: v for k, v in r.items() if k in ("i", "ok", "why", "entry", "n_fly")}), flush=True)
    s = summarize(rows)
    print("SUMMARY", json.dumps(s), flush=True)
    if args.calibrate:
        print("BIAS (subtract in catch_tracker/config/tracker.yaml, metres):",
              [round(x / 1000.0, 5) for x in s.get("fix_bias_mm", [0, 0, 0])], flush=True)
    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        json.dump(dict(args=vars(args), throws=rows, summary=s), open(args.out, "w"))
    rclpy.shutdown()
