"""M5 evaluation: N throw -> catch (-> color check -> in-hand pose -> place) trials, judged with ground truth.

    ros2 run catch_tasks catch_eval [--throws 30] [--trigger force|time] [--time-offset 0.0] [--no-place]
                                    [--no-correction] [--out docs/m5/catch_30.json] [--rec-dir log/m5]

Per trial (color = red, green, blue in turn): reset the scene, spawn the cube on A's palm, A to the stroke start,
B pre-positions from the throw-plan preview (/catch goal, waits for "ready"), A throws (/throw). The catch is
judged from MuJoCo ground truth 1 s after the gripper closed: cube centre < 20 mm from B's TCP and at rest.
Held only by contact + friction: there is no weld in the model (the debug weld does not exist in this build).
Then B checks the color (wrist camera), estimates the in-hand pose (wrist camera), and places the cube in the
slot of that color on its pallet (corrected for the in-hand pose unless --no-correction). The final cube pose is
compared with the slot. The sim state of every trial is saved for replay clips (tools/render_replay.py).
"""
import argparse
import json
import os
import threading
import time

import numpy as np
import rclpy
from rclpy.action import ActionClient

from catch_interfaces.action import Catch, Throw
from catch_interfaces.srv import GetColor, GetInHandPose, GetThrowPlan, ResetScene
from catch_kinematics.transforms import inv_tf, make_tf, rot_x
from catch_sim.scene_config import slot_position

from .moveit_client import _wait
from .recorder import Recorder
from .throw_eval import COLORS, Eval


def fold45(a):
    return (a + np.pi / 4) % (np.pi / 2) - np.pi / 4


def rest_pose_errors(R):
    """(tilt, yaw folded to +-45 deg) of a cube rotation: tilt = angle between world z and the nearest cube axis."""
    k = int(np.argmax(np.abs(R[2])))                      # cube axis closest to world z (column of R)
    tilt = float(np.arccos(min(1.0, abs(R[2, k]))))
    j = (k + 1) % 3                                       # another cube axis: its yaw is the cube yaw (mod 90)
    yaw = float(fold45(np.arctan2(R[1, j], R[0, j])))
    return tilt, yaw


class CatchEval(Eval):
    def __init__(self, args):
        super().__init__(args)
        n = self.node
        self.recorder = Recorder(n)
        self.catch = ActionClient(n, Catch, "/catch")
        self.preview = n.create_client(GetThrowPlan, "/throw/preview")
        self.get_color = n.create_client(GetColor, "/perception/get_color")
        self.in_hand = n.create_client(GetInHandPose, "/perception/in_hand_pose")
        self.rb = self.sk._robot_model("b")
        self.phase = None

    # ------------------------------------------------------------------ ground truth helpers
    def tcp_b(self):
        return self.rb.fk(self.sk.mi.joints("b"))

    def cube_in_tcp(self, color):
        p, R, v = self.recorder.cube(color)
        return inv_tf(self.tcp_b()) @ make_tf(R, p), v

    def gt_held(self, color):
        T, v = self.cube_in_tcp(color)
        rel = T[:3, 3]
        return bool(np.linalg.norm(rel) < 0.02 and np.linalg.norm(v) < 0.05), T

    # ------------------------------------------------------------------ one trial
    def one(self, i):
        a = self.args
        color = COLORS[i % 3]
        self.color = color
        row = dict(i=i, color=color, trigger=a.trigger)
        sk = self.sk
        _wait(self.reset.call_async(ResetScene.Request()), 10)
        sk._grip("b", 0.0)
        if np.max(np.abs(sk.mi.joints("b") - self.poses["b"]["home"])) > 0.01:
            sk.mi.move_joints("b", self.poses["b"]["home"])
        ok, msg = self.load_cube(color)
        if not ok:
            row.update(ok=False, stage="load", why=msg)
            return row
        # B pre-positions from the preview of the throw plan
        pv = _wait(self.preview.call_async(GetThrowPlan.Request()), 10)
        self.catch.wait_for_server(timeout_sec=10)
        self.phase = None
        goal = Catch.Goal(plan=pv.plan, trigger=a.trigger, time_offset=a.time_offset)
        ch = _wait(self.catch.send_goal_async(goal, feedback_callback=lambda f: setattr(self, "phase", f.feedback.phase)), 10)
        t0 = time.monotonic()
        while self.phase != "ready" and time.monotonic() - t0 < 60:
            time.sleep(0.01)
        if self.phase != "ready":
            r = _wait(ch.get_result_async(), 5)
            row.update(ok=False, stage="pre-position", why=r.result.message if r else "catcher not ready")
            return row
        self.recorder.start()
        th = _wait(self.throw.send_goal_async(Throw.Goal(cube_id=f"cube_{color}")), 10)
        cr = _wait(ch.get_result_async(), 20)
        tr = _wait(th.get_result_async(), 20)
        self.sleep_sim(a.hold_check)
        held, T_gt = self.gt_held(color)
        self.recorder.stop()
        res = cr.result if cr else None
        row.update(throw_ok=bool(tr and tr.result.success), catch_msg=res.message if res else "catch timeout",
                   trigger_time_rel_ms=round(1000 * res.trigger_time_rel, 1) if res and res.success else None,
                   peak_force=round(res.peak_force, 1) if res else None, knuckle=round(res.knuckle, 3) if res else None,
                   node_says_held=bool(res and res.success), held=held,
                   inhand_gt=dict(p_mm=(1000 * T_gt[:3, 3]).round(1).tolist(),
                                  angle_deg=round(float(np.degrees(fold45(np.arctan2(T_gt[2, 1], T_gt[1, 1])))), 2)))
        arr = self.recorder.arrays()
        row["min_cube_tcp_mm"] = self.min_cube_tcp(arr, color)
        if a.rec_dir:
            os.makedirs(a.rec_dir, exist_ok=True)
            self.recorder.save(os.path.join(a.rec_dir, f"trial_{i:02d}.npz"), meta=json.dumps(row))
        sk._grip("a", 0.0)
        sk.mi.move_joints("a", self.poses["a"]["home"])
        if not held:
            row.update(ok=False, stage="catch", why=self.classify(row))
            return row
        if a.no_place:
            row.update(ok=True, stage="caught")
            sk._grip("b", 0.0)
            return row
        return self.place(row, color, T_gt)

    def min_cube_tcp(self, arr, color):
        """Closest approach of the cube centre to B's TCP during the recording [mm] (ground truth + FK)."""
        if not len(arr["cubes"]) or not len(arr["t_js"]):
            return None
        k = COLORS.index(color)
        tc, pc = arr["cubes"][:, 0], arr["cubes"][:, 1 + 7 * k:4 + 7 * k]
        names = list(arr["names"])
        idx = [names.index(f"b_{j}") for j in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
                                               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")]
        best = np.inf
        for t, p in zip(tc[::2], pc[::2]):
            j = min(np.searchsorted(arr["t_js"], t), len(arr["t_js"]) - 1)
            best = min(best, np.linalg.norm(self.rb.fk(arr["q"][j, idx])[:3, 3] - p))
        return round(1000 * float(best), 1)

    def classify(self, row):
        if "no contact" in row["catch_msg"]:
            return "missed the palm (no contact detected)"
        if row["min_cube_tcp_mm"] is not None and row["min_cube_tcp_mm"] > 30:
            return "missed the palm"
        if row["node_says_held"]:
            return "slipped out after closing"
        return "bounced out before the fingers closed"

    def place(self, row, color, T_gt):
        a, sk = self.args, self.sk
        rc = _wait(self.get_color.call_async(GetColor.Request(camera="")), 10)
        seen = rc.color if rc and rc.success else ""
        row.update(color_seen=seen, color_ok=seen == color, color_conf=round(rc.confidence, 3) if rc else None)
        if not seen:
            row.update(ok=False, stage="color", why=rc.message if rc else "color check timeout")
            sk._grip("b", 0.0)
            return row
        ri = _wait(self.in_hand.call_async(GetInHandPose.Request(color=seen)), 10)
        if ri and ri.success:
            T_est = make_tf(rot_x(ri.angle), [ri.pose.position.x, ri.pose.position.y, ri.pose.position.z])
            ang_gt = np.arctan2(T_gt[2, 1], T_gt[1, 1])
            row["inhand_est"] = dict(p_mm=(1000 * T_est[:3, 3]).round(1).tolist(), angle_deg=round(float(np.degrees(ri.angle)), 2),
                                     fit_rms_mm=round(1000 * ri.fit_rms, 2),
                                     angle_err_deg=round(float(np.degrees(fold45(ri.angle - ang_gt))), 2),
                                     yz_err_mm=(1000 * (T_est[1:3, 3] - T_gt[1:3, 3])).round(1).tolist())
        else:
            row["inhand_est"] = dict(failed=ri.message if ri else "timeout")
            T_est = make_tf(p=[0, 0, 0])
        T_use = T_est if not a.no_correction else make_tf(p=[0, 0, T_est[2, 3]])
        sk.attach_cube("b", color, offset_z=float(T_use[2, 3]))
        r = sk.place_held("b", seen, "b", T_use, cube_id=color)
        row["place_msg"] = r.message
        self.sleep_sim(0.5)
        p, R, _ = self.recorder.cube(color)
        slot = slot_position(sk.scene, "b", color)
        tilt, yaw = rest_pose_errors(R)
        e = p - slot
        in_slot = bool(abs(e[0]) < (sk.scene["pallets"]["slot_inner"] - sk.edge) / 2 and
                       abs(e[1]) < (sk.scene["pallets"]["slot_inner"] - sk.edge) / 2 and abs(e[2]) < 0.003)
        row.update(place_err_mm=(1000 * e).round(2).tolist(), place_yaw_deg=round(float(np.degrees(yaw)), 2),
                   place_tilt_deg=round(float(np.degrees(tilt)), 2), in_slot=in_slot)
        sk.mi.move_joints("b", self.poses["b"]["home"])
        ok = r.ok and in_slot and row["color_ok"]
        row.update(ok=ok, stage="placed" if ok else "place", why="" if ok else (r.message if not r.ok else "not in slot"))
        return row


def summarize(rows):
    n = len(rows)
    caught = [r for r in rows if r.get("held")]
    out = dict(n=n, caught=len(caught), catch_rate=round(len(caught) / max(n, 1), 3),
               by_color={c: f"{sum(1 for r in caught if r['color'] == c)}/{sum(1 for r in rows if r['color'] == c)}" for c in COLORS},
               failures={})
    for r in rows:
        if not r.get("ok"):
            k = f"{r.get('stage')}: {r.get('why')}"
            out["failures"][k] = out["failures"].get(k, 0) + 1
    def stat(vals):
        a = np.array([v for v in vals if v is not None], float)
        return None if not len(a) else dict(mean=np.round(a.mean(axis=0), 3).tolist(), std=np.round(a.std(axis=0), 3).tolist(),
                                            max_abs=np.round(np.abs(a).max(axis=0), 3).tolist(), n=len(a))
    out["trigger_time_rel_ms"] = stat([r.get("trigger_time_rel_ms") for r in caught])
    out["peak_force"] = stat([r.get("peak_force") for r in caught])
    out["inhand_gt_angle_deg"] = stat([r["inhand_gt"]["angle_deg"] for r in caught])
    out["inhand_gt_p_mm"] = stat([r["inhand_gt"]["p_mm"] for r in caught])
    est = [r["inhand_est"] for r in caught if "inhand_est" in r and "angle_err_deg" in r["inhand_est"]]
    out["inhand_angle_err_deg"] = stat([e["angle_err_deg"] for e in est])
    out["inhand_yz_err_mm"] = stat([e["yz_err_mm"] for e in est])
    placed = [r for r in caught if "place_err_mm" in r]
    out["placed_in_slot"] = f"{sum(1 for r in placed if r['in_slot'])}/{len(placed)}"
    out["color_ok"] = f"{sum(1 for r in placed if r.get('color_ok'))}/{len([r for r in caught if 'color_ok' in r])}"
    out["place_err_mm"] = stat([r["place_err_mm"] for r in placed])
    out["place_yaw_deg"] = stat([r["place_yaw_deg"] for r in placed])
    out["place_tilt_deg"] = stat([r["place_tilt_deg"] for r in placed])
    out["complete"] = f"{sum(1 for r in rows if r.get('ok'))}/{n}"
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--throws", type=int, default=30)
    ap.add_argument("--trigger", default="force", choices=["force", "time"])
    ap.add_argument("--time-offset", type=float, default=0.0)
    ap.add_argument("--hold-check", type=float, default=1.0, help="s after the catch result before judging 'held'")
    ap.add_argument("--no-place", action="store_true")
    ap.add_argument("--no-correction", action="store_true", help="place assuming the cube is square in the hand")
    ap.add_argument("--out", default="")
    ap.add_argument("--rec-dir", default="log/m5")
    ap.add_argument("--from-pallet", action="store_true")
    args, _ = ap.parse_known_args()
    rclpy.init()
    e = CatchEval(args)
    rows = []
    for i in range(args.throws):
        r = e.one(i)
        rows.append(r)
        print("TRIAL", json.dumps(r), flush=True)
    s = summarize(rows)
    print("SUMMARY", json.dumps(s), flush=True)
    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        json.dump(dict(args=vars(args), trials=rows, summary=s), open(args.out, "w"), indent=1)
    e.sk.mi.move_joints("b", e.poses["b"]["home"])
    rclpy.shutdown()
