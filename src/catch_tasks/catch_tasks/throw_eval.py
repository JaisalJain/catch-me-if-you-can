"""M4 evaluation: N throws with nothing catching; release state and landing measured from MuJoCo ground truth.

    ros2 run catch_tasks throw_eval [--throws 20] [--from-pallet] [--out docs/m4/throws.json]

Per throw: cube seated on A's palm (spawned into the palm-up gripper, or --from-pallet: picked from the pallet
by detection and re-seated), MoveIt to the stroke start, /throw action, record ground truth at 200 Hz.
Release state: ballistic least-squares fit (known g) to the free-flight samples, evaluated at the planned
release time; separation time = first sample where the cube is > 2 mm from the TCP along the palm normal.
"""
import argparse
import json
import os
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState

from catch_interfaces.action import Throw
from catch_interfaces.msg import CubeDetectionArray
from catch_interfaces.srv import ResetScene, SpawnCube
from catch_sim.sim_clock import offload_sim_clock
from catch_throw.throw_node import build_plan
from mujoco_ros2_control_msgs.msg import FreeJointStateArray

from .moveit_client import JN, _wait
from .skills import Skills

G = np.array([0, 0, -9.81])
COLORS = ("red", "green", "blue")


def st(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def fit_ballistic(t, p, t_ref):
    """p(t) = p0 + v0 (t - t_ref) + g/2 (t - t_ref)^2  (least squares for p0, v0 per axis)."""
    tau = t - t_ref
    A = np.column_stack([np.ones_like(tau), tau])
    y = p - 0.5 * np.outer(tau ** 2, G)
    sol, res, *_ = np.linalg.lstsq(A, y, rcond=None)
    resid = y - A @ sol
    return sol[0], sol[1], float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1))))


def crossing(p0, v0, t_ref, z):
    """Descending crossing of height z by the fitted parabola: (time, position)."""
    a, b, c = 0.5 * G[2], v0[2], p0[2] - z
    disc = b * b - 4 * a * c
    if disc < 0:
        return None, None
    tau = max((-b - np.sqrt(disc)) / (2 * a), (-b + np.sqrt(disc)) / (2 * a))
    return t_ref + tau, p0 + v0 * tau + 0.5 * G * tau ** 2


class Eval:
    def __init__(self, args, record=True):
        self.args = args
        self.node = rclpy.create_node("throw_eval", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        ex = MultiThreadedExecutor()
        ex.add_node(self.node)
        offload_sim_clock(self.node)          # /clock off the MultiThreadedExecutor (catch_sim.sim_clock)
        threading.Thread(target=ex.spin, daemon=True).start()
        self.sk = Skills(self.node)
        assert self.sk.mi.wait_ready(60)
        self.lock = threading.Lock()
        self.cube, self.js, self.rec, self.det = [], [], False, None
        if record:     # 200 + 500 Hz in rclpy: only for the throw evaluation itself
            self.node.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", self.on_fj, 100)
            self.node.create_subscription(JointState, "/joint_states", self.on_js, 100)
        self.node.create_subscription(CubeDetectionArray, "/perception/overhead_a/cubes",
                                      lambda m: setattr(self, "det", m), 1)
        self.throw = ActionClient(self.node, Throw, "/throw")
        self.spawn = self.node.create_client(SpawnCube, "/scene/spawn_cube")
        self.reset = self.node.create_client(ResetScene, "/scene/reset")
        self.poses = yaml.safe_load(open(get_package_share_directory("catch_description") + "/config/poses.yaml"))
        self.plan, self.T, self.catch_point, self.cfg = build_plan()
        self.robot = self.sk._robot_model("a")
        self.color = None

    def on_fj(self, m):
        if not self.rec:
            return
        for f in m.free_joints:
            if f.name == f"cube_{self.color}":
                p, w = f.pose.pose.position, f.twist.twist.angular
                q = f.pose.pose.orientation
                with self.lock:
                    self.cube.append((st(m.header), [p.x, p.y, p.z], [w.x, w.y, w.z], [q.w, q.x, q.y, q.z]))

    def on_js(self, m):
        if not self.rec:
            return
        d = dict(zip(m.name, zip(m.position, m.velocity)))
        if all(f"a_{j}" in d for j in JN):
            with self.lock:
                self.js.append((st(m.header), [d[f"a_{j}"][0] for j in JN], [d[f"a_{j}"][1] for j in JN],
                                d.get("a_robotiq_85_left_knuckle_joint", (np.nan, 0))[0]))

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, dt):
        t = self.now() + dt
        while self.now() < t:
            time.sleep(0.005)

    def load_cube(self, color):
        sk = self.sk
        if self.args.from_pallet:
            sk.mi.move_joints("a", self.poses["a"]["home"])
            t_ready = self.now() + 0.5
            while self.det is None or st(self.det.header) < t_ready:
                time.sleep(0.02)
            d = {x.color: x for x in self.det.detections}[color]
            pos = np.array([d.pose.position.x, d.pose.position.y, d.pose.position.z])
            yaw = np.arctan2(2 * d.pose.orientation.w * d.pose.orientation.z, 1 - 2 * d.pose.orientation.z ** 2)
            r = sk.pick("a", pos, yaw, cube_id=color)
            if not r.ok:
                return False, r.message
        ok, msg = sk.mi.move_joints("a", self.poses["a"]["palm_up"])
        if not ok:
            return False, f"palm_up: {msg}"
        if self.args.from_pallet:   # re-seat: let the cube drop onto the palm insert, re-grip
            sk.detach_cube("a", color)
            sk._grip("a", self.cfg["release"]["loosen_to"])
            self.sleep_sim(0.4)
        else:
            sk._grip("a", 0.0)
            _wait(self.spawn.call_async(SpawnCube.Request(color=color, where="gripper_a")), 10)
        sk._grip("a", sk.cfg["gripper_close"])
        held, k = sk.in_hand("a")
        if not held:
            return False, f"not held at palm_up (knuckle {k:.3f})"
        sk.attach_cube("a", color, offset_z=0.0)
        ok, msg = sk.mi.move_joints("a", self.plan.q_start)
        return ok, "loaded" if ok else f"to stroke start: {msg}"

    def one(self, i):
        color = COLORS[i % 3]
        self.color = color
        _wait(self.reset.call_async(ResetScene.Request()), 10)
        ok, msg = self.load_cube(color)
        if not ok:
            return dict(i=i, color=color, ok=False, why=msg)
        with self.lock:
            self.cube, self.js = [], []
        self.rec = True
        self.throw.wait_for_server(timeout_sec=10)
        h = _wait(self.throw.send_goal_async(Throw.Goal(cube_id=f"cube_{color}")), 10)
        r = _wait(h.get_result_async(), 20)
        self.sleep_sim(1.2)
        self.rec = False
        self.sk.detach_cube("a", color)
        if r is None or not r.result.success:
            return dict(i=i, color=color, ok=False, why=r.result.message if r else "throw timeout")
        res = self.analyse(r.result)
        res.update(i=i, color=color, ok=True, throw_msg=r.result.message)
        self.sk._grip("a", 0.0)
        self.sk.mi.move_joints("a", self.poses["a"]["home"])
        return res

    def analyse(self, result):
        plan = result.plan
        t_rel = plan.release_time.sec + plan.release_time.nanosec * 1e-9
        t0 = plan.stroke_start_time.sec + plan.stroke_start_time.nanosec * 1e-9
        with self.lock:
            cube = list(self.cube)
            js = list(self.js)
        tc = np.array([c[0] for c in cube])
        pc = np.array([c[1] for c in cube])
        wc = np.array([c[2] for c in cube])
        tj = np.array([j[0] for j in js])
        qj = np.array([j[1] for j in js])
        vj = np.array([j[2] for j in js])
        kj = np.array([j[3] for j in js])
        lag = -0.0025      # /joint_states lags the ground truth by ~2.5 ms (measured in M2)
        # free flight window: 30 ms after separation until 0.15 m above the table (or 0.5 s)
        t_start = t_rel + 0.03
        z_stop = self.sk.scene["table"]["top_z"] + 0.15
        w = (tc > t_start) & (pc[:, 2] > z_stop) & (tc < t_start + 0.6)
        p0, v0, rms = fit_ballistic(tc[w], pc[w], t_rel)
        # separation = earliest instant from which the cube stays on its own fitted free-flight parabola
        # (residual < 1.5 mm); before it, the cube moves with the palm
        tau = tc - t_rel
        resid = np.linalg.norm(pc - (p0 + np.outer(tau, v0) + 0.5 * np.outer(tau ** 2, G)), axis=1)
        sep_t = None
        cand = np.where((tc > t0) & (tc < t_start))[0]
        for k in cand:
            if np.all(resid[k:np.searchsorted(tc, t_start)] < 0.0015):
                sep_t = float(tc[k])
                break
        ev = v0 - self.plan.v_release
        ang = np.degrees(np.arccos(np.clip(v0 @ self.plan.v_release / (np.linalg.norm(v0) * np.linalg.norm(self.plan.v_release)), -1, 1)))
        tcz, pcz = crossing(p0, v0, t_rel, self.catch_point[2])
        # raw-sample crossing of the catch height (no fit), for comparison
        down = np.where((tc > t_rel + 0.05) & (pc[:, 2] < self.catch_point[2]))[0]
        raw_cross = None
        if len(down):
            k = down[0]
            a = (self.catch_point[2] - pc[k - 1, 2]) / (pc[k, 2] - pc[k - 1, 2])
            raw_cross = pc[k - 1] + a * (pc[k] - pc[k - 1])
            raw_t = tc[k - 1] + a * (tc[k] - tc[k - 1])
        # first table contact (cube centre at resting height + 5 mm)
        z_land = self.sk.scene["table"]["top_z"] + self.sk.scene["cube"]["edge"] / 2 + 0.005
        land = np.where((tc > t_rel) & (pc[:, 2] < z_land))[0]
        land_xy = pc[land[0], :2].tolist() if len(land) else None
        t_land_pred, p_land_pred = crossing(self.plan.p_release, self.plan.v_release, t_rel, z_land)
        # joint tracking at the planned release instant
        qd_meas = np.array([np.interp(t_rel + lag, tj, vj[:, k]) for k in range(6)])
        i_rel = int(np.argmin(np.abs(self.plan.t - self.plan.t_release)))
        qd_err = qd_meas - self.plan.qd[i_rel]
        k_at_rel = float(np.interp(t_rel + lag, tj, kj))
        spin = wc[w].mean(axis=0)
        return dict(
            release_pos_err_mm=(1000 * (p0 - self.plan.p_release)).round(2).tolist(),
            release_vel=v0.round(4).tolist(), release_vel_err=ev.round(4).tolist(),
            release_speed_err=round(float(np.linalg.norm(v0) - np.linalg.norm(self.plan.v_release)), 4),
            release_dir_err_deg=round(float(ang), 3), fit_rms_mm=round(1000 * rms, 3),
            sep_time_err_ms=round(1000 * (sep_t - t_rel), 1) if sep_t else None,
            spin=spin.round(3).tolist(), spin_planned=list(np.round(self.plan.spin, 3)),
            catch_height_cross_err_mm=(1000 * (pcz - self.catch_point)).round(1).tolist() if pcz is not None else None,
            catch_height_cross_raw_err_mm=(1000 * (raw_cross - self.catch_point)).round(1).tolist() if raw_cross is not None else None,
            catch_time_err_ms=round(1000 * (tcz - (t_rel + self.T)), 1) if tcz else None,
            table_landing_xy=land_xy,
            table_landing_err_mm=(1000 * (np.array(land_xy) - p_land_pred[:2])).round(1).tolist() if land_xy else None,
            qd_err_at_release=qd_err.round(3).tolist(), knuckle_at_release=round(k_at_rel, 3),
            traj=dict(t=np.round(tc[tc > t0] - t_rel, 4)[::2].tolist(), p=np.round(pc[tc > t0], 4)[::2].tolist()))


def summarize(rows):
    ok = [r for r in rows if r.get("ok")]
    out = dict(n=len(rows), n_ok=len(ok))
    if not ok:
        return out
    def stat(key, f=lambda v: v):
        a = np.array([f(r[key]) for r in ok if r.get(key) is not None], float)
        return dict(mean=np.round(a.mean(axis=0), 4).tolist(), std=np.round(a.std(axis=0), 4).tolist())
    out.update(release_vel_err=stat("release_vel_err"), release_speed_err=stat("release_speed_err"),
               release_dir_err_deg=stat("release_dir_err_deg"), release_pos_err_mm=stat("release_pos_err_mm"),
               sep_time_err_ms=stat("sep_time_err_ms"), spin=stat("spin"),
               catch_height_cross_err_mm=stat("catch_height_cross_err_mm"),
               catch_height_cross_raw_err_mm=stat("catch_height_cross_raw_err_mm"),
               table_landing_err_mm=stat("table_landing_err_mm"),
               table_landing_xy=stat("table_landing_xy"), qd_err_at_release=stat("qd_err_at_release"),
               fit_rms_mm=stat("fit_rms_mm"))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--throws", type=int, default=20)
    ap.add_argument("--from-pallet", action="store_true")
    ap.add_argument("--out", default="")
    args, _ = ap.parse_known_args()
    rclpy.init()
    e = Eval(args)
    e.sk.mi.move_joints("b", e.poses["b"]["stow"])
    rows = []
    for i in range(args.throws):
        r = e.one(i)
        rows.append(r)
        print("THROW", json.dumps(r), flush=True)
    s = summarize(rows)
    print("SUMMARY", json.dumps(s), flush=True)
    if args.out:
        os.makedirs(os.path.dirname(args.out), exist_ok=True)
        json.dump(dict(plan=dict(p_release=e.plan.p_release.tolist(), v_release=e.plan.v_release.tolist(),
                                 spin=e.plan.spin.tolist(), t_release=e.plan.t_release, flight_time=e.T,
                                 catch_point=list(e.catch_point)), throws=rows, summary=s), open(args.out, "w"), indent=1)
    e.sk.mi.move_joints("b", e.poses["b"]["home"])
    rclpy.shutdown()
