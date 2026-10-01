"""M3: overhead detection accuracy vs MuJoCo ground truth over randomised layouts on A's pallet.

    ros2 run catch_perception eval_detection [--layouts 20] [--seed 0]
Layouts alternate between (a) cubes in their slots with random offsets (+-5 mm) and yaw (+-15 deg) and
(b) cubes loose on the table next to the pallet with any yaw. Prints per-layout and aggregate errors.
"""
import argparse
import json
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor

from catch_interfaces.msg import CubeDetectionArray
from catch_interfaces.srv import ResetScene
from catch_sim.scene_config import load_scene, slot_position
from mujoco_ros2_control_msgs.msg import FreeJointState, FreeJointStateArray
from mujoco_ros2_control_msgs.srv import SetFreeJointState

COLORS = ("red", "green", "blue")


def yaw_of(q):
    return np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def fold(a):
    return (a + np.pi / 4) % (np.pi / 2) - np.pi / 4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--layouts", type=int, default=20)
    ap.add_argument("--seed", type=int, default=0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    n = rclpy.create_node("eval_detection", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    threading.Thread(target=ex.spin, daemon=True).start()
    st = {}
    n.create_subscription(CubeDetectionArray, "/perception/overhead_a/cubes", lambda m: st.__setitem__("det", m), 1)
    n.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", lambda m: st.__setitem__("gt", m), 1)
    setfj = n.create_client(SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state")
    reset = n.create_client(ResetScene, "/scene/reset")
    setfj.wait_for_service(timeout_sec=10)
    scene = load_scene()
    rng = np.random.default_rng(args.seed)
    top = scene["table"]["top_z"]
    half = scene["cube"]["edge"] / 2
    pc = scene["pallets"]["a"]["center"]
    errs, yaw_errs, misses, extras, kinds = [], [], 0, 0, []
    for k in range(args.layouts):
        entries = []
        loose = k % 2 == 1
        for i, c in enumerate(COLORS):
            if loose:   # loose on the table in front of the pallet (y = pallet_y + 0.12), spread in x
                p = np.array([pc[0] + (i - 1) * 0.10 + rng.uniform(-0.02, 0.02), pc[1] + 0.13 + rng.uniform(-0.02, 0.02),
                              top + half])
                yaw = rng.uniform(-np.pi, np.pi)
            else:
                p = slot_position(scene, "a", c) + np.r_[rng.uniform(-0.005, 0.005, 2), 0]
                yaw = rng.uniform(-np.radians(15), np.radians(15))
            e = FreeJointState(name=f"cube_{c}")
            e.pose.pose.position.x, e.pose.pose.position.y, e.pose.pose.position.z = [float(v) for v in p + [0, 0, 0.002]]
            e.pose.pose.orientation.z, e.pose.pose.orientation.w = float(np.sin(yaw / 2)), float(np.cos(yaw / 2))
            entries.append(e)
        f = setfj.call_async(SetFreeJointState.Request(free_joints=entries))
        while not f.done():
            time.sleep(0.01)
        # wait for a detection computed from an image rendered >= 1.0 s (sim) after the teleport (settled, fresh)
        t_ready = n.get_clock().now().nanoseconds * 1e-9 + 1.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10.0:
            dm = st.get("det")
            if dm is not None and dm.header.stamp.sec + dm.header.stamp.nanosec * 1e-9 >= t_ready:
                break
            time.sleep(0.02)
        det = {d.color: d for d in st["det"].detections}
        gt = {f.name.replace("cube_", ""): f for f in st["gt"].free_joints}
        extras += max(0, len(st["det"].detections) - len(COLORS))
        line = []
        for c in COLORS:
            if c not in det:
                misses += 1
                line.append(f"{c}: MISS")
                continue
            g, d = gt[c].pose.pose, det[c].pose
            e = 1000 * np.hypot(d.position.x - g.position.x, d.position.y - g.position.y)
            ye = np.degrees(abs(fold(yaw_of(d.orientation) - yaw_of(g.orientation))))
            errs.append(e)
            yaw_errs.append(ye)
            kinds.append("loose" if loose else "slots")
            line.append(f"{c}: {e:.1f} mm {ye:.1f} deg")
        print(f"layout {k:2d} ({'loose' if loose else 'slots'}): " + ", ".join(line), flush=True)
    rf = reset.call_async(ResetScene.Request())
    while not rf.done():
        time.sleep(0.01)
    errs, yaw_errs, kinds = np.array(errs), np.array(yaw_errs), np.array(kinds)
    res = dict(n=int(len(errs)), misses=misses, extra_detections=extras)
    for name, sel in (("all", np.ones(len(errs), bool)), ("slots", kinds == "slots"), ("loose", kinds == "loose")):
        res[name] = dict(n=int(sel.sum()), xy_err_mean_mm=round(float(errs[sel].mean()), 2),
                         xy_err_p95_mm=round(float(np.percentile(errs[sel], 95)), 2),
                         xy_err_max_mm=round(float(errs[sel].max()), 2),
                         yaw_err_mean_deg=round(float(yaw_errs[sel].mean()), 2),
                         yaw_err_max_deg=round(float(yaw_errs[sel].max()), 2))
    print("RESULT", json.dumps(res))
    rclpy.shutdown()
