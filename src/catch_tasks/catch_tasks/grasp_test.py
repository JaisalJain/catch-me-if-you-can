"""M2 integration test: static grasp. Cube held for 10 s while the arm moves at moderate speed; slip of the
cube relative to the TCP must stay below 2 mm. Runs against the live sim (bringup).

    ros2 run catch_tasks grasp_test [--robot a] [--color red] [--duration 10]

Slip = max over time of |p_cube_in_TCP(t) - p_cube_in_TCP(t0)| where the cube pose is MuJoCo ground truth
(FreeJointStatePublisherPlugin) and the TCP pose is FK (catch_kinematics, verified == MJCF) of /joint_states,
matched by sim timestamp.
"""
import argparse
import bisect
import json
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from catch_interfaces.srv import ResetScene, SpawnCube
from catch_kinematics import UR5e
from catch_kinematics.transforms import make_tf, quat_wxyz_to_matrix, rot_z
from catch_sim.scene_config import load_scene, robot_base_xyz
from catch_sim.sim_clock import offload_sim_clock
from mujoco_ros2_control_msgs.msg import FreeJointStateArray

from .moveit_client import JN, MoveItClient, _wait

AMP = np.array([0.6, 0.2, 0.25, 0.3, 0.4, 1.5])        # same collision-free motion as tools/grasp_study.py
FREQ = np.array([0.17, 0.23, 0.19, 0.29, 0.21, 0.16])


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default="a")
    ap.add_argument("--color", default="red")
    ap.add_argument("--duration", type=float, default=10.0)
    args, _ = ap.parse_known_args()
    k = args.robot
    rclpy.init()
    node = rclpy.create_node("grasp_test", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    offload_sim_clock(node)          # /clock off the MultiThreadedExecutor (catch_sim.sim_clock)
    threading.Thread(target=ex.spin, daemon=True).start()
    mi = MoveItClient(node)
    assert mi.wait_ready(60), "move_group not ready"
    scene = load_scene()
    poses = yaml.safe_load(open(get_package_share_directory("catch_description") + "/config/poses.yaml"))
    tcp_off = scene["gripper"]["mount_offset"] + scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2
    robot = UR5e(make_tf(rot_z(scene["robots"][k]["base_yaw"]), robot_base_xyz(scene, k)), make_tf(p=[0, 0, tcp_off]))

    js_log, cube_log = [], []
    lock = threading.Lock()

    def on_js(m):
        d = dict(zip(m.name, m.position))
        if all(f"{k}_{j}" in d for j in JN):
            v = dict(zip(m.name, m.velocity))
            with lock:
                js_log.append((stamp(m.header), np.array([d[f"{k}_{j}"] for j in JN]),
                               np.array([v[f"{k}_{j}"] for j in JN])))

    def on_fj(m):
        for f in m.free_joints:
            if f.name == f"cube_{args.color}":
                p, q = f.pose.pose.position, f.pose.pose.orientation
                with lock:
                    cube_log.append((stamp(m.header), np.array([p.x, p.y, p.z]), np.array([q.w, q.x, q.y, q.z])))

    node.create_subscription(JointState, "/joint_states", on_js, 100)
    node.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", on_fj, 100)
    spawn = node.create_client(SpawnCube, "/scene/spawn_cube")
    reset = node.create_client(ResetScene, "/scene/reset")

    print("move to palm_up:", mi.move_joints(k, poses[k]["palm_up"]))
    mi.gripper(k, 0.0)
    r = _wait(spawn.call_async(SpawnCube.Request(color=args.color, where=f"gripper_{k}")), 10)
    print("spawn:", r.success, r.message)
    ok, res = mi.gripper(k, 0.70, max_effort=5.0)
    print(f"grasp: ok={ok} position={mi.gripper_position(k):.3f}")
    time.sleep(1.0)

    # sinusoidal joint trajectory sent straight to the JTC (moderate speed, peak |qd| ~1.5 rad/s)
    q0 = np.array(poses[k]["palm_up"])
    ac = ActionClient(node, FollowJointTrajectory, f"/arm_{k}_controller/follow_joint_trajectory")
    ac.wait_for_server(timeout_sec=10)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f"{k}_{j}" for j in JN]
    T = args.duration

    def env(t):  # smooth envelope: ramps 0->1 over the first second and 1->0 over the last (C1)
        u = np.clip(min(t, T - t), 0.0, 1.0)
        du = (1.0 if t < 1.0 else (-1.0 if t > T - 1.0 else 0.0)) if 0 < u < 1 else 0.0
        return 3 * u ** 2 - 2 * u ** 3, (6 * u - 6 * u ** 2) * du

    for t in np.arange(0.02, T + 1e-9, 0.02):
        e, de = env(t)
        s_, c_ = np.sin(2 * np.pi * FREQ * t), np.cos(2 * np.pi * FREQ * t)
        q = q0 + e * AMP * s_
        qd = e * AMP * 2 * np.pi * FREQ * c_ + de * AMP * s_
        if t > T - 1e-6:
            q, qd = q0, np.zeros(6)
        sec = int(t)
        goal.trajectory.points.append(JointTrajectoryPoint(positions=q.tolist(), velocities=qd.tolist(),
                                                           time_from_start=Duration(sec=sec, nanosec=int((t - sec) * 1e9))))
    with lock:
        js_log.clear()
        cube_log.clear()
    h = _wait(ac.send_goal_async(goal), 10)
    if h is None or not h.accepted:
        print("FAIL: trajectory goal rejected by the JTC")
        rclpy.shutdown()
        sys.exit(1)
    res = _wait(h.get_result_async(), args.duration + 20)
    print("trajectory result:", res.result.error_code if res else "timeout", res.result.error_string if res else "")
    time.sleep(0.5)

    with lock:
        js_t = np.array([s for s, _, _ in js_log])
        js_q = np.array([q for _, q, _ in js_log])
        js_v = np.array([v for _, _, v in js_log])
        cubes = list(cube_log)
    order = np.argsort(js_t)
    js_t, js_q, js_v = js_t[order], js_q[order], js_v[order]

    def slip_for(offset):
        """Cube position in the TCP frame, joints linearly interpolated to the cube stamp (+offset)."""
        rel = []
        for t, p, _ in cubes:
            tq = t + offset
            if tq <= js_t[0] or tq >= js_t[-1]:
                continue
            q = np.array([np.interp(tq, js_t, js_q[:, i]) for i in range(6)])
            T = robot.fk(q)
            rel.append(T[:3, :3].T @ (p - T[:3, 3]))
        rel = np.array(rel)
        return rel, np.linalg.norm(rel - rel[0], axis=1)

    # /joint_states (stamped at the controller update) and the free-joint plugin (stamped at its own update) are
    # not sampled at the same physics instant; a constant pipeline delay shows up as fake slip ~ v_tcp * delay.
    # Estimate the delay as the offset minimising apparent slip and report raw + corrected values.
    offsets = np.arange(-0.010, 0.0031, 0.0005)
    scan = [float(slip_for(o)[1].max()) for o in offsets]
    best = int(np.argmin(scan))
    rel_raw, slip_raw = slip_for(0.0)
    rel, slip = slip_for(offsets[best])
    sens = {"raw_mm": round(1000 * float(slip_raw.max()), 3), "best_offset_ms": round(1000 * float(offsets[best]), 1)}
    qd_peak = float(np.max(np.abs(js_v)))
    result = dict(samples=len(rel), duration=cubes[-1][0] - cubes[0][0], slip_max_mm=1000 * float(slip.max()),
                  slip_final_mm=1000 * float(slip[-1]), cube_offset_in_tcp_mm=(1000 * rel[0]).round(2).tolist(),
                  peak_joint_speed=float(qd_peak), stamp_alignment=sens,
                  passed=bool(slip.max() < 0.002 and cubes[-1][0] - cubes[0][0] > 0.9 * args.duration and qd_peak > 0.5))
    print("RESULT", json.dumps(result))
    mi.gripper(k, 0.0)
    _wait(reset.call_async(ResetScene.Request()), 10)
    mi.move_joints(k, poses[k]["home"])
    rclpy.shutdown()
    sys.exit(0 if result["passed"] else 1)
