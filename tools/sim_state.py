#!/usr/bin/env python3
"""Print arm joint errors w.r.t. poses.yaml home and cube ground-truth vs slot positions (system python + ROS)."""
import sys
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import JointState

from catch_sim.scene_config import load_scene, slot_position
from mujoco_ros2_control_msgs.msg import FreeJointStateArray

JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
rclpy.init()
n = rclpy.create_node("sim_state")
js, fj = {}, {}
n.create_subscription(JointState, "/joint_states", lambda m: js.update(dict(zip(m.name, m.position))), 10)
n.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states",
                      lambda m: fj.update({f.name: f for f in m.free_joints}), 10)
t = time.time()
while time.time() - t < 1.5:
    rclpy.spin_once(n, timeout_sec=0.05)
poses = yaml.safe_load(open(get_package_share_directory("catch_description") + "/config/poses.yaml"))
for k in "ab":
    q = np.array([js.get(f"{k}_{j}", np.nan) for j in JN])
    print(f"arm {k}: q={np.round(q, 4).tolist()} |q-home|max={np.nanmax(np.abs(q - poses[k]['home'])):.5f} rad, "
          f"gripper={js.get(f'{k}_robotiq_85_left_knuckle_joint', np.nan):.4f}")
scene = load_scene()
for c in ("red", "green", "blue"):
    f = fj.get(f"cube_{c}")
    if f is None:
        print(f"cube_{c}: no data")
        continue
    p = np.array([f.pose.pose.position.x, f.pose.pose.position.y, f.pose.pose.position.z])
    print(f"cube_{c}: pos={np.round(p, 4).tolist()} err to A slot={1000 * np.linalg.norm(p - slot_position(scene, 'a', c)):.2f} mm "
          f"|v|={np.linalg.norm([f.twist.twist.linear.x, f.twist.twist.linear.y, f.twist.twist.linear.z]):.4f}")
