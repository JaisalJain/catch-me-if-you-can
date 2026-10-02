"""M2 check: jog both arms through MoveIt (joint goal, pose goal, back home) and open/close both grippers
through MoveIt (gripper groups). Prints a PASS/FAIL line per step.

    ros2 run catch_tasks jog_test
"""
import sys
import threading

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import Constraints, JointConstraint
from rclpy.executors import MultiThreadedExecutor

from catch_kinematics.transforms import make_tf
from catch_sim.sim_clock import offload_sim_clock
from .moveit_client import MoveItClient, pose_from


def main():
    rclpy.init()
    node = rclpy.create_node("jog_test", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    offload_sim_clock(node)          # /clock off the MultiThreadedExecutor (catch_sim.sim_clock)
    threading.Thread(target=ex.spin, daemon=True).start()
    mi = MoveItClient(node)
    if not mi.wait_ready(60):
        print("FAIL move_group not ready")
        sys.exit(1)
    poses = yaml.safe_load(open(get_package_share_directory("catch_description") + "/config/poses.yaml"))
    results = []

    def step(name, ok_msg):
        ok, msg = ok_msg
        results.append(ok)
        print(f"{'PASS' if ok else 'FAIL'} {name}: {msg}", flush=True)

    R_down = np.column_stack([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    for k, sx in (("a", 1), ("b", -1)):
        home = np.array(poses[k]["home"])
        q1 = home + np.array([0.3, -0.2, 0.2, 0.0, 0.0, 0.5])
        step(f"arm_{k} joint goal", mi.move_joints(k, q1))
        err = np.abs(mi.joints(k) - q1).max()
        step(f"arm_{k} joint goal reached (|err| {err:.4f} rad)", (err < 0.01, ""))
        base_x = -0.8 if k == "a" else 0.8
        target = [base_x + sx * 0.45, 0.15, 1.05]
        step(f"arm_{k} pose goal {target}", mi.move_pose(k, pose_from(R_down, target)))
        step(f"arm_{k} back home", mi.move_joints(k, home))
        for name, val in (("close", 0.70), ("open", 0.0)):
            c = Constraints(joint_constraints=[JointConstraint(joint_name=f"{k}_robotiq_85_left_knuckle_joint",
                                                               position=val, tolerance_above=0.01, tolerance_below=0.01,
                                                               weight=1.0)])
            ok, msg = mi._send(mi._request(f"gripper_{k}", c))
            pos = mi.gripper_position(k)
            step(f"gripper_{k} {name} via MoveIt (knuckle {pos:.3f})", (ok and abs(pos - val) < 0.05, msg))
    print(f"SUMMARY {sum(results)}/{len(results)} passed")
    rclpy.shutdown()
    sys.exit(0 if all(results) else 1)
