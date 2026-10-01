"""Minimal rclpy client for move_group (moveit_py is not available as a Jazzy binary here).

Wraps: MoveGroup action (joint / pose goals, plan+execute), /compute_cartesian_path + ExecuteTrajectory,
and GripperCommand actions. All calls block (spinning the caller's executor in a background thread).
"""
from __future__ import annotations

import threading
import time

import numpy as np
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PlanningOptions, PositionConstraint, RobotState)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def _wait(fut, timeout):
    t0 = time.monotonic()
    while not fut.done():
        if time.monotonic() - t0 > timeout:
            return None
        time.sleep(0.005)
    return fut.result()


class MoveItClient:
    """Use from a node that is spun by another thread (e.g. MultiThreadedExecutor in a background thread)."""

    def __init__(self, node: Node, vel_scale=0.5, acc_scale=0.5, planning_time=3.0):
        self.node = node
        self.vel_scale, self.acc_scale, self.planning_time = vel_scale, acc_scale, planning_time
        self.move = ActionClient(node, MoveGroup, "/move_action")
        self.exec = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
        self.cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.grippers = {k: ActionClient(node, GripperCommand, f"/gripper_{k}_controller/gripper_cmd") for k in "ab"}
        self._js = {}
        self._lock = threading.Lock()
        node.create_subscription(JointState, "/joint_states", self._on_js, 10)

    def _on_js(self, m):
        with self._lock:
            self._js.update(dict(zip(m.name, zip(m.position, m.velocity))))

    def joints(self, robot):
        with self._lock:
            return np.array([self._js[f"{robot}_{j}"][0] for j in JN])

    def gripper_position(self, robot):
        with self._lock:
            return self._js[f"{robot}_robotiq_85_left_knuckle_joint"][0]

    def wait_ready(self, timeout=60.0):
        ok = self.move.wait_for_server(timeout_sec=timeout) and self.exec.wait_for_server(timeout_sec=timeout)
        ok = ok and self.cart.wait_for_service(timeout_sec=timeout)
        t0 = time.monotonic()
        while not self._js and time.monotonic() - t0 < timeout:
            time.sleep(0.05)
        return ok and bool(self._js)

    # ---------------------------------------------------------------- goals
    def _request(self, group, constraints):
        r = MotionPlanRequest()
        r.group_name = group
        r.num_planning_attempts = 5
        r.allowed_planning_time = self.planning_time
        r.max_velocity_scaling_factor = self.vel_scale
        r.max_acceleration_scaling_factor = self.acc_scale
        r.pipeline_id = "ompl"
        r.goal_constraints = [constraints]
        r.start_state = RobotState(is_diff=True)
        return r

    def _send(self, req, timeout=60.0):
        goal = MoveGroup.Goal(request=req, planning_options=PlanningOptions(plan_only=False, replan=True,
                                                                                replan_attempts=2))
        h = _wait(self.move.send_goal_async(goal), 10.0)
        if h is None or not h.accepted:
            return False, "goal rejected"
        res = _wait(h.get_result_async(), timeout)
        if res is None:
            return False, "timeout"
        code = res.result.error_code.val
        return code == MoveItErrorCodes.SUCCESS, f"MoveItErrorCode {code}"

    def move_joints(self, robot, q, tol=1e-3):
        c = Constraints()
        for j, v in zip(JN, q):
            c.joint_constraints.append(JointConstraint(joint_name=f"{robot}_{j}", position=float(v), tolerance_above=tol,
                                                       tolerance_below=tol, weight=1.0))
        return self._send(self._request(f"arm_{robot}", c))

    def move_pose(self, robot, pose: Pose, pos_tol=0.002, ang_tol=0.01, frame="world"):
        c = Constraints()
        link = f"{robot}_tcp"
        pc = PositionConstraint(link_name=link, weight=1.0)
        pc.header.frame_id = frame
        pc.constraint_region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[pos_tol])],
                                              primitive_poses=[Pose(position=pose.position)])
        pc.constraint_region.primitive_poses[0].orientation.w = 1.0
        oc = OrientationConstraint(link_name=link, orientation=pose.orientation, weight=1.0,
                                   absolute_x_axis_tolerance=ang_tol, absolute_y_axis_tolerance=ang_tol,
                                   absolute_z_axis_tolerance=ang_tol)
        oc.header.frame_id = frame
        c.position_constraints.append(pc)
        c.orientation_constraints.append(oc)
        return self._send(self._request(f"arm_{robot}", c))

    def move_cartesian(self, robot, poses, max_step=0.005, vel_scale=0.2, min_fraction=0.99, frame="world"):
        """Straight-line TCP motion through `poses` (geometry_msgs/Pose list)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = frame
        req.group_name = f"arm_{robot}"
        req.link_name = f"{robot}_tcp"
        req.waypoints = list(poses)
        req.max_step = max_step
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = vel_scale
        req.max_acceleration_scaling_factor = vel_scale
        req.start_state = RobotState(is_diff=True)
        res = _wait(self.cart.call_async(req), 10.0)
        if res is None:
            return False, "cartesian service timeout"
        if res.fraction < min_fraction:
            return False, f"cartesian path only {res.fraction:.2f} feasible"
        h = _wait(self.exec.send_goal_async(ExecuteTrajectory.Goal(trajectory=res.solution)), 10.0)
        if h is None or not h.accepted:
            return False, "execute rejected"
        r = _wait(h.get_result_async(), 60.0)
        if r is None:
            return False, "execute timeout"
        return r.result.error_code.val == MoveItErrorCodes.SUCCESS, f"MoveItErrorCode {r.result.error_code.val}"

    def gripper(self, robot, position, max_effort=5.0, timeout=5.0):
        """position: driver joint angle, 0 = open, 0.8 = closed. Returns (ok, result)."""
        ac = self.grippers[robot]
        ac.wait_for_server(timeout_sec=5.0)
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = float(max_effort)
        h = _wait(ac.send_goal_async(g), 5.0)
        if h is None or not h.accepted:
            return False, None
        r = _wait(h.get_result_async(), timeout)
        if r is None:
            return False, None
        return bool(r.result.reached_goal or r.result.stalled), r.result

    def gripper_async(self, robot, position, max_effort=5.0):
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = float(max_effort)
        return self.grippers[robot].send_goal_async(g)


def pose_from(R, p):
    from catch_kinematics.transforms import matrix_to_quat_wxyz
    w, x, y, z = matrix_to_quat_wxyz(R)
    ps = Pose()
    ps.position.x, ps.position.y, ps.position.z = [float(v) for v in p]
    ps.orientation.w, ps.orientation.x, ps.orientation.y, ps.orientation.z = float(w), float(x), float(y), float(z)
    return ps
