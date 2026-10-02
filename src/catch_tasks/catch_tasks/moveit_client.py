"""Minimal rclpy client for move_group (moveit_py is not available as a Jazzy binary here).

Wraps: MoveGroup action (joint / pose goals, plan+execute), /compute_cartesian_path + ExecuteTrajectory,
and GripperCommand actions. All calls block (spinning the caller's executor in a background thread).
"""
from __future__ import annotations

import time

import numpy as np
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (BoundingVolume, Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             OrientationConstraint, PlanningOptions, PositionConstraint, RobotState)
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

from catch_sim import gripper_model
from catch_sim.joint_state_cache import JointStateCache

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
        # both grippers run on JointTrajectoryControllers; every command is a rate-limited, delayed ramp
        # (catch_sim.gripper_model: 2F-85 speed + command latency, config/gripper.yaml)
        self.grippers = {}
        self.grip_traj = {r: ActionClient(node, FollowJointTrajectory, f"/gripper_{r}_controller/follow_joint_trajectory")
                          for r in "ab"}
        self.gripper_profile = gripper_model.load_profile()
        self._last_grip_cmd = {}
        # latest /joint_states on a helper thread (EventsExecutor): a 500 Hz subscription on the caller's
        # MultiThreadedExecutor cost ~50-95 % of a core (catch_sim.joint_state_cache)
        self.js = getattr(node, "_joint_state_cache", None) or JointStateCache(node)
        node._joint_state_cache = self.js

    def joints(self, robot):
        return np.array(self.js.positions([f"{robot}_{j}" for j in JN]))

    def joint_velocities(self, robot):
        return np.array(self.js.velocities([f"{robot}_{j}" for j in JN]))

    def gripper_position(self, robot):
        return self.js.position(f"{robot}_robotiq_85_left_knuckle_joint")

    def wait_ready(self, timeout=60.0):
        ok = self.move.wait_for_server(timeout_sec=timeout) and self.exec.wait_for_server(timeout_sec=timeout)
        ok = ok and self.cart.wait_for_service(timeout_sec=timeout)
        return ok and self.js.wait(timeout)

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

    def move_pose(self, robot, pose: Pose, pos_tol=0.002, ang_tol=0.01, frame="world", attempts=2):
        """Pose goal for the TCP; retried once with doubled planning time (OMPL is randomised)."""
        ok, msg = False, ""
        for i in range(attempts):
            ok, msg = self._move_pose_once(robot, pose, pos_tol, ang_tol, frame, self.planning_time * (1 + i))
            if ok:
                break
        return ok, msg

    def _move_pose_once(self, robot, pose, pos_tol, ang_tol, frame, planning_time):
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
        req = self._request(f"arm_{robot}", c)
        req.allowed_planning_time = planning_time
        return self._send(req)

    def move_cartesian(self, robot, poses, max_step=0.005, vel_scale=0.2, min_fraction=0.99, frame="world",
                       avoid_collisions=True):
        """Straight-line TCP motion through `poses` (geometry_msgs/Pose list)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = frame
        req.group_name = f"arm_{robot}"
        req.link_name = f"{robot}_tcp"
        req.waypoints = list(poses)
        req.max_step = max_step
        req.avoid_collisions = avoid_collisions
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
        """position: driver joint angle, 0 = open, 0.8 = closed. Returns (ok, result).
        Blocks until the fingers stop (reached or stalled on an object). Rate-limited + delayed like a 2F-85."""
        if robot in self.grip_traj:
            return self._gripper_traj(robot, position, timeout)
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

    def gripper_traj_goal(self, robot, points, stamp=None):
        """FollowJointTrajectory goal for a trajectory-controlled gripper: points = [(t_from_start, position)]."""
        from builtin_interfaces.msg import Duration
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = [f"{robot}_robotiq_85_left_knuckle_joint"]
        if stamp is not None:
            g.trajectory.header.stamp = stamp
        for t, q in points:
            sec = int(t)
            g.trajectory.points.append(JointTrajectoryPoint(
                positions=[float(q)], time_from_start=Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)))))
        return g

    def gripper_ramp_goal(self, robot, position, stamp=None, q_hold=None):
        """2F-85 model ramp from the fingers' actual position to `position`, starting one command latency after
        `stamp` (default: on receipt)."""
        q_from = self.gripper_position(robot)
        if q_hold is None:
            q_hold = self._last_grip_cmd.get(robot, q_from)
        prof = self.gripper_profile
        pts = gripper_model.ramp_points(q_hold, q_from, position, prof["gap_speed"], prof["latency"])
        # stamp None = zero stamp: the JTC starts the trajectory when it receives it (immediate command; a stamp
        # of "now" can arrive after a short ramp has already ended and is then rejected). Timed commands pass a stamp.
        self._last_grip_cmd[robot] = float(position)
        return self.gripper_traj_goal(robot, pts, stamp=stamp)

    def _gripper_traj(self, robot, position, timeout, still=0.1):
        ac = self.grip_traj[robot]
        ac.wait_for_server(timeout_sec=5.0)
        h = _wait(ac.send_goal_async(self.gripper_ramp_goal(robot, position)), 5.0)
        if h is None or not h.accepted:
            return False, None
        r = _wait(h.get_result_async(), timeout)
        if r is None:
            return False, None
        # the controller finishes after `ramp`; wait for the fingers themselves to stop (reached or stalled)
        t0 = time.monotonic()
        t_still = None
        while time.monotonic() - t0 < timeout:
            v = abs(self.js.velocity(f"{robot}_robotiq_85_left_knuckle_joint"))
            if v < 0.01:
                t_still = t_still or time.monotonic()
                if time.monotonic() - t_still > still and time.monotonic() - t0 > 0.1:
                    break
            else:
                t_still = None
            time.sleep(0.005)
        return r.result.error_code == 0, r.result

    def gripper_async(self, robot, position, max_effort=5.0):
        if robot in self.grip_traj:
            return self.grip_traj[robot].send_goal_async(self.gripper_ramp_goal(robot, position))
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
