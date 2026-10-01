"""Robot A throw executor: action server /throw (catch_interfaces/Throw).

On a goal it (1) generates the toss trajectory from catch_throw/config/throw.yaml (catch_throw.generator),
(2) checks that arm A is at the stroke start (the caller moves it there with MoveIt), (3) publishes the
ThrowPlan (latched) with the predicted release time in sim time, (4) sends the time-parameterised joint
trajectory directly to arm_a_controller with a future start stamp and (5) loosens the gripper at
t_loosen after the stroke start (sim clock). The cube is released by the arm's deceleration, not by the
fingers.
"""
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import Point, Vector3
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

from catch_ballistics import velocity_for_launch_angle
from catch_interfaces.action import Throw
from catch_interfaces.msg import ThrowPlan
from catch_interfaces.srv import GetThrowPlan
from catch_kinematics import UR5e
from catch_kinematics.transforms import make_tf, rot_z

from .generator import generate_throw
from .profile import ProfileParams

JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def load_yaml(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def to_time(t):
    sec = int(np.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def to_dur(t):
    sec = int(np.floor(t))
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def build_plan(cfg=None, scene=None):
    """Plan the configured throw (pure computation, also used by tests and the evaluation tool)."""
    cfg = cfg or load_yaml("catch_throw", "config/throw.yaml")["throw"]
    scene = scene or load_yaml("catch_description", "config/scene.yaml")
    r = scene["robots"]["a"]
    base = [r["base_xy"][0], r["base_xy"][1], scene["table"]["top_z"] + r.get("pedestal_height", 0.0)]
    tcp = scene["gripper"]["mount_offset"] + scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2
    robot = UR5e(make_tf(rot_z(r["base_yaw"]), base), make_tf(p=[0, 0, tcp]))
    p, c = np.array(cfg["release_point"]), np.array(cfg["catch_point"])
    v, T = velocity_for_launch_angle(p, c, np.radians(cfg["launch_angle_deg"]))
    plan = generate_throw(robot, p, v, cfg["q_release_seed"], ProfileParams(**cfg["profile"]),
                          finger_axis=cfg["finger_axis"], pitch_rate=cfg["pitch_rate"], dt=cfg["sample_dt"])
    return plan, T, c, cfg


def load_feedforward(n_samples):
    """Feed-forward torque table from tools/throw_feedforward.py; None (with reason) if missing or stale."""
    import hashlib
    share = get_package_share_directory("catch_throw")
    path = os.path.join(share, "config", "throw_ff.yaml")
    if not os.path.exists(path):
        return None, "no throw_ff.yaml"
    ff = yaml.safe_load(open(path))
    cfg = yaml.safe_load(open(os.path.join(share, "config", "throw.yaml")))["throw"]
    cfg.pop("release", None)            # gripper timing does not change the trajectory
    h = hashlib.sha256(yaml.safe_dump(cfg, sort_keys=True).encode())
    for f in (os.path.join(get_package_share_directory("catch_description"), "config", "scene.yaml"),
              os.path.join(get_package_share_directory("catch_description"), "config", "physics.yaml")):
        h.update(open(f, "rb").read())
    if ff["inputs_hash"] != h.hexdigest()[:16]:
        return None, "throw_ff.yaml is stale (re-run tools/throw_feedforward.py)"
    if ff["n"] != n_samples:
        return None, f"throw_ff.yaml has {ff['n']} samples, plan has {n_samples}"
    return np.array(ff["tau"]), "ok"


def plan_msg(plan, flight_time, catch_point, cube_id="", t0=None, now=0.0):
    """ThrowPlan message; with t0=None the absolute times are left zero (preview)."""
    msg = ThrowPlan()
    msg.header.stamp = to_time(now)
    msg.header.frame_id = "world"
    msg.cube_id = cube_id
    msg.release_position = Point(x=float(plan.p_release[0]), y=float(plan.p_release[1]), z=float(plan.p_release[2]))
    msg.release_velocity = Vector3(x=float(plan.v_release[0]), y=float(plan.v_release[1]), z=float(plan.v_release[2]))
    msg.release_spin = Vector3(x=float(plan.spin[0]), y=float(plan.spin[1]), z=float(plan.spin[2]))
    if t0 is not None:
        msg.release_time = to_time(t0 + plan.t_release)
        msg.stroke_start_time = to_time(t0)
    msg.flight_time = float(flight_time)
    msg.catch_point = Point(x=float(catch_point[0]), y=float(catch_point[1]), z=float(catch_point[2]))
    return msg


class ThrowNode(Node):
    def __init__(self):
        super().__init__("throw_node")
        self.declare_parameter("start_lead", 0.5)          # s of sim time between commit and stroke start
        self.declare_parameter("start_tolerance", 0.02)    # rad, arm must be this close to the stroke start
        self.declare_parameter("use_feedforward", True)
        cb = ReentrantCallbackGroup()
        self.plan, self.flight_time, self.catch_point, self.cfg = build_plan()
        rel = self.cfg.get("release", {})
        self.declare_parameter("t_loosen", float(rel.get("t_loosen", 0.12)))   # overridable for studies
        self.t_loosen = self.get_parameter("t_loosen").value
        self.loosen_to = float(rel.get("loosen_to", 0.30))
        self.loosen_effort = float(rel.get("loosen_effort", 5.0))
        self.ff, why = load_feedforward(len(self.plan.t))
        if self.ff is None:
            self.get_logger().warn(f"no torque feed-forward: {why}")
        self.js = {}
        self.create_subscription(JointState, "/joint_states", lambda m: self.js.update(dict(zip(m.name, m.position))),
                                 10, callback_group=cb)
        self.plan_pub = self.create_publisher(ThrowPlan, "/throw_plan",
                                              QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.jtc = ActionClient(self, FollowJointTrajectory, "/arm_a_controller/follow_joint_trajectory",
                                callback_group=cb)
        self.grip = ActionClient(self, GripperCommand, "/gripper_a_controller/gripper_cmd", callback_group=cb)
        ActionServer(self, Throw, "/throw", self.execute, callback_group=cb)
        self.create_service(GetThrowPlan, "/throw/preview", self.on_preview, callback_group=cb)
        m = self.plan.metrics
        self.get_logger().info(
            f"throw plan: release {np.round(self.plan.p_release, 3).tolist()} v {np.round(self.plan.v_release, 3).tolist()} "
            f"(|v| {np.linalg.norm(self.plan.v_release):.3f} m/s) spin {np.round(self.plan.spin, 2).tolist()} rad/s, "
            f"t_release {self.plan.t_release:.3f} s into a {m['duration']:.3f} s stroke; peak |qd|/limit "
            f"{m['vel_ratio_max']:.2f}; loosen gripper to {self.loosen_to} at t={self.t_loosen:.3f} s")

    def on_preview(self, req, res):
        res.success = True
        res.plan = plan_msg(self.plan, self.flight_time, self.catch_point, now=self.now())
        return res

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _wait(self, fut, timeout):
        t0 = time.monotonic()
        while not fut.done() and time.monotonic() - t0 < timeout:
            time.sleep(0.002)
        return fut.result() if fut.done() else None

    def _sleep_until(self, t_sim):
        while self.now() < t_sim - 0.0005:
            time.sleep(0.0005)

    def execute(self, gh):
        res = Throw.Result()
        q = np.array([self.js.get(f"a_{j}", np.nan) for j in JN])
        err = float(np.max(np.abs(q - self.plan.q_start)))
        if not np.isfinite(err) or err > self.get_parameter("start_tolerance").value:
            gh.abort()
            res.success, res.message = False, f"arm A not at the stroke start (|dq| {err:.3f} rad)"
            return res
        lead = self.get_parameter("start_lead").value
        self.t_loosen = self.get_parameter("t_loosen").value
        t0 = self.now() + lead
        p = self.plan
        msg = plan_msg(p, self.flight_time, self.catch_point, gh.request.cube_id, t0, self.now())
        self.plan_pub.publish(msg)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.header.stamp = to_time(t0)
        goal.trajectory.joint_names = [f"a_{j}" for j in JN]
        use_ff = self.ff is not None and self.get_parameter("use_feedforward").value
        for i in range(len(p.t)):
            pt = JointTrajectoryPoint(positions=p.q[i].tolist(), velocities=p.qd[i].tolist(),
                                      accelerations=p.qdd[i].tolist(), time_from_start=to_dur(p.t[i]))
            if use_ff:
                pt.effort = self.ff[i].tolist()
            goal.trajectory.points.append(pt)
        last = goal.trajectory.points[-1]
        last.velocities, last.accelerations = [0.0] * 6, [0.0] * 6
        end = JointTrajectoryPoint(positions=p.q[-1].tolist(), velocities=[0.0] * 6, accelerations=[0.0] * 6,
                                   time_from_start=to_dur(p.t[-1] + 0.3))
        if use_ff:
            last.effort, end.effort = [0.0] * 6, [0.0] * 6
        goal.trajectory.points.append(end)
        gh.publish_feedback(Throw.Feedback(phase="committed"))
        h = self._wait(self.jtc.send_goal_async(goal), 5.0)
        if h is None or not h.accepted:
            gh.abort()
            res.success, res.message = False, "JTC rejected the throw trajectory"
            return res
        # loosen the grip during the acceleration phase (sim clock)
        self._sleep_until(t0 + self.t_loosen)
        g = GripperCommand.Goal()
        g.command.position, g.command.max_effort = self.loosen_to, self.loosen_effort
        self.grip.send_goal_async(g)
        t_cmd = self.now()
        gh.publish_feedback(Throw.Feedback(phase=f"gripper loosened at t={t_cmd - t0:.4f}"))
        r = self._wait(h.get_result_async(), p.t[-1] + lead + 5.0)
        ok = r is not None and r.result.error_code == 0
        res.success = bool(ok)
        res.message = (f"stroke done (feed-forward {'on' if use_ff else 'off'}); gripper loosen command at "
                       f"{t_cmd - t0:.4f} s (planned {self.t_loosen:.4f})"
                       if ok else f"JTC error {r.result.error_code if r else 'timeout'} {r.result.error_string if r else ''}")
        res.plan = msg
        if ok:
            gh.succeed()
        else:
            gh.abort()
        self.get_logger().info(res.message)
        return res


def main():
    rclpy.init()
    n = ThrowNode()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    ex.spin()
