"""Robot B catcher: action /catch (catch_interfaces/Catch).

1. From the throw plan in the goal (a preview from /throw/preview) choose the intercept and the cup pose
   (tool z against the incoming velocity, fingers along y, fully open) and move there with MoveIt (joint goal
   from the closed-form IK, collision-checked).
2. Feedback "ready"; wait for the live /throw_plan (absolute release time).
3. Trigger:
   * "force" (default): the palm force sensor (MuJoCo force sensor on the welded palm insert, exported via
     ros2_control + force_torque_sensor_broadcaster) rises `force_threshold` above its pre-release baseline;
   * "time" (baseline only): predicted arrival + time_offset.
   The gripper close command is sent from the force callback itself.
4. Settle, judge "held" from the knuckle angle (no ground truth, no weld).
"""
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from control_msgs.action import GripperCommand
from geometry_msgs.msg import Point, WrenchStamped
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from catch_interfaces.action import Catch
from catch_interfaces.msg import ThrowPlan
from catch_kinematics import UR5e
from catch_kinematics.transforms import make_tf, rot_z
from catch_tasks.moveit_client import MoveItClient

from .intercept import choose_intercept


def stamp_s(t):
    return t.sec + t.nanosec * 1e-9


def load(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


class Catcher(Node):
    def __init__(self):
        super().__init__("catcher")
        self.cfg = load("catch_catcher", "config/catch.yaml")
        scene = load("catch_description", "config/scene.yaml")
        self.poses = load("catch_description", "config/poses.yaml")
        r = scene["robots"]["b"]
        base = [r["base_xy"][0], r["base_xy"][1], scene["table"]["top_z"] + r.get("pedestal_height", 0.0)]
        tcp = scene["gripper"]["mount_offset"] + scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2
        self.robot = UR5e(make_tf(rot_z(r["base_yaw"]), base), make_tf(p=[0, 0, tcp]))
        cb = ReentrantCallbackGroup()
        self.force_cb = MutuallyExclusiveCallbackGroup()
        self.mi = MoveItClient(self)
        self.grip = ActionClient(self, GripperCommand, "/gripper_b_controller/gripper_cmd", callback_group=cb)
        self.lock = threading.Lock()
        self.live_plan = None
        self.create_subscription(ThrowPlan, "/throw_plan", self.on_plan,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL), callback_group=cb)
        self.wrench_sub = None   # created per catch goal (500 Hz in rclpy)
        self.n_wrench = 0
        self.arm = None          # dict while armed: baseline samples, threshold, trigger state
        ActionServer(self, Catch, "/catch", self.execute, callback_group=cb)

    # ------------------------------------------------------------------ callbacks
    def on_plan(self, msg):
        with self.lock:
            self.live_plan = msg

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_wrench(self, msg):
        self.n_wrench += 1
        a = self.arm
        if a is None:
            return
        f = np.array([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        t = stamp_s(msg.header.stamp)
        if t < a["t_watch"]:
            a["base"].append(f)
            a["base"] = a["base"][-a["n_base"]:]
            return
        if a["f0"] is None:
            a["f0"] = np.mean(a["base"], axis=0) if a["base"] else f
        mag = float(np.linalg.norm(f - a["f0"]))
        a["peak"] = max(a["peak"], mag)
        if a["mode"] == "force" and a["t_trig"] is None and mag > a["thr"]:
            self._close()
            a["t_cmd"] = self.now()
            a["t_trig"] = t          # set last: the action thread proceeds as soon as it sees t_trig

    def _disarm(self):
        self.arm = None
        if self.wrench_sub is not None:
            self.destroy_subscription(self.wrench_sub)
            self.wrench_sub = None

    def _close(self):
        g = GripperCommand.Goal()
        g.command.position = float(self.cfg["gripper"]["close"])
        g.command.max_effort = float(self.cfg["gripper"]["effort"])
        self.grip.send_goal_async(g)

    def _open(self):
        g = GripperCommand.Goal()
        g.command.position = float(self.cfg["gripper"]["open"])
        g.command.max_effort = float(self.cfg["gripper"]["effort"])
        f = self.grip.send_goal_async(g)
        t0 = time.monotonic()
        while not f.done() and time.monotonic() - t0 < 5:
            time.sleep(0.01)
        if f.done() and f.result().accepted:
            r = f.result().get_result_async()
            while not r.done() and time.monotonic() - t0 < 5:
                time.sleep(0.01)

    # ------------------------------------------------------------------ action
    def execute(self, gh):
        g = gh.request
        res = Catch.Result()
        p = g.plan
        p_r = np.array([p.release_position.x, p.release_position.y, p.release_position.z])
        v_r = np.array([p.release_velocity.x, p.release_velocity.y, p.release_velocity.z])
        ic_cfg = self.cfg["intercept"]
        ic = choose_intercept(self.robot, p_r, v_r, np.array(self.poses["b"]["home"]), ic_cfg["mode"],
                              p.flight_time, ic_cfg["z_window"], ic_cfg["min_joint_margin"])
        if ic is None:
            gh.abort()
            res.message = "no reachable intercept"
            return res
        if g.cup_offset:
            from .intercept import cup_ik
            p_off = ic.p + ic.R[:, 1] * g.cup_offset
            _, q_off = cup_ik(self.robot, p_off, ic.v, ic.q, min_margin=ic_cfg["min_joint_margin"])
            if q_off is None:
                gh.abort()
                res.message = "no IK for the offset cup pose"
                return res
            self.get_logger().warn(f"EXPERIMENT: cup pose shifted {1000 * g.cup_offset:+.1f} mm along tool y")
            ic.q = q_off
        res.intercept = Point(x=float(ic.p[0]), y=float(ic.p[1]), z=float(ic.p[2]))
        # palm force stream: subscribe now (DDS discovery of a new subscription can take a while) and require
        # samples before reporting "ready"; unsubscribed again after the catch (500 Hz is costly in rclpy)
        self.n_wrench = 0
        # deep queue (2 s): a lagging executor must not drop the 1-2 sample impact spike
        self.wrench_sub = self.create_subscription(WrenchStamped, "/b_palm_ft_broadcaster/wrench", self.on_wrench, 1000,
                                                   callback_group=self.force_cb)
        gh.publish_feedback(Catch.Feedback(phase="moving"))
        self.mi.wait_ready(30)
        self._open()
        ok, msg = self.mi.move_joints("b", ic.q)
        if not ok:
            self._disarm()
            gh.abort()
            res.message = f"could not reach the cup pose: {msg}"
            return res
        self._open()
        t0 = time.monotonic()
        while self.n_wrench < 50 and time.monotonic() - t0 < 10.0:
            time.sleep(0.01)
        if self.n_wrench < 50:
            self._disarm()
            gh.abort()
            res.message = "no palm force data (b_palm_ft_broadcaster)"
            return res
        with self.lock:
            self.live_plan = None
        t_goal = self.now()
        gh.publish_feedback(Catch.Feedback(phase="ready"))
        # wait for the live plan published after "ready"
        timeout = g.timeout or self.cfg["plan_timeout"]
        t0 = time.monotonic()
        live = None
        while time.monotonic() - t0 < timeout:
            with self.lock:
                lp = self.live_plan
            if lp is not None and stamp_s(lp.release_time) > t_goal:
                live = lp
                break
            time.sleep(0.005)
        if live is None:
            self._disarm()
            gh.abort()
            res.message = "no live throw plan"
            return res
        t_rel = stamp_s(live.release_time)
        t_arrive = t_rel + ic.t
        tr = self.cfg["trigger"]
        mode = g.trigger or "force"
        self.arm = dict(mode=mode, thr=tr["force_threshold"], t_watch=t_rel - tr["arm_before_release"],
                        n_base=int(tr["baseline_window"] * 500), base=[], f0=None, peak=0.0, t_trig=None, t_cmd=None)
        gh.publish_feedback(Catch.Feedback(phase="armed"))
        if mode == "time":
            while self.now() < t_arrive + g.time_offset - 0.0005:
                time.sleep(0.0005)
            t = self.now()
            self._close()
            self.arm["t_cmd"] = self.now()
            self.arm["t_trig"] = t
        else:
            while self.arm["t_trig"] is None and self.now() < t_arrive + 0.5:
                time.sleep(0.001)
        a = self.arm
        if a["t_trig"] is None:
            self._disarm()
            gh.abort()
            res.message = "no contact detected"
            res.peak_force = a["peak"]
            return res
        gh.publish_feedback(Catch.Feedback(phase="closing"))
        t_end = a["t_cmd"] + self.cfg["settle_time"]
        while self.now() < t_end:
            time.sleep(0.005)
        self._disarm()
        lo, hi = self.cfg["gripper"]["in_hand_knuckle"]
        k = self.mi.gripper_position("b")
        res.success = bool(lo <= k <= hi)
        res.knuckle = float(k)
        res.peak_force = float(a["peak"])
        res.trigger_time_rel = float(a["t_trig"] - t_arrive)
        res.message = (f"{'held' if res.success else 'not held'}: {mode} trigger at {1000 * res.trigger_time_rel:+.1f} ms "
                       f"vs predicted arrival, peak palm force {a['peak']:.1f} N, knuckle {k:.3f}")
        self.get_logger().info(res.message)
        gh.succeed()
        return res


def main():
    rclpy.init()
    n = Catcher()
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(n)
    ex.spin()
