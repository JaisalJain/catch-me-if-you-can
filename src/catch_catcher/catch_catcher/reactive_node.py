"""v2 robot B: reactive catch from the ready pose (action /reactive_catch, catch_interfaces/ReactiveCatch).

    ros2 run catch_catcher reactive_catcher [--ros-args -p source:=vision|plan]

Prediction source (fixed per node, parameter `source`):
  * vision (default): /tracker/cube only (catch_tracker: two cameras, ballistic LSQ). This node then has NO
    subscription to A's /throw_plan and no /throw/preview client (test_reactive_source.py checks it).
  * plan: A's /throw_plan (the published, nominal plan; with release noise it is wrong by design) — the
    "plan-informed" benchmark baseline, action /reactive_catch_plan.
Per goal: B goes to the ready pose (MoveIt, collision-checked), opens to the pre-gap (56 mm) and reports "ready".
From the first estimate whose 95 % intercept error is <= commit_e95_mm B commits: it picks the intercept on the
predicted descending path with the most limit margin (catch_catcher.reactive) and replaces arm_b_controller's
trajectory (JointTrajectory topic, quintic with inverse-dynamics feed-forward) as the estimate improves, at most every
replan_period, until freeze_before the intercept. Every plan is limit- and collision-checked (capsules incl. arm A).
Close: the timed close is scheduled in the gripper controller in advance (sim-time stamp; 2F-85 model ramp:
catch_sim.gripper_model, B compensates its assumed command latency); a palm contact before that instant closes
immediately (primary trigger); `trigger` in the result says which one fired. Abort (back to the ready pose) when no
feasible intercept exists inside the catch envelope.
"""
import math
import os
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Point, WrenchStamped
from rclpy.action import ActionClient, ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.experimental import EventsExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from catch_interfaces.action import ReactiveCatch
from catch_interfaces.msg import CubeTrack
from catch_kinematics import UR5e
from catch_kinematics.collision import CapsuleModel
from catch_kinematics.dynamics import ArmDynamics
from catch_kinematics.transforms import make_tf, rot_z
from catch_sim import gripper_model
from catch_sim.joint_state_cache import JointStateCache
from catch_sim.sim_clock import offload_sim_clock
from catch_tasks.moveit_client import MoveItClient

from .reactive import ReactivePlanner, ballistic, resolve_config, sample_times

JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
CHI3_95 = 2.7955


def s_of(t):
    return t.sec + t.nanosec * 1e-9


def to_time(t):
    sec = int(math.floor(t))
    return Time(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def to_dur(t):
    sec = int(math.floor(t))
    return Duration(sec=sec, nanosec=int(round((t - sec) * 1e9)) % 1_000_000_000)


def load(pkg, rel):
    with open(os.path.join(get_package_share_directory(pkg), rel)) as f:
        return yaml.safe_load(f)


def make_robot(scene, key):
    r = scene["robots"][key]
    base = [r["base_xy"][0], r["base_xy"][1], scene["table"]["top_z"] + r.get("pedestal_height", 0.0)]
    tcp = scene["gripper"]["mount_offset"] + scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2
    return UR5e(make_tf(rot_z(r["base_yaw"]), base), make_tf(p=[0, 0, tcp]))


class Inputs:
    """Latest estimate (vision: /tracker/cube; plan: /throw_plan) and the palm wrench, received by a helper node
    on an EventsExecutor thread (the 125 / 500 Hz streams never touch the action node's executor)."""

    def __init__(self, node, source):
        self.lock = threading.Lock()
        self.track = None            # latest in-flight CubeTrack
        self.n_track = 0
        self.plan = None             # latest ThrowPlan (plan source only)
        self.wrench = []             # (stamp, |F|) ring
        self.armed = None            # dict while watching for contact
        self.helper = rclpy.create_node(f"{node.get_name()}_inputs", context=node.context,
                                        parameter_overrides=[Parameter("use_sim_time", value=False)],
                                        start_parameter_services=False)
        if source == "vision":
            self.helper.create_subscription(CubeTrack, "/tracker/cube", self.on_track, 50)
        else:
            from catch_interfaces.msg import ThrowPlan
            self.helper.create_subscription(ThrowPlan, "/throw_plan", self.on_plan,
                                            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.helper.create_subscription(WrenchStamped, "/b_palm_ft_broadcaster/wrench", self.on_wrench, 1000)
        self.ex = EventsExecutor(context=node.context)
        self.ex.add_node(self.helper)
        threading.Thread(target=self._spin, daemon=True).start()

    def _spin(self):
        try:
            self.ex.spin()
        except ExternalShutdownException:
            pass

    def on_track(self, m):
        if not m.in_flight:
            return
        with self.lock:
            self.track = m
            self.n_track += 1

    def on_plan(self, m):
        with self.lock:
            self.plan = m

    def on_wrench(self, m):
        f = np.array([m.wrench.force.x, m.wrench.force.y, m.wrench.force.z])
        t = s_of(m.header.stamp)
        a = self.armed
        if a is None:
            return
        if a["f0"] is None:
            a["base"].append(f)
            if t >= a["t_arm"] and len(a["base"]) >= 20:
                a["f0"] = np.mean(a["base"][-100:], axis=0)
            return
        mag = float(np.linalg.norm(f - a["f0"]))
        a["peak"] = max(a["peak"], mag)
        if a["t_contact"] is None and mag > a["thr"]:
            a["t_contact"] = t


class ReactiveCatcher(Node):
    def __init__(self, source=None):
        name = "reactive_catcher"
        super().__init__(name, parameter_overrides=[Parameter("source", value=source)] if source else [])
        self.declare_parameter("source", "vision")
        self.source = self.get_parameter("source").value
        if self.source not in ("vision", "plan"):
            raise ValueError("source must be vision or plan")
        self.cfg = resolve_config(load("catch_catcher", "config/reactive.yaml"))
        self.declare_parameter("latency_assumed", 0.010)   # B's compensation of the gripper command latency
        scene = load("catch_description", "config/scene.yaml")
        self.robot, self.robot_a = make_robot(scene, "b"), make_robot(scene, "a")
        self.dyn, self.caps = ArmDynamics(), CapsuleModel()
        self.q_ready = np.array(self.cfg["ready"]["q"])
        self.planner = ReactivePlanner(self.robot, self.dyn, self.caps, self.cfg, self.q_ready,
                                       self.cfg["ready"]["elev_deg"], self.robot_a)
        self.k_pre = gripper_model.knuckle_for_gap(self.cfg["pre_gap_mm"])
        self.grip_model = gripper_model.load_profile(robot="b")
        self.delay_est = None           # measured gripper delay (latency + actuator lag), delay_probe
        cb = ReentrantCallbackGroup()
        self._joint_state_cache = JointStateCache(self)
        self.js = self._joint_state_cache
        self.mi = MoveItClient(self)
        self.inputs = Inputs(self, self.source)
        self.traj_pub = self.create_publisher(JointTrajectory, "/arm_b_controller/joint_trajectory", 10)
        self.grip = ActionClient(self, FollowJointTrajectory, "/gripper_b_controller/follow_joint_trajectory",
                                 callback_group=cb)
        action = "/reactive_catch" if self.source == "vision" else "/reactive_catch_plan"
        ActionServer(self, ReactiveCatch, action, self.execute, callback_group=cb)
        self.get_logger().info(f"reactive catcher: source {self.source}, action {action}, ready pose "
                               f"{self.cfg['ready_pose']} {np.round(self.q_ready, 3).tolist()}, ik_branch "
                               f"{self.cfg['ik_branch']} (branch {self.robot.branch(self.q_ready)}), gripper model "
                               f"{self.grip_model['name']} ({1000 * self.grip_model['gap_speed']:.0f} mm/s, "
                               f"{1000 * self.grip_model['latency']:.0f} ms), assumed latency "
                               f"{1000 * self.get_parameter('latency_assumed').value:.0f} ms")

    def all_nodes(self):
        return [self, self.inputs.helper, self.js.helper]

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ commands
    def send_plan(self, plan):
        jt = JointTrajectory()
        jt.header.stamp = to_time(plan.t0)
        jt.joint_names = [f"b_{j}" for j in JN]
        T = plan.seg.T
        dt = self.cfg["traj_dt"]
        ts = sample_times(T, dt)
        S = [plan.seg(t) for t in ts]
        Q = np.array([x[0] for x in S] + [plan.q_goal])
        QD = np.array([x[1] for x in S] + [np.zeros(6)])
        QDD = np.array([x[2] for x in S] + [np.zeros(6)])
        TAU = self.planner.feedforward(Q, QD, QDD)
        for k, t in enumerate(ts + [T + 0.2]):
            jt.points.append(JointTrajectoryPoint(positions=Q[k].tolist(), velocities=QD[k].tolist(),
                                                  accelerations=QDD[k].tolist(), effort=TAU[k].tolist(),
                                                  time_from_start=to_dur(t)))
        self.traj_pub.publish(jt)

    def schedule_close(self, t_cmd):
        """Gripper close commanded at sim time t_cmd (JTC-executed 2F-85 model ramp from the pre-gap)."""
        gm = self.grip_model
        pts = gripper_model.ramp_points(self.k_pre, self.k_pre, self.cfg_close(), gm["gap_speed"], gm["latency"])
        goal = self.mi.gripper_traj_goal("b", pts, stamp=to_time(t_cmd))
        self.grip.send_goal_async(goal)

    def cfg_close(self):
        return 0.70

    def probe_delay(self):
        """Gripper delay from the fingers' observed motion (/joint_states only): for moves k_pre -> k_pre+dk -> k_pre,
        (time the knuckle crosses the move's midpoint) - (command sent) - (the 2F-85 ramp's own travel time to the
        midpoint at the specified gap speed). = command latency + actuator lag + transport. Median of 2n moves."""
        pc = self.cfg["delay_probe"]
        kn = "b_robotiq_85_left_knuckle_joint"
        gs = self.grip_model["gap_speed"]
        samples = []
        for _ in range(pc["n"]):
            for target in (self.k_pre + pc["dk"], self.k_pre):
                k0 = self.js.position(kn)
                mid = 0.5 * (k0 + target)
                t_send = self.now()
                self.mi.gripper_async("b", target)
                t_mid = None
                while self.now() < t_send + pc["timeout"]:
                    st, js = self.js.snapshot()          # position and its stamp from the same message
                    k = js.get(kn, (float("nan"), 0.0))[0]
                    if st is not None and (k - mid) * (target - k0) >= 0.0:
                        t_mid = st
                        break
                    time.sleep(0.0005)
                if t_mid is not None:
                    travel = 0.002 + abs(gripper_model.gap_for_knuckle(k0) - gripper_model.gap_for_knuckle(mid)) / 1000.0 / gs
                    samples.append(t_mid - t_send - travel)
                t_s = self.now()
                while self.now() < t_s + pc["settle"]:
                    time.sleep(0.005)
        if len(samples) < pc["n"]:
            self.get_logger().warn(f"gripper delay probe: only {len(samples)} samples; keeping the assumed latency")
            return None
        d = float(np.median(samples))
        self.get_logger().info(f"gripper delay probe: {1000 * d:.1f} ms (latency + actuator lag; samples "
                               f"{', '.join(f'{1000 * x:.1f}' for x in samples)} ms)")
        return d

    def move_direct(self, q_goal, speed_ratio=None):
        """Joint-space quintic from rest to rest straight to q_goal on arm_b_controller (not move_group, which
        serves A meanwhile), at most speed_ratio of 180 deg/s and within the planner's speed / torque limits,
        collision-checked with the capsule model (environment, self, arm A at its current configuration).
        Returns (ok, message); waits until the move is done."""
        speed_ratio = speed_ratio or self.cfg["prepare"]["speed_ratio"]
        q0 = np.array(self.mi.joints("b"))
        dq = float(np.max(np.abs(q_goal - q0)))
        if dq < 1e-3:
            return True, "already there"
        T = max(self.cfg["prepare"]["min_time"], 1.875 * dq / (speed_ratio * np.pi))
        q_a = np.array(self.js.positions([f"a_{j}" for j in JN]))
        t0 = self.now() + self.cfg["send_lead"]
        from .reactive import Plan, Quintic
        seg = Quintic(q0, 0, 0, q_goal, 0, 0, T)
        ratio = self.planner.limit_ratio(seg)
        clr = self.planner.clearance(seg, q_a)
        if ratio > 1.0 or clr < self.caps.min_clearance:
            return False, f"direct move refused (limit ratio {ratio:.2f}, clearance {1000 * clr:.0f} mm)"
        self.send_plan(Plan(t0, seg, np.asarray(q_goal, float), t0 + T, t0 + T, np.zeros(3), np.zeros(3), ratio, clr))
        while self.now() < t0 + T + 0.15:
            time.sleep(0.005)
        err = float(np.max(np.abs(np.array(self.mi.joints("b")) - q_goal)))
        return err < 0.01, f"direct move {T:.2f} s, clearance >= {1000 * clr:.0f} mm, final error {err:.4f} rad"

    def get_ready(self, gh, direct):
        """Ready pose (direct joint move or MoveIt), pre-gap, delay probe. (ok, message)."""
        gh.publish_feedback(ReactiveCatch.Feedback(phase="to ready"))
        msg = "at the ready pose"
        if np.max(np.abs(self.mi.joints("b") - self.q_ready)) > 0.01:
            ok = False
            if direct:
                ok, msg = self.move_direct(self.q_ready)
                if not ok:
                    self.get_logger().warn(f"{msg}; falling back to MoveIt")
            if not ok:
                self.mi.wait_ready(30)
                ok, msg = self.mi.move_joints("b", self.q_ready)
                if not ok:
                    return False, f"could not reach the ready pose: {msg}"
        self.mi.gripper("b", self.k_pre, timeout=5.0)
        if self.delay_est is None and self.cfg.get("delay_probe", {}).get("enabled"):
            self.delay_est = self.probe_delay() or 0.0
        return True, msg

    # ------------------------------------------------------------------ estimates
    def estimate(self, t_after):
        """(stamp, pred(t), e95 [m], t_first) of the newest estimate after t_after, or None."""
        with self.inputs.lock:
            tr, pl = self.inputs.track, self.inputs.plan
        if self.source == "vision":
            if tr is None or s_of(tr.header.stamp) <= t_after:
                return None
            st = s_of(tr.header.stamp)
            pred = ballistic([tr.position.x, tr.position.y, tr.position.z], [tr.velocity.x, tr.velocity.y, tr.velocity.z], st)
            var = np.array(tr.intercept_covariance).reshape(3, 3)
            e95 = CHI3_95 * math.sqrt(max(np.trace(var), 0.0) / 3.0)
            return st, pred, e95, s_of(tr.t_first)
        if pl is None or s_of(pl.release_time) <= t_after:
            return None
        tr_ = s_of(pl.release_time)
        pred = ballistic([pl.release_position.x, pl.release_position.y, pl.release_position.z],
                         [pl.release_velocity.x, pl.release_velocity.y, pl.release_velocity.z], tr_)
        return tr_, pred, 0.0, tr_

    # ------------------------------------------------------------------ action
    def execute(self, gh):
        g = gh.request
        mode = g.mode or "pinch"
        res = ReactiveCatch.Result()
        res.trigger = "none"
        cfg = self.cfg
        res.ready_pose = f"{cfg['ready_pose']}/{cfg['ik_branch']}"
        if g.prepare_only:
            ok, msg = self.get_ready(gh, direct=True)
            res.success, res.t_ready, res.message = ok, self.now(), msg
            (gh.succeed if ok else gh.abort)()
            return res
        if not g.skip_ready_move:
            ok, msg = self.get_ready(gh, direct=False)
            if not ok:
                gh.abort()
                res.message = msg
                return res
        # contact watch (baseline from now until the commit)
        t_ready = self.now()
        self.inputs.armed = dict(base=[], f0=None, t_arm=t_ready + 0.1, thr=cfg["contact_threshold_n"], peak=0.0,
                                 t_contact=None)
        gh.publish_feedback(ReactiveCatch.Feedback(phase="ready"))
        timeout = g.timeout or cfg["flight_timeout"]
        t_limit = t_ready + timeout
        latency_assumed = self.get_parameter("latency_assumed").value
        gm = self.grip_model
        close_offset = g.close_offset if (g.close_offset and not math.isnan(g.close_offset)) else cfg[mode].get("close_offset", 0.0)
        # ---- wait for a committable estimate
        est, t_first_est = None, 0.0
        while self.now() < t_limit:
            e = self.estimate(t_ready)
            if e is not None:
                if not t_first_est:
                    t_first_est = self.now()
                if e[2] * 1000.0 <= cfg["commit_e95_mm"]:
                    est = e
                    break
            time.sleep(0.001)
        res.t_first_estimate = t_first_est
        if est is None:
            self.inputs.armed = None
            gh.abort()
            res.message = "no flight seen" if not t_first_est else "estimate never good enough to commit"
            res.aborted, res.abort_reason = True, res.message
            return res
        stamp, pred, e95, t_first = est
        t_commit = self.now()
        res.t_commit = t_commit - t_first
        q_a = np.array(self.js.positions([f"a_{j}" for j in JN]))
        state = (self.mi.joints("b"), np.zeros(6), np.zeros(6))
        t0 = t_commit + cfg["commit_lead"]
        tp = time.perf_counter()
        plan, reasons = self.planner.search(t0, state, pred, max(t0, stamp), mode, q_a)
        plan_ms = [1000 * (time.perf_counter() - tp)]
        if plan is None:
            self.inputs.armed = None
            res.aborted = True
            res.abort_reason = f"no feasible intercept in the envelope ({reasons})"
            res.message = res.abort_reason
            self.get_logger().warn(res.message)
            gh.succeed()
            return res
        self.send_plan(plan)
        gh.publish_feedback(ReactiveCatch.Feedback(phase="committed"))
        n_sent, t_last_send, ratios, clrs = 1, self.now(), [plan.ratio], [plan.clearance]
        vt = [(plan.info.get("vel_ratio", 0.0), plan.info.get("torque_ratio", 0.0))]
        n_kept = 0

        def t_close_for(p):
            if mode == "pinch":
                d_gap = (self.cfg["pre_gap_mm"] - 40.0) / 1000.0
                if self.delay_est:          # measured delay: the fingers (not the reference) at 40 mm on time
                    return p.t_intercept + cfg["pinch"]["gap40_after_arrival"] - (self.delay_est + 0.002 + d_gap / gm["gap_speed"])
                return p.t_intercept + close_offset - (latency_assumed + 0.002 + d_gap / gm["gap_speed"])
            return p.t_intercept + cfg["cup"]["fallback_after"]

        t_close = t_close_for(plan)
        self.schedule_close(t_close)
        # ---- refine
        last_stamp = stamp
        while True:
            now = self.now()
            a = self.inputs.armed
            if a and a["t_contact"] is not None and res.trigger == "none" and now < t_close:
                # palm contact before the scheduled close: close now (preempts the scheduled goal)
                self.mi.gripper_async("b", self.cfg_close())
                res.trigger, res.t_close_cmd = "contact", now
            if now >= plan.t_intercept - cfg["freeze_before"]:
                break
            if now - t_last_send >= cfg["replan_period"]:
                e = self.estimate(last_stamp)
                if e is not None:
                    last_stamp, pred = e[0], e[1]
                    t0 = now + cfg["send_lead"]
                    st = plan.state(t0)
                    q_a = np.array(self.js.positions([f"a_{j}" for j in JN]))
                    tp = time.perf_counter()
                    p_c, v_c = pred(plan.t_intercept)
                    new, why = self.planner.plan_to(t0, st, plan.t_intercept, p_c, v_c, mode, q_a)
                    if why != "ok":
                        new2, reasons = self.planner.search(t0, st, pred, max(t0, last_stamp), mode, q_a)
                        if new2 is not None:
                            new, why = new2, "ok"
                    plan_ms.append(1000 * (time.perf_counter() - tp))
                    if why != "ok":
                        # no better plan within the limits: keep the current one (a late small correction often
                        # exceeds the limits); abort only if the predicted intercept left the catch envelope
                        c_ = cfg
                        outside = (p_c[2] < c_["z_min"] or not (c_["box_lo"][0] <= p_c[0] <= c_["box_hi"][0])
                                   or not (c_["box_lo"][1] <= p_c[1] <= c_["box_hi"][1])
                                   or self.planner.goal(p_c, v_c, mode, plan.q_goal) is None)
                        n_kept += 1
                        t_last_send = now
                        if not outside:
                            time.sleep(0.001)
                            continue
                        self.send_plan(self.planner.return_plan(t0, st))
                        self.mi.gripper_async("b", self.k_pre)
                        self.inputs.armed = None
                        res.aborted, res.abort_reason = True, f"predicted intercept left the envelope ({why})"
                        res.message = f"aborted after {n_sent} plans: {res.abort_reason}"
                        res.n_replans = n_sent - 1
                        self.get_logger().warn(res.message)
                        gh.succeed()
                        return res
                    if abs(new.t_intercept - plan.t_intercept) > 1e-6:
                        t_close = t_close_for(new)
                        self.schedule_close(t_close)
                    plan = new
                    self.send_plan(plan)
                    n_sent += 1
                    t_last_send = now
                    ratios.append(plan.ratio)
                    clrs.append(plan.clearance)
                    vt.append((plan.info.get("vel_ratio", 0.0), plan.info.get("torque_ratio", 0.0)))
            time.sleep(0.001)
        # ---- intercept: contact (primary) or the scheduled timed close
        t_end = plan.t_intercept + cfg["settle"]
        while self.now() < t_end:
            now = self.now()
            a = self.inputs.armed
            if a and a["t_contact"] is not None and res.trigger == "none":
                if now < t_close:
                    self.mi.gripper_async("b", self.cfg_close())
                    res.trigger, res.t_close_cmd = "contact", now
                else:
                    res.trigger, res.t_close_cmd = "timed", t_close
            if res.trigger == "none" and now >= t_close:
                res.trigger, res.t_close_cmd = "timed", t_close
            time.sleep(0.001)
        a = self.inputs.armed or {}
        self.inputs.armed = None
        k = self.mi.gripper_position("b")
        lo, hi = cfg["in_hand_knuckle"]
        res.success = bool(lo <= k <= hi)
        res.knuckle = float(k)
        res.t_contact = float(a.get("t_contact") or 0.0)
        res.t_intercept = plan.t_intercept
        res.intercept = Point(x=float(plan.p[0]), y=float(plan.p[1]), z=float(plan.p[2]))
        res.n_replans = n_sent - 1
        span = max(plan.t_intercept - cfg["freeze_before"] - t_commit, 1e-3)
        res.replan_rate_hz = (n_sent - 1) / span
        res.max_limit_ratio = float(max(ratios))
        res.max_vel_ratio = float(max(x[0] for x in vt))
        res.max_torque_ratio = float(max(x[1] for x in vt))
        res.min_clearance = float(min(clrs))
        res.plan_ms_median = float(np.median(plan_ms))
        res.message = (f"{'held' if res.success else 'not held'} ({mode}, {self.source}): trigger {res.trigger}, "
                       + (f"gripper delay {1000 * self.delay_est:.1f} ms (probed), " if self.delay_est else "")
                       + f"commit {1000 * res.t_commit:.0f} ms after first fix, {res.n_replans} replans ({n_kept} kept) "
                       f"({res.replan_rate_hz:.0f} Hz, {res.plan_ms_median:.1f} ms each), limit ratio <= "
                       f"{res.max_limit_ratio:.2f} (speed {res.max_vel_ratio:.2f}, torque {res.max_torque_ratio:.2f} of the "
                       f"limits), ready {res.ready_pose}, clearance >= {1000 * res.min_clearance:.0f} mm, knuckle {k:.3f}")
        self.get_logger().info(res.message)
        gh.succeed()
        return res


def main():
    rclpy.init()
    n = ReactiveCatcher()
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(n)
    offload_sim_clock(n)
    try:
        ex.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
