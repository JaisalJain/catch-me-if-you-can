"""v2 reactive intercept planning for robot B (numpy only; used by reactive_node, unit-tested).

From a predicted cube path (position/velocity at any time) B picks an intercept time on the descending branch and a
TCP pose there, and plans one synchronized quintic from its current desired state (position, velocity,
acceleration) to rest at that pose:
  * pinch: closing axis horizontal across the velocity, approach axis = the natural ready axis (made orthogonal to
    the closing axis); B arrives at rest at the intercept time; the fingers (pre-opened) close on timing;
  * cup:   palm normal against the velocity, limited to a cone around the ready axis; B is at rest arrive_early
    before the intercept; the fingers close on palm contact.
Every plan is checked against the limits (joint speed <= 85 %, inverse-dynamics torque with gravity <= 80 %
(catch_kinematics.dynamics), joint margins) and for collisions (capsule model incl. arm A, catch_kinematics.collision)
before it is sent. v2 Phase 1 (tools/v2_catch_sim.py) is the offline version of the same logic.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from catch_kinematics import POS_LIMIT, TORQUE_LIMIT, VEL_LIMIT
from catch_kinematics.transforms import make_tf
from catch_throw.generator import toss_orientation

G = np.array([0.0, 0.0, -9.81])


class Quintic:
    """Per-joint quintic from (q0, v0, a0) at t=0 to (q1, v1, a1) at t=T."""

    def __init__(self, q0, v0, a0, q1, v1, a1, T):
        q0 = np.asarray(q0, float)
        v0, a0, q1, v1, a1 = (np.broadcast_to(np.asarray(x, float), q0.shape) for x in (v0, a0, q1, v1, a1))
        self.T = float(T)
        T = self.T
        h = q1 - q0 - v0 * T - 0.5 * a0 * T * T
        dv = v1 - v0 - a0 * T
        da = a1 - a0
        self.c = np.array([q0, v0, 0.5 * a0, (10 * h - 4 * dv * T + 0.5 * da * T * T) / T ** 3,
                           (-15 * h + 7 * dv * T - da * T * T) / T ** 4, (6 * h - 3 * dv * T + 0.5 * da * T * T) / T ** 5])

    def __call__(self, t):
        t = float(np.clip(t, 0.0, self.T))
        c = self.c
        q = c[0] + t * (c[1] + t * (c[2] + t * (c[3] + t * (c[4] + t * c[5]))))
        qd = c[1] + t * (2 * c[2] + t * (3 * c[3] + t * (4 * c[4] + t * 5 * c[5])))
        qdd = 2 * c[2] + t * (6 * c[3] + t * (12 * c[4] + t * 20 * c[5]))
        return q, qd, qdd


@dataclass
class Plan:
    t0: float                 # absolute start time (sim)
    seg: Quintic
    q_goal: np.ndarray
    t_arrive: float           # absolute: B at rest at q_goal
    t_intercept: float        # absolute: the cube reaches the intercept point
    p: np.ndarray             # intercept point (= TCP)
    v: np.ndarray             # predicted cube velocity there
    ratio: float              # max(speed / cap, torque / cap)
    clearance: float          # capsule model, min over the trajectory
    info: dict = field(default_factory=dict)

    def state(self, t):
        if t >= self.t0 + self.seg.T:
            return self.q_goal.copy(), np.zeros(6), np.zeros(6)
        return self.seg(max(0.0, t - self.t0))


def closing_axis_for(v):
    a = np.cross([0, 0, 1.0], v)
    if np.linalg.norm(a) < 1e-6:
        return np.array([0.0, 1.0, 0.0])
    a /= np.linalg.norm(a)
    return a if a[1] >= 0 else -a


def rotate_towards(z0, z1, max_angle):
    z0, z1 = np.asarray(z0, float) / np.linalg.norm(z0), np.asarray(z1, float) / np.linalg.norm(z1)
    ang = np.arccos(np.clip(z0 @ z1, -1, 1))
    if ang <= max_angle or ang < 1e-9:
        return z1
    ax = np.cross(z0, z1)
    ax /= np.linalg.norm(ax)
    a = max_angle
    z = z0 * np.cos(a) + np.cross(ax, z0) * np.sin(a) + ax * (ax @ z0) * (1 - np.cos(a))
    return z / np.linalg.norm(z)


def catch_rotation(z_ready, v, cone, mode):
    xa = closing_axis_for(v)
    if mode == "cup":
        z = rotate_towards(z_ready, -np.asarray(v) / np.linalg.norm(v), cone)
    else:
        z = np.asarray(z_ready, float) - (np.asarray(z_ready) @ xa) * xa
        z /= np.linalg.norm(z)
    return toss_orientation(z, xa)


def ready_rotation(elev_deg):
    e = np.radians(elev_deg)
    return toss_orientation([-np.cos(e), 0.0, np.sin(e)], [0.0, 1.0, 0.0])


class ReactivePlanner:
    def __init__(self, robot, dyn, caps, cfg, q_ready, ready_elev_deg, robot_a=None):
        self.robot, self.dyn, self.caps, self.cfg = robot, dyn, caps, cfg
        self.robot_a = robot_a
        self.q_ready = np.asarray(q_ready, float)
        self.z_ready = ready_rotation(ready_elev_deg)[:, 2]
        self.lim = cfg["limits"]

    # ------------------------------------------------------------------ pieces
    def goal(self, p, v, mode, q_ref):
        cone = np.radians(self.cfg[mode]["cone_deg"])
        R = catch_rotation(self.z_ready, v, cone, mode)
        q = self.robot.ik_closest(make_tf(R, p), q_ref)
        if q is None or np.max(np.abs(q - self.q_ready)) > self.cfg["max_joint_jump"]:
            return None
        if np.min(POS_LIMIT - np.abs(q)) < self.lim["pos_margin"] or abs(np.sin(q[4])) < 0.25:
            return None
        return q

    def check(self, seg, q_a=None):
        """(limit ratio, clearance) of a quintic."""
        n = self.cfg["check_samples"]
        ts = np.linspace(0, seg.T, n)
        S = [seg(t) for t in ts]
        Q = np.array([s[0] for s in S])
        QD = np.array([s[1] for s in S])
        QDD = np.array([s[2] for s in S])
        vel = float(np.max(np.abs(QD) / VEL_LIMIT))
        tau = self.dyn.torques(Q, QD, QDD, gravity=True)
        tq = float(np.max(np.abs(tau) / TORQUE_LIMIT))
        ratio = max(vel / self.lim["vel_ratio"], tq / self.lim["torque_ratio"])
        if np.min(POS_LIMIT - np.abs(Q)) < self.lim["pos_margin"]:
            ratio = max(ratio, 9.0)
        # collisions: every ~collision_dt along the trajectory
        m = max(2, int(np.ceil(seg.T / self.cfg["collision_dt"])) + 1)
        Qc = np.array([seg(t)[0] for t in np.linspace(0, seg.T, m)])
        extra = None
        if q_a is not None and self.robot_a is not None:
            A0, A1 = self.caps.segments(self.robot_a, q_a)
            extra = (A0[0], A1[0], self.caps.r)
        clr = self.caps.clearance(self.robot, Qc, env=True, self_check=True, extra_capsules=extra)
        return ratio, clr

    def plan_to(self, t_now, state, t_c, p, v, mode, q_a=None):
        q0, qd0, qdd0 = state
        t_arr = t_c - (self.cfg["cup"]["arrive_early"] if mode == "cup" else 0.0)
        T = t_arr - t_now
        if T < self.cfg["min_move_time"]:
            return None, "too late"
        q1 = self.goal(p, v, mode, q0)
        if q1 is None:
            return None, "no IK / margins"
        seg = Quintic(q0, qd0, qdd0, q1, 0, 0, T)
        ratio, clr = self.check(seg, q_a)
        plan = Plan(t_now, seg, q1, t_arr, t_c, np.asarray(p), np.asarray(v), ratio, clr)
        if ratio > 1.0:
            return plan, "over limits"
        if clr < self.caps.min_clearance:
            return plan, "collision"
        return plan, "ok"

    def search(self, t_now, state, pred, t_from, mode, q_a=None):
        """Best intercept on the predicted path: the feasible one with the lowest limit ratio. pred(t) -> (p, v)."""
        c = self.cfg
        best, reasons = None, {}
        for t_c in np.arange(max(t_from, t_now + c["min_move_time"]), t_now + c["horizon"], c["search_dt"]):
            p, v = pred(t_c)
            if v[2] >= 0:
                continue
            if p[2] < c["z_min"] or not (c["box_lo"][0] <= p[0] <= c["box_hi"][0]) or not (c["box_lo"][1] <= p[1] <= c["box_hi"][1]):
                if p[2] < c["z_min"]:
                    break
                reasons["outside envelope box"] = reasons.get("outside envelope box", 0) + 1
                continue
            plan, why = self.plan_to(t_now, state, t_c, p, v, mode, q_a)
            reasons[why] = reasons.get(why, 0) + 1
            if why == "ok" and (best is None or plan.ratio < best.ratio):
                best = plan
        return best, reasons

    def return_plan(self, t_now, state, duration=None):
        """Back to the ready pose (abort)."""
        q0, qd0, qdd0 = state
        T = duration or self.cfg["return_time"]
        seg = Quintic(q0, qd0, qdd0, self.q_ready, 0, 0, T)
        return Plan(t_now, seg, self.q_ready.copy(), t_now + T, t_now + T, np.zeros(3), np.zeros(3), 0.0, 1.0)

    def feedforward(self, q, qd, qdd):
        """Effort feed-forward for the JTC (the sim compensates gravity itself)."""
        return self.dyn.rnea(q, qd, qdd, gravity=False)


def ballistic(p0, v0, t0):
    """pred(t) for a ballistic path through p0 with velocity v0 at time t0."""
    p0, v0 = np.asarray(p0, float), np.asarray(v0, float)

    def pred(t):
        tau = t - t0
        return p0 + v0 * tau + 0.5 * G * tau * tau, v0 + G * tau
    return pred
