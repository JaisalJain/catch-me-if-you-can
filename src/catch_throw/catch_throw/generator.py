"""Throw-trajectory generator: straight-line toss of the TCP along the release direction, mapped to joints.

Everything here is numpy-only so it runs in the offline venv and inside the ROS node.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from catch_ballistics import Projectile
from catch_kinematics import POS_LIMIT, VEL_LIMIT, UR5e
from catch_kinematics.transforms import axis_angle, make_tf

from .profile import G, ProfileParams, TossProfile


def toss_orientation(n, finger_axis=(0.0, 1.0, 0.0)):
    """TCP rotation with tool z = n (palm normal along the release direction), tool x along finger_axis
    (the 2F-85 opens along tool x), projected to be orthogonal to n."""
    n = np.asarray(n, float) / np.linalg.norm(n)
    x = np.asarray(finger_axis, float)
    x = x - (x @ n) * n
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(n, x), n])


@dataclass
class ThrowPlan:
    t: np.ndarray                 # [s] from start of the stroke
    q: np.ndarray                 # (N, 6)
    qd: np.ndarray
    qdd: np.ndarray
    tcp: np.ndarray               # (N, 3) TCP positions
    t_release: float              # predicted separation time
    p_release: np.ndarray
    v_release: np.ndarray
    profile: TossProfile
    q_start: np.ndarray
    q_end: np.ndarray
    spin: np.ndarray = field(default_factory=lambda: np.zeros(3))   # cube angular velocity at release
    metrics: dict = field(default_factory=dict)

    def flight(self) -> Projectile:
        return Projectile(self.p_release, self.v_release, t0=self.t_release)


class InfeasibleThrow(RuntimeError):
    pass


def pitch_axis(n):
    """Horizontal axis perpendicular to the throw plane (rotation about it pitches the palm forward)."""
    a = np.cross([0.0, 0.0, 1.0], n)
    return a / np.linalg.norm(a)


def release_joint_velocity(robot: UR5e, q_r, n, speed, pitch_rate):
    """q_dot at release for TCP velocity speed*n and palm angular velocity pitch_rate*speed about pitch_axis."""
    return np.linalg.solve(robot.jacobian(q_r), np.r_[speed * n, pitch_rate * speed * pitch_axis(n)])


def generate_throw(robot: UR5e, p_release, v_release, q_seed, params: ProfileParams,
                   finger_axis=(0.0, 1.0, 0.0), pitch_rate: float = 0.0, dt: float = 0.002,
                   gravity: float = G, max_branch_jump: float = 0.2) -> ThrowPlan:
    """Build the joint trajectory whose TCP passes p_release with velocity v_release at separation.

    The TCP moves on a straight line along n = v/|v|. The palm normal equals n at release and pitches about
    pitch_axis(n) at pitch_rate [rad per metre of stroke], so the cube leaves with spin pitch_rate*|v|.
    q_seed selects the IK branch (the solution closest to it at the release pose is used, then tracked
    continuously along the stroke)."""
    p_release = np.asarray(p_release, float)
    v_release = np.asarray(v_release, float)
    speed = np.linalg.norm(v_release)
    n = v_release / speed
    prof = TossProfile(speed, n[2], params, gravity)
    R = toss_orientation(n, finger_axis)
    ax = pitch_axis(n)
    p_start = p_release - prof.s_sep * n

    def R_at(si):
        return axis_angle(ax, pitch_rate * (si - prof.s_sep)) @ R

    q_rel = robot.ik_closest(make_tf(R, p_release), q_seed)
    if q_rel is None:
        raise InfeasibleThrow("no IK at the release pose")
    t = np.arange(0.0, prof.duration + 1e-12, dt)
    if t[-1] < prof.duration - 1e-9:
        t = np.append(t, prof.duration)
    s, sd, sdd = prof.pos(t), prof.vel(t), prof.accel(t)
    N = len(t)
    q = np.zeros((N, 6))
    qd = np.zeros((N, 6))
    qdd = np.zeros((N, 6))
    i_rel = int(np.argmin(np.abs(t - prof.t_sep)))
    # track IK continuously outward from the release sample in both directions
    order = list(range(i_rel, N)) + list(range(i_rel - 1, -1, -1))
    prev = None
    for k, i in enumerate(order):
        if i == i_rel - 1:
            prev = q[i_rel]
        ref = q_rel if prev is None else prev
        qi = robot.ik_closest(make_tf(R_at(s[i]), p_start + s[i] * n), ref)
        if qi is None:
            raise InfeasibleThrow(f"IK failed at t={t[i]:.3f}")
        if prev is not None and np.max(np.abs(qi - prev)) > max_branch_jump:
            raise InfeasibleThrow(f"IK branch jump at t={t[i]:.3f} (|dq|={np.max(np.abs(qi - prev)):.3f})")
        q[i] = qi
        prev = qi
    for i in range(N):
        J = robot.jacobian(q[i])
        if abs(np.linalg.det(J)) < 1e-6:
            raise InfeasibleThrow(f"singular Jacobian at t={t[i]:.3f}")
        Jinv = np.linalg.inv(J)
        xd = np.r_[sd[i] * n, pitch_rate * sd[i] * ax]
        qd[i] = Jinv @ xd
        h = 1e-6
        Jdot_qd = (robot.jacobian(q[i] + h * qd[i]) - J) / h @ qd[i]
        qdd[i] = Jinv @ (np.r_[sdd[i] * n, pitch_rate * sdd[i] * ax] - Jdot_qd)
    tcp = p_start + np.outer(s, n)
    plan = ThrowPlan(t=t, q=q, qd=qd, qdd=qdd, tcp=tcp, t_release=prof.t_sep, p_release=p_release,
                     v_release=speed * n, profile=prof, q_start=q[0], q_end=q[-1])
    plan.spin = pitch_rate * speed * ax
    plan.metrics = limit_metrics(plan)
    return plan


def limit_metrics(plan: ThrowPlan) -> dict:
    pos_margin = np.min(POS_LIMIT - np.abs(plan.q), axis=0)
    vel_ratio = np.max(np.abs(plan.qd), axis=0) / VEL_LIMIT
    return {
        "pos_margin_min": float(pos_margin.min()),
        "pos_margin": pos_margin.tolist(),
        "vel_ratio_max": float(vel_ratio.max()),
        "vel_ratio": vel_ratio.tolist(),
        "acc_max": np.max(np.abs(plan.qdd), axis=0).tolist(),
        "stroke_length": plan.profile.length,
        "duration": plan.profile.duration,
        "tcp_z_min": float(plan.tcp[:, 2].min()),
    }


def check_limits(plan: ThrowPlan, vel_ratio_max=0.85, pos_margin_min=0.1, acc_limit=None) -> list[str]:
    """Return a list of violated limits (empty = OK)."""
    m = plan.metrics
    errs = []
    if m["vel_ratio_max"] > vel_ratio_max:
        errs.append(f"velocity {m['vel_ratio_max']:.2f} of limit > {vel_ratio_max}")
    if m["pos_margin_min"] < pos_margin_min:
        errs.append(f"position margin {m['pos_margin_min']:.3f} rad < {pos_margin_min}")
    if acc_limit is not None:
        over = np.array(m["acc_max"]) > np.asarray(acc_limit)
        if over.any():
            errs.append(f"acceleration {np.round(m['acc_max'], 1)} > {acc_limit}")
    return errs
