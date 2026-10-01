"""Intercept + cup pose for robot B (numpy only)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from catch_ballistics import Projectile
from catch_kinematics import POS_LIMIT, UR5e, wrap
from catch_kinematics.transforms import make_tf
from catch_throw.generator import toss_orientation


@dataclass
class Intercept:
    t: float              # flight time from release to intercept [s]
    p: np.ndarray         # cube centre = TCP at the intercept
    v: np.ndarray         # cube velocity at the intercept
    R: np.ndarray         # cup orientation: tool z = -v/|v| (palm faces the cube), fingers along finger_axis
    q: np.ndarray         # B joint configuration


def elbow_up(robot: UR5e, q) -> bool:
    fr = robot.frames(q)
    sh, el, wr = fr[1][:3, 3], fr[2][:3, 3], fr[4][:3, 3]
    s = np.clip(np.dot(el - sh, wr - sh) / np.dot(wr - sh, wr - sh), 0, 1)
    return bool(el[2] > (sh + s * (wr - sh))[2])


def cup_ik(robot: UR5e, p, v, q_ref, finger_axis=(0, 1, 0), min_margin=0.2):
    """Elbow-up IK solution of the cup pose closest to q_ref, or None."""
    R = toss_orientation(-np.asarray(v) / np.linalg.norm(v), finger_axis)
    best, best_d = None, np.inf
    for s in robot.ik_all(make_tf(R, p)):
        s = q_ref + wrap(s - q_ref)
        if np.any(np.abs(s) > POS_LIMIT - min_margin) or not elbow_up(robot, s):
            continue
        dist = float(np.sum(np.abs(s - q_ref)))
        if dist < best_d:
            best, best_d = s, dist
    return R, best


def choose_intercept(robot: UR5e, p_release, v_release, q_home, mode="plan_catch_point", flight_time=None,
                     z_window=(1.0, 1.35), min_margin=0.2, finger_axis=(0, 1, 0), dt=0.005) -> Intercept | None:
    traj = Projectile(p_release, v_release)
    if mode == "plan_catch_point" and flight_time:
        ts = [flight_time]
    else:
        t_end = traj.time_descending_at_height(z_window[0]) or 1.0
        ts = [t for t in np.arange(traj.apex_time, t_end, dt) if z_window[0] <= traj.position(t)[2] <= z_window[1]]
    best = None
    for t in ts:
        p, v = traj.position(t), traj.velocity(t)
        if v[2] >= 0:
            continue
        R, q = cup_ik(robot, p, v, np.asarray(q_home, float), finger_axis, min_margin)
        if q is None:
            continue
        cost = float(np.sum(np.abs(q - q_home)))
        if best is None or cost < best[0]:
            best = (cost, Intercept(float(t), p, v, R, q))
    return None if best is None else best[1]
