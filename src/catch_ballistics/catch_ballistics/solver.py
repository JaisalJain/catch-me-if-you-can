"""Release-velocity and intercept solvers."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .projectile import G, Projectile


def velocity_for_flight_time(p_release, p_target, flight_time: float, gravity: float = G) -> np.ndarray:
    """Release velocity that reaches p_target exactly flight_time after release."""
    if flight_time <= 0:
        raise ValueError("flight_time must be > 0")
    p, c = np.asarray(p_release, float), np.asarray(p_target, float)
    g = np.array([0.0, 0.0, -gravity])
    return (c - p - 0.5 * g * flight_time ** 2) / flight_time


def velocity_for_launch_angle(p_release, p_target, angle: float, gravity: float = G) -> tuple[np.ndarray, float]:
    """Release velocity with elevation `angle` [rad] (in the vertical plane through both points)
    that hits p_target. Returns (v, flight_time). Raises ValueError if unreachable at that angle."""
    p, c = np.asarray(p_release, float), np.asarray(p_target, float)
    d = c - p
    dh = np.hypot(d[0], d[1])
    if dh < 1e-9:
        raise ValueError("target directly above/below release: angle solve undefined")
    u = np.array([d[0] / dh, d[1] / dh, 0.0])
    denom = 2.0 * np.cos(angle) ** 2 * (dh * np.tan(angle) - d[2])
    if denom <= 0:
        raise ValueError("target unreachable at this launch angle")
    speed = np.sqrt(gravity * dh ** 2 / denom)
    v = speed * (np.cos(angle) * u + np.array([0.0, 0.0, np.sin(angle)]))
    return v, dh / (speed * np.cos(angle))


def min_speed_angle(p_release, p_target, gravity: float = G) -> float:
    """Launch elevation that reaches p_target with the minimum release speed."""
    d = np.asarray(p_target, float) - np.asarray(p_release, float)
    dh = np.hypot(d[0], d[1])
    return 0.5 * (np.arctan2(d[2], dh) + np.pi / 2)


@dataclass
class InterceptCandidate:
    t: float
    position: np.ndarray
    velocity: np.ndarray


def intercepts_in_box(traj: Projectile, box_min, box_max, t_end: float, dt: float = 1e-3,
                      descending_only: bool = True) -> list[InterceptCandidate]:
    """All sampled trajectory points inside an axis-aligned box (optionally only where vz < 0)."""
    lo, hi = np.asarray(box_min, float), np.asarray(box_max, float)
    t, pos = traj.sample(t_end, dt)
    vel = traj.velocity(t)
    inside = np.all((pos >= lo) & (pos <= hi), axis=1)
    if descending_only:
        inside &= vel[:, 2] < 0
    return [InterceptCandidate(float(ti), pi, vi) for ti, pi, vi in zip(t[inside], pos[inside], vel[inside])]


def choose_intercept(candidates: list[InterceptCandidate], score) -> InterceptCandidate | None:
    """Pick the candidate with the highest score(candidate)."""
    if not candidates:
        return None
    return max(candidates, key=score)
