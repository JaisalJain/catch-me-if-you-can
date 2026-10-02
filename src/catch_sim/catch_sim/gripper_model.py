"""2F-85 command model: rate-limited, delayed position ramps for the gripper JointTrajectoryControllers.

A real 2F-85 moves from where it is towards the commanded position at its set speed (at most 150 mm/s of finger
gap) and starts ~10 ms after the command is sent (config/gripper.yaml). The sim's MuJoCo actuator only tracks its
reference, so every gripper command is sent as a time-stamped trajectory on the gripper JTC (positions only ->
linear interpolation): hold the current command until the latency has passed, step to the actual finger position,
then move linearly in gap at gap_speed to the target. The JTC executes it in sim time, independent of the
sender's scheduling (M5 lesson: never time actuation with Python sleeps).
"""
import os

import numpy as np
import yaml

# pad gap [mm] vs knuckle (driver) angle [rad], measured in the generated scene (tools/v2_catch_sim.py --gripper)
GAP_TABLE = np.array([[0.0, 85.2], [0.1, 75.7], [0.2, 65.5], [0.3, 54.8], [0.4, 43.7], [0.5, 32.2], [0.6, 20.5],
                      [0.7, 8.6], [0.782, 0.0]])


def gap_for_knuckle(k):
    return float(np.interp(k, GAP_TABLE[:, 0], GAP_TABLE[:, 1]))


def knuckle_for_gap(gap_mm):
    return float(np.interp(-float(gap_mm), -GAP_TABLE[:, 1], GAP_TABLE[:, 0]))


def load_profile(name=None, path=None):
    """{'name', 'gap_speed' [m/s], 'latency' [s]} of the active profile (CATCH_GRIPPER_PROFILE overrides)."""
    if path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory("catch_sim"), "config", "gripper.yaml")
        except Exception:  # noqa: BLE001 - source tree (tests)
            path = os.path.join(os.path.dirname(__file__), "..", "config", "gripper.yaml")
    cfg = yaml.safe_load(open(path))
    name = name or os.environ.get("CATCH_GRIPPER_PROFILE") or cfg["gripper_profile"]
    if name not in cfg["profiles"]:
        raise ValueError(f"unknown gripper profile {name}; one of {list(cfg['profiles'])}")
    prof = dict(name=name, **cfg["profiles"][name])
    if os.environ.get("CATCH_GRIPPER_LATENCY"):          # benchmark latency sweep: the device's TRUE latency [s]
        prof["latency"] = float(os.environ["CATCH_GRIPPER_LATENCY"])
    return prof


def ramp_points(q_hold, q_from, q_to, gap_speed, latency, step=0.01):
    """[(t_from_command, knuckle)] for a JTC trajectory stamped at the command time (times strictly increasing)."""
    return _strict(_ramp_points(q_hold, q_from, q_to, gap_speed, latency, step))


def _strict(pts, eps=1e-4):
    out = []
    for t, q in pts:
        if out and t < out[-1][0] + eps:
            out[-1] = (out[-1][0], q) if t <= out[-1][0] else (out[-1][0] + eps, q)
            continue
        out.append((t, q))
    return out


def _ramp_points(q_hold, q_from, q_to, gap_speed, latency, step=0.01):
    pts = [(0.0, float(q_hold)), (latency, float(q_hold))]
    t = latency + 0.002
    pts.append((t, float(q_from)))
    g0, g1 = gap_for_knuckle(q_from), gap_for_knuckle(q_to)
    # beyond the table ends (q_to > 0.782 or < 0) the gap does not change; move the knuckle at the end-segment rate
    dur = abs(g1 - g0) / 1000.0 / gap_speed
    if dur < 1e-6:
        if abs(q_to - q_from) > 1e-6:
            pts.append((t + 0.002, float(q_to)))
        return pts
    n = max(1, int(np.ceil(dur / step)))
    for i in range(1, n + 1):
        g = g0 + (g1 - g0) * i / n
        q = knuckle_for_gap(g) if i < n else q_to
        pts.append((t + dur * i / n, float(q)))
    return pts


def time_to_gap(q_from, gap_target_mm, gap_speed, latency):
    """Time from command to the fingers reaching gap_target (e.g. the 40 mm cube width)."""
    return latency + 0.002 + abs(gap_for_knuckle(q_from) - gap_target_mm) / 1000.0 / gap_speed
