"""Capsule collision model of the UR5e + 2F-85 (config/ur5e_capsules.yaml from tools/export_collision.py, fitted to
the MuJoCo collision geometry and validated against MuJoCo's collision detection). numpy only, vectorised over the
samples of a trajectory, for the reactive catcher's per-replan checks.
"""
from __future__ import annotations

import os

import numpy as np
import yaml


def _seg_seg(p1, q1, p2, q2):
    """Distances between segments p1q1 and p2q2 (arrays (..., 3)), Ericson's closest-point algorithm."""
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a = np.sum(d1 * d1, -1)
    e = np.sum(d2 * d2, -1)
    f = np.sum(d2 * r, -1)
    c = np.sum(d1 * r, -1)
    b = np.sum(d1 * d2, -1)
    den = a * e - b * b
    s = np.where(den > 1e-12, np.clip((b * f - c * e) / np.where(den > 1e-12, den, 1), 0, 1), 0.0)
    t = (b * s + f) / np.where(e > 1e-12, e, 1)
    t_c = np.clip(t, 0, 1)
    s = np.where((t < 0) | (t > 1), np.clip((b * t_c - c) / np.where(a > 1e-12, a, 1), 0, 1), s)
    t = t_c
    c1 = p1 + d1 * s[..., None]
    c2 = p2 + d2 * t[..., None]
    return np.linalg.norm(c1 - c2, axis=-1)


def _seg_box(p, q, lo, hi, n=7):
    """Approximate distance segment-box: min over n points on the segment of the point-box distance."""
    s = np.linspace(0, 1, n)
    pts = p[..., None, :] + (q - p)[..., None, :] * s[:, None]          # (..., n, 3)
    lo, hi = np.asarray(lo)[..., None, :], np.asarray(hi)[..., None, :]   # (n_obst, 1, 3) broadcast over samples
    d = np.maximum(np.maximum(lo - pts, pts - hi), 0.0)
    return np.min(np.linalg.norm(d, axis=-1), axis=-1)


class CapsuleModel:
    def __init__(self, path=None):
        if path is None:
            path = os.path.join(os.path.dirname(__file__), "..", "config", "ur5e_capsules.yaml")
            if not os.path.exists(path):
                from ament_index_python.packages import get_package_share_directory
                path = os.path.join(get_package_share_directory("catch_kinematics"), "config", "ur5e_capsules.yaml")
        c = yaml.safe_load(open(path))
        self.caps = c["capsules"]
        self.names = [k["name"] for k in self.caps]
        self.frame = np.array([k["frame"] for k in self.caps])
        self.p0 = np.array([k["p0"] + [1.0] for k in self.caps])
        self.p1 = np.array([k["p1"] + [1.0] for k in self.caps])
        self.r = np.array([k["radius"] for k in self.caps])
        self.obst = c["obstacles"]
        self.lo = np.array([o["lo"] for o in self.obst])
        self.hi = np.array([o["hi"] for o in self.obst])
        idx = {n: i for i, n in enumerate(self.names)}
        self.pairs = [tuple(p) for p in c["self_pairs"]]
        self.pi = np.array([idx[a] for a, _ in self.pairs], int)
        self.pj = np.array([idx[b] for _, b in self.pairs], int)
        self.min_clearance = float(c.get("min_clearance", 0.01))
        # capsules of the base / shoulder never move w.r.t. the pedestal they stand on: skip env checks of frame 0/1
        self.env_caps = np.where(self.frame >= 2)[0]

    def segments(self, robot, Q):
        """World segment endpoints (N, n_caps, 3) x2 for configurations Q (N, 6)."""
        Q = np.atleast_2d(Q)
        A = np.zeros((len(Q), len(self.caps), 3))
        B = np.zeros_like(A)
        for i, q in enumerate(Q):
            F = robot.frames(q)
            for k, f in enumerate(self.frame):
                T = F[f]
                A[i, k] = (T @ self.p0[k])[:3]
                B[i, k] = (T @ self.p1[k])[:3]
        return A, B

    def self_distances(self, robot, q):
        A, B = self.segments(robot, q)
        return _seg_seg(A[0, self.pi], B[0, self.pi], A[0, self.pj], B[0, self.pj]) - self.r[self.pi] - self.r[self.pj]

    def clearance(self, robot, Q, env=True, self_check=True, extra_capsules=None):
        """Minimum signed clearance [m] over the configurations Q (negative = penetration)."""
        A, B = self.segments(robot, Q)
        best = np.inf
        if self_check and len(self.pi):
            d = _seg_seg(A[:, self.pi], B[:, self.pi], A[:, self.pj], B[:, self.pj]) - self.r[self.pi] - self.r[self.pj]
            best = min(best, float(d.min()))
        if env and len(self.lo):
            e = self.env_caps
            Pa, Pb = A[:, e][:, :, None, :], B[:, e][:, :, None, :]
            d = _seg_box(np.broadcast_to(Pa, Pa.shape[:2] + (len(self.lo), 3)),
                         np.broadcast_to(Pb, Pb.shape[:2] + (len(self.lo), 3)), self.lo, self.hi) - self.r[e][None, :, None]
            best = min(best, float(d.min()))
        if extra_capsules is not None:       # e.g. the other arm: (P0 (m,3), P1 (m,3), radius (m,))
            P0, P1, R = extra_capsules
            d = _seg_seg(A[:, :, None, :], B[:, :, None, :], P0[None, None], P1[None, None]) - self.r[None, :, None] - R[None, None]
            best = min(best, float(d.min()))
        return best

    def free(self, robot, Q, env=True, self_check=True, extra_capsules=None, margin=None):
        m = self.min_clearance if margin is None else margin
        return self.clearance(robot, Q, env, self_check, extra_capsules) >= m
