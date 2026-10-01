"""Scalar jerk-limited acceleration profile for the straight-line underhand toss (numpy only).

The TCP (cube centre) moves along the release direction n. The scalar acceleration a(t) along n is

  seg 1  [0, tau1)        cosine ramp 0 -> A
  seg 2  [.., +T2)        constant A                 (T2 solved so that v(t_sep) = V)
  seg 3  [.., +tau3)      cosine ramp A -> -Dd       (the cube separates inside this ramp)
  seg 4  [.., +T4)        constant -Dd               (T4 solved so the arm stops)
  seg 5  [.., +tau5)      cosine ramp -Dd -> 0

Separation physics (palm normal = n, palm moving along n): the palm pushes the cube with
N = m (a + g n_z). The cube leaves the palm when a(t) < -g n_z, i.e. at t_sep inside seg 3.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

G = 9.81


@dataclass
class ProfileParams:
    accel: float = 12.0        # A   [m/s^2]
    tau1: float = 0.08         # [s]
    tau3: float = 0.04         # [s]
    decel: float = 25.0        # Dd  [m/s^2]
    tau5: float = 0.08         # [s]


class TossProfile:
    def __init__(self, speed: float, n_z: float, p: ProfileParams, gravity: float = G):
        self.V, self.p, self.g = float(speed), p, gravity
        A, Dd = p.accel, p.decel
        self.a_sep = -gravity * n_z
        if not (-Dd < self.a_sep < A):
            raise ValueError(f"separation accel {self.a_sep:.2f} must lie in (-decel, accel)")
        # seg 3 time where a = a_sep:  A - (A+Dd)(1-cos(pi u/tau3))/2 = a_sep
        c = 1.0 - 2.0 * (A - self.a_sep) / (A + Dd)
        self.u_sep = p.tau3 / np.pi * np.arccos(np.clip(c, -1, 1))
        v1 = A * p.tau1 / 2
        dv3_sep = self._seg3_dv(self.u_sep)
        self.T2 = (self.V - v1 - dv3_sep) / A
        if self.T2 < 0:
            raise ValueError("release speed too low for these ramps (T2 < 0); shorten tau1/tau3")
        v3 = v1 + A * self.T2 + self._seg3_dv(p.tau3)
        self.T4 = v3 / Dd - p.tau5 / 2
        if self.T4 < 0:
            raise ValueError("decel too high for tau5 (T4 < 0)")
        self.knots = np.cumsum([0.0, p.tau1, self.T2, p.tau3, self.T4, p.tau5])
        self.t_sep = self.knots[2] + self.u_sep
        self.duration = self.knots[-1]
        # integrate on a fine grid for s(t) (closed forms exist but this is simpler and exact enough)
        self._tg = np.linspace(0, self.duration, 20001)
        ag = self._accel(self._tg)
        dt = self._tg[1] - self._tg[0]
        vg = np.concatenate([[0], np.cumsum((ag[1:] + ag[:-1]) / 2) * dt])
        sg = np.concatenate([[0], np.cumsum((vg[1:] + vg[:-1]) / 2) * dt])
        self._vg, self._sg = vg, sg

    def _seg3_dv(self, u):
        A, Dd, t3 = self.p.accel, self.p.decel, self.p.tau3
        return A * u - (A + Dd) / 2 * (u - t3 / np.pi * np.sin(np.pi * u / t3))

    def _accel(self, t):
        t = np.atleast_1d(np.asarray(t, float))
        A, Dd, p = self.p.accel, self.p.decel, self.p
        k = self.knots
        a = np.zeros_like(t)
        m = t < k[1]
        a[m] = A * (1 - np.cos(np.pi * t[m] / p.tau1)) / 2
        m = (t >= k[1]) & (t < k[2])
        a[m] = A
        m = (t >= k[2]) & (t < k[3])
        u = t[m] - k[2]
        a[m] = A - (A + Dd) * (1 - np.cos(np.pi * u / p.tau3)) / 2
        m = (t >= k[3]) & (t < k[4])
        a[m] = -Dd
        m = (t >= k[4]) & (t < k[5])
        u = t[m] - k[4]
        a[m] = -Dd * (1 + np.cos(np.pi * u / p.tau5)) / 2
        return a

    def accel(self, t):
        return self._accel(t)

    def vel(self, t):
        return np.interp(t, self._tg, self._vg)

    def pos(self, t):
        return np.interp(t, self._tg, self._sg)

    @property
    def s_sep(self):
        return float(self.pos(self.t_sep))

    @property
    def length(self):
        return float(self._sg[-1])
