"""Point-mass projectile in a uniform gravity field (no drag; MuJoCo's default medium is vacuum)."""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

G = 9.81


def _vec(x) -> np.ndarray:
    a = np.asarray(x, dtype=float).reshape(3)
    return a


@dataclass
class Projectile:
    """State p(t) = p0 + v0 (t - t0) + 0.5 g (t - t0)^2, with g = (0, 0, -gravity)."""

    p0: np.ndarray
    v0: np.ndarray
    t0: float = 0.0
    gravity: float = G
    g: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        self.p0 = _vec(self.p0)
        self.v0 = _vec(self.v0)
        self.g = np.array([0.0, 0.0, -self.gravity])

    def position(self, t):
        """Position at absolute time(s) t. Scalar t -> (3,), array t -> (N, 3)."""
        tau = np.asarray(t, dtype=float) - self.t0
        return self.p0 + np.multiply.outer(tau, self.v0) + 0.5 * np.multiply.outer(tau ** 2, self.g)

    def velocity(self, t):
        tau = np.asarray(t, dtype=float) - self.t0
        return self.v0 + np.multiply.outer(tau, self.g)

    def sample(self, t_end: float, dt: float = 1e-3):
        """Return (t, positions) from t0 to t_end inclusive."""
        n = int(np.floor((t_end - self.t0) / dt + 1e-9)) + 1
        t = self.t0 + dt * np.arange(n)
        return t, self.position(t)

    @property
    def apex_time(self) -> float:
        return self.t0 + max(self.v0[2], 0.0) / self.gravity

    @property
    def apex(self) -> np.ndarray:
        return self.position(self.apex_time)

    def times_at_height(self, z: float) -> list[float]:
        """Absolute times (ascending) at which the projectile is at height z (0, 1 or 2 roots, t >= t0)."""
        # 0.5 g tau^2 - vz tau + (z - z0) = 0  with g > 0
        a, b, c = 0.5 * self.gravity, -self.v0[2], z - self.p0[2]
        disc = b * b - 4 * a * c
        if disc < 0:
            return []
        r = np.sqrt(disc)
        roots = sorted({(-b - r) / (2 * a), (-b + r) / (2 * a)})
        return [self.t0 + tau for tau in roots if tau >= -1e-12]

    def time_descending_at_height(self, z: float) -> float | None:
        """Time at which the projectile passes height z on the way down (None if never)."""
        ts = [t for t in self.times_at_height(z) if self.velocity(t)[2] <= 0.0]
        return ts[0] if ts else None

    def time_at_plane(self, point, normal) -> float | None:
        """First time t >= t0 at which the projectile crosses the plane (point, normal)."""
        n = _vec(normal)
        a = 0.5 * float(self.g @ n)
        b = float(self.v0 @ n)
        c = float((self.p0 - _vec(point)) @ n)
        if abs(a) < 1e-12:
            if abs(b) < 1e-12:
                return None
            tau = -c / b
            return self.t0 + tau if tau >= 0 else None
        disc = b * b - 4 * a * c
        if disc < 0:
            return None
        r = np.sqrt(disc)
        taus = sorted(t for t in ((-b - r) / (2 * a), (-b + r) / (2 * a)) if t >= -1e-12)
        return self.t0 + taus[0] if taus else None
