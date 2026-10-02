"""v2 throw library (system python side): load, verify, plan, perturb.

Entries come from tools/build_throw_library.py (venv + MuJoCo): generation parameters, feed-forward torques and a
hash of everything the trajectory depends on. Here the trajectory is regenerated from the parameters (numpy,
catch_throw.generator, ~30 ms) and the hash is re-computed from the installed config files; a mismatch means the
geometry / physics changed after the library was built (re-run the builder).
"""
from __future__ import annotations

import hashlib
import json
import os

import numpy as np
import yaml

from catch_kinematics.transforms import axis_angle

from .generator import generate_throw
from .profile import ProfileParams


def _share(pkg):
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(pkg)


def entry_hash(params, throw_yaml, scene_yaml, physics_yaml):
    """Same function as tools/build_throw_library.py:entry_hash."""
    cfg = yaml.safe_load(open(throw_yaml))["throw"]
    cfg.pop("release", None)
    h = hashlib.sha256(json.dumps(params, sort_keys=True).encode())
    h.update(yaml.safe_dump(cfg, sort_keys=True).encode())
    for f in (scene_yaml, physics_yaml):
        h.update(open(f, "rb").read())
    return h.hexdigest()[:16]


def config_paths():
    t, d = _share("catch_throw"), _share("catch_description")
    return (os.path.join(t, "config", "throw.yaml"), os.path.join(d, "config", "scene.yaml"),
            os.path.join(d, "config", "physics.yaml"))


def load_v2():
    return yaml.safe_load(open(os.path.join(_share("catch_throw"), "config", "v2.yaml")))


class Library:
    def __init__(self, name, path=None, paths=None):
        path = path or os.path.join(_share("catch_throw"), "config", f"library_{name}.json")
        d = json.load(open(path))
        self.name = d["library"]
        self.entries = d["entries"]
        self.paths = paths or config_paths()

    def __len__(self):
        return len(self.entries)

    def check(self, e):
        h = entry_hash(e["params"], *self.paths)
        return h == e["hash"], h

    def plan(self, robot, e, v_release=None):
        """ThrowPlan of entry e (optionally with a perturbed release velocity)."""
        p = e["params"]
        v = np.array(p["v_release"]) if v_release is None else np.asarray(v_release, float)
        return generate_throw(robot, np.array(p["p_release"]), v, p["q_seed"], ProfileParams(**p["profile"]),
                              finger_axis=p["finger_axis"], pitch_rate=p["pitch_rate"], dt=p["sample_dt"])

    def feedforward(self, e, n):
        """Feed-forward table resampled to n samples (a perturbed plan can be a sample longer or shorter)."""
        tau = np.array(e["tau"])
        if len(tau) == n:
            return tau
        s0, s1 = np.linspace(0, 1, len(tau)), np.linspace(0, 1, n)
        return np.column_stack([np.interp(s1, s0, tau[:, k]) for k in range(tau.shape[1])])


def sample_noise(level_cfg, v, rng):
    """Perturbed release velocity and release time offset for one throw (Gaussian, level_cfg from v2.yaml)."""
    v = np.asarray(v, float)
    speed = np.linalg.norm(v) * (1.0 + rng.normal(0.0, level_cfg["speed_rel"]))
    n = v / np.linalg.norm(v)
    a1 = np.cross(n, [0.0, 0.0, 1.0])
    a1 /= np.linalg.norm(a1)
    a2 = np.cross(n, a1)
    d1, d2 = np.radians(rng.normal(0.0, level_cfg["dir_deg"], 2))
    n2 = axis_angle(a2, d2) @ axis_angle(a1, d1) @ n
    return speed * n2, float(rng.normal(0.0, level_cfg["time_s"]))
