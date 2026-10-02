#!/usr/bin/env python3
"""v2 throw library: precomputed, offline-verified throws for random targets (venv + MuJoCo).

    env -u PYTHONPATH .venv/bin/python tools/build_throw_library.py

Sources (v2 Phase 1, tools/v2_throws.py): docs/v2/throw_distribution.json (palm pitch for minimum joint speed) and
docs/v2/throw_distribution_square.json (palm pitch so the cube lands square on B's cup palm).
  * library "pinch": every A-feasible throw that B can catch from the ready pose with the natural orientation
    (slack_ready >= 0) — the full feasible set for the pinch;
  * library "cup":   every A-feasible square-arrival throw that B can catch with the cup (30 deg cone).
Each entry is regenerated here (catch_throw.generator, sample_dt from throw.yaml), re-checked against the limits
(speed <= 85 %, MuJoCo inverse-dynamics torque <= 80 %), and stores its own feed-forward table (tau = M qdd + c,
gravity excluded: the sim compensates gravity, as throw_ff.yaml) plus a hash of everything it depends on
(its own parameters, throw.yaml without `release`, scene.yaml, physics.yaml). throw_node regenerates the trajectory
from the parameters and refuses an entry whose hash does not match. Output: src/catch_throw/config/library_<name>.json
"""
import hashlib
import json
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v2_common as V  # noqa: E402
from catch_throw.generator import generate_throw  # noqa: E402
from catch_throw.profile import ProfileParams  # noqa: E402

REPO = V.REPO
OUT = os.path.join(REPO, "src/catch_throw/config")


def entry_hash(params, root=REPO):
    """Same function as catch_throw.library.entry_hash (system python side)."""
    cfg = yaml.safe_load(open(os.path.join(root, "src/catch_throw/config/throw.yaml")))["throw"]
    cfg.pop("release", None)
    h = hashlib.sha256(json.dumps(params, sort_keys=True).encode())
    h.update(yaml.safe_dump(cfg, sort_keys=True).encode())
    for f in ("src/catch_description/config/scene.yaml", "src/catch_description/config/physics.yaml"):
        h.update(open(os.path.join(root, f), "rb").read())
    return h.hexdigest()[:16]


def profile_for(r, base):
    if abs(r["accel"] - base.accel) < 1e-9 and abs(r["decel"] - base.decel) < 1e-9:
        return base
    return ProfileParams(accel=10, decel=16, tau3=0.03)      # tools/v2_throws.py: the softer alternative


def main():
    tc = yaml.safe_load(open(os.path.join(REPO, "src/catch_throw/config/throw.yaml")))["throw"]
    base = ProfileParams(**tc["profile"])
    dt = tc["sample_dt"]
    scene, phys = V.build_mjcf.load_cfg()
    robot = V.F.make_robots(scene)["a"]
    dyn_lim = V.F.Dynamics(scene, phys)
    dyn_ff = V.F.Dynamics(scene, phys)
    dyn_ff.m.opt.gravity[:] = 0.0
    lim = V.cfg()["limits"]
    sources = {"pinch": ("throw_distribution.json", "slack_ready"), "cup": ("throw_distribution_square.json", "slack_cup30")}
    for name, (fn, slack) in sources.items():
        src = json.load(open(os.path.join(V.OUT, fn)))
        rows = [r for r in src["a_feasible"] if r.get(slack) is not None and r[slack] >= 0]
        entries, dropped = [], 0
        for i, r in enumerate(rows):
            prof = profile_for(r, base)
            params = dict(p_release=[round(x, 6) for x in r["p_release"]], v_release=[round(x, 6) for x in r["v_release"]],
                          q_seed=[round(x, 6) for x in r["q_release_seed"]], pitch_rate=round(r["kappa"], 6),
                          profile=dict(accel=prof.accel, tau1=prof.tau1, tau3=prof.tau3, decel=prof.decel, tau5=prof.tau5),
                          finger_axis=[0.0, 1.0, 0.0], sample_dt=dt)
            plan = generate_throw(robot, np.array(params["p_release"]), np.array(params["v_release"]), params["q_seed"],
                                  ProfileParams(**params["profile"]), finger_axis=params["finger_axis"],
                                  pitch_rate=params["pitch_rate"], dt=dt)
            m = plan.metrics
            tau_lim = dyn_lim.torques(plan.q, plan.qd, plan.qdd)
            tr = float(np.max(np.abs(tau_lim) / V.TORQUE_LIMIT))
            if m["vel_ratio_max"] > lim["vel_ratio"] or tr > lim["torque_ratio"] or m["pos_margin_min"] < lim["pos_margin"]:
                dropped += 1
                continue
            tau = dyn_ff.torques(plan.q, plan.qd, plan.qdd)
            entries.append(dict(
                id=len(entries), params=params, hash=entry_hash(params),
                target=r["target"], launch_angle_deg=r["deg"], t_target=r["t_target"], spin=r["spin"],
                n=len(plan.t), t_release=float(plan.t_release), duration=float(plan.t[-1]),
                q_start=[round(x, 6) for x in plan.q_start.tolist()], vel_ratio=round(m["vel_ratio_max"], 4),
                torque_ratio=round(tr, 4), arrival_tilt_deg=r.get("arrival_tilt_deg"),
                tau=np.round(tau, 4).tolist()))
        out = dict(library=name, source=f"docs/v2/{fn} ({slack} >= 0)", n=len(entries),
                   note="generated by tools/build_throw_library.py; tau = feed-forward (gravity excluded)",
                   entries=entries)
        path = os.path.join(OUT, f"library_{name}.json")
        json.dump(out, open(path, "w"), separators=(",", ":"))
        tt = np.array([e["t_target"] for e in entries])
        print(f"library {name}: {len(entries)} entries ({dropped} dropped on re-check), flight to target "
              f"{tt.min():.3f}-{tt.max():.3f} s, {os.path.getsize(path) / 1e6:.2f} MB -> {path}")


if __name__ == "__main__":
    main()
