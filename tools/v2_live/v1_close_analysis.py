#!/usr/bin/env python3
"""v1 cup catch: how fast did B's fingers actually close, relative to the palm impact, in the original M5 run (v1
sim gripper) vs the v2 step-0 re-run with the realistic 2F-85 model? (system python, numpy only)

    python3 tools/v2_live/v1_close_analysis.py log/m5/force log/p2/v1_realistic     (workspace sourced)
Per trial (catch_eval recordings: palm wrench, joint states incl. B's knuckle, cube ground truth):
  t_hit   first palm force > 3 N (the contact the trigger reacts to)
  gap     finger gap from the knuckle angle (catch_sim.gripper_model.GAP_TABLE, inlined)
  t_move  fingers start closing (gap 1 mm below its value at t_hit); t_grip: gap <= 41 mm (pads on the 40 mm cube)
  speed   mean gap speed 80 -> 45 mm
  cube    in B's tool frame: max distance from where it is 20 ms after the hit, until t_grip (does it stay in the
          palm while the fingers close?), and its speed relative to the palm at t_grip
"""
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from close_timing import JN, robot_b  # noqa: E402
from catch_kinematics.transforms import inv_tf  # noqa: E402

RB = robot_b()
COLORS = ("red", "green", "blue")

GAP = np.array([[0.0, 85.2], [0.1, 75.7], [0.2, 65.5], [0.3, 54.8], [0.4, 43.7], [0.5, 32.2], [0.6, 20.5],
                [0.7, 8.6], [0.782, 0.0]])


def gap(k):
    return np.interp(k, GAP[:, 0], GAP[:, 1])


def one(f):
    z = np.load(f, allow_pickle=True)
    meta = json.loads(str(z["meta"]))
    names = list(z["names"])
    kk = names.index("b_robotiq_85_left_knuckle_joint")
    t, q = z["t_js"], z["q"]
    g = gap(q[:, kk])
    ft = z["ft"]                       # t, fx, fy, fz
    fmag = np.linalg.norm(ft[:, 1:4], axis=1)
    ih = np.argmax(fmag > 3.0)
    if fmag[ih] <= 3.0:
        return None
    t_hit = ft[ih, 0]
    j0 = np.searchsorted(t, t_hit)
    g0 = g[j0]
    after = np.where((t > t_hit) & (g < g0 - 1.0))[0]
    t_move = t[after[0]] if len(after) else np.nan
    grip = np.where((t > t_hit) & (g <= 41.0))[0]
    t_grip = t[grip[0]] if len(grip) else np.nan
    a, b = np.where((t > t_hit) & (g <= 80.0))[0], np.where((t > t_hit) & (g <= 45.0))[0]
    speed = (35.0 / (t[b[0]] - t[a[0]])) if len(a) and len(b) and t[b[0]] > t[a[0]] else np.nan
    # cube in B's tool frame between the hit and the grip
    jb = [names.index(f"b_{j}") for j in JN]
    k = COLORS.index(meta["color"])
    tc, pc = z["cubes"][:, 0], z["cubes"][:, 1 + 7 * k:4 + 7 * k]
    sel = np.where((tc >= t_hit + 0.02) & (tc <= (t_grip if np.isfinite(t_grip) else t_hit + 0.5)))[0]
    rel = np.array([(inv_tf(RB.fk(q[min(np.searchsorted(t, tt), len(t) - 1), jb])) @ np.r_[pc[i], 1])[:3]
                    for i, tt in zip(sel, tc[sel])])
    drift = 1000 * float(np.max(np.linalg.norm(rel - rel[0], axis=1))) if len(rel) > 1 else np.nan
    vrel = 1000 * float(np.linalg.norm(rel[-1] - rel[-3]) / (tc[sel][-1] - tc[sel][-3])) if len(rel) > 3 else np.nan
    return dict(i=meta["i"], held=meta.get("held"), cube_drift_mm=round(drift, 1), cube_speed_at_grip_mm_s=round(vrel, 0), gap_at_hit=round(float(g0), 1), move_ms=round(1000 * (t_move - t_hit), 1),
                grip_ms=round(1000 * (t_grip - t_hit), 1), speed_mm_s=round(float(speed), 0),
                trigger_rel_ms=meta.get("trigger_time_rel_ms"))


def main():
    for d in sys.argv[1:]:
        rows = [r for f in sorted(glob.glob(os.path.join(d, "trial_*.npz"))) if (r := one(f))]
        print(f"{d}: {len(rows)} trials, held {sum(bool(r['held']) for r in rows)}")
        for k in ("gap_at_hit", "move_ms", "grip_ms", "speed_mm_s", "trigger_rel_ms", "cube_drift_mm",
                  "cube_speed_at_grip_mm_s"):
            v = np.array([r[k] for r in rows if r[k] is not None], dtype=float)
            print(f"   {k:15s} median {np.nanmedian(v):8.1f}   range {np.nanmin(v):7.1f} .. {np.nanmax(v):7.1f}")


if __name__ == "__main__":
    main()
