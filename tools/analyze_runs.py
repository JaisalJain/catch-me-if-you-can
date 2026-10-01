#!/usr/bin/env python3
"""M6: check each full run against ground truth (evaluation only — the state machine never reads it).

    env -u PYTHONPATH .venv/bin/python tools/analyze_runs.py docs/m6/run_seed1.json:log/m6/run_seed1.npz ...

Per run: every VERIFY verdict vs the cube's ground-truth position at that instant (in its slot on B's pallet:
|dx|, |dy| < 6 mm and resting on the pallet), the final state of all cubes, throws / recoveries / duration.
"""
import json
import os
import sys

import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
COLORS = ("red", "green", "blue")


def slot(scene, pallet, color):
    p = scene["pallets"]
    i = p["slot_order"].index(color)
    cx, cy = p[pallet]["center"]
    return np.array([cx + (i - 1) * p["slot_pitch"], cy, scene["table"]["top_z"] + p["base_size"][2] + scene["cube"]["edge"] / 2])


def cube_at(rec, color, t):
    c = rec["cubes"]
    k = min(np.searchsorted(c[:, 0], t), len(c) - 1)
    j = COLORS.index(color)
    return c[k, 1 + 7 * j:4 + 7 * j]


def in_slot(scene, p, color, pallet="b"):
    e = p - slot(scene, pallet, color)
    tol = (scene["pallets"]["slot_inner"] - scene["cube"]["edge"]) / 2
    return bool(abs(e[0]) < tol and abs(e[1]) < tol and abs(e[2]) < 0.003), e


def main():
    scene = yaml.safe_load(open(os.path.join(REPO, "src/catch_description/config/scene.yaml")))
    out = []
    for arg in sys.argv[1:]:
        jf, nf = arg.split(":")
        run = json.load(open(jf))
        rec = dict(np.load(nf, allow_pickle=True))
        verdicts = []
        for e in run["events"]:
            if e["state"] != "verify":
                continue
            # the colour verified is the last colour B saw
            col = [x["b_color"] for x in run["events"] if "b_color" in x and x["t_sim"] <= e["t_sim"]][-1]
            ok_gt, err = in_slot(scene, cube_at(rec, col, e["t_sim"]), col)
            said = "sorted" in e
            verdicts.append(dict(t=e["t"], color=col, verify_said=said, gt_in_slot=ok_gt, gt_err_mm=(1000 * err).round(2).tolist()))
        t_end = float(rec["cubes"][-1, 0])
        final = {c: dict(zip(("in_b_slot", "err_mm"), (lambda r: (r[0], (1000 * r[1]).round(2).tolist()))(
            in_slot(scene, cube_at(rec, c, t_end), c)))) for c in COLORS}
        s = run["summary"]
        row = dict(seed=s["seed"], duration_s=s["duration_s"], throws=s["throws"], recoveries=s["recoveries"],
                   complete=s["complete"], injected=[e["msg"] for e in run["events"] if e.get("injected")],
                   verify_agrees_with_gt=f"{sum(v['verify_said'] == v['gt_in_slot'] for v in verdicts)}/{len(verdicts)}",
                   verdicts=verdicts, final=final,
                   all_sorted_gt=all(f["in_b_slot"] for f in final.values()))
        out.append(row)
        print(json.dumps(row))
    return out


if __name__ == "__main__":
    main()
