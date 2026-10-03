#!/usr/bin/env python3
"""Offline counterpart of EVERY live benchmark cell on the SAME throws: the library entries reactive_eval draws
(--seed 1) with the same release-noise draws (throw_node's sample_noise, seed*1000+i), the measured live perception
(128.7 Hz / 63.9 Hz, 5 ms, first fix 47 ms, 1/1/2 mm), close +10 ms, B assumes 10 ms gripper latency.
venv + MuJoCo (tools/v2_catch_sim.py).

    env -u PYTHONPATH .venv/bin/python tools/v2_live/offline_cells.py [--n 60] [--only LABEL]
Output: docs/v2/phase2/offline_cells.json (resumable: cells already in it are skipped)
"""
import argparse
import json
import multiprocessing as mp
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import fastclose_diag as FD  # noqa: E402
import v2_catch_sim as CS  # noqa: E402
import v2_common as V  # noqa: E402

# label: (library, noise, gripper profile, run() kwargs on top of the live-match pinch settings)
CELLS = {
    "vision_none": ("pinch", None, "realistic", {}),
    "vision_low": ("pinch", "low", "realistic", {}),
    "vision_medium": ("pinch", "medium", "realistic", {}),
    "vision_high": ("pinch", "high", "realistic", {}),
    "cam60_medium": ("pinch", "medium", "realistic", {"camera_rate": 63.9}),
    "latency0_medium": ("pinch", "medium", "realistic", {"latency": 0.0}),
    "latency20_medium": ("pinch", "medium", "realistic", {"latency": 0.020}),
    "latency30_medium": ("pinch", "medium", "realistic", {"latency": 0.030}),
    "fastclose_medium": ("pinch", "medium", "fast_close", {}),
    "fastidealized_medium": ("pinch", "medium", "fast_idealized", {}),
    "cup_medium": ("cup", "medium", "realistic", "cup"),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=60)
    ap.add_argument("--only", default=None)
    ap.add_argument("--procs", type=int, default=1)
    a = ap.parse_args()
    c = V.cfg()
    out_p = os.path.join(V.REPO, "docs/v2/phase2/offline_cells.json")
    out = json.load(open(out_p)) if os.path.exists(out_p) else {}
    for label, (lib, noise, prof, extra) in CELLS.items():
        if (a.only and a.only != label) or label in out:
            continue
        ths = FD.live_throws(a.n, noise=noise, library=lib)
        if extra == "cup":
            kw = FD.live_kw(c, dict(mode="cup", cone_deg=10, cup_pre_gap=c["catch_sim"]["pinch_pre_gap_mm"]))
            kw.pop("close_lead")
        else:
            kw = FD.live_kw(c, extra)
        with mp.Pool(a.procs, initializer=FD._init, initargs=(prof,)) as pool:
            rows = pool.map(FD._job, [(i, th, kw) for i, th in enumerate(ths)])
        k = sum(r["held"] for r in rows)
        lo, hi = CS.wilson(k, len(rows))
        out[label] = dict(library=lib, noise=noise, gripper_profile=prof, kw=kw, held=k, n=len(rows), rate=k / len(rows),
                          ci95=[lo, hi], rows=rows)
        json.dump(out, open(out_p, "w"), indent=1)
        print(f"{label}: {k}/{len(rows)} held ({100 * k / len(rows):.0f} % [{100 * lo:.0f}-{100 * hi:.0f}])", flush=True)


if __name__ == "__main__":
    main()
