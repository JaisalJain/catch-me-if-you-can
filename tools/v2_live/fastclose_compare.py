#!/usr/bin/env python3
"""fast_close, per throw: live (log/p2/bench/fastclose_medium/trial_*.npz, every trial recorded) vs offline on the
same throw and noise draw (docs/v2/phase2/fastclose_diag.json "fast_close|noise_medium"): outcome, cube
misalignment in the hand at the arrival, gap 40 mm vs arrival. System python, workspace sourced.

    python3 tools/v2_live/fastclose_compare.py [--live-dir log/p2/bench/fastclose_medium]
"""
import argparse
import glob
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from close_timing import REPO, robot_b, timing  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--live-dir", default=os.path.join(REPO, "log/p2/bench/fastclose_medium"))
    ap.add_argument("--offline-key", default="fast_close|noise_medium")
    a = ap.parse_args()
    off = {r["i"]: r for r in json.load(open(os.path.join(REPO, "docs/v2/phase2/fastclose_diag.json")))[a.offline_key]["rows"]}
    rb = robot_b()
    rows = []
    for f in sorted(glob.glob(os.path.join(a.live_dir, "trial_*.npz"))):
        t = timing(f, rb)
        if t is None:
            continue
        meta, dt, miss, sp, mis = t
        o = off.get(meta["i"])
        if o is None:
            continue
        rows.append(dict(i=meta["i"], live_held=meta["outcome"] == "held", live_outcome=meta["outcome"], live_mis=mis,
                         live_dt40=dt, off_held=o["held"], off_mis=o["misalign_deg"], off_dt40=o["gap40_minus_arrival_ms"]))
    if not rows:
        print("no live recordings")
        return
    lm, om = np.array([r["live_mis"] for r in rows]), np.array([r["off_mis"] for r in rows])
    lh, oh = np.array([r["live_held"] for r in rows]), np.array([r["off_held"] for r in rows])
    print(f"{len(rows)} throws: live held {lh.sum()}, offline held {oh.sum()}; both {np.sum(lh & oh)}, live only "
          f"{np.sum(lh & ~oh)}, offline only {np.sum(~lh & oh)}, neither {np.sum(~lh & ~oh)}")
    print(f"misalignment at the arrival [deg]: live median {np.median(lm):.1f}, offline {np.median(om):.1f}; "
          f"per-throw correlation {np.corrcoef(lm, om)[0, 1]:.2f}, live - offline median {np.median(lm - om):+.1f}")
    for name, sel in (("live held", lh), ("live failed", ~lh), ("offline held", oh), ("offline failed", ~oh)):
        m = lm if name.startswith("live") else om
        if sel.any():
            print(f"   {name:15s} misalignment median {np.median(m[sel]):5.1f} (range {m[sel].min():.1f}-{m[sel].max():.1f})")
    print("throws whose outcome differs:")
    for r in rows:
        if r["live_held"] != r["off_held"]:
            print(f"   #{r['i']:2d} live {r['live_outcome']:10s} {r['live_mis']:5.1f} deg  gap40 {r['live_dt40']:+5.1f} ms | "
                  f"offline {'held' if r['off_held'] else 'failed':6s} {r['off_mis']:5.1f} deg  gap40 {r['off_dt40']:+5.1f} ms")
    json.dump(rows, open(os.path.join(REPO, "docs/v2/phase2/fastclose_compare.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
