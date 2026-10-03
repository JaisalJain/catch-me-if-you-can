#!/usr/bin/env python3
"""Failure taxonomy clips from the live benchmark recordings (venv, NVIDIA EGL).

    env -u PYTHONPATH .venv/bin/python tools/v2_live/render_failures.py [--per-class 1]
For each failure class (reactive_eval: missed intercept, late close, slide-off, fingertip, abort, collision) the first
recorded example (log/p2/bench/<cell>/trial_*.npz, meta = the trial row; the default cell vision_medium first) is
replayed with tools/render_replay.py (arms + cubes pinned to the recorded states) -> docs/media/v2_fail_<class>.mp4.
Classes without an example are listed.
"""
import argparse
import glob
import json
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "tools"))
import numpy as np  # noqa: E402

import render_replay as RR  # noqa: E402

CLASSES = ["missed intercept", "late close", "slide-off", "fingertip", "abort", "collision"]
CELL_ORDER = ["vision_medium", "latency0_medium", "latency30_medium", "cup_medium_solo", "vision_high", "vision_low", "vision_none", "cup_medium", "cam60_medium", "latency0_medium",
              "latency20_medium", "latency30_medium", "plan_medium", "plan_high", "plan_low", "plan_none",
              "fastclose_medium", "fastidealized_medium"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--per-class", type=int, default=1)
    a = ap.parse_args()
    found = {c: [] for c in CLASSES}
    cells = [c for c in CELL_ORDER + sorted(set(os.listdir(os.path.join(REPO, "log/p2/bench"))) - set(CELL_ORDER))
             if os.path.isdir(os.path.join(REPO, "log/p2/bench", c))]
    for cell in cells:
        # outcome from the cell json (re-classified by tools/v2_live/reclassify.py), not the recording's meta
        cj = os.path.join(REPO, "docs/v2/phase2/bench", f"{cell}.json")
        outcome = {r["i"]: r.get("outcome") for r in json.load(open(cj))["trials"]} if os.path.exists(cj) else {}
        for f in sorted(glob.glob(os.path.join(REPO, "log/p2/bench", cell, "trial_*.npz"))):
            meta = json.loads(str(np.load(f, allow_pickle=True)["meta"]))
            meta["outcome"] = o = outcome.get(meta["i"], meta.get("outcome"))
            if o in found and len(found[o]) < a.per_class:
                found[o].append((cell, f, meta))
    out_dir = os.path.join(REPO, "docs", "media")
    index = []
    for cls, ex in found.items():
        for k, (cell, f, meta) in enumerate(ex):
            rec = dict(np.load(f, allow_pickle=True))
            label = (f"{cell} #{meta['i']}: {cls} (closest {meta.get('min_cube_tcp_mm')} mm, trigger {meta.get('trigger')}"
                     + (f", {meta.get('abort_reason')[:40]}" if cls == "abort" else "") + ")")
            name = f"v2_fail_{cls.replace(' ', '_').replace('-', '_')}" + (f"_{k}" if k else "") + ".mp4"
            RR.render(rec, os.path.join(out_dir, name), view="catch", window="catch", speed=0.25, label=label)
            index.append(dict(cls=cls, clip=f"docs/media/{name}", cell=cell, trial=meta["i"], row=meta))
            print("wrote", name, "-", label)
    missing = [c for c, ex in found.items() if not ex]
    if missing:
        print("no example recorded for:", missing)
    json.dump(dict(clips=index, missing=missing), open(os.path.join(REPO, "docs/v2/phase2/failure_clips.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
