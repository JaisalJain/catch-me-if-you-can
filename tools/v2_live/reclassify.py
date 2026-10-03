#!/usr/bin/env python3
"""Re-classify stored benchmark cells with the current reactive_eval.classify (outcome per trial + summary); the
cell's environment record (summary.cell) is kept. System python with the workspace sourced:
    python3 tools/v2_live/reclassify.py docs/v2/phase2/bench/*.json
"""
import argparse
import json
from collections import Counter

from catch_tasks.reactive_eval import classify, summarize


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    a = ap.parse_args()
    for f in a.files:
        d = json.load(open(f))
        before = Counter(r.get("outcome") for r in d["trials"])
        for r in d["trials"]:
            if r.get("throw_ok") is False and r.get("stage") is None:   # cells recorded before "throw failed" existed
                r.update(stage="throw", why=r.get("throw_msg") or "A did not throw")
            if "held" in r:
                r["outcome"] = classify(r)
        cell = d["summary"]["cell"]
        args = argparse.Namespace(label=cell["label"], mode=cell["mode"], source=cell["source"], noise=cell["noise"],
                                  seed=cell["seed"])
        s = summarize(d["trials"], args)
        s["cell"] = cell
        d["summary"] = s
        json.dump(d, open(f, "w"), indent=1)
        after = Counter(r.get("outcome") for r in d["trials"])
        if before != after:
            print(f"{f}: {dict(before)} -> {dict(after)}")


if __name__ == "__main__":
    main()
