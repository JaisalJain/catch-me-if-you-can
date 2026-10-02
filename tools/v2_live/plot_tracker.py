#!/usr/bin/env python3
"""Live flight tracking vs the Phase 1 offline prediction (venv).

    env -u PYTHONPATH .venv/bin/python tools/v2_live/plot_tracker.py
Inputs: docs/v2/phase2/tracker_eval_50.json (catch_tasks tracker_eval), docs/v2/perception_lsq.json and
perception_detection.json (Phase 1). Output: docs/v2/phase2/tracker_live_vs_offline.png
"""
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
D = os.path.join(REPO, "docs", "v2")


def main():
    live = json.load(open(os.path.join(D, "phase2", "tracker_eval_50.json")))
    lsq = json.load(open(os.path.join(D, "perception_lsq.json")))
    det = json.load(open(os.path.join(D, "perception_detection.json")))
    s = live["summary"]
    fig, axs = plt.subplots(1, 3, figsize=(17, 4.6))
    # (a) prediction error vs time since release
    ax = axs[0]
    pe = s["pred_err_vs_time"]
    t = np.array([float(k) for k in pe]) * 1000
    ax.plot(t, [v["e95_mm"] for v in pe.values()], "o-", color="#d95f02", lw=2, label="LIVE: 95 % (49 throws, 129 Hz)")
    ax.plot(t, [v["median_mm"] for v in pe.values()], "o--", color="#d95f02", label="LIVE: median")
    ax.plot(t, [v["reported_e95_median_mm"] for v in pe.values()], "x:", color="#7570b3", label="LIVE: tracker's own 95 % (covariance)")
    r = det["results"]["320x240"]
    ax.plot(np.array(r["end_to_end_t"]) * 1000, r["end_to_end_e95_mm"], "s-", color="#1b9e77",
            label="Phase 1 offline: rendered detections (120 Hz, 25 ms, tumbling)")
    ts = np.array(lsq["t"]) * 1000
    ax.plot(ts, lsq["curves_e95_mm"]["120|2|0.025"], "k--", lw=1, label="Phase 1 model: white 2 mm, 120 Hz, 25 ms")
    ax.set_xlim(90, 320)
    ax.set_ylim(0, 60)
    ax.set_xlabel("estimate available at [ms after release]")
    ax.set_ylabel("error of the predicted intercept [mm]")
    ax.set_title("prediction error vs time (intercept = closest approach to B's ready TCP)", fontsize=10)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    # (b) latency
    ax = axs[1]
    lat = np.concatenate([np.array(x["latency_ms"]) for x in live["throws"] if x.get("ok") and x.get("n_fly")])
    rcv = np.concatenate([np.array(x["receive_ms"]) for x in live["throws"] if x.get("ok") and x.get("n_fly")])
    bins = np.arange(-0.5, 15.5, 1.0)
    ax.hist(rcv, bins, alpha=0.6, label=f"render stamp -> both images received (median {np.median(rcv):.0f} ms)")
    ax.hist(lat, bins, alpha=0.6, label=f"-> estimate published (median {np.median(lat):.0f}, p95 {np.percentile(lat, 95):.0f}, "
                                      f"max {lat.max():.0f} ms)")
    ax.axvline(40, color="r", ls=":")
    ax.text(39, ax.get_ylim()[1] * 0.5, "40 ms budget", rotation=90, ha="right", color="r", fontsize=8)
    ax.set_xlim(-0.5, 42)
    ax.set_xlabel("latency [ms] (sim time, RTF 1.0; 1 ms clock resolution)")
    ax.set_title("end-to-end perception latency (C++ tracker)", fontsize=10)
    ax.legend(fontsize=7)
    # (c) per-fix error
    ax = axs[2]
    fe = np.vstack([np.array(x["fix_err_mm"]) for x in live["throws"] if x.get("ok") and x.get("n_fly")])
    for k, (lab, col) in enumerate((("x", "#1b9e77"), ("y", "#7570b3"), ("z", "#d95f02"))):
        ax.hist(fe[:, k], np.arange(-8, 8.25, 0.5), histtype="step", lw=2, color=col,
                label=f"{lab}: {fe[:, k].mean():+.2f} +- {fe[:, k].std():.2f} mm")
    ax.set_xlabel("triangulated fix - ground truth [mm] (bias-calibrated)")
    ax.set_title(f"per-frame stereo error, {len(fe)} fixes (side + top, 320x240)", fontsize=10)
    ax.legend(fontsize=8)
    fig.suptitle("v2 step 3: live flight tracking (catch_tracker) vs Phase 1 offline", fontsize=11)
    fig.tight_layout()
    out = os.path.join(D, "phase2", "tracker_live_vs_offline.png")
    fig.savefig(out, dpi=105)
    print(out)


if __name__ == "__main__":
    main()
