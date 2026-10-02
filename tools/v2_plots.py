#!/usr/bin/env python3
"""v2 Phase 1 plots from docs/v2/*.json (offline, venv).

    env -u PYTHONPATH .venv/bin/python tools/v2_plots.py
"""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

import v2_common as V  # noqa: E402


def load(name):
    p = os.path.join(V.OUT, name)
    return json.load(open(p)) if os.path.exists(p) else None


def perception_figure():
    det, bench = load("perception_detection.json"), load("perception_render_bench.json")
    if not det or not bench:
        return
    fig, axs = plt.subplots(1, 3, figsize=(17, 4.5))
    sizes = list(det["results"])
    x = np.arange(3)
    for k, sz in enumerate(sizes):
        r = det["results"][sz]
        axs[0].bar(x + (k - 0.5) * 0.35, r["std_mm"], 0.35, label=f"{sz} stereo: std", alpha=0.85)
        axs[0].errorbar(x + (k - 0.5) * 0.35, r["bias_mm"], fmt="kx", ms=8)
    axs[0].set_xticks(x, ["x", "y", "z"])
    axs[0].set_ylabel("mm")
    axs[0].set_title("Per-frame stereo error, tumbling cube (bars std, x bias)", fontsize=9)
    axs[0].legend(fontsize=8)
    t = det["results"][sizes[0]]["end_to_end_t"]
    for sz in sizes:
        axs[1].plot(np.array(t) * 1000, det["results"][sz]["end_to_end_e95_mm"], "o-", label=f"{sz} measured detections")
    lsq = load("perception_lsq.json")
    if lsq:
        ts = np.array(lsq["t"]) * 1000
        for s, ls in ((2, "--"), (4, ":")):
            axs[1].plot(ts, lsq["curves_e95_mm"][f"120|{s}|0.025"], "k", ls=ls, lw=1, label=f"white noise {s} mm (model)")
    axs[1].set_ylim(0, 60)
    axs[1].set_xlim(100, 350)
    axs[1].set_xlabel("time since release [ms]")
    axs[1].set_ylabel("95 % error of predicted intercept [mm]")
    axs[1].set_title("End to end: LSQ on rendered detections (120 Hz, 25 ms)", fontsize=9)
    axs[1].grid(alpha=0.3)
    axs[1].legend(fontsize=8)
    rows = bench["rows"]
    labels = [f"{r['size']}\nx{r['ncam']} {r['rate']}Hz" for r in rows]
    xx = np.arange(len(rows))
    axs[2].bar(xx, [r["render_ms"] * r["rate"] / 10 for r in rows], color="#7570b3", label="render thread load [% of one core]")
    ax2 = axs[2].twinx()
    ax2.plot(xx, [r["max_rtf_concurrent"] for r in rows], "o-", color="#1b9e77", label="max physics RTF next to it")
    ax2.axhline(bench["physics_alone"]["max_rtf"], color="#1b9e77", ls=":", lw=1)
    ax2.plot(xx, [r["serial_rtf"] for r in rows], "s--", color="#d95f02", label="RTF bound if rendered in the physics thread")
    ax2.set_ylabel("real-time factor bound")
    ax2.set_yscale("log")
    axs[2].set_xticks(xx, labels, fontsize=6)
    axs[2].set_ylabel("% of one core")
    axs[2].set_title("Render cost (RGB + depth, EGL) and physics headroom", fontsize=9)
    h1, l1 = axs[2].get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    axs[2].legend(h1 + h2, l1 + l2, fontsize=7, loc="upper left")
    fig.tight_layout()
    fig.savefig(os.path.join(V.OUT, "perception_detection_bench.png"), dpi=105)


def catch_figure():
    cup, pinch = load("catch_cup.json"), load("catch_pinch.json")
    if not cup and not pinch:
        return
    fig, axs = plt.subplots(1, 3, figsize=(18, 5))
    if cup:
        rows = cup["rows"]
        profs = sorted({k.split("|")[0] for k in rows})
        cones = sorted({float(k.split("|")[1]) for k in rows})
        for prof, col in zip(profs, ("#1b9e77", "#d95f02")):
            for perc, ls in (("measured", "-"), ("perfect", ":")):
                ys = [rows[f"{prof}|{c:g}|{perc}"]["summary"] for c in cones if f"{prof}|{c:g}|{perc}" in rows]
                cs = [c for c in cones if f"{prof}|{c:g}|{perc}" in rows]
                if not ys:
                    continue
                r = np.array([y["rate"] for y in ys]) * 100
                lo = np.array([y["ci95"][0] for y in ys]) * 100
                hi = np.array([y["ci95"][1] for y in ys]) * 100
                xs = np.minimum(cs, 90)
                axs[0].plot(xs, r, "o" + ls, color=col, label=f"{prof}, perception {perc}")
                if perc == "measured":
                    axs[0].fill_between(xs, lo, hi, color=col, alpha=0.15)
        axs[0].set_xticks([0, 10, 20, 30, 45, 60, 90], ["0", "10", "20", "30", "45", "60", "free"])
        axs[0].set_xlabel("cone half-angle around the ready approach axis [deg]")
        axs[0].set_ylabel("caught [%] (95 % CI shaded)")
        axs[0].set_ylim(-3, 103)
        axs[0].grid(alpha=0.3)
        axs[0].legend(fontsize=8)
        axs[0].set_title(f"(a) natural cup vs cone, n={ys[0]['n']} throws each")
    if pinch:
        rows = pinch["rows"]
        for mode, ax_i, ttl in (("pinch", 1, "(b) pinch, natural approach"), ("pinch_side", 1, None)):
            pass
        for prof, col in (("realistic", "#1b9e77"), ("fast_idealized", "#d95f02")):
            for mode, ls in (("pinch", "-"), ("pinch_side", "--")):
                keys = sorted([k for k in rows if k.startswith(f"profile|{prof}|{mode}|")], key=lambda k: float(k.split("|")[3]))
                if not keys:
                    continue
                mt = [float(k.split("|")[3]) for k in keys]
                used = [rows[k]["summary"]["match_used_mean"] for k in keys]
                r = [rows[k]["summary"]["rate"] * 100 for k in keys]
                axs[1].plot(mt, r, "o" + ls, color=col, label=f"{prof}, {mode}")
                for x_, y_, u in zip(mt, r, used):
                    axs[1].annotate(f"{u:.2f}", (x_, y_), fontsize=6, xytext=(2, 3), textcoords="offset points", color=col)
        axs[1].set_xlabel("commanded velocity-matching fraction (labels: delivered within limits, mean)")
        axs[1].set_ylabel("caught within limits [%]")
        axs[1].set_ylim(-3, 103)
        axs[1].grid(alpha=0.3)
        axs[1].legend(fontsize=8)
        axs[1].set_title("(b) pinch vs velocity matching")
        keys = [k for k in rows if k.startswith("speed|")]
        speeds = sorted({int(k.split("|")[1]) for k in keys})
        for mode in ("pinch", "pinch_side"):
            for mt, col in ((0.0, "#7570b3"), (0.5, "#1b9e77"), (1.0, "#d95f02")):
                ks = [f"speed|{s}|{mode}|{mt}" for s in speeds if f"speed|{s}|{mode}|{mt}" in rows]
                if not ks:
                    continue
                axs[2].plot([int(k.split("|")[1]) for k in ks], [rows[k]["summary"]["rate"] * 100 for k in ks],
                            "o" + ("-" if mode == "pinch" else "--"), color=col, label=f"{mode}, match cmd {mt}")
        axs[2].axvline(150, color="grey", ls=":", lw=1)
        axs[2].text(155, 5, "2F-85 max\n150 mm/s", fontsize=7, color="grey")
        axs[2].set_xscale("log")
        axs[2].set_xlabel("finger gap closing speed [mm/s] (fast_idealized actuator for the whole sweep)")
        axs[2].set_ylabel("caught within limits [%]")
        axs[2].set_ylim(-3, 103)
        axs[2].grid(alpha=0.3, which="both")
        axs[2].legend(fontsize=7)
        axs[2].set_title("(c) pinch vs closing speed")
    fig.suptitle("Catch modes, offline MuJoCo, vision-only B (120 Hz, measured-detector noise, 25 ms latency); "
                 "gripper_profile per curve — fast_idealized is NOT a real 2F-85")
    fig.tight_layout()
    fig.savefig(os.path.join(V.OUT, "catch_modes.png"), dpi=105)


def main():
    perception_figure()
    catch_figure()


if __name__ == "__main__":
    main()
