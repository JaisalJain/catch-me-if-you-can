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


def _rate(rows, key):
    r = rows.get(key)
    return None if r is None else r["summary"]


def _cup_panel(ax, cup, title):
    rows = cup["rows"]
    cones = [0, 10, 20, 30, 45, 60, 180]
    for prof, col in (("realistic", "#1b9e77"), ("fast_idealized", "#d95f02")):
        xs, r, lo, hi = [], [], [], []
        for cn in cones:
            sm = _rate(rows, f"{prof}|{cn}|measured")
            if sm:
                xs.append(min(cn, 90))
                r.append(100 * sm["rate"])
                lo.append(100 * sm["ci95"][0])
                hi.append(100 * sm["ci95"][1])
        if xs:
            ax.plot(xs, r, "o-", color=col, label=f"{prof}, measured perception")
            ax.fill_between(xs, lo, hi, color=col, alpha=0.15)
        pp = [(min(cn, 90), 100 * _rate(rows, f"{prof}|{cn}|perfect")["rate"]) for cn in cones if _rate(rows, f"{prof}|{cn}|perfect")]
        if pp:
            ax.plot(*zip(*pp), "x:", color=col, ms=8, label=f"{prof}, perfect perception")
        pg = [(min(cn, 90), 100 * _rate(rows, f"{prof}|{cn}|measured|pregap")["rate"]) for cn in cones
              if _rate(rows, f"{prof}|{cn}|measured|pregap")]
        if pg:
            ax.plot(*zip(*pg), "s--", color=col, mfc="none", ms=8, label=f"{prof}, fingers pre-closed to 56 mm")
    ax.set_xticks([0, 10, 20, 30, 45, 60, 90], ["0", "10", "20", "30", "45", "60", "free"])
    ax.set_xlabel("cup: cone half-angle around the ready approach axis [deg]")
    ax.set_ylabel("caught within limits [%] (95 % CI shaded)")
    ax.set_ylim(-3, 103)
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7)
    ax.set_title(title, fontsize=10)


def catch_figure():
    import v2_throws as T
    cupr, cups = load("catch_cup.json"), load("catch_cup_square.json")
    pinr, pins = load("catch_pinch.json"), load("catch_pinch_square.json")
    if not (cupr or cups or pinr or pins):
        return
    fig, axs = plt.subplots(2, 3, figsize=(19, 10.5))
    if cupr:
        _cup_panel(axs[0, 0], cupr, f"(a) cup, random throws (palm pitch for min joint speed), n={cupr['n_throws'] if 'n_throws' in cupr else 120}")
    if cups:
        _cup_panel(axs[0, 1], cups, f"(b) cup, square-arrival throws, n={cups.get('n_throws', 120)}")
    if cupr:
        # cup success vs the cube's tilt on arrival (random throws, cone 30, measured perception)
        import v2_catch_sim as CS
        ths = CS.throws(V.cfg(), cupr.get("n_throws"), "random")
        z_ready = V.ready_rotation(json.load(open(os.path.join(V.OUT, "ready_envelope.json")))["chosen"]["elev"])[:, 2]
        bins = [(0, 5), (5, 10), (10, 20), (20, 46)]
        for k, (prof, col) in enumerate((("realistic", "#1b9e77"), ("fast_idealized", "#d95f02"))):
            tr = cupr["rows"][f"{prof}|30|measured"]["trials"]
            tilt = np.array([abs(T.arrival_tilt(np.array(th["p_release"]), np.array(th["v_release"]), th["spin"], t["t_intercept"], z_ready))
                             if t.get("t_intercept") else 99 for t, th in zip(tr, ths)])
            held = np.array([t["held"] for t in tr])
            rates = [100 * held[(tilt >= lo) & (tilt < hi)].mean() if np.any((tilt >= lo) & (tilt < hi)) else 0 for lo, hi in bins]
            ns = [int(np.sum((tilt >= lo) & (tilt < hi))) for lo, hi in bins]
            xb = np.arange(len(bins)) + (k - 0.5) * 0.38
            axs[0, 2].bar(xb, rates, 0.38, color=col, label=prof)
            for x_, r_, n_ in zip(xb, rates, ns):
                axs[0, 2].text(x_, r_ + 1.5, f"n={n_}", ha="center", fontsize=7)
        axs[0, 2].set_xticks(range(len(bins)), [f"{lo}-{hi if hi < 46 else 45}" for lo, hi in bins])
        axs[0, 2].set_xlabel("cube tilt relative to B's palm at arrival [deg] (about the closing axis)")
        axs[0, 2].set_ylabel("caught [%]")
        axs[0, 2].set_ylim(0, 110)
        axs[0, 2].legend(fontsize=8)
        axs[0, 2].set_title("(c) cup (30 deg cone, random throws): the cube must land square", fontsize=10)
    for pin, ls, lab in ((pinr, "--", "random"), (pins, "-", "square")):
        if not pin:
            continue
        rows = pin["rows"]
        for prof, col in (("realistic", "#1b9e77"), ("fast_idealized", "#d95f02")):
            keys = sorted([k for k in rows if k.startswith(f"profile|{prof}|pinch|")], key=lambda k: float(k.split("|")[3]))
            if keys:
                mt = [float(k.split("|")[3]) for k in keys]
                r = [100 * rows[k]["summary"]["rate"] for k in keys]
                axs[1, 0].plot(mt, r, "o" + ls, color=col, label=f"{prof}, {lab} throws")
                for x_, y_, k in zip(mt, r, keys):
                    axs[1, 0].annotate(f"{rows[k]['summary']['match_used_mean']:.2f}", (x_, y_), fontsize=6, xytext=(2, 3),
                                       textcoords="offset points", color=col)
        if lab == "square":
            speeds = sorted({int(k.split("|")[1]) for k in rows if k.startswith("speed|")})
            pts = [(sp, 100 * rows[f"speed|{sp}|pinch|0.0"]["summary"]["rate"]) for sp in speeds if f"speed|{sp}|pinch|0.0" in rows]
            if pts:
                axs[1, 1].plot(*zip(*pts), "s:", color="#d95f02", label="5x force (fast_idealized actuator), nominal timing")
    axs[1, 0].set_xlabel("pinch: commanded velocity-matching fraction (labels: delivered within limits, mean)")
    axs[1, 0].set_ylabel("caught within limits [%]")
    axs[1, 0].set_ylim(-3, 103)
    axs[1, 0].grid(alpha=0.3)
    axs[1, 0].legend(fontsize=7)
    axs[1, 0].set_title("(d) pinch vs velocity matching: NOMINAL close timing (0 ms), 56 mm pre-gap,\nno gripper latency "
                        "(Phase 1 rows). Headline config (close +10 ms, 10 ms latency) in (f)", fontsize=9)
    axs[1, 1].axvline(150, color="grey", ls=":", lw=1)
    axs[1, 1].text(155, 5, "2F-85 max\n150 mm/s", fontsize=7, color="grey")
    axs[1, 1].set_xscale("log")
    cor = load("catch_corrections_square.json")
    if cor:
        for lead, col, mk in ((0, "#7570b3", "o--"), (-10, "#1b9e77", "o-")):
            pts = [(sp, 100 * cor["rows"][k]["summary"]["rate"]) for sp in (75, 150, 300, 600)
                   for k in [f"speed_realforce|{sp}|lead{lead}"] if k in cor["rows"]]
            if pts:
                axs[1, 1].plot(*zip(*pts), mk, color=col, label=f"REALISTIC force, close {'+10 ms' if lead else 'nominal'}, 10 ms latency")
    axs[1, 1].set_xlabel("finger gap closing speed [mm/s] (square throws, no velocity matching)")
    axs[1, 1].set_ylabel("caught within limits [%]")
    axs[1, 1].set_ylim(-3, 103)
    axs[1, 1].grid(alpha=0.3, which="both")
    axs[1, 1].legend(fontsize=7)
    axs[1, 1].set_title("(e) pinch vs closing speed: realistic grip force (green/purple) vs 5x force (orange)", fontsize=9)
    # (f) summary: the headline configurations (corrections rows: 10 ms gripper latency, B compensates 10 ms)
    ax = axs[1, 2]
    items = []
    cs_, cr_ = load("catch_corrections_square.json"), load("catch_corrections.json")
    spec = [("pinch, close +10 ms\nrealistic, square", cs_, "pinch|realistic|lead-10", "#1b9e77"),
            ("pinch, close +10 ms\nrealistic, random", cr_, "pinch|realistic|lead-10", "#1b9e77"),
            ("pinch, nominal close\nrealistic, square", cs_, "pinch|realistic|lead0", "#66c2a5"),
            ("cup pre-closed 56 mm, 10 deg\nrealistic, square", cs_, "cup_pregap|realistic|cone10", "#1b9e77"),
            ("cup open, 30 deg\nrealistic, square", cs_, "cup_open|realistic|cone30", "#66c2a5"),
            ("pinch, nominal close\nfast_close (real force), square", cs_, "pinch|fast_close|lead0", "#7570b3"),
            ("pinch, nominal close\nfast_idealized (5x force), square", cs_, "pinch|fast_idealized|lead0", "#d95f02")]
    for lab, src, key, col in spec:
        sm = _rate(src["rows"], key) if src else None
        if sm:
            items.append((lab, 100 * sm["rate"], sm["ci95"], col))
    sm = _rate((pins or {}).get("rows", {}), "profile|realistic|pinch_side|1.0") if pins else None
    if sm:
        items.append(("side chopsticks + match\nany profile", 100 * sm["rate"], sm["ci95"], "#999999"))
    if items:
        y = np.arange(len(items))
        ax.barh(y, [i[1] for i in items], color=[i[3] for i in items],
                xerr=np.array([[i[1] - 100 * i[2][0], 100 * i[2][1] - i[1]] for i in items]).T, capsize=3)
        ax.set_yticks(y, [i[0] for i in items], fontsize=7)
        ax.set_xlim(0, 105)
        ax.invert_yaxis()
        ax.set_xlabel("caught within limits [%]")
        ax.set_title("(f) headline configurations (measured perception, 10 ms gripper latency)", fontsize=10)
        for k, it in enumerate(items):
            ax.text(it[1] + 1, k, f"{it[1]:.0f} %", va="center", fontsize=7)
    fig.suptitle("Catch modes in offline MuJoCo, vision-only B (120 Hz, measured-detector noise, 25 ms latency, re-planning). "
                 "gripper_profile per curve: realistic = 2F-85 150 mm/s; fast_idealized is NOT a real 2F-85", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(V.OUT, "catch_modes.png"), dpi=100)


def corrections_figure():
    """Phase 2 review: close timing, speed vs force, gripper latency (realistic pinch / cup)."""
    cs_, ps = load("catch_corrections_square.json"), load("catch_pinch_square.json")
    if not cs_:
        return
    fig, axs = plt.subplots(1, 3, figsize=(17, 4.6))
    leads, rates = [], []
    for lead in (-60, -40, -30, -20, -10, 0, 10, 20, 30):
        k = f"lead|{lead}|pinch|0.0|realistic" if lead else "profile|realistic|pinch|0.0"
        if ps and k in ps["rows"]:
            leads.append(-lead)
            rates.append(100 * ps["rows"][k]["summary"]["rate"])
    axs[0].plot(leads, rates, "o-", color="#1b9e77", label="no gripper latency (Phase 1 rows)")
    lr = [(-l, 100 * cs_["rows"][f"pinch|realistic|lead{l}"]["summary"]["rate"]) for l in (0, -10, -20)
          if f"pinch|realistic|lead{l}" in cs_["rows"]]
    if lr:
        axs[0].plot(*zip(*lr), "s--", color="#d95f02", ms=8, label="10 ms latency, B compensates 10 ms")
    axs[0].set_xlabel("close offset vs nominal (40 mm gap at predicted arrival) [ms], + = later")
    axs[0].set_title("realistic pinch: close timing (square throws)", fontsize=10)
    for prof, col in (("realistic", "#1b9e77"), ("fast_close", "#7570b3"), ("fast_idealized", "#d95f02")):
        pts = [(-l, 100 * cs_["rows"][f"pinch|{prof}|lead{l}"]["summary"]["rate"]) for l in (0, -10, -20)
               if f"pinch|{prof}|lead{l}" in cs_["rows"]]
        if pts:
            axs[1].plot(*zip(*pts), "o-", color=col, label=prof + (" (realistic force)" if prof == "fast_close" else ""))
    axs[1].set_xlabel("close offset [ms], + = later")
    axs[1].set_title("speed vs force: fast_close has the realistic grip force", fontsize=10)
    for mode, col in (("pinch|realistic|lead-10", "#1b9e77"), ("cup_pregap|realistic|cone10", "#7570b3")):
        pts = [(ms, 100 * cs_["rows"][f"latency|{ms}|{mode}"]["summary"]["rate"]) for ms in (0, 10, 20, 30)
               if f"latency|{ms}|{mode}" in cs_["rows"]]
        if pts:
            axs[2].plot(*zip(*pts), "o-", color=col, label=mode.split("|")[0].replace("_", " "))
    axs[2].axvline(10, color="grey", ls=":", lw=1)
    axs[2].set_xlabel("true gripper command latency [ms] (B assumes 10 ms)")
    axs[2].set_title("gripper command latency (realistic, square throws)", fontsize=10)
    for ax in axs:
        ax.set_ylim(-3, 103)
        ax.set_ylabel("caught within limits [%]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
    fig.suptitle("Phase 1 corrections (Phase 2 review): 120 throws per point, vision-only B, measured perception", fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(V.OUT, "catch_corrections.png"), dpi=105)


def main():
    perception_figure()
    catch_figure()
    corrections_figure()


if __name__ == "__main__":
    main()
