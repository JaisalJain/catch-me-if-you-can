#!/usr/bin/env python3
"""v2 step 6 (v2.1): benchmark table + plots from docs/v2/phase2/bench/*.json (live, every cell ALONE, sequential:
tools/v2_live/bench_seq.sh), the v2.0 cells under load (docs/v2/phase2/bench_v2.0/: three sims in parallel) and the
offline counterparts (docs/v2/catch_livematch*.json).

    env -u PYTHONPATH .venv/bin/python tools/v2_live/plot_bench.py
Outputs: docs/v2/phase2/bench_table.md (clean), bench_under_load.md, noise_trend.md, bench.png
"""
import glob
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
D = os.path.join(REPO, "docs", "v2")
B = os.path.join(D, "phase2", "bench")
B_LOAD = os.path.join(D, "phase2", "bench_v2.0")

ORDER = ["vision_none", "vision_low", "vision_medium", "vision_high", "plan_none", "plan_low", "plan_medium", "plan_high",
         "cup_medium", "cam60_medium", "latency0_medium", "latency20_medium", "latency30_medium", "fastclose_medium",
         "fastidealized_medium"]
DESC = {"vision_none": "vision-only, pinch, noise none", "vision_low": "vision-only, pinch, noise low",
        "vision_medium": "**vision-only, pinch, noise medium (default)**",
        "vision_medium_solo": "... same, sim running alone (no parallel cells)", "vision_high": "vision-only, pinch, noise high",
        "plan_none": "plan-informed, pinch, noise none", "plan_low": "plan-informed, pinch, noise low",
        "plan_medium": "plan-informed, pinch, noise medium", "plan_high": "plan-informed, pinch, noise high",
        "cup_medium": "vision-only, pre-closed cup (square throws), medium", "cam60_medium": "vision-only, pinch, 64 Hz cameras, medium",
        "latency0_medium": "vision-only, pinch, gripper latency 0 ms (B assumes 10), medium",
        "latency0_medium_solo": "... same, sim running alone", "cup_medium_solo": "... same, sim running alone",
        "latency20_medium": "... latency 20 ms", "latency30_medium": "... latency 30 ms",
        "fastclose_medium": "B's gripper fast_close (NOT a 2F-85; 10x speed, realistic force), medium ",
        "fastidealized_medium": "B's gripper fast_idealized (NOT a 2F-85; 10x speed, 5x force), medium"}
# offline counterpart (file, key) for the matching cells (offline throws: exact library throws, no release noise;
# vision-only B re-estimates the flight, so the release noise level only moves the intercept around)
OFFLINE = {"vision_none": ("catch_livematch.json", "live|pinch|realistic"),
           "vision_low": ("catch_livematch.json", "live|pinch|realistic"),
           "vision_medium": ("catch_livematch.json", "live|pinch|realistic"),
           "vision_medium_solo": ("catch_livematch.json", "live|pinch|realistic"),
           "vision_high": ("catch_livematch.json", "live|pinch|realistic"),
           "cup_medium": ("catch_livematch_square.json", "live|cup_pregap|realistic|cone10"),
           "cup_medium_solo": ("catch_livematch_square.json", "live|cup_pregap|realistic|cone10"),
           "latency0_medium_solo": ("catch_livematch.json", "live|pinch|realistic|latency0"),
           "cam60_medium": ("catch_livematch.json", "live|pinch|realistic|cam64"),
           "latency0_medium": ("catch_livematch.json", "live|pinch|realistic|latency0"),
           "latency20_medium": ("catch_livematch.json", "live|pinch|realistic|latency20"),
           "latency30_medium": ("catch_livematch.json", "live|pinch|realistic|latency30"),
           "fastclose_medium": ("catch_livematch.json", "live|pinch|fast_close"),
           "fastidealized_medium": ("catch_livematch.json", "live|pinch|fast_idealized")}


LIB = {}


def v_plan(library, entry):
    if library not in LIB:
        LIB[library] = json.load(open(os.path.join(REPO, "src/catch_throw/config", f"library_{library}.json")))["entries"]
    return np.array(LIB[library][entry]["params"]["v_release"])


def realised(trials, library):
    """Realised release error per throw at the ACTUAL release time t_a = t_p + dt (t_p published, dt the drawn timing
    offset). reactive_eval fits the ground-truth flight at t_p: v_fit(t_p) = v_a - g dt, so v_a = v_plan + dv_fit + g dt.
    Returns dict of arrays: relative speed error, direction error [deg], timing offset [ms], and A's execution error
    |v_a - v_commanded| [mm/s] (v_commanded = v_plan + the drawn perturbation)."""
    G = np.array([0.0, 0.0, -9.81])
    out = dict(speed=[], dir=[], dt=[], exec=[])
    for r in trials:
        if "release_dv_mm_s" not in r or r.get("noise_dt") is None or r.get("noise_dv") is None or "entry" not in r:
            continue
        vp = v_plan(library, r["entry"])
        t = float(r["noise_dt"])
        va = vp + np.array(r["release_dv_mm_s"]) / 1000.0 + G * t
        vc = vp + np.array(r["noise_dv"])
        out["speed"].append(np.linalg.norm(va) / np.linalg.norm(vp) - 1)
        out["dir"].append(np.degrees(np.arccos(np.clip(va @ vp / np.linalg.norm(va) / np.linalg.norm(vp), -1, 1))))
        out["dt"].append(1000 * t)
        out["exec"].append(1000 * np.linalg.norm(va - vc))
    return {k: np.array(v) for k, v in out.items()}


def offline(cell):
    """Offline counterpart on the SAME throws (tools/v2_live/offline_cells.py) if available, else the live-match row
    (Phase 1 throw distribution)."""
    p = os.path.join(D, "phase2", "offline_cells.json")
    if os.path.exists(p):
        oc = json.load(open(p))
        if cell in oc:
            return dict(rate=oc[cell]["rate"], ci95=oc[cell]["ci95"])
    if cell not in OFFLINE:
        return None
    f, k = OFFLINE[cell]
    p = os.path.join(D, f)
    if not os.path.exists(p):
        return None
    rows = json.load(open(p))["rows"]
    return rows[k]["summary"] if k in rows else None


LEVELS = ["none", "low", "medium", "high"]


def load(d):
    return {os.path.basename(f)[:-5]: json.load(open(f)) for f in glob.glob(os.path.join(d, "*.json"))}


def trend(cells, src):
    """Cochran-Armitage test for a linear trend of the catch rate over the noise levels (scores 0..3): z, two-sided p,
    and the slope of a least-squares line through the rates [points per level]."""
    from math import erf, sqrt
    ks, ns = [], []
    for lv in LEVELS:
        s = cells[f"{src}_{lv}"]["summary"]
        ks.append(s["held"])
        ns.append(s["n_valid"])
    k, n, x = np.array(ks, float), np.array(ns, float), np.arange(4.0)
    p = k.sum() / n.sum()
    xb = (n * x).sum() / n.sum()
    t = (x * (k - n * p)).sum()
    var = p * (1 - p) * (n * (x - xb) ** 2).sum()
    z = t / sqrt(var) if var > 0 else 0.0
    pv = 2 * (1 - 0.5 * (1 + erf(abs(z) / sqrt(2))))
    slope = np.polyfit(x, 100 * k / n, 1)[0]
    return z, pv, slope, k, n


def arrival_shift(trials, library):
    """How far each throw's release error moves the cube at the planned arrival (|dv_realised| * t_target + the
    timing offset along the path) [mm] - what B must absorb, from the realised release error of every trial."""
    out = []
    for r in trials:
        if "release_dv_mm_s" not in r or r.get("noise_dt") is None or "entry" not in r:
            continue
        if library not in LIB:
            v_plan(library, 0)
        e = LIB[library][r["entry"]]
        rr = realised([r], library)
        vp = np.array(e["params"]["v_release"])
        va = vp * (1 + rr["speed"][0]) if len(rr["speed"]) else vp
        dv = np.array(r["release_dv_mm_s"]) / 1000.0
        out.append(1000 * np.linalg.norm(dv * e["t_target"]) + abs(r["noise_dt"]) * 1000 * np.linalg.norm(va))
    return np.array(out)


def row(c, s, cells_src, off, extra=True):
    rr = realised(cells_src[c]["trials"], "cup" if c.startswith("cup") else "pinch")
    oc = ", ".join(f"{k} {v}" for k, v in sorted(s["outcomes"].items(), key=lambda x: -x[1]))
    tg = ", ".join(f"{k} {v}" for k, v in s["triggers"].items() if k)
    rp = "—" if c.startswith("plan") else (f"{s.get('replan_rate_hz', 0):.0f} Hz"
                                            + (f" ({s['replan_rate_hz_p10_p90'][0]:.0f}–{s['replan_rate_hz_p10_p90'][1]:.0f})"
                                               if s.get("replan_rate_hz_p10_p90") else ""))
    rtf = s.get("rtf")
    la = s.get("loadavg_1min")
    cm = "before the throw (plan)" if c.startswith("plan") else f"{s.get('t_commit_ms', 0):.0f} ms"
    return (f"| {DESC.get(c, c)} | {s['n_valid']} | {s['held']} | **{100 * s['rate']:.0f} %** "
            f"[{100 * s['ci95'][0]:.0f}–{100 * s['ci95'][1]:.0f}] | "
            + (f"{100 * off['rate']:.0f} % [{100 * off['ci95'][0]:.0f}–{100 * off['ci95'][1]:.0f}]" if off else "—")
            + f" | {oc} | {tg} | {cm} | {rp} | "
            + (f"{rtf['trial_median']:.3f} / {rtf['window_2s_min']:.3f}" if rtf and rtf.get("window_2s_min") else "—")
            + " | " + (f"{la['median']:.1f} / {la['max']:.1f}" if la else "—") + " | "
            + (f"{100 * rr['speed'].std():.2f} %, {np.sqrt(np.mean(rr['dir'] ** 2)):.2f}°, {rr['dt'].std():.1f} ms"
               if len(rr["speed"]) else "—") + " |")


def main():
    cells = load(B)
    under = load(B_LOAD)
    labels = [c for c in ORDER if c in cells]
    hdr = ["| cell | n | caught | rate [95 % CI] | offline, same throws | outcomes | trigger | commit after 1st fix | "
           "re-plan rate median (p10–p90) | RTF trial median / min 2 s window | load avg 1 min median / max (24 cores) | "
           "realised release error (speed sd, direction rms, time sd) |", "|---" * 12 + "|"]
    lines = hdr + [row(c, cells[c]["summary"], cells, offline(c)) for c in labels]
    open(os.path.join(D, "phase2", "bench_table.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    # under load (v2.0): the same cells with three sims in parallel (+ three re-runs alone at the time)
    ul = ["| cell | v2.0 under load (3 sims in parallel) | re-plan rate | v2.1 alone | re-plan rate |", "|---" * 5 + "|"]
    for c in labels:
        if c in under:
            a, b = under[c]["summary"], cells[c]["summary"]
            ul.append(f"| {DESC.get(c, c)} | {100 * a['rate']:.0f} % [{100 * a['ci95'][0]:.0f}–{100 * a['ci95'][1]:.0f}] "
                      f"({a['held']}/{a['n_valid']}) | {a.get('replan_rate_hz', '—')} Hz | {100 * b['rate']:.0f} % "
                      f"[{100 * b['ci95'][0]:.0f}–{100 * b['ci95'][1]:.0f}] ({b['held']}/{b['n_valid']}) | "
                      f"{b.get('replan_rate_hz', '—')} Hz |")
    open(os.path.join(D, "phase2", "bench_under_load.md"), "w").write("\n".join(ul) + "\n")
    # noise trend + how far the release error moves the arrival
    nt = []
    for src in ("vision", "plan"):
        if all(f"{src}_{lv}" in cells for lv in LEVELS):
            z, pv, sl, k, n = trend(cells, src)
            nt.append(f"* {src}: caught {', '.join(f'{int(a)}/{int(b)}' for a, b in zip(k, n))} (none → high); "
                      f"Cochran–Armitage trend z = {z:+.2f}, p = {pv:.3g}; slope {sl:+.1f} points per level")
    sh = []
    for lv in LEVELS:
        c = f"vision_{lv}"
        if c in cells:
            a = arrival_shift(cells[c]["trials"], "pinch")
            sh.append(f"| {lv} | {np.median(a):.0f} | {np.percentile(a, 95):.0f} | {a.max():.0f} |")
    nt += ["", "| noise level | arrival shift median [mm] | p95 | max |", "|---|---|---|---|"] + sh
    open(os.path.join(D, "phase2", "noise_trend.md"), "w").write("\n".join(nt) + "\n")
    print("\n".join(nt))

    fig, axs = plt.subplots(1, 3, figsize=(19, 6.5))
    for src, col in (("vision", "#d95f02"), ("plan", "#7570b3")):
        xs, r, lo, hi = [], [], [], []
        for k, n in enumerate(LEVELS):
            c = f"{src}_{n}"
            if c in cells:
                s = cells[c]["summary"]
                xs.append(k)
                r.append(100 * s["rate"])
                lo.append(100 * s["ci95"][0])
                hi.append(100 * s["ci95"][1])
        if xs:
            axs[0].errorbar(xs, r, yerr=[np.array(r) - lo, np.array(hi) - r], fmt="o-", capsize=4, color=col,
                            label="camera-only (vision)" if src == "vision" else "plan-informed (A's published plan)")
    axs[0].set_xticks(range(4), LEVELS)
    axs[0].set_xlabel("release noise level (catch_throw/config/v2.yaml)")
    axs[0].set_ylabel("caught [%] (95 % Wilson CI)")
    axs[0].set_ylim(-3, 103)
    axs[0].grid(alpha=0.3)
    axs[0].legend()
    axs[0].set_title("camera-only vs plan-informed, pinch, realistic 2F-85", fontsize=10)
    others = [c for c in ("vision_medium", "cup_medium", "cam60_medium", "latency0_medium", "latency20_medium",
                          "latency30_medium", "fastclose_medium", "fastidealized_medium") if c in cells]
    y = np.arange(len(others))
    rl = [100 * cells[c]["summary"]["rate"] for c in others]
    ci = np.array([[100 * (cells[c]["summary"]["rate"] - cells[c]["summary"]["ci95"][0]),
                    100 * (cells[c]["summary"]["ci95"][1] - cells[c]["summary"]["rate"])] for c in others]).T
    axs[1].barh(y - 0.18, rl, 0.36, xerr=ci, capsize=3, color="#d95f02", label="live (sim alone)")
    ro = [100 * offline(c)["rate"] if offline(c) else np.nan for c in others]
    axs[1].barh(y + 0.18, ro, 0.36, color="#1b9e77", label="offline, same throws + noise draws (measured live perception)")
    names = {"vision_medium": "pinch (default)", "cup_medium": "pre-closed cup", "cam60_medium": "64 Hz cameras",
             "latency0_medium": "gripper latency 0 ms", "latency20_medium": "gripper latency 20 ms",
             "latency30_medium": "gripper latency 30 ms", "fastclose_medium": "B fast_close (NOT 2F-85)",
             "fastidealized_medium": "B fast_idealized (NOT 2F-85)"}
    axs[1].set_yticks(y, [names[c] for c in others], fontsize=8)
    axs[1].invert_yaxis()
    axs[1].set_xlim(0, 105)
    axs[1].set_xlabel("caught [%]")
    axs[1].legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2)
    axs[1].set_title("camera-only, medium noise: live vs offline", fontsize=10)
    cats = ["held", "fingertip", "late close", "slide-off", "missed intercept", "abort", "collision"]
    cols = ["#1b9e77", "#a6d8c9", "#e6ab02", "#d95f02", "#7570b3", "#999999", "#e7298a"]
    left = np.zeros(len(labels))
    for cat, col in zip(cats, cols):
        v = np.array([cells[c]["summary"]["outcomes"].get(cat, 0) / max(cells[c]["summary"]["n_valid"], 1) * 100 for c in labels])
        axs[2].barh(range(len(labels)), v, left=left, color=col, label=cat)
        left += v
    axs[2].set_yticks(range(len(labels)), labels, fontsize=7)
    axs[2].invert_yaxis()
    axs[2].set_xlabel("% of throws")
    axs[2].legend(fontsize=7, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=4)
    axs[2].set_title("failure taxonomy (ground truth)", fontsize=10)
    fig.suptitle("v2.1 live benchmark: every cell alone, one sim at a time (60 throws per cell); gripper_profile realistic "
                 "(2F-85 150 mm/s, 10 ms) unless the cell says fast_close / fast_idealized (NOT a real 2F-85)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(D, "phase2", "bench.png"), dpi=100)


if __name__ == "__main__":
    main()
