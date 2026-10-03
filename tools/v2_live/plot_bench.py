#!/usr/bin/env python3
"""v2 step 6: benchmark table + plots from docs/v2/phase2/bench/*.json (live) and the offline counterparts
(docs/v2/catch_livematch*.json, catch_corrections*.json).

    env -u PYTHONPATH .venv/bin/python tools/v2_live/plot_bench.py
Outputs: docs/v2/phase2/bench_table.md, docs/v2/phase2/bench.png
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

ORDER = ["vision_none", "vision_low", "vision_medium", "vision_medium_solo", "vision_high", "plan_none", "plan_low", "plan_medium", "plan_high",
         "cup_medium", "cup_medium_solo", "cam60_medium", "latency0_medium", "latency0_medium_solo", "latency20_medium", "latency30_medium", "fastclose_medium",
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
        "fastclose_medium": "B's gripper fast_close (NOT a 2F-85; 10x speed, realistic force), medium (ran alone)",
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
    if cell not in OFFLINE:
        return None
    f, k = OFFLINE[cell]
    p = os.path.join(D, f)
    if not os.path.exists(p):
        return None
    rows = json.load(open(p))["rows"]
    return rows[k]["summary"] if k in rows else None


def main():
    cells = {os.path.basename(f)[:-5]: json.load(open(f)) for f in glob.glob(os.path.join(B, "*.json"))}
    labels = [c for c in ORDER if c in cells]
    lines = ["| cell | n | caught | rate [95 % CI] | offline (same config) | outcomes | trigger | commit after 1st fix | "
             "replans | realised release error (speed sd, direction rms, time sd; A's execution error median) |",
             "|---" * 10 + "|"]
    for c in labels:
        s = cells[c]["summary"]
        off = offline(c)
        rr = realised(cells[c]["trials"], "cup" if c.startswith("cup") else "pinch")
        oc = ", ".join(f"{k} {v}" for k, v in sorted(s["outcomes"].items(), key=lambda x: -x[1]))
        tg = ", ".join(f"{k} {v}" for k, v in s["triggers"].items() if k)
        lines.append(f"| {DESC.get(c, c)} | {s['n_valid']} | {s['held']} | **{100 * s['rate']:.0f} %** "
                     f"[{100 * s['ci95'][0]:.0f}–{100 * s['ci95'][1]:.0f}] | "
                     + (f"{100 * off['rate']:.0f} % [{100 * off['ci95'][0]:.0f}–{100 * off['ci95'][1]:.0f}]" if off else "—")
                     + f" | {oc} | {tg} | "
                     + (f"before the throw (A's plan) | — | " if c.startswith("plan") else
                        f"{s.get('t_commit_ms', '—'):.0f} ms | {s.get('replan_rate_hz', '—')} Hz | ")
                     + (f"{100 * rr['speed'].std():.2f} % (mean {100 * rr['speed'].mean():+.2f}), "
                        f"{np.sqrt(np.mean(rr['dir'] ** 2)):.2f}°, {rr['dt'].std():.1f} ms; A's own error "
                        f"{np.median(rr['exec']):.0f} mm/s" if len(rr["speed"]) else "—") + " |")
    open(os.path.join(D, "phase2", "bench_table.md"), "w").write("\n".join(lines) + "\n")
    print("\n".join(lines))
    fig, axs = plt.subplots(1, 3, figsize=(19, 6.5))
    lv = ["none", "low", "medium", "high"]
    for src, col in (("vision", "#d95f02"), ("plan", "#7570b3")):
        xs, r, lo, hi = [], [], [], []
        for k, n in enumerate(lv):
            c = f"{src}_{n}"
            if c in cells:
                s = cells[c]["summary"]
                xs.append(k)
                r.append(100 * s["rate"])
                lo.append(100 * s["ci95"][0])
                hi.append(100 * s["ci95"][1])
        if xs:
            axs[0].errorbar(xs, r, yerr=[np.array(r) - lo, np.array(hi) - r], fmt="o-", capsize=4, color=col,
                            label=f"{src}-only" if src == "vision" else "plan-informed (A's published plan)")
    axs[0].set_xticks(range(4), [f"{n}" for n in lv])
    axs[0].set_xlabel("release noise level (catch_throw/config/v2.yaml)")
    axs[0].set_ylabel("caught [%] (95 % Wilson CI)")
    axs[0].set_ylim(-3, 103)
    axs[0].grid(alpha=0.3)
    axs[0].legend()
    axs[0].set_title("vision-only vs plan-informed, pinch, realistic gripper", fontsize=10)
    others = [c for c in ("vision_medium", "vision_medium_solo", "cup_medium", "cup_medium_solo", "cam60_medium",
                          "latency0_medium", "latency0_medium_solo", "latency20_medium",
                          "latency30_medium", "fastclose_medium", "fastidealized_medium") if c in cells]
    y = np.arange(len(others))
    rl = [100 * cells[c]["summary"]["rate"] for c in others]
    ci = np.array([[100 * (cells[c]["summary"]["rate"] - cells[c]["summary"]["ci95"][0]),
                    100 * (cells[c]["summary"]["ci95"][1] - cells[c]["summary"]["rate"])] for c in others]).T
    axs[1].barh(y - 0.18, rl, 0.36, xerr=ci, capsize=3, color="#d95f02", label="live")
    ro = [100 * offline(c)["rate"] if offline(c) else np.nan for c in others]
    axs[1].barh(y + 0.18, ro, 0.36, color="#1b9e77", label="offline, same config (measured live perception)")
    names = {"vision_medium": "pinch (default)", "vision_medium_solo": "pinch, sim alone", "cup_medium": "pre-closed cup", "cup_medium_solo": "pre-closed cup, sim alone",
             "latency0_medium_solo": "gripper latency 0 ms, sim alone", "cam60_medium": "64 Hz cameras",
             "latency0_medium": "gripper latency 0 ms", "latency20_medium": "gripper latency 20 ms",
             "latency30_medium": "gripper latency 30 ms", "fastclose_medium": "B fast_close (NOT 2F-85)",
             "fastidealized_medium": "B fast_idealized (NOT 2F-85)"}
    axs[1].set_yticks(y, [names[c] for c in others], fontsize=8)
    axs[1].invert_yaxis()
    axs[1].set_xlim(0, 105)
    axs[1].set_xlabel("caught [%]")
    axs[1].legend(fontsize=8, loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=2)
    axs[1].set_title("vision-only, medium noise: live vs offline", fontsize=10)
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
    fig.suptitle("v2 live benchmark (ROS pipeline, 60 throws per cell unless noted); gripper_profile realistic "
                 "(2F-85 150 mm/s, 10 ms) unless the cell says fast_close / fast_idealized (NOT a real 2F-85)", fontsize=10)
    fig.tight_layout()
    fig.savefig(os.path.join(D, "phase2", "bench.png"), dpi=100)


if __name__ == "__main__":
    main()
