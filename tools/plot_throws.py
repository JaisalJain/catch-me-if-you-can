#!/usr/bin/env python3
"""M4 plots from docs/m4/throws_20.json: side view of all measured cube flights vs the plan, and where each
cube crossed the catch height relative to the catch point.
    env -u PYTHONPATH .venv/bin/python tools/plot_throws.py
"""
import json
import os

import matplotlib
import numpy as np

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "figure.facecolor": SURF,
                     "axes.facecolor": SURF, "savefig.dpi": 140})


def main():
    d = json.load(open(os.path.join(REPO, "docs/m4/throws_20.json")))
    plan = d["plan"]
    p0, v0 = np.array(plan["p_release"]), np.array(plan["v_release"])
    c = np.array(plan["catch_point"])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.6), gridspec_kw={"width_ratios": [1.7, 1]})
    for k, r in enumerate(d["throws"]):
        if not r.get("ok"):
            continue
        p = np.array(r["traj"]["p"])
        a1.plot(p[:, 0], p[:, 2], color=S1, lw=0.8, alpha=0.5, label="measured cube centre (20 throws)" if k == 0 else None)
    t = np.linspace(0, plan["flight_time"] + 0.12, 200)
    pp = p0 + np.outer(t, v0) + 0.5 * np.outer(t ** 2, [0, 0, -9.81])
    a1.plot(pp[:, 0], pp[:, 2], color=S2, lw=2, ls="--", label="planned flight")
    a1.plot(*p0[[0, 2]], "o", color=S2, ms=7, mec=SURF, mew=2)
    a1.plot(c[0], c[2], "s", color=S3, ms=8, mec=SURF, mew=2, label="catch point")
    a1.set_xlim(-0.25, 0.55)
    a1.set_ylim(0.75, 1.5)
    a1.set_aspect("equal")
    a1.set_xlabel("x [m]")
    a1.set_ylabel("z [m]")
    a1.set_title("Side view: stroke + flight of 20 throws vs plan (nothing catching)", color=INK)
    a1.legend(loc="lower left", frameon=False)
    e = np.array([r["catch_height_cross_err_mm"] for r in d["throws"] if r.get("ok")])
    a2.axhline(0, color=INK2, lw=0.8)
    a2.axvline(0, color=INK2, lw=0.8)
    a2.scatter(e[:, 0], e[:, 1], s=40, color=S1, edgecolor=SURF, linewidth=1.5, zorder=3)
    lim = max(10, np.abs(e[:, :2]).max() * 1.4)
    a2.set_xlim(-lim, lim)
    a2.set_ylim(-lim, lim)
    a2.set_aspect("equal")
    a2.set_xlabel("x error [mm]")
    a2.set_ylabel("y error [mm]")
    s = d["summary"]["catch_height_cross_err_mm"]
    a2.set_title(f"Catch-height crossing (z = {c[2]:.2f} m)\nmean ({s['mean'][0]:.1f}, {s['mean'][1]:.1f}) mm, "
                 f"std ({s['std'][0]:.1f}, {s['std'][1]:.1f}) mm", color=INK)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
    out = os.path.join(REPO, "docs/m4/throws.png")
    fig.savefig(out)
    print("wrote", out)


if __name__ == "__main__":
    main()
