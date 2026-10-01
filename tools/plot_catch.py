#!/usr/bin/env python3
"""M5 plots: catch envelope (offline, tools/catch_study.py --envelope) with the release-velocity errors of the
recorded trials (ballistic fit to the ground truth), place errors and in-hand estimate errors.

    env -u PYTHONPATH .venv/bin/python tools/plot_catch.py docs/m5/catch_30.json log/m5/force --out docs/m5/catch.png
"""
import argparse
import glob
import json
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
G = np.array([0, 0, -9.81])
COLORS = ("red", "green", "blue")


def planned_release():
    import sys
    for p in ("src/catch_ballistics",):
        sys.path.insert(0, os.path.join(REPO, p))
    from catch_ballistics import velocity_for_launch_angle
    cfg = yaml.safe_load(open(os.path.join(REPO, "src/catch_throw/config/throw.yaml")))["throw"]
    p, c = np.array(cfg["release_point"]), np.array(cfg["catch_point"])
    v, _ = velocity_for_launch_angle(p, c, np.radians(cfg["launch_angle_deg"]))
    return p, v


def release_velocity(npz, color, x_release):
    """Ballistic least-squares fit (known g) to the free flight between A and B; velocity at x = x_release."""
    z = np.load(npz, allow_pickle=True)
    c = z["cubes"]
    k = COLORS.index(color)
    t, p = c[:, 0], c[:, 1 + 7 * k:4 + 7 * k]
    w = (p[:, 0] > 0.0) & (p[:, 0] < 0.25) & (p[:, 2] > 1.0)
    if w.sum() < 10:
        return None
    t0 = t[w][0]
    tau = t[w] - t0
    A = np.column_stack([np.ones_like(tau), tau])
    sol = np.linalg.lstsq(A, p[w] - 0.5 * np.outer(tau ** 2, G), rcond=None)[0]
    p0, v0 = sol
    tr = (x_release - p0[0]) / v0[0]
    return v0 + G * tr


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("result")
    ap.add_argument("recdir")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    res = json.load(open(a.result))
    env = json.load(open(os.path.join(REPO, "docs/m5/catch_envelope.json")))
    p_r, v_r = planned_release()
    dv = []
    for r in res["trials"]:
        f = os.path.join(a.recdir, f"trial_{r['i']:02d}.npz")
        if os.path.exists(f):
            v = release_velocity(f, r["color"], p_r[0])
            if v is not None:
                dv.append((v - v_r, r.get("held", False)))
    fig, ax = plt.subplots(1, 3, figsize=(16, 5))
    g = np.array(env["dv"])
    held = np.array(env["held"])
    step = g[1] - g[0]
    ax[0].imshow(held, origin="lower", extent=[g[0] - step / 2, g[-1] + step / 2, g[0] - step / 2, g[-1] + step / 2],
                 cmap="Greens", vmin=-0.3, vmax=1.3, alpha=0.8)
    for d, h in dv:
        ax[0].plot(d[0], d[2], "o" if h else "x", color="k" if h else "r", ms=5)
    ax[0].set_xlabel("release velocity error x [m/s]")
    ax[0].set_ylabel("release velocity error z [m/s]")
    ax[0].set_title("catch envelope (green: held, offline sim)\n"
                    f"dots: the {len(dv)} trials (fit to ground truth)")
    ax[0].axhline(0, lw=0.5, color="grey")
    ax[0].axvline(0, lw=0.5, color="grey")
    tr = [r for r in res["trials"] if "place_err_mm" in r]
    for c in COLORS:
        e = np.array([r["place_err_mm"] for r in tr if r["color"] == c])
        if len(e):
            ax[1].plot(e[:, 0], e[:, 1], "o", color=c, label=c)
    lim = (52 - 40) / 2
    ax[1].add_patch(plt.Rectangle((-lim, -lim), 2 * lim, 2 * lim, fill=False, ls="--", color="grey"))
    ax[1].set_xlim(-8, 8)
    ax[1].set_ylim(-8, 8)
    ax[1].set_aspect("equal")
    ax[1].set_xlabel("place error x [mm]")
    ax[1].set_ylabel("place error y [mm]")
    yaw = [r["place_yaw_deg"] for r in tr]
    ax[1].set_title(f"cube centre vs slot centre (dashed: slot clearance)\n"
                    f"yaw error max {np.max(np.abs(yaw)) if yaw else float('nan'):.2f} deg, n = {len(tr)}")
    ax[1].legend()
    est = [r["inhand_est"] for r in res["trials"] if "inhand_est" in r and "angle_err_deg" in r["inhand_est"]]
    if est:
        ang = np.array([e["angle_err_deg"] for e in est])
        yz = np.array([e["yz_err_mm"] for e in est])
        ax[2].plot(np.arange(len(ang)), ang, "o-", label="angle error [deg]")
        ax[2].plot(np.arange(len(ang)), yz[:, 0], "s", label="y error [mm]")
        ax[2].plot(np.arange(len(ang)), yz[:, 1], "^", label="z error [mm]")
        ax[2].axhline(0, lw=0.5, color="grey")
        ax[2].set_xlabel("trial")
        ax[2].set_title("in-hand pose: wrist camera estimate - ground truth")
        ax[2].legend()
    fig.tight_layout()
    fig.savefig(a.out, dpi=110)
    if dv:
        d = np.array([x for x, _ in dv])
        print(f"release velocity error over {len(d)} trials: mean {np.round(d.mean(0), 4).tolist()} "
              f"std {np.round(d.std(0), 4).tolist()} m/s")
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
