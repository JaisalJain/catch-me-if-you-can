#!/usr/bin/env python3
"""M1 plots: joint profiles of the chosen throw, cube path (side view), feasible region.

    env -u PYTHONPATH .venv/bin/python tools/plot_feasibility.py
Reads catch_throw/config/throw.yaml + docs/feasibility/results.json; writes docs/feasibility/*.png and
docs/feasibility/chosen_throw.json.
"""
import json
import os
import sys

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "tools"))
import feasibility as F  # noqa: E402
from catch_ballistics import velocity_for_launch_angle  # noqa: E402
from catch_kinematics import POS_LIMIT, TORQUE_LIMIT, VEL_LIMIT  # noqa: E402
from catch_throw.generator import generate_throw  # noqa: E402
from catch_throw.profile import ProfileParams  # noqa: E402

OUT = os.path.join(REPO, "docs", "feasibility")
# reference palette (dataviz skill), light mode
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e4e3df", "#fcfcfb"
S1, S2, S3 = "#2a78d6", "#eb6834", "#1baf7a"
plt.rcParams.update({"font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK2, "xtick.color": INK2,
                     "ytick.color": INK2, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.6,
                     "axes.spines.top": False, "axes.spines.right": False, "figure.facecolor": SURF,
                     "axes.facecolor": SURF, "lines.linewidth": 2.0, "savefig.dpi": 140})
JL = ["shoulder pan", "shoulder lift", "elbow", "wrist 1", "wrist 2", "wrist 3"]


def chosen_plan():
    cfg = yaml.safe_load(open(os.path.join(REPO, "src/catch_throw/config/throw.yaml")))["throw"]
    scene, phys, robots = F.load()
    p, c = np.array(cfg["release_point"]), np.array(cfg["catch_point"])
    v, T = velocity_for_launch_angle(p, c, np.radians(cfg["launch_angle_deg"]))
    prof = ProfileParams(**cfg["profile"])
    plan = generate_throw(robots["a"], p, v, cfg["q_release_seed"], prof, finger_axis=cfg["finger_axis"],
                          pitch_rate=cfg["pitch_rate"])
    tau = F.Dynamics(scene, phys).torques(plan.q, plan.qd, plan.qdd)
    return scene, cfg, plan, tau, c, T


def plot_joints(plan, tau):
    cols = [("position [rad]", plan.q, POS_LIMIT), ("velocity [rad/s]", plan.qd, VEL_LIMIT),
            ("acceleration [rad/s²]", plan.qdd, None), ("torque [N·m]", tau, TORQUE_LIMIT)]
    fig, axs = plt.subplots(6, 4, figsize=(13, 11), sharex=True)
    for j in range(6):
        for k, (lab, data, lim) in enumerate(cols):
            ax = axs[j, k]
            ax.plot(plan.t, data[:, j], color=S1)
            if lim is not None and k > 0:
                for sgn in (1, -1):
                    ax.axhline(sgn * lim[j], color=INK2, lw=1, ls="--")
                    ax.axhline(sgn * lim[j] * (0.85 if k == 1 else 0.80), color=INK2, lw=0.8, ls=":")
            ax.axvline(plan.t_release, color=S2, lw=1)
            if j == 0:
                ax.set_title(lab, color=INK, fontsize=10)
            if k == 0:
                ax.set_ylabel(JL[j], color=INK)
            if j == 5:
                ax.set_xlabel("time from stroke start [s]")
            if k in (1, 3):
                ax.set_ylim(-1.1 * lim[j], 1.1 * lim[j])
    fig.suptitle("Chosen throw: joint profiles of robot A (orange line = cube separation; dashed = UR5e limit, "
                 "dotted = design margin 85 % vel / 80 % torque)", color=INK, fontsize=11)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "throw_joint_profiles.png"))
    plt.close(fig)


def plot_path(scene, plan, c, T):
    fl = plan.flight()
    tf = np.linspace(plan.t_release, plan.t_release + T + 0.05, 200)
    pf = fl.position(tf)
    fig, ax = plt.subplots(figsize=(10, 5.2))
    top = scene["table"]["top_z"]
    ax.fill_between([-1.0, 1.0], top - 0.04, top, color="#c9b49a", lw=0)
    for k in "ab":
        bx = scene["robots"][k]["base_xy"][0]
        ph = scene["robots"][k]["pedestal_height"]
        ax.fill_between([bx - 0.09, bx + 0.09], top, top + ph, color="#b9bbbe", lw=0)
        ax.text(bx, top + ph + 0.02, f"robot {k.upper()} base", ha="center", color=INK2)
    ax.plot(plan.tcp[:, 0], plan.tcp[:, 2], color=S1, label="TCP stroke (cube in palm)")
    ax.plot(pf[:, 0], pf[:, 2], color=S2, label="cube flight (ballistic)")
    ax.plot(*plan.p_release[[0, 2]], "o", ms=8, color=S2, mec=SURF, mew=2)
    ax.plot(c[0], c[2], "s", ms=8, color=S3, mec=SURF, mew=2, label="catch point")
    ax.annotate(f"release  |v|={np.linalg.norm(plan.v_release):.2f} m/s @ {np.degrees(np.arctan2(plan.v_release[2], plan.v_release[0])):.0f}°",
                plan.p_release[[0, 2]], xytext=(-150, 25), textcoords="offset points", color=INK,
                arrowprops=dict(arrowstyle="-", color=INK2))
    ap = fl.apex
    ax.annotate(f"apex z={ap[2]:.2f} m", ap[[0, 2]], xytext=(10, 10), textcoords="offset points", color=INK)
    ax.annotate(f"catch after {T:.3f} s", c[[0, 2]], xytext=(10, -20), textcoords="offset points", color=INK)
    ax.set_aspect("equal")
    ax.set_xlim(-1.0, 1.0)
    ax.set_ylim(0.7, 1.55)
    ax.set_xlabel("x [m] (A → B)")
    ax.set_ylabel("z [m]")
    ax.set_title(f"Side view of the chosen throw (plane y = {plan.p_release[1]:.2f} m)", color=INK)
    ax.legend(loc="upper left", frameon=False)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "cube_path.png"))
    plt.close(fig)


def plot_region():
    d = json.load(open(os.path.join(OUT, "summary.json")))
    cells = d["cells"]
    risers = sorted({c["riser"] for c in cells})
    flights = sorted({c["flight"] for c in cells if 0.25 <= c["flight"] <= 0.75})
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(12, 4.4))
    best = [min((c["best_release_vel_ratio"] for c in cells if c["flight"] == f and c["best_release_vel_ratio"]),
                default=np.nan) for f in flights]
    a1.plot(flights, best, "-o", color=S1, ms=6, mec=SURF, mew=2)
    a1.axhline(0.85, color=INK2, ls=":", lw=1)
    a1.axhline(1.0, color=INK2, ls="--", lw=1)
    a1.text(flights[0], 0.86, "design margin 85 %", color=INK2, va="bottom")
    a1.text(flights[0], 1.01, "UR5e limit 180°/s", color=INK2, va="bottom")
    a1.set_xlabel("flight distance |x_catch − x_release| [m]")
    a1.set_ylabel("best peak joint speed at release / limit")
    a1.set_title("Joint-speed demand vs flight distance\n(best over angle, height, plane, IK branch, palm pitch)",
                 color=INK)
    M = np.zeros((len(risers), len(flights)))
    for c in cells:
        if c["flight"] in flights:
            M[risers.index(c["riser"]), flights.index(c["flight"])] = c["n_feasible"]
    cmap = matplotlib.colors.LinearSegmentedColormap.from_list("seq", ["#f1f6fd", "#9cc2ee", "#2a78d6", "#123f75"])
    im = a2.imshow(M, cmap=cmap, aspect="auto", origin="lower")
    for i in range(len(risers)):
        for j in range(len(flights)):
            a2.text(j, i, f"{int(M[i, j])}", ha="center", va="center",
                    color=SURF if M[i, j] > M.max() * 0.5 else INK, fontsize=10)
    a2.set_xticks(range(len(flights)), [f"{f:.1f}" for f in flights])
    a2.set_yticks(range(len(risers)), [f"{r:.2f}" for r in risers])
    a2.grid(False)
    a2.set_xlabel("flight distance [m]")
    a2.set_ylabel("robot pedestal height [m]")
    a2.set_title("Feasible candidates (all checks, with margins)", color=INK)
    fig.colorbar(im, ax=a2, label="count")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "feasible_region.png"))
    plt.close(fig)


def main():
    scene, cfg, plan, tau, c, T = chosen_plan()
    plot_joints(plan, tau)
    plot_path(scene, plan, c, T)
    plot_region()
    m = plan.metrics
    summary = dict(release_point=plan.p_release.tolist(), release_velocity=plan.v_release.tolist(),
                   speed=float(np.linalg.norm(plan.v_release)), spin=plan.spin.tolist(), flight_time=T,
                   apex=plan.flight().apex.tolist(), t_release_in_stroke=plan.t_release,
                   stroke_length=m["stroke_length"], stroke_duration=m["duration"],
                   vel_ratio=m["vel_ratio"], vel_ratio_max=m["vel_ratio_max"],
                   pos_margin=m["pos_margin"], acc_max=m["acc_max"],
                   torque_ratio=(np.max(np.abs(tau), axis=0) / TORQUE_LIMIT).tolist(),
                   q_start=plan.q_start.tolist(), q_end=plan.q_end.tolist())
    json.dump(summary, open(os.path.join(OUT, "chosen_throw.json"), "w"), indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == "__main__":
    main()
