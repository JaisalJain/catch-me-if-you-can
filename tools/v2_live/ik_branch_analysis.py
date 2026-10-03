#!/usr/bin/env python3
"""How often does B change IK branch during a reactive catch? (recorded benchmark trials)

    PYTHONPATH=src/catch_kinematics:$PYTHONPATH python3 tools/v2_live/ik_branch_analysis.py \
        [--pose phase1 | --q-ready=Q1,..,Q6] [--out docs/v2/arm_config/branch_changes_v2.1.json] CELL_OR_DIR ...
(system python; a cell label = log/p2/bench/<label>, or a directory of trial_*.npz)
Branch = catch_kinematics.UR5e.branch(): the (shoulder, elbow, wrist) signs of the closed-form IK. For every
recording: B's branch at the start (ready pose) and at every /joint_states sample; a change = a sample whose branch
differs from the previous sample's (only possible through a singularity: q3 = 0, q5 = 0 or the shoulder's r = d4).
Also per trial: the largest excursion of each joint from the ready pose and whether wrist 2 crossed zero (flip).
"""
import argparse
import glob
import json
import os
import sys

import numpy as np
import yaml

from catch_kinematics import UR5e
from catch_kinematics.transforms import make_tf, rot_z

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
JN = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def robot_b():
    sc = yaml.safe_load(open(os.path.join(REPO, "src/catch_description/config/scene.yaml")))
    r = sc["robots"]["b"]
    base = [r["base_xy"][0], r["base_xy"][1], sc["table"]["top_z"] + r.get("pedestal_height", 0.0)]
    return UR5e(make_tf(rot_z(r["base_yaw"]), base))


def describe(robot, q):
    """Branch + the geometric reading of it: elbow height above the shoulder axis, elbow above the shoulder-wrist
    line, wrist 2 sign."""
    fr = robot.frames(q)
    s, e, w = fr[1][:3, 3], fr[2][:3, 3], fr[4][:3, 3]
    t = np.clip(np.dot(e - s, w - s) / np.dot(w - s, w - s), 0, 1)
    return dict(branch=list(robot.branch(q)), elbow_above_shoulder_m=round(float(e[2] - s[2]), 3),
                elbow_above_line_m=round(float(e[2] - (s + t * (w - s))[2]), 3), wrist2_sign=int(np.sign(np.sin(q[4]))))


def analyse(path, robot, q_ready):
    d = np.load(path, allow_pickle=True)
    names = list(d["names"])
    idx = [names.index(f"b_{j}") for j in JN]
    Q = d["q"][:, idx]
    br = [robot.branch(q) for q in Q]
    ch = [k for k in range(1, len(br)) if br[k] != br[k - 1]]
    meta = json.loads(str(d["meta"]))
    dev = np.abs(Q - q_ready)
    return dict(file=os.path.relpath(path, REPO), outcome=meta.get("outcome"), n=len(Q),
                start_branch=list(br[0]), start_at_ready=bool(np.max(dev[0]) < 0.02),
                n_changes=len(ch), changes=[dict(t=float(d["t_js"][k] - d["t_js"][0]), frm=list(br[k - 1]), to=list(br[k]),
                                                 q=Q[k].round(3).tolist()) for k in ch],
                branches_seen=sorted({tuple(b) for b in br}), max_dev=dev.max(axis=0).round(3).tolist(),
                wrist2_min_abs_sin=float(np.min(np.abs(np.sin(Q[:, 4])))),
                elbow_min_abs=float(np.min(np.abs(Q[:, 2]))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("cells", nargs="+")
    ap.add_argument("--pose", default="", help="reactive.yaml ready_poses entry (default: ready_pose)")
    ap.add_argument("--q-ready", default="", help="comma-separated joints instead (use --q-ready=...)")
    ap.add_argument("--out", default="")
    a = ap.parse_args()
    robot = robot_b()
    cfg = yaml.safe_load(open(os.path.join(REPO, "src/catch_catcher/config/reactive.yaml")))
    q_ready = (np.array([float(x) for x in a.q_ready.split(",")]) if a.q_ready
               else np.array(cfg["ready_poses"][a.pose or cfg["ready_pose"]]["q"]))
    out = dict(q_ready=q_ready.tolist(), ready=describe(robot, q_ready), cells={})
    print(f"ready pose {np.round(q_ready, 3).tolist()}: {out['ready']}")
    for c in a.cells:
        d = c if os.path.isdir(c) else os.path.join(REPO, "log/p2/bench", c)
        files = sorted(glob.glob(os.path.join(d, "trial_*.npz")))
        rows = [analyse(f, robot, q_ready) for f in files]
        n_ch = sum(r["n_changes"] > 0 for r in rows)
        md = np.max([r["max_dev"] for r in rows], axis=0) if rows else []
        summ = dict(n=len(rows), trials_with_change=n_ch, total_changes=sum(r["n_changes"] for r in rows),
                    start_at_ready=sum(r["start_at_ready"] for r in rows),
                    max_dev_over_trials=np.round(md, 3).tolist(),
                    median_max_dev=np.round(np.median([r["max_dev"] for r in rows], axis=0), 3).tolist() if rows else [],
                    min_wrist2_abs_sin=min((r["wrist2_min_abs_sin"] for r in rows), default=None),
                    branches_seen=sorted({b for r in rows for b in r["branches_seen"]}))
        out["cells"][os.path.basename(os.path.normpath(d))] = dict(summary=summ, trials=rows)
        print(f"{os.path.basename(os.path.normpath(d))}: {len(rows)} trials, {n_ch} with a branch change "
              f"({summ['total_changes']} changes), branches seen {summ['branches_seen']}; max |q - q_ready| "
              f"{summ['max_dev_over_trials']} (median per trial {summ['median_max_dev']}); min |sin q5| "
              f"{summ['min_wrist2_abs_sin']}")
    if a.out:
        os.makedirs(os.path.dirname(os.path.join(REPO, a.out)), exist_ok=True)
        json.dump(out, open(os.path.join(REPO, a.out), "w"), indent=1, default=lambda x: list(x))
        print("wrote", a.out)


if __name__ == "__main__":
    sys.exit(main())
