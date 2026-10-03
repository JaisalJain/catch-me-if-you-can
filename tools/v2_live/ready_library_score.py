#!/usr/bin/env python3
"""Ready-pose candidates scored on the throws A actually makes (the pinch library), with the live planner.

    PYTHONPATH=src/catch_catcher:src/catch_kinematics:src/catch_throw:src/catch_ballistics:$PYTHONPATH \
        python3 tools/v2_live/ready_library_score.py [--n 80] [--refine] [--out docs/v2/arm_config/library_score.json]
(system python + ROS sourced; numpy only.) Arm-configuration review: the envelope search (tools/v2_ready_envelope.py
--style home) scores poses on a broad prior throw family; the library entries were selected as catchable from the
Phase 1 pose and arrive low (median intercept z ~1.1 m), so a pose that is good for the family can miss them all.
Candidates: TCP grid x 0.20-0.55, y 0-0.30, z 1.05-1.50 (5 cm), approach 30-75 deg towards A; IK on B home's branch
(1, 1, -1) (all joints in [-pi, pi)); margins >= 0.15 rad, |sin q5| >= 0.5, capsule clearance >= 50 mm to table /
pedestals / pallets, >= min_clearance (self, arm A at home). Score = fraction of n library entries (nominal release) for which catch_catcher.reactive.ReactivePlanner
(ik_branch fixed) finds a feasible intercept when B commits t_commit after the release from rest at the ready pose.
"""
import argparse
import json
import multiprocessing as mp
import os

import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
W = {}


def robots():
    from catch_kinematics import UR5e
    from catch_kinematics.transforms import make_tf, rot_z
    sc = yaml.safe_load(open(os.path.join(REPO, "src/catch_description/config/scene.yaml")))

    def mk(k):
        r = sc["robots"][k]
        base = [r["base_xy"][0], r["base_xy"][1], sc["table"]["top_z"] + r.get("pedestal_height", 0.0)]
        tcp = sc["gripper"]["mount_offset"] + sc["gripper"]["palm_insert"]["top_z"] + sc["cube"]["edge"] / 2
        return UR5e(make_tf(rot_z(r["base_yaw"]), base), make_tf(p=[0, 0, tcp]))
    return mk("b"), mk("a")


def _init(n, t_commit):
    from catch_kinematics.collision import CapsuleModel
    from catch_kinematics.dynamics import ArmDynamics
    from catch_catcher.reactive import resolve_config
    W["B"], W["A"] = robots()
    W["caps"], W["dyn"] = CapsuleModel(), ArmDynamics()
    W["raw"] = yaml.safe_load(open(os.path.join(REPO, "src/catch_catcher/config/reactive.yaml")))
    W["cfg"] = resolve_config(W["raw"], env={"CATCH_IK_BRANCH": "fixed"})
    poses = yaml.safe_load(open(os.path.join(REPO, "src/catch_description/config/poses.yaml")))
    W["qa"], W["home"] = np.array(poses["a"]["home"]), np.array(poses["b"]["home"])
    lib = json.load(open(os.path.join(REPO, "src/catch_throw/config/library_pinch.json")))["entries"]
    lib = [e for e in lib if e.get("moveit_start_valid", True)]
    idx = np.random.default_rng(0).choice(len(lib), size=min(n, len(lib)), replace=False)
    W["throws"] = [((e.get("params") or e)["p_release"], (e.get("params") or e)["v_release"]) for e in (lib[i] for i in idx)]
    W["t_commit"] = t_commit


def elbow_rise(B, q):
    fr = B.frames(q)
    return float(fr[2][2, 3] - fr[1][2, 3])


def score(args):
    p, elev, branch = args
    from catch_catcher.reactive import ReactivePlanner, ballistic, ready_rotation
    from catch_kinematics import POS_LIMIT
    from catch_kinematics.transforms import make_tf
    B, A, caps = W["B"], W["A"], W["caps"]
    sols = [s for s, b in B.ik_all(make_tf(ready_rotation(elev), p), branches=True) if b == tuple(branch)]
    if not sols:
        return None
    q = np.arctan2(np.sin(sols[0]), np.cos(sols[0]))
    if np.min(POS_LIMIT - np.abs(q)) < 0.15 or abs(np.sin(q[4])) < 0.5:
        return None
    A0, A1 = caps.segments(A, W["qa"])
    if (caps.clearance(B, q[None], env=True, self_check=False) < 0.05
            or caps.clearance(B, q[None], env=False, self_check=True, extra_capsules=(A0[0], A1[0], caps.r)) < caps.min_clearance):
        return None
    pl = ReactivePlanner(B, W["dyn"], caps, W["cfg"], q, elev, A)
    ok = 0
    for p0, v0 in W["throws"]:
        pred = ballistic(p0, v0, 0.0)
        t0 = W["t_commit"]
        plan, _ = pl.search(t0, (q, np.zeros(6), np.zeros(6)), pred, t0, "pinch", W["qa"])
        ok += plan is not None
    return dict(p=list(p), elev=elev, q=q.tolist(), elbow_rise=elbow_rise(B, q), score=ok / len(W["throws"]),
                wrist_sin=float(abs(np.sin(q[4]))), margin=float(np.min(POS_LIMIT - np.abs(q))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=80)
    ap.add_argument("--t-commit", type=float, default=0.15, help="s after the release (live: ~0.12 s + 30 ms lead)")
    ap.add_argument("--out", default="docs/v2/arm_config/library_score.json")
    ap.add_argument("--refine", action="store_true", help="finer grid around the coarse run's best elbow-up pose")
    a = ap.parse_args()
    _init(a.n, a.t_commit)
    branch = W["B"].branch(W["home"])
    if a.refine:        # finer grid around the best pose with elbow >= 0.15 m and approach >= 60 deg (coarse run)
        c0 = json.load(open(os.path.join(REPO, a.out)))
        s = [r for r in c0["candidates"] if r["elbow_rise"] >= 0.15 and r["elev"] >= 60]
        b0 = max(s, key=lambda r: r["score"])
        grid = [((x, y, z), e, branch) for x in b0["p"][0] + np.arange(-0.05, 0.051, 0.025)
                for y in b0["p"][1] + np.arange(-0.05, 0.051, 0.025) for z in b0["p"][2] + np.arange(-0.05, 0.051, 0.025)
                for e in (55, 60, 65, 70)]
        a.out = a.out.replace(".json", "_refined.json")
    else:
        grid = [((x, y, z), e, branch) for x in np.arange(0.20, 0.551, 0.05) for y in np.arange(0.0, 0.301, 0.05)
                for z in np.arange(1.05, 1.501, 0.05) for e in (30, 45, 60, 75)]
    with mp.Pool(min(22, os.cpu_count()), initializer=_init, initargs=(a.n, a.t_commit)) as pool:
        res = [r for r in pool.map(score, grid, chunksize=4) if r is not None]
    # reference: the Phase 1 pose, same scoring, free branch (= the v2.2 planner)
    from catch_catcher.reactive import ReactivePlanner, ballistic, resolve_config
    cfg = resolve_config(W["raw"], env={"CATCH_READY_POSE": "phase1", "CATCH_IK_BRANCH": "free"})
    q1 = np.array(cfg["ready"]["q"])
    pl = ReactivePlanner(W["B"], W["dyn"], W["caps"], cfg, q1, cfg["ready"]["elev_deg"], W["A"])
    ref = np.mean([pl.search(a.t_commit, (q1, np.zeros(6), np.zeros(6)), ballistic(p0, v0, 0.0), a.t_commit, "pinch",
                             W["qa"])[0] is not None for p0, v0 in W["throws"]])
    out = dict(n_throws=len(W["throws"]), t_commit=a.t_commit, branch=list(branch), phase1_score=float(ref),
               n_candidates=len(grid), n_valid=len(res), candidates=res, best_by_rise={})
    print(f"{len(res)} valid of {len(grid)} candidates; Phase 1 pose (free branch): {ref:.2f} of {len(W['throws'])} throws")
    for th in (0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30):
        s = [r for r in res if r["elbow_rise"] >= th]
        if s:
            b = max(s, key=lambda r: (round(r["score"], 3), r["elbow_rise"]))
            out["best_by_rise"][str(th)] = b
            s60 = [r for r in s if r["elev"] >= 60]
            if s60:
                b6 = max(s60, key=lambda r: (round(r["score"], 3), r["elbow_rise"]))
                out["best_by_rise"][f"{th}_elev60"] = b6
                print(f"  approach >= 60 deg: best {b6['score']:.2f} at TCP {np.round(b6['p'], 3).tolist()} elev {b6['elev']} "
                      f"(elbow {b6['elbow_rise']:.3f} m, q {np.round(b6['q'], 4).tolist()})")
            print(f"elbow >= {th:.2f} m above the shoulder: best {b['score']:.2f} at TCP {np.round(b['p'], 2).tolist()} "
                  f"elev {b['elev']} (elbow {b['elbow_rise']:.3f} m, q {np.round(b['q'], 3).tolist()})")
    json.dump(out, open(os.path.join(REPO, a.out), "w"), indent=1)


if __name__ == "__main__":
    main()
