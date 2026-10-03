#!/usr/bin/env python3
"""When did B's fingers reach 40 mm relative to the cube's actual arrival? (recorded benchmark failures)

    python3 tools/v2_live/close_timing.py latency0_medium vision_medium log/p2/bench_v2.0/fastclose_medium ...
(system python, workspace sourced; a cell label = log/p2/bench/<label>, or a directory)
For each log/p2/bench/<cell>/trial_*.npz: a ballistic fit (known g) to the ground-truth free flight 150 ms before the
cube comes within 120 mm of B's TCP, extrapolated to its closest approach to the TCP (= the arrival if nothing had
touched it), vs the time the finger gap (gripper_model.GAP_TABLE) drops below 40 mm. B schedules 40 mm at the
predicted arrival + 10 ms + (true - assumed latency); the MuJoCo finger actuator lags its reference by kv/kp = 10 ms.
"""
import glob
import json
import os
import sys

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from catch_kinematics import UR5e
from catch_kinematics.transforms import inv_tf, make_tf
from catch_sim.gripper_model import gap_for_knuckle
from catch_sim.scene_config import robot_base_xyz
from catch_tasks.throw_eval import COLORS

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
G = np.array([0.0, 0.0, -9.81])
JN = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint")


def _syms():
    import itertools
    out = []
    for perm in itertools.permutations(range(3)):
        for sg in itertools.product((1, -1), repeat=3):
            M = np.zeros((3, 3))
            for i, j in enumerate(perm):
                M[i, j] = sg[i]
            if np.linalg.det(M) > 0:
                out.append(M)
    return out


SYMS = _syms()


def misalign_deg(R_tool, R_cube):
    return float(min(np.degrees(np.arccos(np.clip((np.trace(R_tool.T @ R_cube @ S) - 1) / 2, -1, 1))) for S in SYMS))


def robot_b():
    s = yaml.safe_load(open(os.path.join(get_package_share_directory("catch_description"), "config", "scene.yaml")))
    a = s["robots"]["b"]["base_yaw"]
    rz = np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])
    tcp = s["gripper"]["mount_offset"] + s["gripper"]["palm_insert"]["top_z"] + s["cube"]["edge"] / 2
    return UR5e(make_tf(rz, robot_base_xyz(s, "b")), make_tf(p=[0, 0, tcp]))


def timing(f, rb):
    z = np.load(f, allow_pickle=True)
    meta = json.loads(str(z["meta"]))
    names = list(z["names"])
    jb = [names.index(f"b_{j}") for j in JN]
    kk = names.index("b_robotiq_85_left_knuckle_joint")
    k = COLORS.index(meta["color"])
    tc, pc = z["cubes"][:, 0], z["cubes"][:, 1 + 7 * k:4 + 7 * k]
    idx = [min(np.searchsorted(z["t_js"], t), len(z["t_js"]) - 1) for t in tc]
    d = np.array([np.linalg.norm((inv_tf(rb.fk(z["q"][j, jb])) @ np.r_[p, 1])[:3]) for j, p in zip(idx, pc)])
    i_near = int(np.argmax(d < 0.12))
    w = (np.arange(len(tc)) < i_near) & (pc[:, 0] > -0.2) & (tc > tc[i_near] - 0.15)
    if w.sum() < 8:
        return None
    t0 = tc[w][0]
    tau = tc[w] - t0
    sol, *_ = np.linalg.lstsq(np.column_stack([np.ones_like(tau), tau]), pc[w] - 0.5 * np.outer(tau ** 2, G), rcond=None)
    p_tcp = rb.fk(z["q"][idx[int(np.argmin(d))], jb])[:3, 3]
    ts = np.arange(0.0, 0.5, 0.0002)
    path = sol[0] + np.outer(ts, sol[1]) + 0.5 * np.outer(ts ** 2, G)
    ia = int(np.argmin(np.linalg.norm(path - p_tcp, axis=1)))
    gap = np.array([gap_for_knuckle(q) for q in z["q"][:, kk]])
    i40 = int(np.argmax(gap < 40.0))
    t40 = z["t_js"][i40] if gap[i40] < 40.0 else np.nan
    # cube misalignment in the hand at the closest approach (smallest rotation between the cube's and the TCP's
    # axes over the cube's 24 symmetries; 0 = faces square to the pads) - same as fastclose_diag.py offline
    i_min = int(np.argmin(d))
    w, x, y, zq = z["cubes"][i_min, 4 + 7 * k:8 + 7 * k]
    Rc = np.array([[1 - 2 * (y * y + zq * zq), 2 * (x * y - zq * w), 2 * (x * zq + y * w)],
                   [2 * (x * y + zq * w), 1 - 2 * (x * x + zq * zq), 2 * (y * zq - x * w)],
                   [2 * (x * zq - y * w), 2 * (y * zq + x * w), 1 - 2 * (x * x + y * y)]])
    mis = misalign_deg(rb.fk(z["q"][idx[i_min], jb])[:3, :3], Rc)
    a_, b_ = np.argmax(gap < 54.0), np.argmax(gap < 42.0)
    speed = 12.0 / (z["t_js"][b_] - z["t_js"][a_]) if gap[b_] < 42.0 and b_ > a_ else np.nan
    return meta, 1000 * (t40 - (t0 + ts[ia])), 1000 * float(np.linalg.norm(path[ia] - p_tcp)), speed, mis


def main():
    rb = robot_b()
    for cell in sys.argv[1:]:
        d = cell if os.sep in cell else os.path.join(REPO, "log/p2/bench", cell)     # a cell label or a directory
        out = [r for f in sorted(glob.glob(os.path.join(d, "trial_*.npz")))
               if (r := timing(f, rb)) is not None]
        print(cell)
        for meta, dt, miss, sp, mis in out:
            print(f"   #{meta['i']:2d} {meta['outcome']:16s} gap 40 mm at arrival {dt:+6.1f} ms  (path passes {miss:4.1f} mm "
                  f"from the TCP; gap speed 54->42 mm {sp:5.0f} mm/s; cube misaligned {mis:4.1f} deg)")
        if out:
            print(f"   median {np.nanmedian([o[1] for o in out]):+.1f} ms, misalignment {np.nanmedian([o[4] for o in out]):.1f} deg")


if __name__ == "__main__":
    main()
