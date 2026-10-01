#!/usr/bin/env python3
"""Offline check of top-down grasp geometry on the pallet (real MuJoCo 2F-85 + slot borders):
places arm A by IK at the grasp pose for several TCP offsets, closes the gripper physically, and reports
the final knuckle angle and any finger contact with the pallet. Also checks IK/joint margins/clearance of
both arms over all slots for candidate pallet centres.
    env -u PYTHONPATH .venv/bin/python tools/pick_geometry.py
"""
import copy
import os
import sys

import mujoco
import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "tools"))
import feasibility as F  # noqa: E402
import build_mjcf  # noqa: E402
from catch_kinematics import POS_LIMIT  # noqa: E402
from catch_kinematics.transforms import make_tf  # noqa: E402

R_DOWN = np.column_stack([[0, 1, 0], [1, 0, 0], [0, 0, -1]])


def slot_pos(scene, pallet, color):
    p = scene["pallets"]
    i = p["slot_order"].index(color)
    cx, cy = p[pallet]["center"]
    return np.array([cx + (i - 1) * p["slot_pitch"], cy, scene["table"]["top_z"] + p["base_size"][2] + scene["cube"]["edge"] / 2])


def grasp_trial(scene, phys, offset, color="green"):
    m = build_mjcf.compile_model(scene, phys)
    d = mujoco.MjData(m)
    robots = F.make_robots(scene)
    target = slot_pos(scene, "a", color) + [0, 0, offset]
    q = robots["a"].ik_closest(make_tf(R_DOWN, target), [0, -1.0, 2.0, -2.5, -1.57, 0])
    qa = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"a_{j}")] for j in F.JN]
    va = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"a_{j}")] for j in F.JN]
    aid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"a_{j}_motor") for j in F.JN]
    g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "a_robotiq_85_left_knuckle_joint")
    gq = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "a_robotiq_85_left_knuckle_joint")]
    d.qpos[qa] = q
    cube = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"cube_{color}")
    z0 = None
    hits = set()
    for i in range(1500):
        d.ctrl[aid] = 6000 * (q - d.qpos[qa]) - 200 * d.qvel[va]
        d.ctrl[g] = 0.70 if i > 100 else 0.0
        mujoco.mj_step(m, d)
        if i == 100:
            z0 = d.xpos[cube][2]
        for k in range(d.ncon):
            c = d.contact[k]
            b = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[x]) or "" for x in (c.geom1, c.geom2)]
            names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, x) or "" for x in (c.geom1, c.geom2)]
            if any(x.startswith("a_robotiq") for x in b) and any(n.startswith(("pallet", "table")) or bb == "pallet_a"
                                                                  for n, bb in zip(names, b)):
                hits.add(tuple(sorted(b)))
    # lift 5 cm kinematically via IK and check the cube comes along
    q_up = robots["a"].ik_closest(make_tf(R_DOWN, target + [0, 0, 0.05]), q)
    for i in range(1500):
        a_ = min(1.0, i / 1000)
        qq = q + a_ * (q_up - q)
        d.ctrl[aid] = 6000 * (qq - d.qpos[qa]) - 200 * d.qvel[va]
        d.ctrl[g] = 0.70
        mujoco.mj_step(m, d)
    return d.qpos[gq], sorted(hits), 1000 * (d.xpos[cube][2] - z0)


def reach(scene, centers):
    robots = F.make_robots(scene)
    for pal, k in (("a", "a"), ("b", "b")):
        for cx in centers[pal]:
            sc = copy.deepcopy(scene)
            sc["pallets"][pal]["center"][0] = cx
            worst = np.inf
            for color in sc["pallets"]["slot_order"]:
                for dz in (0.013, 0.113):
                    q = robots[k].ik_closest(make_tf(R_DOWN, slot_pos(sc, pal, color) + [0, 0, dz]),
                                             [0, -1.0, 2.0, -2.5, -1.57, 0])
                    worst = min(worst, np.inf if q is None else float(np.min(POS_LIMIT - np.abs(q))))
            dist = abs(cx - scene["robots"][k]["base_xy"][0])
            print(f"pallet {pal} centre x={cx:+.2f} ({dist:.2f} m from base in x): min joint margin over slots {worst:.2f}")


def main():
    scene, phys = build_mjcf.load_cfg()
    reach(scene, {"a": [-0.55, -0.45, -0.40], "b": [0.55, 0.45, 0.40]})
    for off in (0.005, 0.009, 0.013, 0.017):
        k, hits, lift = grasp_trial(scene, phys, off)
        print(f"grasp offset {off * 1000:.0f} mm: knuckle {k:.3f}, finger-pallet contacts {hits}, cube lifted {lift:.1f} mm")


if __name__ == "__main__":
    main()
