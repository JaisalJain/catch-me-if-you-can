#!/usr/bin/env python3
"""Offline static-grasp study (MuJoCo Python, same MJCF generator as the sim).

A cube is placed on A's palm insert, the gripper closes (command 0.70, force-limited), then arm A follows
10 s of multi-joint sinusoids (peak joint speeds ~0.6-1.5 rad/s, gripper sideways / upside down) under a PD
emulating the JTC (gains from catch_sim/config/controllers.yaml, 500 Hz zero-order hold) with MuJoCo gravity
compensation. Slip = drift of the cube centre relative to the TCP site after the grasp has settled.

    env -u PYTHONPATH .venv/bin/python tools/grasp_study.py [--sweep]
"""
import argparse
import copy
import os
import sys

import mujoco
import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "src/catch_description/scripts"))
import build_mjcf  # noqa: E402

JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
AMP = np.array([0.6, 0.2, 0.25, 0.3, 0.4, 1.5])  # collision-free around palm_up (verified by the contact check)   # chosen collision-free (checked below) around palm_up
FREQ = np.array([0.17, 0.23, 0.19, 0.29, 0.21, 0.16])


def gains():
    y = yaml.safe_load(open(os.path.join(REPO, "src/catch_sim/config/controllers.yaml")))
    g = y["arm_a_controller"]["ros__parameters"]["gains"]
    return np.array([g[f"a_{j}"]["p"] for j in JN]), np.array([g[f"a_{j}"]["d"] for j in JN])


def run(scene, phys, duration=10.0, color="red", verbose=False):
    m = build_mjcf.compile_model(scene, phys)
    d = mujoco.MjData(m)
    poses = yaml.safe_load(open(os.path.join(REPO, "src/catch_description/config/poses.yaml")))
    home = np.array(poses["a"]["palm_up"])
    jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"a_{j}") for j in JN]
    qa, va = [m.jnt_qposadr[j] for j in jid], [m.jnt_dofadr[j] for j in jid]
    aid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"a_{j}_motor") for j in JN]
    for k in "ab":
        for i, j in enumerate(JN):
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{k}_{j}")]] = poses[k]["palm_up"][i]
    grip = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "a_robotiq_85_left_knuckle_joint")
    tcp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "a_robotiq_tcp")
    cube = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"cube_{color}")
    cq = m.jnt_qposadr[m.body_jntadr[cube]]
    pads = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"a_robotiq_{s}_pad") for s in ("left", "right")]
    mujoco.mj_forward(m, d)
    d.qpos[cq:cq + 3] = d.site_xpos[tcp]
    q4 = np.zeros(4)
    mujoco.mju_mat2Quat(q4, d.site_xmat[tcp])
    d.qpos[cq + 3:cq + 7] = q4
    kp, kd = gains()
    hold = 0.0
    ctrl_dt, next_ctrl = 0.002, 0.0
    t_settle, slip, rel0, log, rot, R0 = 1.5, [], None, [], [], None
    contacts = set()
    pslip, prel0, fing, frel0, trace = [], None, [], None, []
    nsteps = int((t_settle + duration) / m.opt.timestep)
    for _ in range(nsteps):
        t = d.time
        if t >= next_ctrl:
            next_ctrl += ctrl_dt
            tm = max(0.0, t - t_settle)
            ramp = min(1.0, tm / 1.0)
            qd = home + ramp * AMP * np.sin(2 * np.pi * FREQ * tm)
            vd = ramp * AMP * 2 * np.pi * FREQ * np.cos(2 * np.pi * FREQ * tm)
            d.ctrl[aid] = kp * (qd - d.qpos[qa]) + kd * (vd - d.qvel[va])
            d.ctrl[grip] = 0.70 if t > 0.1 else 0.0
        mujoco.mj_step(m, d)
        for i in range(d.ncon):  # any contact other than cube<->pads/insert or cubes<->pallet invalidates the test
            c = d.contact[i]
            pair = tuple(sorted(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or "" for g in (c.geom1, c.geom2)))
            if "world" in pair:  # name the world geom (table, pedestal, floor)
                pair = tuple(sorted(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or p_
                                    for g, p_ in zip((c.geom1, c.geom2), pair)))
            if not (pair[0].startswith("cube_") and pair[1].startswith("pallet")) and not (
                    f"cube_{color}" in pair and any(x in pair[0] + pair[1] for x in ("_pad", "robotiq_base"))):
                contacts.add(pair)
        R = d.site_xmat[tcp].reshape(3, 3)
        rel = R.T @ (d.xpos[cube] - d.site_xpos[tcp])
        Rrel = R.T @ d.xmat[cube].reshape(3, 3)
        # true slip: cube centre in the frame of the left pad body; finger motion: pad midpoint in the TCP frame
        Rp = d.xmat[pads[0]].reshape(3, 3)
        prel = Rp.T @ (d.xpos[cube] - d.xpos[pads[0]])
        frel = R.T @ ((d.xpos[pads[0]] + d.xpos[pads[1]]) / 2 - d.site_xpos[tcp])
        if t > t_settle and prel0 is None:
            prel0, frel0 = prel.copy(), frel.copy()
        if prel0 is not None:
            pslip.append(np.linalg.norm(prel - prel0))
            fing.append(np.linalg.norm(frel - frel0))
        if t > t_settle and rel0 is None:
            rel0, R0 = rel.copy(), Rrel.copy()
        if rel0 is not None:
            slip.append(np.linalg.norm(rel - rel0))
            rot.append(np.degrees(np.arccos(np.clip((np.trace(R0.T @ Rrel) - 1) / 2, -1, 1))))
    slip = np.array(slip)
    if contacts:
        print("WARNING: test invalid, unexpected contacts:", sorted(contacts))
    return dict(valid=not contacts, pad_slip_max_mm=1000 * max(pslip), finger_motion_max_mm=1000 * max(fing), slip_max_mm=1000 * slip.max(), slip_final_mm=1000 * slip[-1], rot_max_deg=float(np.max(rot)), held=bool(np.linalg.norm(rel) < 0.03),
                knuckle=float(d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, 'a_robotiq_85_left_knuckle_joint')]]),
                contacts=sorted(contacts), peak_qd=float(np.max(AMP * 2 * np.pi * FREQ)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    args = ap.parse_args()
    scene, phys = build_mjcf.load_cfg()
    if not args.sweep:
        r = run(scene, phys)
        print(r)
        return
    rows = []
    for ts in (0.001, 0.002):
        for mu in (0.6, 0.9, 1.2):
            for tors, condim in ((0.01, 4), (0.03, 6)):
                p = copy.deepcopy(phys)
                p["timestep"] = ts
                p["cube"]["friction"][:2] = [mu, tors]
                p["pads"]["friction"][:2] = [mu, tors]
                p["cube"]["condim"] = condim
                r = run(scene, p)
                rows.append((ts, mu, tors, condim, r))
                print(f"dt={ts} mu={mu} torsion={tors} condim={condim}: valid={r['valid']} slip max {r['slip_max_mm']:.2f} mm "
                      f"rot max {r['rot_max_deg']:.2f} deg held={r['held']}", flush=True)


if __name__ == "__main__":
    main()
