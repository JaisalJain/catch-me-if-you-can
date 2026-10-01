#!/usr/bin/env python3
"""Offline M5 catch study (MuJoCo Python on the same generated scene).

Robot B waits in a cup pose (TCP at the intercept point, palm normal = -incoming velocity, fingers along world y,
fully open); the cube is launched with the throw plan's release state; the gripper closes when the palm force
sensor exceeds a threshold (after an emulated detection+command latency), or — baseline — at a fixed predicted
time. Success: cube held between the pads, at rest, 1 s after closing.

    env -u PYTHONPATH .venv/bin/python tools/catch_study.py [--sweep] [--envelope]
"""
import argparse
import os
import sys

import mujoco
import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for p in ("src/catch_kinematics", "src/catch_ballistics", "src/catch_throw", "src/catch_description/scripts", "tools"):
    sys.path.insert(0, os.path.join(REPO, p))
import build_mjcf  # noqa: E402
import feasibility as F  # noqa: E402
from catch_ballistics import Projectile, velocity_for_launch_angle  # noqa: E402
from catch_kinematics import POS_LIMIT  # noqa: E402
from catch_kinematics.transforms import make_tf  # noqa: E402
from catch_throw.generator import toss_orientation  # noqa: E402

JN = F.JN


def throw_state():
    cfg = yaml.safe_load(open(os.path.join(REPO, "src/catch_throw/config/throw.yaml")))["throw"]
    p, c = np.array(cfg["release_point"]), np.array(cfg["catch_point"])
    v, T = velocity_for_launch_angle(p, c, np.radians(cfg["launch_angle_deg"]))
    n = v / np.linalg.norm(v)
    spin = cfg["pitch_rate"] * np.linalg.norm(v) * np.cross([0, 0, 1.0], n) / np.linalg.norm(np.cross([0, 0, 1.0], n))
    return p, v, spin, toss_orientation(n, cfg["finger_axis"]), T, c


def catch_pose_candidates(robot, p_catch, v_catch, m, d, qadr, pallet_geoms):
    """All IK solutions for the cup pose, with clearance (B links vs table/pedestal/pallet) and elbow-up flag."""
    R = toss_orientation(-v_catch / np.linalg.norm(v_catch))
    out = []
    ft = np.zeros(6)
    body = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ""
    bgeoms = [g for g in range(m.ngeom) if body(g).startswith("b_") and body(g) != "b_shoulder_link"
              and (m.geom_contype[g] or m.geom_conaffinity[g])]
    for q in robot.ik_all(make_tf(R, p_catch)):
        if np.any(np.abs(q) > POS_LIMIT - 0.2):
            continue
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_collision(m, d)
        clr = min(mujoco.mj_geomDistance(m, d, a, b, 0.5, ft) for a in bgeoms for b in pallet_geoms)
        fr = robot.frames(q)
        shoulder, elbow, wrist = fr[1][:3, 3], fr[2][:3, 3], fr[4][:3, 3]
        # elbow above the shoulder-wrist line
        t = np.clip(np.dot(elbow - shoulder, wrist - shoulder) / np.dot(wrist - shoulder, wrist - shoulder), 0, 1)
        elbow_up = elbow[2] > (shoulder + t * (wrist - shoulder))[2]
        out.append(dict(q=q, clearance=clr, elbow_up=bool(elbow_up), margin=float(np.min(POS_LIMIT - np.abs(q)))))
    return out


class Sim:
    def __init__(self, phys_override=None):
        scene, phys = build_mjcf.load_cfg()
        if phys_override:
            phys = phys_override(phys)
        self.scene = scene
        self.m = build_mjcf.compile_model(scene, phys)
        self.d = mujoco.MjData(self.m)
        m = self.m
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qa = {k: [m.jnt_qposadr[jid(f"{k}_{j}")] for j in JN] for k in "ab"}
        self.va = {k: [m.jnt_dofadr[jid(f"{k}_{j}")] for j in JN] for k in "ab"}
        self.act = {k: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{k}_{j}_motor") for j in JN] for k in "ab"}
        self.grip = {k: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{k}_robotiq_85_left_knuckle_joint") for k in "ab"}
        self.kq = {k: m.jnt_qposadr[jid(f"{k}_robotiq_85_left_knuckle_joint")] for k in "ab"}
        self.cube = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube_red")
        self.cq = m.jnt_qposadr[m.body_jntadr[self.cube]]
        self.cv = m.jnt_dofadr[m.body_jntadr[self.cube]]
        self.tcp = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "b_robotiq_tcp")
        self.fsens = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "b_robotiq_palm_ft_force")]
        self.touch = m.sensor_adr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "b_robotiq_palm_touch")]
        y = yaml.safe_load(open(os.path.join(REPO, "src/catch_sim/config/controllers.yaml")))
        g = y["arm_a_controller"]["ros__parameters"]["gains"]
        self.kp = np.array([g[f"a_{j}"]["p"] for j in JN])
        self.kd = np.array([g[f"a_{j}"]["d"] for j in JN])
        self.robots = F.make_robots(scene)
        self.poses = yaml.safe_load(open(os.path.join(REPO, "src/catch_description/config/poses.yaml")))

    def pallet_geoms(self):
        m = self.m
        return [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(
            ("table_top", "pallet_", "a_pedestal", "b_pedestal"))]

    def run(self, q_b, trigger="force", f_th=2.0, latency=0.006, t_close_pred=None, close_to=0.70, dv=None,
            settle=1.0, rot_err_deg=0.0):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        p_r, v_r, spin, R_r, T, c = throw_state()
        if dv is not None:
            v_r = v_r + dv
        d.qpos[self.qa["a"]] = self.poses["a"]["home"]
        d.qpos[self.qa["b"]] = q_b
        quat = np.zeros(4)
        R = R_r if not rot_err_deg else R_r @ _rx(np.radians(rot_err_deg))
        mujoco.mju_mat2Quat(quat, R.flatten())
        d.qpos[self.cq:self.cq + 3] = p_r
        d.qpos[self.cq + 3:self.cq + 7] = quat
        d.qvel[self.cv:self.cv + 3] = v_r
        d.qvel[self.cv + 3:self.cv + 6] = R.T @ spin      # free-joint angular velocity is in the body frame
        # other cubes out of the way
        for col in ("green", "blue"):
            b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"cube_{col}")
            a = m.jnt_qposadr[m.body_jntadr[b]]
            d.qpos[a:a + 3] = [0.0, -0.3 if col == "green" else -0.35, 0.8]
        mujoco.mj_forward(m, d)
        t_hit, t_cmd, closed_at, peak_f = None, None, None, 0.0
        log = []
        nxt = 0.0
        while d.time < T + settle + 0.3:
            if d.time >= nxt:
                nxt += 0.002
                for k in "ab":
                    q_t = self.poses["a"]["home"] if k == "a" else q_b
                    d.ctrl[self.act[k]] = self.kp * (np.array(q_t) - d.qpos[self.qa[k]]) - self.kd * d.qvel[self.va[k]]
                f = np.linalg.norm(d.sensordata[self.fsens:self.fsens + 3])
                peak_f = max(peak_f, f)
                if trigger == "force" and t_hit is None and f > f_th:
                    t_hit = d.time
                    t_cmd = t_hit + latency
                if trigger == "time" and t_cmd is None:
                    t_cmd = T + t_close_pred
                d.ctrl[self.grip["b"]] = close_to if (t_cmd is not None and d.time >= t_cmd) else 0.0
                if t_cmd is not None and closed_at is None and d.time >= t_cmd:
                    closed_at = d.time
            mujoco.mj_step(m, d)
            R_t = d.site_xmat[self.tcp].reshape(3, 3)
            log.append((d.time, *(R_t.T @ (d.xpos[self.cube] - d.site_xpos[self.tcp]))))
            if getattr(self, "trace", False) and abs(d.time - T) < 0.06 and int(round(d.time * 1000)) % 4 == 0:
                Rrel = R_t.T @ d.xmat[self.cube].reshape(3, 3)
                print(f"  t-T={1000 * (d.time - T):+5.0f} ms rel={np.round(1000 * np.array(log[-1][1:]), 1)} mm "
                      f"rot_x={np.degrees(np.arctan2(Rrel[2, 1], Rrel[1, 1])):6.1f} F={np.linalg.norm(d.sensordata[self.fsens:self.fsens + 3]):6.1f} "
                      f"knuckle={d.qpos[self.kq['b']]:.3f} vcube={np.round(d.qvel[self.cv:self.cv + 3], 2)}")
        rel = np.array(log[-1][1:])
        knuckle = d.qpos[self.kq["b"]]
        speed = np.linalg.norm(d.qvel[self.cv:self.cv + 3])
        held = bool(np.linalg.norm(rel) < 0.02 and 0.36 <= knuckle <= 0.56 and speed < 0.05)
        # in-hand rotation about the closing axis (tool x) relative to the TCP frame, folded to +-45 deg
        Rc = d.xmat[self.cube].reshape(3, 3)
        Rrel = d.site_xmat[self.tcp].reshape(3, 3).T @ Rc
        ang = np.degrees(np.arctan2(Rrel[2, 1], Rrel[1, 1]))
        ang = (ang + 45) % 90 - 45
        return dict(held=held, t_hit=None if t_hit is None else round(t_hit - T, 4), peak_force=round(peak_f, 1),
                    knuckle=round(float(knuckle), 3), rel_mm=(1000 * rel).round(1).tolist(), inhand_rot_deg=round(float(ang), 1))


def _rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--envelope", action="store_true")
    args = ap.parse_args()
    sim = Sim()
    p_r, v_r, spin, R_r, T, c = throw_state()
    v_c = Projectile(p_r, v_r).velocity(T)
    cands = catch_pose_candidates(sim.robots["b"], c, v_c, sim.m, sim.d, sim.qa["b"], sim.pallet_geoms())
    for k in cands:
        print(f"q={np.round(k['q'], 3).tolist()} elbow_up={k['elbow_up']} clearance={k['clearance'] * 1000:.0f} mm margin={k['margin']:.2f}")
    home_b = np.array(sim.poses["b"]["home"])
    ok = [k for k in cands if k["elbow_up"] and k["clearance"] > 0.05]
    best = min(ok, key=lambda k: float(np.sum(np.abs(((k["q"] - home_b + np.pi) % (2 * np.pi)) - np.pi))))
    q_b = best["q"]
    print("chosen catch pose", np.round(q_b, 4).tolist())
    print("force trigger:", sim.run(q_b))
    print("time trigger (+5 ms after predicted arrival):", sim.run(q_b, trigger="time", t_close_pred=0.005))
    if args.envelope:
        # release-velocity error envelope (in the throw plane): held / not held, force trigger, 6 ms latency
        rng = np.arange(-0.15, 0.151, 0.025)
        grid = np.zeros((len(rng), len(rng)), bool)
        for i, dvz in enumerate(rng):
            for j, dvx in enumerate(rng):
                grid[i, j] = sim.run(q_b, dv=np.array([dvx, 0.0, dvz]))["held"]
        print("held map (rows: dv_z, cols: dv_x, m/s):")
        print("        " + " ".join(f"{v:+.3f}"[:6].rjust(6) for v in rng))
        for i, dvz in enumerate(rng):
            print(f"{dvz:+.3f}  " + " ".join(("  held" if g else "  ----") for g in grid[i]))
        out = os.path.join(REPO, "docs", "m5", "catch_envelope.json")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        import json
        json.dump(dict(dv=rng.round(3).tolist(), held=grid.tolist(), note="rows dv_z, cols dv_x; force trigger, 6 ms latency"),
                  open(out, "w"))
    if args.sweep:
        for lat in (0.002, 0.006, 0.012, 0.025):
            print("latency", lat, sim.run(q_b, latency=lat))
        for dt in (-0.02, -0.01, 0.0, 0.01, 0.02, 0.04):
            print("time trigger offset", dt, sim.run(q_b, trigger="time", t_close_pred=dt))
        for dvx in (-0.03, -0.015, 0.015, 0.03):
            print("velocity error x", dvx, sim.run(q_b, dv=np.array([dvx, 0, 0])))
            print("velocity error z", dvx, sim.run(q_b, dv=np.array([0, 0, dvx])))


if __name__ == "__main__":
    main()
