#!/usr/bin/env python3
"""M1 feasibility study: sweep release/catch geometry and launch angle for the underhand toss.

    env -u PYTHONPATH .venv/bin/python tools/feasibility.py [--quick]

For every candidate geometry it
  1. solves the release velocity (catch_ballistics) for the release point -> catch point at the launch angle,
  2. picks the IK branch and palm pitch rate kappa minimising the peak joint-speed ratio at release,
  3. generates the full stroke (catch_throw.generator) and checks joint position / velocity margins,
  4. checks joint torques with MuJoCo inverse dynamics (Menagerie inertias + gripper + 60 g cube, gravity
     included, against 150/150/150/28/28/28 N m), and clearance of arm + cube to the table/pallets,
  5. checks that B can hold a catch pose there (TCP at the catch point, palm normal against the incoming
     velocity) with joint position margin.
Writes docs/feasibility/results.json, plots, and prints the ranked feasible set.

Limits used (UR5e, ur_description joint_limits.yaml = UR5e user manual): position +-2pi (elbow +-pi),
velocity 180 deg/s = pi rad/s on all joints, torque 150/150/150/28/28/28 N m. UR publishes no joint
acceleration limit ("the robot does not inherently have any limits on joint accelerations (only on
torques)" - ur_moveit_config); we bound torque, and additionally report peak accelerations.
Margins required: velocity <= 85 % of limit, torque <= 80 % of limit, position >= 0.15 rad from limits,
TCP (cube centre) >= 6 cm above the table top, robot geoms >= 2 cm from table/pallets/other robot.
"""
import argparse
import itertools
import json
import os
import sys

import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
for p in ("src/catch_kinematics", "src/catch_ballistics", "src/catch_throw", "src/catch_description/scripts"):
    sys.path.insert(0, os.path.join(REPO, p))

import mujoco  # noqa: E402

import build_mjcf  # noqa: E402
from catch_ballistics import velocity_for_launch_angle  # noqa: E402
from catch_kinematics import POS_LIMIT, TORQUE_LIMIT, VEL_LIMIT, UR5e  # noqa: E402
from catch_kinematics.transforms import make_tf, rot_z  # noqa: E402
from catch_throw.generator import (InfeasibleThrow, generate_throw, pitch_axis, toss_orientation)  # noqa: E402
from catch_throw.profile import ProfileParams  # noqa: E402

VEL_MAX, TAU_MAX, POS_MIN, TCP_CLEAR, GEOM_CLEAR = 0.85, 0.80, 0.15, 0.06, 0.02
JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def load():
    scene, phys = build_mjcf.load_cfg()
    tcp = scene["gripper"]["mount_offset"] + scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2
    return scene, phys, make_robots(scene)


def make_robots(scene):
    tcp = scene["gripper"]["mount_offset"] + scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2
    return {k: UR5e(make_tf(rot_z(scene["robots"][k]["base_yaw"]), build_mjcf.robot_base_xyz(scene, k)),
                    make_tf(p=[0, 0, tcp])) for k in build_mjcf.robot_keys(scene)}


class Dynamics:
    """Single UR5e + 2F-85 + 60 g payload at the TCP, gravity compensation OFF so mj_inverse returns the
    total torque the joint must deliver (inertial + Coriolis + gravity)."""

    def __init__(self, scene, phys):
        phys = dict(phys, arm={"gravcomp": False})
        arm = build_mjcf.make_arm(scene, phys)
        arm.body("robotiq_base") if False else None
        g = [b for b in arm.bodies if b.name == "robotiq_base"][0]
        e = scene["cube"]["edge"]
        g.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[e / 2] * 3, pos=[0, 0, scene["gripper"]["palm_insert"]["top_z"] + e / 2],
                   mass=scene["cube"]["mass"], contype=0, conaffinity=0)
        for b in arm.bodies:
            b.gravcomp = 0.0
        spec = mujoco.MjSpec()
        r = scene["robots"]["a"]
        fr = spec.worldbody.add_frame(pos=build_mjcf.robot_base_xyz(scene, "a"), quat=[np.cos((r["base_yaw"] + np.pi) / 2), 0, 0,
                                                             np.sin((r["base_yaw"] + np.pi) / 2)])
        spec.attach(arm, prefix="", frame=fr)
        assets = os.path.join(REPO, "src/catch_description/mjcf/assets")
        for m in spec.meshes:
            m.file = os.path.join(assets, m.file)
        self.m = spec.compile()
        self.d = mujoco.MjData(self.m)
        self.dofs = [self.m.jnt_dofadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JN]
        self.qadr = [self.m.jnt_qposadr[mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JN]

    def torques(self, q, qd, qdd):
        out = np.zeros_like(q)
        for i in range(len(q)):
            self.d.qpos[:] = 0
            self.d.qvel[:] = 0
            self.d.qacc[:] = 0
            self.d.qpos[self.qadr] = q[i]
            self.d.qvel[self.dofs] = qd[i]
            self.d.qacc[self.dofs] = qdd[i]
            mujoco.mj_inverse(self.m, self.d)
            out[i] = self.d.qfrc_inverse[self.dofs]
        return out


class Clearance:
    """Full scene; distance from robot-A collision geoms (and a cube at the TCP) to table / pallets / B."""

    def __init__(self, path=os.path.join(REPO, "src/catch_description/mjcf/scene.xml")):
        self.m = mujoco.MjModel.from_xml_path(path)
        self.d = mujoco.MjData(self.m)
        m = self.m
        name = lambda i: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[i]) or ""
        # a_shoulder_link only rotates about the vertical base axis: its 2.5 mm gap to the table top is constant
        self.a_geoms = [i for i in range(m.ngeom) if name(i).startswith("a_") and name(i) != "a_shoulder_link"
                        and m.geom_contype[i] | m.geom_conaffinity[i]]
        self.env_geoms = [i for i in range(m.ngeom)
                          if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith(("table_top", "pallet_", "a_pedestal", "b_pedestal"))]
        self.qadr = {k: [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{k}_{j}")] for j in JN]
                     for k in "ab"}

    def min_distance(self, q_traj, q_b=None, every=5):
        dmin, self.worst = np.inf, None
        fromto = np.zeros(6)
        for q in q_traj[::every]:
            self.d.qpos[self.qadr["a"]] = q
            if q_b is not None:
                self.d.qpos[self.qadr["b"]] = q_b
            mujoco.mj_kinematics(self.m, self.d)
            mujoco.mj_collision(self.m, self.d)
            for ga in self.a_geoms:
                for ge in self.env_geoms:
                    dist = mujoco.mj_geomDistance(self.m, self.d, ga, ge, 0.2, fromto)
                    if dist < dmin:
                        dmin = dist
                        bn = mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_BODY, self.m.geom_bodyid[ga])
                        self.worst = (bn, mujoco.mj_id2name(self.m, mujoco.mjtObj.mjOBJ_GEOM, ge))
        return dmin


def catch_pose_ok(robot_b, p_catch, v_catch, q_seed):
    """TCP at the catch point, palm normal (tool z) anti-parallel to the incoming velocity, fingers along y."""
    n = -v_catch / np.linalg.norm(v_catch)
    R = toss_orientation(n)
    best, margin = None, -np.inf
    for q in robot_b.ik_all(make_tf(R, p_catch)):
        mg = float(np.min(POS_LIMIT - np.abs(q)))
        if mg > margin and abs(np.linalg.det(robot_b.jacobian(q))) > 1e-3:
            best, margin = q, mg
    return best, margin


def best_branch_and_kappa(robot, p, v, kappas=np.linspace(-6, 6, 241)):
    V = np.linalg.norm(v)
    n = v / V
    R = toss_orientation(n)
    best = (np.inf, None, None)
    for q in robot.ik_all(make_tf(R, p)):
        if np.any(np.abs(q) > POS_LIMIT - POS_MIN):
            continue
        J = robot.jacobian(q)
        if abs(np.linalg.det(J)) < 1e-4:
            continue
        Ji = np.linalg.inv(J)
        a0 = Ji @ np.r_[V * n, 0, 0, 0]
        a1 = Ji @ np.r_[0, 0, 0, V * pitch_axis(n)]
        ratios = np.max(np.abs(a0[None] + kappas[:, None] * a1[None]) / VEL_LIMIT, axis=1)
        k = int(np.argmin(ratios))
        if ratios[k] < best[0]:
            best = (float(ratios[k]), float(kappas[k]), q)
    return best


def evaluate(robots, dyn, clr, scene, cand, prof):
    xr, zr, xc, dzc, deg, y = cand
    p = np.array([xr, y, zr])
    c = np.array([xc, y, zr + dzc])
    res = dict(xr=xr, zr=zr, xc=xc, zc=zr + dzc, deg=deg, y=y, accel=prof.accel, decel=prof.decel, ok=False)
    try:
        v, T = velocity_for_launch_angle(p, c, np.radians(deg))
    except ValueError as e:
        res["why"] = str(e)
        return res
    res.update(speed=float(np.linalg.norm(v)), flight_time=float(T), apex=float(p[2] + max(v[2], 0) ** 2 / 19.62))
    r0, kappa, q_r = best_branch_and_kappa(robots["a"], p, v)
    if q_r is None:
        res["why"] = "no IK branch at release"
        return res
    res.update(release_vel_ratio=r0, kappa=kappa, spin=kappa * res["speed"])
    if r0 > VEL_MAX:
        res["why"] = f"release joint speed {r0:.2f} of limit"
        return res
    try:
        plan = generate_throw(robots["a"], p, v, q_r, prof, pitch_rate=kappa)
    except (InfeasibleThrow, ValueError) as e:
        res["why"] = f"generation: {e}"
        return res
    m = plan.metrics
    res.update(vel_ratio=m["vel_ratio_max"], pos_margin=m["pos_margin_min"], tcp_z_min=m["tcp_z_min"],
               stroke=m["stroke_length"], duration=m["duration"], acc_max=m["acc_max"])
    top = scene["table"]["top_z"]
    if m["vel_ratio_max"] > VEL_MAX:
        res["why"] = f"stroke joint speed {m['vel_ratio_max']:.2f}"
        return res
    if m["pos_margin_min"] < POS_MIN:
        res["why"] = f"position margin {m['pos_margin_min']:.2f}"
        return res
    if m["tcp_z_min"] < top + TCP_CLEAR:
        res["why"] = f"TCP {m['tcp_z_min'] - top:.3f} m above table"
        return res
    tau = dyn.torques(plan.q, plan.qd, plan.qdd)
    tau_ratio = np.max(np.abs(tau), axis=0) / TORQUE_LIMIT
    res["tau_ratio"] = tau_ratio.tolist()
    res["tau_ratio_max"] = float(tau_ratio.max())
    if tau_ratio.max() > TAU_MAX:
        res["why"] = f"torque {tau_ratio.max():.2f} of limit"
        return res
    # B catch pose
    flight = plan.flight()
    t_c = plan.t_release + T
    q_b, b_margin = catch_pose_ok(robots["b"], c, flight.velocity(t_c), np.zeros(6))
    res["b_pos_margin"] = b_margin
    if q_b is None or b_margin < POS_MIN:
        res["why"] = "B cannot hold the catch pose"
        return res
    dmin = clr.min_distance(plan.q, q_b)
    res["clearance"] = float(dmin)
    if dmin < GEOM_CLEAR:
        res["why"] = f"clearance {dmin:.3f} m {clr.worst}"
        return res
    res["ok"] = True
    res["q_release"] = plan.q[np.argmin(np.abs(plan.t - plan.t_release))].tolist()
    res["q_start"] = plan.q_start.tolist()
    res["q_catch_b"] = q_b.tolist()
    res["why"] = ""
    return res


def score(r):
    """Prefer long flights, then low spin, then margins."""
    margin = min(1 - r["vel_ratio"] / VEL_MAX, 1 - r["tau_ratio_max"] / TAU_MAX)
    return (r["xc"] - r["xr"]) * 2.0 - abs(r["spin"]) * 0.05 + margin * 0.5


def write_summary(results, ok, risers, out_dir):
    """Compact, committed summary (results.json is large and git-ignored)."""
    from collections import Counter
    cells = []
    for riser in risers:
        for D in sorted({round(r["xc"] - r["xr"], 2) for r in results}):
            sel = [r for r in results if r["riser"] == riser and abs((r["xc"] - r["xr"]) - D) < 1e-6]
            near = [r for r in sel if "release_vel_ratio" in r]
            b = min(near, key=lambda r: r["release_vel_ratio"]) if near else None
            cells.append(dict(riser=riser, flight=D, n=len(sel), n_feasible=sum(r["ok"] for r in sel),
                              best_release_vel_ratio=b["release_vel_ratio"] if b else None,
                              best_reason=(b["why"] or "OK") if b else None))
    keep = ["riser", "xr", "zr", "xc", "zc", "deg", "y", "accel", "decel", "speed", "flight_time", "apex", "spin",
            "kappa", "vel_ratio", "tau_ratio_max", "pos_margin", "clearance", "stroke", "b_pos_margin", "q_release"]
    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(dict(limits=dict(vel=VEL_LIMIT.tolist(), torque=TORQUE_LIMIT.tolist(), pos=POS_LIMIT.tolist(),
                                   vel_max_ratio=VEL_MAX, torque_max_ratio=TAU_MAX, pos_margin=POS_MIN,
                                   tcp_clearance=TCP_CLEAR, geom_clearance=GEOM_CLEAR),
                       n_candidates=len(results), n_feasible=len(ok),
                       rejection_reasons=Counter(" ".join(r["why"].split(" ")[:2]) for r in results if not r["ok"]),
                       cells=cells, feasible=[{k: r[k] for k in keep} for r in ok]), f, indent=1, default=float)


def main():
    import copy
    import tempfile
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    args = ap.parse_args()
    scene0, phys = build_mjcf.load_cfg()
    out_dir = os.path.join(REPO, "docs", "feasibility")
    os.makedirs(out_dir, exist_ok=True)
    if args.quick:
        risers, xrs, dzrs, xcs, dzcs, degs, ys = [0.2], [-0.15], [0.2, 0.3], [0.35], [0.0], [65], [0.35]
        profs = [ProfileParams()]
    else:
        risers = [0.0, 0.15, 0.25, 0.35]
        xrs = [-0.35, -0.25, -0.15, -0.05, 0.05]
        dzrs = [0.15, 0.20, 0.25, 0.30, 0.40]       # release height above the robot base
        xcs = [0.25, 0.35, 0.45]
        dzcs = [-0.1, 0.0]
        degs = [55, 65, 75]
        ys = [0.1333, 0.25, 0.35]
        profs = [ProfileParams(accel=12, decel=25), ProfileParams(accel=10, decel=16, tau3=0.03)]
    results = []
    tmp = tempfile.mkdtemp(prefix="feas_", dir=os.environ.get("TMPDIR"))
    for riser in risers:
        scene = copy.deepcopy(scene0)
        for k in build_mjcf.robot_keys(scene):
            scene["robots"][k]["pedestal_height"] = riser
        build_mjcf.generate(out_dir=tmp, verbose=False, scene=scene, phys=phys)
        robots, dyn, clr = make_robots(scene), Dynamics(scene, phys), Clearance(os.path.join(tmp, "scene.xml"))
        base_z = build_mjcf.robot_base_xyz(scene, "a")[2]
        for xr, dzr, xc, dzc, deg, y in itertools.product(xrs, dzrs, xcs, dzcs, degs, ys):
            for prof in profs:
                r = evaluate(robots, dyn, clr, scene, (xr, base_z + dzr, xc, dzc, deg, y), prof)
                r["riser"] = riser
                results.append(r)
        print(f"riser {riser:.2f}: {sum(r['ok'] for r in results if r['riser'] == riser)} feasible", flush=True)
    ok = sorted([r for r in results if r["ok"]], key=score, reverse=True)
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(dict(limits=dict(vel=VEL_LIMIT.tolist(), torque=TORQUE_LIMIT.tolist(), pos=POS_LIMIT.tolist(),
                                   vel_max_ratio=VEL_MAX, torque_max_ratio=TAU_MAX, pos_margin=POS_MIN,
                                   tcp_clearance=TCP_CLEAR, geom_clearance=GEOM_CLEAR),
                       n_candidates=len(results), n_feasible=len(ok), feasible=ok,
                       rejected=[r for r in results if not r["ok"]]), f, indent=1, default=float)
    write_summary(results, ok, risers, out_dir)
    from collections import Counter
    print(f"{len(results)} candidates, {len(ok)} feasible")
    print("rejection reasons:", Counter(" ".join(r["why"].split(" ")[:2]) if r["why"] else "ok"
                                       for r in results).most_common(8))
    print("rank riser xr     zr    xc    zc   deg  y     V     T     apex  spin  vel   tau   pos   clr   stroke")
    for i, r in enumerate(ok[:25]):
        print(f"{i:3d} {r['riser']:.2f} {r['xr']:+.2f} {r['zr']:.2f} {r['xc']:+.2f} {r['zc']:.2f} {r['deg']:3d} {r['y']:.2f} "
              f"{r['speed']:.2f} {r['flight_time']:.3f} {r['apex']:.2f} {r['spin']:+.2f} {r['vel_ratio']:.2f} "
              f"{r['tau_ratio_max']:.2f} {r['pos_margin']:.2f} {r['clearance']:.3f} {r['stroke']:.2f} "
              f"a={r['accel']}/{r['decel']}")
    for riser in risers:
        for D in (0.3, 0.4, 0.5, 0.6, 0.7):
            sel = [r for r in results if r["riser"] == riser and abs((r["xc"] - r["xr"]) - D) < 1e-6]
            f_ = [r for r in sel if r["ok"]]
            near = [r for r in sel if "release_vel_ratio" in r]
            b = min(near, key=lambda r: r["release_vel_ratio"]) if near else None
            print(f"riser {riser:.2f} flight {D:.1f} m: {len(f_):3d} feasible / {len(sel)}"
                  + (f"; best release speed ratio {b['release_vel_ratio']:.2f} -> {b['why'] or 'OK'}" if b else ""))


if __name__ == "__main__":
    main()
