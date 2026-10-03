#!/usr/bin/env python3
"""Capsule collision model of a UR5e + 2F-85 for fast checks in the ROS nodes (catch_kinematics.collision), derived
from the MuJoCo collision geometry and validated against MuJoCo's own collision detection.

    env -u PYTHONPATH .venv/bin/python tools/export_collision.py [--n 4000]

* one capsule per link body (B's collision geoms: mesh vertices / primitive corners in the body frame, principal
  axis by PCA, radius = max distance to the segment), expressed in the DH frame the body moves with
  (catch_kinematics.UR5e.frames: 0 base, 1..5 links, 6 tool0); the gripper bodies are merged (fingers open);
* B's wrist camera hardware (visual-only geoms in the physics, scene.yaml b_wrist_camera_parts): one capsule per
  group (camera head, mount arm, upright) in frame 6, validated with mj_geomDistance (visual geoms make no contacts);
* the camera capsules are then inflated until they also cover MoveIt's view (URDF collision meshes, fcl): in random
  configurations around every reactive.yaml ready pose, whenever a camera part touches one of B's own arm links
  (shoulder .. wrist 1) in the URDF model, the capsule distance of that pair must be below min_clearance (the
  MuJoCo forearm geoms are slimmer than the UR forearm mesh: an elbow-up catch pose that the capsules cleared by
  10.6 mm was in contact for MoveIt, docs/v2_arm_config.md);
* obstacles: table top, both pedestals, both pallets (boxes from the scene);
* self-collision pairs: non-adjacent links;
* validation: random configurations around B's catch region; a configuration counts as "colliding" in MuJoCo if any
  B collision geom penetrates (or comes within `margin`) of another body; the capsule model must report a collision
  for every one of them (no false "free"); false alarms are reported.
Output: src/catch_kinematics/config/ur5e_capsules.yaml, docs/v2/phase2/collision_model.json
"""
import argparse
import json
import os
import sys
import time

import mujoco
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import v2_common as V  # noqa: E402
from catch_kinematics.collision import CapsuleModel  # noqa: E402

CAM_GROUPS = {"head": ["body", "glass", "lens_l", "lens_r", "emitter", "cable", "tilt_block"],
              "arm": ["lug", "arm"], "upright": ["upright", "knob"]}
BODY_FRAME = {"b_base": 0, "b_shoulder_link": 1, "b_upper_arm_link": 2, "b_forearm_link": 3, "b_wrist_1_link": 4,
              "b_wrist_2_link": 5, "b_wrist_3_link": 6}


def geom_points(m, d, g):
    """Points (world) covering a collision geom."""
    t = int(m.geom_type[g])
    R, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    s = m.geom_size[g]
    G = mujoco.mjtGeom
    if t == int(G.mjGEOM_MESH):
        mid = m.geom_dataid[g]
        a, n = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
        loc = m.mesh_vert[a:a + n]
    elif t == int(G.mjGEOM_BOX):
        loc = np.array([[sx * s[0], sy * s[1], sz * s[2]] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    elif t in (int(G.mjGEOM_CAPSULE), int(G.mjGEOM_CYLINDER)):
        r, h = s[0], s[1] + (s[0] if t == int(G.mjGEOM_CAPSULE) else 0)
        ang = np.linspace(0, 2 * np.pi, 12, endpoint=False)
        loc = np.array([[r * np.cos(a), r * np.sin(a), z] for a in ang for z in (-h, h)])
    elif t == int(G.mjGEOM_SPHERE):
        loc = np.array([[sx * s[0], sy * s[0], sz * s[0]] for sx in (-1, 0, 1) for sy in (-1, 0, 1) for sz in (-1, 0, 1)])
    else:
        return np.zeros((0, 3))
    return loc @ R.T + p


def fit_capsule(P):
    c = P.mean(0)
    _, _, Vt = np.linalg.svd(P - c)
    ax = Vt[0]
    s = (P - c) @ ax
    a, b = c + s.min() * ax, c + s.max() * ax
    # radius: max distance to the segment; then shrink the segment ends by the radius where possible
    def dist(P, a, b):
        ab = b - a
        t = np.clip(((P - a) @ ab) / max(ab @ ab, 1e-12), 0, 1)
        return np.linalg.norm(P - (a + np.outer(t, ab)), axis=1)
    r = dist(P, a, b).max()
    for _ in range(20):
        L = np.linalg.norm(b - a)
        k = min(r * 0.3, L * 0.2)
        a2, b2 = a + k * ax, b - k * ax
        r2 = dist(P, a2, b2).max()
        if r2 <= r:
            a, b, r = a2, b2, r2
        else:
            break
    return a, b, float(r)


def camera_mesh_margin(robot, cm, n, seed=1):
    """Extra radius for the camera capsules so that every configuration in which a camera part touches one of B's
    own arm links in the URDF collision model (MoveIt) has a capsule distance < min_clearance for that link pair.
    Configurations: every reactive.yaml ready pose + N(0, 0.6 rad) per joint."""
    import subprocess
    import fcl
    import gen_srdf as GS
    from catch_kinematics import UrdfChain
    urdf = subprocess.check_output(["xacro", os.path.join(V.REPO, "src/catch_description/urdf/scene.urdf.xacro")],
                                   env=GS.ros_env()).decode()
    objs, chain = GS.collision_objects(urdf), UrdfChain(urdf)
    cam_links, arm_links = ("b_wrist_cam_bracket", "b_wrist_cam_housing"), \
        {"b_shoulder_link": 1, "b_upper_arm_link": 2, "b_forearm_link": 3, "b_wrist_1_link": 4}
    rc = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_catcher/config/reactive.yaml")))
    readies = [np.array(p["q"]) for p in rc["ready_poses"].values()]
    rng = np.random.default_rng(seed)
    frame_of = {nm: f for nm, f in zip(cm.names, cm.frame)}
    # the planner flags a configuration when ANY tool-frame capsule (gripper or camera; the clamp collar lies inside
    # the gripper capsule) comes within min_clearance of the touched arm link
    cam_idx = [i for i, (x, y) in enumerate(cm.pairs) if 6 in (frame_of[x], frame_of[y])]
    need, n_contact = 0.0, 0
    for k in range(n):
        q = readies[k % len(readies)] + rng.normal(0, 0.6, 6)
        qd = {f"b_{j}": v for j, v in zip(V.JN, q)}
        hit = set()
        for lc in cam_links:
            for la, fa in arm_links.items():
                for g1, T1 in objs[lc]:
                    W1 = chain.link_pose(lc, qd) @ T1
                    o1 = fcl.CollisionObject(g1, fcl.Transform(W1[:3, :3], W1[:3, 3]))
                    for g2, T2 in objs[la]:
                        W2 = chain.link_pose(la, qd) @ T2
                        o2 = fcl.CollisionObject(g2, fcl.Transform(W2[:3, :3], W2[:3, 3]))
                        if fcl.collide(o1, o2, fcl.CollisionRequest(), fcl.CollisionResult()):
                            hit.add(fa)
        if not hit:
            continue
        n_contact += 1
        dist = cm.self_distances(robot, q)
        # capsule distance of the tool-frame capsules vs the touched arm frame(s)
        dc = min(float(dist[i]) for i in cam_idx
                 if {frame_of[cm.pairs[i][0]], frame_of[cm.pairs[i][1]]} & hit)
        need = max(need, dc - cm.min_clearance)
    extra = round(need + 0.002, 4) if need > 0 else 0.0
    return dict(n=n, n_contact=n_contact, max_capsule_excess=round(need, 4), extra_radius=extra)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=4000)
    ap.add_argument("--margin", type=float, default=0.0)
    ap.add_argument("--n-mesh", type=int, default=3000, help="configurations for the camera vs URDF-mesh check")
    a = ap.parse_args()
    m, scene, phys = V.build_model()
    d = mujoco.MjData(m)
    robot = V.F.make_robots(scene)["b"]
    rd = json.load(open(os.path.join(V.OUT, "ready_envelope.json")))["chosen"]
    q_ready = np.array(rd["q"])
    jid = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
    qadr = [jid(f"b_{j}") for j in V.JN]
    body = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ""
    coll = lambda g: bool(m.geom_contype[g] or m.geom_conaffinity[g])
    # B's collision geoms by DH frame (gripper bodies and everything below wrist_3 -> frame 6)
    def frame_of(b):
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        while name not in BODY_FRAME:
            b = m.body_parentid[b]
            if b <= 0:
                return None
            name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
        return BODY_FRAME[name]
    b_geoms = [g for g in range(m.ngeom) if body(g).startswith("b_") and coll(g)]
    gname = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
    cam_geoms = {grp: [g for g in range(m.ngeom) if gname(g)[len("b_robotiq_wrist_cam_"):] in parts]
                 for grp, parts in CAM_GROUPS.items()}
    assert all(cam_geoms.values()), cam_geoms
    d.qpos[qadr] = q_ready
    mujoco.mj_forward(m, d)
    F = robot.frames(q_ready)
    caps = []
    G = mujoco.mjtGeom
    for k in range(7):
        gs = [g for g in b_geoms if frame_of(m.geom_bodyid[g]) == k]
        if not gs:
            continue
        Fi = np.linalg.inv(F[k])
        mesh_pts = []
        for g in gs:
            t = int(m.geom_type[g])
            if t in (int(G.mjGEOM_CAPSULE), int(G.mjGEOM_CYLINDER)):
                # the UR5e link geoms are capsules / cylinders: exact capsule (a cylinder as its enclosing capsule)
                R, c = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
                r, h = m.geom_size[g][0], m.geom_size[g][1]
                P = np.array([c - h * R[:, 2], c + h * R[:, 2]]) @ Fi[:3, :3].T + Fi[:3, 3]
                caps.append(dict(name=f"f{k}_g{len(caps)}", frame=k, p0=P[0].round(5).tolist(), p1=P[1].round(5).tolist(),
                                 radius=round(float(r) + 0.002, 5)))
            else:
                mesh_pts.append(geom_points(m, d, g))
        if mesh_pts:           # gripper meshes / pads (open fingers): two capsules along tool z
            Pl = np.vstack(mesh_pts) @ Fi[:3, :3].T + Fi[:3, 3]
            z = Pl[:, 2]
            cut = np.percentile(z, 55)
            for i, part in enumerate([Pl[z <= cut], Pl[z > cut - 0.01]]):
                p0, p1, r = fit_capsule(part)
                caps.append(dict(name=f"f{k}_grip{i}", frame=k, p0=p0.round(5).tolist(), p1=p1.round(5).tolist(),
                                 radius=round(r + 0.003, 5)))
    # wrist camera hardware: one capsule per group, frame 6
    Fi = np.linalg.inv(F[6])
    for grp, gs in cam_geoms.items():
        P = np.vstack([geom_points(m, d, g) for g in gs]) @ Fi[:3, :3].T + Fi[:3, 3]
        p0, p1, r = fit_capsule(P)
        caps.append(dict(name=f"f6_cam_{grp}", frame=6, p0=p0.round(5).tolist(), p1=p1.round(5).tolist(),
                         radius=round(r + 0.003, 5)))
    # the clamp collar sits around the gripper base: inside the gripper capsule (checked, no capsule of its own)
    collar = [g for g in range(m.ngeom) if gname(g) == "b_robotiq_wrist_cam_collar"]
    Pc = geom_points(m, d, collar[0]) @ Fi[:3, :3].T + Fi[:3, 3]
    g0 = next(c for c in caps if c["name"] == "f6_grip0")
    a0, a1 = np.array(g0["p0"]), np.array(g0["p1"])
    tt = np.clip((Pc - a0) @ (a1 - a0) / ((a1 - a0) @ (a1 - a0)), 0, 1)
    worst = float(np.max(np.linalg.norm(Pc - (a0 + np.outer(tt, a1 - a0)), axis=1)))
    print(f"collar: farthest point {1000 * worst:.1f} mm from the f6_grip0 axis (radius {1000 * g0['radius']:.1f} mm)")
    assert worst <= g0["radius"], "collar not covered by the gripper capsule"
    # obstacles (boxes) from the MJCF: table top, pedestals, pallet bases
    obst = []
    for g in range(m.ngeom):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
        if n.startswith(("table_top", "a_pedestal", "b_pedestal", "pallet_")) and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
            R = d.geom_xmat[g].reshape(3, 3)
            ext = np.abs(R) @ m.geom_size[g]
            c = d.geom_xpos[g]
            obst.append(dict(name=n, lo=(c - ext).round(5).tolist(), hi=(c + ext).round(5).tolist()))
    # merge pallet parts into one box per pallet
    merged = {}
    for o in obst:
        key = o["name"].split("_slot")[0].split("_border")[0] if o["name"].startswith("pallet_") else o["name"]
        key = "pallet_a" if key.startswith("pallet_a") else ("pallet_b" if key.startswith("pallet_b") else key)
        if key in merged:
            merged[key]["lo"] = np.minimum(merged[key]["lo"], o["lo"]).tolist()
            merged[key]["hi"] = np.maximum(merged[key]["hi"], o["hi"]).tolist()
        else:
            merged[key] = dict(name=key, lo=o["lo"], hi=o["hi"])
    names = [c["name"] for c in caps]
    fr = {c["name"]: c["frame"] for c in caps}
    pairs = [(x, y) for i, x in enumerate(names) for y in names[i + 1:] if abs(fr[x] - fr[y]) >= 2]
    out = dict(note="tools/export_collision.py: capsules in UR5e DH frames (0 base .. 6 tool0), radius incl. 3 mm",
               capsules=caps, obstacles=list(merged.values()), self_pairs=pairs, min_clearance=0.01)
    path = os.path.join(V.REPO, "src/catch_kinematics/config/ur5e_capsules.yaml")
    yaml.safe_dump(out, open(path, "w"), default_flow_style=None, width=200)
    cm = CapsuleModel(path)
    out["camera_mesh_margin"] = camera_mesh_margin(robot, cm, a.n_mesh)
    if out["camera_mesh_margin"]["extra_radius"] > 0:
        for c in caps:
            if c["name"].startswith("f6_cam_"):
                c["radius"] = round(c["radius"] + out["camera_mesh_margin"]["extra_radius"], 5)
        yaml.safe_dump(out, open(path, "w"), default_flow_style=None, width=200)
        cm = CapsuleModel(path)
        out["camera_mesh_margin"]["check_after"] = camera_mesh_margin(robot, cm, a.n_mesh)
    print("camera vs B's arm (URDF meshes):", json.dumps(out["camera_mesh_margin"]))
    # drop self pairs that already touch at the ready pose (adjacent-like geometry, e.g. wrist links)
    base_d = cm.self_distances(robot, q_ready)
    bad = [p for p, dist in zip(cm.pairs, base_d) if dist < 0.0]
    if bad:
        out["self_pairs"] = [p for p in pairs if tuple(p) not in {tuple(b) for b in bad}]
        yaml.safe_dump(out, open(path, "w"), default_flow_style=None, width=200)
        cm = CapsuleModel(path)
    # validation against MuJoCo collisions; A stands at a random pose of its own (home / stroke start / random
    # near home) and its capsules are passed to the check as the live catcher does (extra_capsules)
    rng = np.random.default_rng(0)
    robot_a = V.F.make_robots(scene)["a"]
    poses = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_description/config/poses.yaml")))
    qadr_a = [jid(f"a_{j}") for j in V.JN]
    others = [g for g in range(m.ngeom) if coll(g) and not body(g).startswith(("b_", "cube_"))]
    stats = dict(n=0, mj_coll=0, cap_coll=0, false_free=0, false_alarm=0)
    worst_ff = []
    t_check = []
    ft = np.zeros(6)
    cam_all = [g for gs in cam_geoms.values() for g in gs]
    # what the camera hardware can hit: environment + A (others) and B's links >= 2 frames away from tool0
    cam_vs = others + [g for g in b_geoms if frame_of(m.geom_bodyid[g]) is not None and frame_of(m.geom_bodyid[g]) <= 4]
    stats["cam_coll"] = 0
    for k in range(a.n):
        q = q_ready + rng.normal(0, 0.6, 6)
        qa = np.array(poses["a"]["home"]) + rng.normal(0, 0.3, 6)
        d.qpos[qadr] = q
        d.qpos[qadr_a] = qa
        A0, A1 = cm.segments(robot_a, qa)
        a_caps = (A0[0], A1[0], cm.r)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_collision(m, d)
        mj = False
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if (g1 in b_geoms) != (g2 in b_geoms) or (g1 in b_geoms and g2 in b_geoms):
                b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
                if g1 in b_geoms and g2 in b_geoms:
                    f1, f2 = frame_of(b1), frame_of(b2)
                    if f1 is None or f2 is None or abs(f1 - f2) < 2:
                        continue
                elif (g1 if g1 not in b_geoms else g2) not in others:
                    continue
                if c.dist < a.margin:
                    mj = True
                    break
        if not mj:          # the camera hardware (no contacts: visual geoms): distances, near pairs only
            for g1 in cam_all:
                for g2 in cam_vs:
                    if np.linalg.norm(d.geom_xpos[g1] - d.geom_xpos[g2]) > m.geom_rbound[g1] + m.geom_rbound[g2] + a.margin:
                        continue
                    if mujoco.mj_geomDistance(m, d, g1, g2, 0.05, ft) < a.margin:
                        mj = True
                        stats["cam_coll"] += 1
                        break
                if mj:
                    break
        t0 = time.perf_counter()
        cap = not cm.free(robot, q, env=True, self_check=True, extra_capsules=a_caps)
        t_check.append(time.perf_counter() - t0)
        stats["n"] += 1
        stats["mj_coll"] += mj
        stats["cap_coll"] += cap
        if mj and not cap:
            stats["false_free"] += 1
            pairs_hit = []
            for i in range(d.ncon):
                c = d.contact[i]
                if c.dist < a.margin and (c.geom1 in b_geoms or c.geom2 in b_geoms):
                    pairs_hit.append((body(c.geom1), body(c.geom2), round(float(c.dist), 4)))
            worst_ff.append(dict(q=q.round(3).tolist(), contacts=pairs_hit[:4]))
        if cap and not mj:
            stats["false_alarm"] += 1
    stats["check_ms_median"] = 1000 * float(np.median(t_check))
    print(json.dumps(stats), "capsules:", [(c["name"], c["radius"]) for c in caps])
    for w in worst_ff[:5]:
        print("false free:", w["contacts"])
    os.makedirs(os.path.join(V.OUT, "phase2"), exist_ok=True)
    json.dump(dict(stats=stats, false_free_examples=worst_ff[:10], capsules=caps, obstacles=list(merged.values())),
              open(os.path.join(V.OUT, "phase2", "collision_model.json"), "w"), indent=1)


if __name__ == "__main__":
    main()
