"""Integration test: URDF (MoveIt) and MJCF (MuJoCo) forward kinematics agree to < 1 mm and < 1 deg.

Runs in the offline venv (needs the mujoco Python package) with ROS sourced (for xacro):
    env -u PYTHONPATH .venv/bin/python -m pytest src/catch_description/test/test_fk_agreement.py
Compared frames per arm, at 50 random joint configurations within the joint limits:
    tool0  (URDF <p>tool0)               vs MJCF site <p>attachment_site
    TCP    (URDF <p>tcp)                  vs MJCF site <p>robotiq_tcp
    gripper base (URDF <p>robotiq_85_base_link * Rz(-90deg)) vs MJCF body <p>robotiq_base
    B's wrist camera (URDF b_wrist_cam_optical * diag(1,-1,-1)) vs MJCF camera b_robotiq_wrist_cam
"""
import glob
import os
import subprocess
import sys

import numpy as np
import pytest

mujoco = pytest.importorskip("mujoco")

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.abspath(os.path.join(PKG, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src", "catch_kinematics"))
from catch_kinematics import UrdfChain  # noqa: E402
from catch_kinematics.transforms import make_tf, rot_z, rotation_angle  # noqa: E402

JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
N_SAMPLES, POS_TOL, ANG_TOL = 50, 1e-3, np.radians(1.0)


def ros_env():
    env = dict(os.environ)
    distro = env.get("ROS_DISTRO", "jazzy")
    env["PYTHONPATH"] = ":".join(glob.glob(f"/opt/ros/{distro}/lib/python3*/site-packages"))
    return env


@pytest.fixture(scope="module")
def models():
    urdf = subprocess.check_output(["xacro", os.path.join(PKG, "urdf", "scene.urdf.xacro")], env=ros_env()).decode()
    m = mujoco.MjModel.from_xml_path(os.path.join(PKG, "mjcf", "scene.xml"))
    return UrdfChain(urdf), m, mujoco.MjData(m)


def _mj_pose(m, d, kind, name):
    if kind == "camera":
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, name)
        assert i >= 0, name
        return make_tf(d.cam_xmat[i].reshape(3, 3), d.cam_xpos[i])
    if kind == "site":
        i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, name)
        assert i >= 0, name
        return make_tf(d.site_xmat[i].reshape(3, 3), d.site_xpos[i])
    i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, name)
    assert i >= 0, name
    return make_tf(d.xmat[i].reshape(3, 3), d.xpos[i])


@pytest.mark.parametrize("p", ["a", "b"])
def test_fk_agreement(models, p):
    chain, m, d = models
    rng = np.random.default_rng(42 if p == "a" else 43)
    qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{p}_{j}")] for j in JN]
    pairs = [(f"{p}_tool0", np.eye(4), "site", f"{p}_attachment_site"),
             (f"{p}_tcp", np.eye(4), "site", f"{p}_robotiq_tcp"),
             (f"{p}_robotiq_85_base_link", make_tf(rot_z(-np.pi / 2)), "body", f"{p}_robotiq_base")]
    if p == "b":   # ROS optical frame (x right, y down, z forward) vs MuJoCo camera (x right, y up, looks along -z)
        pairs.append(("b_wrist_cam_optical", make_tf(np.diag([1.0, -1.0, -1.0])), "camera", "b_robotiq_wrist_cam"))
    worst_p, worst_a = 0.0, 0.0
    for _ in range(N_SAMPLES):
        q = rng.uniform(-np.pi, np.pi, 6)
        q[2] = rng.uniform(-3.0, 3.0)
        d.qpos[:] = m.qpos0
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        mujoco.mj_camlight(m, d)      # camera poses are not part of mj_kinematics
        qd = {f"{p}_{j}": v for j, v in zip(JN, q)}
        for link, offset, kind, mjname in pairs:
            Tu = chain.link_pose(link, qd) @ offset
            Tm = _mj_pose(m, d, kind, mjname)
            dp = np.linalg.norm(Tu[:3, 3] - Tm[:3, 3])
            da = rotation_angle(Tu[:3, :3], Tm[:3, :3])
            worst_p, worst_a = max(worst_p, dp), max(worst_a, da)
            assert dp < POS_TOL, f"{link}: {dp * 1000:.3f} mm at q={np.round(q, 3)}"
            assert da < ANG_TOL, f"{link}: {np.degrees(da):.3f} deg at q={np.round(q, 3)}"
    print(f"\n[{p}] worst position error {worst_p * 1000:.6f} mm, worst angle {np.degrees(worst_a):.6f} deg "
          f"over {N_SAMPLES} configs x {len(pairs)} frames")


GRIPPER_ANGLES = (0.0, 0.25, 0.45, 0.7)


def _urdf_collisions(urdf_xml, prefix):
    """{link: [(kind, data, T_link_geom)]} for the gripper links of one arm."""
    import xml.etree.ElementTree as ET
    from catch_kinematics.transforms import rpy_to_matrix
    out = {}
    for link in ET.fromstring(urdf_xml).findall("link"):
        name = link.get("name")
        if not name.startswith(f"{prefix}_robotiq_85"):
            continue
        for c in link.findall("collision"):
            o = c.find("origin")
            T = make_tf(rpy_to_matrix(*[float(v) for v in o.get("rpy").split()]), [float(v) for v in o.get("xyz").split()])
            g = c.find("geometry")[0]
            data = g.get("size") if g.tag == "box" else os.path.basename(g.get("filename"))
            out.setdefault(name, []).append((g.tag, data, T))
    return out


@pytest.mark.parametrize("p", ["a", "b"])
def test_gripper_pads_and_collision_geometry_agree(models, p):
    """Pad frames and every gripper collision vertex (pad boxes, link hulls): URDF vs MJCF < 1 mm / 1 deg over
    the gripper stroke (MJCF fingers settled under the actuator; URDF evaluated at the same knuckle angle)."""
    chain, m, d = models
    urdf = subprocess.check_output(["xacro", os.path.join(PKG, "urdf", "scene.urdf.xacro")], env=ros_env()).decode()
    cols = _urdf_collisions(urdf, p)
    act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{p}_robotiq_85_left_knuckle_joint")
    kq = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{p}_robotiq_85_left_knuckle_joint")]
    side = {"right": "left", "left": "right"}       # MJCF side -> URDF side
    link_of = {"driver": "knuckle", "coupler": "finger", "spring_link": "inner_knuckle", "follower": "finger_tip",
               "pad": "finger_tip"}
    worst_pad = (0.0, 0.0)
    worst_vert = 0.0
    for target in GRIPPER_ANGLES:
        mujoco.mj_resetData(m, d)
        d.ctrl[act] = target
        for _ in range(2000):
            mujoco.mj_step(m, d)
        mujoco.mj_kinematics(m, d)   # after mj_step, xpos lags qpos by one step (the free arm is still moving)
        q = {f"{p}_{j}": d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"{p}_{j}")]] for j in JN}
        q[f"{p}_robotiq_85_left_knuckle_joint"] = d.qpos[kq]
        for ms in ("right", "left"):
            Tm = _mj_pose(m, d, "body", f"{p}_robotiq_{ms}_pad")
            Tu = chain.link_pose(f"{p}_{side[ms]}_pad", q)
            dp, da = np.linalg.norm(Tu[:3, 3] - Tm[:3, 3]), rotation_angle(Tu[:3, :3], Tm[:3, :3])
            worst_pad = (max(worst_pad[0], dp), max(worst_pad[1], da))
            assert dp < POS_TOL and da < ANG_TOL, f"{p} {ms}_pad at knuckle {d.qpos[kq]:.3f}: {dp * 1000:.2f} mm {np.degrees(da):.2f} deg"
        # every MJCF gripper collision geom must have a URDF twin whose vertices coincide within 1 mm
        for g in range(m.ngeom):
            bname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) or ""
            gname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or ""
            if not bname.startswith(f"{p}_robotiq_") or not (m.geom_contype[g] or m.geom_conaffinity[g]) \
                    or gname.endswith("palm_insert"):
                continue
            Tg = make_tf(d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g])
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                s = m.geom_size[g]
                local = np.array([[sx, sy, sz] for sx in (-s[0], s[0]) for sy in (-s[1], s[1]) for sz in (-s[2], s[2])])
                key = ("box", " ".join(f"{2 * v:.6f}" for v in s))
            else:
                mid = m.geom_dataid[g]
                local = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]]
                key = ("mesh", mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid).replace(f"{p}_robotiq_", "") + "_hull.stl")
            pts_m = (Tg[:3, :3] @ local.T).T + Tg[:3, 3]
            rest = bname[len(f"{p}_robotiq_"):]
            if rest in ("base", "base_mount"):
                link = f"{p}_robotiq_85_base_link"
            else:
                s_, part = rest.split("_", 1)
                link = f"{p}_robotiq_85_{side[s_]}_{link_of[part]}_link"
            Tl = chain.link_pose(link, q)
            best = np.inf
            for kind, data, Tlg in cols[link]:
                if (kind, data) != key:
                    continue
                Tw = Tl @ Tlg
                pts_u = (Tw[:3, :3] @ local.T).T + Tw[:3, 3]
                best = min(best, float(np.max(np.linalg.norm(pts_u - pts_m, axis=1))))
            worst_vert = max(worst_vert, best)
            assert best < POS_TOL, f"{gname or bname}: URDF collision vertices {best * 1000:.2f} mm from MJCF"
    print(f"\n[{p}] gripper: worst pad frame {worst_pad[0] * 1000:.3f} mm / {np.degrees(worst_pad[1]):.3f} deg, "
          f"worst collision vertex {worst_vert * 1000:.3f} mm over knuckle {GRIPPER_ANGLES}")


def test_wrist_camera_housing_and_bracket_agree(models):
    """B's wrist-camera housing and its bracket (visual-only boxes on the MJCF gripper base): every box corner of
    the URDF visual twin within 1 mm of the MJCF geom, at random arm configurations."""
    import xml.etree.ElementTree as ET
    from catch_kinematics.transforms import rpy_to_matrix
    chain, m, d = models
    urdf = subprocess.check_output(["xacro", os.path.join(PKG, "urdf", "scene.urdf.xacro")], env=ros_env()).decode()
    boxes = []                    # (link, size, T_link_geom) of the URDF visuals
    for link in ET.fromstring(urdf).findall("link"):
        if link.get("name") not in ("b_wrist_cam_housing", "b_wrist_cam_bracket"):
            continue
        for v in link.findall("visual"):
            o = v.find("origin")
            xyz = [float(x) for x in o.get("xyz", "0 0 0").split()] if o is not None else [0.0] * 3
            rpy = [float(x) for x in o.get("rpy", "0 0 0").split()] if o is not None else [0.0] * 3
            size = np.array([float(x) for x in v.find("geometry/box").get("size").split()])
            boxes.append((link.get("name"), size, make_tf(rpy_to_matrix(*rpy), xyz)))
    geoms = [g for g in range(m.ngeom) if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or "").startswith(
        ("b_robotiq_wrist_cam_housing", "b_robotiq_wrist_cam_bracket"))]
    assert len(geoms) == len(boxes) >= 2
    qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"b_{j}")] for j in JN]
    rng = np.random.default_rng(44)
    worst = 0.0
    for _ in range(10):
        q = rng.uniform(-np.pi, np.pi, 6)
        d.qpos[:] = m.qpos0
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
        qd = {f"b_{j}": v for j, v in zip(JN, q)}
        for g in geoms:
            s = m.geom_size[g]
            corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
            pts_m = (d.geom_xmat[g].reshape(3, 3) @ (corners * s).T).T + d.geom_xpos[g]
            best = np.inf
            for link, size, Tlg in boxes:
                Tw = chain.link_pose(link, qd) @ Tlg
                for perm in ((0, 1, 2), (1, 0, 2), (0, 2, 1), (2, 1, 0), (1, 2, 0), (2, 0, 1)):  # box axes may be permuted
                    if not np.allclose(size[list(perm)], 2 * s, atol=1e-6):
                        continue
                    pts_u = (Tw[:3, :3] @ (corners * size / 2).T).T + Tw[:3, 3]
                    # same corner set, any order
                    dist = max(np.min(np.linalg.norm(pts_u - p, axis=1)) for p in pts_m)
                    best = min(best, float(dist))
            worst = max(worst, best)
            assert best < POS_TOL, f"{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)}: {best * 1000:.2f} mm"
    print(f"\n[b] wrist camera housing + bracket: worst corner {worst * 1000:.4f} mm ({len(geoms)} boxes)")
