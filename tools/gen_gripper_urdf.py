#!/usr/bin/env python3
"""Generate src/catch_description/urdf/robotiq_2f85_catch.xacro: the robotiq_description 2F-85 kinematics,
visuals and inertias, but with the COLLISION geometry of the Menagerie 2F-85 that MuJoCo actually simulates
(pad boxes + convex hulls of the link meshes), each attached to the URDF link it moves rigidly with.

    source /opt/ros/jazzy/setup.bash && source install/setup.bash
    env -u PYTHONPATH .venv/bin/python tools/gen_gripper_urdf.py

The body<->link correspondence and the relative transforms are measured in MuJoCo over the gripper stroke
(knuckle 0..0.7): every Menagerie body stays within 0.31 mm / 1.6 deg of a fixed pose on its URDF link.
Also adds <prefix>left_pad / <prefix>right_pad frames (Menagerie pad bodies) for the FK agreement test.
"""
import glob
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import trimesh

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(REPO, "src", "catch_kinematics"))
from catch_kinematics import UrdfChain  # noqa: E402
from catch_kinematics.transforms import inv_tf, make_tf, matrix_to_quat_wxyz  # noqa: E402

PFX = "PFXX_"
HULL_DIR = os.path.join(REPO, "src/catch_description/meshes/2f85_collision")
OUT = os.path.join(REPO, "src/catch_description/urdf/robotiq_2f85_catch.xacro")
# Menagerie body (MJCF side "right" = URDF side "left", verified in M2) -> URDF link suffix
BODY_TO_LINK = {
    "base_mount": "robotiq_85_base_link", "base": "robotiq_85_base_link",
    "right_driver": "robotiq_85_left_knuckle_link", "right_coupler": "robotiq_85_left_finger_link",
    "right_spring_link": "robotiq_85_left_inner_knuckle_link", "right_follower": "robotiq_85_left_finger_tip_link",
    "right_pad": "robotiq_85_left_finger_tip_link",
    "left_driver": "robotiq_85_right_knuckle_link", "left_coupler": "robotiq_85_right_finger_link",
    "left_spring_link": "robotiq_85_right_inner_knuckle_link", "left_follower": "robotiq_85_right_finger_tip_link",
    "left_pad": "robotiq_85_right_finger_tip_link",
}
PAD_FRAMES = {"right_pad": "left_pad", "left_pad": "right_pad"}   # MJCF pad body -> URDF pad frame name
R_URDF_MJCF_BASE = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def ros_env():
    env = dict(os.environ)
    env["PYTHONPATH"] = ":".join(glob.glob(f"/opt/ros/{env.get('ROS_DISTRO', 'jazzy')}/lib/python3*/site-packages"))
    return env


def robotiq_urdf():
    wrapper = f"""<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="g">
  <xacro:include filename="$(find robotiq_description)/urdf/robotiq_2f_85_macro.urdf.xacro"/>
  <link name="PARENT"/>
  <xacro:robotiq_gripper name="g" prefix="{PFX}" parent="PARENT" include_ros2_control="false">
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </xacro:robotiq_gripper>
</robot>"""
    path = os.path.join(REPO, "log", "robotiq_wrapper.urdf.xacro")
    os.makedirs(os.path.dirname(path), exist_ok=True)
    open(path, "w").write(wrapper)
    return subprocess.check_output(["xacro", path], env=ros_env()).decode()


def package_uri(fn):
    """robotiq_description 0.0.1 writes absolute file://<share>/robotiq_description/... mesh paths; store a
    package:// URI so the URDF does not depend on where the package is installed (resolved by the overlay)."""
    key = "/share/robotiq_description/"
    if fn.startswith("file://") and key in fn:
        fn = "package://robotiq_description/" + fn.split(key, 1)[1]
    if not fn.startswith("package://"):
        raise SystemExit(f"unexpected mesh path {fn}")
    return fn


def rpy_from_R(R):
    """URDF rpy (R = Rz(y) Ry(p) Rx(r))."""
    p = np.arcsin(-np.clip(R[2, 0], -1, 1))
    if abs(np.cos(p)) > 1e-9:
        r = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(R[1, 0], R[0, 0])
    else:
        r, y = np.arctan2(-R[1, 2], R[1, 1]), 0.0
    return r, p, y


def measure(m, d, chain):
    """Mean transform URDF link <- Menagerie body for every body in BODY_TO_LINK, and the worst spread."""
    act = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, "a_robotiq_85_left_knuckle_joint")
    kq = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "a_robotiq_85_left_knuckle_joint")]
    bb = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "a_robotiq_base")
    rels = {b: [] for b in BODY_TO_LINK}
    for target in np.linspace(0.0, 0.7, 15):
        mujoco.mj_resetData(m, d)
        d.ctrl[act] = target
        for _ in range(2000):
            mujoco.mj_step(m, d)
        k = d.qpos[kq]
        Tb = make_tf(d.xmat[bb].reshape(3, 3), d.xpos[bb])
        q = {f"{PFX}robotiq_85_left_knuckle_joint": k}
        Tub = chain.link_pose(f"{PFX}robotiq_85_base_link", q)
        for body, link in BODY_TO_LINK.items():
            i = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"a_robotiq_{body}")
            T_mb = make_tf(R_URDF_MJCF_BASE) @ inv_tf(Tb) @ make_tf(d.xmat[i].reshape(3, 3), d.xpos[i])
            T_ul = inv_tf(Tub) @ chain.link_pose(f"{PFX}{link}", q)
            rels[body].append(inv_tf(T_ul) @ T_mb)
    out, worst = {}, (0.0, 0.0)
    for body, Ts in rels.items():
        P = np.array([T[:3, 3] for T in Ts])
        p = P.mean(axis=0)
        # mean rotation: orthonormalise the average matrix
        U, _, Vt = np.linalg.svd(np.mean([T[:3, :3] for T in Ts], axis=0))
        R = U @ Vt
        spread = float(np.max(np.linalg.norm(P - p, axis=1)))
        ang = float(max(np.arccos(np.clip((np.trace(R.T @ T[:3, :3]) - 1) / 2, -1, 1)) for T in Ts))
        worst = (max(worst[0], spread), max(worst[1], ang))
        out[body] = make_tf(R, p)
    return out, worst


def collision_elements(m, body_tf):
    """<collision> XML snippets per URDF link from the MJCF geoms of each Menagerie body (side 'a')."""
    os.makedirs(HULL_DIR, exist_ok=True)
    per_link = {}
    for body, T_lb in body_tf.items():
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"a_robotiq_{body}")
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != bid or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                continue
            if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) == "a_robotiq_palm_insert":
                continue   # the URDF has its own palm_insert link (same box)
            T_bg = make_tf(_quat_mat(m.geom_quat[g]), m.geom_pos[g])
            T = T_lb @ T_bg
            r, p_, y = rpy_from_R(T[:3, :3])
            origin = f'<origin xyz="{T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f}" rpy="{r:.6f} {p_:.6f} {y:.6f}"/>'
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX:
                s = 2 * m.geom_size[g]
                geo = f'<box size="{s[0]:.6f} {s[1]:.6f} {s[2]:.6f}"/>'
            elif m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mid = m.geom_dataid[g]
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, mid).replace("a_robotiq_", "")
                va, vn = m.mesh_vertadr[mid], m.mesh_vertnum[mid]
                hull = trimesh.Trimesh(m.mesh_vert[va:va + vn]).convex_hull   # MuJoCo collides with the hull
                hull.export(os.path.join(HULL_DIR, f"{name}_hull.stl"))
                geo = f'<mesh filename="package://catch_description/meshes/2f85_collision/{name}_hull.stl"/>'
            else:
                raise NotImplementedError(m.geom_type[g])
            per_link.setdefault(BODY_TO_LINK[body], []).append(f"<collision>{origin}<geometry>{geo}</geometry></collision>")
    return per_link


def _quat_mat(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, np.asarray(q, float))
    return R.reshape(3, 3)


def main():
    urdf = robotiq_urdf()
    chain = UrdfChain(urdf)
    m = mujoco.MjModel.from_xml_path(os.path.join(REPO, "src/catch_description/mjcf/scene.xml"))
    d = mujoco.MjData(m)
    body_tf, worst = measure(m, d, chain)
    print(f"worst spread of a Menagerie body on its URDF link over the stroke: {worst[0] * 1000:.3f} mm, "
          f"{np.degrees(worst[1]):.3f} deg")
    cols = collision_elements(m, body_tf)
    root = ET.fromstring(urdf)
    lines = ['<?xml version="1.0"?>',
             "<!-- GENERATED by tools/gen_gripper_urdf.py: robotiq_description 2F-85 kinematics/visuals/inertias with the",
             "     Menagerie (MuJoCo) collision geometry. Do not edit by hand. -->",
             '<robot xmlns:xacro="http://wiki.ros.org/xacro">',
             '  <xacro:macro name="catch_gripper" params="prefix parent *origin">']
    for link in root.findall("link"):
        name = link.get("name")
        if not name.startswith(PFX):
            continue
        for c in link.findall("collision"):
            link.remove(c)
        for mesh in link.iter("mesh"):
            mesh.set("filename", package_uri(mesh.get("filename")))
        body = ET.tostring(link, encoding="unicode").replace(PFX, "${prefix}")
        suffix = name[len(PFX):]
        inner = "".join(cols.get(suffix, []))
        body = body.replace("</link>", inner + "</link>") if "</link>" in body else body.replace("/>", f">{inner}</link>", 1)
        lines.append("    " + body.strip())
    for j in root.findall("joint"):
        if j.find("parent").get("link") == "PARENT":
            lines.append(f'    <joint name="${{prefix}}robotiq_85_base_joint" type="fixed"><parent link="${{parent}}"/>'
                         f'<child link="{j.find("child").get("link").replace(PFX, "${prefix}")}"/>'
                         f'<xacro:insert_block name="origin"/></joint>')
            continue
        lines.append("    " + ET.tostring(j, encoding="unicode").replace(PFX, "${prefix}").strip())
    for body, frame in PAD_FRAMES.items():
        T = body_tf[body]
        r, p_, y = rpy_from_R(T[:3, :3])
        lines.append(f'    <link name="${{prefix}}{frame}"/><joint name="${{prefix}}{frame}_joint" type="fixed">'
                     f'<parent link="${{prefix}}{BODY_TO_LINK[body]}"/><child link="${{prefix}}{frame}"/>'
                     f'<origin xyz="{T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f}" rpy="{r:.6f} {p_:.6f} {y:.6f}"/></joint>')
    lines += ["  </xacro:macro>", "</robot>"]
    open(OUT, "w").write("\n".join(lines) + "\n")
    print("wrote", OUT, "and", len(os.listdir(HULL_DIR)), "hull meshes")


if __name__ == "__main__":
    main()
