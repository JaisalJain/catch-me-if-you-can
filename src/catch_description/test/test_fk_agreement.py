"""Integration test: URDF (MoveIt) and MJCF (MuJoCo) forward kinematics agree to < 1 mm and < 1 deg.

Runs in the offline venv (needs the mujoco Python package) with ROS sourced (for xacro):
    env -u PYTHONPATH .venv/bin/python -m pytest src/catch_description/test/test_fk_agreement.py
Compared frames per arm, at 50 random joint configurations within the joint limits:
    tool0  (URDF <p>tool0)               vs MJCF site <p>attachment_site
    TCP    (URDF <p>tcp)                  vs MJCF site <p>robotiq_tcp
    gripper base (URDF <p>robotiq_85_base_link * Rz(-90deg)) vs MJCF body <p>robotiq_base
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
    worst_p, worst_a = 0.0, 0.0
    for _ in range(N_SAMPLES):
        q = rng.uniform(-np.pi, np.pi, 6)
        q[2] = rng.uniform(-3.0, 3.0)
        d.qpos[:] = m.qpos0
        d.qpos[qadr] = q
        mujoco.mj_kinematics(m, d)
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
