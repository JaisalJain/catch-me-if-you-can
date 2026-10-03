import shutil
import subprocess

import numpy as np
import pytest

from catch_kinematics import UR5e, UrdfChain, wrap
from catch_kinematics.transforms import make_tf, rot_z, rotation_angle

RNG = np.random.default_rng(0)


def random_q(n):
    q = RNG.uniform(-np.pi, np.pi, size=(n, 6))
    q[:, 2] = RNG.uniform(-2.8, 2.8, size=n)
    return q


def robot():
    return UR5e(world_T_base=make_tf(rot_z(0.3), [-0.8, 0.1, 0.75]), tool0_T_tcp=make_tf(p=[0, 0, 0.14]))


def test_ik_roundtrip():
    r = robot()
    n_ok = 0
    for q in random_q(200):
        T = r.fk(q)
        sols = r.ik_all(T)
        assert sols, "no IK solution for a reachable pose"
        for s in sols:
            Ts = r.fk(s)
            assert np.linalg.norm(Ts[:3, 3] - T[:3, 3]) < 1e-8
            assert rotation_angle(Ts[:3, :3], T[:3, :3]) < 1e-7
        assert min(np.linalg.norm(wrap(s - q)) for s in sols) < 1e-6
        n_ok += 1
    assert n_ok == 200


def test_ik_closest_returns_seed_branch():
    r = robot()
    for q in random_q(50):
        s = r.ik_closest(r.fk(q), q + 0.01)
        np.testing.assert_allclose(s, q, atol=1e-6)


def test_branch_matches_ik_tags():
    r = robot()
    for q in random_q(200):
        for s, b in r.ik_all(r.fk(q), branches=True):
            assert r.branch(s) == b
            assert r.branch(s + 2 * np.pi * np.array([1, 0, 0, -1, 1, 1])) == b   # 2 pi copies: same branch


def test_ik_closest_branch_filter():
    r = robot()
    for q in random_q(50):
        T = r.fk(q)
        for b in {b for _, b in r.ik_all(T, branches=True)}:
            s = r.ik_closest(T, q, check=False, branch=b)
            assert s is not None and r.branch(s) == b
            assert np.linalg.norm(r.fk(s)[:3, 3] - T[:3, 3]) < 1e-8


def test_jacobian_finite_difference():
    r = robot()
    for q in random_q(20):
        J = r.jacobian(q)
        T0 = r.fk(q)
        eps = 1e-7
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = eps
            T1 = r.fk(q + dq)
            np.testing.assert_allclose((T1[:3, 3] - T0[:3, 3]) / eps, J[:3, i], atol=1e-5)
            dR = T1[:3, :3] @ T0[:3, :3].T
            w = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / (2 * eps)
            np.testing.assert_allclose(w, J[3:, i], atol=1e-5)


@pytest.mark.skipif(shutil.which("xacro") is None, reason="needs ROS xacro + ur_description")
def test_dh_matches_ur_description_urdf():
    share = subprocess.check_output(["ros2", "pkg", "prefix", "ur_description"]).decode().strip()
    urdf = subprocess.check_output(["xacro", f"{share}/share/ur_description/urdf/ur.urdf.xacro", "ur_type:=ur5e",
                                    "name:=ur"]).decode()
    chain = UrdfChain(urdf)
    r = UR5e()
    names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
             "wrist_3_joint"]
    for q in random_q(50):
        Tu = chain.link_pose("tool0", dict(zip(names, q)), root="base_link")
        Td = r.fk_tool0(q)
        assert np.linalg.norm(Tu[:3, 3] - Td[:3, 3]) < 1e-6
        assert rotation_angle(Tu[:3, :3], Td[:3, :3]) < 1e-6
