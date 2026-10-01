"""The catch must hold the cube by contact and friction only: no equality constraint (weld/connect) and no joint
other than its free joint may involve a cube body in the generated MJCF. (There is no debug weld in this project.)

    env -u PYTHONPATH .venv/bin/python -m pytest src/catch_description/test/test_no_cube_constraints.py
"""
import os

import pytest

mujoco = pytest.importorskip("mujoco")
PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def test_cubes_are_free_bodies_without_equality_constraints():
    m = mujoco.MjModel.from_xml_path(os.path.join(PKG, "mjcf", "scene.xml"))
    cubes = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"cube_{c}") for c in ("red", "green", "blue")}
    assert -1 not in cubes
    for b in cubes:
        assert m.body_jntnum[b] == 1 and m.jnt_type[m.body_jntadr[b]] == mujoco.mjtJoint.mjJNT_FREE
        assert m.body_parentid[b] == 0                       # child of the world, not of a gripper
    for e in range(m.neq):
        objs = {int(m.eq_obj1id[e]), int(m.eq_obj2id[e])}
        if m.eq_objtype[e] == mujoco.mjtObj.mjOBJ_BODY or m.eq_type[e] in (mujoco.mjtEq.mjEQ_WELD, mujoco.mjtEq.mjEQ_CONNECT):
            assert not (objs & cubes), f"equality {e} ({mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_EQUALITY, e)}) involves a cube"
