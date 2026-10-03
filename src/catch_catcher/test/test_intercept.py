import numpy as np

from catch_ballistics import Projectile
from catch_catcher.intercept import choose_intercept, elbow_up
from catch_kinematics import UR5e
from catch_kinematics.transforms import make_tf, rot_z, rotation_angle

B = UR5e(make_tf(rot_z(np.pi), [0.8, 0.0, 1.10]), make_tf(p=[0, 0, 0.1408]))
HOME = np.array([-0.3907, -1.0556, 2.5894, -3.1046, -1.5708, 2.7509])
P_R, V_R = np.array([-0.05, 0.25, 1.25]), np.array([0.9052, 0.0, 1.9412])


def test_plan_catch_point_cup_pose():
    T = 0.44189444
    ic = choose_intercept(B, P_R, V_R, HOME, mode="plan_catch_point", flight_time=T)
    assert ic is not None
    np.testing.assert_allclose(ic.p, Projectile(P_R, V_R).position(T))
    Tq = B.fk(ic.q)
    np.testing.assert_allclose(Tq[:3, 3], ic.p, atol=1e-9)
    # palm normal (tool z) anti-parallel to the incoming velocity, fingers along world y
    assert np.dot(Tq[:3, 2], -ic.v / np.linalg.norm(ic.v)) > 1 - 1e-9
    assert abs(abs(Tq[:3, 0] @ [0, 1, 0]) - 1) < 1e-9
    assert elbow_up(B, ic.q)


def test_search_mode_descending_and_in_window():
    ic = choose_intercept(B, P_R, V_R, HOME, mode="search", z_window=(1.0, 1.35))
    assert ic is not None and ic.v[2] < 0 and 1.0 <= ic.p[2] <= 1.35
    assert rotation_angle(B.fk(ic.q)[:3, :3], ic.R) < 1e-9


def test_unreachable_returns_none():
    assert choose_intercept(B, [-0.6, 0.25, 1.2], [0.1, 0, 0.5], HOME, mode="search", z_window=(1.0, 1.35)) is None


def test_sample_times_strictly_increasing():
    # T a multiple of dt (float arange ends at ~T): the JTC rejected such trajectories (duplicate point times)
    from catch_catcher.reactive import sample_times
    for T in (0.29, 0.11, 0.05, 0.3, 0.123, 0.2900000001, 0.2899999):
        ts = sample_times(T, 0.01)
        assert ts[0] == 0.0 and ts[-1] == T
        assert all(b - a >= 1e-3 - 1e-12 for a, b in zip(ts, ts[1:])), (T, ts[-3:])


def _planner(ik_branch, q_ready):
    import yaml
    import os
    from catch_catcher.reactive import ReactivePlanner
    from catch_kinematics.collision import CapsuleModel
    from catch_kinematics.dynamics import ArmDynamics
    cfg = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "reactive.yaml")))
    cfg["ik_branch"] = ik_branch
    return ReactivePlanner(B, ArmDynamics(), CapsuleModel(), cfg, q_ready, 60)


def test_ik_branch_fixed_keeps_the_ready_branch():
    """fixed: every goal is on the ready pose's (shoulder, elbow, wrist) branch, also when the closest solution to
    the reference configuration is on another one; free: the closest solution."""
    from catch_catcher.reactive import ready_rotation
    q_ready = np.array([-0.88, -0.11, 1.89, -3.0, 1.18, 3.95])
    R = ready_rotation(60)
    fixed, free = _planner("fixed", q_ready), _planner("free", q_ready)
    assert fixed.branch == B.branch(q_ready) and free.branch is None
    p = B.fk(q_ready)[:3, 3] + [0.03, -0.02, 0.04]
    q_other = next(s for s, b in B.ik_all(make_tf(R, p), branches=True) if b != B.branch(q_ready) and b[2] != B.branch(q_ready)[2])
    for ref in (q_ready, q_other):
        q = B.ik_closest(make_tf(R, p), ref, branch=fixed.branch)
        assert B.branch(q) == B.branch(q_ready)
    assert B.branch(B.ik_closest(make_tf(R, p), q_other)) != B.branch(q_ready)    # free from that reference: other branch
    v = np.array([0.9, 0.0, -1.5])
    q = fixed.goal(p, v, "pinch", q_ready)
    assert q is not None and B.branch(q) == B.branch(q_ready)
    try:
        _planner("sometimes", q_ready)
        assert False, "invalid ik_branch accepted"
    except ValueError:
        pass


def test_resolve_config_selects_ready_pose_and_branch():
    import os
    import yaml
    from catch_catcher.reactive import resolve_config
    raw = yaml.safe_load(open(os.path.join(os.path.dirname(__file__), "..", "config", "reactive.yaml")))
    c = resolve_config(raw, env={})
    assert c["ready"] == raw["ready_poses"][raw["ready_pose"]] and c["ik_branch"] == raw["ik_branch"]
    for name in raw["ready_poses"]:
        c = resolve_config(raw, env={"CATCH_READY_POSE": name, "CATCH_IK_BRANCH": "fixed"})
        assert c["ready"] == raw["ready_poses"][name] and c["ik_branch"] == "fixed"
    try:
        resolve_config(raw, env={"CATCH_READY_POSE": "nope"})
        assert False
    except ValueError:
        pass
