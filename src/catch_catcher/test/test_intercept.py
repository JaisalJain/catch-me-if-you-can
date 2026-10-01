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
