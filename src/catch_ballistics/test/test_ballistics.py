import numpy as np
import pytest
from scipy.integrate import solve_ivp

from catch_ballistics import (G, Projectile, intercepts_in_box, min_speed_angle, velocity_for_flight_time,
                              velocity_for_launch_angle)


def test_matches_numerical_integration():
    p0, v0 = [-0.35, 0.1, 1.0], [1.8, 0.05, 1.9]
    traj = Projectile(p0, v0, t0=0.3)
    sol = solve_ivp(lambda t, y: np.r_[y[3:], 0, 0, -G], (0.3, 0.8), np.r_[p0, v0], rtol=1e-11, atol=1e-12,
                    dense_output=True)
    for t in np.linspace(0.3, 0.8, 11):
        np.testing.assert_allclose(traj.position(t), sol.sol(t)[:3], atol=1e-9)
        np.testing.assert_allclose(traj.velocity(t), sol.sol(t)[3:], atol=1e-9)


def test_vectorised_shapes():
    traj = Projectile([0, 0, 0], [1, 0, 1])
    assert traj.position(0.1).shape == (3,)
    assert traj.position(np.array([0.1, 0.2])).shape == (2, 3)
    t, p = traj.sample(0.2, 0.01)
    assert len(t) == 21 and p.shape == (21, 3)


def test_apex():
    traj = Projectile([0, 0, 1.0], [2.0, 0, 3.0])
    assert traj.apex_time == pytest.approx(3.0 / G)
    assert traj.apex[2] == pytest.approx(1.0 + 9.0 / (2 * G))
    assert traj.velocity(traj.apex_time)[2] == pytest.approx(0.0, abs=1e-12)
    down = Projectile([0, 0, 1.0], [1.0, 0, -1.0])
    assert down.apex_time == 0.0


def test_height_crossings():
    traj = Projectile([0, 0, 1.0], [1.0, 0, 2.0], t0=1.0)
    ts = traj.times_at_height(1.1)
    assert len(ts) == 2 and ts[0] < ts[1]
    for t in ts:
        assert traj.position(t)[2] == pytest.approx(1.1)
    assert traj.time_descending_at_height(1.1) == pytest.approx(ts[1])
    assert traj.times_at_height(5.0) == []
    assert traj.time_descending_at_height(5.0) is None
    # landing on the table plane
    t_land = traj.time_at_plane([0, 0, 0.75], [0, 0, 1])
    assert traj.position(t_land)[2] == pytest.approx(0.75)
    assert traj.velocity(t_land)[2] < 0


def test_plane_crossing_vertical_plane():
    traj = Projectile([-0.35, 0, 1.0], [2.0, 0, 2.0])
    t = traj.time_at_plane([0.35, 0, 0], [1, 0, 0])
    assert t == pytest.approx(0.35)
    assert traj.time_at_plane([-1.0, 0, 0], [1, 0, 0]) is None


@pytest.mark.parametrize("T", [0.2, 0.38, 0.6])
def test_velocity_for_flight_time_hits_target(T):
    p, c = np.array([-0.35, 0.13, 1.0]), np.array([0.35, 0.13, 1.05])
    v = velocity_for_flight_time(p, c, T)
    np.testing.assert_allclose(Projectile(p, v).position(T), c, atol=1e-12)


@pytest.mark.parametrize("deg", [30, 45, 60])
def test_velocity_for_launch_angle(deg):
    p, c = np.array([-0.35, 0.0, 1.0]), np.array([0.35, 0.0, 1.0])
    v, T = velocity_for_launch_angle(p, c, np.radians(deg))
    np.testing.assert_allclose(Projectile(p, v).position(T), c, atol=1e-12)
    assert np.degrees(np.arctan2(v[2], np.hypot(v[0], v[1]))) == pytest.approx(deg)


def test_textbook_numbers_level_45deg():
    # level range R = v^2 sin(2 theta)/g
    p, c = np.zeros(3), np.array([0.7, 0, 0])
    v, T = velocity_for_launch_angle(p, c, np.radians(45))
    assert np.linalg.norm(v) == pytest.approx(np.sqrt(0.7 * G))
    assert T == pytest.approx(0.7 / (np.sqrt(0.7 * G) * np.cos(np.radians(45))))
    assert min_speed_angle(p, c) == pytest.approx(np.radians(45))


def test_unreachable_angle():
    with pytest.raises(ValueError):
        velocity_for_launch_angle([0, 0, 0], [0.5, 0, 1.0], np.radians(45))  # target above the 45 deg line
    with pytest.raises(ValueError):
        velocity_for_launch_angle([0, 0, 0], [0, 0, 1.0], np.radians(45))
    with pytest.raises(ValueError):
        velocity_for_flight_time([0, 0, 0], [1, 0, 0], 0.0)


def test_min_speed_angle_is_minimum():
    p, c = np.array([0, 0, 0.0]), np.array([0.7, 0, 0.1])
    a_star = min_speed_angle(p, c)
    s_star = np.linalg.norm(velocity_for_launch_angle(p, c, a_star)[0])
    for da in (-0.05, 0.05):
        assert np.linalg.norm(velocity_for_launch_angle(p, c, a_star + da)[0]) > s_star


def test_intercepts_descending_only():
    traj = Projectile([-0.35, 0, 1.0], [1.85, 0, 1.85])
    cands = intercepts_in_box(traj, [0.2, -0.1, 0.9], [0.5, 0.1, 1.3], t_end=0.6)
    assert cands
    assert all(c.velocity[2] < 0 for c in cands)
    assert all(0.2 <= c.position[0] <= 0.5 for c in cands)
    both = intercepts_in_box(traj, [-0.5, -0.1, 0.9], [0.5, 0.1, 1.3], t_end=0.6, descending_only=False)
    assert len(both) > len(cands)
