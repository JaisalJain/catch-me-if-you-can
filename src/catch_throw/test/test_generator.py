import numpy as np
import pytest

from catch_ballistics import velocity_for_launch_angle
from catch_kinematics import UR5e
from catch_kinematics.transforms import make_tf, rot_z, rotation_angle
from catch_throw.generator import (InfeasibleThrow, check_limits, generate_throw, pitch_axis, release_joint_velocity,
                                   toss_orientation)
from catch_throw.profile import G, ProfileParams, TossProfile


@pytest.mark.parametrize("deg", [45, 65, 75])
def test_profile_invariants(deg):
    nz = np.sin(np.radians(deg))
    p = ProfileParams(accel=12, tau1=0.08, tau3=0.04, decel=25, tau5=0.08)
    pr = TossProfile(2.2, nz, p)
    assert pr.vel(pr.t_sep) == pytest.approx(2.2, abs=2e-4)
    assert pr.accel(pr.t_sep)[0] == pytest.approx(-G * nz, abs=1e-6)
    assert abs(pr.vel(pr.duration)) < 2e-4
    assert pr.accel(0.0)[0] == pytest.approx(0.0, abs=1e-9)
    # continuity of acceleration (no jumps > what one grid step allows)
    t = np.linspace(0, pr.duration, 5001)
    assert np.max(np.abs(np.diff(pr.accel(t)))) < 0.5
    # before separation the palm pushes the cube (a > -g n_z), after it falls behind
    assert np.all(pr.accel(t[t < pr.t_sep - 1e-4]) > -G * nz)
    assert np.all(pr.accel(t[(t > pr.t_sep + 1e-4) & (t < pr.knots[4])]) < -G * nz)


def test_profile_rejects_bad_params():
    with pytest.raises(ValueError):
        TossProfile(2.0, 0.9, ProfileParams(decel=5.0))        # cannot separate: decel < g n_z
    with pytest.raises(ValueError):
        TossProfile(0.1, 0.7, ProfileParams())                   # too slow for the ramps


def robot():
    return UR5e(make_tf(rot_z(0.0), [-0.8, 0.0, 0.95]), make_tf(p=[0, 0, 0.1408]))


def plan_fixture(kappa=-1.0):
    r = robot()
    p = np.array([-0.15, 0.35, 1.15])
    v, _ = velocity_for_launch_angle(p, [0.35, 0.35, 1.15], np.radians(65))
    q_seed = [-0.2, -1.2, 1.6, -2.0, -1.57, 0.0]
    n = v / np.linalg.norm(v)
    q_r = min(r.ik_all(make_tf(toss_orientation(n), p)),
              key=lambda q: np.max(np.abs(release_joint_velocity(r, q, n, np.linalg.norm(v), kappa))))
    return r, p, v, generate_throw(r, p, v, q_r, ProfileParams(), pitch_rate=kappa)


def test_release_state_and_kinematic_consistency():
    r, p, v, pl = plan_fixture()
    i = int(np.argmin(np.abs(pl.t - pl.t_release)))
    # the sample nearest t_release is within one dt of the release point
    assert np.linalg.norm(r.fk(pl.q[i])[:3, 3] - p) < np.linalg.norm(v) * 0.002
    # FK of every sample lies on the commanded line
    for k in range(0, len(pl.t), 7):
        np.testing.assert_allclose(r.fk(pl.q[k])[:3, 3], pl.tcp[k], atol=1e-8)
    # J qd reproduces the Cartesian twist (velocity along n, spin about the pitch axis)
    n = v / np.linalg.norm(v)
    tw = r.jacobian(pl.q[i]) @ pl.qd[i]
    sd = pl.profile.vel(pl.t[i])
    np.testing.assert_allclose(tw[:3], sd * n, atol=1e-9)
    np.testing.assert_allclose(tw[3:], -1.0 * sd * pitch_axis(n), atol=1e-9)
    # palm normal (tool z) equals the release direction at separation
    Rs = r.fk(pl.q[i])[:3, :3]
    assert rotation_angle(Rs, toss_orientation(n)) < abs(1.0 * np.linalg.norm(v) * 0.002) + 1e-6


def test_joint_derivatives_match_finite_differences():
    _, _, _, pl = plan_fixture()
    dt = pl.t[1] - pl.t[0]
    fd_qd = np.gradient(pl.q, dt, axis=0)
    fd_qdd = np.gradient(pl.qd, dt, axis=0)
    inner = slice(5, -5)
    assert np.max(np.abs(fd_qd[inner] - pl.qd[inner])) < 0.02
    assert np.max(np.abs(fd_qdd[inner] - pl.qdd[inner])) < 2.0   # ramps make FD of qd noisy at knots


def test_limit_checks_flag_violations():
    r, p, v, pl = plan_fixture()
    assert pl.metrics["vel_ratio_max"] == pytest.approx(np.max(np.abs(pl.qd) / np.pi))
    assert check_limits(pl, vel_ratio_max=10.0, pos_margin_min=0.0) == []
    errs = check_limits(pl, vel_ratio_max=0.01, pos_margin_min=10.0, acc_limit=[0.01] * 6)
    assert len(errs) == 3


def test_unreachable_release_raises():
    r = robot()
    with pytest.raises(InfeasibleThrow):
        generate_throw(r, [1.5, 0.0, 1.2], [1.0, 0, 1.0], np.zeros(6), ProfileParams())
