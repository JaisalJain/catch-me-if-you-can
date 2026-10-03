"""2F-85 command model: ramps are strictly increasing in time, start after the latency, move the gap at the model
speed, end at the target."""
import numpy as np
import pytest

from catch_sim import gripper_model as gm


@pytest.mark.parametrize("latency", [0.0, 0.010, 0.030])
@pytest.mark.parametrize("q_from,q_to", [(0.0, 0.70), (0.45, 0.30), (gm.knuckle_for_gap(56.0), 0.70), (0.3, 0.3)])
def test_ramp(latency, q_from, q_to):
    pts = gm.ramp_points(0.70, q_from, q_to, 0.150, latency)
    t = np.array([p[0] for p in pts])
    assert np.all(np.diff(t) > 0)
    assert pts[0] == (0.0, 0.70) and pts[-1][1] == pytest.approx(q_to)
    # nothing moves before the latency: the command stays at the hold value
    assert all(q == 0.70 for tt, q in pts if tt <= latency)
    # gap speed between consecutive ramp points after the step to q_from
    ramp = [p for p in pts if p[0] > latency]
    for (t0, q0), (t1, q1) in zip(ramp[1:], ramp[2:]):
        assert abs(gm.gap_for_knuckle(q1) - gm.gap_for_knuckle(q0)) / 1000 / (t1 - t0) <= 0.150 * 1.001


def test_gap_table_round_trip_and_close_time():
    for g in (85.2, 56.0, 40.0, 8.6):
        assert gm.gap_for_knuckle(gm.knuckle_for_gap(g)) == pytest.approx(g, abs=1e-6)
    # 56 -> 40 mm at 150 mm/s with 10 ms latency: 10 + 2 + 107 ms
    assert gm.time_to_gap(gm.knuckle_for_gap(56.0), 40.0, 0.150, 0.010) == pytest.approx(0.1187, abs=1e-3)


def test_profiles_load():
    p = gm.load_profile("realistic")
    assert p["gap_speed"] == 0.150 and p["latency"] == 0.010


def test_env_override_per_arm(monkeypatch):
    # the benchmark varies only B's gripper: CATCH_GRIPPER_ARMS=b leaves A on the default profile
    monkeypatch.setenv("CATCH_GRIPPER_PROFILE", "fast_idealized")
    monkeypatch.setenv("CATCH_GRIPPER_LATENCY", "0.03")
    monkeypatch.setenv("CATCH_GRIPPER_ARMS", "b")
    assert gm.load_profile(robot="a")["name"] == "realistic"
    assert gm.load_profile(robot="a")["latency"] == 0.010
    assert gm.load_profile(robot="b")["name"] == "fast_idealized"
    assert gm.load_profile(robot="b")["latency"] == 0.03
    monkeypatch.delenv("CATCH_GRIPPER_ARMS")
    assert gm.load_profile(robot="a")["name"] == "fast_idealized"
