"""The throw configured in config/throw.yaml must stay within the UR5e limits with the M1 design margins, and
the feed-forward table must match it. Needs the built workspace (ament index) — skipped otherwise."""
import numpy as np
import pytest

ament = pytest.importorskip("ament_index_python.packages")


def _have(pkg):
    try:
        ament.get_package_share_directory(pkg)
        return True
    except Exception:  # noqa: BLE001
        return False


pytestmark = pytest.mark.skipif(not (_have("catch_throw") and _have("catch_description")),
                                reason="workspace not built/sourced")


def test_configured_throw_within_limits_with_margin():
    from catch_throw.generator import check_limits
    from catch_throw.throw_node import build_plan
    plan, T, catch, cfg = build_plan()
    assert check_limits(plan, vel_ratio_max=0.85, pos_margin_min=0.15) == []
    # the planned flight reaches the catch point
    np.testing.assert_allclose(plan.flight().position(plan.t_release + T), catch, atol=1e-9)
    # the gripper is loosened before separation, while the arm is still accelerating
    assert 0.0 < cfg["release"]["t_loosen"] < plan.t_release
    assert plan.profile.accel(cfg["release"]["t_loosen"])[0] > 0


def test_feedforward_table_matches_plan():
    from catch_throw.throw_node import build_plan, load_feedforward
    plan, *_ = build_plan()
    ff, why = load_feedforward(len(plan.t))
    assert ff is not None, why
    assert ff.shape == (len(plan.t), 6)
