"""numpy RNEA (catch_kinematics.dynamics) == MuJoCo mj_inverse on the same model (fixture from
tools/export_dynamics.py), with and without gravity consistency."""
import json
import os

import numpy as np

from catch_kinematics.dynamics import ArmDynamics

HERE = os.path.dirname(os.path.abspath(__file__))


def test_rnea_matches_mujoco_inverse():
    dyn = ArmDynamics()
    fx = json.load(open(os.path.join(HERE, "dynamics_fixture.json")))
    err = max(np.max(np.abs(dyn.rnea(np.array(f["q"]), np.array(f["qd"]), np.array(f["qdd"])) - np.array(f["tau"])))
              for f in fx)
    assert err < 1e-6, err


def test_gravity_only_difference():
    dyn = ArmDynamics()
    q = np.array([0.3, -1.0, 1.5, -2.0, 1.2, 0.4])
    z = np.zeros(6)
    g = dyn.rnea(q, z, z, gravity=True)
    assert np.allclose(dyn.rnea(q, z, z, gravity=False), 0.0)
    qd, qdd = np.full(6, 0.7), np.full(6, 3.0)
    assert np.allclose(dyn.rnea(q, qd, qdd, True) - dyn.rnea(q, qd, qdd, False), g)


def test_batch_equals_single():
    dyn = ArmDynamics()
    rng = np.random.default_rng(3)
    Q, QD, QDD = rng.uniform(-3, 3, (7, 6)), rng.uniform(-2, 2, (7, 6)), rng.uniform(-20, 20, (7, 6))
    for grav in (True, False):
        single = np.array([dyn.rnea(q, qd, qdd, grav) for q, qd, qdd in zip(Q, QD, QDD)])
        assert np.allclose(dyn.torques(Q, QD, QDD, grav), single, atol=1e-9)
