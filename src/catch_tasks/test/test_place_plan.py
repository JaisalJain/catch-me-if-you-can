"""Corrected place geometry (skills.place_tcp_pose): pure numpy, no ROS graph needed."""
import os

import numpy as np
import pytest
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", ".."))
pytest.importorskip("moveit_msgs")                    # skills imports ROS messages: ROS must be sourced
from catch_kinematics.transforms import make_tf, rot_x  # noqa: E402
from catch_tasks.skills import R_DOWN, place_tcp_pose  # noqa: E402

CFG = yaml.safe_load(open(os.path.join(REPO, "src/catch_tasks/config/skills.yaml")))
SCENE = yaml.safe_load(open(os.path.join(REPO, "src/catch_description/config/scene.yaml")))
SLOT = np.array([0.45, -0.28, 0.75 + 0.012 + 0.02])


@pytest.mark.parametrize("th_deg,dy,dz", [(0, 0, 0.004), (3, -0.003, 0.004), (-12.6, 0.018, 0.005), (30, 0, 0)])
def test_corrected_place_puts_the_cube_square_and_clear(th_deg, dy, dz):
    T = make_tf(rot_x(np.radians(th_deg)), [0, dy, dz])
    Rt, p_tcp, target, tilt = place_tcp_pose(CFG, SCENE, SLOT, T, R_DOWN)
    T_cube = make_tf(Rt, p_tcp) @ T
    # cube axes aligned with the world (yaw 0, no tilt) and its centre straight above the slot
    assert np.allclose(np.abs(T_cube[:3, :3]).max(axis=0), 1.0, atol=1e-9)
    assert np.allclose(T_cube[:2, 3], SLOT[:2], atol=1e-9)
    # tool tilt equals the in-hand angle (folded) and the tool points down
    assert abs(np.degrees(tilt) - abs(th_deg)) < 1e-6 and Rt[2, 2] < 0
    # cube bottom above the pallet, every fingertip corner above the slot borders
    surface = SLOT[2] - SCENE["cube"]["edge"] / 2
    assert T_cube[2, 3] - SCENE["cube"]["edge"] / 2 >= surface + CFG["place_clearance"] - 1e-9
    tips = [[sx * CFG["finger_half_span"], sy * CFG["finger_half_width"], CFG["pad_reach"]] for sx in (-1, 1) for sy in (-1, 1)]
    z_tips = (Rt @ np.array(tips).T)[2] + p_tcp[2]
    assert z_tips.min() >= surface + SCENE["pallets"]["border_height"] + CFG["border_margin"] - 1e-9
