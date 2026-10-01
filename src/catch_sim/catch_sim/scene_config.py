"""Scene geometry helpers shared by the sim nodes (numpy only; mirrors scripts/build_mjcf.py)."""
import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory


def load_scene():
    share = get_package_share_directory("catch_description")
    with open(os.path.join(share, "config", "scene.yaml")) as f:
        return yaml.safe_load(f)


def robot_base_xyz(scene, key):
    r = scene["robots"][key]
    return [r["base_xy"][0], r["base_xy"][1], scene["table"]["top_z"] + r.get("pedestal_height", 0.0)]


def slot_position(scene, pallet, color):
    """World position of a cube centre resting in `color` slot of pallet 'a' or 'b'."""
    p = scene["pallets"]
    i = p["slot_order"].index(color)
    cx, cy = p[pallet]["center"]
    return np.array([cx + (i - 1) * p["slot_pitch"], cy, scene["table"]["top_z"] + p["base_size"][2] + scene["cube"]["edge"] / 2])


def tcp_in_gripper_base(scene):
    """Cube-centre position in the MJCF robotiq base frame."""
    return np.array([0.0, 0.0, scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2])


def _look_axes(pos, target, up=(0, 0, 1.0)):
    fwd = np.subtract(target, pos)
    fwd = fwd / np.linalg.norm(fwd)
    x = np.cross(fwd, up)
    x /= np.linalg.norm(x)
    y = np.cross(x, fwd)
    return x, y


def mujoco_camera_rotations(scene):
    """{camera: (parent_frame, position, R_mujoco_cam)} exactly as build_mjcf.py creates them."""
    cams = scene["cameras"]
    out = {}
    for name in ("overhead_a", "overhead_b"):
        out[name] = ("world", np.array(cams[name]["pos"], float), np.eye(3))
    c = cams["side"]
    x, y = _look_axes(c["pos"], c["target"])
    out["side"] = ("world", np.array(c["pos"], float), np.column_stack([x, y, np.cross(x, y)]))
    w = cams["b_wrist"]
    p = np.array(w["pos_in_gripper_base"], float)
    x, y = _look_axes(p, tcp_in_gripper_base(scene))
    out["b_wrist"] = ("b_robotiq_mjcf_base", p, np.column_stack([x, y, np.cross(x, y)]))
    return out


RX_PI = np.diag([1.0, -1.0, -1.0])        # MuJoCo camera (looks along -z, y up) -> ROS optical (z fwd, y down)
# MJCF robotiq base frame expressed in the URDF robotiq_85_base_link frame: Rz(-90 deg)
URDF_T_MJCF_BASE = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
