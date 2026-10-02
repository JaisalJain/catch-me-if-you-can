"""catch_tracker: the C++ flight tracker, camera parameters derived from scene.yaml (cameras.tracking) — the same
look-at convention as build_mjcf.look_at_axes — and colors from catch_perception/config/perception.yaml.

    ros2 launch catch_tracker tracker.launch.py
"""
import os

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def camera_params(scene):
    """{name: {...}} per tracking camera: pinhole intrinsics and the ROS optical frame (x right, y down, z forward)
    in the world, from the MJCF look-at definition (camera looks along -z, image up = y)."""
    out = {}
    for name, c in scene["cameras"]["tracking"].items():
        pos, tgt = np.array(c["pos"], float), np.array(c["target"], float)
        up = np.array(c.get("up", [0, 0, 1.0]), float)
        fwd = (tgt - pos) / np.linalg.norm(tgt - pos)
        x = np.cross(fwd, up)
        x /= np.linalg.norm(x)
        y_up = np.cross(x, fwd)
        R = np.column_stack([x, -y_up, fwd])            # world <- optical
        W, H = c["resolution"]
        f = (H / 2) / np.tan(np.radians(c["fovy"]) / 2)
        out[name] = {"width": W, "height": H, "fx": f, "fy": f, "cx": W / 2, "cy": H / 2,
                     "R": [float(v) for v in R.flatten()], "t": [float(v) for v in pos],
                     "topic": f"/cameras/{name}/image_raw"}
    return out


def color_params(perception):
    out = {}
    for name, s in perception["colors"].items():
        out[name] = {"h": [int(v) for r in s["h"] for v in r], "s_min": int(s["s_min"]), "v_min": int(s["v_min"])}
    return out


def generate_launch_description():
    desc = get_package_share_directory("catch_description")
    scene = yaml.safe_load(open(os.path.join(desc, "config", "scene.yaml")))
    perception = yaml.safe_load(open(os.path.join(get_package_share_directory("catch_perception"), "config", "perception.yaml")))
    cfg = os.path.join(get_package_share_directory("catch_tracker"), "config", "tracker.yaml")
    cams = camera_params(scene)
    params = {"use_sim_time": True, "cameras": list(cams), "colors": list(perception["colors"])}
    for name, p in cams.items():
        params.update({f"{name}.{k}": v for k, v in p.items()})
    for name, p in color_params(perception).items():
        params.update({f"{name}.{k}": v for k, v in p.items()})
    return LaunchDescription([
        Node(package="catch_tracker", executable="cube_tracker_node", name="cube_tracker", output="both",
             parameters=[cfg, params]),
    ])
