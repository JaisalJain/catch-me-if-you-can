"""URDF sanity: xacro expands, check_urdf parses it, expected frames/joints exist, TCP offset matches scene.yaml."""
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def expand():
    return subprocess.check_output(["xacro", os.path.join(PKG, "urdf", "scene.urdf.xacro")]).decode()


@pytest.fixture(scope="module")
def urdf():
    return expand()


def test_check_urdf_parses(urdf, tmp_path):
    f = tmp_path / "scene.urdf"
    f.write_text(urdf)
    out = subprocess.run(["check_urdf", str(f)], capture_output=True, text=True)
    assert out.returncode == 0, out.stderr
    assert "Successfully Parsed" in out.stdout


def test_expected_frames_and_joints(urdf):
    root = ET.fromstring(urdf)
    links = {l.get("name") for l in root.findall("link")}
    joints = {j.get("name"): j for j in root.findall("joint")}
    for p in ("a_", "b_"):
        for l in ("base_link", "tool0", "tcp", "robotiq_85_base_link", "palm_insert"):
            assert p + l in links
        for j in ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint",
                  "wrist_3_joint", "robotiq_85_left_knuckle_joint"):
            assert joints[p + j].get("type") == "revolute"


def test_tcp_offset_matches_scene_yaml(urdf):
    scene = yaml.safe_load(open(os.path.join(PKG, "config", "scene.yaml")))
    expect = scene["gripper"]["mount_offset"] + scene["gripper"]["palm_insert"]["top_z"] + scene["cube"]["edge"] / 2
    root = ET.fromstring(urdf)
    j = [j for j in root.findall("joint") if j.get("name") == "a_tcp_joint"][0]
    xyz = [float(v) for v in j.find("origin").get("xyz").split()]
    assert xyz == pytest.approx([0, 0, expect])


def test_ros2_control_block(urdf):
    root = ET.fromstring(urdf)
    rc = root.find("ros2_control")
    assert rc.find("hardware/plugin").text == "mujoco_ros2_control/MujocoSystemInterface"
    names = [j.get("name") for j in rc.findall("joint")]
    assert len(names) == 14
    effort = [j.get("name") for j in rc.findall("joint") if j.find("command_interface").get("name") == "effort"]
    assert len(effort) == 12
    model = [p for p in rc.findall("hardware/param") if p.get("name") == "mujoco_model"][0].text
    assert model.endswith("mjcf/scene.xml")


def test_meshes_resolve_into_the_workspace(urdf):
    """Robot descriptions are pinned in src/external (overlay): no mesh may come from /opt/ros (Isaac builds)."""
    from ament_index_python.packages import get_package_prefix
    files = {m.get("filename") for m in ET.fromstring(urdf).iter("mesh")}
    assert files
    for f in files:
        assert f.startswith("package://"), f
        pkg, rel = f[len("package://"):].split("/", 1)
        prefix = get_package_prefix(pkg)
        assert not prefix.startswith("/opt/ros"), f"{pkg} resolves to {prefix}"
        assert os.path.isfile(os.path.join(prefix, "share", pkg, rel)), f
