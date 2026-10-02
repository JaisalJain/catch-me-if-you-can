"""One command: MuJoCo sim + ros2_control + controllers + move_group + RViz (+ helper nodes).

    ros2 launch catch_bringup bringup.launch.py [headless:=false] [rviz:=true]
GUI processes are forced onto the NVIDIA GPU (PRIME offload) via the environment set in catch_sim.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = os.path.join(get_package_share_directory("catch_sim"), "launch", "sim.launch.py")
    mg = os.path.join(get_package_share_directory("catch_moveit_config"), "launch", "move_group.launch.py")
    tracker = os.path.join(get_package_share_directory("catch_tracker"), "launch", "tracker.launch.py")
    rviz_cfg = os.path.join(get_package_share_directory("catch_bringup"), "rviz", "catch.rviz")
    import importlib.util
    spec = importlib.util.spec_from_file_location("mg", mg)
    mgmod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mgmod)
    mc = mgmod.moveit_config()
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("mujoco_model", default_value=""),
        DeclareLaunchArgument("camera_rate", default_value="150.0"),   # measured 128.8 Hz (the plugin restarts its timer at each check; 130-140 gave 116 Hz)
        DeclareLaunchArgument("tracker", default_value="true"),
        DeclareLaunchArgument("plan_catcher", default_value="false"),   # benchmark baseline: B from A's plan
        SetEnvironmentVariable("__NV_PRIME_RENDER_OFFLOAD", "1"),
        SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", "nvidia"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(sim),
                                 launch_arguments={"headless": LaunchConfiguration("headless"),
                                                   "mujoco_model": LaunchConfiguration("mujoco_model"),
                                                   "camera_rate": LaunchConfiguration("camera_rate")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(tracker), condition=IfCondition(LaunchConfiguration("tracker"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(mg)),
        Node(package="catch_perception", executable="pallet_detector", output="log", parameters=[{"use_sim_time": True}]),
        Node(package="catch_perception", executable="color_check", output="both", parameters=[{"use_sim_time": True}]),
        Node(package="catch_throw", executable="throw_node", output="both", parameters=[{"use_sim_time": True}]),
        Node(package="catch_catcher", executable="catcher", output="both", parameters=[{"use_sim_time": True}]),
        # v2: reactive catch from the ready pose; vision-only (never subscribes to A's plan) ...
        Node(package="catch_catcher", executable="reactive_catcher", name="reactive_catcher", output="both",
             parameters=[{"use_sim_time": True, "source": "vision"}]),
        # ... and the plan-informed baseline (benchmark only)
        Node(package="catch_catcher", executable="reactive_catcher", name="reactive_catcher_plan", output="both",
             parameters=[{"use_sim_time": True, "source": "plan"}], condition=IfCondition(LaunchConfiguration("plan_catcher"))),
        Node(package="rviz2", executable="rviz2", arguments=["-d", rviz_cfg], output="log",
             condition=IfCondition(LaunchConfiguration("rviz")),
             parameters=[mc.robot_description, mc.robot_description_semantic, mc.robot_description_kinematics,
                         mc.planning_pipelines, mc.joint_limits, {"use_sim_time": True}]),
    ])
