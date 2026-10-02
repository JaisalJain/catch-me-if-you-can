"""move_group for both arms (robot description from catch_description, sim time)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def moveit_config():
    desc = get_package_share_directory("catch_description")
    return (MoveItConfigsBuilder("catch_scene", package_name="catch_moveit_config")
            .robot_description(file_path=os.path.join(desc, "urdf", "scene.urdf.xacro"))
            .robot_description_semantic(file_path="config/catch_scene.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .joint_limits(file_path="config/joint_limits.yaml")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
            # /robot_description comes from robot_state_publisher only (it may point at an MJCF variant: mujoco_model:=)
            .planning_scene_monitor(publish_robot_description=False, publish_robot_description_semantic=True)
            .to_moveit_configs())


def generate_launch_description():
    mc = moveit_config()
    return LaunchDescription([
        Node(package="moveit_ros_move_group", executable="move_group", output="screen",
             parameters=[mc.to_dict(), {"use_sim_time": True}]),
    ])
