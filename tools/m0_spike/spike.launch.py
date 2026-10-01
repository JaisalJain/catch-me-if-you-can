import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))

def setup(context):
    headless = LaunchConfiguration("headless").perform(context)
    urdf = subprocess.check_output(["xacro", f"{HERE}/spike.urdf.xacro", f"mjcf:={HERE}/spike.xml", f"headless:={headless}"]).decode()
    params = f"{HERE}/controllers.yaml"
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": urdf, "use_sim_time": True}]),
        Node(package="mujoco_ros2_control", executable="ros2_control_node", output="both",
             parameters=[{"use_sim_time": True}, params], on_exit=Shutdown()),
    ]
    for c in ["joint_state_broadcaster", "arm_a_controller", "arm_b_controller", "gripper_a_controller", "gripper_b_controller"]:
        nodes.append(Node(package="controller_manager", executable="spawner", arguments=[c, "--param-file", params]))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("headless", default_value="true"), OpaqueFunction(function=setup)])
