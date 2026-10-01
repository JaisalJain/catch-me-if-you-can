"""MuJoCo sim + ros2_control + controllers + sim helper nodes.

    ros2 launch catch_sim sim.launch.py [headless:=false]
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro

NV_ENV = {  # same as tools/nv_env.sh: never let rendering silently fall back to the Intel iGPU
    "__NV_PRIME_RENDER_OFFLOAD": "1",
    "__GLX_VENDOR_LIBRARY_NAME": "nvidia",
    "__EGL_VENDOR_LIBRARY_FILENAMES": "/usr/share/glvnd/egl_vendor.d/10_nvidia.json",
    "MUJOCO_GL": "egl",
}


def robot_description(headless):
    desc = get_package_share_directory("catch_description")
    doc = xacro.process_file(os.path.join(desc, "urdf", "scene.urdf.xacro"),
                             mappings={"headless": headless,
                                       "mujoco_model": os.path.join(desc, "mjcf", "scene.xml")})
    return doc.toxml()


def setup(context):
    headless = LaunchConfiguration("headless").perform(context)
    sim = get_package_share_directory("catch_sim")
    controllers = os.path.join(sim, "config", "controllers.yaml")
    plugins = os.path.join(sim, "config", "mujoco_plugins.yaml")
    urdf = robot_description(headless)
    nodes = [SetEnvironmentVariable(k, v) for k, v in NV_ENV.items()]
    nodes += [
        Node(package="robot_state_publisher", executable="robot_state_publisher", output="log",
             parameters=[{"robot_description": ParameterValue(urdf, value_type=str), "use_sim_time": True}]),
        Node(package="mujoco_ros2_control", executable="ros2_control_node", output="both", emulate_tty=True,
             parameters=[{"use_sim_time": True}, controllers, plugins], on_exit=Shutdown()),
    ]
    for c in ("joint_state_broadcaster", "arm_a_controller", "arm_b_controller", "gripper_a_controller",
              "gripper_b_controller", "b_palm_ft_broadcaster"):
        nodes.append(Node(package="controller_manager", executable="spawner", output="log",
                          arguments=[c, "--param-file", controllers, "--controller-manager-timeout", "60"]))
    for exe in ("scene_manager", "ground_truth_tf", "rtf_monitor", "gpu_check", "camera_tf"):
        nodes.append(Node(package="catch_sim", executable=exe, output="both", parameters=[{"use_sim_time": exe != "rtf_monitor"}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        OpaqueFunction(function=setup),
    ])
