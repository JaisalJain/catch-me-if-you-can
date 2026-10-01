"""Scene reset / cube spawning for the sim (wraps mujoco_ros2_control's ~/set_free_joint_state)."""
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from catch_interfaces.srv import ResetScene, SpawnCube
from mujoco_ros2_control_msgs.msg import FreeJointState
from mujoco_ros2_control_msgs.srv import SetFreeJointState

from .scene_config import load_scene, slot_position, tcp_in_gripper_base

COLORS = ("red", "green", "blue")


class SceneManager(Node):
    def __init__(self):
        super().__init__("scene_manager")
        self.scene = load_scene()
        cb = ReentrantCallbackGroup()
        self.cli = self.create_client(SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state",
                                      callback_group=cb)
        self.create_service(ResetScene, "/scene/reset", self.on_reset, callback_group=cb)
        self.create_service(SpawnCube, "/scene/spawn_cube", self.on_spawn, callback_group=cb)

    def _entry(self, color, xyz, frame=""):
        e = FreeJointState(name=f"cube_{color}")
        e.pose.header.frame_id = frame
        e.pose.pose.position.x, e.pose.pose.position.y, e.pose.pose.position.z = [float(v) for v in xyz]
        e.pose.pose.orientation.w = 1.0
        return e

    def _call(self, entries):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return False, "set_free_joint_state service not available"
        req = SetFreeJointState.Request(free_joints=entries)
        res = self.cli.call(req)
        return res.success, res.message

    def on_reset(self, req, res):
        entries = [self._entry(c, slot_position(self.scene, "a", c)) for c in COLORS]
        res.success, res.message = self._call(entries)
        self.get_logger().info(f"reset scene: {res.success} {res.message}")
        return res

    def on_spawn(self, req, res):
        if req.color not in COLORS:
            res.success, res.message = False, f"unknown color {req.color}"
            return res
        if req.where in ("gripper_a", "gripper_b"):
            # cube resting on the palm insert, axis-aligned with the gripper base (fingers must be open)
            e = self._entry(req.color, tcp_in_gripper_base(self.scene), frame=f"{req.where[-1]}_robotiq_base")
        elif req.where in ("pallet_a", "pallet_b", "home"):
            pallet = "a" if req.where == "home" else req.where[-1]
            e = self._entry(req.color, slot_position(self.scene, pallet, req.color))
        else:
            res.success, res.message = False, f"unknown location {req.where}"
            return res
        res.success, res.message = self._call([e])
        self.get_logger().info(f"spawn {req.color} at {req.where}: {res.success} {res.message}")
        return res


def main():
    rclpy.init()
    node = SceneManager()
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    ex.spin()
