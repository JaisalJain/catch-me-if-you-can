"""M0 spike client: exercise both arms, both grippers, camera, free-joint GT, teleport and reset."""
import time, sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState, Image
from mujoco_ros2_control_msgs.msg import FreeJointStateArray, FreeJointState
from mujoco_ros2_control_msgs.srv import SetFreeJointState, ResetWorld

J = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


class Spike(Node):
    def __init__(self):
        super().__init__("spike_client", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        self.js = {}
        self.img = None
        self.fj = None
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)
        self.create_subscription(Image, "/mujoco_camera_plugin/overhead/color", self.on_img, 1)
        self.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", self.on_fj, 10)

    def on_js(self, m):
        self.js.update(dict(zip(m.name, m.position)))

    def on_img(self, m):
        self.img = m

    def on_fj(self, m):
        self.fj = m

    def spin_for(self, sec):
        t = time.time()
        while time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, fut, sec=20):
        t = time.time()
        while not fut.done() and time.time() - t < sec:
            rclpy.spin_once(self, timeout_sec=0.05)
        return fut.result() if fut.done() else None


def main():
    rclpy.init()
    n = Spike()
    n.spin_for(2.0)
    print("joint_states names:", sorted(n.js))
    # (a) both arms
    target = [0.3, -1.2, 1.2, -1.5, -1.57, 0.4]
    res = {}
    for p in "ab":
        ac = ActionClient(n, FollowJointTrajectory, f"/arm_{p}_controller/follow_joint_trajectory")
        ac.wait_for_server(timeout_sec=5)
        g = FollowJointTrajectory.Goal()
        g.trajectory.joint_names = [f"{p}_{j}" for j in J]
        g.trajectory.points = [JointTrajectoryPoint(positions=target, velocities=[0.0] * 6, time_from_start=Duration(sec=3))]
        h = n.wait(ac.send_goal_async(g))
        res[p] = h.get_result_async()
    for p in "ab":
        r = n.wait(res[p], 15)
        err = np.array([n.js[f"{p}_{j}"] for j in J]) - target
        print(f"arm {p}: result error_code={r.result.error_code}, final |err|max={np.abs(err).max():.4f} rad")
    # (b) grippers
    for p in "ab":
        ac = ActionClient(n, GripperCommand, f"/gripper_{p}_controller/gripper_cmd")
        ac.wait_for_server(timeout_sec=5)
        g = GripperCommand.Goal()
        g.command.position = 0.6
        g.command.max_effort = 10.0
        t0 = n.get_clock().now()
        h = n.wait(ac.send_goal_async(g))
        r = n.wait(h.get_result_async(), 10)
        dt = (n.get_clock().now() - t0).nanoseconds * 1e-9
        print(f"gripper {p}: cmd 0.6 -> state {n.js[f'{p}_robotiq_85_left_knuckle_joint']:.3f} "
              f"reached={r.result.reached_goal} stalled={r.result.stalled} sim_dt={dt:.2f}s")
    # (c) camera
    n.spin_for(1.0)
    if n.img is not None:
        a = np.frombuffer(n.img.data, np.uint8).reshape(n.img.height, n.img.width, -1)
        print(f"camera: {n.img.width}x{n.img.height} enc={n.img.encoding} frame={n.img.header.frame_id} mean={a.mean():.1f}")
        np.save("overhead.npy", a)
    else:
        print("camera: NO IMAGE")
    # (d) ground truth
    if n.fj:
        f = n.fj.free_joints[0]
        print(f"free joint GT: {f.name} pos=({f.pose.pose.position.x:.3f},{f.pose.pose.position.y:.3f},{f.pose.pose.position.z:.3f})"
              f" frame='{n.fj.header.frame_id}' stamp={n.fj.header.stamp.sec}.{n.fj.header.stamp.nanosec:09d}")
    # (e) teleport + reset
    cli = n.create_client(SetFreeJointState, "/mujoco_ros2_control_node/set_free_joint_state")
    cli.wait_for_service(timeout_sec=5)
    req = SetFreeJointState.Request()
    s = FreeJointState(name="cube_red")
    s.pose.pose.position.x, s.pose.pose.position.y, s.pose.pose.position.z = 0.1, 0.2, 1.2
    s.pose.pose.orientation.w = 1.0
    s.twist.twist.linear.x = 1.0
    req.free_joints = [s]
    r = n.wait(cli.call_async(req))
    print("set_free_joint_state:", r.success, r.message)
    n.spin_for(0.15)
    f = n.fj.free_joints[0]
    print(f"  after teleport: pos=({f.pose.pose.position.x:.3f},{f.pose.pose.position.y:.3f},{f.pose.pose.position.z:.3f}) "
          f"vx={f.twist.twist.linear.x:.3f}")
    rcli = n.create_client(ResetWorld, "/mujoco_ros2_control_node/reset_world")
    print("reset_world service available:", rcli.wait_for_service(timeout_sec=5))
    n.destroy_node()
    rclpy.shutdown()


main()
