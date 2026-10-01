"""Small sim monitors: ground-truth cube TF, real-time factor, GPU check, camera optical-frame TFs."""
import os
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float64
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from mujoco_ros2_control_msgs.msg import FreeJointStateArray

from .scene_config import RX_PI, load_scene, mujoco_camera_rotations


def _quat_xyzw(R):
    w = np.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2
    y = np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2
    z = np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return [x, y, z, w]


class GroundTruthTF(Node):
    """Broadcast world -> cube_<color> from the FreeJointStatePublisherPlugin (ground truth, for RViz/tests)."""

    def __init__(self):
        super().__init__("ground_truth_tf")
        self.br = TransformBroadcaster(self)
        self.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", self.cb, 10)

    def cb(self, msg):
        tfs = []
        for f in msg.free_joints:
            t = TransformStamped()
            t.header.stamp = msg.header.stamp
            t.header.frame_id = "world"
            t.child_frame_id = f"gt_{f.name}"
            p, q = f.pose.pose.position, f.pose.pose.orientation
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation = q
            tfs.append(t)
        self.br.sendTransform(tfs)


class RtfMonitor(Node):
    """Logs the real-time factor (sim seconds per wall second) and warns below the threshold."""

    def __init__(self):
        super().__init__("rtf_monitor")
        self.declare_parameter("window", 2.0)
        self.declare_parameter("warn_below", 0.8)
        self.samples = []
        self.pub = self.create_publisher(Float64, "/sim/rtf", 1)
        self.create_subscription(Clock, "/clock", self.on_clock, 100)
        self.last_report = time.monotonic()

    def on_clock(self, m):
        now = time.monotonic()
        self.samples.append((now, m.clock.sec + m.clock.nanosec * 1e-9))
        win = self.get_parameter("window").value
        while self.samples and now - self.samples[0][0] > win:
            self.samples.pop(0)
        if now - self.last_report >= win and len(self.samples) > 10:
            self.last_report = now
            (w0, s0), (w1, s1) = self.samples[0], self.samples[-1]
            rtf = (s1 - s0) / max(w1 - w0, 1e-6)
            self.pub.publish(Float64(data=rtf))
            if rtf < self.get_parameter("warn_below").value:
                self.get_logger().warn(f"real-time factor {rtf:.2f} < {self.get_parameter('warn_below').value}")
            else:
                self.get_logger().info(f"RTF {rtf:.3f}", throttle_duration_sec=30.0)


class GpuCheck(Node):
    """Verifies via nvidia-smi that the MuJoCo process renders on the NVIDIA GPU (PRIME on-demand laptops
    silently fall back to Intel otherwise). Errors loudly; with fail_hard, shuts the launch down."""

    def __init__(self):
        super().__init__("gpu_check")
        self.declare_parameter("process_pattern", "mujoco_ros2_control/ros2_control_node")
        self.declare_parameter("delay", 8.0)
        self.declare_parameter("fail_hard", False)
        self.timer = self.create_timer(self.get_parameter("delay").value, self.check)
        self.ok = None

    def check(self):
        self.timer.cancel()
        pat = self.get_parameter("process_pattern").value
        try:
            pids = subprocess.check_output(["pgrep", "-f", pat]).decode().split()
            smi = subprocess.check_output(["nvidia-smi", "--query-compute-apps=pid", "--format=csv,noheader"]).decode()
            full = subprocess.check_output(["nvidia-smi"]).decode()
        except (subprocess.CalledProcessError, FileNotFoundError) as e:
            self.get_logger().error(f"GPU check could not run: {e}")
            self.ok = False
            return
        on_gpu = [p for p in pids if f" {p} " in full or p in smi.split()]
        self.ok = bool(on_gpu)
        if self.ok:
            self.get_logger().info(f"GPU check OK: {pat} (pid {on_gpu}) is listed by nvidia-smi")
        else:
            self.get_logger().error(f"GPU check FAILED: {pat} (pids {pids}) is NOT on the NVIDIA GPU. "
                                    "Rendering fell back to Intel/software. Source tools/nv_env.sh.")
            if self.get_parameter("fail_hard").value:
                raise SystemExit(2)


class CameraTF(Node):
    """Static TFs for the camera optical frames (ROS convention), computed like build_mjcf.py places them."""

    def __init__(self):
        super().__init__("camera_tf")
        scene = load_scene()
        self.br = StaticTransformBroadcaster(self)
        tfs = []
        for name, (parent, p, R) in mujoco_camera_rotations(scene).items():
            Ropt = R @ RX_PI
            if parent == "b_robotiq_mjcf_base":
                continue    # B's wrist camera: b_wrist_cam_optical is in the URDF (tested in test_fk_agreement)
            t = TransformStamped()
            t.header.frame_id = parent
            t.child_frame_id = f"{name}_optical"
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = [float(v) for v in p]
            x, y, z, w = _quat_xyzw(Ropt)
            t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = x, y, z, w
            tfs.append(t)
        self.br.sendTransform(tfs)


def _run(cls):
    rclpy.init()
    n = cls()
    try:
        rclpy.spin(n)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def ground_truth_main():
    _run(GroundTruthTF)


def rtf_main():
    _run(RtfMonitor)


def gpu_main():
    _run(GpuCheck)


def camera_tf_main():
    _run(CameraTF)
