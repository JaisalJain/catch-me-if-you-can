"""ROS nodes: overhead pallet detector and B's color check."""
import os

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import DurabilityPolicy, QoSProfile
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

from catch_interfaces.msg import CubeDetection, CubeDetectionArray
from catch_interfaces.srv import GetColor, GetInHandPose
from catch_sim.sim_clock import offload_sim_clock

from .vision import cube_blobs, dominant_color, in_hand_pose, pixel_ray_plane


def static_tf_buffer(node, callback_group=None):
    """TF buffer fed from /tf_static only. All transforms these nodes need (camera mounts, b_tcp -> wrist camera)
    are static; a full TransformListener would also process the 500 Hz /tf stream and starve the executor."""
    buf = Buffer()

    def on_static(msg):
        for t in msg.transforms:
            buf.set_transform_static(t, "tf_static")
    node.create_subscription(TFMessage, "/tf_static", on_static,
                             QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL),
                             callback_group=callback_group)
    return buf


def load_cfg():
    with open(os.path.join(get_package_share_directory("catch_perception"), "config", "perception.yaml")) as f:
        return yaml.safe_load(f)


def load_scene():
    with open(os.path.join(get_package_share_directory("catch_description"), "config", "scene.yaml")) as f:
        return yaml.safe_load(f)


def to_rgb(msg: Image) -> np.ndarray:
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        a = a[..., ::-1]
    return np.ascontiguousarray(a[..., :3])


def _quat_to_R(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class PalletDetector(Node):
    """Detect cubes on the pallets from the overhead cameras: HSV segmentation -> cube-top core -> ray/plane
    intersection with the plane of the cube tops. Publishes /perception/<camera>/cubes (world frame)."""

    def __init__(self):
        super().__init__("pallet_detector")
        cfg, scene = load_cfg(), load_scene()
        self.colors = cfg["colors"]
        self.pd = cfg["pallet_detector"]
        p = scene["pallets"]
        self.z_top = scene["table"]["top_z"] + p["base_size"][2] + scene["cube"]["edge"]       # cube on a pallet
        self.z_top_table = scene["table"]["top_z"] + scene["cube"]["edge"]                     # cube on the table
        self.half = scene["cube"]["edge"] / 2
        bx, by = p["base_size"][0] / 2, p["base_size"][1] / 2
        self.pallet_boxes = [(p[k]["center"][0] - bx, p[k]["center"][0] + bx, p[k]["center"][1] - by, p[k]["center"][1] + by)
                             for k in ("a", "b")]
        self.tf = static_tf_buffer(self)
        self.info, self.pubs = {}, {}
        for cam in self.pd["cameras"]:
            self.pubs[cam] = self.create_publisher(CubeDetectionArray, f"/perception/{cam}/cubes", 10)
            self.create_subscription(CameraInfo, f"/cameras/{cam}/camera_info",
                                     lambda m, c=cam: self.info.__setitem__(c, m), 1)
            self.create_subscription(Image, f"/cameras/{cam}/image_raw", lambda m, c=cam: self.on_image(c, m), 1)

    def on_image(self, cam, msg):
        if cam not in self.info:
            return
        try:
            tf = self.tf.lookup_transform("world", msg.header.frame_id, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            self.get_logger().warn(f"no TF for {msg.header.frame_id}: {e}", throttle_duration_sec=5.0)
            return
        R = _quat_to_R(tf.transform.rotation)
        t = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
        K = np.array(self.info[cam].k).reshape(3, 3)
        out = CubeDetectionArray()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = "world"
        for b in cube_blobs(to_rgb(msg), self.colors, self.pd["core_px"], self.pd["min_core_area"]):
            p = pixel_ray_plane(b.u, b.v, K, R, t, self.z_top)
            if not any(x0 <= p[0] <= x1 and y0 <= p[1] <= y1 for x0, x1, y0, y1 in self.pallet_boxes):
                p = pixel_ray_plane(b.u, b.v, K, R, t, self.z_top_table)   # not on a pallet: on the table
            # confidence = visible core area / core area of an unoccluded cube top at this depth
            depth = float((R.T @ (p - t))[2])
            side = K[0, 0] * 2 * self.half / max(depth, 1e-6)
            expected = max(side - 2 * self.pd["core_px"], 1.0) ** 2
            d = CubeDetection(color=b.color, pixel_area=b.area, confidence=float(min(1.0, b.area / expected)))
            d.pose.position.x, d.pose.position.y, d.pose.position.z = float(p[0]), float(p[1]), float(p[2] - self.half)
            # image yaw -> world yaw: the overhead optical frame has x = world x, y = -world y
            yaw_w = -b.yaw
            d.pose.orientation.z, d.pose.orientation.w = float(np.sin(yaw_w / 2)), float(np.cos(yaw_w / 2))
            out.detections.append(d)
        self.pubs[cam].publish(out)


class ColorCheck(Node):
    """Service /perception/get_color: dominant cube color in the ROI of a camera (default B's wrist camera).
    Uses only the image — never the color reported by A."""

    def __init__(self):
        super().__init__("color_check")
        cfg = load_cfg()
        self.colors = cfg["colors"]
        self.cc = cfg["color_check"]
        self.latest = {}
        cb = ReentrantCallbackGroup()
        for cam in ("b_wrist",):
            self.create_subscription(Image, f"/cameras/{cam}/image_raw", lambda m, c=cam: self.latest.__setitem__(c, m),
                                     1, callback_group=cb)
        self.create_service(GetColor, "/perception/get_color", self.on_req, callback_group=cb)
        # in-hand pose of the caught cube (wrist camera looks along the closing axis)
        self.edge = load_scene()["cube"]["edge"]
        self.ih = cfg["in_hand"]
        self.info = None
        self.create_subscription(CameraInfo, "/cameras/b_wrist/camera_info", lambda m: setattr(self, "info", m), 1,
                                 callback_group=cb)
        self.tf = static_tf_buffer(self, cb)
        self.create_service(GetInHandPose, "/perception/in_hand_pose", self.on_in_hand, callback_group=cb)

    def on_req(self, req, res):
        cam = req.camera or self.cc["camera"]
        msg = self.fresh_image(cam)
        if msg is None:
            res.success, res.message = False, f"no image from {cam}"
            return res
        best, frac, fr = dominant_color(to_rgb(msg), self.colors, self.cc["roi"])
        res.fractions = [fr["red"], fr["green"], fr["blue"]]
        res.confidence = frac
        if frac < self.cc["min_fraction"]:
            res.success, res.color, res.message = False, "", f"no cube color in ROI (best {best} {frac:.3f})"
        else:
            res.success, res.color, res.message = True, best, f"{best} {frac:.3f}"
        self.get_logger().info(f"get_color({cam}): {res.message} fractions={np.round(res.fractions, 3).tolist()}")
        return res


    def fresh_image(self, cam, timeout=1.5):
        """Latest image of `cam` rendered after this request (the camera publishes at 10 Hz)."""
        import time
        t_req = self.get_clock().now().nanoseconds * 1e-9
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            msg = self.latest.get(cam)
            if msg is not None and msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 >= t_req:
                return msg
            time.sleep(0.01)
        return None

    def on_in_hand(self, req, res):
        msg = self.fresh_image("b_wrist")
        if msg is None or self.info is None:
            res.success, res.message = False, "no wrist image / camera_info"
            return res
        try:
            tf = self.tf.lookup_transform("b_tcp", msg.header.frame_id, rclpy.time.Time())
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, f"no TF b_tcp <- {msg.header.frame_id}: {e}"
            return res
        T = np.eye(4)
        T[:3, :3] = _quat_to_R(tf.transform.rotation)
        T[:3, 3] = [tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z]
        rgb = to_rgb(msg)
        if self.ih.get("debug_dir"):
            import cv2
            os.makedirs(self.ih["debug_dir"], exist_ok=True)
            cv2.imwrite(os.path.join(self.ih["debug_dir"], "in_hand_last.png"), rgb[..., ::-1])
        color = req.color or dominant_color(rgb, self.colors, self.cc["roi"])[0]
        r = in_hand_pose(rgb, self.colors[color], np.array(self.info.k).reshape(3, 3), T, self.edge,
                         self.ih["min_px"])
        if r is None:
            res.success, res.message = False, f"no {color} cube face in the wrist image"
            return res
        res.angle, res.fit_rms = r["angle"], r["rms"]
        res.pose.position.x, res.pose.position.y, res.pose.position.z = [float(v) for v in r["p"]]
        res.pose.orientation.w, res.pose.orientation.x = float(np.cos(r["angle"] / 2)), float(np.sin(r["angle"] / 2))
        ok = r["rms"] < self.ih["max_rms"] and r["inlier_frac"] > self.ih["min_inlier_frac"]
        res.success = bool(ok)
        res.message = (f"{color}: angle {np.degrees(r['angle']):+.1f} deg, centre (y, z) "
                       f"({1000 * r['p'][1]:+.1f}, {1000 * r['p'][2]:+.1f}) mm, rms {1000 * r['rms']:.2f} mm, "
                       f"inliers {r['inlier_frac']:.2f}, {r['n_px']} px")
        self.get_logger().info(f"in_hand_pose: {res.message}")
        return res


def _run(cls):
    rclpy.init()
    n = cls()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    offload_sim_clock(n)          # /clock off the MultiThreadedExecutor (catch_sim.sim_clock)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


def detector_main():
    _run(PalletDetector)


def color_main():
    _run(ColorCheck)
