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
from tf2_ros import Buffer, TransformListener

from catch_interfaces.msg import CubeDetection, CubeDetectionArray
from catch_interfaces.srv import GetColor

from .vision import cube_blobs, dominant_color, pixel_ray_plane


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
        self.tf = Buffer()
        TransformListener(self.tf, self)
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
            d = CubeDetection(color=b.color, pixel_area=b.area, confidence=1.0)
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

    def on_req(self, req, res):
        cam = req.camera or self.cc["camera"]
        msg = self.latest.get(cam)
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


def _run(cls):
    rclpy.init()
    n = cls()
    ex = MultiThreadedExecutor()
    ex.add_node(n)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass


def detector_main():
    _run(PalletDetector)


def color_main():
    _run(ColorCheck)
