"""M3 integration test: B places a cube spawned into its gripper (colour read from B's wrist camera), then A
picks every cube from its pallet using the overhead detection and puts it back. Ground truth is used ONLY to
score the result.

    ros2 run catch_tasks pickplace_test [--part b|a|both]
"""
import argparse
import json
import sys
import threading
import time

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor

from catch_interfaces.msg import CubeDetectionArray
from catch_interfaces.srv import GetColor, ResetScene, SpawnCube
from catch_sim.scene_config import slot_position
from mujoco_ros2_control_msgs.msg import FreeJointStateArray

from .moveit_client import _wait
from .skills import Skills

COLORS = ("red", "green", "blue")


def yaw_tilt(q):
    w, x, y, z = q.w, q.x, q.y, q.z
    zz = 1 - 2 * (x * x + y * y)                 # cube z-axis . world z
    return np.degrees(np.arccos(np.clip(abs(zz), -1, 1)))


class Harness:
    def __init__(self):
        self.node = rclpy.create_node("pickplace_test",
                                      parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
        ex = MultiThreadedExecutor()
        ex.add_node(self.node)
        threading.Thread(target=ex.spin, daemon=True).start()
        self.sk = Skills(self.node)
        assert self.sk.mi.wait_ready(60), "move_group not ready"
        self.st = {}
        self.node.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states",
                                      lambda m: self.st.__setitem__("gt", m), 1)
        self.node.create_subscription(CubeDetectionArray, "/perception/overhead_a/cubes",
                                      lambda m: self.st.__setitem__("det_a", m), 1)
        self.spawn = self.node.create_client(SpawnCube, "/scene/spawn_cube")
        self.reset = self.node.create_client(ResetScene, "/scene/reset")
        self.color = self.node.create_client(GetColor, "/perception/get_color")
        self.poses = yaml.safe_load(open(get_package_share_directory("catch_description") + "/config/poses.yaml"))

    def now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def sleep_sim(self, dt):
        t = self.now() + dt
        while self.now() < t:
            time.sleep(0.01)

    def gt(self, color):
        for f in self.st["gt"].free_joints:
            if f.name == f"cube_{color}":
                p = f.pose.pose.position
                return np.array([p.x, p.y, p.z]), f.pose.pose.orientation
        raise KeyError(color)

    def fresh_detections(self, after):
        t0 = time.monotonic()
        while time.monotonic() - t0 < 10:
            d = self.st.get("det_a")
            if d is not None and d.header.stamp.sec + d.header.stamp.nanosec * 1e-9 >= after:
                return {x.color: x for x in d.detections}
            time.sleep(0.02)
        return {}

    def score_place(self, color, pallet):
        self.sleep_sim(0.8)
        p, q = self.gt(color)
        s = slot_position(self.sk.scene, pallet, color)
        err = 1000 * np.hypot(*(p - s)[:2])
        inner = self.sk.scene["pallets"]["slot_inner"] / 2 - self.sk.scene["cube"]["edge"] / 2
        in_slot = abs(p[0] - s[0]) < inner + 1e-3 and abs(p[1] - s[1]) < inner + 1e-3 and abs(p[2] - s[2]) < 0.004
        return dict(xy_err_mm=round(err, 2), dz_mm=round(1000 * (p[2] - s[2]), 2), tilt_deg=round(yaw_tilt(q), 2),
                    in_slot=bool(in_slot))

    def part_b(self):
        out = []
        sk, k = self.sk, "b"
        for truth in COLORS:
            _wait(self.reset.call_async(ResetScene.Request()), 10)
            ok, msg = sk.mi.move_joints(k, self.poses[k]["palm_up"])
            sk._grip(k, 0.0)
            r = _wait(self.spawn.call_async(SpawnCube.Request(color=truth, where="gripper_b")), 10)
            sk._grip(k, sk.cfg["gripper_close"])
            held, knuckle = sk.in_hand(k)
            self.sleep_sim(0.3)
            c = _wait(self.color.call_async(GetColor.Request(camera="b_wrist")), 10)
            seen = c.color if c and c.success else ""
            res = dict(truth=truth, seen=seen, color_ok=seen == truth,
                       fractions=[round(v, 3) for v in (c.fractions if c else [])], held=held)
            if held and seen:
                sk.attach_cube(k, truth, offset_z=0.0038)
                pr = sk.place(k, seen, "b", cube_id=truth, held_offset=0.0038)
                res.update(place_ok=pr.ok, place_msg=pr.message, **self.score_place(truth, "b"))
            out.append(res)
            print("B", json.dumps(res), flush=True)
            sk.mi.move_joints(k, self.poses[k]["home"])
        _wait(self.reset.call_async(ResetScene.Request()), 10)
        return out

    def part_a(self):
        out = []
        sk, k = self.sk, "a"
        _wait(self.reset.call_async(ResetScene.Request()), 10)
        sk.mi.move_joints(k, self.poses[k]["home"])
        for truth in COLORS:
            sk.mi.move_joints(k, self.poses[k]["home"])      # keep the arm out of the overhead view
            dets = self.fresh_detections(self.now() + 0.5)
            if truth not in dets:
                out.append(dict(truth=truth, detected=False))
                continue
            if dets[truth].confidence < 0.7:
                out.append(dict(truth=truth, detected=True, low_confidence=dets[truth].confidence))
                continue
            d = dets[truth].pose
            pos = np.array([d.position.x, d.position.y, d.position.z])
            yaw = np.arctan2(2 * d.orientation.w * d.orientation.z, 1 - 2 * d.orientation.z ** 2)
            gt_p, _ = self.gt(truth)
            pr = sk.pick(k, pos, yaw, cube_id=truth)
            res = dict(truth=truth, det_err_mm=round(1000 * float(np.hypot(*(pos - gt_p)[:2])), 2),
                       det_conf=round(dets[truth].confidence, 3), pick_ok=pr.ok,
                       pick_msg=pr.message)
            if pr.ok:
                self.sleep_sim(0.3)
                cube_p, _ = self.gt(truth)
                tcp = sk._current_R(k)  # noqa: F841 (orientation only; position check below uses FK)
                from catch_kinematics import UR5e  # local import keeps module import light
                from catch_kinematics.transforms import make_tf, rot_z
                from catch_sim.scene_config import robot_base_xyz
                s = sk.scene
                off = s["gripper"]["mount_offset"] + s["gripper"]["palm_insert"]["top_z"] + s["cube"]["edge"] / 2
                ur = UR5e(make_tf(rot_z(s["robots"][k]["base_yaw"]), robot_base_xyz(s, k)), make_tf(p=[0, 0, off]))
                T = ur.fk(sk.mi.joints(k))
                rel = T[:3, :3].T @ (cube_p - T[:3, 3])
                res["cube_in_tcp_mm"] = (1000 * rel).round(1).tolist()
                pl = sk.place(k, truth, "a", cube_id=truth)
                res.update(place_ok=pl.ok, place_msg=pl.message, **self.score_place(truth, "a"))
            out.append(res)
            print("A", json.dumps(res), flush=True)
        sk.mi.move_joints(k, self.poses[k]["home"])
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--part", default="both")
    args, _ = ap.parse_known_args()
    rclpy.init()
    h = Harness()
    results = {}
    if args.part in ("b", "both"):
        results["b_place"] = h.part_b()
    if args.part in ("a", "both"):
        results["a_pick_place"] = h.part_a()
    ok = all(r.get("color_ok", True) and r.get("in_slot", False) for r in results.get("b_place", [])) and \
        all(r.get("pick_ok", False) and r.get("in_slot", False) for r in results.get("a_pick_place", []))
    print("RESULT", json.dumps(dict(passed=ok, **results)))
    rclpy.shutdown()
    sys.exit(0 if ok else 1)
