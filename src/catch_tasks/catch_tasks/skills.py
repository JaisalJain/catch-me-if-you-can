"""Pick / place skills on top of MoveIt (catch_tasks.moveit_client). No ground truth is used here."""
from __future__ import annotations

import os
import time
from dataclasses import dataclass

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive

from catch_kinematics.transforms import rot_z
from catch_sim.scene_config import load_scene, slot_position

from .moveit_client import MoveItClient, _wait, pose_from

R_DOWN = np.column_stack([[0, 1, 0], [1, 0, 0], [0, 0, -1]])   # tool z down, fingers (tool x) along world y
R_UP = np.column_stack([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])     # tool z up (palm up), fingers along world y
GRIPPER_LINKS = ["robotiq_85_base_link", "robotiq_85_left_knuckle_link", "robotiq_85_right_knuckle_link",
                 "robotiq_85_left_finger_link", "robotiq_85_right_finger_link", "robotiq_85_left_inner_knuckle_link",
                 "robotiq_85_right_inner_knuckle_link", "robotiq_85_left_finger_tip_link",
                 "robotiq_85_right_finger_tip_link", "palm_insert", "tcp"]


def load_skill_cfg():
    with open(os.path.join(get_package_share_directory("catch_tasks"), "config", "skills.yaml")) as f:
        return yaml.safe_load(f)


@dataclass
class SkillResult:
    ok: bool
    message: str


class Skills:
    def __init__(self, node, mi: MoveItClient | None = None):
        self.node = node
        self.mi = mi or MoveItClient(node)
        self.cfg = load_skill_cfg()
        self.scene = load_scene()
        self.edge = self.scene["cube"]["edge"]
        self.ps = node.create_client(ApplyPlanningScene, "/apply_planning_scene")

    # ------------------------------------------------------------ planning scene
    def attach_cube(self, robot, cube_id, offset_z=0.0):
        """Attach a 40 mm box at the TCP (+offset along tool z) so MoveIt plans with the held cube."""
        aco = AttachedCollisionObject(link_name=f"{robot}_tcp", touch_links=[f"{robot}_{l}" for l in GRIPPER_LINKS])
        co = CollisionObject(id=f"held_{cube_id}", operation=CollisionObject.ADD)
        co.header.frame_id = f"{robot}_tcp"
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[self.edge + 0.004] * 3)]
        p = Pose()
        p.position.z, p.orientation.w = float(offset_z), 1.0
        co.primitive_poses = [p]
        aco.object = co
        return self._apply(attached=[aco])

    def detach_cube(self, robot, cube_id):
        aco = AttachedCollisionObject(link_name=f"{robot}_tcp")
        aco.object = CollisionObject(id=f"held_{cube_id}", operation=CollisionObject.REMOVE)
        co = CollisionObject(id=f"held_{cube_id}", operation=CollisionObject.REMOVE)
        co.header.frame_id = "world"
        return self._apply(attached=[aco], world=[co])

    def _apply(self, attached=(), world=()):
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects = list(attached)
        ps.world.collision_objects = list(world)
        if not self.ps.wait_for_service(timeout_sec=5.0):
            return False
        r = _wait(self.ps.call_async(ApplyPlanningScene.Request(scene=ps)), 5.0)
        return bool(r and r.success)

    # ------------------------------------------------------------ helpers
    def _grip(self, robot, value):
        ok, res = self.mi.gripper(robot, value, self.cfg["gripper_effort"])
        time.sleep(self.cfg["settle_time"])
        return ok

    def in_hand(self, robot):
        lo, hi = self.cfg["in_hand_knuckle"]
        q = self.mi.gripper_position(robot)
        return lo <= q <= hi, q

    def _down_rotation(self, robot, cube_yaw):
        """Top-down orientation whose finger axis is aligned with a cube face; of the 4 equivalent yaws pick the
        one needing the least wrist rotation from the current TCP orientation."""
        best, best_ang = None, np.inf
        cur = self._current_R(robot)
        for k in range(4):
            R = rot_z(cube_yaw + k * np.pi / 2) @ R_DOWN
            ang = np.arccos(np.clip((np.trace(cur.T @ R) - 1) / 2, -1, 1))
            if ang < best_ang:
                best, best_ang = R, ang
        return best

    def _current_R(self, robot):
        return self._robot_model(robot).fk(self.mi.joints(robot))[:3, :3]

    def _robot_model(self, robot):
        from catch_kinematics import UR5e
        from catch_kinematics.transforms import make_tf
        from catch_sim.scene_config import robot_base_xyz
        s = self.scene
        tcp = s["gripper"]["mount_offset"] + s["gripper"]["palm_insert"]["top_z"] + s["cube"]["edge"] / 2
        return UR5e(make_tf(rot_z(s["robots"][robot]["base_yaw"]), robot_base_xyz(s, robot)), make_tf(p=[0, 0, tcp]))

    def move_to_pose(self, robot, R, p, max_tries=3):
        """Collision-checked MoveIt joint goal to a TCP pose; the joint target comes from the closed-form IK
        (all 8 branches, closest to the current configuration first). More reliable than KDL pose goals here."""
        from catch_kinematics import POS_LIMIT, wrap
        from catch_kinematics.transforms import make_tf
        q0 = self.mi.joints(robot)
        sols = [q0 + wrap(s - q0) for s in self._robot_model(robot).ik_all(make_tf(R, p))]
        sols = [s for s in sols if np.all(np.abs(s) < POS_LIMIT - 0.05)]
        sols.sort(key=lambda s: float(np.sum(np.abs(s - q0) * [1.0, 1.0, 1.0, 0.5, 0.5, 0.3])))
        msg = "no IK solution"
        for s in sols[:max_tries]:
            ok, msg = self.mi.move_joints(robot, s)
            if ok:
                return True, msg
        return False, msg

    def _vertical(self, robot, R, p):
        """Short straight vertical TCP move inside the slot column. MoveIt collision checking is OFF here: the
        URDF fingertip meshes do not match the MJCF pads (which are what physically exist); finger/border clearance
        is guaranteed by the place-height formula and verified in MuJoCo (tools/pick_geometry.py)."""
        return self.mi.move_cartesian(robot, [pose_from(R, p)], vel_scale=self.cfg["cartesian_vel_scale"],
                                      avoid_collisions=False)

    def _vs(self):
        self.mi.vel_scale = self.mi.acc_scale = self.cfg["free_vel_scale"]

    # ------------------------------------------------------------ skills
    def pick(self, robot, cube_pos, cube_yaw=0.0, cube_id="cube") -> SkillResult:
        """Top-down grasp of a cube whose centre is at cube_pos (world)."""
        c = self.cfg
        p = np.asarray(cube_pos, float)
        R = self._down_rotation(robot, cube_yaw)
        grasp = p + [0, 0, c["grasp_offset"]]
        above = grasp + [0, 0, c["approach_height"]]
        self._vs()
        if not self._grip(robot, c["gripper_open"]):
            return SkillResult(False, "gripper did not open")
        ok, msg = self.move_to_pose(robot, R, above)
        if not ok:
            return SkillResult(False, f"approach failed: {msg}")
        ok, msg = self._vertical(robot, R, grasp)
        if not ok:
            return SkillResult(False, f"descent failed: {msg}")
        self._grip(robot, c["gripper_close"])
        held, q = self.in_hand(robot)
        if not held:
            self._vertical(robot, R, above)
            return SkillResult(False, f"grasp missed (knuckle {q:.3f})")
        ok, msg = self._vertical(robot, R, above)
        if not ok:
            return SkillResult(False, f"lift failed: {msg}")
        self.attach_cube(robot, cube_id, offset_z=c["grasp_offset"])
        held, q = self.in_hand(robot)
        return SkillResult(held, f"picked (knuckle {q:.3f})" if held else f"lost after lift (knuckle {q:.3f})")

    def place(self, robot, slot_color, pallet, cube_id="cube", held_offset=None, slot_yaw=0.0) -> SkillResult:
        """Put the held cube into `slot_color` slot of `pallet` ('a' | 'b'), top-down."""
        c = self.cfg
        held_offset = c["grasp_offset"] if held_offset is None else held_offset
        # fingertips stick out (pad_reach - edge/2 - held_offset) beyond the cube bottom; they must clear the borders
        tip_below = c["pad_reach"] - self.edge / 2 - held_offset
        bh = self.scene["pallets"]["border_height"]
        clearance = max(c["place_clearance"], tip_below + bh + c["border_margin"])
        target = slot_position(self.scene, pallet, slot_color) + [0, 0, clearance]   # cube centre at release
        R = self._down_rotation(robot, slot_yaw)
        tcp_at = target + [0, 0, held_offset]          # cube centre sits held_offset below the TCP (tool z down)
        above = tcp_at + [0, 0, c["approach_height"]]
        self._vs()
        ok, msg = self.move_to_pose(robot, R, above)
        if not ok:
            self.detach_cube(robot, cube_id)
            return SkillResult(False, f"pre-place failed: {msg}")
        self.detach_cube(robot, cube_id)
        ok, msg = self._vertical(robot, R, tcp_at)
        if not ok:
            return SkillResult(False, f"descent failed: {msg}")
        self._grip(robot, c["gripper_open"])
        ok, msg = self._vertical(robot, R, above)
        return SkillResult(ok, "placed" if ok else f"retreat failed: {msg}")
