#!/usr/bin/env python3
"""Flag throw-library entries whose stroke start MoveIt considers in collision (sim + move_group running).

    python3 tools/v2_live/check_library_moveit.py        (system python, workspace sourced, bringup running)
The library builder checks clearance in MuJoCo; MoveIt plans A to the stroke start with the URDF collision meshes,
which are more conservative (entry 79 of the pinch library: a_upper_arm_link vs a_pedestal, so OMPL "cannot sample
any valid states for goal tree" -> the "setup" exclusion in every v2.0/v2.1 benchmark cell). Each entry's start is
checked with /check_state_validity (group arm_a, the rest of the robot at its current state) and the result stored
as "moveit_start_valid" in src/catch_throw/config/library_{pinch,cup}.json (not part of the entry hash);
catch_throw.library.Library.draw skips invalid entries. Rebuild catch_throw afterwards.
"""
import json
import os
import sys

import rclpy
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity
from sensor_msgs.msg import JointState

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
JN = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def main():
    rclpy.init()
    node = rclpy.create_node("check_library_moveit")
    cli = node.create_client(GetStateValidity, "/check_state_validity")
    if not cli.wait_for_service(timeout_sec=30.0):
        sys.exit("no /check_state_validity (is the bringup running?)")
    for name in ("pinch", "cup"):
        path = os.path.join(REPO, "src/catch_throw/config", f"library_{name}.json")
        d = json.load(open(path))
        bad = []
        for i, e in enumerate(d["entries"]):
            req = GetStateValidity.Request(group_name="arm_a")
            req.robot_state = RobotState(joint_state=JointState(name=[f"a_{j}" for j in JN],
                                                                position=[float(x) for x in e["q_start"]]),
                                         is_diff=True)
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
            r = fut.result()
            ok = bool(r and r.valid)
            e["moveit_start_valid"] = ok
            if not ok:
                bad.append((i, [f"{c.contact_body_1}-{c.contact_body_2}" for c in (r.contacts if r else [])][:2]))
        d["moveit_check"] = dict(tool="tools/v2_live/check_library_moveit.py", invalid=[b[0] for b in bad])
        json.dump(d, open(path, "w"), separators=(",", ":"))
        print(f"{name}: {len(d['entries'])} entries, MoveIt-invalid stroke starts: {bad}")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
