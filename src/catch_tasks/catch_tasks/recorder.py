"""Record the sim state from ROS topics (joint states, cube ground truth, B's palm force) for evaluation and for
offline replay rendering (tools/render_replay.py). Ground truth is used for evaluation/rendering only."""
from __future__ import annotations

import threading

import numpy as np
from geometry_msgs.msg import WrenchStamped
from sensor_msgs.msg import JointState

from mujoco_ros2_control_msgs.msg import FreeJointStateArray

CUBES = ("cube_red", "cube_green", "cube_blue")


def stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


class Recorder:
    def __init__(self, node):
        self.lock = threading.Lock()
        self.active = False
        self.js_names = None
        self.clear()
        self.last_cubes = {}          # latest ground truth, always updated: name -> (t, pos[3], quat_wxyz[4], vel[3])
        node.create_subscription(JointState, "/joint_states", self._on_js, 100)
        node.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", self._on_fj, 100)
        node.create_subscription(WrenchStamped, "/b_palm_ft_broadcaster/wrench", self._on_ft, 100)

    def clear(self):
        with self.lock:
            self.js, self.cubes, self.ft = [], [], []

    def start(self):
        self.clear()
        self.active = True

    def stop(self):
        self.active = False

    def _on_js(self, m):
        if not self.active:
            return
        with self.lock:
            if self.js_names is None:
                self.js_names = list(m.name)
            if list(m.name) == self.js_names:
                self.js.append((stamp(m.header), list(m.position)))

    def _on_fj(self, m):
        t = stamp(m.header)
        row = {}
        for f in m.free_joints:
            p, q, v = f.pose.pose.position, f.pose.pose.orientation, f.twist.twist.linear
            row[f.name] = (t, [p.x, p.y, p.z], [q.w, q.x, q.y, q.z], [v.x, v.y, v.z])
        self.last_cubes.update(row)
        if not self.active:
            return
        if all(c in row for c in CUBES):
            with self.lock:
                self.cubes.append([t] + sum((row[c][1] + row[c][2] for c in CUBES), []))

    def _on_ft(self, m):
        if not self.active:
            return
        f = m.wrench.force
        with self.lock:
            self.ft.append((stamp(m.header), f.x, f.y, f.z))

    def arrays(self):
        """t_js[N], q[N, J], names; cubes[M, 1 + 3 * 7] (t, then pos+quat_wxyz per cube); ft[K, 4]."""
        with self.lock:
            js, cubes, ft = list(self.js), list(self.cubes), list(self.ft)
        return dict(t_js=np.array([j[0] for j in js]), q=np.array([j[1] for j in js]),
                    names=np.array(self.js_names or []), cubes=np.array(cubes), ft=np.array(ft))

    def save(self, path, **extra):
        a = self.arrays()
        a.update(extra)
        np.savez_compressed(path, **a)

    def cube(self, color):
        """Latest ground-truth (pos, R, vel) of cube_<color>, or None."""
        c = self.last_cubes.get(f"cube_{color}")
        if c is None:
            return None
        w, x, y, z = c[2]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return np.array(c[1]), R, np.array(c[3])
