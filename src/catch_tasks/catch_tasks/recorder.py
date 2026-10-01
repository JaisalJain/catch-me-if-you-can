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
    """Subscribes only while recording (rclpy is slow: /joint_states and the palm wrench arrive at 500 Hz, the
    ground truth at 200 Hz). Outside recordings, cube() takes one fresh ground-truth sample on demand."""

    def __init__(self, node):
        self.node = node
        self.lock = threading.Lock()
        self.active = False
        self.js_names = None
        self.subs = []
        self.clear()
        self.last_cubes = {}          # latest ground truth: name -> (t, pos[3], quat_wxyz[4], vel[3])

    def _subscribe(self, with_rates=True):
        n = self.node
        subs = [n.create_subscription(FreeJointStateArray, "/free_joint_state_publisher/free_joint_states", self._on_fj, 1000)]
        if with_rates:
            subs += [n.create_subscription(JointState, "/joint_states", self._on_js, 1000),
                     n.create_subscription(WrenchStamped, "/b_palm_ft_broadcaster/wrench", self._on_ft, 1000)]
        return subs

    def _unsubscribe(self, subs):
        for s in subs:
            self.node.destroy_subscription(s)

    def clear(self):
        with self.lock:
            self.js, self.cubes, self.ft = [], [], []

    def start(self, wait=3.0):
        """Subscribe and wait (wall time) until every stream delivers (new subscriptions need DDS discovery)."""
        import time
        self.clear()
        self.active = True
        self.subs = self._subscribe()
        t0 = time.monotonic()
        while time.monotonic() - t0 < wait:
            with self.lock:
                if self.js and self.cubes and self.ft:
                    break
            time.sleep(0.01)

    def stop(self):
        self.active = False
        self._unsubscribe(self.subs)
        self.subs = []

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

    def fresh(self, timeout=2.0):
        """Wait for a ground-truth sample newer than now (one-shot subscription when not recording)."""
        import time
        t_req = self.node.get_clock().now().nanoseconds * 1e-9
        subs = [] if self.active else self._subscribe(with_rates=False)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            c = self.last_cubes.get(CUBES[0])
            if c is not None and c[0] >= t_req:
                break
            time.sleep(0.005)
        self._unsubscribe(subs)

    def cube(self, color, fresh=True):
        """Ground-truth (pos, R, vel) of cube_<color> (a new sample unless fresh=False), or None."""
        if fresh:
            self.fresh()
        c = self.last_cubes.get(f"cube_{color}")
        if c is None:
            return None
        w, x, y, z = c[2]
        R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                      [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                      [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
        return np.array(c[1]), R, np.array(c[3])


def main():
    """Standalone recorder (own process, so it does not load the state machine's executor):
        ros2 run catch_tasks record_run --out log/m6/run_seed1.npz      (stop with SIGINT / SIGTERM)"""
    import argparse
    import signal
    import time

    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = rclpy.create_node("record_run", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    rec = Recorder(node)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    import threading
    threading.Thread(target=ex.spin, daemon=True).start()
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    rec.start()
    while not stop.is_set():
        time.sleep(0.1)
    rec.stop()
    import os
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    rec.save(args.out)
    a = rec.arrays()
    print(f"saved {args.out}: {len(a['t_js'])} joint-state samples, {len(a['cubes'])} ground-truth samples", flush=True)
    ex.shutdown()
