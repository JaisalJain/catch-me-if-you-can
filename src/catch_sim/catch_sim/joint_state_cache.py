"""Latest /joint_states without burning a core.

/joint_states arrives at ~470 Hz. A subscription on a node spun by a MultiThreadedExecutor costs ~50-95 % of a
core in rclpy (v1.0: throw_node 95 %, catcher / run_demo ~50 %, idle). JointStateCache keeps only the latest sample,
received by a small helper node ("<name>_js") spun by an EventsExecutor in a daemon thread — the same pattern as
catch_sim.sim_clock.offload_sim_clock for /clock. Readers get a copy under a lock; nothing runs on the node's own
executor.
"""
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.experimental import EventsExecutor
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateCache:
    def __init__(self, node, topic="/joint_states"):
        self._lock = threading.Lock()
        self._pos, self._vel, self._eff = {}, {}, {}
        self._stamp = None
        self.n = 0
        self.helper = rclpy.create_node(f"{node.get_name()}_js", namespace=node.get_namespace(), context=node.context,
                                        parameter_overrides=[Parameter("use_sim_time", value=False)],
                                        start_parameter_services=False)
        # depth 1: only the latest sample matters; best effort is not used (the broadcaster publishes reliable)
        self.helper.create_subscription(JointState, topic, self._on, QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE))
        self._ex = EventsExecutor(context=node.context)
        self._ex.add_node(self.helper)
        threading.Thread(target=self._spin, name=f"{node.get_name()}_js", daemon=True).start()

    def _spin(self):
        try:
            self._ex.spin()
        except ExternalShutdownException:
            pass

    def _on(self, m):
        with self._lock:
            self._pos.update(zip(m.name, m.position))
            if m.velocity:
                self._vel.update(zip(m.name, m.velocity))
            if m.effort:
                self._eff.update(zip(m.name, m.effort))
            self._stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            self.n += 1

    # ------------------------------------------------------------------ readers
    def ready(self):
        return self.n > 0

    def wait(self, timeout=10.0):
        t0 = time.monotonic()
        while not self.n and time.monotonic() - t0 < timeout:
            time.sleep(0.01)
        return self.n > 0

    def position(self, name, default=float("nan")):
        with self._lock:
            return self._pos.get(name, default)

    def velocity(self, name, default=0.0):
        with self._lock:
            return self._vel.get(name, default)

    def positions(self, names):
        with self._lock:
            return [self._pos.get(n, float("nan")) for n in names]

    def velocities(self, names):
        with self._lock:
            return [self._vel.get(n, 0.0) for n in names]

    def stamp(self):
        with self._lock:
            return self._stamp

    def snapshot(self):
        """(stamp, {name: (position, velocity)})"""
        with self._lock:
            return self._stamp, {k: (v, self._vel.get(k, 0.0)) for k, v in self._pos.items()}
