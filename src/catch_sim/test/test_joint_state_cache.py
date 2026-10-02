"""JointStateCache: the latest /joint_states is available without a subscription on the node itself. Own DDS
domain, so a running sim cannot interfere."""
import os
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from sensor_msgs.msg import JointState  # noqa: E402

from catch_sim.joint_state_cache import JointStateCache  # noqa: E402

DOMAIN = 100 + (os.getpid() + 37) % 100


def _wait(cond, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_cache_holds_latest_sample_off_the_node():
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=DOMAIN)
    try:
        node = rclpy.create_node("js_cache_test", context=ctx)
        cache = JointStateCache(node)
        assert "/joint_states" not in [s.topic_name for s in node.subscriptions]
        src = rclpy.create_node("js_src", context=ctx)
        pub = src.create_publisher(JointState, "/joint_states", 10)
        assert _wait(lambda: pub.get_subscription_count() >= 1)
        for k in range(200):                         # 200 samples, the cache must end on the last one
            m = JointState(name=["a_j", "b_j"], position=[0.001 * k, -0.001 * k], velocity=[1.0, 2.0])
            m.header.stamp.sec, m.header.stamp.nanosec = 5, 1000 * k
            pub.publish(m)
            time.sleep(0.0005)
        assert _wait(lambda: abs(cache.position("a_j") - 0.199) < 1e-12)
        assert cache.positions(["b_j", "missing"])[0] == pytest.approx(-0.199)
        assert cache.velocity("b_j") == 2.0
        assert cache.stamp() == pytest.approx(5 + 199e-6)
        assert cache.ready()
    finally:
        rclpy.shutdown(context=ctx)
