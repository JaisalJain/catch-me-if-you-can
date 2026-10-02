"""offload_sim_clock: the node's clock follows /clock from the helper thread, the node itself no longer subscribes
/clock, and sim-time timers on the node's own MultiThreadedExecutor still fire on sim time. Own DDS domain, so a
running sim cannot interfere."""
import os
import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402

from catch_sim.sim_clock import offload_sim_clock  # noqa: E402

DOMAIN = 100 + os.getpid() % 100


def _wait(cond, timeout=5.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_offloaded_clock_drives_node_time_and_timers():
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=DOMAIN)
    try:
        node = rclpy.create_node("offload_test", context=ctx, parameter_overrides=[Parameter("use_sim_time", value=True)])
        ex = MultiThreadedExecutor(context=ctx)
        ex.add_node(node)
        assert offload_sim_clock(node) is not None
        assert "/clock" not in [s.topic_name for s in node.subscriptions]
        assert node.get_parameter("use_sim_time").value
        ticks = []
        node.create_timer(0.1, lambda: ticks.append(node.get_clock().now().nanoseconds))   # 0.1 s of sim time
        threading.Thread(target=ex.spin, daemon=True).start()

        src = rclpy.create_node("clock_src", context=ctx)
        pub = src.create_publisher(Clock, "/clock", 10)
        assert _wait(lambda: pub.get_subscription_count() >= 1)

        def publish(t_ns):
            m = Clock()
            m.clock.sec, m.clock.nanosec = divmod(t_ns, 10 ** 9)
            pub.publish(m)

        publish(42 * 10 ** 9)
        assert _wait(lambda: node.get_clock().now().nanoseconds == 42 * 10 ** 9)
        for k in range(1, 1001):                    # 1 s of sim time in 1 ms steps, ~5x faster than real time
            publish(42 * 10 ** 9 + k * 10 ** 6)
            time.sleep(0.0002)
        assert _wait(lambda: node.get_clock().now().nanoseconds == 43 * 10 ** 9)
        assert _wait(lambda: len(ticks) >= 11)
        # the timer ran on sim time: one tick at the 0 -> 42 s jump, then every 0.1 s of sim time (1 ms clock steps)
        # while only ~0.25 s of wall time passed
        # (tick k is due at 42 s + k * 0.1 s; now() read in the callback may be a few 1 ms steps later)
        assert 11 <= len(ticks) <= 12, ticks
        assert all(0 <= t - (42 * 10 ** 9 + k * 10 ** 8) <= 3 * 10 ** 6 for k, t in enumerate(ticks)), ticks
        ex.shutdown()
    finally:
        rclpy.shutdown(context=ctx)


def test_no_offload_without_sim_time():
    ctx = rclpy.Context()
    rclpy.init(context=ctx, domain_id=DOMAIN)
    try:
        node = rclpy.create_node("wall_time", context=ctx)
        assert offload_sim_clock(node) is None
    finally:
        rclpy.shutdown(context=ctx)
