"""Sim time for rclpy nodes without the /clock CPU burn.

With use_sim_time, rclpy's TimeSource subscribes /clock on the node itself, so the node's executor runs a Python
callback for every sim step (1 kHz). Under a MultiThreadedExecutor that alone costs ~1 core per node (measured:
108 % for a node doing nothing else; SingleThreadedExecutor 21 %, EventsExecutor 7 %).

offload_sim_clock(node) moves that work to a small helper node ("<name>_clock") spun by an EventsExecutor in a
daemon thread: the helper's TimeSource drives the node's ROS clock, and the node's own /clock subscription is
removed. The node keeps use_sim_time = true and its own executor; its clock, timers and rates behave as before
(sim-time timers are woken by the clock update itself, whichever thread delivers it).
"""
import threading

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.experimental import EventsExecutor
from rclpy.parameter import Parameter
from rclpy.time_source import TimeSource


def offload_sim_clock(node):
    """Drive `node`'s clock from /clock on a helper node + EventsExecutor thread. Returns that executor, or None
    when the node does not use sim time (nothing to do)."""
    if not node.get_parameter("use_sim_time").value:
        return None
    helper = rclpy.create_node(f"{node.get_name()}_clock", namespace=node.get_namespace(), context=node.context,
                               parameter_overrides=[Parameter("use_sim_time", value=False)],
                               start_parameter_services=False)
    ts = TimeSource(node=helper)
    ts.attach_clock(node.get_clock())
    ts.ros_time_is_active = True            # subscribes /clock on the helper; the node's clock stays on sim time
    node._time_source.detach_node()          # drop the node's own /clock subscription (rclpy has no public API)
    helper._sim_clock_source = ts            # keep the TimeSource alive with the helper
    ex = EventsExecutor(context=node.context)
    ex.add_node(helper)
    threading.Thread(target=_spin_quietly, args=(ex,), name=f"{node.get_name()}_clock", daemon=True).start()
    return ex


def _spin_quietly(ex):
    try:
        ex.spin()
    except ExternalShutdownException:      # rclpy.shutdown() from the node's main thread: normal exit
        pass


def spin_events(node):
    """Spin a node whose callbacks never block on the EventsExecutor (lowest per-callback overhead in rclpy),
    including its own /clock subscription."""
    ex = EventsExecutor(context=node.context)
    ex.add_node(node)
    ex.spin()
