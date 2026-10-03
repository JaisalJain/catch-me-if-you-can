#!/usr/bin/env python3
"""Measure the camera / detection rates from message stamps (ROS sourced, sim running): python3 tools/v2_live/rate_probe.py"""
import sys, time, rclpy
from sensor_msgs.msg import Image
from catch_interfaces.msg import CubeDetectionArray
rclpy.init(); n = rclpy.create_node("rate_probe", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
st = {}
def cb(topic):
    def f(m):
        st.setdefault(topic, []).append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    return f
for t, ty in (("/cameras/track_side/image_raw", Image), ("/cameras/track_top/image_raw", Image), ("/perception/overhead_a/cubes", CubeDetectionArray)):
    n.create_subscription(ty, t, cb(t), 50)
t0 = time.monotonic()
while time.monotonic() - t0 < 6: rclpy.spin_once(n, timeout_sec=0.05)
import numpy as np
for t, s in st.items():
    d = np.diff(s); print(t, len(s), "msgs; stamp spacing ms: median %.2f mean %.2f -> %.1f Hz" % (1000*np.median(d), 1000*d.mean(), 1/d.mean()))
