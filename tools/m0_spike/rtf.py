import time, rclpy
from rosgraph_msgs.msg import Clock
rclpy.init(); n = rclpy.create_node("rtf"); s = []
n.create_subscription(Clock, "/clock", lambda m: s.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9)), 100)
t0 = time.time()
while time.time() - t0 < 8: rclpy.spin_once(n, timeout_sec=0.1)
w0, c0 = s[len(s)//4]; w1, c1 = s[-1]
print(f"RTF={(c1-c0)/(w1-w0):.3f} over {w1-w0:.1f}s wall, clock msgs/s={len(s)/(w1-s[0][0]):.0f}")
