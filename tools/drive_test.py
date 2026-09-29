#!/usr/bin/env python3
"""Drive straight at a fixed speed for N sim-seconds on the current world and
log position, height, and tilt every few sim-seconds. Run inside the container
with a sim up:  python3 /tools/drive_test.py [speed] [sim_seconds]"""
import math
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

sys.path.insert(0, '/tools')
from bench import rover_pose_full, wait_for_subscriber  # noqa: E402


class Drive(Node):
    def __init__(self, speed):
        super().__init__('jezero_drive_test')
        self.sim_t = None
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist = Twist()
        self.twist.linear.x = speed
        self.create_timer(0.05, lambda: self.pub.publish(self.twist))

    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9


def main():
    speed = float(sys.argv[1]) if len(sys.argv) > 1 else 0.3
    duration = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    rclpy.init()
    node = Drive(speed)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    while node.sim_t is None:
        time.sleep(0.1)
    wait_for_subscriber(node, node.pub)
    t0, w0 = node.sim_t, time.time()
    x0, y0, z0, *_ = rover_pose_full()
    print(f'{"sim_s":>6} {"x":>8} {"y":>8} {"z":>6} {"dist":>6} {"roll":>6} {"pitch":>6}')
    while True:
        x, y, z, roll, pitch, _ = rover_pose_full()
        t = node.sim_t - t0
        print(f'{t:6.1f} {x:8.2f} {y:8.2f} {z:6.2f} {math.hypot(x - x0, y - y0):6.2f} '
              f'{math.degrees(roll):6.1f} {math.degrees(pitch):6.1f}', flush=True)
        if t >= duration:
            break
        time.sleep(0.3)
    print(f'climbed {z - z0:+.2f} m over {math.hypot(x - x0, y - y0):.2f} m; '
          f'RTF {(node.sim_t - t0) / (time.time() - w0):.1f}')
    executor.shutdown()
    spinner.join(timeout=5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
