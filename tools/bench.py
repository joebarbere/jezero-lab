#!/usr/bin/env python3
"""Drive benchmark for a running OSR sim. Run inside the container after
`ros2 launch osr_gz sim.launch.py` (or upstream osr_gazebo) is up:

    python3 /osr_ws/src/../tools/bench.py        # see containers/bench.sh

Measures, against sim time:
  - real-time factor (sim seconds per wall second, from /clock)
  - steady-state forward speed for 0.3 m/s commanded
  - yaw change for rotate-in-place (angular.y = 0.5)

Position comes from the simulator's own CLI (`gz model`), because upstream
runs the rover node with odometry disabled. Heading comes from the IMU, which
streams, so turns are unwrapped continuously even when the sim runs many times
faster than real time.
"""
import math
import os
import re
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

HARMONIC = os.environ.get('ROS_DISTRO') != 'humble'
NUM = r'[-+]?\d*\.?\d+(?:[eE][-+]?\d+)?'


def rover_pose_full():
    """(x, y, z, roll, pitch, yaw) of model 'rover', from the simulator CLI."""
    out = subprocess.run(['gz', 'model', '-m', 'rover', '-p'],
                         capture_output=True, text=True, timeout=20).stdout
    if HARMONIC:
        # gz-sim 8: "Pose [ XYZ (m) ] [ RPY (rad) ]:\n [x y z]\n [r p y]"
        vecs = re.findall(r'\[\s*(' + NUM + r')\s+(' + NUM + r')\s+(' + NUM + r')\s*\]', out)
        if len(vecs) < 2:
            raise RuntimeError(f'could not parse gz model output:\n{out}')
        (x, y, z), (roll, pitch, yaw) = vecs[0], vecs[1]
    else:
        # Gazebo 11: first line "x y z roll pitch yaw"
        x, y, z, roll, pitch, yaw = out.split()[:6]
    return tuple(float(v) for v in (x, y, z, roll, pitch, yaw))


def rover_pose():
    """(x, y, yaw) of model 'rover'."""
    x, y, _, _, _, yaw = rover_pose_full()
    return x, y, yaw

def wait_for_subscriber(node, pub, timeout=30.0):
    """DDS discovery takes ~1-2 s of wall time -- tens of sim-seconds when
    running unthrottled -- so don't start timing until /cmd_vel has a reader."""
    deadline = time.time() + timeout
    while pub.get_subscription_count() == 0:
        if time.time() > deadline:
            sys.exit('/cmd_vel has no subscriber: is the rover node running?')
        time.sleep(0.05)


def gz_stats():
    """(sim_time, real_time) in seconds from gz-sim's /stats topic."""
    out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', '/stats'],
                         capture_output=True, text=True, timeout=20).stdout

    def t(key):
        m = re.search(key + r' \{\s*(?:sec: (\d+))?\s*(?:nsec: (\d+))?', out)
        return int(m.group(1) or 0) + int(m.group(2) or 0) / 1e9
    return t('sim_time'), t('real_time')


class Bench(Node):
    def __init__(self):
        super().__init__('jezero_bench')
        self.sim_t = None
        self.create_subscription(Clock, '/clock', self.on_clock, qos_profile_sensor_data)
        # osr_gz publishes /imu; upstream osr_gazebo publishes /imu_plugin/out.
        self.yaw_total = 0.0  # unwrapped, radians
        self._last_yaw = None
        for topic in ('/imu', '/imu_plugin/out'):
            self.create_subscription(Imu, topic, self.on_imu, qos_profile_sensor_data)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cmd = Twist()
        self.create_timer(0.1, lambda: self.pub.publish(self._cmd))

    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9

    @property
    def cmd(self):
        return self._cmd

    @cmd.setter
    def cmd(self, twist):
        # Publish immediately: the 0.1 s repeat timer is wall time, which is
        # several sim-seconds when running unthrottled.
        self._cmd = twist
        self.pub.publish(twist)

    def on_imu(self, msg):
        q = msg.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self._last_yaw is not None:
            d = yaw - self._last_yaw
            self.yaw_total += math.atan2(math.sin(d), math.cos(d))
        self._last_yaw = yaw

    def wait_clock(self):
        deadline = time.time() + 30
        while self.sim_t is None:
            if time.time() > deadline:
                sys.exit('no /clock after 30 s: is the sim running and unpaused?')
            time.sleep(0.1)

    def stamped_pose(self):
        """rover_pose() plus the sim time at the middle of the CLI call, which
        takes ~0.5 s of wall time -- many sim-seconds when running unthrottled."""
        before = self.sim_t
        pose = rover_pose()
        return pose, (before + self.sim_t) / 2

    def sim_sleep(self, seconds):
        """Sleep for `seconds` of sim time; returns wall seconds elapsed."""
        start_sim, start_wall = self.sim_t, time.time()
        while self.sim_t - start_sim < seconds:
            time.sleep(0.02)
        return time.time() - start_wall


def main():
    rclpy.init()
    node = Bench()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    node.wait_clock()
    wait_for_subscriber(node, node.pub)

    # Forward: settle for 2 sim-s, then measure over 5 sim-s.
    fwd = Twist()
    fwd.linear.x = 0.3
    node.cmd = fwd
    node.sim_sleep(2.0)
    (x0, y0, _), t0 = node.stamped_pose()
    stats0 = gz_stats() if HARMONIC else None
    wall = node.sim_sleep(5.0)
    stats1 = gz_stats() if HARMONIC else None
    (x1, y1, _), t1 = node.stamped_pose()
    # On Harmonic, RTF comes from gz-sim's own sim/real clocks; the wall-clock
    # estimate below also counts this script's ROS latency and reads low.
    if HARMONIC:
        rtf = (stats1[0] - stats0[0]) / (stats1[1] - stats0[1])
    else:
        rtf = 5.0 / wall
    speed = math.hypot(x1 - x0, y1 - y0) / (t1 - t0)

    # Rotate in place: stop, let the corners steer, then 4 sim-s of angular.y.
    node.cmd = Twist()
    node.sim_sleep(1.0)
    (rx0, ry0, _), _ = node.stamped_pose()
    rot = Twist()
    rot.angular.y = 0.5
    node.cmd = rot
    yaw0 = node.yaw_total
    node.sim_sleep(4.0)
    dyaw = math.degrees(node.yaw_total - yaw0)
    node.cmd = Twist()
    node.sim_sleep(0.5)
    (rx1, ry1, _), _ = node.stamped_pose()
    drift = math.hypot(rx1 - rx0, ry1 - ry0)
    if node._last_yaw is None:
        dyaw = float('nan')  # no IMU topic seen

    print(f'rtf={rtf:.2f} speed={speed:.3f}m/s(cmd 0.3) '
          f'rotate: dyaw={dyaw:+.1f}deg/4s drift={drift * 100:.1f}cm')
    executor.shutdown()
    spinner.join(timeout=5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
