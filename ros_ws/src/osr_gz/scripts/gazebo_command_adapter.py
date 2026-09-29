#!/usr/bin/env python3
# Copied unchanged from osr-rover-code/ROS/osr_gazebo/scripts (Apache-2.0,
# Copyright 2018 California Institute of Technology).
"""Bridge CommandDrive / CommandCorner from rover.py to Gazebo ros2_control topics.

Hardware CommandDrive uses opposite signs on the right-side motors (mounting).
Gazebo joint axes expect the same rotational sense for forward motion on all
wheels, so right-side velocities are negated again here.
Corner commands use the motor frame from rover.py (z down); Gazebo steering
joints use the opposite sign convention.
"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from osr_interfaces.msg import CommandCorner, CommandDrive


class GazeboCommandAdapter(Node):
    # Matches wheel_controller joints order in controller_velocity.yaml
    WHEEL_JOINT_ORDER = (
        'middle_wheel_joint_right',
        'middle_wheel_joint_left',
        'front_wheel_joint_right',
        'front_wheel_joint_left',
        'rear_wheel_joint_right',
        'rear_wheel_joint_left',
    )
    SERVO_JOINT_NAMES = (
        'front_wheel_joint_R',
        'front_wheel_joint_L',
        'rear_wheel_joint_R',
        'rear_wheel_joint_L',
    )

    def __init__(self):
        super().__init__('gazebo_command_adapter')
        self.declare_parameter('trajectory_time_from_start', 0.2)

        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_controller/commands', 1)
        self.servo_pub = self.create_publisher(
            JointTrajectory, '/servo_controller/joint_trajectory', 1)

        self.create_subscription(CommandDrive, '/cmd_drive', self.drive_cb, 1)
        self.create_subscription(CommandCorner, '/cmd_corner', self.corner_cb, 1)
        self.get_logger().info(
            'Bridging /cmd_drive and /cmd_corner to Gazebo controllers')

    def drive_cb(self, msg: CommandDrive):
        # Undo hardware right-side sign convention for Gazebo joint axes
        out = Float64MultiArray()
        out.data = [
            -msg.right_middle_vel,
            msg.left_middle_vel,
            -msg.right_front_vel,
            msg.left_front_vel,
            -msg.right_back_vel,
            msg.left_back_vel,
        ]
        self.wheel_pub.publish(out)

    def corner_cb(self, msg: CommandCorner):
        traj = JointTrajectory()
        traj.joint_names = list(self.SERVO_JOINT_NAMES)
        point = JointTrajectoryPoint()
        # Invert motor-frame (z-down) angles into Gazebo joint frames
        point.positions = [
            -msg.right_front_pos,
            -msg.left_front_pos,
            -msg.right_back_pos,
            -msg.left_back_pos,
        ]
        point.velocities = [0.0, 0.0, 0.0, 0.0]
        dt = self.get_parameter('trajectory_time_from_start').get_parameter_value().double_value
        point.time_from_start.sec = int(dt)
        point.time_from_start.nanosec = int((dt % 1.0) * 1e9)
        traj.points = [point]
        self.servo_pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCommandAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
