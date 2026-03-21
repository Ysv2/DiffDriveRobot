#!/usr/bin/env python3

import argparse
import math
import sys
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class OdomMotionCommander(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('odom_motion_commander')
        self.args = args
        self.current_odom: Optional[Odometry] = None
        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None
        self.start_yaw: Optional[float] = None
        self.goal_reached = False

        self.publisher = self.create_publisher(TwistStamped, args.cmd_topic, 10)
        self.subscription = self.create_subscription(
            Odometry,
            args.odom_topic,
            self.odom_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / args.rate_hz, self.control_loop)

        self.get_logger().info(
            f'Waiting for odometry on {args.odom_topic}, publishing commands to {args.cmd_topic}'
        )

    def odom_callback(self, msg: Odometry) -> None:
        self.current_odom = msg

        if self.start_x is None:
            position = msg.pose.pose.position
            orientation = msg.pose.pose.orientation
            self.start_x = position.x
            self.start_y = position.y
            self.start_yaw = yaw_from_quaternion(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            self.get_logger().info('Captured starting pose from odometry')

    def publish_stop(self) -> None:
        self.publisher.publish(self.build_twist_msg())

    def build_twist_msg(
        self,
        linear_x: float = 0.0,
        angular_z: float = 0.0,
    ) -> TwistStamped:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    def finish(self, reason: str) -> None:
        if self.goal_reached:
            return

        self.goal_reached = True
        self.publish_stop()
        self.get_logger().info(reason)
        self.destroy_timer(self.timer)

    def distance_progress(self) -> float:
        assert self.current_odom is not None
        assert self.start_x is not None
        assert self.start_y is not None
        assert self.start_yaw is not None

        position = self.current_odom.pose.pose.position
        dx = position.x - self.start_x
        dy = position.y - self.start_y
        return (math.cos(self.start_yaw) * dx) + (math.sin(self.start_yaw) * dy)

    def angle_progress(self) -> float:
        assert self.current_odom is not None
        assert self.start_yaw is not None

        orientation = self.current_odom.pose.pose.orientation
        current_yaw = yaw_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        return normalize_angle(current_yaw - self.start_yaw)

    def control_loop(self) -> None:
        if self.goal_reached:
            return

        if self.current_odom is None or self.start_x is None:
            return

        if self.args.distance is not None:
            progress = self.distance_progress()
            remaining = self.args.distance - progress

            if abs(remaining) <= self.args.distance_tolerance:
                self.finish(
                    f'Distance goal reached. Target: {self.args.distance:.3f} m, '
                    f'progress: {progress:.3f} m'
                )
                return

            self.publisher.publish(
                self.build_twist_msg(
                    linear_x=math.copysign(self.args.linear_speed, remaining)
                )
            )
            return

        progress = self.angle_progress()
        remaining = self.args.angle_rad - progress

        if abs(remaining) <= self.args.angle_tolerance_rad:
            self.finish(
                f'Angle goal reached. Target: {math.degrees(self.args.angle_rad):.2f} deg, '
                    f'progress: {math.degrees(progress):.2f} deg'
                )
            return

        self.publisher.publish(
            self.build_twist_msg(
                angular_z=math.copysign(self.args.angular_speed, remaining)
            )
        )


def positive_float(value: str) -> float:
    number = float(value)
    if number <= 0.0:
        raise argparse.ArgumentTypeError('value must be greater than zero')
    return number


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Command a robot to move a target distance or angle using /odom feedback.'
    )
    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument(
        '--distance',
        type=float,
        help='Target signed distance in meters. Positive is forward, negative is backward.',
    )
    target_group.add_argument(
        '--angle-deg',
        type=float,
        help='Target signed angle in degrees. Positive is left, negative is right.',
    )
    parser.add_argument(
        '--cmd-topic',
        default='/diff_drive_controller/cmd_vel',
        help='Velocity command topic.',
    )
    parser.add_argument(
        '--odom-topic',
        default='diff_drive_controller/odom',
        help='Odometry topic.',
    )
    parser.add_argument(
        '--linear-speed',
        type=positive_float,
        default=0.10,
        help='Commanded linear speed magnitude in m/s for distance moves.',
    )
    parser.add_argument(
        '--angular-speed',
        type=positive_float,
        default=0.20,
        help='Commanded angular speed magnitude in rad/s for turns.',
    )
    parser.add_argument(
        '--distance-tolerance',
        type=positive_float,
        default=0.01,
        help='Stop when distance error is within this many meters.',
    )
    parser.add_argument(
        '--angle-tolerance-deg',
        type=positive_float,
        default=0.5,
        help='Stop when angle error is within this many degrees.',
    )
    parser.add_argument(
        '--rate-hz',
        type=positive_float,
        default=20.0,
        help='Control loop rate.',
    )
    return parser


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = build_arg_parser()
    args = parser.parse_args(argv)
    args.angle_rad = None if args.angle_deg is None else math.radians(args.angle_deg)
    args.angle_tolerance_rad = math.radians(args.angle_tolerance_deg)
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv if argv is not None else sys.argv[1:])
    rclpy.init()
    node = OdomMotionCommander(args)

    try:
        while rclpy.ok() and not node.goal_reached:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user, stopping robot')
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
