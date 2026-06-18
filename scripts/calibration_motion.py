#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


class CalibrationMotionPublisher(Node):
    def __init__(self, topic: str):
        super().__init__('calibration_motion_publisher')
        self.publisher = self.create_publisher(TwistStamped, topic, 10)
        self.topic = topic

    def publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(linear_x)
        msg.twist.angular.z = float(angular_z)
        self.publisher.publish(msg)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Publish a deterministic TwistStamped calibration motion with an explicit zero-velocity stop.'
    )
    parser.add_argument('--topic', default='/diff_cont/cmd_vel')
    parser.add_argument('--rate', type=float, default=50.0)
    parser.add_argument('--linear-speed', type=float, default=0.05)
    parser.add_argument('--angular-speed', type=float, default=0.3141592653589793)
    parser.add_argument('--zero-cycles', type=int, default=10)

    target_group = parser.add_mutually_exclusive_group(required=True)
    target_group.add_argument('--distance-m', type=float)
    target_group.add_argument('--angle-deg', type=float)
    target_group.add_argument('--angle-rad', type=float)

    return parser.parse_args()


def _compute_motion(args: argparse.Namespace) -> tuple[float, float, float, str]:
    if args.distance_m is not None:
        if args.distance_m == 0.0:
            raise ValueError('distance-m must be non-zero')
        speed = abs(args.linear_speed)
        if speed <= 0.0:
            raise ValueError('linear-speed must be positive')
        linear_x = math.copysign(speed, args.distance_m)
        duration_s = abs(args.distance_m) / speed
        return linear_x, 0.0, duration_s, f'{args.distance_m:.4f} m straight motion'

    angle_rad = args.angle_rad
    if args.angle_deg is not None:
        angle_rad = math.radians(args.angle_deg)
    if angle_rad is None or angle_rad == 0.0:
        raise ValueError('angle must be non-zero')

    speed = abs(args.angular_speed)
    if speed <= 0.0:
        raise ValueError('angular-speed must be positive')
    angular_z = math.copysign(speed, angle_rad)
    duration_s = abs(angle_rad) / speed
    return 0.0, angular_z, duration_s, f'{math.degrees(angle_rad):.4f} deg rotation'


def main(args=None):
    cli_args = _parse_args()
    linear_x, angular_z, duration_s, description = _compute_motion(cli_args)

    rclpy.init(args=args)
    node = CalibrationMotionPublisher(cli_args.topic)

    period_s = 1.0 / cli_args.rate
    end_time = time.monotonic() + duration_s

    node.get_logger().info(
        f'Starting calibration motion on {cli_args.topic}: {description}, '
        f'linear_x={linear_x:.6f} m/s, angular_z={angular_z:.6f} rad/s, '
        f'duration={duration_s:.6f} s, rate={cli_args.rate:.2f} Hz'
    )

    try:
        while rclpy.ok() and time.monotonic() < end_time:
            node.publish_twist(linear_x, angular_z)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)

        # Publish several explicit zero commands so stopping behavior doesn't depend on cmd_vel_timeout.
        for _ in range(max(cli_args.zero_cycles, 1)):
            node.publish_twist(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(period_s)
    finally:
        node.get_logger().info('Calibration motion finished.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
