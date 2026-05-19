#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node


class TwistToTwistStampedBridge(Node):
    def __init__(self) -> None:
        super().__init__('twist_to_twist_stamped_bridge')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/diff_cont/cmd_vel')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.publisher_ = self.create_publisher(TwistStamped, output_topic, 10)
        self.subscription_ = self.create_subscription(
            Twist,
            input_topic,
            self._twist_callback,
            10,
        )

        self.get_logger().info(
            f"Bridging Twist '{input_topic}' to TwistStamped '{output_topic}'."
        )

    def _twist_callback(self, msg: Twist) -> None:
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.twist = msg
        self.publisher_.publish(stamped)


def main() -> None:
    rclpy.init()
    node = TwistToTwistStampedBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
