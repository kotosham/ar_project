#!/usr/bin/env python3

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self) -> None:
        super().__init__('odom_to_tf')
        self._fallback_child_frame = self.declare_parameter(
            'fallback_child_frame_id', 'base_link'
        ).value
        self._broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry,
            '/odom',
            self._handle_odom,
            10,
        )

    def _handle_odom(self, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.child_frame_id = msg.child_frame_id or self._fallback_child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main() -> None:
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
