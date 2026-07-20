#!/usr/bin/env python3
"""Edge-side CameraInfo relay (single-egress camera plan).

Subscribes the Pi's color CameraInfo once and republishes it on the edge-local
/camera_edge namespace so every edge consumer (RTAB-Map, detector, tools) reads
intrinsics locally instead of each opening its own Wi-Fi subscription.

QoS: the input subscription is BEST_EFFORT (compatible with both reliable and
sensor-data publishers, so it works regardless of how the RealSense driver
offers camera_info). The output is RELIABLE + TRANSIENT_LOCAL(depth=1) so a
late-joining edge consumer immediately receives the latest intrinsics.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo


class CameraInfoRelay(Node):
    def __init__(self):
        super().__init__('camera_info_relay')
        self.declare_parameter('input_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('output_topic', '/camera_edge/color/camera_info')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        sub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._pub = self.create_publisher(CameraInfo, output_topic, pub_qos)
        self._sub = self.create_subscription(CameraInfo, input_topic, self._on_info, sub_qos)
        self._relayed = 0

        self.get_logger().info(f'CameraInfo relay ready: {input_topic} -> {output_topic}')

    def _on_info(self, msg):
        self._pub.publish(msg)
        self._relayed += 1
        if self._relayed == 1:
            self.get_logger().info(
                'First CameraInfo relayed (fx=%.1f fy=%.1f cx=%.1f cy=%.1f, %dx%d).'
                % (msg.k[0], msg.k[4], msg.k[2], msg.k[5], msg.width, msg.height))


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
