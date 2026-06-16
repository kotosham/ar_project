#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String, UInt32


class TrackerRgbBridge(Node):
    def __init__(self):
        super().__init__('tracker_rgb_bridge')

        self.declare_parameter('input_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('output_topic', '/tracker/color/image/compressed')
        self.declare_parameter('jpeg_quality', 90)
        self.declare_parameter('max_publish_rate', 3.0)
        self.declare_parameter('burst_frame_count', 3)
        self.declare_parameter('prompt_topic', '/target_prompt')
        self.declare_parameter('goal_locked_topic', '/target_goal_locked')
        self.declare_parameter('burst_complete_topic', '/tracker/burst_complete')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.max_publish_rate = float(self.get_parameter('max_publish_rate').value)
        self.burst_frame_count = int(self.get_parameter('burst_frame_count').value)
        self.prompt_topic = self.get_parameter('prompt_topic').value
        self.goal_locked_topic = self.get_parameter('goal_locked_topic').value
        self.burst_complete_topic = self.get_parameter('burst_complete_topic').value
        self.publish_period = 0.0 if self.max_publish_rate <= 0.0 else 1.0 / self.max_publish_rate

        self.bridge = CvBridge()
        self.last_publish_time = 0.0
        self.frames_seen = 0
        self.frames_published = 0
        self.current_prompt = None
        self.goal_locked = False
        self.subscription = None
        self.burst_frames_remaining = 0
        self.current_burst_published = 0

        self.publisher = self.create_publisher(CompressedImage, self.output_topic, 10)
        self.burst_complete_pub = self.create_publisher(UInt32, self.burst_complete_topic, 10)
        self.prompt_sub = self.create_subscription(String, self.prompt_topic, self.prompt_callback, 10)

        latched_state_qos = QoSProfile(depth=1)
        latched_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_state_qos.reliability = ReliabilityPolicy.RELIABLE
        self.goal_locked_sub = self.create_subscription(
            Bool,
            self.goal_locked_topic,
            self.goal_locked_callback,
            latched_state_qos,
        )

        self.get_logger().info(
            f'Bridging {self.input_topic} -> {self.output_topic} '
            f'with JPEG quality={self.jpeg_quality}, max_rate={self.max_publish_rate:.2f} Hz; '
            f'waiting for prompt on {self.prompt_topic}, '
            f'burst_frame_count={self.burst_frame_count}'
        )

    def image_callback(self, msg):
        self.frames_seen += 1
        now = self.get_clock().now().nanoseconds / 1e9
        if self.publish_period > 0.0 and (now - self.last_publish_time) < self.publish_period:
            return

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'RGB conversion failed: {exc}')
            return

        ok, encoded = cv2.imencode(
            '.jpg',
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().warn('JPEG encoding failed for tracker RGB frame')
            return

        compressed = CompressedImage()
        compressed.header = msg.header
        compressed.format = 'jpeg'
        compressed.data = np.asarray(encoded).tobytes()
        self.publisher.publish(compressed)

        self.last_publish_time = now
        self.frames_published += 1
        self.current_burst_published += 1

        if self.burst_frame_count > 0 and self.burst_frames_remaining > 0:
            self.burst_frames_remaining -= 1

        if self.frames_published == 1 or self.frames_published % 30 == 0:
            self.get_logger().info(
                f'Published tracker RGB frame {self.frames_published} '
                f'(seen {self.frames_seen} source frames)'
            )

        if self.burst_frame_count > 0 and self.burst_frames_remaining == 0:
            self._publish_burst_complete()
            self._set_streaming_enabled(False, 'burst completed')

    def prompt_callback(self, msg):
        if msg.data == self.current_prompt and self.subscription is not None:
            self.get_logger().info(
                f'Ignoring duplicate prompt "{msg.data}" while the current RGB burst is still active.'
            )
            return

        self.current_prompt = msg.data
        self.goal_locked = False
        if self.current_prompt:
            self.burst_frames_remaining = max(0, self.burst_frame_count)
            self._set_streaming_enabled(True, f'prompt "{self.current_prompt}" received')
        else:
            self.burst_frames_remaining = 0
            self._set_streaming_enabled(False, 'empty prompt received')

    def goal_locked_callback(self, msg):
        self.goal_locked = bool(msg.data)
        if self.goal_locked:
            self.burst_frames_remaining = 0
            self._set_streaming_enabled(False, 'goal lock received')
        elif self.current_prompt:
            self.burst_frames_remaining = max(0, self.burst_frame_count)
            self._set_streaming_enabled(True, f'goal lock cleared for prompt "{self.current_prompt}"')

    def _set_streaming_enabled(self, enabled, reason):
        if enabled and self.subscription is None:
            self.subscription = self.create_subscription(
                Image,
                self.input_topic,
                self.image_callback,
                rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
            )
            self.last_publish_time = 0.0
            self.current_burst_published = 0
            if self.burst_frame_count > 0:
                self.get_logger().info(
                    f'Enabled tracker RGB export because {reason}. '
                    f'Will publish up to {self.burst_frames_remaining} frame(s) in this burst.'
                )
            else:
                self.get_logger().info(f'Enabled tracker RGB export because {reason}.')
        elif not enabled and self.subscription is not None:
            self.destroy_subscription(self.subscription)
            self.subscription = None
            self.get_logger().info(f'Disabled tracker RGB export because {reason}.')

    def _publish_burst_complete(self):
        msg = UInt32()
        msg.data = max(0, self.current_burst_published)
        self.burst_complete_pub.publish(msg)
        self.get_logger().info(
            f'Published burst-complete signal with {msg.data} exported frame(s).'
        )


def main(args=None):
    rclpy.init(args=args)
    node = TrackerRgbBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
