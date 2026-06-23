#!/usr/bin/env python3

import time
from collections import deque
from copy import deepcopy

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String


class TrackerRgbdBridge(Node):
    def __init__(self):
        super().__init__('tracker_rgbd_bridge')

        self.declare_parameter('input_rgb_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('input_depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('output_rgb_topic', '/tracker/color/image_raw')
        self.declare_parameter('output_depth_topic', '/tracker/aligned_depth_to_color/image_raw')
        self.declare_parameter('prompt_topic', '/target_prompt')
        self.declare_parameter('goal_locked_topic', '/target_goal_locked')
        self.declare_parameter('max_publish_rate', 1.0)
        self.declare_parameter('sync_tolerance', 0.15)
        self.declare_parameter('pause_on_goal_lock', True)
        self.declare_parameter('jpeg_quality', 90)

        self.input_rgb_topic = self.get_parameter('input_rgb_topic').value
        self.input_depth_topic = self.get_parameter('input_depth_topic').value
        self.output_rgb_topic = self.get_parameter('output_rgb_topic').value
        self.output_depth_topic = self.get_parameter('output_depth_topic').value
        self.prompt_topic = self.get_parameter('prompt_topic').value
        self.goal_locked_topic = self.get_parameter('goal_locked_topic').value
        self.max_publish_rate = float(self.get_parameter('max_publish_rate').value)
        self.sync_tolerance = float(self.get_parameter('sync_tolerance').value)
        self.pause_on_goal_lock = bool(self.get_parameter('pause_on_goal_lock').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.publish_period = 0.0 if self.max_publish_rate <= 0.0 else 1.0 / self.max_publish_rate
        self.rgb_buffer = deque(maxlen=30)
        self.depth_buffer = deque(maxlen=30)
        self.bridge = CvBridge()
        self.current_prompt = None
        self.goal_locked = False
        self.streaming_enabled = False
        self.last_publish_time = 0.0
        self.published_pairs = 0
        self.last_sync_warn_time = 0.0
        self.sync_warn_period = 2.0

        self.rgb_pub = self.create_publisher(CompressedImage, self.output_rgb_topic, 10)
        self.depth_pub = self.create_publisher(Image, self.output_depth_topic, 10)

        self.rgb_sub = self.create_subscription(
            Image,
            self.input_rgb_topic,
            self.rgb_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.depth_sub = self.create_subscription(
            Image,
            self.input_depth_topic,
            self.depth_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
        )
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

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'Continuous RGB-D bridge ready: {self.input_rgb_topic} + {self.input_depth_topic} -> '
            f'{self.output_rgb_topic} (compressed RGB) + {self.output_depth_topic} (raw depth); '
            f'max_rate={self.max_publish_rate:.2f} Hz, sync_tolerance={self.sync_tolerance:.3f}s, '
            f'jpeg_quality={self.jpeg_quality}'
        )

    def prompt_callback(self, msg):
        if msg.data == self.current_prompt and self.streaming_enabled:
            self.get_logger().info(
                f'Ignoring duplicate prompt "{msg.data}" while continuous RGB-D export is already active.'
            )
            return

        self.current_prompt = msg.data
        self.goal_locked = False
        self.streaming_enabled = bool(self.current_prompt)
        if self.streaming_enabled:
            self.get_logger().info(
                f'Enabled continuous RGB-D export because prompt "{self.current_prompt}" was received.'
            )
        else:
            self.get_logger().info('Disabled continuous RGB-D export because an empty prompt was received.')

    def goal_locked_callback(self, msg):
        self.goal_locked = bool(msg.data)
        if self.pause_on_goal_lock and self.goal_locked:
            completed_prompt = self.current_prompt
            self.streaming_enabled = False
            self.current_prompt = None
            self.get_logger().info('Disabled continuous RGB-D export because goal lock was received.')
            if completed_prompt:
                self.get_logger().info(
                    f'Completed prompt "{completed_prompt}" and cleared continuous RGB-D bridge state. '
                    'Waiting for the next prompt.'
                )

    def rgb_callback(self, msg):
        self.rgb_buffer.append({
            'stamp_ns': self._stamp_to_ns(msg.header.stamp),
            'msg': msg,
        })

    def depth_callback(self, msg):
        self.depth_buffer.append({
            'stamp_ns': self._stamp_to_ns(msg.header.stamp),
            'msg': msg,
        })

    def timer_callback(self):
        if not self.streaming_enabled or not self.rgb_buffer or not self.depth_buffer:
            return

        now = time.monotonic()
        if self.publish_period > 0.0 and (now - self.last_publish_time) < self.publish_period:
            return

        rgb_entry = self.rgb_buffer[-1]
        depth_entry = self._find_matching_depth(rgb_entry['stamp_ns'])
        if depth_entry is None:
            return

        rgb_msg = deepcopy(rgb_entry['msg'])
        depth_msg = deepcopy(depth_entry['msg'])
        depth_msg.header.stamp = rgb_msg.header.stamp

        compressed_rgb_msg = self._compress_rgb(rgb_msg)
        if compressed_rgb_msg is None:
            return

        self.rgb_pub.publish(compressed_rgb_msg)
        self.depth_pub.publish(depth_msg)
        self.last_publish_time = now
        self.published_pairs += 1

        if self.published_pairs == 1 or self.published_pairs % 20 == 0:
            delta_s = abs(depth_entry['stamp_ns'] - rgb_entry['stamp_ns']) / 1e9
            self.get_logger().info(
                f'Published continuous RGB-D pair {self.published_pairs} '
                f'(stamp delta={delta_s:.3f}s).'
            )

    def _find_matching_depth(self, rgb_stamp_ns):
        if not self.depth_buffer:
            return None

        best = min(self.depth_buffer, key=lambda entry: abs(entry['stamp_ns'] - rgb_stamp_ns))
        delta_s = abs(best['stamp_ns'] - rgb_stamp_ns) / 1e9
        if delta_s > self.sync_tolerance:
            now = time.monotonic()
            if (now - self.last_sync_warn_time) >= self.sync_warn_period:
                self.last_sync_warn_time = now
                self.get_logger().warn(
                    f'Cannot export continuous RGB-D pair: closest depth frame is {delta_s:.3f}s away '
                    f'from RGB stamp, tolerance is {self.sync_tolerance:.3f}s.'
                )
            return None

        return best

    def _compress_rgb(self, rgb_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'RGB conversion failed while exporting continuous frame: {exc}')
            return None

        ok, encoded = cv2.imencode(
            '.jpg',
            image,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self.get_logger().warn('JPEG encoding failed for continuous tracker RGB frame.')
            return None

        compressed = CompressedImage()
        compressed.header = rgb_msg.header
        compressed.format = 'jpeg'
        compressed.data = np.asarray(encoded).tobytes()
        return compressed

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = TrackerRgbdBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
