#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class ReliablePromptSender(Node):
    def __init__(self):
        super().__init__('reliable_prompt_sender')

        self.declare_parameter('request_topic', '/target_prompt_request')
        self.declare_parameter('prompt_topic', '/target_prompt')
        self.declare_parameter('prompt_ack_topic', '/target_prompt_ack')
        self.declare_parameter('retry_period', 0.5)
        self.declare_parameter('max_retries', 12)

        self.request_topic = self.get_parameter('request_topic').value
        self.prompt_topic = self.get_parameter('prompt_topic').value
        self.prompt_ack_topic = self.get_parameter('prompt_ack_topic').value
        self.retry_period = float(self.get_parameter('retry_period').value)
        self.max_retries = int(self.get_parameter('max_retries').value)

        self.prompt_pub = self.create_publisher(String, self.prompt_topic, 10)
        self.request_sub = self.create_subscription(String, self.request_topic, self.request_callback, 10)
        latched_ack_qos = QoSProfile(depth=1)
        latched_ack_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_ack_qos.reliability = ReliabilityPolicy.RELIABLE
        self.ack_sub = self.create_subscription(
            String,
            self.prompt_ack_topic,
            self.ack_callback,
            latched_ack_qos,
        )
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.pending_prompt = None
        self.pending_request_time = 0.0
        self.last_send_time = 0.0
        self.retry_count = 0

        self.get_logger().info(
            f'Reliable prompt sender ready: {self.request_topic} -> {self.prompt_topic}, '
            f'waiting for ack on {self.prompt_ack_topic}; '
            f'retry_period={self.retry_period:.2f}s, max_retries={self.max_retries}'
        )

    def request_callback(self, msg):
        prompt = str(msg.data)
        self.pending_prompt = prompt
        self.pending_request_time = time.monotonic()
        self.last_send_time = 0.0
        self.retry_count = 0
        self.get_logger().info(f'Received prompt request: "{prompt}"')
        self._send_prompt()

    def ack_callback(self, msg):
        if self.pending_prompt is None:
            return

        if msg.data != self.pending_prompt:
            return

        self.get_logger().info(
            f'Prompt "{self.pending_prompt}" was acknowledged by Raspberry Pi after '
            f'{self.retry_count} send attempt(s).'
        )
        self.pending_prompt = None
        self.pending_request_time = 0.0
        self.last_send_time = 0.0
        self.retry_count = 0

    def timer_callback(self):
        if self.pending_prompt is None:
            return

        now = time.monotonic()
        if self.retry_count >= self.max_retries:
            self.get_logger().warn(
                f'Prompt "{self.pending_prompt}" was not acknowledged after {self.retry_count} attempt(s). '
                'Stopping retries and waiting for a new request.'
            )
            self.pending_prompt = None
            return

        if (now - self.last_send_time) >= self.retry_period:
            self._send_prompt()

    def _send_prompt(self):
        if self.pending_prompt is None:
            return

        msg = String()
        msg.data = self.pending_prompt
        self.prompt_pub.publish(msg)
        self.last_send_time = time.monotonic()
        self.retry_count += 1
        self.get_logger().info(
            f'Sent prompt "{self.pending_prompt}" to {self.prompt_topic} '
            f'(attempt {self.retry_count}/{self.max_retries}).'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ReliablePromptSender()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
