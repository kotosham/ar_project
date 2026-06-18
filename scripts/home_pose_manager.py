#!/usr/bin/env python3

import math

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty


class HomePoseManager(Node):
    def __init__(self) -> None:
        super().__init__('home_pose_manager')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('home_pose_topic', '/home_pose')
        self.declare_parameter('save_home_topic', '/save_home_pose')
        self.declare_parameter('return_home_topic', '/return_home')
        self.declare_parameter('auto_capture_on_start', True)

        self.target_frame = self.get_parameter('target_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.home_pose_topic = self.get_parameter('home_pose_topic').value
        self.save_home_topic = self.get_parameter('save_home_topic').value
        self.return_home_topic = self.get_parameter('return_home_topic').value
        self.auto_capture_on_start = bool(self.get_parameter('auto_capture_on_start').value)

        latched_pose_qos = QoSProfile(depth=1)
        latched_pose_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_pose_qos.reliability = ReliabilityPolicy.RELIABLE

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.home_pose_pub = self.create_publisher(PoseStamped, self.home_pose_topic, latched_pose_qos)
        self.save_home_sub = self.create_subscription(Empty, self.save_home_topic, self.save_home_callback, 10)
        self.return_home_sub = self.create_subscription(Empty, self.return_home_topic, self.return_home_callback, 10)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.home_pose = None
        self.auto_capture_attempts = 0
        self.auto_capture_timer = None

        self.get_logger().info(
            f'Home-pose manager ready: save on {self.save_home_topic}, return on {self.return_home_topic}, '
            f'publishing goals to {self.goal_topic}.'
        )

        if self.auto_capture_on_start:
            self.auto_capture_timer = self.create_timer(1.0, self.auto_capture_home_pose)

    def auto_capture_home_pose(self) -> None:
        if self.home_pose is not None:
            if self.auto_capture_timer is not None:
                self.auto_capture_timer.cancel()
            return

        self.auto_capture_attempts += 1
        if self._capture_home_pose('automatic startup capture'):
            if self.auto_capture_timer is not None:
                self.auto_capture_timer.cancel()
            return

        if self.auto_capture_attempts % 5 == 0:
            self.get_logger().info(
                f'Still waiting for TF {self.target_frame} -> {self.robot_frame} to auto-save the home pose '
                f'({self.auto_capture_attempts} attempts).'
            )

    def save_home_callback(self, _msg: Empty) -> None:
        self._capture_home_pose('manual save request')

    def return_home_callback(self, _msg: Empty) -> None:
        if self.home_pose is None:
            self.get_logger().warn(
                'Return-home requested, but no home pose is saved yet. '
                f'Publish Empty on {self.save_home_topic} first.'
            )
            return

        goal = PoseStamped()
        goal.header.frame_id = self.home_pose.header.frame_id
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = self.home_pose.pose
        self.goal_pub.publish(goal)

        yaw = self._quaternion_to_yaw(goal.pose.orientation)
        self.get_logger().info(
            f'Published return-home goal: x={goal.pose.position.x:.2f}, '
            f'y={goal.pose.position.y:.2f}, yaw={yaw:.2f} rad'
        )

    def _capture_home_pose(self, reason: str) -> bool:
        pose = self._lookup_current_pose()
        if pose is None:
            return False

        self.home_pose = pose
        self.home_pose_pub.publish(self.home_pose)

        yaw = self._quaternion_to_yaw(self.home_pose.pose.orientation)
        self.get_logger().info(
            f'Saved home pose ({reason}): x={self.home_pose.pose.position.x:.2f}, '
            f'y={self.home_pose.pose.position.y:.2f}, yaw={yaw:.2f} rad'
        )
        return True

    def _lookup_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().debug(f'Failed to look up current pose: {exc}')
            return None

        pose = PoseStamped()
        pose.header.frame_id = self.target_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    @staticmethod
    def _quaternion_to_yaw(quaternion) -> float:
        return math.atan2(
            2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
            1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
        )


def main() -> None:
    rclpy.init()
    node = HomePoseManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
