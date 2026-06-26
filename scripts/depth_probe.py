#!/usr/bin/env python3

import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image


class DepthProbe(Node):
    def __init__(self):
        super().__init__('depth_probe')

        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('once', True)
        self.declare_parameter('report_period_s', 1.0)
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('front_robot_x', 0.275)
        self.declare_parameter('front_robot_y', 0.0)
        self.declare_parameter('fd_distance_mode', 'forward')
        self.declare_parameter('fd_sample_step', 2)
        self.declare_parameter('fd_nearest_depth_band_m', 0.03)
        self.declare_parameter('fd_lateral_limit_m', 0.60)

        self.depth_topic = str(self.get_parameter('depth_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.once = bool(self.get_parameter('once').value)
        self.report_period_s = float(self.get_parameter('report_period_s').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.front_robot_x = float(self.get_parameter('front_robot_x').value)
        self.front_robot_y = float(self.get_parameter('front_robot_y').value)
        self.fd_distance_mode = str(self.get_parameter('fd_distance_mode').value).strip().lower()
        if self.fd_distance_mode not in ('planar', 'forward'):
            self.fd_distance_mode = 'forward'
        self.fd_sample_step = max(1, int(self.get_parameter('fd_sample_step').value))
        self.fd_nearest_depth_band_m = float(self.get_parameter('fd_nearest_depth_band_m').value)
        self.fd_lateral_limit_m = float(self.get_parameter('fd_lateral_limit_m').value)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.latest_depth = None
        self.last_report_time = 0.0
        self.reported_once = False

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSPresetProfiles.SENSOR_DATA.value
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 1)
        self.create_subscription(Image, self.depth_topic, self.depth_callback, sensor_qos)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'Depth probe ready: depth_topic={self.depth_topic}, camera_info_topic={self.camera_info_topic}, '
            f'robot_frame={self.robot_frame}, once={self.once}, report_period_s={self.report_period_s:.2f}'
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        if self.fx is not None:
            return

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.get_logger().info(
            f'Camera intrinsics locked: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}'
        )

    def depth_callback(self, msg: Image) -> None:
        try:
            if msg.encoding == '16UC1':
                depth = np.frombuffer(msg.data, dtype=np.uint16).reshape((msg.height, msg.width)).astype(np.float32) / 1000.0
            elif msg.encoding == '32FC1':
                depth = np.frombuffer(msg.data, dtype=np.float32).reshape((msg.height, msg.width))
            else:
                self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
                return
        except Exception as exc:
            self.get_logger().error(f'Failed to decode depth image: {exc}')
            return

        self.latest_depth = {
            'frame_id': msg.header.frame_id,
            'stamp': msg.header.stamp,
            'depth': depth,
            'width': msg.width,
            'height': msg.height,
        }

    def timer_callback(self) -> None:
        if self.reported_once and self.once:
            self.destroy_node()
            rclpy.shutdown()
            return

        if self.fx is None or self.latest_depth is None:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        if not self.once and (now - self.last_report_time) < self.report_period_s:
            return

        result = self._compute_probe()
        if result is None:
            return

        nearest_depth, nearest_point_base, fd_auto = result
        self.get_logger().info(
            f'nearest_depth={nearest_depth:.3f}m, '
            f'nearest_point_in_{self.robot_frame}=({nearest_point_base[0]:.3f}, {nearest_point_base[1]:.3f}, {nearest_point_base[2]:.3f}), '
            f'fd_auto_m={fd_auto:.3f}m'
        )

        self.last_report_time = now
        if self.once:
            self.reported_once = True

    def _compute_probe(self):
        depth_frame = self.latest_depth
        depth = depth_frame['depth']

        valid = (
            np.isfinite(depth)
            & (depth >= self.min_depth_m)
            & (depth <= self.max_depth_m)
        )
        if not np.any(valid):
            self.get_logger().warn('No valid depth samples in the latest frame.')
            return None

        nearest_depth = float(np.min(depth[valid]))
        nearest_pixels = np.argwhere(valid & (depth <= (nearest_depth + self.fd_nearest_depth_band_m)))
        if nearest_pixels.size == 0:
            nearest_pixels = np.argwhere(valid)
        if nearest_pixels.size == 0:
            self.get_logger().warn('Nearest-depth candidate set is empty.')
            return None

        center_v = depth_frame['height'] / 2.0
        center_u = depth_frame['width'] / 2.0
        dv = nearest_pixels[:, 0].astype(np.float32) - center_v
        du = nearest_pixels[:, 1].astype(np.float32) - center_u
        nearest_idx = int(np.argmin(du * du + dv * dv))
        v = int(nearest_pixels[nearest_idx, 0])
        u = int(nearest_pixels[nearest_idx, 1])
        nearest_depth = float(depth[v, u])

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                depth_frame['frame_id'],
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed: {exc}')
            return None

        point_base = self._pixel_to_base_link_point(u, v, nearest_depth, transform)
        fd_auto = self._compute_fd_auto_from_latest_depth(transform)
        if fd_auto is None:
            return None

        return nearest_depth, point_base, fd_auto

    def _pixel_to_base_link_point(self, u: int, v: int, depth_m: float, transform):
        x_cam = (float(u) - self.cx) * depth_m / self.fx
        y_cam = (float(v) - self.cy) * depth_m / self.fy
        z_cam = depth_m
        point_cam = np.array([x_cam, y_cam, z_cam], dtype=np.float32)
        rotation = self._quaternion_to_rotation_matrix(transform.transform.rotation)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        point_base = point_cam @ rotation.T + translation
        return point_base

    def _compute_fd_auto_from_latest_depth(self, transform):
        depth_frame = self.latest_depth
        depth = depth_frame['depth']
        step = self.fd_sample_step
        sampled_depth = depth[::step, ::step]
        valid = (
            np.isfinite(sampled_depth)
            & (sampled_depth >= self.min_depth_m)
            & (sampled_depth <= self.max_depth_m)
        )
        if not np.any(valid):
            self.get_logger().warn('Cannot compute fd_auto_m: latest depth frame has no valid samples.')
            return None

        row_idx, col_idx = np.nonzero(valid)
        depths = sampled_depth[row_idx, col_idx].astype(np.float32)
        u = (col_idx * step).astype(np.float32)
        v = (row_idx * step).astype(np.float32)

        x_cam = (u - self.cx) * depths / self.fx
        y_cam = (v - self.cy) * depths / self.fy
        z_cam = depths
        points_cam = np.stack((x_cam, y_cam, z_cam), axis=1)

        rotation = self._quaternion_to_rotation_matrix(transform.transform.rotation)
        translation = np.array(
            [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ],
            dtype=np.float32,
        )
        points_robot = points_cam @ rotation.T + translation

        robot_valid = points_robot[:, 0] > self.front_robot_x
        if self.fd_lateral_limit_m > 0.0:
            robot_valid &= np.abs(points_robot[:, 1]) <= self.fd_lateral_limit_m

        if not np.any(robot_valid):
            self.get_logger().warn('Cannot compute fd_auto_m: no valid depth samples remained after filtering.')
            return None

        candidate_points = points_robot[robot_valid]
        candidate_depths = depths[robot_valid]
        nearest_depth = float(np.min(candidate_depths))
        nearest_band = candidate_depths <= (nearest_depth + self.fd_nearest_depth_band_m)
        candidate_points = candidate_points[nearest_band]

        if candidate_points.size == 0:
            self.get_logger().warn('Cannot compute fd_auto_m: nearest-depth band became empty.')
            return None

        if self.fd_distance_mode == 'forward':
            fd_values = candidate_points[:, 0] - self.front_robot_x
        else:
            fd_values = np.hypot(
                candidate_points[:, 0] - self.front_robot_x,
                candidate_points[:, 1] - self.front_robot_y,
            )

        best_idx = int(np.argmin(fd_values))
        return float(max(0.0, fd_values[best_idx]))

    @staticmethod
    def _quaternion_to_rotation_matrix(quaternion):
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z
        return np.array(
            [
                [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
                [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
                [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
            ],
            dtype=np.float32,
        )


def main(args=None):
    rclpy.init(args=args)
    node = DepthProbe()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
