#!/usr/bin/env python3

import math
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, String


class TargetPixelToGoal(Node):
    def __init__(self):
        super().__init__('target_pixel_to_goal')

        self.declare_parameter('target_pixel_topic', '/target_pixel')
        self.declare_parameter('target_mask_topic', '/target_mask')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('target_point_topic', '/experiment/target_point')
        self.declare_parameter('prompt_topic', '/target_prompt')
        self.declare_parameter('goal_locked_topic', '/target_goal_locked')
        self.declare_parameter('nav_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('fd_auto_topic', '/experiment/fd_auto')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('approach_offset', 0.58)
        self.declare_parameter('depth_match_tolerance', 0.35)
        self.declare_parameter('min_depth_m', 0.1)
        self.declare_parameter('max_depth_m', 6.0)
        self.declare_parameter('min_goal_update_distance', 0.15)
        self.declare_parameter('min_goal_update_angle', 0.2)
        self.declare_parameter('lock_goal_on_publish', True)
        self.declare_parameter('required_stable_detections', 3)
        self.declare_parameter('stable_pixel_tolerance', 40.0)
        self.declare_parameter('front_robot_x', 0.275)
        self.declare_parameter('front_robot_y', 0.0)
        self.declare_parameter('fd_distance_mode', 'forward')
        self.declare_parameter('fd_floor_z_threshold', -0.01)
        self.declare_parameter('fd_sample_step', 2)
        self.declare_parameter('fd_nearest_depth_band_m', 0.03)
        self.declare_parameter('fd_lateral_limit_m', 0.60)

        self.target_pixel_topic = self.get_parameter('target_pixel_topic').value
        self.target_mask_topic = self.get_parameter('target_mask_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.target_point_topic = self.get_parameter('target_point_topic').value
        self.prompt_topic = self.get_parameter('prompt_topic').value
        self.goal_locked_topic = self.get_parameter('goal_locked_topic').value
        self.nav_status_topic = self.get_parameter('nav_status_topic').value
        self.fd_auto_topic = self.get_parameter('fd_auto_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.approach_offset = float(self.get_parameter('approach_offset').value)
        self.depth_match_tolerance = float(self.get_parameter('depth_match_tolerance').value)
        self.min_depth_m = float(self.get_parameter('min_depth_m').value)
        self.max_depth_m = float(self.get_parameter('max_depth_m').value)
        self.min_goal_update_distance = float(self.get_parameter('min_goal_update_distance').value)
        self.min_goal_update_angle = float(self.get_parameter('min_goal_update_angle').value)
        self.lock_goal_on_publish = bool(self.get_parameter('lock_goal_on_publish').value)
        self.required_stable_detections = max(1, int(self.get_parameter('required_stable_detections').value))
        self.stable_pixel_tolerance = float(self.get_parameter('stable_pixel_tolerance').value)
        self.front_robot_x = float(self.get_parameter('front_robot_x').value)
        self.front_robot_y = float(self.get_parameter('front_robot_y').value)
        self.fd_distance_mode = str(self.get_parameter('fd_distance_mode').value).strip().lower()
        if self.fd_distance_mode not in ('planar', 'forward'):
            self.fd_distance_mode = 'forward'
        self.fd_floor_z_threshold = float(self.get_parameter('fd_floor_z_threshold').value)
        self.fd_sample_step = max(1, int(self.get_parameter('fd_sample_step').value))
        self.fd_nearest_depth_band_m = float(self.get_parameter('fd_nearest_depth_band_m').value)
        self.fd_lateral_limit_m = float(self.get_parameter('fd_lateral_limit_m').value)

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.depth_buffer = deque(maxlen=30)
        self.mask_buffer = deque(maxlen=30)
        self.pending_pixel_msgs = deque(maxlen=10)
        self.last_goal = None
        self.goal_locked = False
        self.current_prompt = None
        self.pending_pixel = None
        self.pending_pixel_count = 0
        self.pending_mask_wait_s = 0.25
        self.front_depth_margin_m = 0.02
        self.fd_auto_reported_for_current_goal = False
        self.current_goal_status_min_ns = 0

        latched_state_qos = QoSProfile(depth=1)
        latched_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        latched_state_qos.reliability = ReliabilityPolicy.RELIABLE

        self.goal_pub = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.target_point_pub = self.create_publisher(PointStamped, self.target_point_topic, 10)
        self.goal_locked_pub = self.create_publisher(Bool, self.goal_locked_topic, latched_state_qos)
        self.fd_auto_pub = self.create_publisher(Float32, self.fd_auto_topic, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, 1)
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.mask_sub = self.create_subscription(Image, self.target_mask_topic, self.target_mask_callback, 10)
        self.pixel_sub = self.create_subscription(PointStamped, self.target_pixel_topic, self.target_pixel_callback, 10)
        self.prompt_sub = self.create_subscription(String, self.prompt_topic, self.prompt_callback, 10)
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            self.nav_status_topic,
            self.nav_status_callback,
            10,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'Bridging {self.target_pixel_topic} + local depth -> {self.goal_topic} '
            f'using depth topic {self.depth_topic}; '
            f'approach_offset={self.approach_offset:.2f}m, '
            f'lock_goal_on_publish={self.lock_goal_on_publish}, '
            f'required_stable_detections={self.required_stable_detections}, '
            f'fd_auto_topic={self.fd_auto_topic}'
        )
        self._publish_goal_locked(False)

    def camera_info_callback(self, msg):
        if self.fx is None:
            self.fx = msg.k[0]
            self.fy = msg.k[4]
            self.cx = msg.k[2]
            self.cy = msg.k[5]
            self.get_logger().info(
                f'Camera intrinsics locked: fx={self.fx:.2f}, fy={self.fy:.2f}, '
                f'cx={self.cx:.2f}, cy={self.cy:.2f}'
            )

    def prompt_callback(self, msg):
        if msg.data == self.current_prompt and not self.goal_locked:
            self.get_logger().info(
                f'Ignoring duplicate prompt "{msg.data}" while the current target-selection cycle is still active.'
            )
            return

        self.current_prompt = msg.data
        self.goal_locked = False
        self.last_goal = None
        self.pending_pixel = None
        self.pending_pixel_count = 0
        self.pending_pixel_msgs.clear()
        self.fd_auto_reported_for_current_goal = False
        self.current_goal_status_min_ns = 0
        self._publish_goal_locked(False)
        self.get_logger().info(
            f'New prompt received on bridge: "{self.current_prompt}". '
            f'Cleared locked goal and stability history.'
        )

    def depth_callback(self, msg):
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

        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        self.depth_buffer.append(
            {
                'stamp_ns': stamp_ns,
                'stamp': msg.header.stamp,
                'frame_id': msg.header.frame_id,
                'depth': depth,
                'width': msg.width,
                'height': msg.height,
            }
        )
        self._process_pending_pixels()

    def target_mask_callback(self, msg):
        try:
            if msg.encoding not in ('mono8', '8UC1'):
                self.get_logger().warn(f'Unsupported target mask encoding: {msg.encoding}')
                return
            mask = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width)) > 0
        except Exception as exc:
            self.get_logger().error(f'Failed to decode target mask image: {exc}')
            return

        stamp_ns = self._stamp_to_ns(msg.header.stamp)
        self.mask_buffer.append(
            {
                'stamp_ns': stamp_ns,
                'stamp': msg.header.stamp,
                'mask': mask,
                'width': msg.width,
                'height': msg.height,
            }
        )
        self._process_pending_pixels()

    def target_pixel_callback(self, msg):
        if self.goal_locked and self.lock_goal_on_publish:
            return

        if not self._update_stability(msg):
            return

        self.pending_pixel_msgs.append(
            {
                'msg': msg,
                'received_monotonic': time.monotonic(),
            }
        )
        self._process_pending_pixels()

    def _process_pending_pixels(self):
        if self.fx is None or not self.depth_buffer or not self.pending_pixel_msgs:
            return

        remaining = deque(maxlen=self.pending_pixel_msgs.maxlen)
        while self.pending_pixel_msgs:
            entry = self.pending_pixel_msgs.popleft()
            msg = entry['msg']
            target_stamp_ns = self._stamp_to_ns(msg.header.stamp)
            depth_frame = self._find_matching_depth_frame(target_stamp_ns)
            if depth_frame is None:
                if (time.monotonic() - entry['received_monotonic']) < self.pending_mask_wait_s:
                    remaining.append(entry)
                else:
                    self.get_logger().warn('No matching depth frame found for target pixel')
                continue

            mask_frame = self._find_matching_mask_frame(target_stamp_ns)
            if mask_frame is None and (time.monotonic() - entry['received_monotonic']) < self.pending_mask_wait_s:
                remaining.append(entry)
                continue

            if not self._process_target_pixel_message(msg, depth_frame, mask_frame):
                continue

            if self.goal_locked and self.lock_goal_on_publish:
                remaining.clear()
                self.pending_pixel_msgs.clear()
                break

        self.pending_pixel_msgs = remaining

    def _process_target_pixel_message(self, msg, depth_frame, mask_frame):
        u_center = int(round(msg.point.x))
        v_center = int(round(msg.point.y))
        if not (0 <= u_center < depth_frame['width'] and 0 <= v_center < depth_frame['height']):
            self.get_logger().warn(f'Target pixel out of bounds: ({u_center}, {v_center})')
            return False

        u, v = self._select_depth_pixel(u_center, v_center, depth_frame, mask_frame)
        depth_m = float(depth_frame['depth'][v, u])
        if not math.isfinite(depth_m) or depth_m < self.min_depth_m or depth_m > self.max_depth_m:
            self.get_logger().warn(f'Invalid depth at target pixel ({u}, {v}): {depth_m:.3f} m')
            return False

        x = (u - self.cx) * depth_m / self.fx
        y = (v - self.cy) * depth_m / self.fy
        z = depth_m

        point_stamped = PointStamped()
        point_stamped.header.frame_id = depth_frame['frame_id']
        point_stamped.header.stamp = depth_frame['stamp']
        point_stamped.point.x = float(x)
        point_stamped.point.y = float(y)
        point_stamped.point.z = float(z)

        try:
            transform_point = self.tf_buffer.lookup_transform(
                self.target_frame,
                depth_frame['frame_id'],
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            point_world = tf2_geometry_msgs.do_transform_point(point_stamped, transform_point)

            transform_base = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warn(f'TF lookup failed while building goal: {exc}')
            return False

        robot_x = transform_base.transform.translation.x
        robot_y = transform_base.transform.translation.y
        dx = point_world.point.x - robot_x
        dy = point_world.point.y - robot_y
        distance = math.hypot(dx, dy)

        if distance <= self.approach_offset:
            goal_x = robot_x
            goal_y = robot_y
        else:
            scale = max(0.0, (distance - self.approach_offset) / distance)
            goal_x = robot_x + dx * scale
            goal_y = robot_y + dy * scale

        yaw = math.atan2(dy, dx)
        goal = PoseStamped()
        goal.header.frame_id = self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.orientation = self._yaw_to_quaternion(yaw)

        if not self._should_publish_goal(goal, yaw):
            return False

        self.goal_pub.publish(goal)
        self.target_point_pub.publish(point_world)
        self.last_goal = goal
        self.current_goal_status_min_ns = self._stamp_to_ns(goal.header.stamp)
        if self.lock_goal_on_publish:
            self.goal_locked = True
            self._publish_goal_locked(True)
        self.get_logger().info(
            f'Published goal from target pixel ({u}, {v})'
            f'{" using nearest mask depth" if mask_frame is not None else ""}: '
            f'depth={depth_m:.2f}m goal=({goal_x:.2f}, {goal_y:.2f}) yaw={yaw:.2f}rad'
        )
        if self.goal_locked:
            self.get_logger().info('Goal locked for static-object experiment. Ignoring further pixel updates until a new prompt arrives.')
        return True

    def nav_status_callback(self, msg):
        if not self.goal_locked or self.fd_auto_reported_for_current_goal:
            return

        latest_terminal = None
        for status in msg.status_list:
            status_stamp_ns = self._stamp_to_ns(status.goal_info.stamp)
            if status_stamp_ns == 0 or status_stamp_ns < self.current_goal_status_min_ns:
                continue
            if status.status not in (4, 5, 6):
                continue
            if latest_terminal is None or status_stamp_ns >= latest_terminal[0]:
                latest_terminal = (status_stamp_ns, status.status)

        if latest_terminal is None or latest_terminal[1] != 4:
            return

        fd_auto = self._compute_fd_auto_from_latest_depth()
        if fd_auto is None:
            return
        self.fd_auto_pub.publish(Float32(data=float(fd_auto)))
        self.fd_auto_reported_for_current_goal = True
        self.get_logger().info(
            f'Published fd_auto_m={fd_auto:.3f} m on {self.fd_auto_topic} '
            f'for goal stamp {latest_terminal[0]}.'
        )

    def _select_depth_pixel(self, u_center, v_center, depth_frame, mask_frame):
        if mask_frame is None:
            return u_center, v_center

        mask = mask_frame['mask']
        if mask.shape != depth_frame['depth'].shape:
            mask = cv2.resize(
                mask.astype(np.uint8),
                (depth_frame['width'], depth_frame['height']),
                interpolation=cv2.INTER_NEAREST,
            ) > 0

        depth = depth_frame['depth']
        valid = (
            mask
            & np.isfinite(depth)
            & (depth >= self.min_depth_m)
            & (depth <= self.max_depth_m)
        )
        if not np.any(valid):
            self.get_logger().warn('Target mask had no valid depth pixels, falling back to mask center pixel.')
            return u_center, v_center

        valid_depths = depth[valid]
        nearest_depth = float(np.min(valid_depths))
        front_band = valid & (depth <= (nearest_depth + self.front_depth_margin_m))
        candidate_pixels = np.argwhere(front_band if np.any(front_band) else valid)
        if candidate_pixels.size == 0:
            return u_center, v_center

        dv = candidate_pixels[:, 0] - v_center
        du = candidate_pixels[:, 1] - u_center
        nearest_idx = int(np.argmin(du * du + dv * dv))
        v = int(candidate_pixels[nearest_idx, 0])
        u = int(candidate_pixels[nearest_idx, 1])
        return u, v

    def _find_matching_mask_frame(self, target_stamp_ns):
        if not self.mask_buffer:
            return None

        best = min(self.mask_buffer, key=lambda frame: abs(frame['stamp_ns'] - target_stamp_ns))
        delta_s = abs(best['stamp_ns'] - target_stamp_ns) / 1e9
        if delta_s > self.depth_match_tolerance:
            return None
        return best

    def _compute_fd_auto_from_latest_depth(self):
        if self.fx is None or not self.depth_buffer:
            self.get_logger().warn('Cannot compute fd_auto_m: no camera intrinsics or no depth frame available yet.')
            return None

        depth_frame = self.depth_buffer[-1]
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

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                depth_frame['frame_id'],
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
        except Exception as exc:
            self.get_logger().warn(f'Cannot compute fd_auto_m: TF lookup failed: {exc}')
            return None

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
            self.get_logger().warn(
                'Cannot compute fd_auto_m: no valid depth samples remained after filtering.'
            )
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
        best_point = candidate_points[best_idx]
        fd_value = float(max(0.0, fd_values[best_idx]))
        self.get_logger().info(
            'fd_auto candidate selected from latest depth frame: '
            f'point_in_{self.robot_frame}=({best_point[0]:.3f}, {best_point[1]:.3f}, {best_point[2]:.3f}), '
            f'nearest_depth={nearest_depth:.3f}m, fd_auto_m={fd_value:.3f}m'
        )
        return fd_value

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

    def _update_stability(self, msg):
        pixel = np.array([float(msg.point.x), float(msg.point.y)], dtype=np.float32)

        if self.pending_pixel is None:
            self.pending_pixel = pixel
            self.pending_pixel_count = 1
            if self.required_stable_detections > 1:
                self.get_logger().info(
                    f'Accepted first candidate pixel ({pixel[0]:.0f}, {pixel[1]:.0f}); '
                    f'waiting for {self.required_stable_detections - 1} more stable detections.'
                )
            return self.required_stable_detections == 1

        pixel_delta = float(np.linalg.norm(pixel - self.pending_pixel))
        if pixel_delta <= self.stable_pixel_tolerance:
            self.pending_pixel_count += 1
            self.pending_pixel = pixel
        else:
            self.pending_pixel = pixel
            self.pending_pixel_count = 1
            self.get_logger().info(
                f'Reset stability window on target pixel jump; '
                f'new candidate=({pixel[0]:.0f}, {pixel[1]:.0f}), delta={pixel_delta:.1f}px'
            )

        if self.pending_pixel_count < self.required_stable_detections:
            return False

        return True

    def _find_matching_depth_frame(self, target_stamp_ns):
        if not self.depth_buffer:
            return None

        best = min(self.depth_buffer, key=lambda frame: abs(frame['stamp_ns'] - target_stamp_ns))
        delta_s = abs(best['stamp_ns'] - target_stamp_ns) / 1e9
        if delta_s > self.depth_match_tolerance:
            self.get_logger().warn(
                f'Closest depth frame is too far from RGB stamp: {delta_s:.3f}s > {self.depth_match_tolerance:.3f}s'
            )
            return None
        return best

    def _should_publish_goal(self, goal, yaw):
        if self.last_goal is None:
            return True

        dx = goal.pose.position.x - self.last_goal.pose.position.x
        dy = goal.pose.position.y - self.last_goal.pose.position.y
        distance_delta = math.hypot(dx, dy)
        last_yaw = self._quaternion_to_yaw(self.last_goal.pose.orientation)
        yaw_delta = abs(math.atan2(math.sin(yaw - last_yaw), math.cos(yaw - last_yaw)))
        return distance_delta >= self.min_goal_update_distance or yaw_delta >= self.min_goal_update_angle

    def _publish_goal_locked(self, locked):
        msg = Bool()
        msg.data = bool(locked)
        self.goal_locked_pub.publish(msg)

    @staticmethod
    def _yaw_to_quaternion(yaw):
        q = Quaternion()
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    @staticmethod
    def _quaternion_to_yaw(quaternion):
        return math.atan2(2.0 * quaternion.w * quaternion.z, 1.0 - 2.0 * quaternion.z * quaternion.z)

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPixelToGoal()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
