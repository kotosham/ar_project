#!/usr/bin/env python3

import csv
import math
import time
from datetime import datetime
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32, String
import tf2_geometry_msgs
import tf2_ros


class ExperimentMetricsLogger(Node):
    def __init__(self):
        super().__init__('experiment_metrics_logger')

        self.declare_parameter('prompt_topic', '/target_prompt')
        self.declare_parameter('target_pixel_topic', '/target_pixel')
        self.declare_parameter('cv_runtime_topic', '/experiment/cv_runtime')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('target_point_topic', '/experiment/target_point')
        self.declare_parameter('fd_auto_topic', '/experiment/fd_auto')
        self.declare_parameter('nav_status_topic', '/navigate_to_pose/_action/status')
        self.declare_parameter('trial_timeout_s', 30.0)
        self.declare_parameter('output_csv', '~/ros2_ws/experiment_logs/experiment_metrics.csv')
        self.declare_parameter('enable_fd_auto_measurement', False)
        self.declare_parameter('fd_auto_wait_s', 1.0)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('front_robot_x', 0.275)
        self.declare_parameter('front_robot_y', 0.0)
        self.declare_parameter('fd_distance_mode', 'planar')

        self.prompt_topic = str(self.get_parameter('prompt_topic').value)
        self.target_pixel_topic = str(self.get_parameter('target_pixel_topic').value)
        self.cv_runtime_topic = str(self.get_parameter('cv_runtime_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.target_point_topic = str(self.get_parameter('target_point_topic').value)
        self.fd_auto_topic = str(self.get_parameter('fd_auto_topic').value)
        self.nav_status_topic = str(self.get_parameter('nav_status_topic').value)
        self.trial_timeout_s = float(self.get_parameter('trial_timeout_s').value)
        self.output_csv = Path(str(self.get_parameter('output_csv').value)).expanduser()
        self.enable_fd_auto_measurement = bool(self.get_parameter('enable_fd_auto_measurement').value)
        self.fd_auto_wait_s = float(self.get_parameter('fd_auto_wait_s').value)
        self.robot_frame = str(self.get_parameter('robot_frame').value)
        self.front_robot_x = float(self.get_parameter('front_robot_x').value)
        self.front_robot_y = float(self.get_parameter('front_robot_y').value)
        self.fd_distance_mode = str(self.get_parameter('fd_distance_mode').value).strip().lower()
        if self.fd_distance_mode not in ('planar', 'forward'):
            self.fd_distance_mode = 'planar'

        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.csv_fields = [
            'trial_id',
            'prompt',
            'prompt_time_iso',
            'cv_runtime_s',
            'goal_publish_latency_s',
            'trial_duration_s',
            'total_time_to_object_s',
            'fd_auto_m',
            'nav_outcome',
        ]
        self._ensure_csv_header()

        self.current_trial = None
        self.next_trial_id = self._load_next_trial_id()

        tracking_input_qos = QoSProfile(depth=1)
        tracking_input_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        tracking_input_qos.durability = DurabilityPolicy.VOLATILE

        self.prompt_sub = self.create_subscription(String, self.prompt_topic, self.prompt_callback, 10)
        self.target_pixel_sub = self.create_subscription(
            PointStamped,
            self.target_pixel_topic,
            self.target_pixel_callback,
            tracking_input_qos,
        )
        self.goal_sub = self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)
        self.cv_runtime_sub = self.create_subscription(
            Float32,
            self.cv_runtime_topic,
            self.cv_runtime_callback,
            10,
        )
        self.target_point_sub = self.create_subscription(
            PointStamped,
            self.target_point_topic,
            self.target_point_callback,
            10,
        )
        self.fd_auto_sub = self.create_subscription(
            Float32,
            self.fd_auto_topic,
            self.fd_auto_callback,
            10,
        )
        self.nav_status_sub = self.create_subscription(
            GoalStatusArray,
            self.nav_status_topic,
            self.nav_status_callback,
            10,
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info(
            f'Experiment metrics logger writing to {self.output_csv} '
            f'(timeout={self.trial_timeout_s:.1f}s, '
            f'fd_auto_measurement={"on" if self.enable_fd_auto_measurement else "off"}, '
            f'fd_auto_topic={self.fd_auto_topic}).'
        )

    def prompt_callback(self, msg):
        if self.current_trial is not None:
            self._finalize_trial('superseded', time.monotonic())

        now_monotonic = time.monotonic()
        now_iso = datetime.now().isoformat(timespec='seconds')
        self.current_trial = {
            'trial_id': self.next_trial_id,
            'prompt': msg.data,
            'prompt_time_iso': now_iso,
            'start_monotonic': now_monotonic,
            'start_ros_ns': self._now_ros_ns(),
            'target_pixel_monotonic': None,
            'cv_runtime_s': None,
            'goal_monotonic': None,
            'fd_auto_m': None,
            'target_point': None,
            'pending_outcome': None,
            'pending_end_monotonic': None,
            'fd_auto_wait_deadline_monotonic': None,
            'current_goal_status_min_ns': 0,
            'goal_publish_count': 0,
        }
        self.next_trial_id += 1

        self.get_logger().info(
            f'Started trial {self.current_trial["trial_id"]} for prompt "{msg.data}".'
        )

    def target_pixel_callback(self, _msg):
        if self.current_trial is None:
            return

        if self.current_trial['target_pixel_monotonic'] is None:
            self.current_trial['target_pixel_monotonic'] = time.monotonic()
            cv_runtime = (
                self.current_trial['target_pixel_monotonic'] - self.current_trial['start_monotonic']
            )
            self.get_logger().info(
                f'Trial {self.current_trial["trial_id"]}: target pixel received, '
                f'cv_runtime_s={cv_runtime:.3f}.'
            )

    def cv_runtime_callback(self, msg):
        if self.current_trial is None:
            return

        self.current_trial['cv_runtime_s'] = float(msg.data)

    def goal_callback(self, _msg):
        if self.current_trial is None:
            return

        goal_stamp_ns = self._stamp_to_ns(_msg.header.stamp)
        if goal_stamp_ns == 0:
            goal_stamp_ns = self._now_ros_ns()

        self.current_trial['current_goal_status_min_ns'] = goal_stamp_ns
        self.current_trial['goal_publish_count'] += 1

        if self.current_trial['goal_monotonic'] is None:
            self.current_trial['goal_monotonic'] = time.monotonic()
            goal_latency = self.current_trial['goal_monotonic'] - self.current_trial['start_monotonic']
            self.get_logger().info(
                f'Trial {self.current_trial["trial_id"]}: goal published, '
                f'goal_publish_latency_s={goal_latency:.3f}.'
            )
            return

        self.get_logger().info(
            f'Trial {self.current_trial["trial_id"]}: updated goal published '
            f'(count={self.current_trial["goal_publish_count"]}).'
        )

    def target_point_callback(self, msg):
        if self.current_trial is None:
            return

        self.current_trial['target_point'] = msg
        self.get_logger().info(
            f'Trial {self.current_trial["trial_id"]}: target point stored in frame "{msg.header.frame_id}".'
        )

    def fd_auto_callback(self, msg):
        if self.current_trial is None:
            return

        self.current_trial['fd_auto_m'] = float(msg.data)
        self.get_logger().info(
            f'Trial {self.current_trial["trial_id"]}: received fd_auto_m={msg.data:.3f}.'
        )

        if self.current_trial.get('pending_outcome') == 'succeeded':
            self._finalize_pending_success()

    def nav_status_callback(self, msg):
        if self.current_trial is None or self.current_trial['goal_monotonic'] is None:
            return

        current_goal_status_min_ns = self.current_trial.get('current_goal_status_min_ns', 0)
        if current_goal_status_min_ns == 0:
            return

        latest_terminal = None

        for status in msg.status_list:
            status_stamp_ns = self._stamp_to_ns(status.goal_info.stamp)
            if status_stamp_ns == 0 or status_stamp_ns < current_goal_status_min_ns:
                continue

            if status.status in (4, 5, 6):
                if latest_terminal is None or status_stamp_ns >= latest_terminal[0]:
                    latest_terminal = (status_stamp_ns, status.status)

        if latest_terminal is None:
            return

        outcome = {
            4: 'succeeded',
            5: 'canceled',
            6: 'aborted',
        }[latest_terminal[1]]
        end_monotonic = time.monotonic()
        if outcome == 'succeeded' and self.enable_fd_auto_measurement:
            self.current_trial['pending_outcome'] = outcome
            self.current_trial['pending_end_monotonic'] = end_monotonic
            self.current_trial['fd_auto_wait_deadline_monotonic'] = end_monotonic + self.fd_auto_wait_s
            if self.current_trial.get('fd_auto_m') is not None:
                self._finalize_pending_success()
            else:
                self.get_logger().info(
                    f'Trial {self.current_trial["trial_id"]}: Nav2 succeeded, waiting up to '
                    f'{self.fd_auto_wait_s:.1f}s for fd_auto_m on {self.fd_auto_topic}.'
                )
            return

        self._finalize_trial(outcome, end_monotonic)

    def timer_callback(self):
        if self.current_trial is None:
            return

        if self.current_trial.get('pending_outcome') == 'succeeded':
            deadline = self.current_trial.get('fd_auto_wait_deadline_monotonic')
            if deadline is not None and time.monotonic() >= deadline:
                self.get_logger().warn(
                    f'Trial {self.current_trial["trial_id"]}: fd_auto_m did not arrive within '
                    f'{self.fd_auto_wait_s:.1f}s, finalizing with the available value.'
                )
                self._finalize_pending_success()
            return

        elapsed = time.monotonic() - self.current_trial['start_monotonic']
        if elapsed < self.trial_timeout_s:
            return

        if self.current_trial['target_pixel_monotonic'] is None:
            outcome = 'no_detection_timeout'
        elif self.current_trial['goal_monotonic'] is None:
            outcome = 'goal_publish_timeout'
        else:
            outcome = 'nav_timeout'

        self._finalize_trial(outcome, time.monotonic())

    def _finalize_trial(self, outcome, end_monotonic):
        if self.current_trial is None:
            return

        start = self.current_trial['start_monotonic']
        target_pixel = self.current_trial['target_pixel_monotonic']
        goal_time = self.current_trial['goal_monotonic']

        row = {
            'trial_id': self.current_trial['trial_id'],
            'prompt': self.current_trial['prompt'],
            'prompt_time_iso': self.current_trial['prompt_time_iso'],
            'cv_runtime_s': self._format_duration(
                self.current_trial.get('cv_runtime_s')
                if self.current_trial.get('cv_runtime_s') is not None
                else (target_pixel - start if target_pixel is not None else None)
            ),
            'goal_publish_latency_s': self._format_duration(goal_time - start if goal_time is not None else None),
            'trial_duration_s': self._format_duration(end_monotonic - start),
            'total_time_to_object_s': self._format_duration(end_monotonic - start if outcome == 'succeeded' else None),
            'fd_auto_m': self._format_duration(self.current_trial.get('fd_auto_m')),
            'nav_outcome': outcome,
        }

        self._append_row(row)
        self.get_logger().info(
            f'Finalized trial {row["trial_id"]}: outcome={outcome}, '
            f'cv_runtime_s={row["cv_runtime_s"] or "NA"}, '
            f'total_time_to_object_s={row["total_time_to_object_s"] or "NA"}, '
            f'fd_auto_m={row["fd_auto_m"] or "NA"}.'
        )
        self.current_trial = None

    def _ensure_csv_header(self):
        if not self.output_csv.exists():
            with self.output_csv.open('w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=self.csv_fields)
                writer.writeheader()
            return

        try:
            with self.output_csv.open('r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                existing_fields = reader.fieldnames or []
                rows = list(reader)
        except Exception:
            return

        if existing_fields == self.csv_fields:
            return

        normalized_rows = []
        for row in rows:
            normalized_row = {field: row.get(field, '') for field in self.csv_fields}
            normalized_rows.append(normalized_row)

        with self.output_csv.open('w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.csv_fields)
            writer.writeheader()
            writer.writerows(normalized_rows)

    def _load_next_trial_id(self):
        if not self.output_csv.exists():
            return 1

        try:
            with self.output_csv.open('r', newline='') as csvfile:
                reader = csv.DictReader(csvfile)
                rows = list(reader)
        except Exception:
            return 1

        if not rows:
            return 1

        try:
            return max(int(row['trial_id']) for row in rows if row.get('trial_id')) + 1
        except Exception:
            return len(rows) + 1

    def _append_row(self, row):
        with self.output_csv.open('a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=self.csv_fields)
            writer.writerow(row)

    @staticmethod
    def _format_duration(value):
        if value is None:
            return ''
        return f'{value:.3f}'

    def _finalize_pending_success(self):
        if self.current_trial is None:
            return

        end_monotonic = self.current_trial.get('pending_end_monotonic')
        if end_monotonic is None:
            end_monotonic = time.monotonic()

        self.current_trial['pending_outcome'] = None
        self.current_trial['pending_end_monotonic'] = None
        self.current_trial['fd_auto_wait_deadline_monotonic'] = None
        self._finalize_trial('succeeded', end_monotonic)

    def _compute_fd_auto(self):
        if self.current_trial is None:
            return None

        target_point = self.current_trial.get('target_point')
        if target_point is None:
            self.get_logger().warn(
                f'Trial {self.current_trial["trial_id"]}: cannot compute fd_auto_m because no target point was stored.'
            )
            return None

        point = PointStamped()
        point.header.frame_id = target_point.header.frame_id
        point.header.stamp = self.get_clock().now().to_msg()
        point.point = target_point.point

        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_frame,
                point.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2),
            )
            point_robot = tf2_geometry_msgs.do_transform_point(point, transform)
        except Exception as exc:
            self.get_logger().warn(
                f'Trial {self.current_trial["trial_id"]}: failed to compute fd_auto_m from target point: {exc}'
            )
            return None

        px = float(point_robot.point.x)
        py = float(point_robot.point.y)
        if self.fd_distance_mode == 'forward':
            fd_value = max(0.0, px - self.front_robot_x)
        else:
            fd_value = math.hypot(px - self.front_robot_x, py - self.front_robot_y)

        self.get_logger().info(
            f'Trial {self.current_trial["trial_id"]}: computed fd_auto_m={fd_value:.3f} m '
            f'from target point in {self.robot_frame}: ({px:.3f}, {py:.3f}).'
        )
        return fd_value

    def _now_ros_ns(self):
        return self._stamp_to_ns(self.get_clock().now().to_msg())

    @staticmethod
    def _stamp_to_ns(stamp):
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentMetricsLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
