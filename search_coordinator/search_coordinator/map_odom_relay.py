#!/usr/bin/env python3
"""map_odom_relay (ROADMAP Phase 2.6).

Applies the low-rate map->odom correction from edge SLAM
(ar_project_msgs/MapOdomCorrection) and rebroadcasts map->odom on /tf locally at
a high rate, so Nav2 always has a fresh transform even across Wi-Fi loss. It is
NOT a TF stream consumer — the edge sends one correction, the relay holds
last-good and republishes it.

Gating pipeline (each rejection holds last-good):
  1. stale-by-seq   — drop duplicates / out-of-order (uint32 wrap-safe)
  2. stale-by-stamp — drop older-than-last or older than max_correction_age_s
  3. covariance     — drop if x/y/yaw variance exceeds cov_reject_threshold
  4. fitness        — drop if SLAM fit < fitness_min
  5. jump           — drop a large pose jump UNLESS relocalized==true

Until the first accepted correction it broadcasts identity map->odom so the TF
tree is never broken. When this node runs, RTAB-Map must be launched with
publish_tf_map:=false (no duplicate broadcaster).
"""
import math

import rclpy
from ar_project_msgs.msg import MapOdomCorrection
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from fleet_comms.qos import correction_lowrate

_UINT32 = 1 << 32
_HALF_UINT32 = 1 << 31


def _seq_is_newer(seq, last_seq):
    """True if uint32 seq is strictly newer than last_seq (wrap-safe)."""
    diff = (seq - last_seq) % _UINT32
    return 0 < diff < _HALF_UINT32


def _quat_angle(a, b):
    """Smallest rotation angle (rad) between two quaternions."""
    dot = a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w
    return 2.0 * math.acos(min(1.0, abs(dot)))


class MapOdomRelay(Node):
    def __init__(self):
        super().__init__('map_odom_relay')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'odom')
        self.declare_parameter('rebroadcast_rate', 10.0)
        self.declare_parameter('transform_tolerance', 0.2)
        self.declare_parameter('max_correction_age_s', 1.0)
        self.declare_parameter('cov_reject_threshold', 0.5)
        self.declare_parameter('fitness_min', 0.3)
        self.declare_parameter('max_jump_m', 0.5)
        self.declare_parameter('max_jump_rad', 0.5)

        self.parent_frame = self.get_parameter('parent_frame').value
        self.child_frame = self.get_parameter('child_frame').value
        rate = float(self.get_parameter('rebroadcast_rate').value)
        self.transform_tolerance = float(self.get_parameter('transform_tolerance').value)
        self.max_correction_age_s = float(self.get_parameter('max_correction_age_s').value)
        self.cov_reject_threshold = float(self.get_parameter('cov_reject_threshold').value)
        self.fitness_min = float(self.get_parameter('fitness_min').value)
        self.max_jump_m = float(self.get_parameter('max_jump_m').value)
        self.max_jump_rad = float(self.get_parameter('max_jump_rad').value)

        # last-good map->odom transform (geometry_msgs/Transform), None until first accept
        self._last_good = None
        self._last_seq = None
        self._last_accept_stamp_ns = None

        self._broadcaster = TransformBroadcaster(self)
        self._sub = self.create_subscription(
            MapOdomCorrection, '/map_odom_correction', self._on_correction, correction_lowrate())
        self._timer = self.create_timer(1.0 / rate, self._rebroadcast)

        self.get_logger().info(
            f'map_odom_relay up: {self.parent_frame}->{self.child_frame} @ {rate:.1f} Hz '
            f'(identity until first valid correction). Ensure RTAB-Map publish_tf_map:=false.')

    def _on_correction(self, msg: MapOdomCorrection) -> None:
        # 1. stale-by-seq
        if self._last_seq is not None and not _seq_is_newer(msg.seq, self._last_seq):
            self.get_logger().debug(f'drop correction seq={msg.seq} (<= last {self._last_seq})')
            return

        # 2. stale-by-stamp
        stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)
        if self._last_accept_stamp_ns is not None and stamp_ns < self._last_accept_stamp_ns:
            self.get_logger().debug('drop correction: stamp older than last accepted')
            return
        age_s = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age_s > self.max_correction_age_s:
            self.get_logger().warn(f'drop correction: aged {age_s:.2f}s > {self.max_correction_age_s}s')
            return

        # 3. covariance gate (x, y, yaw variances on the 6x6 diagonal)
        cov = msg.covariance
        if max(cov[0], cov[7], cov[35]) > self.cov_reject_threshold:
            self.get_logger().warn('drop correction: covariance over threshold')
            return

        # 4. fitness gate
        if msg.fitness < self.fitness_min:
            self.get_logger().warn(f'drop correction: fitness {msg.fitness:.2f} < {self.fitness_min}')
            return

        # 5. jump gate (accept large jumps only when relocalized)
        incoming = msg.map_to_odom.transform
        if self._last_good is not None and not msg.relocalized:
            dt = math.dist(
                (incoming.translation.x, incoming.translation.y, incoming.translation.z),
                (self._last_good.translation.x, self._last_good.translation.y, self._last_good.translation.z))
            da = _quat_angle(incoming.rotation, self._last_good.rotation)
            if dt > self.max_jump_m or da > self.max_jump_rad:
                self.get_logger().warn(
                    f'drop correction: jump dt={dt:.2f}m da={da:.2f}rad without relocalized flag')
                return
        if msg.relocalized and self._last_good is not None:
            self.get_logger().info('accepting relocalization jump (relocalized=true)')

        # accept
        self._last_good = incoming
        self._last_seq = msg.seq
        self._last_accept_stamp_ns = stamp_ns

    def _rebroadcast(self) -> None:
        t = TransformStamped()
        # Stamp slightly ahead so the transform stays valid within transform_tolerance.
        t.header.stamp = (self.get_clock().now() + Duration(seconds=self.transform_tolerance)).to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        if self._last_good is not None:
            t.transform = self._last_good
        else:
            t.transform.rotation.w = 1.0  # identity until first correction
        self._broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapOdomRelay()
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
