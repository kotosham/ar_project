#!/usr/bin/env python3
"""Publish low-rate RTAB-Map map->odom corrections for the Pi relay.

RTAB-Map already computes the optimized map->odom transform and exposes it in
rtabmap_msgs/MapGraph.map_to_odom. In the two-host hardware architecture the
edge must not stream map->odom as TF directly over Wi-Fi. Instead, this node
caches the latest MapGraph transform and republishes it as a low-rate
ar_project_msgs/MapOdomCorrection heartbeat; the Pi side map_odom_relay gates it
and rebroadcasts a fresh local TF.
"""

import math
import time

import rclpy
from ar_project_msgs.msg import MapOdomCorrection
from fleet_comms.qos import correction_lowrate
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import Info, MapGraph


def _rtabmap_map_graph_qos():
    """Compatible with RTAB-Map's latched /mapGraph publisher.

    Do not request a deadline here: RTAB-Map does not promise one, and a stricter
    requested deadline can make DDS refuse the connection. RTAB-Map publishes
    /mapGraph as TRANSIENT_LOCAL, so request the same durability to receive the
    last graph even when this node starts after RTAB-Map.
    """
    q = QoSProfile(depth=1)
    q.reliability = ReliabilityPolicy.RELIABLE
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return q


def _rtabmap_info_qos():
    """Compatible with RTAB-Map's ordinary volatile telemetry topics."""
    q = QoSProfile(depth=10)
    q.reliability = ReliabilityPolicy.RELIABLE
    q.durability = DurabilityPolicy.VOLATILE
    return q


def _yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _angle_abs_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _copy_transform(src, dst):
    dst.translation.x = src.translation.x
    dst.translation.y = src.translation.y
    dst.translation.z = src.translation.z
    dst.rotation.x = src.rotation.x
    dst.rotation.y = src.rotation.y
    dst.rotation.z = src.rotation.z
    dst.rotation.w = src.rotation.w


class RtabmapMapOdomCorrectionPublisher(Node):
    def __init__(self):
        super().__init__('rtabmap_map_odom_correction_publisher')

        self.declare_parameter('map_graph_topic', '/mapGraph')
        self.declare_parameter('info_topic', '/rtabmap/info')
        self.declare_parameter('correction_topic', '/map_odom_correction')
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'odom')
        self.declare_parameter('covariance_x', 0.05)
        self.declare_parameter('covariance_y', 0.05)
        self.declare_parameter('covariance_yaw', 0.05)
        self.declare_parameter('fitness_default', 1.0)
        self.declare_parameter('use_info_relocalized', True)
        self.declare_parameter('info_relocalization_window_s', 2.0)
        self.declare_parameter('mark_jump_as_relocalized', True)
        self.declare_parameter('relocalized_jump_m', 0.75)
        self.declare_parameter('relocalized_jump_rad', 0.75)
        self.declare_parameter('publish_rate_hz', 2.0)

        g = lambda name: self.get_parameter(name).value
        self.map_graph_topic = str(g('map_graph_topic'))
        self.info_topic = str(g('info_topic'))
        self.correction_topic = str(g('correction_topic'))
        self.parent_frame = str(g('parent_frame'))
        self.child_frame = str(g('child_frame'))
        self.covariance_x = float(g('covariance_x'))
        self.covariance_y = float(g('covariance_y'))
        self.covariance_yaw = float(g('covariance_yaw'))
        self.fitness_default = float(g('fitness_default'))
        self.use_info_relocalized = bool(g('use_info_relocalized'))
        self.info_relocalization_window_s = float(g('info_relocalization_window_s'))
        self.mark_jump_as_relocalized = bool(g('mark_jump_as_relocalized'))
        self.relocalized_jump_m = float(g('relocalized_jump_m'))
        self.relocalized_jump_rad = float(g('relocalized_jump_rad'))
        self.publish_rate_hz = float(g('publish_rate_hz'))

        self._seq = 0
        self._last_transform = None
        self._latest_transform = None
        self._last_info_relocalization_mono = 0.0
        self._pending_relocalized = False
        self._published_once = False

        self._pub = self.create_publisher(
            MapOdomCorrection, self.correction_topic, correction_lowrate())
        self.create_subscription(MapGraph, self.map_graph_topic,
                                 self._on_map_graph, _rtabmap_map_graph_qos())
        if self.use_info_relocalized:
            self.create_subscription(Info, self.info_topic, self._on_info,
                                     _rtabmap_info_qos())
        self.create_timer(
            1.0 / max(0.1, self.publish_rate_hz), self._publish_latest)

        self.get_logger().info(
            'RTAB-Map correction publisher ready: %s.map_to_odom -> %s '
            '(%s->%s, %.1f Hz, info_reloc=%s, jump_reloc=%s)'
            % (self.map_graph_topic, self.correction_topic,
               self.parent_frame, self.child_frame, self.publish_rate_hz,
               self.use_info_relocalized, self.mark_jump_as_relocalized))

    def _on_info(self, msg):
        # RTAB-Map reports graph corrections through loop/proximity closures.
        if int(msg.loop_closure_id) > 0 or int(msg.proximity_detection_id) > 0:
            self._last_info_relocalization_mono = time.monotonic()

    def _info_relocalized_is_recent(self):
        if not self.use_info_relocalized or self._last_info_relocalization_mono <= 0.0:
            return False
        age = time.monotonic() - self._last_info_relocalization_mono
        return age <= self.info_relocalization_window_s

    def _jump_looks_like_relocalization(self, transform):
        if not self.mark_jump_as_relocalized or self._last_transform is None:
            return False
        a = transform.translation
        b = self._last_transform.translation
        dist = math.dist((a.x, a.y, a.z), (b.x, b.y, b.z))
        dyaw = _angle_abs_diff(
            _yaw_from_quat(transform.rotation),
            _yaw_from_quat(self._last_transform.rotation),
        )
        return dist > self.relocalized_jump_m or dyaw > self.relocalized_jump_rad

    def _on_map_graph(self, msg):
        relocalized = (
            self._info_relocalized_is_recent()
            or self._jump_looks_like_relocalization(msg.map_to_odom)
        )
        self._pending_relocalized = self._pending_relocalized or relocalized
        _copy_transform(msg.map_to_odom, self._ensure_latest_transform())
        _copy_transform(msg.map_to_odom, self._ensure_last_transform())
        self._publish_latest()

    def _publish_latest(self):
        if self._latest_transform is None:
            return

        # Fresh stamps keep the Pi relay and dashboard alive even when RTAB-Map's
        # latched /mapGraph does not change for a long stationary interval.
        stamp = self.get_clock().now().to_msg()
        out = MapOdomCorrection()
        out.header.stamp = stamp
        out.header.frame_id = self.parent_frame
        out.map_to_odom = TransformStamped()
        out.map_to_odom.header.stamp = stamp
        out.map_to_odom.header.frame_id = self.parent_frame
        out.map_to_odom.child_frame_id = self.child_frame
        _copy_transform(self._latest_transform, out.map_to_odom.transform)

        cov = [0.0] * 36
        cov[0] = self.covariance_x
        cov[7] = self.covariance_y
        cov[35] = self.covariance_yaw
        out.covariance = cov
        out.fitness = self.fitness_default
        self._seq = (self._seq + 1) & 0xFFFFFFFF
        out.seq = self._seq
        out.relocalized = self._pending_relocalized
        self._pending_relocalized = False

        self._pub.publish(out)

        if not self._published_once:
            self._published_once = True
            self.get_logger().info(
                'Published first map->odom correction seq=%d stamp=%d.%09d'
                % (out.seq, out.header.stamp.sec, out.header.stamp.nanosec))
        elif out.relocalized:
            self.get_logger().info('Published relocalized map->odom correction seq=%d' % out.seq)

    def _ensure_latest_transform(self):
        if self._latest_transform is None:
            from geometry_msgs.msg import Transform
            self._latest_transform = Transform()
        return self._latest_transform

    def _ensure_last_transform(self):
        if self._last_transform is None:
            from geometry_msgs.msg import Transform
            self._last_transform = Transform()
        return self._last_transform


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapMapOdomCorrectionPublisher()
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
