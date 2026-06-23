"""Frontier extractor node (ROADMAP 2.3).

Subscribes the SLAM occupancy grid (`/map`, latched), extracts free<->unknown
frontiers, ranks them (size vs distance) with anti-oscillation hysteresis, and
publishes an `ar_project_msgs/FrontierArray` (consumed by the ExploreFrontier
skill, 2.4) plus RViz markers. All heavy logic lives in the ROS-free
`frontier_lib`; this node only does I/O, TF, timing and the source guard.

Fail-loud (must-fix #1): if the incoming grid carries no UNKNOWN cells (e.g. a
rolling local costmap without `track_unknown_space`, or a non-SLAM source) no
frontier can ever exist; the node logs an error (throttled) and publishes an
empty list with `source_has_unknown=False` instead of silently going quiet.
"""
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy

from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, \
    ExtrapolationException

from ar_project_msgs.msg import Frontier, FrontierArray

from search_coordinator.frontier_lib import (
    GridInfo,
    HysteresisParams,
    ScoreParams,
    extract_frontiers,
    should_switch,
)

NO_FRONTIER = -1


def _latched_qos():
    """QoS matching a SLAM /map: reliable + transient_local + keep_last(1)."""
    q = QoSProfile(depth=1)
    q.history = HistoryPolicy.KEEP_LAST
    q.reliability = ReliabilityPolicy.RELIABLE
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return q


class FrontierNode(Node):
    def __init__(self):
        super().__init__('frontier_extractor')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('frontiers_topic', '/frontiers')
        self.declare_parameter('markers_topic', '/frontiers/markers')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_link')
        self.declare_parameter('min_frontier_cells', 8)
        self.declare_parameter('score_size_weight', 1.0)
        self.declare_parameter('score_distance_weight', 2.0)
        self.declare_parameter('hysteresis_score_margin', 5.0)
        self.declare_parameter('hysteresis_min_dwell_s', 3.0)
        self.declare_parameter('id_quant_m', 1.0)
        self.declare_parameter('max_frontiers', 10)

        g = self.get_parameter
        self.map_frame = g('map_frame').value
        self.robot_base_frame = g('robot_base_frame').value
        self.min_cells = int(g('min_frontier_cells').value)
        self.score_params = ScoreParams(
            size_weight=float(g('score_size_weight').value),
            distance_weight=float(g('score_distance_weight').value))
        self.hyst = HysteresisParams(
            score_margin=float(g('hysteresis_score_margin').value),
            min_dwell_s=float(g('hysteresis_min_dwell_s').value))
        self.id_quant_m = float(g('id_quant_m').value)
        self.max_frontiers = int(g('max_frontiers').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._committed_id = NO_FRONTIER
        self._commit_time = None
        self._throttle = {}

        self.frontiers_pub = self.create_publisher(
            FrontierArray, g('frontiers_topic').value, _latched_qos())
        self.markers_pub = self.create_publisher(
            MarkerArray, g('markers_topic').value, 1)
        self.create_subscription(
            OccupancyGrid, g('map_topic').value, self._on_map, _latched_qos())

        self.get_logger().info(
            'frontier_extractor up: map_frame=%s base=%s min_cells=%d '
            'margin=%.1f dwell=%.1fs' % (self.map_frame, self.robot_base_frame,
                                         self.min_cells, self.hyst.score_margin,
                                         self.hyst.min_dwell_s))

    # -- helpers --------------------------------------------------------------

    def _log_throttle(self, tag, period_s, level, msg):
        now = self.get_clock().now()
        last = self._throttle.get(tag)
        if last is None or (now - last) >= Duration(seconds=period_s):
            self._throttle[tag] = now
            getattr(self.get_logger(), level)(msg)

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_base_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    # -- main callback --------------------------------------------------------

    def _on_map(self, msg: OccupancyGrid):
        info = GridInfo(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y)

        robot_xy = self._robot_xy()
        if robot_xy is None:
            self._log_throttle('no_tf', 5.0, 'warning',
                               'No TF %s->%s yet; cannot rank frontiers by distance.'
                               % (self.map_frame, self.robot_base_frame))
            return

        clusters, source_has_unknown = extract_frontiers(
            msg.data, info, robot_xy, self.min_cells, self.score_params,
            self.id_quant_m)

        if not source_has_unknown:
            self._log_throttle(
                'no_unknown', 5.0, 'error',
                'Map source on %s carries NO unknown cells -> frontier exploration '
                'cannot function. Check the source is the SLAM occupancy grid, not a '
                'rolling local costmap.' % self.get_parameter('map_topic').value)
            self._publish(msg.header.stamp, [], source_has_unknown=False)
            return

        clusters = clusters[:self.max_frontiers]
        self._update_commit(clusters)
        self._publish(msg.header.stamp, clusters, source_has_unknown=True)

    def _update_commit(self, clusters):
        now = self.get_clock().now()
        present = {c.fid: c for c in clusters}
        committed_present = self._committed_id in present
        committed_score = present[self._committed_id].score if committed_present else 0.0
        if not clusters:
            self._committed_id = NO_FRONTIER
            self._commit_time = None
            return
        best = clusters[0]
        dwell_s = 0.0 if self._commit_time is None else (now - self._commit_time).nanoseconds / 1e9
        if should_switch(committed_present, committed_score, best.fid, best.score,
                         self._committed_id, dwell_s, self.hyst):
            if best.fid != self._committed_id:
                self._committed_id = best.fid
                self._commit_time = now

    # -- output ---------------------------------------------------------------

    def _publish(self, stamp, clusters, source_has_unknown):
        arr = FrontierArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.map_frame
        arr.committed_id = self._committed_id
        arr.source_has_unknown = source_has_unknown
        for c in clusters:
            f = Frontier()
            f.id = int(c.fid)
            f.centroid = Point(x=float(c.centroid_world[0]),
                               y=float(c.centroid_world[1]), z=0.0)
            f.size = int(c.size)
            f.score = float(c.score)
            f.distance_m = float(c.distance_m)
            arr.frontiers.append(f)
        self.frontiers_pub.publish(arr)
        self._publish_markers(stamp, clusters)

    def _publish_markers(self, stamp, clusters):
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = stamp
        clear.ns = 'frontiers'
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)
        for i, c in enumerate(clusters):
            m = Marker()
            m.header.frame_id = self.map_frame
            m.header.stamp = stamp
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position = Point(x=float(c.centroid_world[0]),
                                    y=float(c.centroid_world[1]), z=0.0)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            committed = (c.fid == self._committed_id)
            m.color.r = 0.0 if committed else 1.0
            m.color.g = 1.0 if committed else 0.6
            m.color.b = 0.0
            m.color.a = 1.0
            ma.markers.append(m)
        self.markers_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierNode()
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
