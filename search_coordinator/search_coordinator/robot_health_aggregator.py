"""Pi-side per-component health aggregator -> /robot_health (1 Hz).

Answers "what is each robot element receiving and executing, and what failed"
without adding any high-rate Wi-Fi traffic (DATA_CONTRACTS L8: one small
aggregated DiagnosticArray @1Hz is the only stream that crosses the link).

Health sources, per component:
  * /heartbeat roster   -- components that actively beat (executive, VLM
                           orchestrator, detector). Driven over an EXPECTED
                           roster: a component that never beat still gets a row
                           (DOWN) instead of being silently omitted.
  * topic liveness      -- reactive-loop components that cannot host a
                           heartbeat (RealSense, EKF, /scan, ros2_control,
                           SLAM correction, detection stream). Probes are
                           BEST_EFFORT depth-1 subscriptions to SMALL topics
                           (camera_info, odometry, scan...), never images.
  * node presence       -- infrastructure whose liveness IS its process being
                           in the ROS graph (Nav2 servers, twist_mux,
                           collision monitor, RTAB-Map on the edge).

DiagnosticStatus levels: OK=0, WARN=1, ERROR=2, STALE=3. `advisory` probes
(sporadic-by-design streams like /target_pixel or /cmd_vel_out) never rise
above WARN -- staleness there is normal outside their active phase.
"""
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, JointState, LaserScan

from ar_project_msgs.msg import Heartbeat, MapOdomCorrection
from fleet_comms.heartbeat import HeartbeatMonitor


def _lossy_qos():
    """Compatible with any publisher (BEST_EFFORT sub accepts RELIABLE offers)."""
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


class _TopicProbe:
    """Liveness probe: last-seen age + observed rate for one small topic."""

    def __init__(self, node, name, msg_type, topic, stale_s,
                 qos=None, advisory=False, detail=''):
        self.name = name
        self.topic = topic
        self.stale_s = float(stale_s)
        self.advisory = advisory        # sporadic stream: staleness is WARN max
        self.detail = detail
        self._last_rx = 0.0             # monotonic; 0 = never seen
        self._stamps = deque(maxlen=20)
        node.create_subscription(msg_type, topic, self._on_msg,
                                 qos if qos is not None else _lossy_qos())

    def _on_msg(self, _msg):
        now = time.monotonic()
        self._last_rx = now
        self._stamps.append(now)

    def rate_hz(self):
        if len(self._stamps) < 2:
            return 0.0
        span = self._stamps[-1] - self._stamps[0]
        return (len(self._stamps) - 1) / span if span > 0 else 0.0

    def status(self):
        """-> (level, message, values)."""
        values = [KeyValue(key='topic', value=self.topic)]
        if self._last_rx == 0.0:
            level = (DiagnosticStatus.WARN if self.advisory
                     else DiagnosticStatus.ERROR)
            msg = 'no messages seen yet'
            if self.advisory and self.detail:
                msg += ' (%s)' % self.detail
            return level, msg, values
        age = time.monotonic() - self._last_rx
        values.append(KeyValue(key='age_s', value='%.1f' % age))
        values.append(KeyValue(key='rate_hz', value='%.1f' % self.rate_hz()))
        if age > self.stale_s:
            level = (DiagnosticStatus.WARN if self.advisory
                     else DiagnosticStatus.STALE)
            msg = 'stale: last message %.1fs ago (limit %.1fs)' % (age, self.stale_s)
            if self.advisory and self.detail:
                msg += ' (%s)' % self.detail
            return level, msg, values
        return DiagnosticStatus.OK, 'receiving @ %.1f Hz' % self.rate_hz(), values


class RobotHealthAggregator(Node):
    HEARTBEAT_PERIOD_S = 0.5
    # Components expected to beat on /heartbeat (roster-driven so a producer
    # that NEVER started still shows up as DOWN instead of being omitted).
    HEARTBEAT_ROSTER = {
        'search_coordinator': 'executive FSM + skill servers (Pi)',
        'planner_orchestrator': 'VLM planner (edge)',
        'detector': 'YOLOE detector (edge)',
    }
    # Node-presence roster: {component: ([node names], detail)}.
    NODE_ROSTER = {
        'nav2': (['controller_server', 'planner_server', 'bt_navigator'],
                 'Nav2 navigation servers (Pi)'),
        'twist_mux': (['twist_mux'], 'command mux (Pi)'),
        'collision_monitor': (['collision_monitor'], 'reactive stop layer (Pi)'),
        'cmd_vel_watchdog': (['cmd_vel_watchdog'], 'cmd_vel watchdog (Pi)'),
        'slam_rtabmap': (['rtabmap'], 'RTAB-Map SLAM process (edge, via graph)'),
    }

    def __init__(self):
        super().__init__('robot_health_aggregator')
        self.declare_parameter('publish_rate_hz', 1.0)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._hb_monitor = HeartbeatMonitor(
            self, expected_period_s=self.HEARTBEAT_PERIOD_S)

        self._probes = [
            _TopicProbe(self, 'realsense', CameraInfo,
                        '/camera/camera/color/camera_info', stale_s=2.0,
                        detail='RGB-D camera stream'),
            _TopicProbe(self, 'ekf_odometry', Odometry,
                        '/odometry/filtered', stale_s=1.5,
                        detail='EKF fused odometry'),
            _TopicProbe(self, 'scan', LaserScan, '/scan', stale_s=2.0,
                        detail='depth->laserscan obstacle source'),
            _TopicProbe(self, 'control_epos4', JointState,
                        '/joint_states', stale_s=2.0,
                        detail='ros2_control joint states (EPOS4/CAN alive)'),
            _TopicProbe(self, 'wheel_odometry', Odometry,
                        '/diff_cont/odom', stale_s=2.0,
                        detail='diff-drive wheel odometry'),
            # NOTE: deliberately NO probe on /map — Nav2 + frontier_extractor
            # already subscribe it on the Pi, and one more subscription would be
            # another cross-link copy of the grid for monitoring only. SLAM
            # health = correction freshness below + rtabmap node presence.
            _TopicProbe(self, 'slam_correction', MapOdomCorrection,
                        '/map_odom_correction', stale_s=5.0, advisory=True,
                        detail='map->odom correction from edge SLAM; staleness '
                               'means the LINK or SLAM output stalled, relay '
                               'holds last-good'),
            _TopicProbe(self, 'detection_stream', PointStamped,
                        '/target_pixel', stale_s=2.0, advisory=True,
                        detail='sporadic: only flows while a target is visible'),
            _TopicProbe(self, 'cmd_vel', Twist, '/cmd_vel_out', stale_s=2.0,
                        advisory=True,
                        detail='muxed drive command: only flows while driving'),
        ]

        self._pub = self.create_publisher(DiagnosticArray, '/robot_health', 5)
        period = 1.0 / max(0.1, rate)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            'robot_health_aggregator up: %d topic probes + %d heartbeat rows + '
            '%d node-presence rows -> /robot_health @ %.1f Hz'
            % (len(self._probes), len(self.HEARTBEAT_ROSTER),
               len(self.NODE_ROSTER), rate))

    # -- assembly ------------------------------------------------------------

    _HB_LEVEL = {
        HeartbeatMonitor.OK: DiagnosticStatus.OK,
        HeartbeatMonitor.DEGRADED: DiagnosticStatus.WARN,
        HeartbeatMonitor.STALE: DiagnosticStatus.STALE,
        HeartbeatMonitor.DOWN: DiagnosticStatus.ERROR,
    }

    def _heartbeat_rows(self):
        rows = []
        for name, detail in self.HEARTBEAT_ROSTER.items():
            health = self._hb_monitor.get_health(name)
            level = self._HB_LEVEL.get(health, DiagnosticStatus.ERROR)
            msg = ('heartbeat %s' % health) + (' — never seen'
                                              if health == HeartbeatMonitor.DOWN
                                              and name not in self._hb_monitor._nodes
                                              else '')
            rows.append((name, level, msg,
                         [KeyValue(key='detail', value=detail),
                          KeyValue(key='source', value='/heartbeat')]))
        # Extra producers not in the roster still get a row (forward-compatible).
        for name, health in self._hb_monitor.health_snapshot().items():
            if name in self.HEARTBEAT_ROSTER:
                continue
            rows.append((name, self._HB_LEVEL.get(health, DiagnosticStatus.ERROR),
                         'heartbeat %s' % health,
                         [KeyValue(key='source', value='/heartbeat')]))
        return rows

    def _node_presence_rows(self):
        try:
            alive = set(self.get_node_names())
        except Exception:
            alive = set()
        rows = []
        for comp, (expected, detail) in self.NODE_ROSTER.items():
            missing = [n for n in expected if n not in alive]
            if not missing:
                level, msg = DiagnosticStatus.OK, 'nodes present: %s' % ', '.join(expected)
            elif len(missing) < len(expected):
                level, msg = DiagnosticStatus.WARN, 'missing nodes: %s' % ', '.join(missing)
            else:
                level, msg = DiagnosticStatus.ERROR, 'no nodes in graph: %s' % ', '.join(expected)
            rows.append((comp, level, msg,
                         [KeyValue(key='detail', value=detail),
                          KeyValue(key='source', value='node graph')]))
        return rows

    def _tick(self):
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for probe in self._probes:
            level, msg, values = probe.status()
            values.append(KeyValue(key='detail', value=probe.detail))
            arr.status.append(DiagnosticStatus(
                level=level, name=probe.name, message=msg,
                hardware_id='', values=values))
        for name, level, msg, values in self._heartbeat_rows():
            arr.status.append(DiagnosticStatus(
                level=level, name=name, message=msg, hardware_id='', values=values))
        for name, level, msg, values in self._node_presence_rows():
            arr.status.append(DiagnosticStatus(
                level=level, name=name, message=msg, hardware_id='', values=values))
        self._pub.publish(arr)


def main():
    rclpy.init()
    node = RobotHealthAggregator()
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
