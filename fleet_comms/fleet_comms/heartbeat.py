"""Heartbeat producer + monitor utilities (ROADMAP Phase 1.3).

Every cross-link producer (edge SLAM / detector / planner_orchestrator, and the
Pi executive for symmetry) publishes ar_project_msgs/Heartbeat on /heartbeat with
the liveliness_status QoS. The executive's degradation supervisor subscribes via
HeartbeatMonitor, which tracks per-node health from the message status, the QoS
deadline-missed / liveliness-lost events, and a stale-timeout fallback.

Phase 1.3 scope: these helpers only emit and expose/log health. Wiring health
loss into the actual VLM->FLAT degradation FSM + circuit-breaker is Phase 4.4/5.1.
"""
import os

from ar_project_msgs.msg import Heartbeat

from .qos import liveliness_status

try:  # rclpy >= Jazzy
    from rclpy.event_handler import SubscriptionEventCallbacks
except ImportError:  # older distros
    from rclpy.qos_event import SubscriptionEventCallbacks


def _cpu_load() -> float:
    """1-minute load average normalised by CPU count (0..~1+). 0.0 if unavailable."""
    try:
        return os.getloadavg()[0] / (os.cpu_count() or 1)
    except (OSError, AttributeError):
        return 0.0


class HeartbeatPublisher:
    """Periodically publishes a Heartbeat for one producer node.

    Usage:
        self._hb = HeartbeatPublisher(self, 'planner_orchestrator', period_s=0.5)
        ...
        self._hb.set_status(Heartbeat.DEGRADED)      # on high load / backed-up queue
        self._hb.set_latency_ms(measured_ms)         # feeds p99 timeout (Phase 4.4)
        self._hb.set_mission_epoch(epoch)            # so stale-epoch beats are ignored
    """

    def __init__(self, node, node_name: str, period_s: float = 1.0, topic: str = '/heartbeat'):
        self._node = node
        self._node_name = node_name
        self._status = Heartbeat.OK
        self._last_latency_ms = 0.0
        self._mission_epoch = 0
        self._pub = node.create_publisher(Heartbeat, topic, liveliness_status(period_s))
        self._timer = node.create_timer(period_s, self._tick)

    def set_status(self, status: int) -> None:
        self._status = int(status)

    def set_latency_ms(self, latency_ms: float) -> None:
        self._last_latency_ms = float(latency_ms)

    def set_mission_epoch(self, epoch: int) -> None:
        self._mission_epoch = int(epoch)

    def _tick(self) -> None:
        msg = Heartbeat()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.node_name = self._node_name
        msg.status = self._status
        msg.cpu_load = _cpu_load()
        msg.last_latency_ms = self._last_latency_ms
        msg.mission_epoch = self._mission_epoch
        self._pub.publish(msg)
        # Writing a sample asserts MANUAL_BY_TOPIC liveliness; call explicitly too
        # where supported, so liveliness holds even if a tick publishes nothing new.
        try:
            self._pub.assert_liveliness()
        except (AttributeError, RuntimeError):
            pass


class HeartbeatMonitor:
    """Subscribes /heartbeat and exposes per-producer health.

    Health per node_name is the worst of: the reported status (OK/DEGRADED/DOWN),
    a stale-timeout (no beat for > stale_factor * period -> STALE), and the
    topic-level QoS deadline/liveliness events (logged as a fast global signal;
    DDS cannot attribute them to our node_name field).
    """

    OK = 'OK'
    DEGRADED = 'DEGRADED'
    DOWN = 'DOWN'
    STALE = 'STALE'

    class _Health:
        __slots__ = ('status', 'last_seen_ns')

        def __init__(self, status: int, last_seen_ns: int):
            self.status = status
            self.last_seen_ns = last_seen_ns

    def __init__(self, node, expected_period_s: float = 1.0, topic: str = '/heartbeat',
                 stale_factor: float = 2.5):
        self._node = node
        self._stale_ns = int(stale_factor * expected_period_s * 1e9)
        self._nodes = {}
        callbacks = SubscriptionEventCallbacks(
            deadline=self._on_deadline_missed,
            liveliness=self._on_liveliness_changed,
        )
        self._sub = node.create_subscription(
            Heartbeat, topic, self._on_msg, liveliness_status(expected_period_s),
            event_callbacks=callbacks)
        self._timer = node.create_timer(expected_period_s, self._check_stale)

    def _on_msg(self, msg: Heartbeat) -> None:
        self._nodes[msg.node_name] = self._Health(
            status=msg.status,
            last_seen_ns=self._node.get_clock().now().nanoseconds,
        )

    def _on_deadline_missed(self, info) -> None:
        self._node.get_logger().warn(
            f'/heartbeat deadline missed (total={info.total_count}) — a producer is silent')

    def _on_liveliness_changed(self, info) -> None:
        if getattr(info, 'alive_count', 1) == 0:
            self._node.get_logger().warn('/heartbeat liveliness LOST — all producers silent')

    def get_health(self, node_name: str) -> str:
        h = self._nodes.get(node_name)
        if h is None:
            return self.DOWN  # never seen
        now = self._node.get_clock().now().nanoseconds
        if now - h.last_seen_ns > self._stale_ns:
            return self.STALE
        if h.status == Heartbeat.DOWN:
            return self.DOWN
        if h.status == Heartbeat.DEGRADED:
            return self.DEGRADED
        return self.OK

    def health_snapshot(self) -> dict:
        """{node_name: health_str} for every producer seen so far."""
        return {name: self.get_health(name) for name in self._nodes}

    def is_healthy(self, node_name: str) -> bool:
        return self.get_health(node_name) == self.OK

    def _check_stale(self) -> None:
        for name in list(self._nodes):
            health = self.get_health(name)
            if health in (self.STALE, self.DOWN):
                self._node.get_logger().warn(f'/heartbeat: producer "{name}" is {health}')
