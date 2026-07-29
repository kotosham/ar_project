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

from .mode_profiles import freshness
from .qos import liveliness_status

try:  # rclpy >= Jazzy
    from rclpy.event_handler import SubscriptionEventCallbacks
except ImportError:  # older distros
    from rclpy.qos_event import SubscriptionEventCallbacks


UINT32_HALF = 0x80000000
UINT32_MASK = 0xFFFFFFFF


def _epoch_is_stale(beat_epoch: int, current_epoch: int) -> bool:
    """uint32 wrap-safe: True if `beat_epoch` is strictly OLDER than the current
    mission epoch (a zombie heartbeat from a previous mission). Equal or newer
    (a monitor that lags its own authority — shouldn't happen) is NOT stale."""
    diff = (current_epoch - beat_epoch) & UINT32_MASK
    return 0 < diff < UINT32_HALF


def _cpu_load() -> float:
    """1-minute load average normalised by CPU count (0..~1+). 0.0 if unavailable."""
    try:
        return os.getloadavg()[0] / (os.cpu_count() or 1)
    except (OSError, AttributeError):
        return 0.0


def _is_unsupported_qos_event(exc: Exception) -> bool:
    name = type(exc).__name__
    text = str(exc).lower()
    return name == 'UnsupportedEventTypeError' or (
        'unsupported' in text and 'event' in text
    )


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
        # СОБСТВЕННАЯ callback-группа, а не группа ноды по умолчанию. Смысл
        # heartbeat — «процесс жив»; если тик стоит в общей очереди с рабочей
        # нагрузкой, он замолкает ровно тогда, когда узел занят, то есть врёт в
        # худший момент. Так это и ловилось: первый DETECT_ALL в детекторе
        # тянет текстовый энкодер YOLOE и держит executor ~16 с — heartbeat
        # пропадал, потребители объявляли детектор STALE, и преflight консоли
        # выбрасывал живой стек в «НЕ ГОТОВ». С отдельной группой
        # MultiThreadedExecutor тикает независимо от инференса. Узлам на
        # SingleThreadedExecutor это не мешает.
        try:
            from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
            self._cb_group = MutuallyExclusiveCallbackGroup()
            self._timer = node.create_timer(period_s, self._tick,
                                            callback_group=self._cb_group)
        except (ImportError, TypeError):
            self._cb_group = None
            self._timer = node.create_timer(period_s, self._tick)

    def set_status(self, status: int) -> None:
        self._status = int(status)

    def set_latency_ms(self, latency_ms: float) -> None:
        self._last_latency_ms = float(latency_ms)

    def set_mission_epoch(self, epoch: int) -> None:
        self._mission_epoch = int(epoch)

    def stop(self) -> None:
        """Stop beating (on shutdown) so peers observe liveliness loss promptly."""
        self._timer.cancel()

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
                 stale_factor: float = 2.5, mission_epoch: int = 0):
        self._node = node
        # Порог = максимум из «фактора x период» и порога из mode_profiles.
        # Иначе получаем ровно то расхождение, ради которого mode_profiles и
        # заведён: там записано 3.0 с (и прямо сказано, что это 2.5 x 0.5 с,
        # округлённое вверх, чтобы одиночная потеря пакета не красила
        # индикатор), а монитор жил по своим 1.25 с. В симуляции Gazebo,
        # RTAB-Map, Nav2, детектор и оркестратор делят один контейнер, таймеры
        # rclpy под нагрузкой плывут, и на 1.25 с STALE ловили ВСЕ продюсеры —
        # включая search_coordinator, жаловавшийся сам на себя.
        # Режим определяем по use_sim_time, а не по отдельному параметру: он
        # объявлен у КАЖДОГО узла и в симуляции всегда true, так что лишней
        # ручки, которую можно забыть прокинуть, не появляется.
        # max(), а не замена: порог может стать только мягче объявленного, но
        # никогда строже.
        self._stale_ns = max(int(stale_factor * expected_period_s * 1e9),
                             int(freshness(self._freshness_key(node)) * 1e9))
        self._nodes = {}
        self._mission_epoch = int(mission_epoch) & UINT32_MASK
        self._ignored_stale_epoch = 0
        qos = liveliness_status(expected_period_s)
        callbacks = SubscriptionEventCallbacks(
            deadline=self._on_deadline_missed,
            liveliness=self._on_liveliness_changed,
        )
        try:
            self._sub = node.create_subscription(
                Heartbeat, topic, self._on_msg, qos, event_callbacks=callbacks)
        except Exception as exc:
            if not _is_unsupported_qos_event(exc):
                raise
            node.get_logger().warn(
                'heartbeat QoS event callbacks are not supported by this RMW; '
                'falling back to timer-based stale detection only')
            self._sub = node.create_subscription(Heartbeat, topic, self._on_msg, qos)
        self._timer = node.create_timer(expected_period_s, self._check_stale)

    def set_mission_epoch(self, epoch: int) -> None:
        """Track the active mission epoch so zombie beats from a previous mission
        (stamped with an older epoch) are ignored (FMEA 2.5, must-fix #8). The
        executive calls this whenever it bumps the epoch."""
        self._mission_epoch = int(epoch) & UINT32_MASK

    def ignored_stale_epoch_count(self) -> int:
        return self._ignored_stale_epoch

    def _on_msg(self, msg: Heartbeat) -> None:
        if _epoch_is_stale(msg.mission_epoch, self._mission_epoch):
            # A producer still beating for a dead mission must NOT refresh its
            # health: it should read as STALE until it adopts the new epoch.
            self._ignored_stale_epoch += 1
            self._node.get_logger().debug(
                'heartbeat: ignoring stale-epoch beat from "%s" (beat=%d current=%d)'
                % (msg.node_name, msg.mission_epoch, self._mission_epoch))
            return
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

    @staticmethod
    def _freshness_key(node) -> str:
        """'heartbeat_sim' под Gazebo, иначе 'heartbeat'. Узел без объявленного
        use_sim_time (голый юнит-тест с заглушкой) читается как железо — то есть
        получает СТРОГИЙ порог: ошибиться в эту сторону безопаснее."""
        try:
            if bool(node.get_parameter('use_sim_time').value):
                return 'heartbeat_sim'
        except Exception:
            pass
        return 'heartbeat'

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
