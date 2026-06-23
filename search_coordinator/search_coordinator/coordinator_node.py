#!/usr/bin/env python3
"""Search Coordinator executive (Phase 1.6 scaffold).

Builds and runs as an empty lifecycle-less node for now. The real content lands
in Phase 2: mission-state ownership, local frontier extraction with hysteresis
(score margin + min dwell), idempotent skill-action servers (ExploreFrontier /
GoToPose / ApproachDetection / GetObservation / Stop), map_odom_relay, and the
mission-epoch / UUID idempotency that invalidates in-flight goals on instruction
change. See ar_project/docs/ROADMAP.md Phase 2.
"""
import rclpy
from rclpy.node import Node

from fleet_comms.heartbeat import HeartbeatMonitor, HeartbeatPublisher


class SearchCoordinator(Node):
    # Fleet-wide heartbeat period (Phase 1.3). Producers and the monitor share it
    # so the deadline/liveliness QoS stays compatible.
    HEARTBEAT_PERIOD_S = 0.5

    def __init__(self) -> None:
        super().__init__('search_coordinator')

        # Phase 1.3: the executive emits its own heartbeat (symmetry/logging) and,
        # as the degradation supervisor, monitors every cross-link producer. In
        # Phase 5.1 the monitor's health drives VLM->FLAT degradation; for now it
        # only exposes/logs health via heartbeat_monitor.health_snapshot().
        self.heartbeat = HeartbeatPublisher(self, 'search_coordinator',
                                            period_s=self.HEARTBEAT_PERIOD_S)
        self.heartbeat_monitor = HeartbeatMonitor(self,
                                                  expected_period_s=self.HEARTBEAT_PERIOD_S)

        self.get_logger().info('search_coordinator scaffold up (Phase 1.6) with '
                               'Phase 1.3 heartbeat publisher + monitor.')


def main() -> None:
    rclpy.init()
    node = SearchCoordinator()
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
