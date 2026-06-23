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
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fleet_comms.heartbeat import HeartbeatMonitor, HeartbeatPublisher

from search_coordinator.mission_state import MissionState
from search_coordinator.skills import Nav2Driver, build_all_skills


class SearchCoordinator(Node):
    """Pi-side executive node. Phase 2.4: owns mission_state + the five skill
    action servers (ExploreFrontier/GoToPose/ApproachDetection/GetObservation/
    Stop), each on its own ReentrantCallbackGroup so the in-process FSM->skill
    loopback (added in Phase 2.2) cannot deadlock. The SeekObject entry server +
    FSM + full mission-epoch behavior land in Phase 2.2/2.5."""

    # Fleet-wide heartbeat period (Phase 1.3). Producers and the monitor share it
    # so the deadline/liveliness QoS stays compatible.
    HEARTBEAT_PERIOD_S = 0.5

    def __init__(self) -> None:
        super().__init__('search_coordinator')

        # Phase 1.3: own heartbeat + degradation monitor (logs health for now).
        self.heartbeat = HeartbeatPublisher(self, 'search_coordinator',
                                            period_s=self.HEARTBEAT_PERIOD_S)
        self.heartbeat_monitor = HeartbeatMonitor(self,
                                                  expected_period_s=self.HEARTBEAT_PERIOD_S)

        # Phase 2.4: mission epoch authority + skill servers.
        self.mission_state = MissionState()
        self.nav_driver = Nav2Driver(self)
        self.skills = build_all_skills(self, self.mission_state, self.nav_driver)

        self.get_logger().info(
            'search_coordinator up (Phase 2.4): skill servers %s; epoch=%d.'
            % (sorted(self.skills.keys()), self.mission_state.current_epoch()))


def main() -> None:
    rclpy.init()
    node = SearchCoordinator()
    # MultiThreadedExecutor so concurrent skill execute_callbacks + their Nav2
    # client polling do not starve each other (each is on a ReentrantCallbackGroup).
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
