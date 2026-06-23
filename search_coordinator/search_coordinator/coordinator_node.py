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


class SearchCoordinator(Node):
    def __init__(self) -> None:
        super().__init__('search_coordinator')
        self.get_logger().info('search_coordinator scaffold up (Phase 1.6).')


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
