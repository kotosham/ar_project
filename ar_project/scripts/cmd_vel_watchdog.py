#!/usr/bin/env python3
"""cmd_vel watchdog / deadman (ROADMAP Phase 0.5).

Sits on the navigation velocity path *before* twist_mux. While fresh commands
arrive on ``input_topic`` it republishes them verbatim on ``output_topic``
(pass-through). If no fresh command is seen for longer than ``timeout`` seconds
it enters a HOLD state: it keeps publishing a zero command at ``publish_rate``
(an explicit deadman, so the robot stops even if a downstream timeout misbehaves)
and raises a status flag the executive/diagnostics can observe.

This is a soft, network-side safety net. It does NOT replace the two hard stops
(the diff_drive_controller ``cmd_vel_timeout`` and the CiA-402 quick-stop in the
hardware interface) — it is defence in depth and the single, tunable place where
"navigation command went stale" is turned into a deterministic zero + HOLD.

By default HOLD auto-clears as soon as a fresh command arrives. Set
``latch_hold:=true`` to require an explicit ``~/clear_hold`` (std_srvs/Trigger)
call before pass-through resumes.
"""

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


class CmdVelWatchdog(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_watchdog')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel_safe')
        self.declare_parameter('status_topic', '~/hold')
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('use_stamped', False)
        self.declare_parameter('latch_hold', False)

        self._input_topic = self.get_parameter('input_topic').value
        self._output_topic = self.get_parameter('output_topic').value
        status_topic = self.get_parameter('status_topic').value
        self._timeout = float(self.get_parameter('timeout').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self._use_stamped = bool(self.get_parameter('use_stamped').value)
        self._latch_hold = bool(self.get_parameter('latch_hold').value)

        self._msg_type = TwistStamped if self._use_stamped else Twist

        self._pub = self.create_publisher(self._msg_type, self._output_topic, 10)
        self._status_pub = self.create_publisher(Bool, status_topic, 10)
        self._sub = self.create_subscription(
            self._msg_type, self._input_topic, self._on_cmd, 10
        )
        self._clear_srv = self.create_service(
            Trigger, '~/clear_hold', self._on_clear_hold
        )

        # Start in HOLD: until we have seen a fresh command we must not assume
        # the upstream planner is alive.
        self._last_cmd_time = None
        self._in_hold = True

        period = 1.0 / publish_rate if publish_rate > 0.0 else 0.05
        self._timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"cmd_vel watchdog: '{self._input_topic}' -> '{self._output_topic}', "
            f"timeout={self._timeout:.2f}s, rate={publish_rate:.1f}Hz, "
            f"stamped={self._use_stamped}, latch_hold={self._latch_hold}."
        )

    def _on_cmd(self, msg) -> None:
        # While HOLD is latched we drop upstream commands and keep holding until
        # an operator explicitly clears it.
        if self._in_hold and self._latch_hold:
            return

        self._last_cmd_time = self.get_clock().now()
        if self._in_hold:
            self._in_hold = False
            self.get_logger().info('cmd_vel watchdog: fresh command, leaving HOLD.')
            self._publish_status()
        self._pub.publish(msg)

    def _on_timer(self) -> None:
        now = self.get_clock().now()
        stale = (
            self._last_cmd_time is None
            or (now - self._last_cmd_time).nanoseconds * 1e-9 > self._timeout
        )

        if stale and not self._in_hold:
            self._in_hold = True
            self.get_logger().warn(
                'cmd_vel watchdog: no fresh command within '
                f'{self._timeout:.2f}s, entering HOLD (publishing zero).'
            )
            self._publish_status()

        if self._in_hold:
            # Deterministic deadman: keep emitting an explicit zero command.
            self._pub.publish(self._zero_cmd())
            self._publish_status()

    def _on_clear_hold(self, request, response):
        del request
        self._in_hold = True if self._last_cmd_time is None else False
        if self._last_cmd_time is None:
            # No command ever received; cannot leave HOLD yet.
            response.success = False
            response.message = 'No command received yet; HOLD retained.'
        else:
            self._last_cmd_time = self.get_clock().now()
            self._in_hold = False
            response.success = True
            response.message = 'HOLD cleared.'
        self._publish_status()
        return response

    def _zero_cmd(self):
        msg = self._msg_type()
        if self._use_stamped:
            msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def _publish_status(self) -> None:
        status = Bool()
        status.data = self._in_hold
        self._status_pub.publish(status)


def main() -> None:
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
