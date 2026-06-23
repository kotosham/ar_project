"""Prompt bridge (ROADMAP 2.x, must-fix #2/#3).

The FLAT detect path only produces `/target_pixel` after a label reaches the
tracker on `/target_prompt`. `reliable_prompt_sender.py` (and its latch-soup
handshake) is deleted in Phase 2.9, so the executive itself owns this: when a
`SeekObject` goal arrives, the FSM calls `PromptBridge.publish(instruction)` to
tell the tracker what to look for.

QoS: `control_cmd_latched` (RELIABLE + TRANSIENT_LOCAL) — the principled
replacement for the prompt-sender retry loop. The latest prompt is latched so a
tracker that (re)starts after a Wi-Fi drop recovers the current target. (For the
late-join replay to reach the tracker its `/target_prompt` subscription must also
be TRANSIENT_LOCAL — tracked in Phase 2.9; live delivery works regardless.)
"""
from std_msgs.msg import String

from fleet_comms.qos import control_cmd_latched

DEFAULT_PROMPT_TOPIC = '/target_prompt'


class PromptBridge:
    """Publishes the active mission instruction to the tracker's `/target_prompt`.

    Owned by the executive node; not a node itself. Idempotent: re-publishing the
    same label is harmless (the tracker just re-confirms its target)."""

    def __init__(self, node, topic: str = DEFAULT_PROMPT_TOPIC):
        self._node = node
        self._topic = topic
        self._last = None
        self._pub = node.create_publisher(String, topic, control_cmd_latched())

    @property
    def last_prompt(self):
        return self._last

    def publish(self, instruction: str) -> None:
        """Latch `instruction` on `/target_prompt`. Empty/whitespace is ignored."""
        if instruction is None:
            return
        label = instruction.strip()
        if not label:
            return
        self._pub.publish(String(data=label))
        self._last = label
        self._node.get_logger().info("prompt_bridge -> '%s' on %s" % (label, self._topic))
