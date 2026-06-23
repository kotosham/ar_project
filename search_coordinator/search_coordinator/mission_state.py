"""Mission epoch authority + per-server idempotency (ROADMAP 2.4/2.5, FMEA).

Built in T2.4 (not T2.5) on purpose: the skill servers need real epoch-gating and
request-id dedup to be testable, and the SeekObject FSM (T2.2/2.5) needs the same
object to drive ABORT-AND-RESET. So the authority lives here as a ROS-free object
the executive node owns; T2.5 wires the full instruction-change behavior on top.

Semantics (FMEA 2.5):
  * `mission_epoch` is uint32, wrap-safe. A new instruction = ABORT-AND-RESET:
    cancel in-flight goals (the node's job), `epoch++`, clear committed subgoal +
    dedup. Every dispatched skill goal is stamped with the current epoch; servers
    reject a goal whose epoch != current (a "zombie" from a previous mission).
  * Idempotency: a repeated `request_id` within the same epoch is a no-op that
    returns the cached terminal result. Dedup is per-server and epoch-scoped.
"""
from dataclasses import dataclass, field
from typing import Any, Dict, Optional

UINT32_MASK = 0xFFFFFFFF


def epoch_inc(epoch: int) -> int:
    """uint32 wrap-safe increment."""
    return (epoch + 1) & UINT32_MASK


@dataclass
class CommittedSubgoal:
    skill: str
    args: dict
    step_id: str          # UUID stamped at dispatch
    epoch: int


class MissionState:
    """Single source of truth for the active mission and its epoch."""

    def __init__(self, start_epoch: int = 0):
        self._epoch = start_epoch & UINT32_MASK
        self.instruction = ''
        self.allow_vlm = False
        self.active = False
        self.committed: Optional[CommittedSubgoal] = None

    # -- epoch ---------------------------------------------------------------

    def current_epoch(self) -> int:
        return self._epoch

    def is_current(self, epoch: int) -> bool:
        """True if `epoch` matches the active mission epoch (else it's a zombie)."""
        return (epoch & UINT32_MASK) == self._epoch

    # -- mission lifecycle ---------------------------------------------------

    def start_mission(self, instruction: str, allow_vlm: bool) -> int:
        """New instruction => ABORT-AND-RESET: bump epoch, set instruction, clear
        committed subgoal. (Cancelling in-flight goals is the node's job.) Returns
        the new epoch."""
        self.instruction = instruction
        self.allow_vlm = allow_vlm
        self.active = True
        self.committed = None
        self._epoch = epoch_inc(self._epoch)
        return self._epoch

    def abort_and_reset(self) -> int:
        """Bump epoch + clear committed (invalidates all in-flight UUIDs)."""
        self.committed = None
        self._epoch = epoch_inc(self._epoch)
        return self._epoch

    def finish(self) -> None:
        self.active = False
        self.committed = None

    # -- committed subgoal ---------------------------------------------------

    def commit(self, skill: str, args: dict, step_id: str) -> CommittedSubgoal:
        self.committed = CommittedSubgoal(skill=skill, args=dict(args),
                                          step_id=step_id, epoch=self._epoch)
        return self.committed

    def clear_commit(self) -> None:
        self.committed = None


class RequestDedup:
    """Per-server idempotency cache, epoch-scoped (FMEA 2.5).

    A `request_id` seen again in the same epoch returns its cached terminal
    result instead of re-executing. The cache is cleared whenever the epoch
    changes, so a re-used id in a new mission is treated as fresh."""

    def __init__(self):
        self._epoch: Optional[int] = None
        self._cache: Dict[str, Any] = {}

    def _sync(self, epoch: int) -> None:
        if epoch != self._epoch:
            self._epoch = epoch
            self._cache.clear()

    def cached_result(self, request_id: str, epoch: int):
        """Return the cached terminal result for (request_id, epoch), or None."""
        self._sync(epoch)
        return self._cache.get(request_id)

    def remember(self, request_id: str, epoch: int, result: Any) -> None:
        self._sync(epoch)
        if request_id:
            self._cache[request_id] = result

    def size(self) -> int:
        return len(self._cache)
