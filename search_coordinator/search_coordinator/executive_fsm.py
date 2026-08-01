"""Pure executive FSM core for SeekObject (ROADMAP 2.2).

ROS-free so the transition table + subgoal selection are unit-testable without
actions. The SeekObject server (coordinator) feeds events in and acts on the
returned (next_state, subgoal). FLAT-mode walk:

    IDLE --START--> SEARCH --DETECTED--> DETECT --COMPUTABLE--> APPROACH
    APPROACH --REACHED--> DONE
    APPROACH/DETECT --LOST--> SEARCH        (never a false reach)
    SEARCH --NO_FRONTIER--> FAILED          (frontiers exhausted)
    <any active> --NEW_INSTRUCTION--> STOP   (then RESET re-arms SEARCH, epoch++)
    STOP --RESET--> SEARCH

Invariants (FMEA):
  * committed-subgoal: exactly one committed subgoal while a goal is active.
  * default-productive-action: select_subgoal never returns None in an active
    state; the SEARCH default is EXPLORE_FRONTIER. Terminal/STOP states return
    None (nothing to drive). Never idle-spin, never reactive cmd_vel.
"""


class STATE:
    IDLE = 'IDLE'
    SEARCH = 'SEARCH'
    DETECT = 'DETECT'
    APPROACH = 'APPROACH'
    VLM = 'VLM'                 # Handoff to Planner Orchestrator; no FLAT FSM skills
    STOP = 'STOP'
    DEGRADED = 'DEGRADED'     # VLM->FLAT (Phase 5); never entered in FLAT
    DONE = 'DONE'
    FAILED = 'FAILED'


class EVENT:
    START = 'START'                   # SeekObject goal accepted
    DETECTED = 'DETECTED'             # a fresh target pixel arrived
    COMPUTABLE = 'COMPUTABLE'         # a 3D approach goal could be computed
    REACHED = 'REACHED'              # ApproachDetection reached the pose while fresh
    LOST = 'LOST'                     # detection went stale / was lost
    NO_FRONTIER = 'NO_FRONTIER'       # ExploreFrontier found nothing
    NEW_INSTRUCTION = 'NEW_INSTRUCTION'
    RESET = 'RESET'                   # abort-and-reset complete -> re-arm


# Skill names the FSM dispatches (loopback action clients in the server).
SKILL_EXPLORE = 'ExploreFrontier'
SKILL_APPROACH = 'ApproachDetection'
SKILL_STOP = 'Stop'

TERMINAL_STATES = frozenset({STATE.DONE, STATE.FAILED})
ACTIVE_STATES = frozenset({STATE.SEARCH, STATE.DETECT, STATE.APPROACH})

# (state, event) -> next_state. Missing pairs => no transition (stay put).
_TABLE = {
    (STATE.IDLE, EVENT.START): STATE.SEARCH,

    (STATE.SEARCH, EVENT.DETECTED): STATE.DETECT,
    (STATE.SEARCH, EVENT.NO_FRONTIER): STATE.FAILED,

    (STATE.DETECT, EVENT.COMPUTABLE): STATE.APPROACH,
    (STATE.DETECT, EVENT.LOST): STATE.SEARCH,

    (STATE.APPROACH, EVENT.REACHED): STATE.DONE,
    (STATE.APPROACH, EVENT.LOST): STATE.SEARCH,

    (STATE.STOP, EVENT.RESET): STATE.SEARCH,
}


def next_state(state: str, event: str) -> str:
    """Return the next FSM state for (state, event).

    NEW_INSTRUCTION from any non-terminal state goes to STOP (then the server
    runs ABORT-AND-RESET and feeds RESET to re-arm SEARCH). Otherwise the
    transition table applies; an unknown pair is a no-op (stay in `state`)."""
    if event == EVENT.NEW_INSTRUCTION:
        return STATE.STOP if state not in TERMINAL_STATES else state
    return _TABLE.get((state, event), state)


def select_subgoal(state: str):
    """The committed subgoal to drive in `state`, or None when there is nothing
    to drive (terminal / STOP / IDLE / transient DETECT). Default-productive:
    SEARCH always returns EXPLORE_FRONTIER (never idle-spin)."""
    if state == STATE.SEARCH:
        return SKILL_EXPLORE
    if state == STATE.APPROACH:
        return SKILL_APPROACH
    if state == STATE.STOP:
        return SKILL_STOP
    return None


def is_terminal(state: str) -> bool:
    return state in TERMINAL_STATES


def progress_for(state: str) -> float:
    """Coarse 0..1 progress for SeekObject feedback."""
    return {
        STATE.IDLE: 0.0,
        STATE.SEARCH: 0.25,
        STATE.DETECT: 0.5,
        STATE.APPROACH: 0.75,
        STATE.VLM: 0.5,
        STATE.DONE: 1.0,
        STATE.FAILED: 1.0,
        STATE.STOP: 0.0,
    }.get(state, 0.0)
