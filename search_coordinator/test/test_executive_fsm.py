"""Unit tests for the pure executive FSM core (ROADMAP 2.2)."""
from search_coordinator.executive_fsm import (
    EVENT,
    SKILL_APPROACH,
    SKILL_EXPLORE,
    SKILL_STOP,
    STATE,
    is_terminal,
    next_state,
    progress_for,
    select_subgoal,
)


def test_happy_path_walk():
    s = STATE.IDLE
    s = next_state(s, EVENT.START);      assert s == STATE.SEARCH
    s = next_state(s, EVENT.DETECTED);   assert s == STATE.DETECT
    s = next_state(s, EVENT.COMPUTABLE); assert s == STATE.APPROACH
    s = next_state(s, EVENT.REACHED);    assert s == STATE.DONE
    assert is_terminal(s)


def test_approach_lost_returns_to_search():
    assert next_state(STATE.APPROACH, EVENT.LOST) == STATE.SEARCH


def test_detect_lost_returns_to_search():
    assert next_state(STATE.DETECT, EVENT.LOST) == STATE.SEARCH


def test_no_frontier_fails():
    assert next_state(STATE.SEARCH, EVENT.NO_FRONTIER) == STATE.FAILED
    assert is_terminal(STATE.FAILED)


def test_new_instruction_from_active_goes_stop():
    for s in (STATE.SEARCH, STATE.DETECT, STATE.APPROACH):
        assert next_state(s, EVENT.NEW_INSTRUCTION) == STATE.STOP


def test_new_instruction_terminal_is_noop():
    assert next_state(STATE.DONE, EVENT.NEW_INSTRUCTION) == STATE.DONE
    assert next_state(STATE.FAILED, EVENT.NEW_INSTRUCTION) == STATE.FAILED


def test_stop_reset_rearms_search():
    assert next_state(STATE.STOP, EVENT.RESET) == STATE.SEARCH


def test_unknown_pair_is_noop():
    assert next_state(STATE.SEARCH, EVENT.REACHED) == STATE.SEARCH
    assert next_state(STATE.IDLE, EVENT.DETECTED) == STATE.IDLE


def test_select_subgoal_default_productive():
    # Active SEARCH always drives EXPLORE_FRONTIER (never None / idle-spin).
    assert select_subgoal(STATE.SEARCH) == SKILL_EXPLORE
    assert select_subgoal(STATE.APPROACH) == SKILL_APPROACH
    assert select_subgoal(STATE.STOP) == SKILL_STOP


def test_select_subgoal_none_when_nothing_to_drive():
    for s in (STATE.IDLE, STATE.DETECT, STATE.VLM, STATE.DONE, STATE.FAILED):
        assert select_subgoal(s) is None


def test_progress_monotone_milestones():
    assert progress_for(STATE.SEARCH) < progress_for(STATE.DETECT)
    assert progress_for(STATE.DETECT) < progress_for(STATE.APPROACH)
    assert progress_for(STATE.VLM) > progress_for(STATE.SEARCH)
    assert progress_for(STATE.DONE) == 1.0
