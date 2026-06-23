"""Unit tests for the mission epoch authority + idempotency (ROADMAP 2.4/2.5)."""
from search_coordinator.mission_state import (
    MissionState,
    RequestDedup,
    UINT32_MASK,
    epoch_inc,
)


def test_epoch_inc_wraps():
    assert epoch_inc(0) == 1
    assert epoch_inc(UINT32_MASK) == 0  # uint32 wrap


def test_start_mission_bumps_and_sets():
    ms = MissionState()
    assert ms.current_epoch() == 0
    e = ms.start_mission('find the red cup', allow_vlm=False)
    assert e == 1 and ms.current_epoch() == 1
    assert ms.instruction == 'find the red cup'
    assert ms.active is True


def test_is_current_rejects_zombie_epoch():
    ms = MissionState()
    ms.start_mission('a', False)          # epoch 1
    assert ms.is_current(1)
    assert not ms.is_current(0)           # stale (previous) epoch = zombie
    ms.start_mission('b', False)          # epoch 2 (new instruction)
    assert ms.is_current(2)
    assert not ms.is_current(1)


def test_abort_and_reset_bumps_and_clears_commit():
    ms = MissionState()
    ms.start_mission('a', False)          # epoch 1
    ms.commit('ExploreFrontier', {'frontier_id': -1}, 'uuid-1')
    assert ms.committed is not None
    e = ms.abort_and_reset()
    assert e == 2
    assert ms.committed is None


def test_commit_records_epoch_and_args_copy():
    ms = MissionState()
    ms.start_mission('a', False)
    args = {'frontier_id': 3}
    c = ms.commit('ExploreFrontier', args, 'uuid-x')
    args['frontier_id'] = 99              # external mutation must not leak in
    assert c.args == {'frontier_id': 3}
    assert c.epoch == 1 and c.step_id == 'uuid-x'


def test_finish_deactivates():
    ms = MissionState()
    ms.start_mission('a', False)
    ms.finish()
    assert ms.active is False and ms.committed is None


# --- idempotency dedup ------------------------------------------------------

def test_dedup_returns_cached_in_same_epoch():
    d = RequestDedup()
    assert d.cached_result('req-1', epoch=1) is None
    d.remember('req-1', 1, 'SUCCEEDED')
    assert d.cached_result('req-1', 1) == 'SUCCEEDED'   # repeat = no-op replay


def test_dedup_cleared_on_epoch_change():
    d = RequestDedup()
    d.remember('req-1', 1, 'SUCCEEDED')
    assert d.cached_result('req-1', 1) == 'SUCCEEDED'
    # same id in a new epoch is fresh (cache reset)
    assert d.cached_result('req-1', 2) is None
    assert d.size() == 0


def test_dedup_ignores_empty_request_id():
    d = RequestDedup()
    d.remember('', 1, 'X')
    assert d.size() == 0
