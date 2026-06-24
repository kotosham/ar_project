"""Phase 5 FMEA tests (sim/logic level). Each pins a failure-mode invariant:

  5.4  edge loss mid-approach   -> stale pixel is NOT fresh => no false `reached`
  5.6  instruction-change mid-mission -> epoch bump invalidates in-flight UUIDs
  5.7  frontier oscillation under noise -> hysteresis holds the committed choice

5.1 (seamless VLM->FLAT degradation) is tested in planner_orchestrator
(test_planner_logic.py::DegradationLatch) + a live sim demo. 5.2/5.3 (stale /
jumped MapOdomCorrection gating) are covered by test_map_odom_relay_logic.py
(seq/stamp/jump/reloc/fitness/cov). 5.5 (CAN bus-off / EPOS4 fault quick-stop) is
HIL-only (no CAN in Gazebo) -- see ROADMAP 0.4/0.7/6.2.
"""
from search_coordinator.mission_state import MissionState, RequestDedup
from search_coordinator.frontier_lib import should_switch, HysteresisParams
from search_coordinator.skill_logic import is_fresh


# ---- 5.4: edge loss mid-approach -> no false reached -----------------------

def test_stale_pixel_is_not_fresh_no_false_reached():
    assert is_fresh(0.5, 1.5)            # fresh detection -> may declare reached
    assert not is_fresh(2.0, 1.5)        # stale (edge/Wi-Fi lost) -> STALE_DETECTION
    assert not is_fresh(None, 1.5)       # never seen -> LOST_TARGET, not a false reach


# ---- 5.6: instruction change mid-mission invalidates in-flight UUIDs -------

def test_instruction_change_makes_inflight_goal_a_zombie():
    ms = MissionState()
    e1 = ms.start_mission('find red cup', allow_vlm=False)
    assert ms.is_current(e1)             # a skill goal sent now carries e1
    e2 = ms.start_mission('find blue ball', allow_vlm=False)   # new instruction
    assert e2 != e1
    assert not ms.is_current(e1)         # the in-flight e1 goal is now a zombie
    assert ms.is_current(e2)


def test_abort_and_reset_bumps_epoch_and_clears_commit():
    ms = MissionState(start_epoch=5)
    ms.commit('explore_frontier', {'frontier_id': 1}, 'step-1')
    e = ms.abort_and_reset()
    assert e != 5 and ms.is_current(e) and not ms.is_current(5)
    assert ms.committed is None


def test_dedup_replays_in_epoch_but_never_across_epoch_change():
    dd = RequestDedup()
    dd.remember('uuid-1', 1, ('succeed', 'R'))
    assert dd.cached_result('uuid-1', 1) == ('succeed', 'R')   # idempotent replay
    assert dd.cached_result('uuid-1', 2) is None               # new epoch -> cache cleared
    assert dd.cached_result('uuid-1', 1) is None               # no zombie replay of old result


# ---- 5.7: frontier oscillation under noise -> hysteresis holds -------------

HYS = HysteresisParams(score_margin=5.0, min_dwell_s=3.0)


def test_hysteresis_holds_committed_under_subthreshold_noise():
    # committed id=1 @100; a competitor id=2 jitters within +/- margin -> NEVER switch
    for noise in (1.0, 4.9, -2.0, 3.0, 0.0):
        assert not should_switch(True, 100.0, best_id=2, best_score=100.0 + noise,
                                 committed_id=1, dwell_s=10.0, params=HYS)


def test_hysteresis_switches_only_when_beaten_by_margin_after_dwell():
    assert should_switch(True, 100.0, 2, 106.0, 1, dwell_s=5.0, params=HYS)   # margin+dwell
    assert not should_switch(True, 100.0, 2, 106.0, 1, dwell_s=1.0, params=HYS)  # too soon
    assert not should_switch(True, 100.0, 2, 104.0, 1, dwell_s=9.0, params=HYS)  # within margin


def test_hysteresis_switches_immediately_when_committed_vanishes():
    # committed frontier consumed/disappeared -> switch now for progress (no dwell wait)
    assert should_switch(False, 100.0, best_id=2, best_score=10.0,
                         committed_id=1, dwell_s=0.0, params=HYS)
