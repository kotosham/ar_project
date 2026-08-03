from fleet_comms.flat_mission_logger import FlatMissionTracker


def _status(state, epoch=1, instruction='chair', stamp=0.0, outcome=''):
    return {
        'state': state,
        'active_subtask': '',
        'progress': 0.0,
        'mission_epoch': epoch,
        'instruction': instruction,
        'outcome': outcome,
        'stamp': stamp,
    }


def test_flat_tracker_successful_mission():
    tracker = FlatMissionTracker('flat_scene_1')

    assert tracker.update(_status('SEARCH', stamp=10.0), rx=100.0) is None
    assert tracker.update(_status('DETECT', stamp=11.0), rx=101.0) is None
    assert tracker.update(_status('APPROACH', stamp=12.0), rx=102.0) is None
    row = tracker.update(_status('DONE', stamp=15.0, outcome='reached'), rx=105.0)

    assert row['terminal_state'] == 'DONE'
    assert row['success_auto'] == 1
    assert row['flat_progress_rate'] == 1.0
    assert row['max_state_reached'] == 'DONE'
    assert row['duration_s'] == 5.0
    assert row['duration_status_s'] == 5.0
    assert row['time_to_first_action_s'] == 0.0
    assert row['time_to_detect_s'] == 1.0
    assert row['time_to_approach_s'] == 2.0
    assert row['states_seen'] == 'SEARCH>DETECT>APPROACH>DONE'


def test_flat_tracker_records_detector_runtime_during_mission():
    tracker = FlatMissionTracker('flat_scene_runtime')

    tracker.record_detector_runtime(99.0)
    tracker.update(_status('SEARCH', stamp=10.0), rx=100.0)
    tracker.record_detector_runtime(0.52)
    tracker.record_detector_runtime(0.48)
    row = tracker.update(_status('DONE', stamp=12.0, outcome='reached'), rx=102.0)

    assert row['detector_runtime_mean_s'] == 0.48
    assert row['detector_runtime_samples'] == 2


def test_flat_tracker_failed_after_search_is_partial_progress():
    tracker = FlatMissionTracker('flat_scene_2')

    tracker.update(_status('SEARCH', stamp=20.0), rx=200.0)
    row = tracker.update(_status('FAILED', stamp=25.0, outcome='frontiers exhausted'),
                         rx=205.0)

    assert row['terminal_state'] == 'FAILED'
    assert row['success_auto'] == 0
    assert row['flat_progress_rate'] == 0.33
    assert row['max_state_reached'] == 'SEARCH'
    assert row['time_to_first_action_s'] == 0.0
    assert row['time_to_detect_s'] == ''
    assert row['time_to_approach_s'] == ''


def test_flat_tracker_failed_after_approach_stops_at_detection_progress():
    tracker = FlatMissionTracker('flat_scene_3')

    tracker.update(_status('SEARCH', stamp=30.0), rx=300.0)
    tracker.update(_status('DETECT', stamp=31.0), rx=301.0)
    tracker.update(_status('APPROACH', stamp=32.0), rx=302.0)
    row = tracker.update(_status('FAILED', stamp=40.0, outcome='lost target'), rx=310.0)

    assert row['terminal_state'] == 'FAILED'
    assert row['success_auto'] == 0
    assert row['flat_progress_rate'] == 0.66
    assert row['max_state_reached'] == 'APPROACH'
    assert row['time_to_detect_s'] == 1.0
    assert row['time_to_approach_s'] == 2.0


def test_flat_tracker_ignores_vlm_epoch_by_default():
    tracker = FlatMissionTracker('mixed_run')

    assert tracker.update(_status('VLM', epoch=7, instruction='chair'), rx=1.0) is None
    assert tracker.update(_status('DONE', epoch=7, instruction='chair'), rx=2.0) is None
    assert tracker.mission_index == 0


def test_flat_tracker_starts_new_epoch():
    tracker = FlatMissionTracker('flat_scene_4')

    tracker.update(_status('SEARCH', epoch=1, instruction='chair'), rx=10.0)
    first = tracker.update(_status('DONE', epoch=1, instruction='chair'), rx=11.0)
    tracker.update(_status('SEARCH', epoch=2, instruction='chair'), rx=20.0)
    second = tracker.update(_status('DONE', epoch=2, instruction='chair'), rx=21.0)

    assert first['mission_index'] == 1
    assert second['mission_index'] == 2


def test_flat_tracker_ignores_terminal_without_active_mission():
    tracker = FlatMissionTracker('flat_scene_5')

    assert tracker.update(_status('FAILED', epoch=1, instruction='chair'), rx=10.0) is None
    assert tracker.mission_index == 0


def test_flat_tracker_ignores_duplicate_terminal_status():
    tracker = FlatMissionTracker('flat_scene_6')

    tracker.update(_status('SEARCH', epoch=1, instruction='chair'), rx=10.0)
    row = tracker.update(_status('FAILED', epoch=1, instruction='chair'), rx=11.0)
    duplicate = tracker.update(
        _status('FAILED', epoch=1, instruction='chair', outcome='frontiers exhausted'),
        rx=12.0)

    assert row['mission_index'] == 1
    assert duplicate is None
    assert tracker.mission_index == 1
