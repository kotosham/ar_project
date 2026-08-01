"""Unit tests for the SeekObject -> /vlm_mission handoff helpers."""

from search_coordinator.seek_object_server import (
    flat_initial_scan_turns,
    vlm_activity_matches_instruction,
    vlm_activity_stamp,
)


def test_vlm_activity_matches_raw_query_for_riddle():
    payload = {
        'event': 'mission_start',
        'raw_query': 'the thing people sit on while working at a desk',
        'target': 'office chair',
        'detection_query': 'office chair',
    }
    assert vlm_activity_matches_instruction(
        payload, 'the thing people sit on while working at a desk')
    assert not vlm_activity_matches_instruction(payload, 'drawer cabinet')


def test_vlm_activity_matches_direct_or_normalized_target_fields():
    assert vlm_activity_matches_instruction(
        {'event': 'mission_end', 'target': 'chair'}, ' chair ')
    assert vlm_activity_matches_instruction(
        {'event': 'mission_end', 'canonical_target': 'office chair'}, 'office chair')
    assert vlm_activity_matches_instruction(
        {'event': 'mission_end', 'detection_query': 'black office chair'},
        'black office chair')


def test_vlm_activity_stamp_invalid_is_zero():
    assert vlm_activity_stamp({'stamp': '12.5'}) == 12.5
    assert vlm_activity_stamp({'stamp': 'bad'}) == 0.0
    assert vlm_activity_stamp({}) == 0.0


def test_flat_initial_scan_turns_match_vlm_overview_pattern():
    assert flat_initial_scan_turns(1.57, 3.14) == [
        ('right', -1.57),
        ('forward', 1.57),
        ('left', 1.57),
    ]


def test_flat_initial_scan_turns_are_magnitude_params():
    assert flat_initial_scan_turns(-0.6, -1.2) == [
        ('right', -0.6),
        ('forward', 0.6),
        ('left', 0.6),
    ]
