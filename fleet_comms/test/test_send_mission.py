"""Tests for the lightweight send_mission CLI argument parsing."""

import pytest

from fleet_comms.send_mission import _parse_args, parse_bool


def test_parse_bool_accepts_mode_words():
    assert parse_bool('true') is True
    assert parse_bool('vlm') is True
    assert parse_bool('false') is False
    assert parse_bool('flat') is False


def test_parse_short_positional_mode():
    args = _parse_args(['office chair', 'true'])
    assert args.instruction == 'office chair'
    assert args.allow_vlm is True
    assert args.request_id.startswith('vlm_office_chair_')


def test_parse_flag_mode_keeps_multiword_instruction():
    args = _parse_args(['the thing people sit on', '--vlm', 'true'])
    assert args.instruction == 'the thing people sit on'
    assert args.allow_vlm is True


def test_parse_flat_mode():
    args = _parse_args(['drawer cabinet', 'false'])
    assert args.instruction == 'drawer cabinet'
    assert args.allow_vlm is False
    assert args.request_id.startswith('flat_drawer_cabinet_')


def test_missing_mode_exits():
    with pytest.raises(SystemExit):
        _parse_args(['chair'])
