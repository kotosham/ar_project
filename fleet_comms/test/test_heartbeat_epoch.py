"""Unit tests for the HeartbeatMonitor mission-epoch filter (ROADMAP 2.5, must-fix #8).

The wrap-safe staleness predicate is pure; the monitor's behavior on top of it
(ignore stale-epoch beats so a lagging producer reads STALE) is exercised in the
T2.5 node smoke."""
from fleet_comms.heartbeat import _epoch_is_stale


def test_equal_epoch_not_stale():
    assert _epoch_is_stale(5, 5) is False


def test_older_epoch_is_stale():
    assert _epoch_is_stale(4, 5) is True
    assert _epoch_is_stale(1, 1000) is True


def test_newer_epoch_not_stale():
    # a beat ahead of the monitor (shouldn't happen — executive owns both) is
    # tolerated rather than dropped.
    assert _epoch_is_stale(6, 5) is False


def test_uint32_wrap_previous_epoch_is_stale():
    # current just wrapped to 0; the previous epoch 0xFFFFFFFF is OLDER -> stale.
    assert _epoch_is_stale(0xFFFFFFFF, 0) is True


def test_uint32_wrap_next_epoch_not_stale():
    # current=0xFFFFFFFF, a beat at 0 is the NEXT epoch (newer) -> not stale.
    assert _epoch_is_stale(0, 0xFFFFFFFF) is False
