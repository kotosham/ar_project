"""Unit tests for map_odom_relay pure gating helpers (ROADMAP Phase 2.6).

Tests the ROS-free logic (uint32 seq wrap, quaternion angle). The full gating
pipeline (covariance/fitness/jump/stale) is exercised node-level in the T2.6
verification script.
"""
import math

from geometry_msgs.msg import Quaternion

from search_coordinator.map_odom_relay import _quat_angle, _seq_is_newer


def test_seq_is_newer_basic():
    assert _seq_is_newer(2, 1)
    assert not _seq_is_newer(1, 1)   # duplicate
    assert not _seq_is_newer(1, 2)   # older


def test_seq_is_newer_uint32_wrap():
    # 0 is newer than 0xFFFFFFFF (just wrapped)
    assert _seq_is_newer(0, 0xFFFFFFFF)
    # 0xFFFFFFFF is older than 0
    assert not _seq_is_newer(0xFFFFFFFF, 0)
    # a few past the wrap
    assert _seq_is_newer(3, 0xFFFFFFFE)


def _q(z, w):
    q = Quaternion()
    q.z, q.w = z, w
    return q


def test_quat_angle_identical_is_zero():
    assert math.isclose(_quat_angle(_q(0.0, 1.0), _q(0.0, 1.0)), 0.0, abs_tol=1e-9)


def test_quat_angle_ninety_degrees():
    a = _q(0.0, 1.0)                                 # identity
    b = _q(math.sin(math.pi / 4), math.cos(math.pi / 4))  # 90 deg about z
    assert math.isclose(_quat_angle(a, b), math.pi / 2, abs_tol=1e-6)
