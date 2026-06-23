"""QoS profile + Request-vs-Offered compatibility tests (ROADMAP Phase 1.3).

Locks the six cross-link profiles against accidental drift and encodes the DDS
compatibility rules — including the action-status bug class (a default VOLATILE
depth subscriber cannot read a TRANSIENT_LOCAL publisher's late-joined sample,
and a BEST_EFFORT publisher cannot satisfy a RELIABLE subscriber).
"""
from rclpy.duration import Duration
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from fleet_comms.qos import (
    control_cmd,
    control_cmd_latched,
    correction_lowrate,
    detection_stream,
    detection_stream_nodeadline,
    is_compatible,
    liveliness_status,
    media_besteffort,
)


def _tracker_target_pixel_offered():
    """Mirror of rgb_tracker_node.py's `tracking_output_qos` for /target_pixel:
    BEST_EFFORT, VOLATILE, depth 1, NO offered deadline (sporadic stream)."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def test_profile_fields():
    cc = control_cmd()
    assert cc.reliability == QoSReliabilityPolicy.RELIABLE
    assert cc.durability == QoSDurabilityPolicy.VOLATILE
    assert cc.history == QoSHistoryPolicy.KEEP_LAST and cc.depth == 1
    assert cc.liveliness == QoSLivelinessPolicy.MANUAL_BY_TOPIC

    lat = control_cmd_latched()
    assert lat.durability == QoSDurabilityPolicy.TRANSIENT_LOCAL

    corr = correction_lowrate()
    assert corr.reliability == QoSReliabilityPolicy.RELIABLE
    assert corr.durability == QoSDurabilityPolicy.VOLATILE

    det = detection_stream()
    assert det.reliability == QoSReliabilityPolicy.BEST_EFFORT

    med = media_besteffort()
    assert med.reliability == QoSReliabilityPolicy.BEST_EFFORT
    assert med.depth == 1


def test_liveliness_status_scales_with_period():
    fast = liveliness_status(0.5)
    slow = liveliness_status(2.0)
    assert fast.deadline.nanoseconds == int(1.5 * 0.5 * 1e9)
    assert slow.liveliness_lease_duration.nanoseconds == int(3.0 * 2.0 * 1e9)


def test_same_profile_is_compatible():
    for factory in (control_cmd, control_cmd_latched, correction_lowrate,
                    detection_stream, media_besteffort):
        ok, reasons = is_compatible(factory(), factory())
        assert ok, reasons
    ok, reasons = is_compatible(liveliness_status(0.5), liveliness_status(0.5))
    assert ok, reasons


def test_heartbeat_producer_monitor_same_period_compatible():
    # Producer (offered) and monitor (requested) MUST agree on period.
    ok, reasons = is_compatible(liveliness_status(0.5), liveliness_status(0.5))
    assert ok, reasons
    # Mismatched periods break the deadline (offered 1.5*1.0 > requested 1.5*0.5).
    ok, _ = is_compatible(liveliness_status(1.0), liveliness_status(0.5))
    assert not ok


def test_durability_action_status_bug_class():
    # The real bug: a default VOLATILE subscriber wants a latched terminal state.
    offered = media_besteffort()  # VOLATILE
    requested = control_cmd_latched()  # TRANSIENT_LOCAL
    ok, reasons = is_compatible(offered, requested)
    assert not ok
    assert any('durability' in r for r in reasons)
    # Reverse is fine: a durable publisher satisfies a volatile subscriber.
    ok, _ = is_compatible(control_cmd_latched(), media_besteffort())
    assert ok


def test_reliability_besteffort_cannot_satisfy_reliable():
    ok, reasons = is_compatible(media_besteffort(), control_cmd())
    assert not ok
    assert any('reliability' in r for r in reasons)


def test_target_pixel_deadline_consumer_is_the_bug():
    # must-fix #2: the no-deadline tracker publisher CANNOT satisfy a consumer that
    # requests detection_stream()'s 1.5 s deadline (offered inf > requested 1.5)
    # -> silent zero samples. This is exactly the pair to forbid.
    offered = _tracker_target_pixel_offered()
    ok, reasons = is_compatible(offered, detection_stream())
    assert not ok
    assert any('deadline' in r for r in reasons)


def test_target_pixel_nodeadline_consumer_is_the_fix():
    # The fix: a no-deadline consumer is compatible with the no-deadline publisher.
    offered = _tracker_target_pixel_offered()
    ok, reasons = is_compatible(offered, detection_stream_nodeadline())
    assert ok, reasons


def test_detection_stream_nodeadline_fields():
    d = detection_stream_nodeadline()
    assert d.reliability == QoSReliabilityPolicy.BEST_EFFORT
    assert d.durability == QoSDurabilityPolicy.VOLATILE
    assert d.depth == 1
    assert d.deadline.nanoseconds == 0  # unset == infinite (no deadline requested)


def test_deadline_offered_must_not_exceed_requested():
    fast_deadline = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                               deadline=Duration(seconds=1.0))
    slow_deadline = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                               deadline=Duration(seconds=2.0))
    # offered 2.0 > requested 1.0 -> incompatible
    ok, reasons = is_compatible(slow_deadline, fast_deadline)
    assert not ok and any('deadline' in r for r in reasons)
    # offered 1.0 <= requested 2.0 -> compatible
    ok, _ = is_compatible(fast_deadline, slow_deadline)
    assert ok
