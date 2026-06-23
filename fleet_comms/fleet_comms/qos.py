"""Reusable QoS profiles for cross-link (Pi<->edge) ROS 2 endpoints.

Single source of truth for ROADMAP Phase 1.3. Every cross-link publisher/
subscriber should take its QoS from one of these factories instead of
hand-rolling reliability/durability/deadline/liveliness and drifting apart
(which is exactly the bug class — default RELIABLE depth=10 vs Nav2
TRANSIENT_LOCAL depth=1 — that breaks action-status subscriptions silently).

See ar_project/docs/qos_policy.md for which endpoint uses which profile and why.
"""
from rclpy.duration import Duration
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSLivelinessPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)


def control_cmd() -> QoSProfile:
    """Cross-link command/control where loss is unacceptable but only the latest
    matters: SeekObject goal, DetectTarget goal, PlanStep. RELIABLE + deadline +
    liveliness so the executive learns within seconds if the producer goes silent.
    """
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=2.0),
        liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=Duration(seconds=3.0),
    )


def control_cmd_latched() -> QoSProfile:
    """Latched terminal state a late/reconnecting peer must recover after a Wi-Fi
    drop: SeekObject result/status. Durable, depth=1. Principled replacement for
    the reliable_prompt_sender retry loop (deleted in Phase 2.9)."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        liveliness=QoSLivelinessPolicy.AUTOMATIC,
    )


def liveliness_status(period_s: float = 1.0) -> QoSProfile:
    """Heartbeat.msg and other periodic cross-link health/status. deadline ~1.5x
    publish period, liveliness lease ~3x period. The missed-deadline and
    lost-liveliness callbacks are the VLM->FLAT degradation trigger (Phase 5.1).

    The producer and the monitor MUST use the SAME period so the offered/requested
    QoS stay compatible (see is_compatible)."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=1.5 * period_s),
        liveliness=QoSLivelinessPolicy.MANUAL_BY_TOPIC,
        liveliness_lease_duration=Duration(seconds=3.0 * period_s),
    )


def correction_lowrate() -> QoSProfile:
    """Low-rate localization correction over the link: MapOdomCorrection (~1-2 Hz).
    RELIABLE so a correction is not dropped; VOLATILE so no stale relocalization is
    replayed on reconnect (the relay keeps last-good and seq-gates itself);
    deadline lets the relay detect a SLAM stall and hold last-good (Phase 5.2)."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=1.0),
        liveliness=QoSLivelinessPolicy.AUTOMATIC,
        liveliness_lease_duration=Duration(seconds=3.0),
    )


def detection_stream() -> QoSProfile:
    """Lossy detection stream where only the freshest sample matters and stale
    data is harmful: /target_pixel. deadline mirrors the consumer max_pixel_age_s
    staleness gate so the freshness guard is QoS-observable."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
        deadline=Duration(seconds=1.5),
    )


def media_besteffort() -> QoSProfile:
    """COMPRESSED media explicitly allowed to cross: CompressedImage views/bursts,
    annotated Set-of-Mark frames. depth=1 to avoid head-of-line blocking and
    stale-frame buildup on lossy Wi-Fi. RAW depth / PointCloud2 must NEVER cross
    at all (see ROADMAP 1.4/3.5) — this profile is not a license to send them."""
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


# --- DDS Request-vs-Offered compatibility (the subset that bites cross-link
# pub/sub). Lets a test catch incompatible pairs before they silently drop data.

_RELIABILITY_RANK = {
    QoSReliabilityPolicy.BEST_EFFORT: 0,
    QoSReliabilityPolicy.RELIABLE: 1,
}
_DURABILITY_RANK = {
    QoSDurabilityPolicy.VOLATILE: 0,
    QoSDurabilityPolicy.TRANSIENT_LOCAL: 1,
}
_LIVELINESS_RANK = {
    QoSLivelinessPolicy.AUTOMATIC: 0,
    QoSLivelinessPolicy.MANUAL_BY_TOPIC: 1,
}


def _seconds(duration: Duration) -> float:
    """Seconds, treating the unset (0) duration as infinite — which is how DDS
    interprets a default deadline / liveliness lease."""
    ns = duration.nanoseconds
    return float('inf') if ns == 0 else ns / 1e9


def is_compatible(offered: QoSProfile, requested: QoSProfile):
    """Return (compatible: bool, reasons: list[str]) for a publisher (offered) and
    subscriber (requested) QoS pair, per the DDS Request-vs-Offered rules:
      reliability/durability/liveliness-kind: offered must be >= requested;
      deadline period:        offered must be <= requested;
      liveliness lease:       offered must be <= requested.
    """
    reasons = []
    if _RELIABILITY_RANK.get(offered.reliability, 1) < _RELIABILITY_RANK.get(requested.reliability, 1):
        reasons.append('reliability: offered BEST_EFFORT < requested RELIABLE')
    if _DURABILITY_RANK.get(offered.durability, 0) < _DURABILITY_RANK.get(requested.durability, 0):
        reasons.append('durability: offered VOLATILE < requested TRANSIENT_LOCAL')
    if _seconds(offered.deadline) > _seconds(requested.deadline):
        reasons.append('deadline: offered period > requested period')
    if _LIVELINESS_RANK.get(offered.liveliness, 0) < _LIVELINESS_RANK.get(requested.liveliness, 0):
        reasons.append('liveliness kind: offered < requested')
    if _seconds(offered.liveliness_lease_duration) > _seconds(requested.liveliness_lease_duration):
        reasons.append('liveliness lease: offered > requested')
    return (len(reasons) == 0, reasons)
