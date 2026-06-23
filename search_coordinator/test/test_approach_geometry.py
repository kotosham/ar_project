"""Unit tests for the pure pixel->3D-goal geometry (ROADMAP Phase 2.8).

Pure math — no ROS, no TF — so this runs standalone and fast.
"""
import math

from search_coordinator.approach_geometry import (
    CameraIntrinsics,
    DepthRingBuffer,
    approach_goal,
    backproject_pixel,
    embedded_depth,
    goal_update_needed,
    pixel_age_s,
    quaternion_to_yaw,
    valid_depth,
    yaw_to_quaternion_zw,
)

INTR = CameraIntrinsics(fx=600.0, fy=600.0, cx=320.0, cy=240.0)


def test_camera_intrinsics_from_k():
    k = [600.0, 0.0, 320.0, 0.0, 600.0, 240.0, 0.0, 0.0, 1.0]
    intr = CameraIntrinsics.from_k(k)
    assert (intr.fx, intr.fy, intr.cx, intr.cy) == (600.0, 600.0, 320.0, 240.0)


def test_backproject_center_pixel_is_on_axis():
    # A pixel at the principal point projects to (0, 0, depth).
    x, y, z = backproject_pixel(320.0, 240.0, 2.5, INTR)
    assert abs(x) < 1e-9 and abs(y) < 1e-9 and z == 2.5


def test_backproject_offset_pixel():
    # x = (u-cx)*d/fx ; pixel 80px right of center at 3m -> 0.4 m.
    x, y, z = backproject_pixel(400.0, 240.0, 3.0, INTR)
    assert math.isclose(x, (400 - 320) * 3.0 / 600.0)  # 0.4
    assert abs(y) < 1e-9 and z == 3.0


def test_valid_and_embedded_depth():
    assert valid_depth(2.0, 0.1, 6.0)
    assert not valid_depth(0.05, 0.1, 6.0)
    assert not valid_depth(float('nan'), 0.1, 6.0)
    assert embedded_depth(3.0, 0.1, 6.0) == 3.0
    assert embedded_depth(99.0, 0.1, 6.0) is None
    assert embedded_depth(float('inf'), 0.1, 6.0) is None


def test_approach_goal_backs_off_along_ray_and_faces_target():
    # Robot at origin, target 4 m ahead on +x, offset 0.58 -> goal at (3.42, 0), yaw 0.
    gx, gy, yaw = approach_goal(4.0, 0.0, 0.0, 0.0, 0.58)
    assert math.isclose(gx, 3.42, abs_tol=1e-6)
    assert math.isclose(gy, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


def test_approach_goal_diagonal_yaw():
    gx, gy, yaw = approach_goal(3.0, 3.0, 0.0, 0.0, 0.0)
    assert math.isclose(yaw, math.pi / 4, abs_tol=1e-9)
    assert math.isclose(gx, 3.0) and math.isclose(gy, 3.0)


def test_approach_goal_inside_offset_holds_position_but_faces_target():
    # Target only 0.3 m away, offset 0.58 -> hold robot position, still face target.
    gx, gy, yaw = approach_goal(0.3, 0.0, 0.0, 0.0, 0.58)
    assert gx == 0.0 and gy == 0.0
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)


def test_yaw_quaternion_roundtrip():
    for yaw in (0.0, 0.5, -1.2, math.pi / 2, 2.9):
        qz, qw = yaw_to_quaternion_zw(yaw)
        back = quaternion_to_yaw(qz, qw)
        assert math.isclose(math.atan2(math.sin(yaw - back), math.cos(yaw - back)), 0.0, abs_tol=1e-9)


def test_goal_update_needed():
    # Below both thresholds -> no update.
    assert not goal_update_needed(0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.2)
    # Translation past threshold -> update.
    assert goal_update_needed(0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.15, 0.2)
    # Yaw past threshold -> update.
    assert goal_update_needed(0.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.15, 0.2)


def test_pixel_age():
    assert math.isclose(pixel_age_s(2_000_000_000, 1_000_000_000), 1.0)


def test_depth_ring_buffer_match_within_and_outside_tolerance():
    buf = DepthRingBuffer(maxlen=5)
    buf.append(1_000_000_000, 'a')
    buf.append(1_100_000_000, 'b')  # +0.1 s
    buf.append(1_500_000_000, 'c')  # +0.5 s
    # Closest to 1.12 s is 'b' (0.02 s away) within 0.35 s tolerance.
    assert buf.match(1_120_000_000, 0.35) == 'b'
    # Nothing within 0.05 s of 2.0 s.
    assert buf.match(2_000_000_000, 0.05) is None
    assert len(buf) == 3


def test_depth_ring_buffer_empty():
    assert DepthRingBuffer().match(0, 1.0) is None
