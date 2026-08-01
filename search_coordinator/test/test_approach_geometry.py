"""Unit tests for the pure pixel->3D-goal geometry (ROADMAP Phase 2.8).

Pure math — no ROS, no TF — so this runs standalone and fast.
"""
import math

from search_coordinator.approach_geometry import (
    CameraIntrinsics,
    DepthRingBuffer,
    approach_goal,
    backproject_pixel,
    bounded_drive_steps,
    embedded_depth,
    goal_update_needed,
    occupancy_clearance_status_at_world,
    occupancy_known_free_at_world,
    occupancy_value_at_world,
    path_clearance_status,
    pixel_age_s,
    quaternion_to_yaw,
    select_safe_bounded_goal,
    select_safe_forward_goal,
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


def test_bounded_drive_steps_descend_to_minimum():
    steps = bounded_drive_steps(1.2, 0.35, 0.2)
    assert steps[0] == 1.2
    assert steps[-1] == 0.35
    assert all(a > b for a, b in zip(steps, steps[1:]))


def test_select_safe_bounded_goal_shortens_until_known_free():
    def status(x, _y):
        return 'known_free' if x <= 0.8 else 'occupied_100'

    selected, last_status = select_safe_bounded_goal(
        target_x=5.0, target_y=0.0,
        robot_x=0.0, robot_y=0.0,
        max_step_m=1.2, min_step_m=0.35, resolution_m=0.2,
        status_fn=status)
    assert selected is not None
    gx, gy, yaw, drive_step, safe_status = selected
    assert math.isclose(gx, 0.8, abs_tol=1e-9)
    assert math.isclose(gy, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)
    assert math.isclose(drive_step, 0.8, abs_tol=1e-9)
    assert safe_status == 'known_free'
    assert last_status == 'known_free'


def test_select_safe_bounded_goal_reports_when_no_safe_step():
    selected, last_status = select_safe_bounded_goal(
        target_x=5.0, target_y=0.0,
        robot_x=0.0, robot_y=0.0,
        max_step_m=1.2, min_step_m=0.35, resolution_m=0.2,
        status_fn=lambda _x, _y: 'unknown')
    assert selected is None
    assert last_status == 'unknown'


def test_select_safe_bounded_goal_allows_short_unknown_probe_when_enabled():
    selected, last_status = select_safe_bounded_goal(
        target_x=5.0, target_y=0.0,
        robot_x=0.0, robot_y=0.0,
        max_step_m=1.2, min_step_m=0.35, resolution_m=0.2,
        status_fn=lambda _x, _y: 'clearance_unknown',
        allow_unknown=True,
        unknown_max_step_m=0.6)
    assert selected is not None
    gx, gy, yaw, drive_step, status = selected
    assert math.isclose(gx, 0.6, abs_tol=1e-9)
    assert math.isclose(gy, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)
    assert math.isclose(drive_step, 0.6, abs_tol=1e-9)
    assert status == 'clearance_unknown'
    assert last_status == 'clearance_unknown'


def test_select_safe_bounded_goal_still_rejects_occupied_when_unknown_allowed():
    selected, last_status = select_safe_bounded_goal(
        target_x=5.0, target_y=0.0,
        robot_x=0.0, robot_y=0.0,
        max_step_m=1.2, min_step_m=0.35, resolution_m=0.2,
        status_fn=lambda _x, _y: 'clearance_occupied_100',
        allow_unknown=True,
        unknown_max_step_m=0.6)
    assert selected is None
    assert last_status == 'clearance_occupied_100'


def test_path_clearance_status_checks_intermediate_segment():
    def status(x, _y):
        return 'clearance_occupied_100' if x > 0.4 else 'known_free'

    ok, status_name = path_clearance_status(
        0.0, 0.0, 0.8, 0.0, 0.1, status)
    assert not ok
    assert status_name == 'clearance_occupied_100'


def test_select_safe_forward_goal_prefers_center_when_clear():
    selected, last_status = select_safe_forward_goal(
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0, desired_step_m=0.55,
        status_fn=lambda _x, _y: 'known_free',
        lateral_offsets_m=(0.0, 0.25, -0.25))
    assert selected is not None
    gx, gy, yaw, step, lateral, status = selected
    assert math.isclose(gx, 0.55, abs_tol=1e-9)
    assert math.isclose(gy, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)
    assert math.isclose(step, 0.55, abs_tol=1e-9)
    assert lateral == 0.0
    assert status == 'known_free'
    assert last_status == 'known_free'


def test_select_safe_forward_goal_shifts_side_when_center_endpoint_blocked():
    def status(x, y):
        if x > 0.5 and abs(y) < 0.1:
            return 'clearance_occupied_100'
        return 'known_free'

    selected, last_status = select_safe_forward_goal(
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0, desired_step_m=0.55,
        status_fn=status,
        lateral_offsets_m=(0.0, 0.25, -0.25),
        path_resolution_m=0.1)
    assert selected is not None
    gx, gy, yaw, step, lateral, safe_status = selected
    assert math.isclose(gx, 0.55, abs_tol=1e-9)
    assert math.isclose(gy, 0.25, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)
    assert math.isclose(step, 0.55, abs_tol=1e-9)
    assert math.isclose(lateral, 0.25, abs_tol=1e-9)
    assert safe_status == 'known_free'
    assert last_status == 'known_free'


def test_select_safe_forward_goal_reports_when_no_fan_candidate_is_safe():
    selected, last_status = select_safe_forward_goal(
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0, desired_step_m=0.55,
        status_fn=lambda _x, _y: 'clearance_unknown',
        lateral_offsets_m=(0.0, 0.25, -0.25))
    assert selected is None
    assert last_status == 'clearance_unknown'


def test_select_safe_forward_goal_allows_short_unknown_frontier_when_enabled():
    selected, last_status = select_safe_forward_goal(
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0, desired_step_m=0.55,
        status_fn=lambda _x, _y: 'clearance_unknown',
        lateral_offsets_m=(0.0, 0.25, -0.25),
        allow_unknown=True,
        unknown_max_step_m=0.6)
    assert selected is not None
    gx, gy, yaw, step, lateral, safe_status = selected
    assert math.isclose(gx, 0.55, abs_tol=1e-9)
    assert math.isclose(gy, 0.0, abs_tol=1e-9)
    assert math.isclose(yaw, 0.0, abs_tol=1e-9)
    assert math.isclose(step, 0.55, abs_tol=1e-9)
    assert lateral == 0.0
    assert safe_status == 'clearance_unknown'
    assert last_status == 'clearance_unknown'


def test_select_safe_forward_goal_prefers_known_free_over_unknown_frontier():
    def status(_x, y):
        return 'clearance_unknown' if abs(y) < 0.01 else 'known_free'

    selected, last_status = select_safe_forward_goal(
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0, desired_step_m=0.55,
        status_fn=status,
        lateral_offsets_m=(0.0, 0.25, -0.25),
        allow_unknown=True,
        unknown_max_step_m=0.6)
    assert selected is not None
    gx, gy, _yaw, _step, lateral, safe_status = selected
    assert math.isclose(gx, 0.55, abs_tol=1e-9)
    assert math.isclose(gy, 0.25, abs_tol=1e-9)
    assert math.isclose(lateral, 0.25, abs_tol=1e-9)
    assert safe_status == 'known_free'
    assert last_status == 'known_free'


def test_select_safe_forward_goal_still_rejects_occupied_when_unknown_enabled():
    selected, last_status = select_safe_forward_goal(
        robot_x=0.0, robot_y=0.0, robot_yaw=0.0, desired_step_m=0.55,
        status_fn=lambda _x, _y: 'clearance_occupied_100',
        lateral_offsets_m=(0.0, 0.25, -0.25),
        allow_unknown=True,
        unknown_max_step_m=0.6)
    assert selected is None
    assert last_status == 'clearance_occupied_100'


def test_occupancy_value_at_world_classifies_free_unknown_and_outside():
    data = [
        0, -1, 100,
        0, 10, 20,
    ]
    assert occupancy_value_at_world(data, 3, 2, 1.0, 0.0, 0.0, 0.0, 0.5, 0.5) == 0
    assert occupancy_value_at_world(data, 3, 2, 1.0, 0.0, 0.0, 0.0, 1.5, 0.5) == -1
    assert occupancy_value_at_world(data, 3, 2, 1.0, 0.0, 0.0, 0.0, 3.5, 0.5) is None


def test_occupancy_known_free_at_world_rejects_unknown_and_occupied():
    data = [0, 20, 65, -1]
    assert occupancy_known_free_at_world(
        data, 4, 1, 1.0, 0.0, 0.0, 0.0, 0.5, 0.5, 65)
    assert occupancy_known_free_at_world(
        data, 4, 1, 1.0, 0.0, 0.0, 0.0, 1.5, 0.5, 65)
    assert not occupancy_known_free_at_world(
        data, 4, 1, 1.0, 0.0, 0.0, 0.0, 2.5, 0.5, 65)
    assert not occupancy_known_free_at_world(
        data, 4, 1, 1.0, 0.0, 0.0, 0.0, 3.5, 0.5, 65)


def test_occupancy_clearance_status_accepts_known_free_radius():
    data = [0] * 25
    ok, status = occupancy_clearance_status_at_world(
        data, 5, 5, 1.0, 0.0, 0.0, 0.0, 2.5, 2.5, 1.1, 65)
    assert ok
    assert status == 'known_free'


def test_occupancy_clearance_status_rejects_near_unknown_and_occupied():
    data = [0] * 25
    data[2 * 5 + 3] = -1
    ok, status = occupancy_clearance_status_at_world(
        data, 5, 5, 1.0, 0.0, 0.0, 0.0, 2.5, 2.5, 1.1, 65)
    assert not ok
    assert status == 'unknown'

    data[2 * 5 + 3] = 80
    ok, status = occupancy_clearance_status_at_world(
        data, 5, 5, 1.0, 0.0, 0.0, 0.0, 2.5, 2.5, 1.1, 65)
    assert not ok
    assert status == 'occupied_80'


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
