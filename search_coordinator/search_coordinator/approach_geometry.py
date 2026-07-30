"""Pure pixel->3D-goal geometry for ApproachDetection (ROADMAP Phase 2.8).

Extracted from the legacy ar_project/scripts/target_pixel_to_goal.py so the math
is reusable and unit-testable WITHOUT a ROS node, TF, CameraInfo subscription, or
any of the goal_locked / prompt_ack / final_approach_freeze "soup" (that latch
logic is deleted in Phase 2.9). The ApproachDetection action server (Phase 2.4)
wires TF lookups and message construction around these functions:

    cam_xyz = backproject_pixel(u, v, depth, intr)         # camera optical frame
    # server: TF-transform cam point -> target frame, read robot pose in target
    gx, gy, yaw = approach_goal(tx, ty, rx, ry, offset)    # target frame goal
"""
import math
from collections import deque
from dataclasses import dataclass


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    @classmethod
    def from_k(cls, k):
        """From CameraInfo.k (row-major 3x3: [fx,0,cx, 0,fy,cy, 0,0,1])."""
        return cls(fx=float(k[0]), fy=float(k[4]), cx=float(k[2]), cy=float(k[5]))


def backproject_pixel(u, v, depth_m, intr):
    """Pixel (u, v) + metric depth -> 3D point (x, y, z) in the camera optical frame."""
    x = (u - intr.cx) * depth_m / intr.fx
    y = (v - intr.cy) * depth_m / intr.fy
    return (x, y, float(depth_m))


def valid_depth(depth_m, min_depth_m, max_depth_m):
    return math.isfinite(depth_m) and (min_depth_m <= depth_m <= max_depth_m)


def embedded_depth(point_z, min_depth_m, max_depth_m):
    """Depth carried in PointStamped.z, or None if non-finite / out of range."""
    d = float(point_z)
    return d if valid_depth(d, min_depth_m, max_depth_m) else None


def approach_goal(target_x, target_y, robot_x, robot_y, approach_offset):
    """Goal (x, y, yaw) at `approach_offset` back from the target along the
    robot->target ray, facing the target. If the robot is already inside the
    offset, hold position but still face the target."""
    dx = target_x - robot_x
    dy = target_y - robot_y
    distance = math.hypot(dx, dy)
    if distance <= approach_offset:
        goal_x, goal_y = robot_x, robot_y
    else:
        scale = max(0.0, (distance - approach_offset) / distance)
        goal_x = robot_x + dx * scale
        goal_y = robot_y + dy * scale
    return (goal_x, goal_y, math.atan2(dy, dx))


def bounded_drive_steps(max_step_m, min_step_m, resolution_m):
    """Descending candidate drive distances for a bounded visual approach."""
    max_step = max(0.0, float(max_step_m))
    if max_step <= 0.0:
        return []
    min_step = max(0.0, min(float(min_step_m), max_step))
    resolution = max(0.01, float(resolution_m))
    steps = []
    cur = max_step
    while cur >= min_step - 1e-9:
        steps.append(cur)
        cur -= resolution
    if not steps or abs(steps[-1] - min_step) > 1e-6:
        steps.append(min_step)
    return steps


def bounded_unknown_status_allowed(status):
    """Whether an unknown map classification may be used for a cautious probe.

    Occupied and outside-map statuses are still hard rejects. The intent is only
    to avoid blocking a visible target when online SLAM has not filled a short
    corridor cell yet.
    """
    return str(status) in ('unknown', 'clearance_unknown')


def select_safe_bounded_goal(target_x, target_y, robot_x, robot_y,
                             max_step_m, min_step_m, resolution_m,
                             status_fn, allow_unknown=False,
                             unknown_max_step_m=0.6):
    """Pick the longest safe bounded goal toward the target.

    status_fn(x, y) must return the same status strings as
    occupancy_clearance_status_at_world: 'known_free' means safe, anything else
    is rejected but reported to the caller for logging.
    """
    dx = float(target_x) - float(robot_x)
    dy = float(target_y) - float(robot_y)
    target_range = math.hypot(dx, dy)
    if target_range <= 1e-6:
        return None, 'zero_range'
    last_status = 'not_checked'
    unknown_candidate = None
    for drive_step in bounded_drive_steps(max_step_m, min_step_m, resolution_m):
        scale = min(float(drive_step), target_range) / target_range
        gx = float(robot_x) + dx * scale
        gy = float(robot_y) + dy * scale
        status = str(status_fn(gx, gy))
        last_status = status
        if status == 'known_free':
            yaw = math.atan2(dy, dx)
            return (gx, gy, yaw, float(drive_step), status), status
        if (allow_unknown
                and unknown_candidate is None
                and float(drive_step) <= float(unknown_max_step_m) + 1e-9
                and bounded_unknown_status_allowed(status)):
            yaw = math.atan2(dy, dx)
            unknown_candidate = (gx, gy, yaw, float(drive_step), status)
    if unknown_candidate is not None:
        return unknown_candidate, unknown_candidate[4]
    return None, last_status


def yaw_to_quaternion_zw(yaw):
    """(qz, qw) for a yaw-only rotation (qx = qy = 0)."""
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(qz, qw):
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


def occupancy_value_at_world(data, width, height, resolution,
                             origin_x, origin_y, origin_yaw,
                             world_x, world_y):
    """OccupancyGrid value at a world point, or None when outside the grid."""
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return None
    dx = float(world_x) - float(origin_x)
    dy = float(world_y) - float(origin_y)
    c = math.cos(-float(origin_yaw))
    s = math.sin(-float(origin_yaw))
    mx = c * dx - s * dy
    my = s * dx + c * dy
    ix = int(math.floor(mx / float(resolution)))
    iy = int(math.floor(my / float(resolution)))
    if ix < 0 or iy < 0 or ix >= int(width) or iy >= int(height):
        return None
    idx = iy * int(width) + ix
    if idx < 0 or idx >= len(data):
        return None
    return int(data[idx])


def occupancy_known_free_at_world(data, width, height, resolution,
                                  origin_x, origin_y, origin_yaw,
                                  world_x, world_y, occupied_threshold=65):
    """True only for known free OccupancyGrid cells.

    ROS convention: -1 is unknown, 0 is free, values >= occupied_threshold are
    occupied. Unknown/outside is deliberately not treated as free for direct
    long visual approaches.
    """
    value = occupancy_value_at_world(
        data, width, height, resolution,
        origin_x, origin_y, origin_yaw,
        world_x, world_y)
    return value is not None and 0 <= value < int(occupied_threshold)


def occupancy_clearance_status_at_world(data, width, height, resolution,
                                        origin_x, origin_y, origin_yaw,
                                        world_x, world_y, radius_m,
                                        occupied_threshold=65):
    """Return whether a circular neighborhood around a point is known free.

    The raw SLAM map can mark a single goal cell as free while Nav2's inflated
    costmap still rejects the pose. Requiring a small known-free radius prevents
    long direct visual approaches to standoff poses that are too close to
    obstacles, unknown space, or the object itself.
    """
    radius_m = max(0.0, float(radius_m))
    center = occupancy_value_at_world(
        data, width, height, resolution,
        origin_x, origin_y, origin_yaw,
        world_x, world_y)
    if center is None:
        return False, 'outside_map'
    if center < 0:
        return False, 'unknown'
    if center >= int(occupied_threshold):
        return False, 'occupied_%d' % center
    if radius_m <= 0.0:
        return True, 'known_free'

    if width <= 0 or height <= 0 or resolution <= 0.0:
        return False, 'outside_map'
    steps = max(1, int(math.ceil(radius_m / float(resolution))))
    for ix in range(-steps, steps + 1):
        for iy in range(-steps, steps + 1):
            ox = ix * float(resolution)
            oy = iy * float(resolution)
            if math.hypot(ox, oy) > radius_m:
                continue
            value = occupancy_value_at_world(
                data, width, height, resolution,
                origin_x, origin_y, origin_yaw,
                world_x + ox, world_y + oy)
            if value is None:
                return False, 'outside_map'
            if value < 0:
                return False, 'unknown'
            if value >= int(occupied_threshold):
                return False, 'occupied_%d' % value
    return True, 'known_free'


def goal_update_needed(new_x, new_y, new_yaw, last_x, last_y, last_yaw,
                       min_dist, min_angle):
    """Whether a new goal differs enough from the last to be worth republishing
    (anti-jitter): True if the translation OR yaw delta exceeds its threshold."""
    dist = math.hypot(new_x - last_x, new_y - last_y)
    yaw_delta = abs(math.atan2(math.sin(new_yaw - last_yaw), math.cos(new_yaw - last_yaw)))
    return dist >= min_dist or yaw_delta >= min_angle


def pixel_age_s(now_ns, stamp_ns):
    """Age in seconds of a detection stamped at stamp_ns relative to now_ns."""
    return (now_ns - stamp_ns) / 1e9


class DepthRingBuffer:
    """Stamp-indexed ring buffer of depth frames, matched to a target stamp within
    a tolerance. ROS-free: frames are opaque payloads keyed by integer ns stamps."""

    def __init__(self, maxlen=30):
        self._frames = deque(maxlen=maxlen)

    def append(self, stamp_ns, frame):
        self._frames.append((int(stamp_ns), frame))

    def match(self, target_stamp_ns, tolerance_s):
        """Closest frame within tolerance_s of target_stamp_ns, else None."""
        if not self._frames:
            return None
        stamp_ns, frame = min(self._frames, key=lambda f: abs(f[0] - target_stamp_ns))
        if abs(stamp_ns - target_stamp_ns) / 1e9 > tolerance_s:
            return None
        return frame

    def __len__(self):
        return len(self._frames)
