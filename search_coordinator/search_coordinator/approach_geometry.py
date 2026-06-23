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


def yaw_to_quaternion_zw(yaw):
    """(qz, qw) for a yaw-only rotation (qx = qy = 0)."""
    return (math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def quaternion_to_yaw(qz, qw):
    return math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)


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
