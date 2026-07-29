"""Skill action servers for the Pi executive (ROADMAP 2.4).

Five preemptable, feedback-carrying, UUID-idempotent skill servers, all owned by
the one `search_coordinator` node (MultiThreadedExecutor; each server on its own
ReentrantCallbackGroup so the in-process FSM->skill loopback cannot deadlock):

  ExploreFrontier  GoToPose  ApproachDetection  GetObservation  Stop

Common contract (FMEA 2.4/2.5):
  * epoch gate on acceptance — reject a goal whose mission_epoch != current
    (a zombie from a previous mission). Stop is exempt (a safe stop is always honored).
  * idempotency — a repeated request_id in the same epoch replays the cached
    terminal result instead of re-executing (RequestDedup, per server).
  * preemption — cancel is always accepted; the drive loop also aborts if the
    epoch changes mid-flight.
  * the executive NEVER publishes cmd_vel — motion is Nav2 navigate_to_pose only.

Branchy decisions live in skill_logic.py / approach_geometry.py (pure, unit-tested);
these classes are the thin ROS glue.
"""
import math
import time

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import CameraInfo, CompressedImage
from std_msgs.msg import Empty
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, \
    LookupException, TransformListener
from tf2_geometry_msgs import do_transform_point

from ar_project_msgs.action import (
    ApproachDetection,
    ExploreFrontier,
    GetObservation,
    GoToPose,
    Stop,
)
from ar_project_msgs.msg import FrontierArray
from object_tracking_msgs.action import DetectTarget

from fleet_comms.qos import control_cmd_latched, detection_stream_nodeadline, media_besteffort
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from search_coordinator import approach_geometry as ag
from search_coordinator.mission_state import RequestDedup
from search_coordinator.skill_logic import (
    approach_not_reached_outcome,
    explore_goal_xy,
    is_fresh,
    nav_succeeded,
    resolve_frontier,
    should_blacklist_frontier,
)


def _latched_qos():
    q = QoSProfile(depth=1)
    q.history = HistoryPolicy.KEEP_LAST
    q.reliability = ReliabilityPolicy.RELIABLE
    q.durability = DurabilityPolicy.TRANSIENT_LOCAL
    return q


def _apply_terminal(goal_handle, terminal):
    if terminal == 'succeed':
        goal_handle.succeed()
    elif terminal == 'canceled':
        goal_handle.canceled()
    else:
        goal_handle.abort()


class Nav2Driver:
    """Shared, non-blocking wrapper around Nav2 navigate_to_pose.

    `drive()` runs inside a skill's execute_callback (its own ReentrantCallbackGroup
    thread): it sends the pose, then polls the result future with short sleeps —
    yielding to the executor — while publishing feedback and honoring cancel + a
    mid-flight epoch change. The executive never sends cmd_vel directly."""

    def __init__(self, node, action_name='/navigate_to_pose'):
        self._node = node
        self.cb_group = ReentrantCallbackGroup()
        self._client = ActionClient(node, NavigateToPose, action_name,
                                    callback_group=self.cb_group)
        self._distance_remaining = float('nan')
        self._active = None        # current in-flight Nav2 goal handle

    def server_available(self, timeout_s=2.0):
        return self._client.wait_for_server(timeout_sec=timeout_s)

    def cancel_all(self):
        """Cancel the in-flight Nav2 goal, if any (used by Stop)."""
        handle = self._active
        if handle is not None:
            handle.cancel_goal_async()

    def _on_feedback(self, msg):
        try:
            self._distance_remaining = msg.feedback.distance_remaining
        except AttributeError:
            self._distance_remaining = float('nan')

    def drive(self, goal_handle, pose, epoch, mission_state, on_tick):
        """Returns (terminal, reached_pose):
        terminal in {'reached','canceled','zombie','no_server','rejected','failed'}.
        """
        if not self._client.server_is_ready() and not self.server_available(2.0):
            return 'no_server', None
        self._distance_remaining = float('nan')
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = pose
        send_future = self._client.send_goal_async(nav_goal,
                                                   feedback_callback=self._on_feedback)
        while not send_future.done():
            if goal_handle.is_cancel_requested:
                return 'canceled', None
            time.sleep(0.05)
        nav_handle = send_future.result()
        if nav_handle is None or not nav_handle.accepted:
            return 'rejected', None
        self._active = nav_handle
        try:
            result_future = nav_handle.get_result_async()
            while not result_future.done():
                if goal_handle.is_cancel_requested:
                    nav_handle.cancel_goal_async()
                    return 'canceled', None
                if not mission_state.is_current(epoch):
                    nav_handle.cancel_goal_async()
                    return 'zombie', None
                on_tick(self._distance_remaining)
                time.sleep(0.1)
            status = result_future.result().status
        finally:
            self._active = None
        if nav_succeeded(status):
            return 'reached', pose
        return 'failed', None


class SkillServer:
    action_type = None
    action_name = ''
    epoch_gated = True

    def __init__(self, node, mission_state, nav_driver):
        self.node = node
        self.ms = mission_state
        self.nav = nav_driver
        self.dedup = RequestDedup()
        self.cb_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            node, self.action_type, self.action_name,
            execute_callback=self._execute,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=self.cb_group)

    def _goal_cb(self, goal_request):
        if self.epoch_gated and not self.ms.is_current(goal_request.mission_epoch):
            self.node.get_logger().warning(
                '%s: reject zombie epoch %d (current %d)'
                % (self.action_name, goal_request.mission_epoch, self.ms.current_epoch()))
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute(self, goal_handle):
        goal = goal_handle.request
        rid = getattr(goal, 'request_id', '')
        epoch = getattr(goal, 'mission_epoch', 0)
        cached = self.dedup.cached_result(rid, epoch)
        if cached is not None:
            terminal, result = cached
            self.node.get_logger().info('%s: idempotent replay of %s'
                                        % (self.action_name, rid))
        else:
            terminal, result = self._run(goal_handle)
            self.dedup.remember(rid, epoch, (terminal, result))
        _apply_terminal(goal_handle, terminal)
        return result

    def _run(self, goal_handle):
        raise NotImplementedError


class GoToPoseServer(SkillServer):
    action_type = GoToPose
    action_name = 'go_to_pose'

    def _run(self, goal_handle):
        goal = goal_handle.request
        result = GoToPose.Result()

        def tick(dist):
            fb = GoToPose.Feedback()
            fb.distance_remaining = 0.0 if math.isnan(dist) else float(dist)
            fb.stamp = self.node.get_clock().now().to_msg()
            goal_handle.publish_feedback(fb)

        terminal, reached = self.nav.drive(goal_handle, goal.target_pose,
                                           goal.mission_epoch, self.ms, tick)
        if terminal == 'reached':
            result.outcome = GoToPose.Result.SUCCEEDED
            result.reached_pose = reached
            return 'succeed', result
        if terminal == 'canceled':
            result.outcome = GoToPose.Result.PREEMPTED
            return 'canceled', result
        if terminal == 'zombie':
            result.outcome = GoToPose.Result.PREEMPTED
            return 'abort', result
        result.outcome = GoToPose.Result.ABORTED
        return 'abort', result


class ExploreFrontierServer(SkillServer):
    action_type = ExploreFrontier
    action_name = 'explore_frontier'

    def __init__(self, node, mission_state, nav_driver):
        super().__init__(node, mission_state, nav_driver)
        self._frontiers = None
        self._failed_ids = set()       # frontier ids Nav2 failed to reach this mission
        self._fail_epoch = None        # epoch the blacklist belongs to (cleared on new mission)
        self._sub_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        node.declare_parameter('explore_map_frame', 'map')
        node.declare_parameter('explore_robot_frame', 'base_link')
        # standoff 0.0 => aim at the (free) frontier centroid, not into unknown:
        # a goal in unknown space makes the global planner abort and Nav2 burns
        # seconds on recovery behaviours. min_drive 0.25 (> xy_goal_tolerance 0.10)
        # guarantees real motion without a degenerate instant 'reached'.
        node.declare_parameter('explore_standoff_m', 0.0)
        node.declare_parameter('explore_min_drive_m', 0.25)
        # How long to wait for Nav2 navigate_to_pose to be ready before a drive.
        # Absorbs Nav2 lifecycle activation at mission start so a not-yet-up server
        # is NOT mistaken for an unreachable frontier (no false blacklisting).
        node.declare_parameter('explore_nav_ready_timeout_s', 10.0)
        node.create_subscription(FrontierArray, '/frontiers', self._on_frontiers,
                                 _latched_qos(), callback_group=self._sub_group)

    def _on_frontiers(self, msg):
        self._frontiers = msg

    def _run(self, goal_handle):
        goal = goal_handle.request
        result = ExploreFrontier.Result()
        if goal.mission_epoch != self._fail_epoch:    # new mission -> fresh blacklist
            self._failed_ids = set()
            self._fail_epoch = goal.mission_epoch
        snap = self._frontiers
        frontiers = list(snap.frontiers) if snap is not None else []
        # Skip ONLY a frontier the robot is essentially standing on (distance≈0):
        # there explore_goal_xy can't derive a heading, returns the robot's own cell,
        # and Nav2 'reaches' instantly with no motion. A merely-close frontier (e.g.
        # 0.1 m) is fine -- explore_goal_xy clamps the goal to explore_min_drive_m so
        # the robot still travels ~0.25 m. (A 0.25 m filter was too aggressive: it left
        # a dead window when the nearest frontier was the boundary and the rest exceeded
        # the orchestrator's max_travel cap -> NO_FRONTIER, robot frozen.)
        DEGENERATE_FRONTIER_M = 0.05
        sel, reason = resolve_frontier(frontiers, goal.frontier_id, goal.max_travel_m,
                                       exclude_ids=self._failed_ids,
                                       min_travel_m=DEGENERATE_FRONTIER_M)
        if sel is None and reason in ('NO_MATCH', 'TOO_NEAR') and goal.frontier_id >= 0:
            # A VLM-chosen frontier id can go stale between plan and dispatch (the
            # frontier_extractor regenerates ids each map update), which otherwise
            # stalls the mission on NO_FRONTIER. Honor the "explore" intent: fall
            # back to the best current non-degenerate frontier.
            sel, reason = resolve_frontier(frontiers, -1, goal.max_travel_m,
                                           exclude_ids=self._failed_ids,
                                           min_travel_m=DEGENERATE_FRONTIER_M)
            if sel is not None:
                self.node.get_logger().info(
                    'explore_frontier: requested frontier id=%d is stale; fell back to best id=%d'
                    % (goal.frontier_id, sel.id))
        if sel is None:
            self.node.get_logger().info('explore_frontier: NO_FRONTIER (%s, %d blacklisted)'
                                        % (reason, len(self._failed_ids)))
            result.outcome = ExploreFrontier.Result.NO_FRONTIER
            return 'abort', result

        map_frame = (snap.header.frame_id or
                     self.node.get_parameter('explore_map_frame').value)
        robot_frame = self.node.get_parameter('explore_robot_frame').value
        cx, cy = float(sel.centroid.x), float(sel.centroid.y)
        # Default to the raw centroid; project past it once we know where we are.
        gx, gy, yaw = cx, cy, 0.0
        try:
            tf_rob = self.tf_buffer.lookup_transform(map_frame, robot_frame,
                                                     rclpy.time.Time())
            rx = tf_rob.transform.translation.x
            ry = tf_rob.transform.translation.y
            gx, gy, yaw = explore_goal_xy(
                (rx, ry), (cx, cy),
                self.node.get_parameter('explore_min_drive_m').value,
                self.node.get_parameter('explore_standoff_m').value)
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.node.get_logger().warn(
                'explore_frontier: no %s->%s TF; driving to raw centroid'
                % (map_frame, robot_frame))

        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position = Point(x=gx, y=gy, z=0.0)
        qz, qw = ag.yaw_to_quaternion_zw(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.node.get_logger().info(
            'explore_frontier: drive to frontier id=%d centroid=(%.2f,%.2f) '
            'goal=(%.2f,%.2f) score=%.1f dist=%.2f frame=%s (of %d frontiers)'
            % (sel.id, cx, cy, gx, gy, sel.score, sel.distance_m,
               map_frame, len(frontiers)))

        def tick(dist):
            fb = ExploreFrontier.Feedback()
            fb.distance_remaining = 0.0 if math.isnan(dist) else float(dist)
            fb.selected_frontier_id = int(sel.id)
            fb.frontier_score = float(sel.score)
            goal_handle.publish_feedback(fb)

        # Gate on Nav2 readiness: a server still activating is NOT an unreachable
        # frontier. Wait (bounded); if still down, abort transiently WITHOUT
        # blacklisting so the FSM retries and the frontier stays a candidate.
        ready_timeout = float(self.node.get_parameter('explore_nav_ready_timeout_s').value)
        if not self.nav.server_available(ready_timeout):
            self.node.get_logger().warn(
                'explore_frontier: Nav2 navigate_to_pose not ready after %.1fs; '
                'retry without blacklisting frontier id=%d' % (ready_timeout, sel.id))
            result.outcome = ExploreFrontier.Result.ABORTED
            return 'abort', result

        terminal, reached = self.nav.drive(goal_handle, pose, goal.mission_epoch,
                                           self.ms, tick)
        self.node.get_logger().info('explore_frontier: nav drive terminal=%s' % terminal)
        if terminal == 'reached':
            result.outcome = ExploreFrontier.Result.SUCCEEDED
            result.reached_pose = reached
            return 'succeed', result
        if terminal == 'canceled':
            result.outcome = ExploreFrontier.Result.PREEMPTED
            return 'canceled', result
        if terminal == 'zombie':
            result.outcome = ExploreFrontier.Result.PREEMPTED
            return 'abort', result
        if should_blacklist_frontier(terminal):
            # genuine nav failure (rejected/failed while driving): blacklist so we
            # try others instead of looping on an unreachable frontier (behind a wall).
            self._failed_ids.add(sel.id)
            self.node.get_logger().info(
                'explore_frontier: frontier id=%d unreachable, blacklisted (%d total)'
                % (sel.id, len(self._failed_ids)))
        else:
            # transient (e.g. no_server): Nav2 not ready -- do NOT blacklist; retry.
            self.node.get_logger().warn(
                'explore_frontier: drive terminal=%s on frontier id=%d is transient; '
                'NOT blacklisting' % (terminal, sel.id))
        result.outcome = ExploreFrontier.Result.ABORTED
        return 'abort', result


class ApproachDetectionServer(SkillServer):
    """Final approach to a detection (FMEA): SUCCEEDED only when Nav2 reached the
    pose I set AND the last pixel was fresh. STALE_DETECTION / LOST_TARGET abort
    rather than drive blindly or declare a false reach. No goal_locked latch."""
    action_type = ApproachDetection
    action_name = 'approach_detection'

    def __init__(self, node, mission_state, nav_driver):
        super().__init__(node, mission_state, nav_driver)
        self._last_pixel = None        # geometry_msgs/PointStamped (x=u,y=v,z=depth_m)
        self._intr = None              # ag.CameraIntrinsics
        self._last_map = None          # nav_msgs/OccupancyGrid from edge RTAB-Map
        self._sub_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        node.declare_parameter('approach_map_frame', 'map')
        node.declare_parameter('approach_map_topic', '/map')
        node.declare_parameter('approach_camera_frame', 'camera_color_optical_frame')
        node.declare_parameter('approach_robot_frame', 'base_link')
        node.declare_parameter('approach_camera_info_topic', '/camera/camera/color/camera_info')
        node.declare_parameter('approach_min_depth_m', 0.1)
        node.declare_parameter('approach_max_depth_m', 8.0)
        # Prefer a direct visual approach when the final standoff goal is already
        # inside known free SLAM space. Otherwise bound the approach to a short
        # segment so online SLAM can grow before the next re-observation.
        node.declare_parameter('approach_direct_if_goal_in_known_free_map', True)
        node.declare_parameter('approach_direct_occupancy_threshold', 65)
        node.declare_parameter('approach_direct_clearance_m', 0.35)
        node.declare_parameter('approach_max_goal_step_m', 1.6)
        # A detector with a minimum range (e.g. a billboard/large object that
        # overflows the camera frame at close range) legitimately loses the target
        # in the final stretch of the approach. If we tracked the target until the
        # robot was already within this distance of the standoff goal, a reach whose
        # detection has since gone stale is accepted as SUCCEEDED rather than a false
        # reach. A target lost while still FAR stays STALE/LOST (likely transient).
        node.declare_parameter('approach_reacquire_dist_m', 0.8)
        node.create_subscription(PointStamped, '/target_pixel', self._on_pixel,
                                 detection_stream_nodeadline(), callback_group=self._sub_group)
        node.create_subscription(CameraInfo,
                                 node.get_parameter('approach_camera_info_topic').value,
                                 self._on_info, media_besteffort(), callback_group=self._sub_group)
        node.create_subscription(OccupancyGrid,
                                 node.get_parameter('approach_map_topic').value,
                                 self._on_map, _latched_qos(),
                                 callback_group=self._sub_group)

    def _on_pixel(self, msg):
        self._last_pixel = msg

    def _on_info(self, msg):
        self._intr = ag.CameraIntrinsics.from_k(msg.k)

    def _on_map(self, msg):
        self._last_map = msg

    def _pixel_age(self, msg):
        now = self.node.get_clock().now().nanoseconds / 1e9
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        return now - stamp

    def _run(self, goal_handle):
        goal = goal_handle.request
        result = ApproachDetection.Result()
        max_age = goal.max_pixel_age_s if goal.max_pixel_age_s > 0 else 1.5

        # Pre-drive gate: classify LOST_TARGET / STALE_DETECTION before moving.
        px = self._last_pixel
        if px is None:
            self.node.get_logger().warn('approach_detection: LOST_TARGET (no /target_pixel received)')
            result.outcome = ApproachDetection.Result.LOST_TARGET
            return 'abort', result
        age = self._pixel_age(px)
        if not is_fresh(age, max_age):
            self.node.get_logger().warn(
                'approach_detection: STALE_DETECTION (pixel age %.2fs > %.2fs)' % (age, max_age))
            result.outcome = ApproachDetection.Result.STALE_DETECTION
            return 'abort', result

        goal_data = self._compute_goal(px, goal.approach_offset)
        if goal_data is None:
            # cannot compute geometry (no intrinsics / TF) — treat as lost, never
            # drive blindly.
            self.node.get_logger().warn(
                'approach_detection: LOST_TARGET (cannot compute goal; intr=%s depth=%.2f)'
                % (self._intr is not None, px.point.z))
            result.outcome = ApproachDetection.Result.LOST_TARGET
            return 'abort', result
        pose, goal_info = goal_data
        result.bounded_step = bool(goal_info.get('limited', False))
        result.final_distance_m = float(goal_info.get('expected_final_distance_m', 0.0))
        if goal_info['limited']:
            limit_note = ' bounded_step=%.2fm target_range=%.2fm reason=%s' % (
                goal_info['drive_step_m'], goal_info['target_range_m'],
                goal_info.get('direct_goal_status', 'limited'))
        elif goal_info.get('direct_goal_status') == 'known_free':
            limit_note = ' direct_goal=known_free target_range=%.2fm' % (
                goal_info['target_range_m'])
        else:
            limit_note = ''
        self.node.get_logger().info(
            'approach_detection: px=(%.0f,%.0f) depth=%.2f age=%.2fs -> goal=(%.2f,%.2f) driving%s'
            % (px.point.x, px.point.y, px.point.z, age,
               pose.pose.position.x, pose.pose.position.y, limit_note))

        # Closest remaining-distance-to-goal at which we still had a FRESH detection.
        # Lets us tell "lost at close range" (legit) from "lost while far" (suspect).
        reacquire_dist = float(self.node.get_parameter('approach_reacquire_dist_m').value)
        tracked = {'min_fresh_dist': float('inf')}

        def tick(dist):
            cur = self._last_pixel
            cage = self._pixel_age(cur) if cur is not None else None
            fresh = is_fresh(cage, max_age)
            if fresh and not math.isnan(dist):
                tracked['min_fresh_dist'] = min(tracked['min_fresh_dist'], float(dist))
            fb = ApproachDetection.Feedback()
            fb.distance_to_target = 0.0 if math.isnan(dist) else float(dist)
            fb.detection_age_s = float(cage) if cage is not None else 1e3
            fb.detection_fresh = fresh
            goal_handle.publish_feedback(fb)

        terminal, reached = self.nav.drive(goal_handle, pose, goal.mission_epoch,
                                           self.ms, tick)
        self.node.get_logger().info('approach_detection: nav terminal=%s' % terminal)
        if terminal == 'reached':
            # FMEA: declare reached if the detection is STILL fresh, OR if we tracked
            # it until we were already near the goal (the detector's min-range blind
            # spot, not a moved/false target). Lost-while-far stays STALE/LOST.
            cur = self._last_pixel
            cage = self._pixel_age(cur) if cur is not None else None
            if is_fresh(cage, max_age):
                self.node.get_logger().info(
                    'approach_detection: SUCCEEDED (reached pose, detection fresh age=%.2fs)'
                    % (cage if cage is not None else -1.0))
                result.outcome = ApproachDetection.Result.SUCCEEDED
                result.final_distance_m = float(goal_info.get('expected_final_distance_m', 0.0))
                result.reached_pose = reached
                return 'succeed', result
            if tracked['min_fresh_dist'] <= reacquire_dist:
                self.node.get_logger().info(
                    'approach_detection: SUCCEEDED (reached pose; target tracked to %.2fm <= '
                    '%.2fm then lost at close range, age=%s)'
                    % (tracked['min_fresh_dist'], reacquire_dist,
                       '%.2fs' % cage if cage is not None else 'none'))
                result.outcome = ApproachDetection.Result.SUCCEEDED
                result.final_distance_m = float(goal_info.get('expected_final_distance_m', 0.0))
                result.reached_pose = reached
                return 'succeed', result
            self.node.get_logger().warn(
                'approach_detection: reached pose but detection NOT fresh (age=%s) and target '
                'was lost while still far (min_fresh_dist=%.2fm) -> %s'
                % ('%.2fs' % cage if cage is not None else 'none',
                   tracked['min_fresh_dist'],
                   'STALE' if cur is not None else 'LOST'))
            result.outcome = approach_not_reached_outcome_code(cur is not None)
            return 'abort', result
        if terminal == 'canceled':
            result.outcome = ApproachDetection.Result.PREEMPTED
            return 'canceled', result
        if terminal == 'zombie':
            result.outcome = ApproachDetection.Result.PREEMPTED
            return 'abort', result
        self.node.get_logger().warn('approach_detection: ABORTED (nav terminal=%s)' % terminal)
        result.outcome = ApproachDetection.Result.ABORTED
        return 'abort', result

    def _compute_goal(self, px, offset):
        if self._intr is None:
            return None
        min_d = self.node.get_parameter('approach_min_depth_m').value
        max_d = self.node.get_parameter('approach_max_depth_m').value
        depth = ag.embedded_depth(px.point.z, min_d, max_d)
        if depth is None:
            return None
        cx, cy, cz = ag.backproject_pixel(px.point.x, px.point.y, depth, self._intr)
        map_frame = self.node.get_parameter('approach_map_frame').value
        cam_frame = px.header.frame_id or self.node.get_parameter('approach_camera_frame').value
        robot_frame = self.node.get_parameter('approach_robot_frame').value
        try:
            tf_cam = self.tf_buffer.lookup_transform(map_frame, cam_frame, rclpy.time.Time())
            tf_rob = self.tf_buffer.lookup_transform(map_frame, robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        # Properly transform the camera-frame point into the map frame.
        cam_pt = PointStamped()
        cam_pt.header.frame_id = cam_frame
        cam_pt.point.x, cam_pt.point.y, cam_pt.point.z = cx, cy, cz
        map_pt = do_transform_point(cam_pt, tf_cam)
        tx, ty = map_pt.point.x, map_pt.point.y
        rx = tf_rob.transform.translation.x
        ry = tf_rob.transform.translation.y
        approach_offset = offset if offset > 0 else 0.58
        dx = tx - rx
        dy = ty - ry
        target_range = math.hypot(dx, dy)
        yaw = math.atan2(dy, dx) if target_range > 1e-6 else 0.0
        full_gx, full_gy, full_yaw = ag.approach_goal(
            tx, ty, rx, ry, approach_offset)
        max_step = float(self.node.get_parameter('approach_max_goal_step_m').value)
        full_drive = math.hypot(full_gx - rx, full_gy - ry)
        wants_limit = max_step > 0.0 and full_drive > max_step
        direct_status = self._direct_goal_status(full_gx, full_gy, map_frame) if wants_limit else ''
        limited = wants_limit and direct_status != 'known_free'
        if limited and target_range > 1e-6:
            scale = max_step / target_range
            gx = rx + dx * scale
            gy = ry + dy * scale
            drive_step = max_step
        else:
            gx, gy, yaw = full_gx, full_gy, full_yaw
            drive_step = math.hypot(gx - rx, gy - ry)
        expected_final_distance = math.hypot(tx - gx, ty - gy)
        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position = Point(x=gx, y=gy, z=0.0)
        qz, qw = ag.yaw_to_quaternion_zw(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose, {
            'limited': limited,
            'target_range_m': target_range,
            'drive_step_m': drive_step,
            'expected_final_distance_m': expected_final_distance,
            'direct_goal_status': direct_status,
        }

    def _direct_goal_status(self, gx, gy, map_frame):
        if not bool(self.node.get_parameter('approach_direct_if_goal_in_known_free_map').value):
            return 'disabled'
        grid = self._last_map
        if grid is None:
            return 'no_map'
        if grid.header.frame_id and grid.header.frame_id != map_frame:
            return 'map_frame_mismatch'
        info = grid.info
        origin = info.origin.position
        yaw = ag.quaternion_to_yaw(info.origin.orientation.z,
                                   info.origin.orientation.w)
        value = ag.occupancy_value_at_world(
            grid.data, info.width, info.height, info.resolution,
            origin.x, origin.y, yaw, gx, gy)
        if value is None:
            return 'outside_map'
        if value < 0:
            return 'unknown'
        threshold = int(self.node.get_parameter(
            'approach_direct_occupancy_threshold').value)
        clearance = float(self.node.get_parameter('approach_direct_clearance_m').value)
        ok, status = ag.occupancy_clearance_status_at_world(
            grid.data, info.width, info.height, info.resolution,
            origin.x, origin.y, yaw, gx, gy, clearance,
            occupied_threshold=threshold)
        return status if ok else 'clearance_%s' % status


def approach_not_reached_outcome_code(have_pixel):
    name = approach_not_reached_outcome(have_pixel)
    return (ApproachDetection.Result.LOST_TARGET if name == 'LOST_TARGET'
            else ApproachDetection.Result.STALE_DETECTION)


class GetObservationServer(SkillServer):
    """Phase 3.5: capture an observation as a compressed frame + Set-of-Mark
    candidates (no PointCloud2 over Wi-Fi). Calls the edge DetectTarget service
    with the active mission instruction as the open-vocab query; the returned
    Candidate[] and annotated Set-of-Mark frame are handed up so the VLM can pick
    a target by mark_id. Degrades gracefully: if the detector is unavailable or
    times out, returns the last raw camera frame with an empty candidate list
    (still SUCCEEDED) rather than blocking the mission."""
    action_type = GetObservation
    action_name = 'get_observation'

    def __init__(self, node, mission_state, nav_driver,
                 image_topic='/tracker/color/image/compressed',
                 detect_action_name='detect_target', detect_timeout_s=5.0,
                 query_default='object', map_frame='map', robot_frame='base_link'):
        super().__init__(node, mission_state, nav_driver)
        self._last_image = None
        self._detect_timeout_s = float(detect_timeout_s)
        self._query_default = query_default
        self._map_frame = map_frame
        self._robot_frame = robot_frame
        self._sub_group = ReentrantCallbackGroup()
        q = QoSProfile(depth=1)
        q.history = HistoryPolicy.KEEP_LAST
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        q.durability = DurabilityPolicy.VOLATILE
        node.create_subscription(CompressedImage, image_topic, self._on_image, q,
                                 callback_group=self._sub_group)
        self._detect = ActionClient(node, DetectTarget, detect_action_name,
                                    callback_group=ReentrantCallbackGroup())
        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, node)

    def _on_image(self, msg):
        self._last_image = msg

    def _run(self, goal_handle):
        goal = goal_handle.request
        result = GetObservation.Result()
        result.observed_from = self._observed_from()

        fb = GetObservation.Feedback()
        fb.phase = 'CAPTURING'
        goal_handle.publish_feedback(fb)

        detect = self._call_detect(goal)
        if detect is not None and detect.outcome != DetectTarget.Result.ABORTED:
            fb.phase = 'RENDERING'
            goal_handle.publish_feedback(fb)
            result.candidates = list(detect.candidates)
            # prefer the annotated Set-of-Mark frame; fall back to the raw frame
            if goal.with_setofmark and detect.annotated.data:
                result.view = detect.annotated
            elif self._last_image is not None:
                result.view = self._last_image
        elif self._last_image is not None:
            result.view = self._last_image      # detector down -> image-only, no candidates

        result.outcome = GetObservation.Result.SUCCEEDED
        return 'succeed', result

    def _observed_from(self):
        pose = PoseStamped()
        pose.header.frame_id = self._map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        try:
            tf = self._tf.lookup_transform(self._map_frame, self._robot_frame,
                                           rclpy.time.Time())
            pose.pose.position.x = tf.transform.translation.x
            pose.pose.position.y = tf.transform.translation.y
            pose.pose.position.z = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass
        return pose

    def _call_detect(self, goal):
        """Send a DetectTarget goal and block-poll its result (bounded). Returns the
        DetectTarget.Result, or None if the server is absent / times out."""
        if not self._detect.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warning('get_observation: detect_target server absent')
            return None
        dg = DetectTarget.Goal()
        dg.request_id = getattr(goal, 'request_id', '')
        dg.mission_epoch = getattr(goal, 'mission_epoch', 0)
        dg.query = self.ms.instruction or self._query_default
        dg.render_setofmark = bool(goal.with_setofmark)
        dg.conf_threshold = 0.0                 # detector uses its configured default
        send_future = self._detect.send_goal_async(dg)
        deadline = time.time() + self._detect_timeout_s
        while not send_future.done():
            if time.time() > deadline:
                return None
            time.sleep(0.02)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        while not result_future.done():
            if time.time() > deadline:
                handle.cancel_goal_async()
                return None
            time.sleep(0.02)
        return result_future.result().result


class StopServer(SkillServer):
    """Idempotent safe stop. Always accepted regardless of epoch (a stop is never
    a zombie). SOFT_STOP cancels Nav2 — the actual zeroing comes from the
    velocity_smoother input-timeout + watchdog, NOT a decel ramp (must-fix #4).
    HOLD cancels + latches. QUICK_STOP_REQUEST also fires the external hardware
    quick-stop trigger (closes ROADMAP 0.7)."""
    action_type = Stop
    action_name = 'stop'
    epoch_gated = False

    def __init__(self, node, mission_state, nav_driver):
        super().__init__(node, mission_state, nav_driver)
        self.hold_latched = False
        self._speed = None
        self._sub_group = ReentrantCallbackGroup()
        node.declare_parameter('zero_velocity_eps', 0.02)
        node.declare_parameter('quick_stop_topic', '/quick_stop_trigger')
        node.create_subscription(Odometry, '/odometry/filtered', self._on_odom,
                                 10, callback_group=self._sub_group)
        self._qs_pub = node.create_publisher(
            Empty, node.get_parameter('quick_stop_topic').value, control_cmd_latched())

    def _on_odom(self, msg):
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self._speed = math.sqrt(v.x * v.x + v.y * v.y) + abs(w.z)

    def _run(self, goal_handle):
        goal = goal_handle.request
        result = Stop.Result()
        if goal.mode == Stop.Goal.QUICK_STOP_REQUEST:
            self._qs_pub.publish(Empty())
            self.node.get_logger().warning('stop: QUICK_STOP_REQUEST -> hardware trigger fired')
        self.nav.cancel_all()
        if goal.mode == Stop.Goal.HOLD:
            self.hold_latched = True

        eps = self.node.get_parameter('zero_velocity_eps').value
        deadline = time.time() + 5.0
        confirmed = False
        while time.time() < deadline:
            fb = Stop.Feedback()
            confirmed = self._speed is not None and self._speed < eps
            fb.zero_velocity_confirmed = confirmed
            goal_handle.publish_feedback(fb)
            if confirmed:
                break
            if goal_handle.is_cancel_requested:
                break
            time.sleep(0.1)
        result.outcome = Stop.Result.SUCCEEDED
        return 'succeed', result


def build_all_skills(node, mission_state, nav_driver):
    """Instantiate the five skill servers on the node. Returns a dict by name."""
    return {
        'go_to_pose': GoToPoseServer(node, mission_state, nav_driver),
        'explore_frontier': ExploreFrontierServer(node, mission_state, nav_driver),
        'approach_detection': ApproachDetectionServer(node, mission_state, nav_driver),
        'get_observation': GetObservationServer(node, mission_state, nav_driver),
        'stop': StopServer(node, mission_state, nav_driver),
    }
