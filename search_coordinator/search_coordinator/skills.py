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
from nav_msgs.msg import Odometry
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

from fleet_comms.qos import control_cmd_latched, detection_stream_nodeadline
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from search_coordinator import approach_geometry as ag
from search_coordinator.mission_state import RequestDedup
from search_coordinator.skill_logic import (
    approach_not_reached_outcome,
    is_fresh,
    nav_succeeded,
    resolve_frontier,
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
        self._sub_group = ReentrantCallbackGroup()
        node.create_subscription(FrontierArray, '/frontiers', self._on_frontiers,
                                 _latched_qos(), callback_group=self._sub_group)

    def _on_frontiers(self, msg):
        self._frontiers = msg

    def _run(self, goal_handle):
        goal = goal_handle.request
        result = ExploreFrontier.Result()
        snap = self._frontiers
        frontiers = list(snap.frontiers) if snap is not None else []
        sel, reason = resolve_frontier(frontiers, goal.frontier_id, goal.max_travel_m)
        if sel is None:
            self.node.get_logger().info('explore_frontier: NO_FRONTIER (%s)' % reason)
            result.outcome = ExploreFrontier.Result.NO_FRONTIER
            return 'abort', result

        pose = PoseStamped()
        pose.header.frame_id = (snap.header.frame_id or 'map')
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position = Point(x=float(sel.centroid.x), y=float(sel.centroid.y), z=0.0)
        pose.pose.orientation.w = 1.0
        self.node.get_logger().info(
            'explore_frontier: drive to frontier id=%d centroid=(%.2f,%.2f) score=%.1f '
            'dist=%.2f frame=%s (of %d frontiers)'
            % (sel.id, sel.centroid.x, sel.centroid.y, sel.score, sel.distance_m,
               pose.header.frame_id, len(frontiers)))

        def tick(dist):
            fb = ExploreFrontier.Feedback()
            fb.distance_remaining = 0.0 if math.isnan(dist) else float(dist)
            fb.selected_frontier_id = int(sel.id)
            fb.frontier_score = float(sel.score)
            goal_handle.publish_feedback(fb)

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
        self._sub_group = ReentrantCallbackGroup()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        node.declare_parameter('approach_map_frame', 'map')
        node.declare_parameter('approach_camera_frame', 'camera_link_optical')
        node.declare_parameter('approach_robot_frame', 'base_link')
        node.declare_parameter('approach_camera_info_topic', '/camera/camera_info')
        node.declare_parameter('approach_min_depth_m', 0.1)
        node.declare_parameter('approach_max_depth_m', 8.0)
        node.create_subscription(PointStamped, '/target_pixel', self._on_pixel,
                                 detection_stream_nodeadline(), callback_group=self._sub_group)
        node.create_subscription(CameraInfo,
                                 node.get_parameter('approach_camera_info_topic').value,
                                 self._on_info, 1, callback_group=self._sub_group)

    def _on_pixel(self, msg):
        self._last_pixel = msg

    def _on_info(self, msg):
        self._intr = ag.CameraIntrinsics.from_k(msg.k)

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
            result.outcome = ApproachDetection.Result.LOST_TARGET
            return 'abort', result
        age = self._pixel_age(px)
        if not is_fresh(age, max_age):
            result.outcome = ApproachDetection.Result.STALE_DETECTION
            return 'abort', result

        pose = self._compute_goal(px, goal.approach_offset)
        if pose is None:
            # cannot compute geometry (no intrinsics / TF) — treat as lost, never
            # drive blindly.
            result.outcome = ApproachDetection.Result.LOST_TARGET
            return 'abort', result

        def tick(dist):
            cur = self._last_pixel
            cage = self._pixel_age(cur) if cur is not None else None
            fb = ApproachDetection.Feedback()
            fb.distance_to_target = 0.0 if math.isnan(dist) else float(dist)
            fb.detection_age_s = float(cage) if cage is not None else 1e3
            fb.detection_fresh = is_fresh(cage, max_age)
            goal_handle.publish_feedback(fb)

        terminal, reached = self.nav.drive(goal_handle, pose, goal.mission_epoch,
                                           self.ms, tick)
        if terminal == 'reached':
            # FMEA: only declare reached if the detection is STILL fresh.
            cur = self._last_pixel
            cage = self._pixel_age(cur) if cur is not None else None
            if is_fresh(cage, max_age):
                result.outcome = ApproachDetection.Result.SUCCEEDED
                result.reached_pose = reached
                return 'succeed', result
            result.outcome = approach_not_reached_outcome_code(cur is not None)
            return 'abort', result
        if terminal == 'canceled':
            result.outcome = ApproachDetection.Result.PREEMPTED
            return 'canceled', result
        if terminal == 'zombie':
            result.outcome = ApproachDetection.Result.PREEMPTED
            return 'abort', result
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
        gx, gy, yaw = ag.approach_goal(tx, ty, rx, ry, offset if offset > 0 else 0.58)
        pose = PoseStamped()
        pose.header.frame_id = map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position = Point(x=gx, y=gy, z=0.0)
        qz, qw = ag.yaw_to_quaternion_zw(yaw)
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose


def approach_not_reached_outcome_code(have_pixel):
    name = approach_not_reached_outcome(have_pixel)
    return (ApproachDetection.Result.LOST_TARGET if name == 'LOST_TARGET'
            else ApproachDetection.Result.STALE_DETECTION)


class GetObservationServer(SkillServer):
    """Phase-2 functional stub: capture one CompressedImage from a real source if
    one exists (else leave view empty — must-fix #3); candidates empty (detector
    is Phase 3)."""
    action_type = GetObservation
    action_name = 'get_observation'

    def __init__(self, node, mission_state, nav_driver, image_topic='/tracker/color/image/compressed'):
        super().__init__(node, mission_state, nav_driver)
        self._last_image = None
        self._sub_group = ReentrantCallbackGroup()
        q = QoSProfile(depth=1)
        q.history = HistoryPolicy.KEEP_LAST
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        q.durability = DurabilityPolicy.VOLATILE
        node.create_subscription(CompressedImage, image_topic, self._on_image, q,
                                 callback_group=self._sub_group)

    def _on_image(self, msg):
        self._last_image = msg

    def _run(self, goal_handle):
        result = GetObservation.Result()
        fb = GetObservation.Feedback()
        fb.phase = 'CAPTURING'
        goal_handle.publish_feedback(fb)
        if self._last_image is not None:
            result.view = self._last_image
        # candidates stay empty (detector is Phase 3)
        result.outcome = GetObservation.Result.SUCCEEDED
        return 'succeed', result


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
