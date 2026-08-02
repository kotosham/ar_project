"""SeekObject mission entry server + FSM orchestration (ROADMAP 2.2).

The single mission entry point. Owns the FSM and drives the skill servers via
in-process (loopback) action clients on its own ReentrantCallbackGroup. The
executive NEVER publishes cmd_vel — motion is always through a skill -> Nav2.

Mission-epoch supersession (FMEA 2.5): a new SeekObject goal calls
mission_state.start_mission() which bumps the epoch. Every running loop checks
is_current(my_epoch); the superseded mission cancels its in-flight skill and
finalizes PREEMPTED, the skill servers reject the now-zombie epoch, and the new
mission re-arms at SEARCH. The pure transition logic lives in executive_fsm.
"""
import json
import math
import threading
import time
import uuid

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import String
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, \
    LookupException, TransformListener

from object_tracking_msgs.action import SeekObject

from ar_project_msgs.action import ApproachDetection, ExploreFrontier, GoToPose, Stop

from fleet_comms.qos import control_cmd_latched, detection_stream_nodeadline

from search_coordinator import executive_fsm as fsm
from search_coordinator.executive_fsm import EVENT, STATE


def _norm_target_text(text):
    return ' '.join(str(text or '').strip().lower().split())


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _quat_to_yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _yaw_to_quaternion_zw(yaw):
    half = 0.5 * float(yaw)
    return math.sin(half), math.cos(half)


def flat_initial_scan_turns(right_rad=1.57, left_rad=3.14):
    """Relative yaw steps for the non-semantic FLAT overview scan.

    The forward view is observed before these turns. We go right first, then
    split the left sweep into two 90-degree-ish turns to avoid a single pi-radian
    goal, where Nav2 may choose either physical rotation direction.
    """
    turns = []
    right = abs(float(right_rad))
    left = abs(float(left_rad))
    if right > 1e-3:
        turns.append(('right', -right))
    left_step = left / 2.0
    if left_step > 1e-3:
        turns.append(('forward', left_step))
        turns.append(('left', left_step))
    return turns


def vlm_activity_matches_instruction(payload, instruction):
    """True when a /vlm/activity JSON event belongs to this SeekObject request."""
    want = _norm_target_text(instruction)
    if not want or not isinstance(payload, dict):
        return False
    for key in ('raw_query', 'target', 'canonical_target', 'detection_query'):
        if _norm_target_text(payload.get(key)) == want:
            return True
    return False


def vlm_activity_stamp(payload):
    try:
        return float(payload.get('stamp'))
    except (TypeError, ValueError, AttributeError):
        return 0.0


class SeekObjectServer:
    def __init__(self, node, mission_state, prompt_bridge, sync_epoch_cb,
                 pixel_fresh_s=1.5):
        self.node = node
        self.ms = mission_state
        self.prompt = prompt_bridge
        self.sync_epoch = sync_epoch_cb
        node.declare_parameter('flat_target_pixel_max_age_s', 4.0)
        self._pixel_fresh_s = float(
            node.get_parameter('flat_target_pixel_max_age_s').value)
        # When ExploreFrontier reports NO_FRONTIER, wait this long for a fresh
        # detection before declaring failure: the target can be visible at spawn
        # (the tracker streams /target_pixel) with no unexplored frontier left to
        # drive to -- that must become DETECT, not a spurious 'frontiers exhausted'.
        self._no_frontier_detect_wait_s = self._pixel_fresh_s + 1.5
        node.declare_parameter('vlm_handoff_start_timeout_s', 10.0)
        node.declare_parameter('vlm_handoff_result_timeout_s', 0.0)
        node.declare_parameter('flat_initial_scan_enabled', True)
        node.declare_parameter('flat_initial_scan_forward_wait_s', 4.0)
        node.declare_parameter('flat_initial_scan_settle_s', 2.0)
        node.declare_parameter('flat_initial_scan_view_detect_wait_s', 4.0)
        node.declare_parameter('flat_initial_scan_right_rad', 1.57)
        node.declare_parameter('flat_initial_scan_left_rad', 3.14)
        node.declare_parameter('flat_initial_scan_frame', 'odom')
        node.declare_parameter('flat_initial_scan_robot_frame', 'base_link')
        self._vlm_handoff_start_timeout_s = float(
            node.get_parameter('vlm_handoff_start_timeout_s').value)
        # 0 means wait until /vlm/activity mission_end, cancel, or supersession.
        self._vlm_handoff_result_timeout_s = float(
            node.get_parameter('vlm_handoff_result_timeout_s').value)

        self._last_pixel = None
        self._last_pixel_recv_ns = 0
        self._search_start_ns = 0
        self._flat_scan_done_epoch = None
        self._vlm_events = []
        self._vlm_events_lock = threading.Lock()

        self._client_group = ReentrantCallbackGroup()
        self._explore = ActionClient(node, ExploreFrontier, 'explore_frontier',
                                     callback_group=self._client_group)
        self._goto = ActionClient(node, GoToPose, 'go_to_pose',
                                  callback_group=self._client_group)
        self._approach = ActionClient(node, ApproachDetection, 'approach_detection',
                                      callback_group=self._client_group)
        self._stop = ActionClient(node, Stop, 'stop', callback_group=self._client_group)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        self._sub_group = ReentrantCallbackGroup()
        node.create_subscription(PointStamped, '/target_pixel', self._on_pixel,
                                 detection_stream_nodeadline(), callback_group=self._sub_group)
        self._vlm_mission_pub = node.create_publisher(String, '/vlm_mission', 1)
        node.create_subscription(String, '/vlm/activity', self._on_vlm_activity, 10,
                                 callback_group=self._sub_group)

        self._srv_group = ReentrantCallbackGroup()
        self._server = ActionServer(
            node, SeekObject, 'seek_object',
            execute_callback=self._execute,
            goal_callback=lambda g: GoalResponse.ACCEPT,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
            callback_group=self._srv_group)

        # Latched side-channel broadcast of the FSM state. Action feedback only
        # reaches the goal OWNER; this lets any monitor (mission dashboard,
        # ros2 topic echo) see state/subtask/progress live and after a late join.
        self._status_pub = node.create_publisher(String, '/mission/status',
                                                 control_cmd_latched())
        self._broadcast_status(STATE.IDLE, epoch=0, instruction='')

    # -- pixel tracking -------------------------------------------------------

    def _on_pixel(self, msg):
        self._last_pixel = msg
        self._last_pixel_recv_ns = self.node.get_clock().now().nanoseconds

    def _on_vlm_activity(self, msg):
        try:
            payload = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            return
        if not isinstance(payload, dict):
            return
        with self._vlm_events_lock:
            self._vlm_events.append(payload)
            self._vlm_events = self._vlm_events[-300:]

    def _fresh_pixel_event(self, min_recv_ns=0):
        """EVENT.DETECTED if a fresh pixel arrived since SEARCH started, else None."""
        if self._last_pixel is None or self._last_pixel_recv_ns <= self._search_start_ns:
            return None
        if min_recv_ns and self._last_pixel_recv_ns <= int(min_recv_ns):
            return None
        now_ns = self.node.get_clock().now().nanoseconds
        stamp_s = (
            float(self._last_pixel.header.stamp.sec)
            + float(self._last_pixel.header.stamp.nanosec) / 1e9
        )
        if stamp_s > 0.0:
            age = now_ns / 1e9 - stamp_s
        else:
            age = (now_ns - self._last_pixel_recv_ns) / 1e9
        return EVENT.DETECTED if age <= self._pixel_fresh_s else None

    def _await_detection(self, my_epoch, parent, timeout_s, min_recv_ns=0):
        """Poll for a fresh /target_pixel for up to timeout_s. Returns EVENT.DETECTED
        or None. Honors parent cancel + epoch supersession. Used to catch a target
        that is already in view when there is no frontier left to explore."""
        deadline = self.node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.node.get_clock().now().nanoseconds < deadline:
            if self._should_abort(parent, my_epoch):
                return None
            ev = self._fresh_pixel_event(min_recv_ns=min_recv_ns)
            if ev is not None:
                return ev
            time.sleep(0.1)
        return None

    def _sleep_or_detect(self, my_epoch, parent, timeout_s):
        return self._await_detection(my_epoch, parent, max(0.0, float(timeout_s)))

    def _sleep_ignoring_detections(self, my_epoch, parent, timeout_s):
        deadline = self.node.get_clock().now().nanoseconds + int(max(0.0, float(timeout_s)) * 1e9)
        while self.node.get_clock().now().nanoseconds < deadline:
            if self._should_abort(parent, my_epoch):
                return False
            time.sleep(0.1)
        return True

    # -- helpers --------------------------------------------------------------

    def _new_id(self):
        return str(uuid.uuid4())

    def _vlm_events_since(self, since_wall_s):
        with self._vlm_events_lock:
            return [
                ev for ev in self._vlm_events
                if vlm_activity_stamp(ev) >= float(since_wall_s) - 0.25
            ]

    def _should_abort(self, parent, my_epoch):
        return parent.is_cancel_requested or not self.ms.is_current(my_epoch)

    def _current_pose(self, frame):
        robot_frame = str(self.node.get_parameter(
            'flat_initial_scan_robot_frame').value)
        try:
            tf = self._tf_buffer.lookup_transform(
                frame, robot_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.node.get_logger().warn(
                'flat_initial_scan: no %s->%s TF; skipping scan turn'
                % (frame, robot_frame))
            return None
        t = tf.transform.translation
        return float(t.x), float(t.y), _quat_to_yaw(tf.transform.rotation)

    def _scan_turn_goal(self, delta_yaw_rad, my_epoch):
        frame = str(self.node.get_parameter('flat_initial_scan_frame').value)
        pose = self._current_pose(frame)
        if pose is None:
            return None
        x, y, yaw = pose
        target_yaw = _wrap_angle(yaw + float(delta_yaw_rad))
        ps = PoseStamped()
        ps.header.frame_id = frame
        ps.header.stamp = self.node.get_clock().now().to_msg()
        ps.pose.position.x = x
        ps.pose.position.y = y
        qz, qw = _yaw_to_quaternion_zw(target_yaw)
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        goal = GoToPose.Goal()
        goal.request_id = 'flat_scan:' + self._new_id()
        goal.mission_epoch = my_epoch
        goal.target_pose = ps
        goal.xy_tolerance = 0.15
        goal.yaw_tolerance = 0.25
        return goal

    def _drive_skill(self, client, goal_msg, my_epoch, parent, monitor=None):
        """Dispatch a loopback skill goal; poll until result, honoring parent
        cancel + epoch supersession + an optional monitor() early-exit predicate.
        Returns ('result', result_msg) | ('monitor', event) | ('aborted', None) |
        ('no_server', None) | ('rejected', None)."""
        if not client.wait_for_server(timeout_sec=3.0):
            return ('no_server', None)
        send_future = client.send_goal_async(goal_msg)
        while not send_future.done():
            if self._should_abort(parent, my_epoch):
                return ('aborted', None)
            time.sleep(0.05)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            return ('rejected', None)
        result_future = gh.get_result_async()
        while not result_future.done():
            if self._should_abort(parent, my_epoch):
                gh.cancel_goal_async()
                return ('aborted', None)
            if monitor is not None:
                ev = monitor()
                if ev is not None:
                    gh.cancel_goal_async()
                    return ('monitor', ev)
            time.sleep(0.1)
        return ('result', result_future.result().result)

    def _publish_feedback(self, parent, my_epoch, state, subtask='', instruction=''):
        fb = SeekObject.Feedback()
        fb.state = state
        fb.active_subtask = subtask
        fb.progress = fsm.progress_for(state)
        fb.mission_epoch = my_epoch
        parent.publish_feedback(fb)
        self._broadcast_status(state, epoch=my_epoch, subtask=subtask,
                               instruction=instruction)

    def _broadcast_status(self, state, epoch=0, subtask='', instruction='', outcome=''):
        """Publish the latched human-readable /mission/status JSON snapshot."""
        msg = String()
        msg.data = json.dumps({
            'state': state,
            'active_subtask': subtask,
            'progress': fsm.progress_for(state),
            'mission_epoch': int(epoch),
            'instruction': instruction,
            'outcome': outcome,
            'stamp': self.node.get_clock().now().nanoseconds / 1e9,
        }, ensure_ascii=False)
        self._status_pub.publish(msg)

    # -- state handlers -------------------------------------------------------

    def _do_flat_initial_scan(self, my_epoch, parent):
        if self._flat_scan_done_epoch == my_epoch:
            return None
        if not bool(self.node.get_parameter('flat_initial_scan_enabled').value):
            self._flat_scan_done_epoch = my_epoch
            return None

        forward_wait = float(self.node.get_parameter(
            'flat_initial_scan_forward_wait_s').value)
        ev = self._await_detection(my_epoch, parent, forward_wait)
        if ev == EVENT.DETECTED:
            self._flat_scan_done_epoch = my_epoch
            self.node.get_logger().info(
                'flat_initial_scan: target detected in forward view; skip scan')
            return ev

        right = float(self.node.get_parameter('flat_initial_scan_right_rad').value)
        left = float(self.node.get_parameter('flat_initial_scan_left_rad').value)
        settle = float(self.node.get_parameter('flat_initial_scan_settle_s').value)
        view_detect_wait = float(self.node.get_parameter(
            'flat_initial_scan_view_detect_wait_s').value)
        attempted = False
        for view, delta_yaw in flat_initial_scan_turns(right, left):
            if self._should_abort(parent, my_epoch):
                return 'ABORTED'
            goal = self._scan_turn_goal(delta_yaw, my_epoch)
            if goal is None:
                continue
            attempted = True
            self.ms.commit(fsm.SKILL_GOTO, {
                'flat_initial_scan_view': view,
                'turn_yaw_rad': delta_yaw,
            }, goal.request_id)
            self._publish_feedback(
                parent, my_epoch, STATE.SEARCH, 'FlatInitialScan',
                instruction=self.ms.instruction)
            self.node.get_logger().info(
                'flat_initial_scan: TURN %+.2frad -> %s view'
                % (delta_yaw, view))
            kind, payload = self._drive_skill(self._goto, goal, my_epoch, parent)
            if kind == 'aborted':
                return 'ABORTED'
            if kind in ('no_server', 'rejected'):
                self.node.get_logger().warn(
                    'flat_initial_scan: go_to_pose unavailable (%s); continue to frontiers'
                    % kind)
                return None
            if settle > 0.0:
                self.node.get_logger().info(
                    'flat_initial_scan: settle %.1fs before detecting %s view'
                    % (settle, view))
            if not self._sleep_ignoring_detections(my_epoch, parent, settle):
                return 'ABORTED'
            detect_start_ns = self.node.get_clock().now().nanoseconds
            ev = self._await_detection(
                my_epoch, parent, view_detect_wait, min_recv_ns=detect_start_ns)
            if ev == EVENT.DETECTED:
                self._flat_scan_done_epoch = my_epoch
                return ev
        if attempted:
            self._flat_scan_done_epoch = my_epoch
            self.node.get_logger().info(
                'flat_initial_scan: target not detected after forward/right/left overview; '
                'continue with ExploreFrontier')
        return None

    def _do_search(self, my_epoch, parent):
        ev = self._do_flat_initial_scan(my_epoch, parent)
        if ev is not None:
            return ev

        goal = ExploreFrontier.Goal()
        goal.request_id = self._new_id()
        goal.mission_epoch = my_epoch
        goal.frontier_id = -1
        goal.max_travel_m = 0.0
        self.ms.commit(fsm.SKILL_EXPLORE, {'frontier_id': -1}, goal.request_id)
        kind, payload = self._drive_skill(self._explore, goal, my_epoch, parent,
                                          monitor=self._fresh_pixel_event)
        if kind == 'monitor':
            return payload  # EVENT.DETECTED
        if kind == 'aborted':
            return 'ABORTED'
        if kind in ('no_server', 'rejected'):
            return 'FAIL'
        # kind == 'result'
        if payload.outcome == ExploreFrontier.Result.NO_FRONTIER:
            # No frontier to drive to (e.g. cold-start before the map has grown, or
            # exploration genuinely exhausted). The target may already be visible --
            # give detection a brief window before failing the mission.
            ev = self._await_detection(my_epoch, parent, self._no_frontier_detect_wait_s)
            if ev == EVENT.DETECTED:
                return ev
            return EVENT.NO_FRONTIER
        if payload.outcome == ExploreFrontier.Result.SUCCEEDED:
            # reached a frontier without a detection -> keep exploring
            return 'CONTINUE'
        return 'CONTINUE'  # ABORTED/PREEMPTED frontier drive: try again

    def _do_approach(self, my_epoch, parent, instruction):
        goal = ApproachDetection.Goal()
        goal.request_id = self._new_id()
        goal.mission_epoch = my_epoch
        goal.target_label = instruction
        goal.approach_offset = 0.0      # server default 0.58
        goal.max_pixel_age_s = self._pixel_fresh_s
        self.ms.commit(fsm.SKILL_APPROACH, {'label': instruction}, goal.request_id)
        kind, payload = self._drive_skill(self._approach, goal, my_epoch, parent)
        if kind == 'aborted':
            return 'ABORTED'
        if kind in ('no_server', 'rejected'):
            return 'FAIL'
        if payload.outcome == ApproachDetection.Result.SUCCEEDED:
            return EVENT.REACHED
        # STALE_DETECTION / LOST_TARGET / ABORTED -> back to SEARCH
        return EVENT.LOST

    def _do_stop(self, my_epoch, parent, mode):
        goal = Stop.Goal()
        goal.request_id = self._new_id()
        goal.mission_epoch = my_epoch
        goal.mode = mode
        # Stop is epoch-exempt; drive it best-effort, ignore supersession.
        self._drive_skill(self._stop, goal, my_epoch, parent)

    def _execute_vlm_handoff(self, parent, goal, my_epoch):
        """Unified /seek_object entry for VLM mode.

        The Pi executive owns the action handle, but the actual high-level policy
        lives in planner_orchestrator. We forward the instruction to /vlm_mission
        and keep the action alive until the VLM trace publishes mission_end.
        """
        result = SeekObject.Result()
        start_wall = time.time()
        msg = String()
        msg.data = json.dumps({
            'instruction': goal.instruction,
            'request_id': goal.request_id or self._new_id(),
            'mission_epoch': int(my_epoch),
        }, ensure_ascii=False)
        self._vlm_mission_pub.publish(msg)
        self.node.get_logger().info(
            'SeekObject VLM handoff: published "%s" epoch=%d to /vlm_mission; '
            'waiting for /vlm/activity'
            % (goal.instruction, my_epoch))
        self._broadcast_status(
            STATE.VLM, epoch=my_epoch, subtask='planner_orchestrator',
            instruction=goal.instruction, outcome='handoff')

        started = False
        start_deadline = start_wall + max(0.1, self._vlm_handoff_start_timeout_s)
        result_deadline = None
        if self._vlm_handoff_result_timeout_s > 0.0:
            result_deadline = start_wall + self._vlm_handoff_result_timeout_s

        while rclpy.ok():
            if parent.is_cancel_requested:
                self._do_stop(my_epoch, parent, Stop.Goal.SOFT_STOP)
                self.ms.finish()
                result.outcome = SeekObject.Result.PREEMPTED
                result.summary = 'cancelled VLM mission handoff'
                self._broadcast_status(
                    STATE.STOP, epoch=my_epoch, instruction=goal.instruction,
                    outcome='cancelled')
                parent.canceled()
                return result
            if not self.ms.is_current(my_epoch):
                result.outcome = SeekObject.Result.PREEMPTED
                result.summary = 'superseded by newer instruction'
                parent.abort()
                return result

            self._publish_feedback(
                parent, my_epoch, STATE.VLM, 'planner_orchestrator',
                instruction=goal.instruction)

            events = self._vlm_events_since(start_wall)
            for ev in events:
                event = str(ev.get('event') or '')
                if event == 'mission_start' and vlm_activity_matches_instruction(
                        ev, goal.instruction):
                    started = True
                    self._broadcast_status(
                        STATE.VLM, epoch=my_epoch, subtask='planner_orchestrator',
                        instruction=goal.instruction, outcome='running')
                if started and event == 'mission_end':
                    self.ms.finish()
                    steps = int(ev.get('steps') or 0)
                    degraded = bool(ev.get('degraded'))
                    result.outcome = (
                        SeekObject.Result.DEGRADED_SUCCESS if degraded
                        else SeekObject.Result.SUCCEEDED)
                    result.summary = (
                        'VLM mission ended after %d steps%s'
                        % (steps, ' (degraded)' if degraded else ''))
                    self._publish_feedback(parent, my_epoch, STATE.DONE,
                                           instruction=goal.instruction)
                    self._broadcast_status(
                        STATE.DONE, epoch=my_epoch, instruction=goal.instruction,
                        outcome=result.summary)
                    parent.succeed()
                    return result

            now = time.time()
            if not started and now >= start_deadline:
                self.ms.finish()
                result.outcome = SeekObject.Result.ABORTED
                result.summary = 'VLM orchestrator did not publish mission_start'
                self.node.get_logger().warn(result.summary)
                self._broadcast_status(
                    STATE.FAILED, epoch=my_epoch, instruction=goal.instruction,
                    outcome=result.summary)
                parent.abort()
                return result
            if result_deadline is not None and now >= result_deadline:
                self.ms.finish()
                result.outcome = SeekObject.Result.ABORTED
                result.summary = 'VLM mission handoff timed out waiting for mission_end'
                self.node.get_logger().warn(result.summary)
                self._broadcast_status(
                    STATE.FAILED, epoch=my_epoch, instruction=goal.instruction,
                    outcome=result.summary)
                parent.abort()
                return result

            time.sleep(0.2)

        result.outcome = SeekObject.Result.ABORTED
        result.summary = 'ROS shutdown during VLM mission handoff'
        parent.abort()
        return result

    # -- main loop ------------------------------------------------------------

    def _execute(self, parent):
        goal = parent.request
        my_epoch = self.ms.start_mission(goal.instruction, goal.allow_vlm)
        self.sync_epoch()
        self._search_start_ns = self.node.get_clock().now().nanoseconds
        self.node.get_logger().info(
            'SeekObject: "%s" epoch=%d allow_vlm=%s' % (goal.instruction, my_epoch, goal.allow_vlm))
        if goal.allow_vlm:
            return self._execute_vlm_handoff(parent, goal, my_epoch)

        self.prompt.publish(goal.instruction)
        state = fsm.next_state(STATE.IDLE, EVENT.START)   # -> SEARCH
        result = SeekObject.Result()

        while not fsm.is_terminal(state) and rclpy.ok():
            if parent.is_cancel_requested:
                self._do_stop(my_epoch, parent, Stop.Goal.SOFT_STOP)
                self.ms.finish()
                result.outcome = SeekObject.Result.PREEMPTED
                result.summary = 'cancelled'
                self._broadcast_status(STATE.STOP, epoch=my_epoch,
                                       instruction=goal.instruction, outcome='cancelled')
                parent.canceled()
                return result
            if not self.ms.is_current(my_epoch):
                # a newer SeekObject goal superseded this mission
                result.outcome = SeekObject.Result.PREEMPTED
                result.summary = 'superseded by newer instruction'
                parent.abort()
                return result

            # Report WHICH skill the FSM is driving (ExploreFrontier in SEARCH,
            # ApproachDetection in APPROACH) so a mission monitor can tell "idle in
            # SEARCH" from "actively driving" -- active_subtask was always '' before,
            # making a live run impossible to diagnose.
            self._publish_feedback(parent, my_epoch, state,
                                   fsm.select_subgoal(state) or '',
                                   instruction=goal.instruction)

            if state == STATE.SEARCH:
                self._search_start_ns = self.node.get_clock().now().nanoseconds
                ev = self._do_search(my_epoch, parent)
                if ev == 'CONTINUE':
                    continue
                if ev == 'ABORTED':
                    continue   # let the top-of-loop checks resolve it
                if ev == 'FAIL':
                    state = STATE.FAILED
                else:
                    state = fsm.next_state(state, ev)
            elif state == STATE.DETECT:
                # transient: the ApproachDetection server does the geometry and
                # will classify LOST if it cannot compute a goal.
                state = fsm.next_state(state, EVENT.COMPUTABLE)
            elif state == STATE.APPROACH:
                ev = self._do_approach(my_epoch, parent, goal.instruction)
                if ev == 'ABORTED':
                    continue
                if ev == 'FAIL':
                    state = STATE.FAILED
                else:
                    state = fsm.next_state(state, ev)
            else:
                break

        self.ms.finish()
        self._publish_feedback(parent, my_epoch, state, instruction=goal.instruction)
        if state == STATE.DONE:
            result.outcome = SeekObject.Result.SUCCEEDED
            result.summary = 'reached target'
            parent.succeed()
        else:
            result.outcome = SeekObject.Result.ABORTED
            result.summary = 'frontiers exhausted' if state == STATE.FAILED else 'ended'
            parent.abort()
        self._broadcast_status(state, epoch=my_epoch, instruction=goal.instruction,
                               outcome=result.summary)
        return result
