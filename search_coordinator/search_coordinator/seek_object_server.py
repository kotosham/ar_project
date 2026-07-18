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
import time
import uuid

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PointStamped
from std_msgs.msg import String

from object_tracking_msgs.action import SeekObject

from ar_project_msgs.action import ApproachDetection, ExploreFrontier, Stop

from fleet_comms.qos import control_cmd_latched, detection_stream_nodeadline

from search_coordinator import executive_fsm as fsm
from search_coordinator.executive_fsm import EVENT, STATE


class SeekObjectServer:
    def __init__(self, node, mission_state, prompt_bridge, sync_epoch_cb,
                 pixel_fresh_s=1.5):
        self.node = node
        self.ms = mission_state
        self.prompt = prompt_bridge
        self.sync_epoch = sync_epoch_cb
        self._pixel_fresh_s = pixel_fresh_s
        # When ExploreFrontier reports NO_FRONTIER, wait this long for a fresh
        # detection before declaring failure: the target can be visible at spawn
        # (the tracker streams /target_pixel) with no unexplored frontier left to
        # drive to -- that must become DETECT, not a spurious 'frontiers exhausted'.
        self._no_frontier_detect_wait_s = pixel_fresh_s + 1.5

        self._last_pixel = None
        self._last_pixel_recv_ns = 0
        self._search_start_ns = 0

        self._client_group = ReentrantCallbackGroup()
        self._explore = ActionClient(node, ExploreFrontier, 'explore_frontier',
                                     callback_group=self._client_group)
        self._approach = ActionClient(node, ApproachDetection, 'approach_detection',
                                      callback_group=self._client_group)
        self._stop = ActionClient(node, Stop, 'stop', callback_group=self._client_group)

        self._sub_group = ReentrantCallbackGroup()
        node.create_subscription(PointStamped, '/target_pixel', self._on_pixel,
                                 detection_stream_nodeadline(), callback_group=self._sub_group)

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

    def _fresh_pixel_event(self):
        """EVENT.DETECTED if a fresh pixel arrived since SEARCH started, else None."""
        if self._last_pixel is None or self._last_pixel_recv_ns <= self._search_start_ns:
            return None
        age = (self.node.get_clock().now().nanoseconds - self._last_pixel_recv_ns) / 1e9
        return EVENT.DETECTED if age <= self._pixel_fresh_s else None

    def _await_detection(self, my_epoch, parent, timeout_s):
        """Poll for a fresh /target_pixel for up to timeout_s. Returns EVENT.DETECTED
        or None. Honors parent cancel + epoch supersession. Used to catch a target
        that is already in view when there is no frontier left to explore."""
        deadline = self.node.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while self.node.get_clock().now().nanoseconds < deadline:
            if self._should_abort(parent, my_epoch):
                return None
            ev = self._fresh_pixel_event()
            if ev is not None:
                return ev
            time.sleep(0.1)
        return None

    # -- helpers --------------------------------------------------------------

    def _new_id(self):
        return str(uuid.uuid4())

    def _should_abort(self, parent, my_epoch):
        return parent.is_cancel_requested or not self.ms.is_current(my_epoch)

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

    def _do_search(self, my_epoch, parent):
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

    # -- main loop ------------------------------------------------------------

    def _execute(self, parent):
        goal = parent.request
        my_epoch = self.ms.start_mission(goal.instruction, goal.allow_vlm)
        self.sync_epoch()
        self.prompt.publish(goal.instruction)
        self._search_start_ns = self.node.get_clock().now().nanoseconds
        self.node.get_logger().info(
            'SeekObject: "%s" epoch=%d allow_vlm=%s' % (goal.instruction, my_epoch, goal.allow_vlm))

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
