#!/usr/bin/env python3
"""Small operator CLI for the unified SeekObject mission entrypoint."""
import argparse
import re
import sys
import time
import uuid

import rclpy
from rclpy.action import ActionClient

from object_tracking_msgs.action import SeekObject


def parse_bool(value):
    text = str(value or '').strip().lower()
    if text in ('true', '1', 'yes', 'y', 'vlm'):
        return True
    if text in ('false', '0', 'no', 'n', 'flat'):
        return False
    raise argparse.ArgumentTypeError(
        'mode must be true/false (also accepts vlm/flat)')


def _slug(text):
    value = re.sub(r'[^a-zA-Z0-9]+', '_', (text or '').strip().lower()).strip('_')
    return value[:32] or 'mission'


def _parse_args(argv=None):
    p = argparse.ArgumentParser(
        description='Send a SeekObject mission. Examples: '
                    'send_mission "chair" true  |  send_mission "chair" false')
    p.add_argument('tokens', nargs='+',
                   help='instruction plus optional final true/false mode')
    p.add_argument('--vlm', '--allow-vlm', dest='allow_vlm', type=parse_bool,
                   default=None, help='true = VLM, false = FLAT')
    p.add_argument('--request-id', default='', help='optional idempotency id')
    p.add_argument('--mission-epoch', type=int, default=0,
                   help='kept for interface compatibility; normally leave 0')
    args = p.parse_args(argv)

    tokens = list(args.tokens)
    if args.allow_vlm is None:
        if len(tokens) < 2:
            p.error('pass mode true/false, e.g. send_mission "chair" true')
        try:
            args.allow_vlm = parse_bool(tokens[-1])
        except argparse.ArgumentTypeError as e:
            p.error(str(e))
        tokens = tokens[:-1]

    args.instruction = ' '.join(tokens).strip()
    if not args.instruction:
        p.error('instruction must not be empty')
    if not args.request_id:
        mode = 'vlm' if args.allow_vlm else 'flat'
        args.request_id = '%s_%s_%s_%s' % (
            mode, _slug(args.instruction), int(time.time()), uuid.uuid4().hex[:6])
    return args


def main(argv=None):
    args = _parse_args(argv)
    rclpy.init()
    node = rclpy.create_node('send_mission')
    client = ActionClient(node, SeekObject, 'seek_object')
    try:
        if not client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error('/seek_object action server is not available')
            return 2

        goal = SeekObject.Goal()
        goal.instruction = args.instruction
        goal.request_id = args.request_id
        goal.mission_epoch = int(args.mission_epoch)
        goal.allow_vlm = bool(args.allow_vlm)

        mode = 'VLM' if goal.allow_vlm else 'FLAT'
        print('Sending %s mission: %r (request_id=%s)' %
              (mode, goal.instruction, goal.request_id), flush=True)

        send_future = client.send_goal_async(
            goal,
            feedback_callback=lambda fb: print(
                'feedback: state=%s subtask=%s progress=%.2f epoch=%d' % (
                    fb.feedback.state, fb.feedback.active_subtask,
                    float(fb.feedback.progress), int(fb.feedback.mission_epoch)),
                flush=True))
        rclpy.spin_until_future_complete(node, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            print('Goal rejected', flush=True)
            return 1

        result_future = gh.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        wrapped = result_future.result()
        result = wrapped.result
        status = wrapped.status
        print('result: status=%s outcome=%s summary=%s' %
              (status, result.outcome, result.summary), flush=True)
        return 0 if result.outcome in (
            SeekObject.Result.SUCCEEDED,
            SeekObject.Result.DEGRADED_SUCCESS,
        ) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
