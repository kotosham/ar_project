"""Persistent logger for FLAT SeekObject experiments.

Unlike the VLM logger, this node does not listen to /vlm/activity.  FLAT mode has
no high-level reasoning trace, so the logger records the executive FSM snapshots
from /mission/status and emits one compact CSV row per completed mission.
"""
from __future__ import annotations

import csv
import json
import os
import re
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


CSV_FIELDS = [
    'rx_iso',
    'event',
    'run_id',
    'scene_id',
    'repeat_id',
    'mission_index',
    'mission_id',
    'mission_epoch',
    'instruction',
    'start_iso',
    'end_iso',
    'duration_s',
    'duration_status_s',
    'time_to_first_action_s',
    'time_to_detect_s',
    'time_to_approach_s',
    'terminal_state',
    'terminal_outcome',
    'max_state_reached',
    'flat_progress_rate',
    'success_auto',
    'success_manual',
    'states_seen',
]

TERMINAL_STATES = {'DONE', 'FAILED'}
TRACKED_STATES = {'SEARCH', 'DETECT', 'APPROACH', 'STOP', 'DONE', 'FAILED'}
STATE_RANK = {
    'SEARCH': 1,
    'DETECT': 2,
    'APPROACH': 3,
    'DONE': 4,
}


def _status_qos(depth=50):
    # /mission/status is latched for dashboards, but experiment loggers must not
    # replay stale terminal messages after restart and create phantom runs.
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _slug(text):
    value = re.sub(r'[^a-zA-Z0-9_.-]+', '_', (text or '').strip()).strip('_')
    return value[:48] or 'mission'


def _iso(ts):
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(float(ts)))


def _fmt(value):
    if value is None:
        return ''
    if isinstance(value, float):
        return '%.3f' % value
    return str(value)


def _parse_status(data):
    if not isinstance(data, dict):
        return {}
    out = dict(data)
    out['state'] = str(out.get('state') or '').strip()
    out['instruction'] = str(out.get('instruction') or '').strip()
    out['active_subtask'] = str(out.get('active_subtask') or '').strip()
    out['outcome'] = str(out.get('outcome') or '').strip()
    try:
        out['mission_epoch'] = int(out.get('mission_epoch') or 0)
    except (TypeError, ValueError):
        out['mission_epoch'] = 0
    try:
        out['stamp'] = float(out.get('stamp') or 0.0)
    except (TypeError, ValueError):
        out['stamp'] = 0.0
    return out


class FlatMissionTracker:
    """Pure state tracker used by the ROS node and unit tests."""

    def __init__(self, run_id, scene_id='', repeat_id='', include_vlm=False):
        self.run_id = run_id
        self.scene_id = scene_id
        self.repeat_id = repeat_id
        self.include_vlm = include_vlm
        self.mission_index = 0
        self.current = None
        self.last_context = {}
        self.ignored_epochs = set()

    def update(self, status, rx):
        status = _parse_status(status)
        state = status.get('state', '')
        if not state or state == 'IDLE':
            return None
        epoch = int(status.get('mission_epoch') or 0)

        if state == 'VLM' and not self.include_vlm:
            self.ignored_epochs.add(epoch)
            if self.current and self.current.get('mission_epoch') == epoch:
                self.current = None
                self.last_context = {}
            return None
        if epoch in self.ignored_epochs and not self.include_vlm:
            return None
        if state not in TRACKED_STATES:
            return None

        if self._needs_new_mission(status):
            self._start(status, rx)

        self._remember_state(status, rx)
        self.last_context = dict(self.current or {})
        if state in TERMINAL_STATES:
            row = self._finish(status, rx)
            self.current = None
            return row
        return None

    def _needs_new_mission(self, status):
        if self.current is None:
            return True
        return (
            int(self.current.get('mission_epoch') or 0) != int(status.get('mission_epoch') or 0)
            or str(self.current.get('instruction') or '') != str(status.get('instruction') or '')
        )

    def _start(self, status, rx):
        self.mission_index += 1
        instruction = status.get('instruction') or 'mission'
        self.current = {
            'mission_index': self.mission_index,
            'mission_id': '%s_m%03d_%s' % (
                _slug(self.run_id), self.mission_index, _slug(instruction)),
            'mission_epoch': int(status.get('mission_epoch') or 0),
            'instruction': instruction,
            'start_rx': float(rx),
            'start_status_stamp': float(status.get('stamp') or 0.0),
            'states_seen': [],
            'first_state_rx': {},
        }

    def _remember_state(self, status, rx):
        if self.current is None:
            return
        state = status.get('state') or ''
        seen = self.current.setdefault('states_seen', [])
        if state and (not seen or seen[-1] != state):
            seen.append(state)
        first_state_rx = self.current.setdefault('first_state_rx', {})
        if state and state not in first_state_rx:
            first_state_rx[state] = float(rx)

    def _max_state_reached(self):
        states = list(self.current.get('states_seen') or []) if self.current else []
        ranked = [s for s in states if s in STATE_RANK and s != 'DONE']
        if 'DONE' in states:
            return 'DONE'
        if ranked:
            return max(ranked, key=lambda s: STATE_RANK.get(s, 0))
        return states[-1] if states else ''

    def _progress_rate(self, terminal_state):
        max_state = self._max_state_reached()
        if terminal_state == 'DONE':
            return 1.0
        if STATE_RANK.get(max_state, 0) >= STATE_RANK['DETECT']:
            return 0.66
        if STATE_RANK.get(max_state, 0) >= STATE_RANK['SEARCH']:
            return 0.33
        return 0.0

    def _finish(self, status, rx):
        cur = self.current or {}
        terminal_state = status.get('state') or ''
        start_rx = float(cur.get('start_rx') or rx)
        start_stamp = float(cur.get('start_status_stamp') or 0.0)
        end_stamp = float(status.get('stamp') or 0.0)
        duration_status = ''
        if start_stamp > 0.0 and end_stamp >= start_stamp:
            duration_status = end_stamp - start_stamp
        progress = self._progress_rate(terminal_state)
        success = 1 if terminal_state == 'DONE' else 0

        def elapsed_to_state(*states):
            first_state_rx = cur.get('first_state_rx') or {}
            values = [float(first_state_rx[s]) for s in states if s in first_state_rx]
            if not values:
                return ''
            return max(0.0, min(values) - start_rx)

        return {
            'rx_iso': _iso(rx),
            'event': 'mission_end',
            'run_id': self.run_id,
            'scene_id': self.scene_id,
            'repeat_id': self.repeat_id,
            'mission_index': cur.get('mission_index', ''),
            'mission_id': cur.get('mission_id', ''),
            'mission_epoch': cur.get('mission_epoch', ''),
            'instruction': cur.get('instruction', status.get('instruction', '')),
            'start_iso': _iso(start_rx),
            'end_iso': _iso(rx),
            'duration_s': max(0.0, float(rx) - start_rx),
            'duration_status_s': duration_status,
            'time_to_first_action_s': elapsed_to_state('SEARCH', 'DETECT', 'APPROACH'),
            'time_to_detect_s': elapsed_to_state('DETECT'),
            'time_to_approach_s': elapsed_to_state('APPROACH'),
            'terminal_state': terminal_state,
            'terminal_outcome': status.get('outcome', ''),
            'max_state_reached': self._max_state_reached(),
            'flat_progress_rate': progress,
            'success_auto': success,
            'success_manual': '',
            'states_seen': '>'.join(cur.get('states_seen') or []),
        }


class FlatMissionLogger(Node):
    def __init__(self):
        super().__init__('flat_mission_logger')
        self.declare_parameter('status_topic', '/mission/status')
        self.declare_parameter('output_dir', '~/ros2_ws/experiment_logs/flat_missions')
        self.declare_parameter('run_id', '')
        self.declare_parameter('scene_id', '')
        self.declare_parameter('repeat_id', '')
        self.declare_parameter('include_vlm', False)
        self.declare_parameter('flush_every_event', True)

        self.status_topic = str(self.get_parameter('status_topic').value)
        self.output_dir = Path(os.path.expanduser(
            str(self.get_parameter('output_dir').value))).resolve()
        configured_run_id = str(self.get_parameter('run_id').value).strip()
        self.run_id = configured_run_id or time.strftime('%Y%m%d_%H%M%S')
        self.scene_id = str(self.get_parameter('scene_id').value).strip()
        self.repeat_id = str(self.get_parameter('repeat_id').value).strip()
        self.flush_every_event = bool(self.get_parameter('flush_every_event').value)
        include_vlm = bool(self.get_parameter('include_vlm').value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        artifact_stem = _slug(self.run_id)
        self.jsonl_path = self.output_dir / ('%s.jsonl' % artifact_stem)
        self.csv_path = self.output_dir / ('%s.csv' % artifact_stem)
        self._jsonl = self.jsonl_path.open('a', encoding='utf-8')
        self._csv = self.csv_path.open('a', newline='', encoding='utf-8')
        self._csv_writer = csv.DictWriter(self._csv, fieldnames=CSV_FIELDS)
        if self.csv_path.stat().st_size == 0:
            self._csv_writer.writeheader()

        self._tracker = FlatMissionTracker(
            self.run_id, scene_id=self.scene_id, repeat_id=self.repeat_id,
            include_vlm=include_vlm)

        self.create_subscription(String, self.status_topic, self._on_status,
                                 _status_qos(depth=50))
        self.get_logger().info(
            'FLAT mission logger writing JSONL=%s CSV=%s'
            % (self.jsonl_path, self.csv_path))

    def destroy_node(self):
        try:
            self._jsonl.close()
            self._csv.close()
        finally:
            super().destroy_node()

    def _on_status(self, msg):
        rx = time.time()
        try:
            status = json.loads(msg.data)
            if not isinstance(status, dict):
                status = {'raw': status}
        except (TypeError, ValueError):
            status = {'raw': msg.data}

        row = self._tracker.update(status, rx)
        self._write_jsonl(status, rx)
        if row is not None:
            self._csv_writer.writerow({k: _fmt(row.get(k, '')) for k in CSV_FIELDS})
        if self.flush_every_event:
            self._jsonl.flush()
            self._csv.flush()

    def _write_jsonl(self, status, rx):
        current = self._tracker.current or self._tracker.last_context or {}
        record = {
            'logger_rx_stamp': rx,
            'logger_rx_iso': _iso(rx),
            'run_id': self.run_id,
            'scene_id': self.scene_id,
            'repeat_id': self.repeat_id,
            'mission_index': current.get('mission_index', self._tracker.mission_index),
            'mission_id': current.get('mission_id', ''),
            'status': status,
        }
        self._jsonl.write(json.dumps(record, ensure_ascii=False, default=str) + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = FlatMissionLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
