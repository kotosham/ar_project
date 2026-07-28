"""Persistent logger for the VLM mission trace.

Subscribes to /vlm/activity (JSON String events from planner_orchestrator) and
stores two edge-local artifacts:

  * JSONL: every raw activity event, one JSON object per line.
  * CSV: compact step/failure/final rows for quick experiment analysis.
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
    'mission_index',
    'mission_id',
    'target',
    'step',
    'role',
    'action',
    'result',
    'duration_s',
    'latency_ms',
    'n_detections',
    'best_detection_label',
    'best_detection_score',
    'best_detection_distance_m',
    'n_context',
    'best_context_label',
    'best_context_score',
    'best_context_distance_m',
    'best_context_side',
    'map',
    'client',
    'degraded',
    'error',
    'rationale',
    'notes_summary',
]


def _activity_qos(depth=50):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def _slug(text):
    value = re.sub(r'[^a-zA-Z0-9_.-]+', '_', (text or '').strip()).strip('_')
    return value[:48] or 'mission'


def _iso(ts):
    return time.strftime('%Y-%m-%dT%H:%M:%S', time.localtime(float(ts)))


def _best(items):
    if not items:
        return {}
    return max(items, key=lambda x: float(x.get('score') or 0.0))


def _fmt(value):
    if value is None:
        return ''
    if isinstance(value, float):
        return '%.3f' % value
    return str(value)


class VlmMissionLogger(Node):
    def __init__(self):
        super().__init__('vlm_mission_logger')
        self.declare_parameter('activity_topic', '/vlm/activity')
        self.declare_parameter('output_dir', '~/ros2_ws/experiment_logs/vlm_missions')
        self.declare_parameter('run_id', '')
        self.declare_parameter('flush_every_event', True)

        self.activity_topic = str(self.get_parameter('activity_topic').value)
        self.output_dir = Path(os.path.expanduser(
            str(self.get_parameter('output_dir').value))).resolve()
        configured_run_id = str(self.get_parameter('run_id').value).strip()
        self.run_id = configured_run_id or time.strftime('%Y%m%d_%H%M%S')
        self.flush_every_event = bool(self.get_parameter('flush_every_event').value)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.jsonl_path = self.output_dir / ('vlm_activity_%s.jsonl' % _slug(self.run_id))
        self.csv_path = self.output_dir / ('vlm_steps_%s.csv' % _slug(self.run_id))
        self._jsonl = self.jsonl_path.open('a', encoding='utf-8')
        self._csv = self.csv_path.open('a', newline='', encoding='utf-8')
        self._csv_writer = csv.DictWriter(self._csv, fieldnames=CSV_FIELDS)
        if self.csv_path.stat().st_size == 0:
            self._csv_writer.writeheader()

        self._mission_index = 0
        self._mission_id = ''
        self._target = ''
        self._step_cache = {}
        self._notes_summary = ''

        self.create_subscription(String, self.activity_topic, self._on_activity,
                                 _activity_qos(depth=50))
        self.get_logger().info(
            'VLM mission logger writing JSONL=%s CSV=%s'
            % (self.jsonl_path, self.csv_path))

    def destroy_node(self):
        try:
            self._jsonl.close()
            self._csv.close()
        finally:
            super().destroy_node()

    def _on_activity(self, msg):
        rx = time.time()
        try:
            event = json.loads(msg.data)
            if not isinstance(event, dict):
                event = {'event': 'raw', 'detail': event}
        except (TypeError, ValueError):
            event = {'event': 'raw', 'detail': msg.data}

        self._update_mission_state(event)
        self._write_jsonl(event, rx)
        row = self._csv_row(event, rx)
        if row is not None:
            self._csv_writer.writerow(row)
        if self.flush_every_event:
            self._jsonl.flush()
            self._csv.flush()

    def _update_mission_state(self, event):
        kind = str(event.get('event') or '')
        if kind == 'mission_start':
            self._mission_index += 1
            self._target = str(event.get('target') or '')
            self._mission_id = '%s_m%03d_%s' % (
                _slug(self.run_id), self._mission_index, _slug(self._target))
            self._step_cache = {}
            self._notes_summary = ''
        elif kind == 'notes':
            self._notes_summary = str(event.get('summary') or '')

    def _write_jsonl(self, event, rx):
        record = {
            'logger_rx_stamp': rx,
            'logger_rx_iso': _iso(rx),
            'run_id': self.run_id,
            'mission_index': self._mission_index,
            'mission_id': self._mission_id,
            'target': self._target or event.get('target', ''),
            'activity': event,
        }
        self._jsonl.write(json.dumps(record, ensure_ascii=False, default=str) + '\n')

    def _cache_for_step(self, step):
        try:
            key = int(step)
        except (TypeError, ValueError):
            key = -1
        return self._step_cache.setdefault(key, {})

    def _csv_row(self, event, rx):
        kind = str(event.get('event') or '')
        step = event.get('step', '')
        cache = self._cache_for_step(step)

        if kind == 'mission_start':
            return self._row(rx, kind, '', cache, result='started')

        if kind == 'observe':
            detections = list(event.get('detections') or [])
            contexts = list(event.get('context_marks') or [])
            det = _best(detections)
            ctx = _best(contexts)
            cache.update({
                'n_detections': len(detections),
                'best_detection_label': det.get('label', ''),
                'best_detection_score': det.get('score', ''),
                'best_detection_distance_m': det.get('distance_m', ''),
                'n_context': len(contexts),
                'best_context_label': ctx.get('label', ''),
                'best_context_score': ctx.get('score', ''),
                'best_context_distance_m': ctx.get('distance_m', ''),
                'best_context_side': ctx.get('side', ''),
                'map': event.get('map', ''),
                'client': event.get('client', ''),
            })
            return None

        if kind == 'plan':
            actions = list(event.get('actions') or [])
            first = actions[0] if actions else {}
            cache.update({
                'latency_ms': event.get('latency_ms', ''),
                'action': first.get('action', ''),
                'role': first.get('role', ''),
                'rationale': first.get('rationale', ''),
            })
            return None

        if kind == 'step_start':
            cache.update({
                'action': event.get('action', cache.get('action', '')),
                'role': event.get('role', cache.get('role', '')),
                'rationale': event.get('rationale', cache.get('rationale', '')),
            })
            if str(event.get('action') or '').startswith('DONE'):
                return self._row(rx, 'done', step, cache, result='done')
            return None

        if kind == 'step_result':
            cache.update({
                'action': event.get('action', cache.get('action', '')),
                'result': event.get('result', ''),
                'duration_s': event.get('duration_s', ''),
            })
            return self._row(rx, kind, step, cache)

        if kind in ('plan_failed', 'degraded', 'auto_done', 'mission_end', 'step_progress'):
            if kind == 'mission_end' and step == '':
                step = event.get('steps', '')
            extra = {
                'result': event.get('result', kind),
                'duration_s': event.get('duration_s', ''),
                'error': event.get('error', ''),
                'degraded': event.get('degraded', ''),
            }
            cache.update(extra)
            return self._row(rx, kind, step, cache)

        return None

    def _row(self, rx, event_kind, step, cache, **overrides):
        row = {
            'rx_iso': _iso(rx),
            'event': event_kind,
            'run_id': self.run_id,
            'mission_index': self._mission_index,
            'mission_id': self._mission_id,
            'target': self._target,
            'step': step,
            'role': cache.get('role', ''),
            'action': cache.get('action', ''),
            'result': cache.get('result', ''),
            'duration_s': cache.get('duration_s', ''),
            'latency_ms': cache.get('latency_ms', ''),
            'n_detections': cache.get('n_detections', ''),
            'best_detection_label': cache.get('best_detection_label', ''),
            'best_detection_score': cache.get('best_detection_score', ''),
            'best_detection_distance_m': cache.get('best_detection_distance_m', ''),
            'n_context': cache.get('n_context', ''),
            'best_context_label': cache.get('best_context_label', ''),
            'best_context_score': cache.get('best_context_score', ''),
            'best_context_distance_m': cache.get('best_context_distance_m', ''),
            'best_context_side': cache.get('best_context_side', ''),
            'map': cache.get('map', ''),
            'client': cache.get('client', ''),
            'degraded': cache.get('degraded', ''),
            'error': cache.get('error', ''),
            'rationale': cache.get('rationale', ''),
            'notes_summary': self._notes_summary,
        }
        row.update(overrides)
        return {k: _fmt(row.get(k, '')) for k in CSV_FIELDS}


def main(args=None):
    rclpy.init(args=args)
    node = VlmMissionLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
