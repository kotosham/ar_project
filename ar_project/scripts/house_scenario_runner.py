#!/usr/bin/env python3
"""House benchmark episode runner -- drives ONE diagnostic episode end to end.

EPISODE LIFECYCLE
-----------------
  load     resolve <scenario>.yaml + perturbations/<pid>.yaml, merge them into one
           "merged scenario" (union of lights_off, concatenated spawn lists, scaled
           timeout) and validate every predicate BEFORE anything is touched in the sim.
  arm      wait for /clock to advance and for a usable robot pose source.  Three
           sources are tried in order -- TF map->base_link, TF odom->base_link, the
           /odom topic -- because during sim bring-up SLAM may not have published
           map->odom yet.  The winner is logged loudly and stored in the record as
           `pose_source`: a run scored against odom is NOT metrically identical to one
           scored against a SLAM map, so the analysis must be able to see this.
  dress    teleport the robot (optional), spawn every prop, dim every light in
           lights_off, latch the camera perturbation profile.  Any failure here is
           terminal and produces outcome=setup_failed -- an episode that silently ran
           without its target prop or without its darkness is a corrupt datapoint and
           is far more damaging to the benchmark than a missing one.
  run      publish the mission on /vlm_mission, then tick at tick_hz: sample the pose,
           integrate path length, latch subgoal predicates, watch /scan for a
           catastrophic collision, watch the clock for the timeout.
  finish   write <episode_key>.jsonl (the ordered event trace) and <episode_key>.json
           (the scalar episode record), print a compact summary table.

PAIRED FLAT-vs-PLANNER DESIGN
-----------------------------
`planner_label` is the paired-comparison key and nothing else -- the runner never
changes its own behaviour based on it.  Two episodes sharing the same
(scenario, perturbation, seed) but differing in planner_label form ONE matched pair:

    s3__p_dark__vlm__seed0        <-- the VLM planner under test
    s3__p_dark__flat_mock__seed0  <-- the MockPlanner / flat baseline

Because the world, the props, the lighting, the perturbation profile and the oracle
predicates are byte-identical across the pair, the difference in ordered_progress
between the two members is attributable to the planner alone.  This is why the setup
phase is fail-loud: a pair in which one member silently lost a prop is worse than no
pair at all.  Aggregate over pairs (paired test), never over the raw pooled runs.

METRICS (mirrors the RoboCerebra-Diagnostic protocol so the two benchmarks compare)
-----------------------------------------------------------------------------------
  ordered_progress   longest completed PREFIX of the subgoal list / n_subgoals
  unordered_progress completed subgoals in any order / n_subgoals
  progress_auc       mean over ticks of the instantaneous ordered_progress -- rewards
                     reaching progress EARLY, so a planner that dithers and then
                     succeeds at the buzzer scores below one that goes straight there
  first_failure_subgoal  id of the first subgoal never completed, else null

Each predicate is latched independently the first time it fires (that is what makes
unordered_progress meaningful and gives an honest first-completion timestamp); the
ordering requirement is then expressed by the PREFIX rule above.  Every subgoal_done
event also carries `in_order`, so a downstream analysis can reconstruct either view.

Only rclpy / std_msgs / nav_msgs / sensor_msgs / geometry_msgs / tf2_ros / PyYAML and
the standard library are used -- no numpy, no cv2, so the runner starts on a Pi.
"""

import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import tf2_ros
from tf2_ros import Buffer, TransformListener

from ament_index_python.packages import get_package_share_directory


SCHEMA_VERSION = 1

# Room axis-aligned bounding boxes as (x_min, x_max, y_min, y_max), metres, world frame.
# MUST MATCH worlds/house.sdf AND config/scenarios/README.md -- the `in_room` predicate
# is the oracle, so a divergence here silently rescores every episode ever run.
ROOMS = {
    'hallway':  (-7.40,  7.40, -1.10,  1.10),
    'bedroom':  (-7.40, -2.00,  1.10,  4.90),
    'bathroom': (-2.00,  2.00,  1.10,  4.90),
    'kitchen':  ( 2.00,  7.40,  1.10,  4.90),
    'storage':  (-7.40, -2.00, -4.90, -1.10),
    'living':   (-2.00,  7.40, -4.90, -1.10),
}

# Canonical world-level point-light positions (x, y, z).
# MUST STAY IN SYNC WITH worlds/house.sdf.  gz light_config REPLACES the whole light
# config, so dimming a light means re-sending its pose too; if the pose sent here does
# not match the SDF the light teleports instead of going dark, which looks like a
# working perturbation in the log but is not one.  A scenario may override a pose via
# a `light_poses` mapping; anything unknown to both is an authoring error (fail loud).
LIGHT_POSES = {
    'light_hallway_w': (-4.50,  0.00, 2.0),
    'light_hallway_c': ( 0.00,  0.00, 2.0),
    'light_hallway_e': ( 4.00,  0.00, 2.0),
    'light_bedroom':   (-4.70,  3.00, 2.0),
    'light_bathroom':  ( 0.00,  3.00, 2.0),
    'light_kitchen':   ( 4.70,  3.00, 2.0),
    'light_storage':   (-4.70, -3.00, 2.0),
    'light_living':    ( 2.70, -3.00, 2.0),
}

# Attenuation of the house lights, copied from worlds/house.sdf (see note above: every
# field must be re-sent on light_config or it is reset to the gz default).
LIGHT_RANGE = 12.0
LIGHT_ATT_CONSTANT = 0.80
LIGHT_ATT_LINEAR = 0.05
LIGHT_ATT_QUADRATIC = 0.005

# Full camera perturbation profile.  Sent complete every time for the same reason the
# light config is: the consumer replaces its profile wholesale, so a partial dict would
# leave stale fields from a previous episode active.
CAMERA_PROFILE_DEFAULTS = {
    'smudge': 0.0,
    'blur_sigma': 0.0,
    'darkness': 1.0,        # multiplicative gain, 1.0 = unchanged
    'noise_sigma': 0.0,
    'dropout_prob': 0.0,
    'motion_blur_px': 0,
}

PREDICATE_TYPES = ('near_xy', 'in_room', 'saw_label', 'faced_xy', 'mission_done')
OUTCOMES = ('success', 'timeout', 'collision', 'setup_failed', 'planner_gave_up', 'aborted')

GZ_TIMEOUT_MS = 3000
GZ_ATTEMPTS = 3

# How long to wait (wall seconds) for somebody to subscribe to /vlm_mission before
# publishing anyway. See _await_mission_subscriber for why this is not optional.
MISSION_SUB_WAIT_S = 60.0

# Wheel radius is 0.038 m; drop the chassis in a hair above the floor so the teleport
# does not spawn it interpenetrating the ground plane and launch it.
ROBOT_SPAWN_Z = 0.05

# A single tick may not plausibly move the robot further than this.  The diff-drive base
# tops out around 0.5 m/s, so anything larger is a SLAM loop-closure jump in map->odom,
# not motion -- accumulating it would silently inflate path_length_m.
MAX_TICK_STEP_M = 0.50

# Sim seconds to keep ticking after the planner announces mission_end, so a subgoal that
# latches on the very last motion is not scored as a failure by a one-tick race.
MISSION_END_GRACE_S = 2.0

# Wall seconds without /clock advancing before we conclude Gazebo died and bail out.
CLOCK_STALL_S = 30.0

POSE_SAMPLE_PERIOD_S = 1.0      # pose_sample events are decimated to 1 Hz


class SetupError(Exception):
    """Raised for any condition that makes the episode unscoreable -> setup_failed."""


def _yaw_to_quat(yaw):
    """Yaw-only rotation as (x, y, z, w).  Done by hand to keep numpy off the deps."""
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def _quat_to_yaw(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


def _cmd_display(cmd):
    """Re-quote an argv list so the logged line can be pasted straight into a shell."""
    parts = []
    for a in cmd:
        parts.append("'%s'" % a if (' ' in a or '"' in a or '{' in a) else a)
    return ' '.join(parts)


def _fnum(v):
    """Format a float for a gz protobuf text request (gz rejects 'nan'/'inf')."""
    f = float(v)
    if not math.isfinite(f):
        raise SetupError('non-finite number in scenario data: %r' % (v,))
    return '%.6f' % f


def _safe_token(s):
    """Make a string safe to embed in a filename (episode_key becomes two filenames)."""
    out = []
    for ch in str(s):
        out.append(ch if (ch.isalnum() or ch in '-_.') else '_')
    return ''.join(out) or 'unknown'


class HouseScenarioRunner(Node):

    def __init__(self):
        super().__init__('house_scenario_runner')

        self.declare_parameter('scenario', '')
        self.declare_parameter('perturbation', 'p_none')
        self.declare_parameter('scenario_dir', '')
        self.declare_parameter('world_name', 'house')
        self.declare_parameter('robot_name', 'my_bot')
        self.declare_parameter('out_dir', '~/ros2_ws/house_benchmark')
        self.declare_parameter('planner_label', 'vlm')
        self.declare_parameter('seed', 0)
        self.declare_parameter('start_delay_s', 8.0)
        self.declare_parameter('tick_hz', 5.0)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('teleport_robot', True)
        # Set true only if the pose source already reports the Gazebo world frame (e.g. a
        # bridged ground-truth pose). odom and map do NOT: both are anchored at the spawn
        # pose. See _to_world.
        self.declare_parameter('pose_frame_is_world', False)
        self.declare_parameter('spawn_props', True)
        self.declare_parameter('apply_lights', True)
        self.declare_parameter('publish_mission', True)
        self.declare_parameter('collision_range_m', 0.16)
        self.declare_parameter('collision_frames', 5)
        self.declare_parameter('shutdown_on_finish', True)
        # Render-race evidence probe. gz-sim occasionally FAILS TO RENDER an
        # entity created at runtime: physics has it (gz model -p answers), but
        # the camera image never shows it, so every detect honestly reports
        # "0 candidates" and the episode reads like a planner failure. Seen
        # live on ctl_reachable: 3 runs where target-detect returned 0 from a
        # pose where a healthy run detects the chair at conf 0.86 -- with
        # fresh frames (stale-frame gate silent) and zero duplicate-server
        # warnings. This probe saves ONE camera frame right after the props
        # are spawned, next to the episode record -- post-hoc proof of what
        # the sensors actually saw at t0. '' disables.
        self.declare_parameter('spawn_probe_topic', '/camera/camera/color/image_perturbed')

        g = lambda n: self.get_parameter(n).value
        self.p_scenario = str(g('scenario')).strip()
        self.p_perturbation = str(g('perturbation')).strip() or 'p_none'
        self.p_scenario_dir = str(g('scenario_dir')).strip()
        self.world_name = str(g('world_name')).strip()
        self.robot_name = str(g('robot_name')).strip()
        self.out_dir = Path(str(g('out_dir'))).expanduser()
        self.planner_label = str(g('planner_label')).strip() or 'vlm'
        self.seed = int(g('seed'))
        self.start_delay_s = float(g('start_delay_s'))
        self.tick_hz = max(0.5, float(g('tick_hz')))
        self.odom_topic = str(g('odom_topic'))
        self.map_frame = str(g('map_frame'))
        self.robot_frame = str(g('robot_frame'))
        self.teleport_robot = bool(g('teleport_robot'))
        self.pose_frame_is_world = bool(g('pose_frame_is_world'))
        self.spawn_props = bool(g('spawn_props'))
        self.apply_lights = bool(g('apply_lights'))
        self.publish_mission = bool(g('publish_mission'))
        self.collision_range_m = float(g('collision_range_m'))
        self.collision_frames = max(1, int(g('collision_frames')))
        self.shutdown_on_finish = bool(g('shutdown_on_finish'))
        self.spawn_probe_topic = str(g('spawn_probe_topic')).strip()
        self.use_sim_time = bool(self.get_parameter('use_sim_time').value)

        # ---- episode identity (provisional until the YAML is parsed) ----
        self.scenario_id = _safe_token(Path(self.p_scenario).stem or self.p_scenario or 'unknown')
        self.perturbation_id = self.p_perturbation
        self.episode_key = self._make_key()

        # ---- merged scenario ----
        self.merged = {}
        self.mission = ''
        self.subgoals = []
        self.success = {'type': 'all_subgoals'}
        self.timeout_s = 0.0
        self.max_path_m = 0.0
        self.lights_off = []
        self.camera_profile = dict(CAMERA_PROFILE_DEFAULTS)
        self.spawned = {}

        # ---- live episode state ----
        self._finished = False
        self._outcome = None
        self._outcome_reason = ''
        self._jsonl = None
        self._event_seq = 0
        self._t_wall0 = time.time()
        self._started_iso = datetime.now().astimezone().isoformat(timespec='seconds')
        self._t_sim_mission = None      # sim time at which the mission was published
        self._t_sim_end = None

        self.pose_source = 'none'
        self._odom = None
        self._odom_frame = 'odom'
        self._scan = None
        self._last_pose = None
        self._pose_warned = False
        self.path_length_m = 0.0
        self._path_budget_warned = False

        self._sg_done = []
        self._sg_time = {}
        self._sg_step = {}
        self._auc_sum = 0.0
        self._auc_n = 0

        self._collision_streak = 0
        self.collided = False

        # Activity trace counters (filled from /vlm/activity).
        self.vlm_steps = 0
        self.action_histogram = {}
        self.detect_all_calls = 0
        self.plan_failures = 0
        self.degraded_events = 0
        self.notes_events = 0
        self._best_conf_by_label = {}
        self._mission_end = False
        self._t_sim_mission_end = None
        # /vlm/activity is TRANSIENT_LOCAL depth 50, so subscribing replays up to 50
        # events from a PREVIOUS mission in the same sim session.  Everything stamped
        # before this wall time is that backlog and must not be scored.
        self._activity_epoch_wall = None

        # ---- I/O ----
        self._tf = Buffer()
        self._tfl = TransformListener(self._tf, self)

        latched = QoSProfile(depth=1)
        latched.history = HistoryPolicy.KEEP_LAST
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Latched so a perturbation node / orchestrator that joins late still receives
        # the profile and the mission instead of the episode running unperturbed.
        self._profile_pub = self.create_publisher(String, '/sim_perturbation/profile', latched)
        self._mission_pub = self.create_publisher(String, '/vlm_mission', latched)

        activity_qos = QoSProfile(depth=50)
        activity_qos.history = HistoryPolicy.KEEP_LAST
        activity_qos.reliability = ReliabilityPolicy.RELIABLE
        activity_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, '/vlm/activity', self._on_activity, activity_qos)
        self.create_subscription(String, '/mission/status', self._on_mission_status, 10)

        # BEST_EFFORT on both sensor inputs: a BEST_EFFORT reader is compatible with a
        # RELIABLE writer but not the other way round, so this connects regardless of
        # how the bridge was configured.  We only ever read the LATEST sample at tick
        # time, so dropped intermediate messages cost nothing.
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.history = HistoryPolicy.KEEP_LAST
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(Odometry, self.odom_topic, self._on_odom, sensor_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, sensor_qos)

        self.get_logger().info(
            'house_scenario_runner up: scenario=%r perturbation=%r planner_label=%r seed=%d '
            'world=%s out_dir=%s sim_time=%s'
            % (self.p_scenario, self.p_perturbation, self.planner_label, self.seed,
               self.world_name, self.out_dir, self.use_sim_time))

    # ------------------------------------------------------------------ helpers

    def _make_key(self):
        return '%s__%s__%s__seed%d' % (_safe_token(self.scenario_id),
                                       _safe_token(self.perturbation_id),
                                       _safe_token(self.planner_label),
                                       self.seed)

    def _sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _pump(self, seconds):
        """Service callbacks for `seconds` of WALL time.

        The runner is deliberately single-threaded: setup blocks on gz CLI subprocesses,
        and pumping explicitly (rather than spinning on another thread) keeps the event
        ordering in the .jsonl trace deterministic and reproducible.
        """
        end = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ------------------------------------------------------------------ gz plumbing

    def _gz_service(self, service, reqtype, reptype, req, timeout_ms=GZ_TIMEOUT_MS):
        """The ONE choke point for every Gazebo CLI call.

        Logs the exact command line and the raw reply so a failed episode can be
        diagnosed from the log alone, without re-running the sim.  Returns
        (ok, stdout, stderr); `ok` means the service replied `data: true`.
        """
        cmd = ['gz', 'service', '-s', service,
               '--reqtype', reqtype, '--reptype', reptype,
               '--timeout', str(int(timeout_ms)),
               '--req', req]
        self.get_logger().info('gz call: %s' % _cmd_display(cmd))
        try:
            proc = subprocess.run(cmd, capture_output=True, text=True,
                                  timeout=(timeout_ms / 1000.0) + 5.0)
        except FileNotFoundError:
            msg = "'gz' CLI not found on PATH"
            self.get_logger().error('gz call failed: %s' % msg)
            return False, '', msg
        except subprocess.TimeoutExpired:
            msg = 'gz CLI wall timeout (service %s never replied)' % service
            self.get_logger().error('gz call failed: %s' % msg)
            return False, '', msg
        out = (proc.stdout or '').strip()
        err = (proc.stderr or '').strip()
        self.get_logger().info('gz reply: rc=%d stdout=%r stderr=%r' % (proc.returncode, out, err))
        ok = False
        if proc.returncode == 0:
            for line in out.splitlines():
                if line.strip().lower().startswith('data:'):
                    ok = line.split(':', 1)[1].strip().lower() == 'true'
        return ok, out, err

    def _gz_retry(self, what, service, reqtype, reptype, req, attempts=GZ_ATTEMPTS):
        """Retry a gz call; transient failures during sim bring-up are common."""
        detail = 'no attempt made'
        for k in range(1, attempts + 1):
            ok, out, err = self._gz_service(service, reqtype, reptype, req)
            if ok:
                return True, out
            detail = err or out or 'empty reply'
            self.get_logger().warn('%s: attempt %d/%d failed (%s)' % (what, k, attempts, detail))
            if k < attempts:
                self._pump(0.5)
        return False, detail

    # ------------------------------------------------------------------ subscriptions

    def _on_odom(self, msg):
        self._odom = msg
        fid = (msg.header.frame_id or '').lstrip('/')
        if fid:
            self._odom_frame = fid

    def _on_scan(self, msg):
        self._scan = msg

    def _on_mission_status(self, msg):
        # Recorded, never acted on: the coordinator's own opinion of the mission is a
        # planner output, not the oracle.  Only the subgoal predicates decide outcome.
        self._event('mission_status', text=msg.data)

    def _on_activity(self, msg):
        try:
            payload = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn('undecodable /vlm/activity payload (%s): %r' % (e, msg.data[:200]))
            self._event('activity', parse_error=str(e), raw=msg.data[:500])
            return
        if not isinstance(payload, dict):
            self._event('activity', parse_error='payload is not an object', raw=msg.data[:500])
            return

        stamp = payload.get('stamp')
        stale = False
        if self._activity_epoch_wall is None:
            stale = True                      # arrived before our mission started
        elif isinstance(stamp, (int, float)) and stamp < (self._activity_epoch_wall - 1.0):
            stale = True                      # TRANSIENT_LOCAL backlog of a prior mission
        self._event('activity', stale=stale, payload=payload)
        if stale:
            return

        event = str(payload.get('event', ''))
        if event == 'step_start':
            self.vlm_steps += 1
            # `action` looks like 'TURN +0.20rad' / 'DRIVE_TO_VISIBLE mark=3' / 'DONE';
            # the leading verb is the histogram bucket.
            verb = str(payload.get('action', '')).split(' ', 1)[0] or 'UNKNOWN'
            self.action_histogram[verb] = self.action_histogram.get(verb, 0) + 1
        elif event == 'detect_all':
            self.detect_all_calls += 1
            for obj in (payload.get('objects') or []):
                if not isinstance(obj, dict):
                    continue
                label = str(obj.get('label', '')).strip().lower()
                if not label:
                    continue
                try:
                    score = float(obj.get('score', 0.0))
                except (TypeError, ValueError):
                    score = 0.0
                prev = self._best_conf_by_label.get(label, -1.0)
                if score > prev:
                    self._best_conf_by_label[label] = score
        elif event == 'plan_failed':
            self.plan_failures += 1
        elif event == 'degraded':
            self.degraded_events += 1
        elif event == 'notes':
            self.notes_events += 1
        elif event == 'mission_end':
            if not self._mission_end:
                self._mission_end = True
                self._t_sim_mission_end = self._sim_now()
                self.get_logger().info('activity: mission_end after %s step(s)'
                                       % payload.get('steps'))

    # ------------------------------------------------------------------ trace output

    def _open_outputs(self):
        """Open the .jsonl trace.  Idempotent -- _finish() calls it too, so even a
        setup that died before this point still leaves a machine-readable record."""
        if self._jsonl is not None:
            return
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.jsonl_path = self.out_dir / ('%s.jsonl' % self.episode_key)
        self.json_path = self.out_dir / ('%s.json' % self.episode_key)
        self._jsonl = self.jsonl_path.open('w', encoding='utf-8')
        self.get_logger().info('episode trace -> %s' % self.jsonl_path)

    def _event(self, kind, **fields):
        """Append one event to the .jsonl trace, flushed immediately so a hard kill
        (Gazebo OOM, ctrl-C) still leaves everything observed up to that instant."""
        if self._jsonl is None:
            return
        self._event_seq += 1
        rec = {'seq': self._event_seq,
               'event': kind,
               't_wall': round(time.time() - self._t_wall0, 3),
               't_sim': round(self._sim_now(), 3)}
        rec.update(fields)
        try:
            self._jsonl.write(json.dumps(rec, ensure_ascii=False) + '\n')
            self._jsonl.flush()
        except Exception as e:
            self.get_logger().error('failed to write trace event %r: %s' % (kind, e))

    # ------------------------------------------------------------------ scenario load

    def _default_scenario_dir(self):
        try:
            share = get_package_share_directory('ar_project')
        except Exception as e:
            raise SetupError('cannot resolve the ar_project share dir (%s); '
                             'pass scenario_dir explicitly' % e)
        return Path(share) / 'config' / 'scenarios'

    def _resolve_paths(self):
        if not self.p_scenario:
            raise SetupError("parameter 'scenario' is empty -- nothing to run")
        base = Path(self.p_scenario_dir).expanduser() if self.p_scenario_dir \
            else self._default_scenario_dir()
        cand = Path(self.p_scenario).expanduser()
        # Accept either a bare scenario id or a full path to the YAML.
        if cand.suffix in ('.yaml', '.yml') or cand.is_absolute() or len(cand.parts) > 1:
            spath = cand
        else:
            spath = base / ('%s.yaml' % self.p_scenario)
        if not spath.is_file():
            raise SetupError('scenario file not found: %s' % spath)
        ppath = base / 'perturbations' / ('%s.yaml' % self.p_perturbation)
        return base, spath, ppath

    def _load_yaml(self, path, what):
        try:
            with path.open('r', encoding='utf-8') as fh:
                data = yaml.safe_load(fh)
        except Exception as e:
            raise SetupError('cannot parse %s %s: %s' % (what, path, e))
        if data is None:
            data = {}
        if not isinstance(data, dict):
            raise SetupError('%s %s must be a YAML mapping, got %s'
                             % (what, path, type(data).__name__))
        return data

    def _load_and_merge(self):
        base, spath, ppath = self._resolve_paths()
        scen = self._load_yaml(spath, 'scenario')

        sid = str(scen.get('id', '') or '').strip()
        stem = spath.stem
        if sid and sid != stem:
            # The schema requires id == filename stem; disagreeing means one of the two
            # is a copy/paste leftover and episode_key would mislabel the datapoint.
            raise SetupError("scenario id %r does not match filename stem %r in %s"
                             % (sid, stem, spath))
        self.scenario_id = _safe_token(sid or stem)

        if ppath.is_file():
            pert = self._load_yaml(ppath, 'perturbation')
        elif self.p_perturbation == 'p_none':
            # p_none is the identity overlay; tolerate it not existing on disk so the
            # unperturbed control arm can always run.  Any OTHER missing overlay is an
            # authoring error -- running it as identity would mislabel the episode.
            self.get_logger().warn('no overlay file at %s; using the identity overlay for p_none'
                                   % ppath)
            pert = {'id': 'p_none'}
        else:
            raise SetupError('perturbation overlay not found: %s' % ppath)

        pid = str(pert.get('id', '') or '').strip()
        if pid and pid != self.p_perturbation:
            raise SetupError("perturbation id %r does not match requested %r in %s"
                             % (pid, self.p_perturbation, ppath))
        self.perturbation_id = _safe_token(self.p_perturbation)
        self.episode_key = self._make_key()

        # --- merge: union lights_off (order preserved), concatenate spawns, scale timeout
        lights = []
        for name in list(scen.get('lights_off') or []) + list(pert.get('lights_off') or []):
            name = str(name)
            if name not in lights:
                lights.append(name)

        spawn = [dict(e) for e in (scen.get('spawn') or []) if isinstance(e, dict)]
        for e in (pert.get('spawn') or []):
            if not isinstance(e, dict):
                continue
            e = dict(e)
            # 'p_' prefix guarantees an overlay prop can never collide with a scenario
            # prop of the same name (gz create would otherwise silently rename or fail).
            e['name'] = 'p_%s' % e.get('name', 'prop')
            e['_from_overlay'] = True
            spawn.append(e)

        try:
            tscale = float(pert.get('timeout_scale', 1.0))
        except (TypeError, ValueError):
            raise SetupError('perturbation timeout_scale is not a number: %r'
                             % pert.get('timeout_scale'))
        if tscale <= 0.0:
            raise SetupError('perturbation timeout_scale must be > 0, got %r' % tscale)

        merged = dict(scen)
        merged['lights_off'] = lights
        merged['spawn'] = spawn
        try:
            merged['timeout_s'] = float(scen.get('timeout_s', 0.0)) * tscale
        except (TypeError, ValueError):
            raise SetupError('scenario timeout_s is not a number: %r' % scen.get('timeout_s'))
        merged['camera'] = dict(pert.get('camera') or {})
        merged['timeout_scale'] = tscale
        self.merged = merged

        self.get_logger().info(
            'merged scenario %s + %s: %d prop(s), %d light(s) off, timeout %.1fs '
            '(=%.1f x %.2f)'
            % (self.scenario_id, self.perturbation_id, len(spawn), len(lights),
               merged['timeout_s'], float(scen.get('timeout_s', 0.0)), tscale))

    def _validate(self):
        m = self.merged
        self.mission = str(m.get('mission', '') or '')
        if self.publish_mission and not self.mission:
            raise SetupError('scenario has no `mission` string but publish_mission is true')

        self.timeout_s = float(m.get('timeout_s', 0.0))
        if self.timeout_s <= 0.0:
            raise SetupError('scenario timeout_s must be > 0, got %r' % self.timeout_s)
        try:
            self.max_path_m = float(m.get('max_path_m', 0.0))
        except (TypeError, ValueError):
            raise SetupError('scenario max_path_m is not a number: %r' % m.get('max_path_m'))

        # Anchor for _to_world: odom/map are zeroed at the spawn pose, so the scenario's
        # own robot_start IS the transform from the localisation frame to the world frame.
        start = m.get('robot_start') or {}
        if not isinstance(start, dict):
            raise SetupError('robot_start must be a mapping, got %s' % type(start).__name__)
        try:
            self._start_x = float(start.get('x', 0.0))
            self._start_y = float(start.get('y', 0.0))
            self._start_yaw = float(start.get('yaw', 0.0))
        except (TypeError, ValueError):
            raise SetupError('robot_start has non-numeric x/y/yaw: %r' % (start,))

        self.lights_off = list(m.get('lights_off') or [])
        for name in self.lights_off:
            if name not in LIGHT_POSES and not self._scenario_light_pose(name):
                raise SetupError('lights_off names unknown light %r (not in LIGHT_POSES and no '
                                 'light_poses override in the scenario)' % name)

        cam = dict(CAMERA_PROFILE_DEFAULTS)
        for k, v in (m.get('camera') or {}).items():
            if k not in CAMERA_PROFILE_DEFAULTS:
                raise SetupError('perturbation camera has unknown key %r (allowed: %s)'
                                 % (k, ', '.join(sorted(CAMERA_PROFILE_DEFAULTS))))
            cam[k] = int(v) if k == 'motion_blur_px' else float(v)
        self.camera_profile = cam

        subgoals = list(m.get('subgoals') or [])
        if not subgoals:
            raise SetupError('scenario declares no subgoals -- progress would be undefined')
        seen_ids = set()
        for i, sg in enumerate(subgoals):
            if not isinstance(sg, dict):
                raise SetupError('subgoal #%d is not a mapping' % i)
            sid = str(sg.get('id', '') or '')
            if not sid:
                raise SetupError('subgoal #%d has no id' % i)
            if sid in seen_ids:
                raise SetupError('duplicate subgoal id %r' % sid)
            seen_ids.add(sid)
            stype = str(sg.get('type', '') or '')
            if stype not in PREDICATE_TYPES:
                raise SetupError('subgoal %r has unknown type %r (allowed: %s)'
                                 % (sid, stype, ', '.join(PREDICATE_TYPES)))
            if stype == 'near_xy':
                self._need(sg, sid, ('x', 'y', 'radius'))
            elif stype == 'in_room':
                room = str(sg.get('room', '') or '')
                if room not in ROOMS:
                    raise SetupError('subgoal %r names unknown room %r (allowed: %s)'
                                     % (sid, room, ', '.join(sorted(ROOMS))))
            elif stype == 'saw_label':
                if not str(sg.get('label', '') or ''):
                    raise SetupError('subgoal %r (saw_label) has no label' % sid)
            elif stype == 'faced_xy':
                self._need(sg, sid, ('x', 'y', 'tol_rad'))
        self.subgoals = subgoals
        self._sg_done = [False] * len(subgoals)
        self._sg_index = dict((str(sg['id']), i) for i, sg in enumerate(subgoals))

        succ = m.get('success') or {'type': 'all_subgoals'}
        if not isinstance(succ, dict):
            raise SetupError('scenario `success` must be a mapping')
        stype = str(succ.get('type', 'all_subgoals'))
        if stype not in ('all_subgoals', 'subgoal'):
            raise SetupError('scenario success.type must be all_subgoals or subgoal, got %r' % stype)
        if stype == 'subgoal':
            sid = str(succ.get('id', '') or '')
            if sid not in self._sg_index:
                raise SetupError('success.id %r is not one of the subgoal ids' % sid)
        self.success = succ

        for i, e in enumerate(m.get('spawn') or []):
            for key in ('model', 'name'):
                if not str(e.get(key, '') or ''):
                    raise SetupError('spawn entry #%d has no %s' % (i, key))
            self._need(e, 'spawn %r' % e['name'], ('x', 'y'))
        names = [e['name'] for e in (m.get('spawn') or [])]
        dupes = set(n for n in names if names.count(n) > 1)
        if dupes:
            raise SetupError('duplicate spawn name(s): %s' % ', '.join(sorted(dupes)))

    @staticmethod
    def _need(d, who, keys):
        for k in keys:
            if k not in d:
                raise SetupError('%s is missing required key %r' % (who, k))
            try:
                float(d[k])
            except (TypeError, ValueError):
                raise SetupError('%s has non-numeric %r: %r' % (who, k, d[k]))

    def _scenario_light_pose(self, name):
        """Optional per-scenario light pose override, as {name: [x,y,z]} or {name:{x,y,z}}."""
        table = self.merged.get('light_poses') or {}
        if name not in table:
            return None
        v = table[name]
        if isinstance(v, dict):
            return (float(v.get('x', 0.0)), float(v.get('y', 0.0)), float(v.get('z', 2.0)))
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            return (float(v[0]), float(v[1]), float(v[2]))
        raise SetupError('light_poses[%r] must be {x,y,z} or [x,y,z], got %r' % (name, v))

    # ------------------------------------------------------------------ arming

    def _wait_for_clock(self, timeout_wall_s=60.0):
        if not self.use_sim_time:
            self.get_logger().warn('use_sim_time is FALSE -- scoring against wall time. '
                                   'Benchmark episodes are meant to run on /clock.')
            return
        self.get_logger().info('waiting for /clock to advance (use_sim_time=true)...')
        deadline = time.monotonic() + timeout_wall_s
        first = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = self._sim_now()
            if now <= 0.0:
                continue
            if first is None:
                first = now
                continue
            if now > first:
                self.get_logger().info('/clock is live (sim t=%.3f)' % now)
                return
        raise SetupError('/clock never advanced within %.0fs -- is Gazebo running and is the '
                         'clock bridged?' % timeout_wall_s)

    def _resolve_pose_source(self, timeout_wall_s=30.0):
        """Pick the best available pose source and say so LOUDLY.

        Preference order is map (SLAM-corrected, what the predicates are authored
        against) > odom TF > the raw /odom topic.  The chosen source lands in the
        episode record because it changes how comparable the metrics are.
        """
        self.get_logger().info('resolving pose source: TF %s->%s, then TF %s->%s, then %s'
                               % (self.map_frame, self.robot_frame, self._odom_frame,
                                  self.robot_frame, self.odom_topic))
        deadline = time.monotonic() + timeout_wall_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._can_tf(self.map_frame):
                self.pose_source = 'tf_map'
                self.get_logger().info('POSE SOURCE = TF %s->%s (preferred: SLAM-corrected)'
                                       % (self.map_frame, self.robot_frame))
                return
            if self._can_tf(self._odom_frame):
                # Keep looking for map for a bit; odom is a fallback, not a tie.
                if time.monotonic() > deadline - (timeout_wall_s * 0.5):
                    self.pose_source = 'tf_odom'
                    self.get_logger().warn(
                        'POSE SOURCE = TF %s->%s -- map->%s never appeared. Positions are '
                        'ODOMETRIC and will drift; predicates were authored in map coords.'
                        % (self._odom_frame, self.robot_frame, self.robot_frame))
                    return
            if self._odom is not None and time.monotonic() > deadline - (timeout_wall_s * 0.25):
                self.pose_source = 'odom_topic'
                self.get_logger().warn(
                    'POSE SOURCE = %s topic -- no usable TF at all. Positions are RAW ODOMETRY.'
                    % self.odom_topic)
                return
        raise SetupError('no robot pose source within %.0fs: no TF %s->%s, no TF %s->%s, '
                         'no message on %s' % (timeout_wall_s, self.map_frame, self.robot_frame,
                                               self._odom_frame, self.robot_frame, self.odom_topic))

    def _can_tf(self, frame):
        try:
            return self._tf.can_transform(frame, self.robot_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException, tf2_ros.TransformException):
            return False

    def _to_world(self, pose):
        """Map a localisation-frame pose into the GAZEBO WORLD frame.

        THIS IS NOT COSMETIC. Every subgoal coordinate in the scenario YAML is written in
        the world frame (that is the only frame in which "the bathroom is at x in [-2, 2]"
        means anything), but neither odom nor map is that frame:

          * the gz DiffDrive plugin zeroes odom AT THE SPAWN POSE, so odom (0, 0) is world
            (-7, 0) for s1;
          * RTAB-Map likewise anchors `map` wherever SLAM started, i.e. the same spawn pose.

        Scoring odom coordinates against world targets offsets every predicate by the start
        pose. Measured: a smoke episode drove 29.65 m straight down the hallway, passed
        every waypoint, and still scored ordered_progress 0.000 -- with a non-zero start
        pose (all seven scenarios have one) the oracle could never fire.

        So compose the fixed start transform: world = R(yaw0) * local + (x0, y0).
        The cost is that odom/SLAM drift now enters the oracle; that is the honest price of
        scoring the robot's own estimate. Set pose_frame_is_world:=true if a source that is
        already world-frame (e.g. a bridged gz ground-truth pose) is ever wired in.
        """
        if pose is None or self.pose_frame_is_world:
            return pose
        px, py, pyaw = pose
        c, s = math.cos(self._start_yaw), math.sin(self._start_yaw)
        return (self._start_x + px * c - py * s,
                self._start_y + px * s + py * c,
                _wrap_pi(self._start_yaw + pyaw))

    def _sample_pose(self):
        """Return (x, y, yaw) in the WORLD frame, or None if the source is momentarily gone."""
        return self._to_world(self._sample_pose_local())

    def _sample_pose_local(self):
        """Raw (x, y, yaw) from the chosen source, in that source's own frame."""
        if self.pose_source in ('tf_map', 'tf_odom'):
            frame = self.map_frame if self.pose_source == 'tf_map' else self._odom_frame
            try:
                tr = self._tf.lookup_transform(frame, self.robot_frame, Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException, tf2_ros.TransformException) as e:
                if not self._pose_warned:
                    self._pose_warned = True
                    self.get_logger().warn('TF %s->%s temporarily unavailable: %s'
                                           % (frame, self.robot_frame, e))
                return None
            t = tr.transform.translation
            q = tr.transform.rotation
            return (t.x, t.y, _quat_to_yaw(q.x, q.y, q.z, q.w))
        if self._odom is None:
            return None
        p = self._odom.pose.pose.position
        q = self._odom.pose.pose.orientation
        return (p.x, p.y, _quat_to_yaw(q.x, q.y, q.z, q.w))

    # ------------------------------------------------------------------ dressing the world

    def _do_teleport(self):
        start = self.merged.get('robot_start')
        if not isinstance(start, dict):
            self.get_logger().warn('teleport_robot is true but the scenario has no robot_start; '
                                   'keeping wherever the launch spawned the robot')
            return
        x = float(start.get('x', 0.0))
        y = float(start.get('y', 0.0))
        yaw = float(start.get('yaw', 0.0))
        # Loud, deliberate warning: this is the discouraged path.
        self.get_logger().warn(
            'TELEPORTING %s to (%.2f, %.2f, yaw %.2f) via set_pose. WARNING: teleporting AFTER '
            'SLAM has started makes the map->odom correction jump, which corrupts the map and '
            'can inflate path_length_m. The PREFERRED path is to spawn the robot at the scenario '
            'pose at LAUNCH time and run with teleport_robot:=false.'
            % (self.robot_name, x, y, yaw))
        qx, qy, qz, qw = _yaw_to_quat(yaw)
        req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
               'orientation: {x: %s, y: %s, z: %s, w: %s}'
               % (self.robot_name, _fnum(x), _fnum(y), _fnum(ROBOT_SPAWN_Z),
                  _fnum(qx), _fnum(qy), _fnum(qz), _fnum(qw)))
        ok, detail = self._gz_retry('set_pose %s' % self.robot_name,
                                    '/world/%s/set_pose' % self.world_name,
                                    'gz.msgs.Pose', 'gz.msgs.Boolean', req)
        if not ok:
            raise SetupError('set_pose failed for %r: %s' % (self.robot_name, detail))
        self._pump(1.0)     # let the physics settle before anything is measured

    def _do_spawns(self):
        for e in (self.merged.get('spawn') or []):
            model = str(e['model'])
            name = str(e['name'])
            x, y = float(e['x']), float(e['y'])
            z = float(e.get('z', 0.0))
            yaw = float(e.get('yaw', 0.0))
            qx, qy, qz, qw = _yaw_to_quat(yaw)
            # allow_renaming stays FALSE on purpose: a silent rename on collision would
            # leave the episode running against a prop we cannot identify afterwards.
            req = ('sdf_filename: "model://%s", name: "%s", allow_renaming: false, '
                   'pose: {position: {x: %s, y: %s, z: %s}, '
                   'orientation: {x: %s, y: %s, z: %s, w: %s}}'
                   % (model, name, _fnum(x), _fnum(y), _fnum(z),
                      _fnum(qx), _fnum(qy), _fnum(qz), _fnum(qw)))
            ok, detail = self._gz_retry('create %s (%s)' % (name, model),
                                        '/world/%s/create' % self.world_name,
                                        'gz.msgs.EntityFactory', 'gz.msgs.Boolean', req)
            if not ok:
                # Never continue with a missing target: the episode would score a
                # planner failure that is really our failure.
                raise SetupError('spawn failed for %r (model://%s) after %d attempts: %s'
                                 % (name, model, GZ_ATTEMPTS, detail))
            self.spawned[name] = {'model': model, 'x': x, 'y': y, 'z': z, 'yaw': yaw,
                                  'from_overlay': bool(e.get('_from_overlay', False))}
            self.get_logger().info('spawned %s (model://%s) at (%.2f, %.2f, %.2f) yaw %.2f'
                                   % (name, model, x, y, z, yaw))

    def _do_lights(self):
        for name in self.lights_off:
            pose = self._scenario_light_pose(name) or LIGHT_POSES.get(name)
            if pose is None:
                raise SetupError('no pose known for light %r' % name)
            lx, ly, lz = pose
            # Every field is re-sent: light_config REPLACES the config, so omitting
            # attenuation or pose would move/brighten the light instead of dimming it.
            req = ('name: "%s", type: POINT, cast_shadows: false, '
                   'diffuse: {r: 0, g: 0, b: 0, a: 1}, specular: {r: 0, g: 0, b: 0, a: 1}, '
                   'range: %s, attenuation_constant: %s, attenuation_linear: %s, '
                   'attenuation_quadratic: %s, pose: {position: {x: %s, y: %s, z: %s}, '
                   'orientation: {x: 0, y: 0, z: 0, w: 1}}'
                   % (name, _fnum(LIGHT_RANGE), _fnum(LIGHT_ATT_CONSTANT),
                      _fnum(LIGHT_ATT_LINEAR), _fnum(LIGHT_ATT_QUADRATIC),
                      _fnum(lx), _fnum(ly), _fnum(lz)))
            ok, detail = self._gz_retry('light_config %s' % name,
                                        '/world/%s/light_config' % self.world_name,
                                        'gz.msgs.Light', 'gz.msgs.Boolean', req)
            if not ok:
                # Fatal for the same reason a missing prop is: "lights off" IS the
                # perturbation, so an episode that ran lit but is labelled dark
                # poisons its whole matched pair.
                raise SetupError('light_config failed for %r after %d attempts: %s'
                                 % (name, GZ_ATTEMPTS, detail))
            self.get_logger().info('dimmed light %s at (%.2f, %.2f, %.2f)' % (name, lx, ly, lz))

    def _do_profile(self):
        msg = String()
        # The episode seed is injected HERE rather than being a key of the overlay YAML:
        # the overlay describes the KIND of degradation, the seed decides which particular
        # smudge mask and which dropped frames realise it. Without this, every seed of the
        # same overlay saw a byte-identical smudge pattern and "3 seeds" measured one
        # sample three times -- the paired statistics would look sound and mean nothing.
        profile = dict(self.camera_profile)
        profile['seed'] = int(self.seed)
        msg.data = json.dumps(profile, sort_keys=True)
        self._profile_pub.publish(msg)
        self.get_logger().info('published camera perturbation profile (latched): %s' % msg.data)
        self._pump(0.5)

    # ------------------------------------------------------------------ setup

    def _setup(self):
        self._load_and_merge()
        self._validate()
        self._open_outputs()
        self._event('setup', episode_key=self.episode_key, scenario_id=self.scenario_id,
                    perturbation_id=self.perturbation_id, planner_label=self.planner_label,
                    seed=self.seed, mission=self.mission, world=self.world_name,
                    timeout_s=round(self.timeout_s, 3), max_path_m=self.max_path_m,
                    n_subgoals=len(self.subgoals),
                    subgoals=[{'id': str(sg['id']), 'type': str(sg['type'])} for sg in self.subgoals],
                    success=self.success, lights_off=list(self.lights_off),
                    camera=dict(self.camera_profile),
                    spawn=[{'name': e['name'], 'model': e['model']}
                           for e in (self.merged.get('spawn') or [])])

        self._wait_for_clock()

        if self.start_delay_s > 0.0:
            self.get_logger().info('grace period: %.1fs for the stack to settle before setup'
                                   % self.start_delay_s)
            self._pump(self.start_delay_s)

        self._resolve_pose_source()

        if self.teleport_robot:
            self._do_teleport()
        else:
            self.get_logger().info('teleport_robot=false: using the launch-time spawn pose '
                                   '(preferred -- no map->odom jump)')
        if self.spawn_props:
            self._do_spawns()
        else:
            self.get_logger().warn('spawn_props=false: NO props created. Any subgoal that '
                                   'depends on a prop cannot be satisfied.')
        if self.apply_lights:
            self._do_lights()
        else:
            self.get_logger().warn('apply_lights=false: lights_off %s NOT applied.'
                                   % (self.lights_off or '[]'))
        self._do_profile()
        self._save_spawn_probe()

        self._event('setup', phase='ready', pose_source=self.pose_source,
                    spawned=dict(self.spawned), lights_off=list(self.lights_off))

    def _save_spawn_probe(self):
        """Save one FRESH camera frame (subscribed only now, i.e. rendered after
        the props were spawned) next to the episode record. Pure evidence for the
        gz runtime-spawn render race (see the parameter comment); never gates or
        fails the episode -- a probe error is logged and swallowed."""
        if not self.spawn_probe_topic:
            return
        try:
            import numpy as np
            from sensor_msgs.msg import Image
            from rclpy.qos import qos_profile_sensor_data
            box = {}

            def _cb(msg):
                if 'msg' not in box:
                    box['msg'] = msg
            sub = self.create_subscription(Image, self.spawn_probe_topic, _cb,
                                           qos_profile_sensor_data)
            t0 = time.monotonic()
            while 'msg' not in box and time.monotonic() - t0 < 6.0:
                rclpy.spin_once(self, timeout_sec=0.2)
            self.destroy_subscription(sub)
            if 'msg' not in box:
                self.get_logger().warn('spawn probe: no frame on %s within 6s '
                                       '(camera pipeline dead at t0?)'
                                       % self.spawn_probe_topic)
                self._event('spawn_probe', saved=False, topic=self.spawn_probe_topic)
                return
            m = box['msg']
            arr = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, -1)
            path = self.out_dir / ('%s__spawn_probe.png' % self.episode_key)
            import cv2
            bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if m.encoding == 'rgb8' else arr
            cv2.imwrite(str(path), bgr)
            self.get_logger().info('spawn probe: %dx%d %s mean=%.1f -> %s'
                                   % (m.width, m.height, m.encoding,
                                      float(arr.mean()), path))
            self._event('spawn_probe', saved=True, mean=round(float(arr.mean()), 1),
                        path=str(path))
        except Exception as exc:                                # pragma: no cover
            self.get_logger().warn('spawn probe failed (non-fatal): %r' % (exc,))

    # ------------------------------------------------------------------ predicates

    def _predicate(self, sg, pose):
        stype = str(sg['type'])
        if stype == 'mission_done':
            return bool(self._mission_end)
        if stype == 'saw_label':
            want = str(sg['label']).strip().lower()
            try:
                min_conf = float(sg.get('min_conf', 0.0))
            except (TypeError, ValueError):
                min_conf = 0.0
            for label, score in self._best_conf_by_label.items():
                if score < min_conf:
                    continue
                # Open-vocabulary detectors phrase things loosely ('a sports ball'),
                # so accept an exact match or the wanted label as a substring.
                if label == want or want in label:
                    return True
            return False
        if pose is None:
            return False
        px, py, pyaw = pose
        if stype == 'near_xy':
            return math.hypot(px - float(sg['x']), py - float(sg['y'])) <= float(sg['radius'])
        if stype == 'in_room':
            x0, x1, y0, y1 = ROOMS[str(sg['room'])]
            return (x0 <= px <= x1) and (y0 <= py <= y1)
        if stype == 'faced_xy':
            bearing = math.atan2(float(sg['y']) - py, float(sg['x']) - px)
            return abs(_wrap_pi(bearing - pyaw)) <= float(sg['tol_rad'])
        return False

    def _ordered_k(self):
        k = 0
        for done in self._sg_done:
            if not done:
                break
            k += 1
        return k

    def _success_reached(self):
        if str(self.success.get('type', 'all_subgoals')) == 'subgoal':
            return self._sg_done[self._sg_index[str(self.success['id'])]]
        return all(self._sg_done)

    # ------------------------------------------------------------------ the run loop

    def _await_mission_subscriber(self):
        """Block (bounded, spinning) until something subscribes to /vlm_mission.

        The publisher is TRANSIENT_LOCAL, but planner_orchestrator subscribes with the
        default VOLATILE profile -- and a volatile late joiner does NOT get the latched
        sample. So a mission published one second too early is dropped on the floor and
        the episode times out with an empty /vlm/activity trace, which looks exactly
        like "the planner did nothing" in the report. That is the worst possible failure
        mode for a benchmark: a harness bug that scores as a planner failure.

        The race is real: vlm_sim_bringup starts the orchestrator at t+34 s and it then
        imports cv2/requests before subscribing, while runner_delay_s defaults to 45 s.
        The margin is only ~15 s and shrinks the moment anyone lowers runner_delay_s or
        runs on a slower machine. Waiting is free when the subscriber is already up.
        """
        if not self.publish_mission:
            return
        if self._mission_pub.get_subscription_count() > 0:
            return
        self.get_logger().info('waiting up to %.0fs for a /vlm_mission subscriber '
                               '(planner_orchestrator)...' % MISSION_SUB_WAIT_S)
        deadline = time.monotonic() + MISSION_SUB_WAIT_S
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._mission_pub.get_subscription_count() > 0:
                self.get_logger().info('/vlm_mission subscriber appeared after %.1fs'
                                       % (MISSION_SUB_WAIT_S - (deadline - time.monotonic())))
                return
        # Fail loud but do not abort: publishing anyway still produces a scorable
        # episode if the subscriber shows up microseconds later, and the warning makes
        # an empty trace attributable to the harness rather than to the planner.
        self.get_logger().error(
            'NO subscriber on /vlm_mission after %.0fs. Publishing anyway, but if the '
            'trace comes back empty this is why -- is planner_orchestrator running '
            '(start_edge:=true) and did it finish importing?' % MISSION_SUB_WAIT_S)

    def _publish_mission(self):
        self._await_mission_subscriber()
        # Stamped AFTER the wait so the scenario timeout budget covers the mission, not
        # the bring-up.
        self._t_sim_mission = self._sim_now()
        # Anything on /vlm/activity stamped before this instant belongs to a previous
        # mission replayed by TRANSIENT_LOCAL durability and must not be scored.
        self._activity_epoch_wall = time.time()
        if not self.publish_mission:
            self.get_logger().warn('publish_mission=false: NOT starting a mission; the episode '
                                   'will only observe (expect timeout unless something else '
                                   'drives the robot).')
            return
        msg = String()
        msg.data = self.mission
        self._mission_pub.publish(msg)
        self.get_logger().info('MISSION PUBLISHED on /vlm_mission: %r (timeout %.1fs sim)'
                               % (self.mission, self.timeout_s))

    def _loop(self):
        self._publish_mission()
        tick_dt = 1.0 / self.tick_hz
        next_tick = self._sim_now()
        last_pose_sample = -1e9
        last_sim = self._sim_now()
        last_advance_wall = time.monotonic()

        while rclpy.ok() and self._outcome is None:
            rclpy.spin_once(self, timeout_sec=0.01)
            now = self._sim_now()

            if now > last_sim:
                last_sim = now
                last_advance_wall = time.monotonic()
            elif time.monotonic() - last_advance_wall > CLOCK_STALL_S:
                self._outcome = 'aborted'
                self._outcome_reason = ('sim clock stalled for %.0fs of wall time at t_sim=%.2f '
                                        '(Gazebo died?)' % (CLOCK_STALL_S, now))
                break

            if now < next_tick:
                continue
            # Resync rather than burning through a backlog of ticks if sim time jumped.
            next_tick = now + tick_dt if (now - next_tick) > 1.0 else next_tick + tick_dt

            pose = self._tick(now)

            if pose is not None and (now - last_pose_sample) >= POSE_SAMPLE_PERIOD_S:
                last_pose_sample = now
                self._event('pose_sample', x=round(pose[0], 3), y=round(pose[1], 3),
                            yaw=round(pose[2], 4),
                            path_m=round(self.path_length_m, 3),
                            ordered=round(self._ordered_k() / float(len(self.subgoals)), 4))

    def _tick(self, now):
        pose = self._sample_pose()

        # --- path integration
        if pose is not None:
            if self._last_pose is not None:
                d = math.hypot(pose[0] - self._last_pose[0], pose[1] - self._last_pose[1])
                if d > MAX_TICK_STEP_M:
                    self.get_logger().warn('ignoring a %.2fm pose jump in one tick (SLAM '
                                           'correction, not motion)' % d)
                else:
                    self.path_length_m += d
            self._last_pose = pose
            if self.max_path_m > 0.0 and self.path_length_m > self.max_path_m \
                    and not self._path_budget_warned:
                self._path_budget_warned = True
                self.get_logger().warn('path budget exceeded: %.1fm > max_path_m %.1fm '
                                       '(soft budget -- logged, episode continues)'
                                       % (self.path_length_m, self.max_path_m))

        # --- subgoals: latch each predicate independently on its first firing
        for i, sg in enumerate(self.subgoals):
            if self._sg_done[i] or not self._predicate(sg, pose):
                continue
            in_order = all(self._sg_done[j] for j in range(i))
            self._sg_done[i] = True
            sid = str(sg['id'])
            self._sg_time[sid] = round(now - (self._t_sim_mission or now), 3)
            self._sg_step[sid] = self.vlm_steps
            self._event('subgoal_done', subgoal_id=sid, index=i, type=str(sg['type']),
                        in_order=in_order, t_since_mission_s=self._sg_time[sid],
                        vlm_step=self.vlm_steps,
                        x=(round(pose[0], 3) if pose else None),
                        y=(round(pose[1], 3) if pose else None))
            self.get_logger().info('SUBGOAL %s (%s) satisfied at t+%.1fs, step %d%s'
                                   % (sid, sg['type'], self._sg_time[sid], self.vlm_steps,
                                      '' if in_order else ' [OUT OF ORDER]'))

        # --- progress AUC: sampled every tick, so it rewards reaching progress early
        n = float(len(self.subgoals))
        self._auc_sum += self._ordered_k() / n
        self._auc_n += 1

        # --- collision watch
        self._check_collision(now)

        # --- termination
        if self._outcome is not None:
            return pose
        if self._success_reached():
            self._outcome = 'success'
            self._outcome_reason = ('success predicate satisfied (%s)'
                                    % self.success.get('type', 'all_subgoals'))
            return pose
        if self._mission_end:
            grace = now - (self._t_sim_mission_end or now)
            if grace >= MISSION_END_GRACE_S:
                self._outcome = 'planner_gave_up'
                self._outcome_reason = ('planner published mission_end with the success '
                                        'predicate unsatisfied (%d/%d subgoals)'
                                        % (sum(self._sg_done), len(self.subgoals)))
                return pose
        elapsed = now - (self._t_sim_mission or now)
        if elapsed >= self.timeout_s:
            self._outcome = 'timeout'
            self._outcome_reason = ('timeout after %.1fs of sim time (budget %.1fs)'
                                    % (elapsed, self.timeout_s))
        return pose

    def _check_collision(self, now):
        scan = self._scan
        if scan is None:
            return
        rmin = None
        lo = max(float(scan.range_min), 1e-3)
        hi = float(scan.range_max)
        for r in scan.ranges:
            # Drop inf/nan and out-of-band returns: depthimage_to_laserscan emits those
            # for "no return", and treating them as 0 would fire a phantom collision.
            if not math.isfinite(r) or r < lo or r > hi:
                continue
            if rmin is None or r < rmin:
                rmin = r
        if rmin is None:
            return
        if rmin < self.collision_range_m:
            self._collision_streak += 1
            if self._collision_streak >= self.collision_frames and not self.collided:
                self.collided = True
                self._event('collision', min_range_m=round(rmin, 4),
                            threshold_m=self.collision_range_m,
                            frames=self._collision_streak)
                self.get_logger().error('CATASTROPHIC COLLISION: /scan min %.3fm < %.3fm for %d '
                                        'consecutive ticks' % (rmin, self.collision_range_m,
                                                               self._collision_streak))
                self._outcome = 'collision'
                self._outcome_reason = ('scan minimum %.3fm below %.3fm for %d ticks'
                                        % (rmin, self.collision_range_m, self._collision_streak))
        else:
            self._collision_streak = 0

    # ------------------------------------------------------------------ finish

    def _finish(self, outcome, reason):
        if self._finished:
            return
        self._finished = True
        if outcome not in OUTCOMES:
            reason = '%s (invalid outcome %r coerced)' % (reason, outcome)
            outcome = 'aborted'
        self._outcome = outcome
        self._outcome_reason = reason
        self._t_sim_end = self._sim_now()

        try:
            self._open_outputs()
        except Exception as e:
            self.get_logger().error('cannot open the output files in %s: %s' % (self.out_dir, e))

        n = len(self.subgoals)
        ordered = (self._ordered_k() / float(n)) if n else 0.0
        unordered = (sum(1 for d in self._sg_done if d) / float(n)) if n else 0.0
        auc = (self._auc_sum / self._auc_n) if self._auc_n else 0.0
        first_fail = None
        for i, done in enumerate(self._sg_done):
            if not done:
                first_fail = str(self.subgoals[i]['id'])
                break

        sim_duration = 0.0
        if self._t_sim_mission is not None and self._t_sim_end is not None:
            sim_duration = max(0.0, self._t_sim_end - self._t_sim_mission)

        record = {
            'schema_version': SCHEMA_VERSION,
            'episode_key': self.episode_key,
            'scenario_id': self.scenario_id,
            'perturbation_id': self.perturbation_id,
            'planner_label': self.planner_label,
            'seed': self.seed,
            'mission': self.mission,
            'started_iso': self._started_iso,
            'wall_duration_s': round(time.time() - self._t_wall0, 3),
            'sim_duration_s': round(sim_duration, 3),
            'outcome': self._outcome,
            'outcome_reason': self._outcome_reason,
            'n_subgoals': n,
            'ordered_progress': round(ordered, 4),
            'unordered_progress': round(unordered, 4),
            'progress_auc': round(auc, 4),
            'first_failure_subgoal': first_fail,
            'subgoal_times': dict(self._sg_time),
            'path_length_m': round(self.path_length_m, 3),
            'max_path_m': self.max_path_m,
            'timeout_s': round(self.timeout_s, 3),
            'collided': bool(self.collided),
            'vlm_steps': self.vlm_steps,
            'action_histogram': dict(self.action_histogram),
            'detect_all_calls': self.detect_all_calls,
            'plan_failures': self.plan_failures,
            'degraded_events': self.degraded_events,
            'notes_events': self.notes_events,
            'pose_source': self.pose_source,
            # Needed to reinterpret a trace: subgoals are world-frame, the pose source is
            # not, and this is the transform that was composed between them (see _to_world).
            'robot_start': [self._start_x, self._start_y, self._start_yaw],
            'pose_frame_is_world': self.pose_frame_is_world,
            'spawned': dict(self.spawned),
            'lights_off': list(self.lights_off),
            # Carried through from the scenario so the report can group episodes by what
            # they diagnose (ocr, depth_gap, room_adjacency, ...) instead of by scenario id.
            # Without this the tags exist only in the YAML and never reach any analysis.
            'diagnoses': [str(t) for t in (self.merged.get('diagnoses') or [])],
            'camera_profile': dict(self.camera_profile),
        }

        self._event('finish', **record)
        if self._jsonl is not None:
            try:
                self._jsonl.close()
            except Exception as e:
                self.get_logger().error('failed to close the trace file: %s' % e)
            self._jsonl = None

        try:
            self.out_dir.mkdir(parents=True, exist_ok=True)
            path = self.out_dir / ('%s.json' % self.episode_key)
            with path.open('w', encoding='utf-8') as fh:
                json.dump(record, fh, ensure_ascii=False, indent=2, sort_keys=False)
                fh.write('\n')
            self.get_logger().info('episode record -> %s' % path)
        except Exception as e:
            self.get_logger().error('FAILED to write the episode record: %s' % e)

        self._print_summary(record)

    def _print_summary(self, r):
        n = r['n_subgoals']
        rows = [
            ('outcome', '%s -- %s' % (r['outcome'], r['outcome_reason'])),
            ('scenario / pert', '%s / %s' % (r['scenario_id'], r['perturbation_id'])),
            ('planner_label', '%s   (paired key: seed %d)' % (r['planner_label'], r['seed'])),
            ('mission', r['mission'] or '-'),
            ('ordered_progress', '%.3f  (%d/%d prefix)' % (r['ordered_progress'],
                                                           int(round(r['ordered_progress'] * n)), n)),
            ('unordered_progress', '%.3f  (%d/%d any order)' % (r['unordered_progress'],
                                                                int(round(r['unordered_progress'] * n)), n)),
            ('progress_auc', '%.3f' % r['progress_auc']),
            ('first_failure', r['first_failure_subgoal'] or '-'),
            ('duration', 'sim %.1fs / wall %.1fs (timeout %.1fs)'
             % (r['sim_duration_s'], r['wall_duration_s'], r['timeout_s'])),
            ('path_length_m', '%.2f  (budget %.2f)' % (r['path_length_m'], r['max_path_m'])),
            ('collided', str(r['collided'])),
            ('pose_source', r['pose_source']),
            ('vlm_steps', '%d  %s' % (r['vlm_steps'], r['action_histogram'] or '{}')),
            ('detect_all / fails', '%d / %d plan_failed' % (r['detect_all_calls'], r['plan_failures'])),
            ('degraded / notes', '%d / %d' % (r['degraded_events'], r['notes_events'])),
            ('spawned', ', '.join(sorted(r['spawned'])) or '-'),
            ('lights_off', ', '.join(r['lights_off']) or '-'),
        ]
        width = 78
        lines = ['', '=' * width, ' EPISODE  %s' % r['episode_key'], '-' * width]
        for k, v in rows:
            lines.append('  %-20s %s' % (k, v))
        if self._sg_time or n:
            lines.append('-' * width)
            lines.append('  %-20s %s' % ('subgoals', 'id / type / t+s'))
            for i, sg in enumerate(self.subgoals):
                sid = str(sg['id'])
                mark = 'OK ' if self._sg_done[i] else '.. '
                t = self._sg_time.get(sid)
                lines.append('  %-20s %s%-18s %-14s %s'
                             % ('', mark, sid, sg['type'],
                                ('t+%.1fs' % t) if t is not None else '-'))
        lines.append('=' * width)
        print('\n'.join(lines), flush=True)

    # ------------------------------------------------------------------ entry point

    def run_episode(self):
        try:
            self._setup()
        except SetupError as e:
            self.get_logger().error('SETUP FAILED: %s' % e)
            self._finish('setup_failed', str(e))
            return
        except Exception as e:
            self.get_logger().error('SETUP FAILED (unexpected %s): %s' % (type(e).__name__, e))
            self._finish('setup_failed', 'unexpected %s: %s' % (type(e).__name__, e))
            return

        try:
            self._loop()
        except KeyboardInterrupt:
            self.get_logger().warn('interrupted by the operator')
            self._finish('aborted', 'KeyboardInterrupt')
            return
        except Exception as e:
            self.get_logger().error('EPISODE ABORTED (unexpected %s): %s' % (type(e).__name__, e))
            self._finish('aborted', 'unexpected %s: %s' % (type(e).__name__, e))
            return

        if self._outcome is None:
            # rclpy went down under us (external shutdown) before any terminal condition.
            self._finish('aborted', 'rclpy shut down before a terminal condition was reached')
        else:
            self._finish(self._outcome, self._outcome_reason)


def main(args=None):
    rclpy.init(args=args)
    node = None
    outcome = 'aborted'
    shutdown_on_finish = True
    try:
        node = HouseScenarioRunner()
        shutdown_on_finish = node.shutdown_on_finish
        node.run_episode()
        outcome = node._outcome or 'aborted'
    except KeyboardInterrupt:
        outcome = 'aborted'
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    # Only a setup failure is a PROCESS failure: a timeout, a collision or a planner
    # giving up are legitimate benchmark results and the sweep driver must keep going.
    if shutdown_on_finish and outcome == 'setup_failed':
        sys.exit(1)


if __name__ == '__main__':
    main()
