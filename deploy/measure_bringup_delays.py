#!/usr/bin/env python3
"""Measure the REAL per-stage bring-up delays of mission_bringup on the robot.

WHY: `mission_bringup.launch.py` staggers the hardware stack with TimerAction
(`delay_realsense_s` / `delay_map_relay_s` / `delay_nav2_s` /
`delay_executive_s`). Those defaults were rescaled from the simulation ladder,
not measured on the bench, and the failure they guard against is a quiet one:
Nav2 costmaps coming up before `map_odom_relay` broadcasts map->odom.

HOW: this script OWNS the launch. It starts mission_bringup with the stages
spread far apart (`--spacing`, default 30 s) so that every stage boots on a Pi
that is no longer busy with the previous one, and timestamps the readiness
signal of each stage. The intrinsic boot time of a stage is then
`readiness - that stage's own start`, which is exactly what a TimerAction period
has to cover. Running with the CURRENT defaults instead would measure the
timers, not the hardware: no stage can become ready before its own timer fires,
so `/map_odom_correction` at t+5.1 s would only prove that
`delay_map_relay_s` is 5.0.

Readiness signal per stage (each one measured, none assumed):

  hardware_bringup  first `/joint_states` -- ros2_control up, which on this
                    robot means the lely master walked both EPOS4 drives to
                    Operation Enabled over CAN
  realsense         first `/camera/camera/color/camera_info`
  map_odom_relay    first map->odom on `/tf`. Identity counts: the relay
                    broadcasts identity until the edge sends a correction
                    (`map_odom_relay.py:17-19`), and identity is already enough
                    for Nav2's costmaps to have a transform
  nav2              all five lifecycle nodes ACTIVE (`navigation_launch.py:46-50`)
                    plus the `navigate_to_pose` action server
  executive         the `seek_object` action server (`seek_object_server.py:64-65`)

`/map_odom_correction` is watched too, because it is what turns that identity
transform into a real correction -- but it is published on the EDGE by
`rtabmap_map_odom_correction_publisher` (`edge_bringup.launch.py:155`). Under
`layer:=robot` alone it never arrives, and the report says so rather than
reporting a failed stage.

Usage (on the Pi, with ROS + workspace + transport_env.sh sourced, wheels on a
stand -- this brings the drives to Operation Enabled):

    python3 ~/ros2_ws/src/ar_project/deploy/measure_bringup_delays.py
    python3 ... --spacing 45 --json /tmp/bringup_delays.json
    python3 ... -- rgb_profile:=640x480x15     # extra launch args go after --
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import time

import rclpy
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, JointState
from tf2_msgs.msg import TFMessage

# navigation_launch.py:46-50. No map_server/amcl here: localization on this
# robot is map_odom_relay, not AMCL.
NAV2_LIFECYCLE_NODES = ('controller_server', 'planner_server', 'behavior_server',
                        'bt_navigator', 'velocity_smoother')

# Asking for BEST_EFFORT/VOLATILE makes the subscription compatible with every
# publisher in this stack (a RELIABLE or TRANSIENT_LOCAL offer still satisfies
# it), so a measurement can never silently miss a topic over a QoS mismatch --
# which would read as "the stage never came up".
PERMISSIVE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

JOINT_STATES_TOPIC = '/joint_states'
CAMERA_INFO_TOPIC = '/camera/camera/color/camera_info'
CORRECTION_TOPIC = '/map_odom_correction'

# An action server is detected through the graph rather than through an
# ActionClient so that this script needs no message package beyond the core
# ones -- object_tracking_msgs (SeekObject) is an edge-side build target and is
# not guaranteed to be installed on the Pi.
ACTION_PROBES = (
    ('nav2_navigate_to_pose', '/navigate_to_pose/_action/send_goal'),
    ('executive_seek_object', '/seek_object/_action/send_goal'),
)


class BringupProbe(Node):
    """Stamps the first occurrence of every readiness signal, relative to t0."""

    def __init__(self, t0):
        super().__init__('measure_bringup_delays')
        self.t0 = t0
        self.marks = {}
        self.correction_available = True

        self.create_subscription(JointState, JOINT_STATES_TOPIC,
                                 lambda _m: self.mark('joint_states'), PERMISSIVE_QOS)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC,
                                 lambda _m: self.mark('camera_info'), PERMISSIVE_QOS)
        self.create_subscription(TFMessage, '/tf', self._on_tf, PERMISSIVE_QOS)

        try:
            from ar_project_msgs.msg import MapOdomCorrection
            self.create_subscription(MapOdomCorrection, CORRECTION_TOPIC,
                                     lambda _m: self.mark('map_odom_correction'),
                                     PERMISSIVE_QOS)
        except ImportError:
            # Not fatal: the correction is an edge-layer signal anyway.
            self.correction_available = False

        self._state_clients = {n: self.create_client(GetState, '/%s/get_state' % n)
                               for n in NAV2_LIFECYCLE_NODES}
        self._state_futures = {n: None for n in NAV2_LIFECYCLE_NODES}
        self.create_timer(0.2, self._poll)

    def mark(self, key):
        if key not in self.marks:
            self.marks[key] = time.monotonic() - self.t0
            print('  [probe] %-24s t+%7.2f s' % (key, self.marks[key]), flush=True)

    def _on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id.lstrip('/') == 'map' and tf.child_frame_id.lstrip('/') == 'odom':
                self.mark('map_odom_tf')
                return

    def _poll(self):
        for name in NAV2_LIFECYCLE_NODES:
            key = 'nav2_active:' + name
            if key in self.marks:
                continue
            future = self._state_futures[name]
            if future is not None:
                if not future.done():
                    continue
                self._state_futures[name] = None
                try:
                    response = future.result()
                except Exception:
                    response = None
                if response is not None and response.current_state.id == State.PRIMARY_STATE_ACTIVE:
                    self.mark(key)
                    continue
            if self._state_clients[name].service_is_ready():
                self._state_futures[name] = self._state_clients[name].call_async(GetState.Request())

        services = dict(self.get_service_names_and_types())
        for key, service in ACTION_PROBES:
            if key not in self.marks and service in services:
                self.mark(key)

    def nav2_ready_at(self):
        """When every Nav2 lifecycle node was ACTIVE *and* the BT server answered."""
        keys = ['nav2_active:' + n for n in NAV2_LIFECYCLE_NODES] + ['nav2_navigate_to_pose']
        if any(k not in self.marks for k in keys):
            return None
        return max(self.marks[k] for k in keys)

    def done(self):
        # /map_odom_correction is deliberately NOT required: it belongs to the
        # edge layer and never arrives under layer:=robot.
        return (self.nav2_ready_at() is not None
                and all(k in self.marks for k in
                        ('joint_states', 'camera_info', 'map_odom_tf', 'executive_seek_object')))


def _ceil_half(value):
    return math.ceil(value * 2.0) / 2.0


def _shutdown_launch(proc):
    """SIGINT first, and never SIGKILL: ros2_control must run its shutdown so the
    EPOS4 drives leave Operation Enabled instead of staying energized."""
    if proc.poll() is not None:
        return
    print('\n[measure] stopping the stack (SIGINT to the launch process group)...', flush=True)
    for sig, grace in ((signal.SIGINT, 30.0), (signal.SIGTERM, 15.0)):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
        except (ProcessLookupError, PermissionError):
            return
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return
            time.sleep(0.5)
    print('[measure] WARNING: the launch did not exit. Stop it by hand and check the '
          'drives before the next run; do NOT kill -9 (it leaves Fast DDS shared '
          'memory segments behind).', flush=True)


def _report(probe, stages, margin, spacing):
    marks = probe.marks
    print('\n' + '=' * 72)
    print('MEASURED (seconds from launch start)')
    print('=' * 72)
    for key in ('joint_states', 'camera_info', 'map_odom_tf', 'map_odom_correction',
                'executive_seek_object'):
        if key in marks:
            print('  %-24s t+%7.2f s' % (key, marks[key]))
        elif key == 'map_odom_correction':
            reason = ('ar_project_msgs missing' if not probe.correction_available
                      else 'edge layer not running -- expected under layer:=robot')
            print('  %-24s  n/a  (%s)' % (key, reason))
        else:
            print('  %-24s  NOT SEEN' % key)
    for name in NAV2_LIFECYCLE_NODES:
        key = 'nav2_active:' + name
        print('  %-24s %s' % ('nav2 ' + name,
                              ('t+%7.2f s' % marks[key]) if key in marks else ' NOT SEEN'))
    nav2_ready = probe.nav2_ready_at()

    # Intrinsic boot time of each stage = readiness minus that stage's own start.
    # Rounded because the probe polls at 5 Hz anyway: without it, subtracting two
    # floats leaves noise that can push a boundary value up a whole 0.5 s step.
    boots = {}
    if 'joint_states' in marks:
        boots['hardware'] = round(marks['joint_states'], 3)
    if 'camera_info' in marks:
        boots['realsense'] = round(marks['camera_info'] - stages['realsense'], 3)
    if 'map_odom_tf' in marks:
        boots['map_relay'] = round(marks['map_odom_tf'] - stages['map_relay'], 3)
    if nav2_ready is not None:
        boots['nav2'] = round(nav2_ready - stages['nav2'], 3)
    if 'executive_seek_object' in marks:
        boots['executive'] = round(marks['executive_seek_object'] - stages['executive'], 3)

    print('\n' + '=' * 72)
    print('STAGE BOOT TIME (readiness minus that stage\'s own start)')
    print('=' * 72)
    for key in ('hardware', 'realsense', 'map_relay', 'nav2', 'executive'):
        if key in boots:
            flag = '  <-- EXCEEDS --spacing, rerun with a larger one' \
                if boots[key] > spacing else ''
            print('  %-12s %7.2f s%s' % (key, boots[key], flag))
        else:
            print('  %-12s  NOT MEASURED' % key)

    required = ('hardware', 'realsense', 'map_relay', 'nav2')
    if any(k not in boots for k in required):
        print('\n[measure] Some stage never became ready, so no delays are recommended: '
              'a guessed number here would be exactly the problem this run exists to fix. '
              'Check the launch output above for the stage that failed and rerun.')
        return boots, None

    # Cumulative, because each stage may only start once the previous one has
    # stopped competing for the Pi's CPU -- and map_odom_relay must additionally
    # be broadcasting before Nav2 builds its costmaps.
    delays = {}
    delays['delay_realsense_s'] = _ceil_half(margin * boots['hardware'])
    delays['delay_map_relay_s'] = _ceil_half(
        delays['delay_realsense_s'] + margin * boots['realsense'])
    delays['delay_nav2_s'] = _ceil_half(
        delays['delay_map_relay_s'] + margin * boots['map_relay'])
    delays['delay_executive_s'] = _ceil_half(
        delays['delay_nav2_s'] + margin * boots['nav2'])

    print('\n' + '=' * 72)
    print('RECOMMENDED default_value (measured x %.2f, rounded up to 0.5 s)' % margin)
    print('=' * 72)
    for key in ('delay_realsense_s', 'delay_map_relay_s', 'delay_nav2_s', 'delay_executive_s'):
        print('  %-20s %6.1f' % (key, delays[key]))
    print('\n  Verify in one run:')
    print('  ros2 launch ar_project mission_bringup.launch.py mode:=hardware layer:=robot \\')
    print('    ' + ' '.join('%s:=%.1f' % (k, delays[k]) for k in (
        'delay_realsense_s', 'delay_map_relay_s', 'delay_nav2_s', 'delay_executive_s')))
    if 'executive' in boots:
        print('\n  Note: the executive took %.2f s to expose seek_object. coordinator_node does '
              'NOT block on Nav2\n  at startup (skills.py:246-249 waits per drive, '
              'explore_nav_ready_timeout_s=10.0), so\n  delay_executive_s only has to keep the '
              'two stages from booting on top of each other.' % boots['executive'])
    return boots, delays


def main():
    parser = argparse.ArgumentParser(
        description='Measure the real per-stage bring-up delays of mission_bringup on the robot.')
    parser.add_argument('--spacing', type=float, default=30.0,
                        help='seconds between stage starts during the measurement run; must '
                             'exceed every stage boot time (default: 30)')
    parser.add_argument('--settle', type=float, default=60.0,
                        help='extra seconds to wait after the last stage starts (default: 60)')
    parser.add_argument('--margin', type=float, default=1.5,
                        help='safety factor applied to the measured times (default: 1.5)')
    parser.add_argument('--json', default='', metavar='PATH',
                        help='also write the raw measurement to this file')
    parser.add_argument('--yes', action='store_true',
                        help='skip the confirmation prompt')
    parser.add_argument('launch_args', nargs='*', metavar='ARG',
                        help='extra launch arguments, after --  (e.g. rgb_profile:=640x480x15)')
    args = parser.parse_args()

    stages = {
        'realsense': args.spacing,
        'map_relay': 2.0 * args.spacing,
        'nav2': 3.0 * args.spacing,
        'executive': 4.0 * args.spacing,
    }
    total = stages['executive'] + args.settle

    print('This starts the REAL stack: the EPOS4 drives will go to Operation Enabled.')
    print('Put the wheels on a stand first. The run takes about %.0f s.' % total)
    if not args.yes:
        try:
            if input('Continue? [y/N] ').strip().lower() not in ('y', 'yes'):
                print('aborted.')
                return 1
        except EOFError:
            print('aborted (no tty -- pass --yes to run unattended).')
            return 1

    cmd = ['ros2', 'launch', 'ar_project', 'mission_bringup.launch.py',
           'mode:=hardware', 'layer:=robot',
           'delay_realsense_s:=%g' % stages['realsense'],
           'delay_map_relay_s:=%g' % stages['map_relay'],
           'delay_nav2_s:=%g' % stages['nav2'],
           'delay_executive_s:=%g' % stages['executive']] + args.launch_args
    print('\n[measure] %s\n' % ' '.join(cmd), flush=True)

    # start_new_session so the whole launch tree can be signalled as one group.
    proc = subprocess.Popen(cmd, start_new_session=True)
    t0 = time.monotonic()

    rclpy.init()
    probe = BringupProbe(t0)
    executor = SingleThreadedExecutor()
    executor.add_node(probe)
    try:
        while time.monotonic() - t0 < total:
            executor.spin_once(timeout_sec=0.2)
            if probe.done():
                print('[measure] every stage is ready, stopping early.', flush=True)
                break
            if proc.poll() is not None:
                print('[measure] the launch exited on its own (rc=%s) -- the numbers below '
                      'are incomplete.' % proc.returncode, flush=True)
                break
    except KeyboardInterrupt:
        print('\n[measure] interrupted; reporting what was captured.', flush=True)
    finally:
        boots, delays = _report(probe, stages, args.margin, args.spacing)
        if args.json:
            with open(args.json, 'w') as handle:
                json.dump({'spacing_s': args.spacing, 'margin': args.margin,
                           'stage_starts_s': stages, 'marks_s': probe.marks,
                           'stage_boot_s': boots, 'recommended': delays},
                          handle, indent=2, sort_keys=True)
            print('\n[measure] raw measurement written to %s' % args.json)
        probe.destroy_node()
        rclpy.shutdown()
        _shutdown_launch(proc)
    return 0


if __name__ == '__main__':
    sys.exit(main())
