"""VLM-oriented simulation bring-up.

This is a thin, explicit wrapper around the simulated robot stack:

  - Gazebo world defaults to flat_detect.world (bus billboard target)
  - RViz and Gazebo GUI default to enabled for visual debugging
  - Optional edge-side detector + planner_orchestrator can be started from the
    ML Python environment; VLM credentials are inherited from the shell env.

The executive/search_coordinator is still started because it owns the safe skill
servers (GoToPose, ApproachDetection, Stop). The mission itself is VLM-driven:
publish a target on /vlm_mission rather than sending a FLAT /seek_object goal.
"""
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool_value(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _site_packages_for(package_name):
    prefix = get_package_prefix(package_name)
    return os.path.join(
        prefix,
        'lib',
        f'python{sys.version_info.major}.{sys.version_info.minor}',
        'site-packages',
    )


def _edge_env():
    env = dict(os.environ)
    env['PYTHONUNBUFFERED'] = '1'
    env.setdefault('HF_HOME', os.path.expanduser('~/.cache/huggingface'))
    env.setdefault('HF_HUB_DISABLE_PROGRESS_BARS', '1')
    env.setdefault('TRANSFORMERS_VERBOSITY', 'error')

    package_paths = [
        _site_packages_for('object_tracking'),
        _site_packages_for('planner_orchestrator'),
        _site_packages_for('object_tracking_msgs'),
        _site_packages_for('ar_project_msgs'),
        _site_packages_for('fleet_comms'),
    ]
    env['PYTHONPATH'] = os.pathsep.join(
        path for path in package_paths + [env.get('PYTHONPATH', '')] if path
    )
    return env


def _edge_processes(context, *args, **kwargs):
    if not _bool_value(LaunchConfiguration('start_edge').perform(context)):
        return []

    venv_python = os.path.expanduser(LaunchConfiguration('venv_python').perform(context))
    if not os.path.isfile(venv_python):
        raise RuntimeError(
            f'ML Python interpreter was not found: {venv_python}. '
            'Pass venv_python:=/path/to/python or start edge nodes manually.'
        )

    env = _edge_env()
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    detector_image_topic = LaunchConfiguration('detector_image_topic').perform(context)
    detector_depth_topic = LaunchConfiguration('detector_depth_topic').perform(context)

    detector = TimerAction(period=6.0, actions=[
        ExecuteProcess(
            cmd=[
                venv_python, '-m', 'object_tracking.detect_target_server',
                '--ros-args',
                '-p', f'use_sim_time:={use_sim_time}',
                '-p', f"image_topic:={detector_image_topic}",
                '-p', f"depth_topic:={detector_depth_topic}",
                '-p', f"use_compressed_input:={LaunchConfiguration('detector_use_compressed_input').perform(context)}",
                '-p', f"input_reliability:={LaunchConfiguration('detector_input_reliability').perform(context)}",
                '-p', f"model_mode:={LaunchConfiguration('detector_model_mode').perform(context)}",
                '-p', f"conf_default:={LaunchConfiguration('detector_conf_default').perform(context)}",
                '-p', f"target_conf_default:={LaunchConfiguration('detector_target_conf_default').perform(context)}",
                '-p', f"vocab_conf_default:={LaunchConfiguration('detector_vocab_conf_default').perform(context)}",
                '-p', f"min_mask_area:={LaunchConfiguration('detector_min_mask_area').perform(context)}",
                '-p', f"use_depth:={LaunchConfiguration('detector_use_depth').perform(context)}",
            ],
            env=env,
            name='detect_target_server',
            output='screen',
        )
    ])

    orchestrator = TimerAction(period=34.0, actions=[
        ExecuteProcess(
            cmd=[
                venv_python, '-m', 'planner_orchestrator.orchestrator_node',
                '--ros-args',
                '-p', f'use_sim_time:={use_sim_time}',
                '-p', f"use_mock:={LaunchConfiguration('use_mock').perform(context)}",
                '-p', f"async_replan:={LaunchConfiguration('async_replan').perform(context)}",
                '-p', f"replan_every_n:={LaunchConfiguration('replan_every_n').perform(context)}",
                '-p', f"max_steps:={LaunchConfiguration('max_steps').perform(context)}",
                '-p', f"detect_conf:={LaunchConfiguration('detect_conf').perform(context)}",
                '-p', f"target_detect_conf:={LaunchConfiguration('target_detect_conf').perform(context)}",
                '-p', f"detect_all_conf:={LaunchConfiguration('detect_all_conf').perform(context)}",
                '-p', f"context_detect_conf:={LaunchConfiguration('context_detect_conf').perform(context)}",
                '-p', f"vlm_timeout_s:={LaunchConfiguration('vlm_timeout_s').perform(context)}",
                '-p', f"send_map:={LaunchConfiguration('send_map').perform(context)}",
                '-p', f"motion_fallback_frame:={LaunchConfiguration('motion_fallback_frame').perform(context)}",
            ],
            env=env,
            name='planner_orchestrator',
            output='screen',
        )
    ])

    return [detector, orchestrator]


def generate_launch_description():
    pkg = get_package_share_directory('ar_project')
    launch_dir = os.path.join(pkg, 'launch')
    default_world = os.path.join(pkg, 'worlds', 'flat_detect.world')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=default_world,
            description='Gazebo world for VLM sim. Default has a bus billboard target.',
        ),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'start_edge',
            default_value='false',
            description='Also start detect_target_server + planner_orchestrator from venv_python.',
        ),
        DeclareLaunchArgument(
            'venv_python',
            default_value='~/.venvs/ros-jazzy-ml/bin/python',
            description='Python interpreter with torch/cv2/rclpy for edge-side nodes.',
        ),
        DeclareLaunchArgument('use_mock', default_value='false'),
        DeclareLaunchArgument('async_replan', default_value='false'),
        DeclareLaunchArgument('replan_every_n', default_value='3'),
        DeclareLaunchArgument('max_steps', default_value='40'),
        DeclareLaunchArgument('detect_conf', default_value='0.0'),
        DeclareLaunchArgument('target_detect_conf', default_value='0.50'),
        DeclareLaunchArgument('detect_all_conf', default_value='0.08'),
        DeclareLaunchArgument('context_detect_conf', default_value='0.25'),
        DeclareLaunchArgument('vlm_timeout_s', default_value='30.0'),
        DeclareLaunchArgument('send_map', default_value='true'),
        DeclareLaunchArgument('motion_fallback_frame', default_value='odom'),
        DeclareLaunchArgument('detector_image_topic', default_value='/camera/camera/color/image_raw'),
        DeclareLaunchArgument('detector_depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw'),
        DeclareLaunchArgument('detector_use_compressed_input', default_value='false'),
        DeclareLaunchArgument('detector_input_reliability', default_value='best_effort'),
        DeclareLaunchArgument('detector_model_mode', default_value='hybrid_dino_yoloe'),
        DeclareLaunchArgument('detector_conf_default', default_value='-1.0'),
        DeclareLaunchArgument('detector_target_conf_default', default_value='0.50'),
        DeclareLaunchArgument('detector_vocab_conf_default', default_value='0.08'),
        DeclareLaunchArgument('detector_min_mask_area', default_value='200'),
        DeclareLaunchArgument('detector_use_depth', default_value='true'),
        DeclareLaunchArgument(
            'start_monitor',
            default_value='true',
            description='Start robot_health_aggregator + the mission dashboard '
                        '(http://localhost:8088) alongside the sim stack.',
        ),
        DeclareLaunchArgument(
            'start_vlm_logger',
            default_value='true',
            description='Persist /vlm/activity as JSONL + compact CSV during VLM sim runs.',
        ),
        DeclareLaunchArgument(
            'vlm_log_output_dir',
            default_value='~/ros2_ws/experiment_logs/vlm_missions',
            description='Directory for VLM mission logger artifacts.',
        ),
        DeclareLaunchArgument(
            'vlm_log_run_id',
            default_value='',
            description='Optional run id prefix for VLM mission logs. Empty means timestamp.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'flat_sim_bringup.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'world': LaunchConfiguration('world'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'gui': LaunchConfiguration('gui'),
                'rviz': LaunchConfiguration('rviz'),
            }.items(),
        ),
        # Monitoring: per-component health rollup + the human-readable web
        # dashboard. In sim everything is one host, so both just run here.
        Node(
            package='search_coordinator',
            executable='robot_health_aggregator',
            name='robot_health_aggregator',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_monitor')),
        ),
        Node(
            package='fleet_comms',
            executable='mission_dashboard',
            name='mission_dashboard',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_monitor')),
        ),
        Node(
            package='fleet_comms',
            executable='vlm_mission_logger',
            name='vlm_mission_logger',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'output_dir': LaunchConfiguration('vlm_log_output_dir'),
                'run_id': LaunchConfiguration('vlm_log_run_id'),
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_vlm_logger')),
        ),
        OpaqueFunction(function=_edge_processes),
    ])
