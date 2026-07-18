"""Edge (GPU box) hardware bring-up: single-ingest camera relay + RTAB-Map SLAM.

One command for the edge side of the two-host deployment (RUNBOOK §4a):

    ros2 launch ar_project edge_bringup.launch.py

It starts:
  1. edge_camera_relay.launch.py — the ONLY Wi-Fi consumer of the Pi camera.
     One compressed RGB + one compressedDepth + one CameraInfo stream cross the
     link; the relay decompresses once and republishes raw on /camera_edge/*.
  2. rtabmap_rgbd_launch.py wired to /camera_edge/* (edge-local, free).

The detector and VLM orchestrator still start from the ML venv (they need
torch/cv2), but MUST also be pointed at /camera_edge/*:

    ~/ot_venv/bin/python .../detect_target_server --ros-args \
        -p use_sim_time:=false \
        -p image_topic:=/camera_edge/color/image_raw \
        -p use_compressed_input:=false \
        -p depth_topic:=/camera_edge/aligned_depth_to_color/image_raw

    ~/ot_venv/bin/python .../orchestrator_node --ros-args \
        -p use_sim_time:=false \
        -p camera_image_topic:=/camera_edge/color/image_raw ...

Never point an edge node at /camera/camera/* directly — each such subscription
re-opens its own Wi-Fi stream from the Pi and reintroduces the fan-out.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('ar_project')
    launch_dir = os.path.join(package_share, 'launch')

    start_slam = LaunchConfiguration('start_slam')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    detection_rate = LaunchConfiguration('detection_rate')
    publish_tf_map = LaunchConfiguration('publish_tf_map')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_slam',
            default_value='true',
            description='Start RTAB-Map on the edge (set false to run only the camera relay).',
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Run RTAB-Map in localization mode against an existing database.',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap_rgbd.db',
            description='RTAB-Map database path.',
        ),
        DeclareLaunchArgument(
            'delete_db_on_start',
            default_value='true',
            description='Delete the RTAB-Map database on startup (keep false to reuse a map).',
        ),
        DeclareLaunchArgument(
            'detection_rate',
            default_value='2',
            description='RTAB-Map detection rate in Hz.',
        ),
        DeclareLaunchArgument(
            'publish_tf_map',
            default_value='true',
            description='Let RTAB-Map broadcast map->odom on /tf. Set false when map_odom_relay owns map->odom.',
        ),
        DeclareLaunchArgument(
            'start_dashboard',
            default_value='true',
            description='Serve the human-readable mission dashboard (http://<edge>:8088).',
        ),
        DeclareLaunchArgument(
            'dashboard_port',
            default_value='8088',
            description='HTTP port of the mission dashboard.',
        ),
        # 1. Single-ingest camera relay: the only Wi-Fi camera consumer.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'edge_camera_relay.launch.py')
            ),
        ),
        # 2. RTAB-Map consumes the edge-local relayed streams (NOT the Pi topics).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rtabmap_rgbd_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'localization': localization,
                'database_path': database_path,
                'delete_db_on_start': delete_db_on_start,
                'detection_rate': detection_rate,
                'publish_tf_map': publish_tf_map,
                'rgb_topic': '/camera_edge/color/image_raw',
                'depth_topic': '/camera_edge/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera_edge/color/camera_info',
                # Inputs are already decompressed by the relay; RTAB-Map's own
                # compressed_transport path must stay off or it would open a
                # second Wi-Fi subscription to the Pi.
                'compressed_transport': 'false',
            }.items(),
            condition=IfCondition(start_slam),
        ),
        # 3. Human-readable mission dashboard (per-component health, VLM
        # thinking/actions, robot view). Edge-hosted so the heavy views stay
        # link-free; open http://<edge-host>:8088.
        Node(
            package='fleet_comms',
            executable='mission_dashboard',
            name='mission_dashboard',
            parameters=[{
                'port': ParameterValue(LaunchConfiguration('dashboard_port'),
                                       value_type=int),
                'use_sim_time': False,
            }],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_dashboard')),
        ),
    ])
