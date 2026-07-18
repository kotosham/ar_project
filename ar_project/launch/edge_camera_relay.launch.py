"""Edge-side single-ingest camera relay (fixes the Pi camera fan-out).

PROBLEM: on hardware every edge consumer used to open its OWN subscription to
the Pi's RealSense topics — RTAB-Map pulled RAW color + RAW aligned depth +
camera_info, the detector pulled the color stream + RAW aligned depth, and the
VLM orchestrator pulled RAW color. With multicast disabled (zenoh/DDS unicast),
the Pi serialized and shipped the same camera content across Wi-Fi several
times in several encodings (raw color ~921 KB/frame, raw aligned depth
~614 KB/frame, each at 15 Hz per consumer) — saturating the Pi's uplink.

FIX: exactly ONE compressed stream crosses Wi-Fi. This launch runs on the EDGE
machine and is the only Wi-Fi consumer of the camera:

    Pi (image_transport):                         Edge (this launch):
    /camera/.../color/image_raw/compressed   -->  republish --> /camera_edge/color/image_raw
    /camera/.../aligned_depth.../compressedDepth->republish --> /camera_edge/aligned_depth_to_color/image_raw
    /camera/.../color/camera_info            -->  relay     --> /camera_edge/color/camera_info (latched)

All edge consumers (RTAB-Map SLAM, detect_target_server, planner_orchestrator)
must then subscribe to /camera_edge/* — edge-local, so each additional
subscriber costs ZERO extra Wi-Fi bandwidth. The Pi compresses each stream once
(image_transport compresses lazily, only while a subscriber exists).

The Pi's raw topics stay strictly Pi-local (EKF/IMU, /scan via
depthimage_to_laserscan, per DATA_CONTRACTS.md §4). Do NOT point any edge node
at /camera/camera/* directly — that silently reintroduces the raw fan-out.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    rgb_transport = LaunchConfiguration('rgb_transport')
    depth_transport = LaunchConfiguration('depth_transport')
    out_rgb_topic = LaunchConfiguration('out_rgb_topic')
    out_depth_topic = LaunchConfiguration('out_depth_topic')
    out_camera_info_topic = LaunchConfiguration('out_camera_info_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rgb_topic',
            default_value='/camera/camera/color/image_raw',
            description='Pi-side RGB base topic; the relay subscribes to its /compressed variant.',
        ),
        DeclareLaunchArgument(
            'depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Pi-side aligned depth base topic; the relay subscribes to its /compressedDepth variant.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/color/camera_info',
            description='Pi-side color CameraInfo topic (small; relayed 1:1).',
        ),
        DeclareLaunchArgument(
            'rgb_transport',
            default_value='compressed',
            description='image_transport used for RGB across Wi-Fi (compressed = JPEG).',
        ),
        DeclareLaunchArgument(
            'depth_transport',
            default_value='compressedDepth',
            description='image_transport used for depth across Wi-Fi (compressedDepth = PNG/RVL 16UC1).',
        ),
        DeclareLaunchArgument(
            'out_rgb_topic',
            default_value='/camera_edge/color/image_raw',
            description='Edge-local raw RGB topic all edge consumers subscribe to.',
        ),
        DeclareLaunchArgument(
            'out_depth_topic',
            default_value='/camera_edge/aligned_depth_to_color/image_raw',
            description='Edge-local raw aligned depth topic all edge consumers subscribe to.',
        ),
        DeclareLaunchArgument(
            'out_camera_info_topic',
            default_value='/camera_edge/color/camera_info',
            description='Edge-local latched CameraInfo topic.',
        ),
        # Compressed RGB (Wi-Fi) -> raw RGB (edge-local). Decompress ONCE here;
        # every edge consumer then reads the local raw topic for free.
        Node(
            package='image_transport',
            executable='republish',
            name='edge_relay_rgb',
            output='screen',
            parameters=[{
                'in_transport': rgb_transport,
                'out_transport': 'raw',
            }],
            remappings=[
                (['in/', rgb_transport], [rgb_topic, '/', rgb_transport]),
                ('out', out_rgb_topic),
            ],
        ),
        # Compressed depth (Wi-Fi) -> raw 16UC1 depth (edge-local).
        Node(
            package='image_transport',
            executable='republish',
            name='edge_relay_depth',
            output='screen',
            parameters=[{
                'in_transport': depth_transport,
                'out_transport': 'raw',
            }],
            remappings=[
                (['in/', depth_transport], [depth_topic, '/', depth_transport]),
                ('out', out_depth_topic),
            ],
        ),
        # CameraInfo (small) -> latched edge-local copy.
        Node(
            package='ar_project',
            executable='camera_info_relay.py',
            name='camera_info_relay',
            output='screen',
            parameters=[{
                'input_topic': camera_info_topic,
                'output_topic': out_camera_info_topic,
            }],
        ),
    ])
