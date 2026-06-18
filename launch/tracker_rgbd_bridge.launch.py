from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    input_depth_topic = LaunchConfiguration('input_depth_topic')
    input_rgb_topic = LaunchConfiguration('input_rgb_topic')
    output_depth_topic = LaunchConfiguration('output_depth_topic')
    output_rgb_topic = LaunchConfiguration('output_rgb_topic')
    prompt_topic = LaunchConfiguration('prompt_topic')
    goal_locked_topic = LaunchConfiguration('goal_locked_topic')
    max_publish_rate = LaunchConfiguration('max_publish_rate')
    sync_tolerance = LaunchConfiguration('sync_tolerance')
    jpeg_quality = LaunchConfiguration('jpeg_quality')

    bridge_node = Node(
        package='ar_project',
        executable='tracker_rgbd_bridge.py',
        output='screen',
        parameters=[{
            'input_rgb_topic': input_rgb_topic,
            'input_depth_topic': input_depth_topic,
            'output_rgb_topic': output_rgb_topic,
            'output_depth_topic': output_depth_topic,
            'prompt_topic': prompt_topic,
            'goal_locked_topic': goal_locked_topic,
            'max_publish_rate': max_publish_rate,
            'sync_tolerance': sync_tolerance,
            'pause_on_goal_lock': True,
            'jpeg_quality': jpeg_quality,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'input_depth_topic',
            default_value='/camera/camera/aligned_depth_to_color/image_raw',
            description='Local aligned depth topic from RealSense on the Raspberry Pi.',
        ),
        DeclareLaunchArgument(
            'input_rgb_topic',
            default_value='/camera/camera/color/image_raw',
            description='Local RGB topic from RealSense on the Raspberry Pi.',
        ),
        DeclareLaunchArgument(
            'output_depth_topic',
            default_value='/tracker/aligned_depth_to_color/image_raw',
            description='Throttled raw depth topic exported by the Raspberry Pi bridge.',
        ),
        DeclareLaunchArgument(
            'output_rgb_topic',
            default_value='/tracker/color/image_raw/compressed',
            description='Throttled compressed RGB topic exported by the Raspberry Pi bridge.',
        ),
        DeclareLaunchArgument(
            'prompt_topic',
            default_value='/target_prompt',
            description='Prompt topic used to enable continuous RGB-D export while tracking is active.',
        ),
        DeclareLaunchArgument(
            'goal_locked_topic',
            default_value='/target_goal_locked',
            description='Latched Bool topic used to pause RGB-D export after a goal lock.',
        ),
        DeclareLaunchArgument(
            'max_publish_rate',
            default_value='1.0',
            description='Maximum RGB-D export rate in Hz for continuous tracking.',
        ),
        DeclareLaunchArgument(
            'sync_tolerance',
            default_value='0.15',
            description='Maximum RGB/depth timestamp mismatch tolerated when exporting a continuous pair.',
        ),
        DeclareLaunchArgument(
            'jpeg_quality',
            default_value='90',
            description='JPEG quality used for direct compressed RGB export to the laptop.',
        ),
        bridge_node,
    ])
