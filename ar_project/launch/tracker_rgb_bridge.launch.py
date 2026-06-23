from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'input_topic',
                default_value='/camera/camera/color/image_raw',
                description='Local RGB topic from RealSense on the Raspberry Pi.',
            ),
            DeclareLaunchArgument(
                'output_topic',
                default_value='/tracker/color/image/compressed',
                description='Compressed RGB topic exported for laptop-side object tracking.',
            ),
            DeclareLaunchArgument(
                'jpeg_quality',
                default_value='90',
                description='JPEG quality for exported tracker frames.',
            ),
            DeclareLaunchArgument(
                'max_publish_rate',
                default_value='3.0',
                description='Maximum RGB export rate in Hz. Set <=0 to export every frame.',
            ),
            DeclareLaunchArgument(
                'burst_frame_count',
                default_value='3',
                description='How many compressed RGB frames to export after each prompt. Set <=0 for continuous streaming.',
            ),
            DeclareLaunchArgument(
                'prompt_topic',
                default_value='/target_prompt',
                description='Prompt topic used to enable RGB export only while searching.',
            ),
            DeclareLaunchArgument(
                'goal_locked_topic',
                default_value='/target_goal_locked',
                description='Latched Bool topic used to disable RGB export after the goal is locked.',
            ),
            DeclareLaunchArgument(
                'burst_complete_topic',
                default_value='/tracker/burst_complete',
                description='Topic used to notify the laptop that the current RGB burst has fully finished publishing.',
            ),
            Node(
                package='ar_project',
                executable='tracker_rgb_bridge.py',
                output='screen',
                parameters=[
                    {
                        'input_topic': LaunchConfiguration('input_topic'),
                        'output_topic': LaunchConfiguration('output_topic'),
                        'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                        'max_publish_rate': LaunchConfiguration('max_publish_rate'),
                        'burst_frame_count': LaunchConfiguration('burst_frame_count'),
                        'prompt_topic': LaunchConfiguration('prompt_topic'),
                        'goal_locked_topic': LaunchConfiguration('goal_locked_topic'),
                        'burst_complete_topic': LaunchConfiguration('burst_complete_topic'),
                    }
                ],
            ),
        ]
    )
