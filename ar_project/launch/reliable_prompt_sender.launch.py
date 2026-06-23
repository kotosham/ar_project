from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'request_topic',
            default_value='/target_prompt_request',
            description='Local topic where the user publishes a prompt request once.',
        ),
        DeclareLaunchArgument(
            'prompt_topic',
            default_value='/target_prompt',
            description='Network-visible prompt topic consumed by the tracking stack.',
        ),
        DeclareLaunchArgument(
            'prompt_ack_topic',
            default_value='/target_prompt_ack',
            description='Ack topic published by the Raspberry Pi after it accepts a new prompt.',
        ),
        DeclareLaunchArgument(
            'retry_period',
            default_value='0.5',
            description='Seconds between repeated prompt publications while waiting for ack.',
        ),
        DeclareLaunchArgument(
            'max_retries',
            default_value='12',
            description='Maximum number of repeated prompt publications before giving up.',
        ),
        Node(
            package='ar_project',
            executable='reliable_prompt_sender.py',
            output='screen',
            parameters=[{
                'request_topic': LaunchConfiguration('request_topic'),
                'prompt_topic': LaunchConfiguration('prompt_topic'),
                'prompt_ack_topic': LaunchConfiguration('prompt_ack_topic'),
                'retry_period': LaunchConfiguration('retry_period'),
                'max_retries': LaunchConfiguration('max_retries'),
            }],
        ),
    ])
