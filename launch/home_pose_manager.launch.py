from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'target_frame',
                default_value='map',
                description='Frame in which the home pose is stored.',
            ),
            DeclareLaunchArgument(
                'robot_frame',
                default_value='base_link',
                description='Robot body frame to track and send back home.',
            ),
            DeclareLaunchArgument(
                'goal_topic',
                default_value='/goal_pose',
                description='Nav2 goal topic used both by RViz and the target bridge.',
            ),
            DeclareLaunchArgument(
                'home_pose_topic',
                default_value='/home_pose',
                description='Latched debug topic with the currently saved home pose.',
            ),
            DeclareLaunchArgument(
                'save_home_topic',
                default_value='/save_home_pose',
                description='Publish std_msgs/Empty here to overwrite the saved home pose.',
            ),
            DeclareLaunchArgument(
                'return_home_topic',
                default_value='/return_home',
                description='Publish std_msgs/Empty here to send the robot back to the saved home pose.',
            ),
            DeclareLaunchArgument(
                'auto_capture_on_start',
                default_value='true',
                description='Automatically save the first valid map->base_link pose after startup.',
            ),
            Node(
                package='ar_project',
                executable='home_pose_manager.py',
                output='screen',
                parameters=[
                    {
                        'target_frame': LaunchConfiguration('target_frame'),
                        'robot_frame': LaunchConfiguration('robot_frame'),
                        'goal_topic': LaunchConfiguration('goal_topic'),
                        'home_pose_topic': LaunchConfiguration('home_pose_topic'),
                        'save_home_topic': LaunchConfiguration('save_home_topic'),
                        'return_home_topic': LaunchConfiguration('return_home_topic'),
                        'auto_capture_on_start': LaunchConfiguration('auto_capture_on_start'),
                    }
                ],
            ),
        ]
    )
