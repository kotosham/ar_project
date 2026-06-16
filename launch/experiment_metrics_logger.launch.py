from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'output_csv',
            default_value='~/ros2_ws/experiment_logs/experiment_metrics.csv',
            description='CSV file where experiment timing metrics will be appended.',
        ),
        DeclareLaunchArgument(
            'trial_timeout_s',
            default_value='30.0',
            description='Timeout in seconds for one experiment trial.',
        ),
        DeclareLaunchArgument(
            'target_point_topic',
            default_value='/experiment/target_point',
            description='Topic where the selected 3D target point is published for automatic FD estimation.',
        ),
        DeclareLaunchArgument(
            'fd_auto_topic',
            default_value='/experiment/fd_auto',
            description='Topic where the Raspberry Pi publishes automatic FD measurements after goal arrival.',
        ),
        DeclareLaunchArgument(
            'enable_fd_auto_measurement',
            default_value='false',
            description='Whether to record fd_auto_m from the Raspberry Pi measurement topic when Nav2 succeeds.',
        ),
        DeclareLaunchArgument(
            'fd_auto_wait_s',
            default_value='1.0',
            description='How long the logger waits for fd_auto_m after Nav2 reports success.',
        ),
        DeclareLaunchArgument(
            'robot_frame',
            default_value='base_link',
            description='Robot frame used for automatic FD estimation.',
        ),
        DeclareLaunchArgument(
            'front_robot_x',
            default_value='0.275',
            description='X coordinate of the front-center reference point in base_link.',
        ),
        DeclareLaunchArgument(
            'front_robot_y',
            default_value='0.0',
            description='Y coordinate of the front-center reference point in base_link.',
        ),
        DeclareLaunchArgument(
            'fd_distance_mode',
            default_value='planar',
            description='Automatic FD estimation mode: planar or forward.',
        ),
        Node(
            package='ar_project',
            executable='experiment_metrics_logger.py',
            output='screen',
            parameters=[{
                'output_csv': LaunchConfiguration('output_csv'),
                'trial_timeout_s': LaunchConfiguration('trial_timeout_s'),
                'target_point_topic': LaunchConfiguration('target_point_topic'),
                'fd_auto_topic': LaunchConfiguration('fd_auto_topic'),
                'enable_fd_auto_measurement': LaunchConfiguration('enable_fd_auto_measurement'),
                'fd_auto_wait_s': LaunchConfiguration('fd_auto_wait_s'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'front_robot_x': LaunchConfiguration('front_robot_x'),
                'front_robot_y': LaunchConfiguration('front_robot_y'),
                'fd_distance_mode': LaunchConfiguration('fd_distance_mode'),
            }],
        ),
    ])
