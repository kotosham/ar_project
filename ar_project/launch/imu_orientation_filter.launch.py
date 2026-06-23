import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    raw_imu_topic = LaunchConfiguration('raw_imu_topic')
    filtered_imu_topic = LaunchConfiguration('filtered_imu_topic')

    package_share = get_package_share_directory('ar_project')
    imu_filter_params = os.path.join(package_share, 'config', 'imu_filter_madgwick.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'raw_imu_topic',
            default_value='/camera/camera/imu',
            description='Raw fused IMU topic produced by realsense2_camera.',
        ),
        DeclareLaunchArgument(
            'filtered_imu_topic',
            default_value='/camera/camera/imu/data',
            description='Orientation-estimated IMU topic published by imu_filter_madgwick.',
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[imu_filter_params],
            remappings=[
                ('imu/data_raw', raw_imu_topic),
                ('imu/data', filtered_imu_topic),
            ],
        ),
    ])
