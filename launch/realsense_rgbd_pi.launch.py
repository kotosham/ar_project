import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    camera_namespace = LaunchConfiguration('camera_namespace')
    color_profile = LaunchConfiguration('rgb_camera.color_profile')
    depth_profile = LaunchConfiguration('depth_module.depth_profile')
    enable_sync = LaunchConfiguration('enable_sync')
    align_depth = LaunchConfiguration('align_depth.enable')
    initial_reset = LaunchConfiguration('initial_reset')
    enable_gyro = LaunchConfiguration('enable_gyro')
    enable_accel = LaunchConfiguration('enable_accel')
    unite_imu_method = LaunchConfiguration('unite_imu_method')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='RealSense node name.',
        ),
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='camera',
            description='RealSense ROS namespace.',
        ),
        DeclareLaunchArgument(
            'rgb_camera.color_profile',
            default_value='640x480x15',
            description='Color stream profile chosen to reduce Raspberry Pi load on the RGB pipeline.',
        ),
        DeclareLaunchArgument(
            'depth_module.depth_profile',
            default_value='424x240x15',
            description='Depth stream profile chosen to reduce Raspberry Pi load on the depth pipeline.',
        ),
        DeclareLaunchArgument(
            'enable_sync',
            default_value='true',
            description='Enable RealSense sync mode so RTAB-Map can pair RGB and depth more reliably.',
        ),
        DeclareLaunchArgument(
            'align_depth.enable',
            default_value='true',
            description='Align depth to color stream.',
        ),
        DeclareLaunchArgument(
            'initial_reset',
            default_value='false',
            description='Reset the RealSense device once on startup. Disabled by default because it did not improve the D435i motion-module startup on the Raspberry Pi.',
        ),
        DeclareLaunchArgument(
            'enable_gyro',
            default_value='true',
            description='Enable the D435i gyro stream so EKF can use angular velocity for heading correction.',
        ),
        DeclareLaunchArgument(
            'enable_accel',
            default_value='true',
            description='Enable the D435i accel stream so RealSense can publish the fused /imu topic.',
        ),
        DeclareLaunchArgument(
            'unite_imu_method',
            default_value='2',
            description='RealSense IMU unification method. 2 publishes a fused /imu topic for downstream consumers.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('realsense2_camera'),
                    'launch',
                    'rs_launch.py',
                )
            ),
            launch_arguments={
                'camera_name': camera_name,
                'camera_namespace': camera_namespace,
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_sync': enable_sync,
                'align_depth.enable': align_depth,
                'pointcloud.enable': 'false',
                'initial_reset': initial_reset,
                'enable_gyro': enable_gyro,
                'enable_accel': enable_accel,
                'unite_imu_method': unite_imu_method,
                'rgb_camera.color_profile': color_profile,
                'depth_module.depth_profile': depth_profile,
            }.items(),
        ),
    ])
