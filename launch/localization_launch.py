import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('ar_project')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz = LaunchConfiguration('rviz')
    database_path = LaunchConfiguration('database_path')
    odom_topic = LaunchConfiguration('odom_topic')
    initial_pose = LaunchConfiguration('initial_pose')
    start_at_origin = LaunchConfiguration('start_at_origin')
    enable_visual_odometry = LaunchConfiguration('enable_visual_odometry')
    visual_odom_topic = LaunchConfiguration('visual_odom_topic')
    detection_rate = LaunchConfiguration('detection_rate')
    linear_update = LaunchConfiguration('linear_update')
    angular_update = LaunchConfiguration('angular_update')

    include_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'rtabmap_rgbd_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': 'true',
            'rtabmap_viz': rtabmap_viz,
            'rviz': rviz,
            'database_path': database_path,
            'odom_topic': odom_topic,
            'initial_pose': initial_pose,
            'start_at_origin': start_at_origin,
            'enable_visual_odometry': enable_visual_odometry,
            'visual_odom_topic': visual_odom_topic,
            'detection_rate': detection_rate,
            'linear_update': linear_update,
            'angular_update': angular_update,
            'delete_db_on_start': 'false',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock. Keep false on hardware.',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap_rgbd.db',
            description='RTAB-Map database path used for localization.',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odometry/filtered',
            description='Filtered odometry topic used by RTAB-Map. Override to /odom for simulation.',
        ),
        DeclareLaunchArgument(
            'initial_pose',
            default_value='',
            description='Optional approximate starting pose for RTAB-Map localization: "x y z roll pitch yaw".',
        ),
        DeclareLaunchArgument(
            'start_at_origin',
            default_value='true',
            description='Start localization from the map origin instead of the last saved localization pose.',
        ),
        DeclareLaunchArgument(
            'enable_visual_odometry',
            default_value='false',
            description='Launch a separate RGB-D visual odometry node and publish it on visual_odom_topic for optional EKF fusion.',
        ),
        DeclareLaunchArgument(
            'visual_odom_topic',
            default_value='/visual_odom',
            description='Topic name used by the optional RGB-D visual odometry node.',
        ),
        DeclareLaunchArgument(
            'detection_rate',
            default_value='2',
            description='RTAB-Map localization update rate in Hz.',
        ),
        DeclareLaunchArgument(
            'linear_update',
            default_value='0.05',
            description='Minimum linear motion before RTAB-Map processes a new RGB-D update.',
        ),
        DeclareLaunchArgument(
            'angular_update',
            default_value='0.05',
            description='Minimum angular motion before RTAB-Map processes a new RGB-D update.',
        ),
        DeclareLaunchArgument(
            'rtabmap_viz',
            default_value='false',
            description='Launch RTAB-Map native visualization.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Launch RViz from rtabmap_launch.',
        ),
        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Compatibility placeholder. Static map files are not used in the RGB-D localization workflow.',
        ),
        include_rtabmap,
    ])
