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
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock.',
        ),
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap_rgbd.db',
            description='RTAB-Map database path used for localization.',
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
