"""End-to-end ZERO-VLM FLAT bring-up in simulation (ROADMAP 2.10).

Composes the full FLAT autonomy stack on one machine, all on sim time:

  L1  launch_sim         Gazebo + robot (gz DiffDrive) + ros_gz_bridge +
                         depthimage_to_laserscan (/scan) + twist_mux +
                         Collision Monitor  (odom->base_link via odom_to_tf)
  L2  rtabmap_rgbd       RTAB-Map RGB-D SLAM -> /map + map->odom TF
  L3  navigation_launch  lightened Nav2 (NavFn + DWB), costmaps in map/odom
  L4  frontier_extractor SLAM /map -> /frontiers (+ markers, fail-loud guard)
  L4  coordinator_node   SeekObject entry + executive FSM + skill servers

Layers are staggered with TimerAction so each dependency is up before the next
needs it (sim -> SLAM map+TF -> Nav2 costmaps -> executive). Everything runs with
use_sim_time:=true (the /clock from Gazebo). odom_topic:=/odom because the sim's gz
DiffDrive publishes /odom directly (no EKF/IMU in sim — that path is hardware).

NOTE (sim vs hardware): on hardware this is replaced by hardware_bringup +
edge-side RTAB-Map + map_odom_relay; here SLAM publishes map->odom directly.
The open-vocab detector (object_tracking) that produces /target_pixel is launched
separately when testing the DETECT->APPROACH leg; without it the mission performs
pure frontier exploration (SEARCH) until frontiers are exhausted.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('ar_project')
    launch_dir = os.path.join(pkg, 'launch')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    odom_topic = LaunchConfiguration('odom_topic')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the Gazebo /clock. true for this sim bring-up.')
    declare_world = DeclareLaunchArgument(
        'world', default_value=os.path.join(pkg, 'worlds', 'flat_detect.world'),
        description='Gazebo world. Default: flat_detect.world with a bus billboard target. '
                    'Use oscillation.world for frontier-only FLAT tests.')
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic', default_value='/odom',
        description='Odometry topic. /odom in sim (gz DiffDrive); /odometry/filtered on hardware (EKF).')
    declare_gui = DeclareLaunchArgument(
        'gui', default_value='false',
        description='Show the Gazebo Sim GUI window (true) or run headless (false). '
                    'Headless is the default for this full-stack bring-up; pass gui:=true to watch the sim.')
    declare_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz after the stack has started. Use rviz:=true for visual debugging.')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'launch_sim.launch.py')),
        launch_arguments={'gui': gui, 'world': world}.items())

    rtabmap = TimerAction(period=10.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rtabmap_rgbd_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                'rviz': 'false',
                'rtabmap_viz': 'false',
            }.items())])

    nav2 = TimerAction(period=20.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'navigation_launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'odom_topic': odom_topic,
                # Pin Nav2's params explicitly: collision_monitor.launch.py (included via
                # launch_sim) declares a 'params_file' arg defaulting to collision_monitor.yaml,
                # and that LaunchConfiguration leaks into this include — without this line Nav2
                # loads collision_monitor.yaml, DWB finds no critics, and lifecycle bringup aborts.
                'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
            }.items())])

    executive = TimerAction(period=28.0, actions=[
        Node(package='search_coordinator', executable='frontier_extractor',
             name='frontier_extractor', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
        Node(package='search_coordinator', executable='coordinator_node',
             name='search_coordinator', output='screen',
             parameters=[{'use_sim_time': use_sim_time}]),
    ])

    rviz_view = TimerAction(period=32.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
            condition=IfCondition(rviz),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'config': os.path.join(pkg, 'config', 'drive_bot.rviz'),
            }.items())])

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        declare_odom_topic,
        declare_gui,
        declare_rviz,
        sim,
        rtabmap,
        nav2,
        executive,
        rviz_view,
    ])
