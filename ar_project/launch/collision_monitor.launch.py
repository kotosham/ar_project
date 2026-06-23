"""Nav2 Collision Monitor bring-up (ROADMAP Phase 0.6).

Standalone, includable launch for the reactive collision-safety layer. Run it
alongside hardware_bringup / navigation, then route the hardware Twist->Stamped
bridge through /cmd_vel_collision_safe to activate it (see collision_monitor.yaml).

The monitor needs the local /scan from depthimage_to_laserscan (Phase 1.4); until
that exists it passes commands through (source goes stale and is ignored).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    bringup_dir = get_package_share_directory('ar_project')

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    cmd_vel_in_topic = LaunchConfiguration('cmd_vel_in_topic')
    cmd_vel_out_topic = LaunchConfiguration('cmd_vel_out_topic')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'use_sim_time': use_sim_time,
                'cmd_vel_in_topic': cmd_vel_in_topic,
                'cmd_vel_out_topic': cmd_vel_out_topic,
            },
            convert_types=True),
        allow_substs=True)

    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=[configured_params],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_collision_monitor',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': ['collision_monitor']},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true.'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(bringup_dir, 'config', 'collision_monitor.yaml'),
            description='Full path to the collision_monitor parameters file.'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically start (configure+activate) the collision monitor.'),
        DeclareLaunchArgument(
            'cmd_vel_in_topic',
            default_value='cmd_vel_out',
            description='Input Twist topic (the muxed velocity command to guard).'),
        DeclareLaunchArgument(
            'cmd_vel_out_topic',
            default_value='cmd_vel_collision_safe',
            description='Output Twist topic (guarded command sent downstream).'),
        collision_monitor_node,
        lifecycle_manager,
    ])
