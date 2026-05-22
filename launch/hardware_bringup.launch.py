import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('ar_project')
    xacro_file = os.path.join(package_share, 'description', 'robot_hardware.urdf.xacro')
    controllers_file = os.path.join(
        package_share, 'config', 'epos4_diffdrive', 'ros2_controllers.yaml'
    )
    twist_mux_params = os.path.join(package_share, 'config', 'twist_mux.yaml')

    can_interface_name = LaunchConfiguration('can_interface_name')
    use_twist_mux = LaunchConfiguration('use_twist_mux')
    start_controllers = LaunchConfiguration('start_controllers')
    start_joint_state_broadcaster = LaunchConfiguration('start_joint_state_broadcaster')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            xacro_file,
            ' ',
            'can_interface_name:=',
            can_interface_name,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_file],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out', '/cmd_vel_out')],
        output='screen',
        condition=IfCondition(use_twist_mux),
    )

    twist_to_stamped_from_mux = Node(
        package='ar_project',
        executable='twist_to_twist_stamped.py',
        parameters=[
            {
                'input_topic': '/cmd_vel_out',
                'output_topic': '/diff_cont/cmd_vel',
            }
        ],
        output='screen',
        condition=IfCondition(use_twist_mux),
    )

    twist_to_stamped_direct = Node(
        package='ar_project',
        executable='twist_to_twist_stamped.py',
        parameters=[
            {
                'input_topic': '/cmd_vel',
                'output_topic': '/diff_cont/cmd_vel',
            }
        ],
        output='screen',
        condition=IfCondition(PythonExpression(["'", use_twist_mux, "' == 'false'"])),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(
                ["'", start_controllers, "' == 'true' and '", start_joint_state_broadcaster, "' == 'true'"]
            )
        ),
    )

    diff_drive_controller_spawner_direct = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(
                ["'", start_controllers, "' == 'true' and '", start_joint_state_broadcaster, "' == 'false'"]
            )
        ),
    )

    diff_drive_controller_spawner_after_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_cont',
            '--controller-manager',
            '/controller_manager',
            '--controller-manager-timeout',
            '120',
        ],
        output='screen',
        condition=IfCondition(
            PythonExpression(
                ["'", start_controllers, "' == 'true' and '", start_joint_state_broadcaster, "' == 'true'"]
            )
        ),
    )

    start_diff_drive_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner_after_jsb],
        )
        ,
        condition=IfCondition(
            PythonExpression(
                ["'", start_controllers, "' == 'true' and '", start_joint_state_broadcaster, "' == 'true'"]
            )
        ),
    )

    actions = [
        DeclareLaunchArgument(
            'can_interface_name',
            default_value='can0',
            description='SocketCAN interface connected to the EPOS4 drives.',
        ),
        DeclareLaunchArgument(
            'use_twist_mux',
            default_value='true',
            description='Start twist_mux so /cmd_vel, /cmd_vel_tracker and /cmd_vel_joy are merged.',
        ),
        DeclareLaunchArgument(
            'start_controllers',
            default_value='true',
            description='Automatically spawn joint_state_broadcaster and diff_drive_controller.',
        ),
        DeclareLaunchArgument(
            'start_joint_state_broadcaster',
            default_value='true',
            description='Spawn joint_state_broadcaster alongside the diff drive controller.',
        ),
        control_node,
        robot_state_publisher,
        twist_mux,
        twist_to_stamped_from_mux,
        twist_to_stamped_direct,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner_direct,
        start_diff_drive_after_jsb,
    ]

    return LaunchDescription(actions)
