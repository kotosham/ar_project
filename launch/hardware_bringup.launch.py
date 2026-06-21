import os

from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    package_share = get_package_share_directory('ar_project')
    xacro_file = os.path.join(package_share, 'description', 'robot_hardware.urdf.xacro')
    controllers_file = os.path.join(
        package_share, 'config', 'epos4_diffdrive', 'ros2_controllers.yaml'
    )
    twist_mux_params = os.path.join(package_share, 'config', 'twist_mux.yaml')
    ekf_params_file_raw_imu = os.path.join(package_share, 'config', 'ekf_gyro.yaml')
    ekf_params_file_filtered_imu = os.path.join(package_share, 'config', 'ekf_imu_filtered.yaml')
    imu_filter_launch_file = os.path.join(package_share, 'launch', 'imu_orientation_filter.launch.py')

    can_interface_name = LaunchConfiguration('can_interface_name')
    use_twist_mux = LaunchConfiguration('use_twist_mux')
    start_controllers = LaunchConfiguration('start_controllers')
    start_joint_state_broadcaster = LaunchConfiguration('start_joint_state_broadcaster')
    enable_imu_ekf = LaunchConfiguration('enable_imu_ekf')
    enable_imu_orientation_filter = LaunchConfiguration('enable_imu_orientation_filter')
    imu_topic = LaunchConfiguration('imu_topic')
    filtered_imu_topic = LaunchConfiguration('filtered_imu_topic')
    odom_input_topic = LaunchConfiguration('odom_input_topic')
    wheel_separation_multiplier = LaunchConfiguration('wheel_separation_multiplier')
    left_wheel_radius_multiplier = LaunchConfiguration('left_wheel_radius_multiplier')
    right_wheel_radius_multiplier = LaunchConfiguration('right_wheel_radius_multiplier')

    configured_controllers = ParameterFile(
        RewrittenYaml(
            source_file=controllers_file,
            param_rewrites={
                'wheel_separation_multiplier': wheel_separation_multiplier,
                'left_wheel_radius_multiplier': left_wheel_radius_multiplier,
                'right_wheel_radius_multiplier': right_wheel_radius_multiplier,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

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
        parameters=[robot_description, configured_controllers],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
    )

    imu_orientation_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(imu_filter_launch_file),
        launch_arguments={
            'raw_imu_topic': imu_topic,
            'filtered_imu_topic': filtered_imu_topic,
        }.items(),
        condition=IfCondition(enable_imu_orientation_filter),
    )

    ekf_filter_node_raw_imu = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file_raw_imu,
            {
                'use_sim_time': False,
                'imu0': imu_topic,
                'odom0': odom_input_topic,
            },
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", enable_imu_ekf, "' == 'true' and '", enable_imu_orientation_filter, "' != 'true'"]
            )
        ),
    )

    ekf_filter_node_filtered_imu = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            ekf_params_file_filtered_imu,
            {
                'use_sim_time': False,
                'imu0': filtered_imu_topic,
                'odom0': odom_input_topic,
            },
        ],
        condition=IfCondition(
            PythonExpression(
                ["'", enable_imu_ekf, "' == 'true' and '", enable_imu_orientation_filter, "' == 'true'"]
            )
        ),
    )

    cmd_vel_watchdog = Node(
        package='ar_project',
        executable='cmd_vel_watchdog.py',
        name='cmd_vel_watchdog',
        parameters=[
            {
                'input_topic': '/cmd_vel',
                'output_topic': '/cmd_vel_safe',
                'timeout': 0.5,
                'publish_rate': 20.0,
                'use_sim_time': False,
            }
        ],
        output='screen',
        condition=IfCondition(use_twist_mux),
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
        DeclareLaunchArgument(
            'enable_imu_ekf',
            default_value='true',
            description='Start robot_localization EKF to fuse wheel odometry with RealSense gyro yaw rate.',
        ),
        DeclareLaunchArgument(
            'enable_imu_orientation_filter',
            default_value='true',
            description='Run imu_filter_madgwick on the RealSense IMU and feed filtered yaw + yaw rate into the EKF.',
        ),
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/camera/camera/imu',
            description='Raw RealSense IMU topic. Used directly by EKF when IMU orientation filtering is disabled.',
        ),
        DeclareLaunchArgument(
            'filtered_imu_topic',
            default_value='/camera/camera/imu/data',
            description='Filtered IMU topic published by imu_filter_madgwick and consumed by the EKF when filtering is enabled.',
        ),
        DeclareLaunchArgument(
            'odom_input_topic',
            default_value='/diff_cont/odom',
            description='Raw wheel odometry input topic fused by the EKF.',
        ),
        DeclareLaunchArgument(
            'wheel_separation_multiplier',
            default_value='1.0052',
            description='Calibration multiplier for differential-drive wheel separation. Primarily affects turn-angle accuracy.',
        ),
        DeclareLaunchArgument(
            'left_wheel_radius_multiplier',
            default_value='1.0249',
            description='Calibration multiplier for the left wheel radius. Use the same value on both sides for average straight-distance calibration.',
        ),
        DeclareLaunchArgument(
            'right_wheel_radius_multiplier',
            default_value='1.0249',
            description='Calibration multiplier for the right wheel radius. Adjust asymmetrically only for persistent left/right drift.',
        ),
        control_node,
        robot_state_publisher,
        imu_orientation_filter,
        ekf_filter_node_raw_imu,
        ekf_filter_node_filtered_imu,
        cmd_vel_watchdog,
        twist_mux,
        twist_to_stamped_from_mux,
        twist_to_stamped_direct,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner_direct,
        start_diff_drive_after_jsb,
    ]

    return LaunchDescription(actions)
