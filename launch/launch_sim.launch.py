import os
import shutil

from ament_index_python.packages import get_package_share_directory
from catkin_pkg.package import InvalidPackage, PACKAGE_MANIFEST_FILENAME, parse_package
from ros2pkg.api import get_package_names


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def _get_gazebo_paths():
    gazebo_model_path = []
    gazebo_plugin_path = []
    gazebo_media_path = []

    for package_name in get_package_names():
        package_share_path = get_package_share_directory(package_name)
        package_file_path = os.path.join(package_share_path, PACKAGE_MANIFEST_FILENAME)
        if not os.path.isfile(package_file_path):
            continue

        try:
            package = parse_package(package_file_path)
        except InvalidPackage:
            continue

        for export in package.exports:
            if export.tagname != 'gazebo_ros':
                continue

            if 'gazebo_model_path' in export.attributes:
                xml_path = export.attributes['gazebo_model_path']
                gazebo_model_path.append(xml_path.replace('${prefix}', package_share_path))
            if 'plugin_path' in export.attributes:
                xml_path = export.attributes['plugin_path']
                gazebo_plugin_path.append(xml_path.replace('${prefix}', package_share_path))
            if 'gazebo_media_path' in export.attributes:
                xml_path = export.attributes['gazebo_media_path']
                gazebo_media_path.append(xml_path.replace('${prefix}', package_share_path))

    return (
        os.pathsep.join(gazebo_model_path + gazebo_media_path),
        os.pathsep.join(gazebo_plugin_path),
    )


def _build_gz_env():
    # Gazebo GUI launched from the VS Code snap inherits a broken runtime
    # environment. Start Gazebo with a minimal clean environment so GUI mode
    # works while the ROS nodes keep their regular environment.
    model_paths, plugin_paths = _get_gazebo_paths()
    whitelist = [
        'HOME',
        'USER',
        'LOGNAME',
        'SHELL',
        'DISPLAY',
        'WAYLAND_DISPLAY',
        'XDG_RUNTIME_DIR',
        'XAUTHORITY',
        'PATH',
        'LD_LIBRARY_PATH',
        'AMENT_PREFIX_PATH',
        'COLCON_PREFIX_PATH',
        'CMAKE_PREFIX_PATH',
        'PYTHONPATH',
        'ROS_DISTRO',
        'ROS_VERSION',
        'GZ_CONFIG_PATH',
    ]
    env = {key: os.environ[key] for key in whitelist if os.environ.get(key)}
    env['GZ_SIM_SYSTEM_PLUGIN_PATH'] = os.pathsep.join(
        path
        for path in [
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.environ.get('LD_LIBRARY_PATH', ''),
            plugin_paths,
        ]
        if path
    )
    env['GZ_SIM_RESOURCE_PATH'] = os.pathsep.join(
        path
        for path in [
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            model_paths,
        ]
        if path
    )
    return env



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='ar_project' #<--- CHANGE ME
    default_world = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'empty.world',
    )
    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load',
    )
    gui = LaunchConfiguration('gui')
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo Sim with the graphical interface',
    )
    gui_config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gz_gui.config',
    )
    gz_script = shutil.which('gz') or '/opt/ros/jazzy/opt/gz_tools_vendor/bin/gz'
    ruby_bin = shutil.which('ruby') or 'ruby'
    gz_env = _build_gz_env()
    gz_env_prefix = ['env', '-i'] + [f'{key}={value}' for key, value in gz_env.items()]

    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control_arg = DeclareLaunchArgument(
        'use_ros2_control',
        default_value='false',
        description='Use gz_ros2_control and ROS diff drive controller for wheel control',
    )

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': use_ros2_control}.items()
    )

    cmd_vel_watchdog = Node(
        package=package_name,
        executable='cmd_vel_watchdog.py',
        name='cmd_vel_watchdog',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/cmd_vel_safe',
            'timeout': 0.5,
            'publish_rate': 20.0,
            'use_sim_time': True,
        }],
        output='screen',
    )

    twist_mux_params = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package = "twist_mux",
        executable = "twist_mux",
        parameters = [twist_mux_params, {'use_sim_time': True}],
        remappings = [('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )
    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')

    # Launch Gazebo Sim directly so we can give it a clean environment in GUI mode.
    gazebo_gui = ExecuteProcess(
        cmd=gz_env_prefix + [ruby_bin, gz_script, 'sim', '-r', '-v', '4', '--gui-config', gui_config, world, '--force-version', '8'],
        name='gazebo',
        output='screen',
        condition=IfCondition(gui),
        on_exit=Shutdown(),
    )
    gazebo_headless = ExecuteProcess(
        cmd=gz_env_prefix + [ruby_bin, gz_script, 'sim', '-r', '-s', '--headless-rendering', '-v', '4', world, '--force-version', '8'],
        name='gazebo',
        output='screen',
        condition=UnlessCondition(gui),
        on_exit=Shutdown(),
    )

    # Spawn the URDF model into Gazebo Sim from robot_description.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'my_bot',
                                   '-z', '0.1'],
                        output='screen')

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ros2_control),
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        output='screen',
        condition=IfCondition(use_ros2_control),
    )

    spawn_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                joint_state_broadcaster_spawner,
                diff_drive_controller_spawner,
            ],
        )
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    odom_tf_bridge = Node(
        package=package_name,
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # The parameter bridge publishes both the new RealSense-like RGB-D topics
    # and the legacy aliases consumed by the current stack.
    # Launch them all!
    return LaunchDescription([
        world_arg,
        gui_arg,
        use_ros2_control_arg,
        rsp,
        cmd_vel_watchdog,
        twist_mux,
        gazebo_gui,
        gazebo_headless,
        spawn_entity,
        spawn_controllers,
        ros_gz_bridge,
        odom_tf_bridge,
    ])
