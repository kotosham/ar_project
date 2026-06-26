import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def _build_clean_gui_env():
    # VS Code from snap leaks GTK / SNAP runtime variables that crash RViz.
    # Keep ROS/DDS discovery variables though, otherwise RViz can silently start
    # in a different graph/domain and show no map, TF or images.
    keep = [
        'HOME',
        'USER',
        'LOGNAME',
        'SHELL',
        'DISPLAY',
        'WAYLAND_DISPLAY',
        'XDG_RUNTIME_DIR',
        'XAUTHORITY',
        'PATH',
        'ROS_DOMAIN_ID',
        'ROS_LOCALHOST_ONLY',
        'ROS_AUTOMATIC_DISCOVERY_RANGE',
        'ROS_STATIC_PEERS',
        'ROS_DISCOVERY_SERVER',
        'RMW_IMPLEMENTATION',
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        'CYCLONEDDS_URI',
    ]
    env = {key: os.environ[key] for key in keep if os.environ.get(key)}
    env['PATH'] = os.pathsep.join(
        path for path in ['/opt/ros/jazzy/bin', env.get('PATH', '')] if path
    )
    return env


def generate_launch_description():
    package_share = get_package_share_directory('ar_project')
    default_config = os.path.join(package_share, 'config', 'drive_bot.rviz')

    config = LaunchConfiguration('config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    clean_env = _build_clean_gui_env()

    rviz = ExecuteProcess(
        cmd=[
            'bash',
            '-lc',
            'source /opt/ros/jazzy/setup.bash >/dev/null 2>&1 && '
            'rviz2 -d "$0" --ros-args -p use_sim_time:="$1"',
            config,
            use_sim_time,
        ],
        env=clean_env,
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Absolute path to an RViz config file',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time in RViz',
        ),
        rviz,
    ])
