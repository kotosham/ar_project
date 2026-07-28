import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    # Sim camera tuning forwarded verbatim into the xacro. Defaults here MUST match
    # the <xacro:arg> defaults in robot.urdf.xacro, otherwise the same description
    # would render differently depending on whether it came through this launch file.
    cam_width = LaunchConfiguration('cam_width')
    cam_height = LaunchConfiguration('cam_height')
    cam_rate = LaunchConfiguration('cam_rate')
    cam_far = LaunchConfiguration('cam_far')
    depth_far = LaunchConfiguration('depth_far')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('ar_project'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    #robot_description_config = xacro.process_file(xacro_file).toxml()
    # value_type=str so the URDF is always treated as a string (otherwise launch
    # tries to parse it as YAML and chokes on any ': ' in the XML, e.g. in comments).
    robot_description_config = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' use_ros2_control:=', use_ros2_control,
                 ' cam_width:=', cam_width,
                 ' cam_height:=', cam_height,
                 ' cam_rate:=', cam_rate,
                 ' cam_far:=', cam_far,
                 ' depth_far:=', depth_far]),
        value_type=str)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2 control if true'),

        DeclareLaunchArgument(
            'cam_width',
            default_value='320',
            description='Sim RGB+depth image width. 320 keeps WSL2 gz rendering affordable; '
                        'raise it (e.g. 640) only when signs/text have to be legible.'),

        DeclareLaunchArgument(
            'cam_height',
            default_value='240',
            description='Sim RGB+depth image height. Must keep the 4:3 ratio of horizontal_fov.'),

        DeclareLaunchArgument(
            'cam_rate',
            default_value='15',
            description='Sim camera update rate [Hz] for both the RGB and the depth sensor.'),

        DeclareLaunchArgument(
            'cam_far',
            default_value='30.0',
            description='RGB far clip [m]. A colour camera sees across a room, so this is '
                        'deliberately much longer than depth_far.'),

        DeclareLaunchArgument(
            'depth_far',
            default_value='8.0',
            description='Depth far clip [m]. Models the RealSense range limit and therefore '
                        'the /scan range_max produced by depthimage_to_laserscan.'),

        node_robot_state_publisher
    ])
