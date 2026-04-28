import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz = LaunchConfiguration('rviz')
    database_path = LaunchConfiguration('database_path')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock.',
    )
    declare_localization = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch RTAB-Map in localization mode.',
    )
    declare_rtabmap_viz = DeclareLaunchArgument(
        'rtabmap_viz',
        default_value='false',
        description='Launch RTAB-Map visualization UI.',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz from rtabmap_launch.',
    )
    declare_database_path = DeclareLaunchArgument(
        'database_path',
        default_value='~/.ros/rtabmap_rgbd.db',
        description='RTAB-Map database path.',
    )
    declare_rgb_topic = DeclareLaunchArgument(
        'rgb_topic',
        default_value='/camera/camera/color/image_raw',
        description='RGB image topic.',
    )
    declare_depth_topic = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/camera/aligned_depth_to_color/image_raw',
        description='Depth image topic aligned to the color stream.',
    )
    declare_camera_info_topic = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/camera/camera/aligned_depth_to_color/camera_info',
        description='Camera info topic associated with the RGB-D pair.',
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Wheel odometry topic used as the primary motion estimate for RTAB-Map.',
    )
    declare_point_cloud_topic = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/camera/camera/depth/color/points_rgbd',
        description='Depth point cloud generated from the RGB-D pair for future navigation integration and debugging.',
    )

    depth_cloud_from_rgbd = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='depth_cloud_from_rgbd',
        output='screen',
        parameters=[{
            'approx_sync': True,
        }],
        remappings=[
            ('rgb/image', rgb_topic),
            ('depth/image', depth_topic),
            ('rgb/camera_info', camera_info_topic),
            ('cloud', point_cloud_topic),
        ],
    )

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py',
            )
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'localization': localization,
            'rtabmap_viz': rtabmap_viz,
            'rviz': rviz,
            'namespace': '',
            'frame_id': 'base_link',
            'map_frame_id': 'map',
            'publish_tf_map': 'true',
            'rgb_topic': rgb_topic,
            'depth_topic': depth_topic,
            'camera_info_topic': camera_info_topic,
            'odom_topic': odom_topic,
            'subscribe_scan': 'false',
            'subscribe_scan_cloud': 'false',
            'scan_cloud_topic': point_cloud_topic,
            'depth': 'false',
            'rgbd_sync': 'true',
            'approx_sync': 'true',
            'approx_rgbd_sync': 'true',
            'approx_sync_max_interval': '0.01',
            'compressed': 'false',
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'publish_tf_odom': 'false',
            'database_path': database_path,
            'args': '--delete_db_on_start --Rtabmap/DetectionRate 10 --Reg/Strategy 1 --Reg/Force3DoF true --Mem/NotLinkedNodesKept false --Mem/InitWMWithAllNodes true --RGBD/LinearUpdate 0.05 --RGBD/AngularUpdate 0.05 --Grid/3D false --Grid/RayTracing true --Grid/RangeMax 3 --Grid/NormalsSegmentation false --Grid/MaxGroundHeight 0.05 --Grid/MaxObstacleHeight 0.4 --Optimizer/GravitySigma 0',
            'odom_sensor_sync': 'true',
            'wait_for_transform': '0.5',
            'qos': '2',
        }.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_localization,
        declare_rtabmap_viz,
        declare_rviz,
        declare_database_path,
        declare_rgb_topic,
        declare_depth_topic,
        declare_camera_info_topic,
        declare_odom_topic,
        declare_point_cloud_topic,
        depth_cloud_from_rgbd,
        rtabmap_launch,
    ])
