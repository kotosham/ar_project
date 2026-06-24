import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    rtabmap_viz = LaunchConfiguration('rtabmap_viz')
    rviz = LaunchConfiguration('rviz')
    database_path = LaunchConfiguration('database_path')
    initial_pose = LaunchConfiguration('initial_pose')
    start_at_origin = LaunchConfiguration('start_at_origin')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    point_cloud_topic = LaunchConfiguration('point_cloud_topic')
    compressed_transport = LaunchConfiguration('compressed_transport')
    rgb_image_transport = LaunchConfiguration('rgb_image_transport')
    depth_image_transport = LaunchConfiguration('depth_image_transport')
    approx_sync_max_interval = LaunchConfiguration('approx_sync_max_interval')
    topic_queue_size = LaunchConfiguration('topic_queue_size')
    sync_queue_size = LaunchConfiguration('sync_queue_size')
    publish_rgbd_cloud = LaunchConfiguration('publish_rgbd_cloud')
    enable_visual_odometry = LaunchConfiguration('enable_visual_odometry')
    visual_odom_topic = LaunchConfiguration('visual_odom_topic')
    detection_rate = LaunchConfiguration('detection_rate')
    linear_update = LaunchConfiguration('linear_update')
    angular_update = LaunchConfiguration('angular_update')
    delete_db_on_start = LaunchConfiguration('delete_db_on_start')
    publish_tf_map = LaunchConfiguration('publish_tf_map')
    rgb_topic_input = PythonExpression([
        "'",
        rgb_topic,
        "_relay' if '",
        compressed_transport,
        "'.lower() in ('true', '1', 'yes') else '",
        rgb_topic,
        "'",
    ])
    depth_topic_input = PythonExpression([
        "'",
        depth_topic,
        "_relay' if '",
        compressed_transport,
        "'.lower() in ('true', '1', 'yes') else '",
        depth_topic,
        "'",
    ])

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock. Keep false on hardware.',
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
    declare_initial_pose = DeclareLaunchArgument(
        'initial_pose',
        default_value='',
        description='Optional initial pose for RTAB-Map localization. Format: "x y z roll pitch yaw".',
    )
    declare_start_at_origin = DeclareLaunchArgument(
        'start_at_origin',
        default_value='true',
        description='When localizing, start from the map origin instead of the last saved localization pose.',
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
        default_value='/camera/camera/color/camera_info',
        description='Color camera info used together with the aligned depth image for RTAB-Map RGB-D synchronization.',
    )
    declare_odom_topic = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odometry/filtered',
        description='Filtered odometry topic used as the primary motion estimate for RTAB-Map.',
    )
    declare_point_cloud_topic = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/camera/camera/depth/color/points_rgbd',
        description='Depth point cloud generated from the RGB-D pair for future navigation integration and debugging.',
    )
    declare_compressed_transport = DeclareLaunchArgument(
        'compressed_transport',
        default_value='false',
        description='Subscribe to RGB-D image topics through image_transport compression. Useful when RTAB-Map runs on the laptop over Wi-Fi.',
    )
    declare_rgb_image_transport = DeclareLaunchArgument(
        'rgb_image_transport',
        default_value='compressed',
        description='Image transport used for RGB when compressed_transport is enabled.',
    )
    declare_depth_image_transport = DeclareLaunchArgument(
        'depth_image_transport',
        default_value='compressedDepth',
        description='Image transport used for depth when compressed_transport is enabled.',
    )
    declare_approx_sync_max_interval = DeclareLaunchArgument(
        'approx_sync_max_interval',
        default_value='0.2',
        description='Maximum time gap allowed by approximate RGB-D sync on hardware. Looser than sim defaults for Pi + RealSense.',
    )
    declare_topic_queue_size = DeclareLaunchArgument(
        'topic_queue_size',
        default_value='60',
        description='Queue size for each subscribed sensor topic to absorb jitter on hardware.',
    )
    declare_sync_queue_size = DeclareLaunchArgument(
        'sync_queue_size',
        default_value='60',
        description='Synchronizer queue size used by RTAB-Map RGB-D synchronization nodes.',
    )
    declare_publish_rgbd_cloud = DeclareLaunchArgument(
        'publish_rgbd_cloud',
        default_value='false',
        description='Publish a debug point cloud from the RGB-D stream. Disabled by default on Pi to reduce load.',
    )
    declare_enable_visual_odometry = DeclareLaunchArgument(
        'enable_visual_odometry',
        default_value='false',
        description='Launch a separate RGB-D visual odometry node and publish it on visual_odom_topic for optional EKF fusion.',
    )
    declare_visual_odom_topic = DeclareLaunchArgument(
        'visual_odom_topic',
        default_value='/visual_odom',
        description='Topic name used by the optional RGB-D visual odometry node.',
    )
    declare_detection_rate = DeclareLaunchArgument(
        'detection_rate',
        default_value='2',
        description='RTAB-Map detection rate in Hz. Lower default keeps the Pi responsive while validating RGB-D SLAM.',
    )
    declare_linear_update = DeclareLaunchArgument(
        'linear_update',
        default_value='0.05',
        description='Minimum linear motion in meters before RTAB-Map processes a new RGB-D update.',
    )
    declare_angular_update = DeclareLaunchArgument(
        'angular_update',
        default_value='0.05',
        description='Minimum angular motion in radians before RTAB-Map processes a new RGB-D update.',
    )
    declare_delete_db_on_start = DeclareLaunchArgument(
        'delete_db_on_start',
        default_value='true',
        description='Delete the RTAB-Map database on startup. Keep false when reusing a map for localization.',
    )
    declare_publish_tf_map = DeclareLaunchArgument(
        'publish_tf_map',
        default_value='true',
        description=(
            'Let RTAB-Map broadcast map->odom on /tf. Set false when map_odom_relay '
            '(ROADMAP 2.6) is the map->odom source, so the two do not fight on /tf.'
        ),
    )

    depth_cloud_from_rgbd = Node(
        package='rtabmap_util',
        executable='point_cloud_xyzrgb',
        name='depth_cloud_from_rgbd',
        output='screen',
        condition=IfCondition(publish_rgbd_cloud),
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': approx_sync_max_interval,
        }],
        remappings=[
            ('rgb/image', rgb_topic_input),
            ('depth/image', depth_topic_input),
            ('rgb/camera_info', camera_info_topic),
            ('cloud', point_cloud_topic),
        ],
    )

    republish_rgb_compressed = Node(
        package='image_transport',
        executable='republish',
        name='republish_rgb_compressed',
        output='screen',
        condition=IfCondition(compressed_transport),
        parameters=[{
            'in_transport': rgb_image_transport,
            'out_transport': 'raw',
        }],
        remappings=[
            (['in/', rgb_image_transport], [rgb_topic, '/', rgb_image_transport]),
            ('out', [rgb_topic, '_relay']),
        ],
    )

    republish_depth_compressed = Node(
        package='image_transport',
        executable='republish',
        name='republish_depth_compressed',
        output='screen',
        condition=IfCondition(compressed_transport),
        parameters=[{
            'in_transport': depth_image_transport,
            'out_transport': 'raw',
        }],
        remappings=[
            (['in/', depth_image_transport], [depth_topic, '/', depth_image_transport]),
            ('out', [depth_topic, '_relay']),
        ],
    )

    rgbd_visual_odometry = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_visual_odometry',
        output='screen',
        condition=IfCondition(enable_visual_odometry),
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'publish_tf': False,
            'wait_for_transform': 0.5,
            'approx_sync': True,
            'approx_sync_max_interval': approx_sync_max_interval,
            'topic_queue_size': topic_queue_size,
            'sync_queue_size': sync_queue_size,
            'qos': 2,
            'qos_camera_info': 2,
            'guess_frame_id': '',
            'guess_min_translation': 0.0,
            'guess_min_rotation': 0.0,
        }],
        remappings=[
            ('rgb/image', rgb_topic_input),
            ('depth/image', depth_topic_input),
            ('rgb/camera_info', camera_info_topic),
            ('odom', visual_odom_topic),
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
            'publish_tf_map': publish_tf_map,
            'initial_pose': initial_pose,
            'rgb_topic': rgb_topic_input,
            'depth_topic': depth_topic_input,
            'camera_info_topic': camera_info_topic,
            'odom_topic': odom_topic,
            'subscribe_scan': 'false',
            'subscribe_scan_cloud': 'false',
            'scan_cloud_topic': point_cloud_topic,
            'depth': 'false',
            'rgbd_sync': 'true',
            'approx_sync': 'true',
            'approx_rgbd_sync': 'true',
            'approx_sync_max_interval': approx_sync_max_interval,
            'topic_queue_size': topic_queue_size,
            'sync_queue_size': sync_queue_size,
            'compressed': 'false',
            'rgb_image_transport': rgb_image_transport,
            'depth_image_transport': depth_image_transport,
            'visual_odometry': 'false',
            'icp_odometry': 'false',
            'publish_tf_odom': 'false',
            'database_path': database_path,
            'args': [
                PythonExpression([
                    "'--delete_db_on_start ' if '",
                    delete_db_on_start,
                    "'.lower() in ('true', '1', 'yes') else ''",
                ]),
                '--Rtabmap/DetectionRate ', detection_rate,
                ' --Reg/Strategy 1',
                ' --Reg/Force3DoF true',
                ' --Mem/NotLinkedNodesKept false',
                ' --Mem/InitWMWithAllNodes true',
                ' --RGBD/StartAtOrigin ', start_at_origin,
                ' --RGBD/LinearUpdate ', linear_update,
                ' --RGBD/AngularUpdate ', angular_update,
                ' --Grid/3D true',
                ' --Grid/RayTracing true',
                ' --Grid/RangeMax 5',
                ' --Grid/NormalsSegmentation false',
                ' --Grid/MaxGroundHeight 0.05',
                ' --Grid/MaxObstacleHeight 0.4',
                ' --Optimizer/GravitySigma 0',
            ],
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
        declare_initial_pose,
        declare_start_at_origin,
        declare_rgb_topic,
        declare_depth_topic,
        declare_camera_info_topic,
        declare_odom_topic,
        declare_point_cloud_topic,
        declare_compressed_transport,
        declare_rgb_image_transport,
        declare_depth_image_transport,
        declare_approx_sync_max_interval,
        declare_topic_queue_size,
        declare_sync_queue_size,
        declare_publish_rgbd_cloud,
        declare_enable_visual_odometry,
        declare_visual_odom_topic,
        declare_detection_rate,
        declare_linear_update,
        declare_angular_update,
        declare_delete_db_on_start,
        declare_publish_tf_map,
        republish_rgb_compressed,
        republish_depth_compressed,
        depth_cloud_from_rgbd,
        rgbd_visual_odometry,
        rtabmap_launch,
    ])
