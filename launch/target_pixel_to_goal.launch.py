from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'target_pixel_topic',
                default_value='/target_pixel',
                description='Incoming target point topic from the laptop-side tracker. x/y are pixel coordinates; z may optionally carry depth in meters for continuous mode.',
            ),
            DeclareLaunchArgument(
                'target_mask_topic',
                default_value='/target_mask',
                description='Incoming binary target mask topic from the laptop-side tracker.',
            ),
            DeclareLaunchArgument(
                'depth_topic',
                default_value='/camera/camera/aligned_depth_to_color/image_raw',
                description='Local aligned depth topic on the Raspberry Pi.',
            ),
            DeclareLaunchArgument(
                'camera_info_topic',
                default_value='/camera/camera/color/camera_info',
                description='Local color camera info topic on the Raspberry Pi.',
            ),
            DeclareLaunchArgument(
                'goal_topic',
                default_value='/goal_pose',
                description='Nav2 goal topic to publish.',
            ),
            DeclareLaunchArgument(
                'target_point_topic',
                default_value='/experiment/target_point',
                description='3D target point selected for the current goal, published for experiment logging.',
            ),
            DeclareLaunchArgument(
                'approach_offset',
                default_value='0.58',
                description='How far from the detected 3D target point the robot should stop.',
            ),
            DeclareLaunchArgument(
                'prompt_topic',
                default_value='/target_prompt',
                description='Prompt topic used to reset a locked goal when the target object changes.',
            ),
            DeclareLaunchArgument(
                'goal_locked_topic',
                default_value='/target_goal_locked',
                description='Latched Bool topic announcing whether the current static-object goal is locked.',
            ),
            DeclareLaunchArgument(
                'fd_auto_topic',
                default_value='/experiment/fd_auto',
                description='Topic where the Raspberry Pi publishes automatic FD measurements after goal arrival.',
            ),
            DeclareLaunchArgument(
                'nav_status_topic',
                default_value='/navigate_to_pose/_action/status',
                description='Nav2 action status topic used to trigger post-arrival FD measurement.',
            ),
            DeclareLaunchArgument(
                'front_robot_x',
                default_value='0.275',
                description='X coordinate of the front-center reference point in base_link.',
            ),
            DeclareLaunchArgument(
                'front_robot_y',
                default_value='0.0',
                description='Y coordinate of the front-center reference point in base_link.',
            ),
            DeclareLaunchArgument(
                'fd_distance_mode',
                default_value='forward',
                description='How to turn the selected nearest depth point into FD: forward or planar.',
            ),
            DeclareLaunchArgument(
                'fd_floor_z_threshold',
                default_value='-0.01',
                description='Reject depth points at or below this z height in base_link to avoid measuring the floor.',
            ),
            DeclareLaunchArgument(
                'fd_sample_step',
                default_value='2',
                description='Depth subsampling step used for automatic FD measurement after arrival.',
            ),
            DeclareLaunchArgument(
                'fd_nearest_depth_band_m',
                default_value='0.03',
                description='Band around the nearest depth used to choose a stable post-arrival FD point.',
            ),
            DeclareLaunchArgument(
                'fd_lateral_limit_m',
                default_value='0.60',
                description='Ignore depth points farther than this lateral distance from robot center.',
            ),
            DeclareLaunchArgument(
                'lock_goal_on_publish',
                default_value='true',
                description='For static-object experiments, publish one goal and ignore future pixel updates until a new prompt arrives.',
            ),
            DeclareLaunchArgument(
                'max_target_pixel_age_s',
                default_value='1.5',
                description='Drop target-pixel updates that are older than this many seconds by the time they reach the Raspberry Pi.',
            ),
            DeclareLaunchArgument(
                'final_approach_freeze_distance',
                default_value='0.60',
                description='In continuous mode, stop accepting goal updates when the observed target gets this close to the robot and finish the final approach blindly.',
            ),
            DeclareLaunchArgument(
                'required_stable_detections',
                default_value='2',
                description='How many target-pixel detections must arrive before the goal is accepted. For burst best-candidate mode, override this back to 1.',
            ),
            DeclareLaunchArgument(
                'stable_pixel_tolerance',
                default_value='25.0',
                description='Maximum pixel jump between consecutive detections to keep them in the same stability window.',
            ),
            Node(
                package='ar_project',
                executable='target_pixel_to_goal.py',
                output='screen',
                parameters=[
                    {
                        'target_pixel_topic': LaunchConfiguration('target_pixel_topic'),
                        'target_mask_topic': LaunchConfiguration('target_mask_topic'),
                        'depth_topic': LaunchConfiguration('depth_topic'),
                        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
                        'goal_topic': LaunchConfiguration('goal_topic'),
                        'target_point_topic': LaunchConfiguration('target_point_topic'),
                        'approach_offset': LaunchConfiguration('approach_offset'),
                        'prompt_topic': LaunchConfiguration('prompt_topic'),
                        'goal_locked_topic': LaunchConfiguration('goal_locked_topic'),
                        'fd_auto_topic': LaunchConfiguration('fd_auto_topic'),
                        'nav_status_topic': LaunchConfiguration('nav_status_topic'),
                        'front_robot_x': LaunchConfiguration('front_robot_x'),
                        'front_robot_y': LaunchConfiguration('front_robot_y'),
                        'fd_distance_mode': LaunchConfiguration('fd_distance_mode'),
                        'fd_floor_z_threshold': LaunchConfiguration('fd_floor_z_threshold'),
                        'fd_sample_step': LaunchConfiguration('fd_sample_step'),
                        'fd_nearest_depth_band_m': LaunchConfiguration('fd_nearest_depth_band_m'),
                        'fd_lateral_limit_m': LaunchConfiguration('fd_lateral_limit_m'),
                        'lock_goal_on_publish': LaunchConfiguration('lock_goal_on_publish'),
                        'max_target_pixel_age_s': LaunchConfiguration('max_target_pixel_age_s'),
                        'final_approach_freeze_distance': LaunchConfiguration('final_approach_freeze_distance'),
                        'required_stable_detections': LaunchConfiguration('required_stable_detections'),
                        'stable_pixel_tolerance': LaunchConfiguration('stable_pixel_tolerance'),
                    }
                ],
            ),
        ]
    )
