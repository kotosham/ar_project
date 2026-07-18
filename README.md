# AR Project

ROS 2 Jazzy package for the base mobile-robot implementation used in the diploma experiments. The current primary target is a real differential-drive robot with a Raspberry Pi 5, EPOS4 motor controllers, a RealSense RGB-D camera, RTAB-Map RGB-D SLAM, Nav2, and a laptop-side CV tracker.

The base pipeline is:

```text
text prompt -> CV tracker -> /target_pixel -> target_pixel_to_goal -> /goal_pose -> Nav2
```

The VLM/Qwen/planner-orchestrator stack is not part of this base workflow.

## Repository Structure

- `config/` - Nav2, EKF, RTAB-Map RViz, controller, twist_mux, and simulation config files.
- `description/` - URDF/XACRO description for simulation and hardware.
- `launch/` - hardware, RealSense, RTAB-Map, Nav2, bridge, prompt, and experiment launch files.
- `scripts/` - Python nodes for prompt forwarding, RGB-D export, target-pixel conversion, metrics, home pose, and calibration.
- `docs/` - development reports, experiment summaries, and known limitations.
- `maps/`, `worlds/`, `models/` - legacy Gazebo maps/worlds/models used for earlier simulation work.

## Main Runtime Roles

The Raspberry Pi runs the robot-side stack:

- EPOS4 hardware bringup and diff-drive controller.
- RealSense RGB-D camera.
- EKF odometry fusion.
- RTAB-Map RGB-D SLAM.
- Nav2.
- RGB-D bridge to the laptop.
- `/target_pixel` to Nav2 goal conversion.

The laptop runs the ML tracker from the `object_tracking` package:

- receives throttled RGB-D data from the Pi;
- runs CLIPSeg, GroundingDINO + MobileSAM, Florence-2, or YOLOE;
- publishes `/target_pixel`, and optionally `/target_mask`;
- may publish soft step-wise search rotation commands on `/cmd_vel_tracker`.

## Build

From the workspace root:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select ar_project
source ~/ros2_ws/install/setup.bash
```

For a full workspace rebuild:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Network Environment

For the two-machine setup, use the same ROS domain and DDS settings on both Raspberry Pi and laptop. Example:

```bash
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export ROS_STATIC_PEERS='<other_machine_ip>'
```

Set `ROS_STATIC_PEERS` to the IP address of the other machine.

## Raspberry Pi Bringup

Bring up CAN before starting the hardware stack:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

Then start the robot-side terminals.

### 1. RealSense RGB-D

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch ar_project realsense_rgbd_pi.launch.py
```

Current defaults are tuned for Raspberry Pi load:

- color profile: `640x480x15`;
- depth profile: `424x240x15`;
- `enable_sync:=true`;
- `align_depth.enable:=true`;
- RealSense spatial/temporal/hole-filling filters disabled by default;
- RealSense pointcloud disabled by default.

### 2. Hardware And EKF

```bash
ros2 launch ar_project hardware_bringup.launch.py
```

This starts the robot description, ros2_control hardware interface, controller spawners, `twist_mux`, and the EKF by default.

`twist_mux` merges:

- `/cmd_vel` from Nav2;
- `/cmd_vel_tracker` from the CV search behavior;
- `/cmd_vel_joy` from joystick teleop.

### 3. RTAB-Map RGB-D SLAM

```bash
ros2 launch ar_project rtabmap_rgbd_launch.py
```

Important current defaults:

- `use_sim_time:=false`;
- `detection_rate:=5`;
- `linear_update:=0.0`;
- `angular_update:=0.0`;
- `approx_sync_max_interval:=0.2`;
- `publish_rgbd_cloud:=false`;
- `delete_db_on_start:=true`.

To publish a lightweight depth-only cloud for the Nav2 local costmap:

```bash
ros2 launch ar_project rtabmap_rgbd_launch.py publish_rgbd_cloud:=true
```

The cloud is downsampled with `cloud_decimation:=2`, `cloud_max_depth:=2.5`, and `cloud_voxel_size:=0.03`.

### 4. Nav2

```bash
ros2 launch ar_project navigation_launch.py use_sim_time:=false
```

Nav2 uses `config/nav2_params.yaml`. The current hardware-oriented settings include reduced angular speed/acceleration, reverse motion support for backing away from objects, and a local obstacle layer that can consume `/camera/camera/depth/color/points_rgbd` when the cloud is enabled.

### 5. RGB-D Bridge To Laptop

```bash
ros2 launch ar_project tracker_rgbd_bridge.launch.py max_publish_rate:=5.0
```

The bridge exports:

- `/tracker/color/image_raw/compressed`;
- `/tracker/aligned_depth_to_color/image_raw`.

It starts exporting after a prompt arrives and stops when `/target_goal_locked` indicates that a goal has been locked.

### 6. Target Pixel To Goal

```bash
ros2 launch ar_project target_pixel_to_goal.launch.py
```

This node converts `/target_pixel` into `/goal_pose` for Nav2. `target_pixel.x/y` are image pixel coordinates; `target_pixel.z` may carry embedded depth from the laptop tracker.

Important defaults:

- `approach_offset:=0.58`;
- `front_robot_x:=0.275`;
- `lock_goal_on_publish:=true`;
- `required_stable_detections:=2`;
- `stable_pixel_tolerance:=25.0`;
- `max_target_pixel_age_s:=1.5`;
- `embedded_depth_guard_radius_px:=12`.

For continuous goal updates during debugging:

```bash
ros2 launch ar_project target_pixel_to_goal.launch.py lock_goal_on_publish:=false
```

## Prompt Sending

The prompt topic used by the tracker is `/target_prompt`.

Direct one-shot prompt:

```bash
ros2 topic pub --once /target_prompt std_msgs/msg/String "{data: 'office chair'}"
```

Reliable repeated prompt sender:

```bash
ros2 launch ar_project reliable_prompt_sender.launch.py
ros2 topic pub --once /target_prompt_request std_msgs/msg/String "{data: 'office chair'}"
```

## Experiment Metrics

Metrics are logged by:

```bash
ros2 launch ar_project experiment_metrics_logger.launch.py
```

Default CSV path:

```text
~/ros2_ws/experiment_logs/experiment_metrics.csv
```

Recorded fields include prompt, CV model, CV runtime, goal publication latency, trial duration, total time to object, final automatic distance, and navigation outcome.

## Useful Debug Commands

Check odometry drift:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

Check target and goal flow:

```bash
ros2 topic echo /target_pixel
ros2 topic echo /goal_pose
ros2 topic echo /target_goal_locked
```

Check image export:

```bash
ros2 topic hz /tracker/color/image_raw/compressed
ros2 topic hz /tracker/aligned_depth_to_color/image_raw
```

## RViz

The main RTAB-Map RViz config is:

```bash
rviz2 -d ~/ros2_ws/src/ar_project/config/rtabmap_rgbd.rviz
```

The point cloud display is disabled in the saved config to reduce Raspberry Pi and network load during normal runs.

## Simulation

Simulation assets are still available for older tests:

```bash
ros2 launch ar_project launch_sim.launch.py world:=./src/ar_project/worlds/test_1.world
```

The current diploma base implementation, however, is centered on the real robot with RealSense RGB-D, RTAB-Map, Nav2, and laptop-side CV tracking.

## Development Notes

See:

- `docs/slam_cv_navigation_architecture_report.md`;
- `docs/odometry_sensor_fusion_report.md`;
- `docs/experiment_results_report.md`;
- `docs/development_challenges_summary.md`.
