> **Verification corrections.**
> - **`tc netem` does not work on the stock WSL2 kernel** because `sch_netem` is
>   missing, just like `vcan`. Wi-Fi degradation injection in WSL2 requires a
>   custom WSL2 kernel with `CONFIG_NET_SCH_NETEM` and related classifier options,
>   or a second physical host / non-WSL VM. Without that, network FMEA rows are
>   not reproducible in stock WSL2.
> - **The VLM is an external OpenAI-compatible API, not self-hosted on edge.**
>   Planner Orchestrator is a light HTTP client and does not require GPU. Edge GPU
>   is used for segmentation (YOLOE/DINO+SAM) and SLAM (RTAB-Map).
> - **Gazebo GUI under WSLg:** prefer XWayland/X11. If OGRE2 shows a black screen
>   or crashes, unset `WAYLAND_DISPLAY` and/or use `LIBGL_ALWAYS_SOFTWARE=1`.
> - **OGRE2 GPU sensor rendering under WSL2 is unreliable.** Validate target FPS
>   on native Linux or the edge host when possible.
> - **gz `DiffDrive` expects `geometry_msgs/Twist`, while Jazzy
>   `diff_drive_controller`/Nav2 can publish `TwistStamped`.** Use a bridge or
>   `use_stamped_vel` consistently.
> - **vcan does not reproduce real bus-off.** It can only be scripted in a
>   fake-EPOS4; real bus-off needs hardware.

# Gazebo Testing on WSL2 Ubuntu

This document describes how to start and debug the `robust` system in simulation
before moving to hardware. Simulation is the first required stage: measure the
clean Pi baseline (FLAT, no VLM) first, then enable VLM mode and degradation
tests. Commands assume WSL2 Ubuntu 24.04 with ROS 2 Jazzy.

## 1. Simulator: Gazebo Sim (gz), not Classic

The repository uses **Gazebo Sim** (`gz`, formerly Ignition), not Gazebo Classic.
Robot plugins use the `gz::sim::systems::*` namespace, `gz_ros2_control` provides
ROS 2 control integration, `ros_gz_bridge` connects topics, and
`launch_sim.launch.py` starts `gz sim --force-version 8`. For ROS 2 Jazzy, the
supported stack is **Gazebo Harmonic (gz-sim8)** + `ros_gz` +
`gz_ros2_control`.

```bash
# FLAT baseline with the built-in gz DiffDrive plugin
ros2 launch ar_project launch_sim.launch.py world:=<path>/test_world.sdf gui:=true

# Full stack with the same control interface shape as hardware
ros2 launch ar_project launch_sim.launch.py use_ros2_control:=true gui:=false
```

## 2. Simulated Sensor Set

The goal is for perception (`object_tracking`) and `target_pixel_to_goal` to see
the same topic names and frame IDs as on the RealSense hardware.

**RGB + depth.** `description/camera_gazebo_sensors.xacro` provides two gz
sensors (`camera` and `depth`), 640x480 @ 30 Hz, with matching FOV and optical
frame. Legacy `depth_camera_link*` frames remain aliases so existing code keeps
working.

**Aligned depth emulation.** Sim depth is mounted to the same optical frame and
resolution as RGB, so pixels already match. `config/gz_bridge.yaml` remaps the
depth topic to the names expected by the stack, including
`camera/camera/aligned_depth_to_color/image_raw`. PointCloud2 is allowed only for
local checks in simulation; raw depth and point clouds must not cross Wi-Fi.

**IMU.** gz-IMU was not present at the time this note was written. To reproduce
the EKF pipeline, add the `gz-sim-imu-system` plugin, an IMU sensor on `base_link`
or `imu_link`, and a `sensor_msgs/msg/Imu` bridge with the same topic name as the
hardware driver.

## 3. EPOS4/CAN Hardware Interface Emulation

On hardware, wheels are controlled by `ar_project/EmbodiedRobotSystem` using
CiA-402 over SocketCAN. In simulation this plugin is replaced by one of two
paths controlled by `use_ros2_control`:

- **`use_ros2_control:=false`**: uses the built-in gz DiffDrive plugin. This is
  the simplest FLAT baseline path, but it is not identical to the hardware
  control stack.
- **`use_ros2_control:=true`**: uses `gz_ros2_control/GazeboSimSystem` with the
  same controllers (`diff_cont`, `joint_broad`) and wheel command/state
  interfaces. This is the preferred integration test mode because everything
  above the hardware plugin matches hardware.

Gazebo does **not** test real CiA-402 quick-stop, statusword/fault behavior,
non-blocking SDO, CAN bus-off, or frame loss. Those require real EPOS4 hardware
or a fake-EPOS4 over SocketCAN/vcan.

Stock WSL2 does not provide `vcan`. A custom WSL2 kernel with CAN support is
required before using:

```bash
sudo modprobe can can_raw vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
candump vcan0
```

## 4. Emulating the Pi-PC Split on One WSL Machine

The real system has two hosts over Wi-Fi with `rmw_zenoh`. On one WSL machine:

- Use different `ROS_DOMAIN_ID` values to prove the Pi FLAT stack is autonomous
  when edge disappears. This gives isolation but no cross-link.
- Use a local zenoh router and run both node groups with
  `RMW_IMPLEMENTATION=rmw_zenoh_cpp` to reproduce the real transport path for VLM
  mode, QoS deadline/liveliness tests, circuit-breaker behavior, and commit-point
  plan adoption.

Network degradation requires `tc netem` or network namespaces on a kernel that
supports them. Typical injections are latency, jitter, packet loss, duplication,
reordering, and full link loss. These tests validate VLM/edge/Wi-Fi degradation
to FLAT, last-good behavior in `map_odom_relay`, stale stamp rejection, and the
Planner Orchestrator circuit breaker.

## 5. GPU Workloads under WSL2

The VLM is an external OpenAI-compatible API and is **not** hosted on the
edge/WSL2 machine. Planner Orchestrator is only an async HTTP client. Edge GPU is
used for local workloads: segmentation models (YOLOE/GroundingDINO+MobileSAM)
and RTAB-Map.

Under WSL2, the CUDA driver is the Windows NVIDIA driver. Do not install the
Linux display driver inside Ubuntu WSL. Install only the WSL-specific CUDA
toolkit. `nvidia-smi` inside WSL should see the GPU.

## 6. GUI: WSLg vs Headless gz

- **WSLg** provides Gazebo/RViz windows without a separate Windows X server.
  `launch_sim.launch.py` already handles the environment whitelist needed when
  running from VS Code/snap-like environments.
- **Headless** (`gui:=false`) starts `gz sim -s --headless-rendering`. This is
  suitable for CI, FMEA matrix runs, and machines without rendering support.
  Cameras/depth still render off-screen.

If GPU rendering fails under WSL2, use `LIBGL_ALWAYS_SOFTWARE=1` as a slower
fallback.

## 7. `use_sim_time`, Clock, and TF Discipline

Hard rule: **all nodes use `use_sim_time:=true` in simulation and `false` on
hardware**. The simulation clock comes from gz through `/clock`. EKF, Nav2,
controllers, `target_pixel_to_goal`, perception, and `map_odom_relay` must all
use the same clock, otherwise TF tolerance, depth-match, and pixel-age windows
are measured against inconsistent time.

On hardware, chrony keeps host clocks synchronized. In simulation on one machine,
true Pi-edge clock skew is not reproduced automatically; inject stamp offsets in
mock publishers when testing stale-stamp rejection.

RTAB-Map on edge should publish a low-rate map->odom correction message, not a TF
stream. `map_odom_relay` on the Pi applies it, holds last-good, gates bad jumps
or covariance, rejects stale stamps, and rebroadcasts `map->odom` locally.

## 8. Simulation Test Matrix

| FMEA scenario | Sim injection | Expected reaction | Gazebo reproducible? |
|---|---|---|---|
| Slow VLM | delay mock/OpenAI API response | FLAT continues current subtask; replan is adopted at commit point | yes |
| VLM API unavailable / edge down | HTTP 5xx/timeout or kill edge nodes | circuit breaker opens, VLM mode degrades to FLAT | yes |
| Wi-Fi loss | netem loss 100% or veth down | relay holds last-good, Nav2 continues in odom, Pi stack does not freeze | yes with proper netem/netns support |
| map->odom stuck/diverging | freeze or jump mock SLAM correction | jump/covariance gate rejects bad correction and holds last-good | yes |
| Clock skew | offset stamps in edge publisher | stale stamps are rejected by relay and pixel-age checks | partial |
| Detector OOM/down | kill detector or return no detections | exploration continues; no false reached | yes |
| Stale/duplicate result | replay UUID or stale result | UUID and age filters reject it | yes |
| Instruction change in flight | send new instruction during active subtask | abort/reset, mission epoch increment, in-flight UUID invalidation | yes |
| Frontier oscillation | world with two nearly equal frontiers | hysteresis prevents switching back and forth | yes |
| Approach with stale pixel | stop detector stream during approach | no auto-success from old pixel; re-detect/abort path | yes |
| CiA-402 quick-stop / CAN bus-off | hardware/fake-EPOS4 only | real quick-stop, per-cycle fault poll, bus-off handling | no in Gazebo |
| cmd_vel watchdog / Collision Monitor | stop `/cmd_vel`; add obstacle | watchdog brakes; Collision Monitor cuts speed/stops | yes |

Not fully covered by Gazebo: real CiA-402 dynamics, CAN bus-off and CAN frame
loss, true clock skew between physical hosts, real Wi-Fi physics, and honest CUDA
OOM on the exact target edge GPU if the dev GPU differs.

Relevant files:

- `ar_project/description/camera_gazebo_sensors.xacro`
- `ar_project/description/camera.xacro`, `depth_camera.xacro`
- `ar_project/description/ros2_control.xacro`
- `ar_project/description/ros2_control_hardware.xacro`
- `ar_project/description/gazebo_control.xacro`
- `ar_project/description/robot.urdf.xacro`
- `ar_project/launch/launch_sim.launch.py`
- `ar_project/config/gz_bridge.yaml`
- `ar_project/config/my_controllers.yaml`
- `ar_project/launch/hardware_bringup.launch.py`

Pending `robust` work noted here: add gz IMU sensor/plugin/bridge and expose a
single top-level `use_sim_time` argument through all launches instead of hardcoded
`True` values.
