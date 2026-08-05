# Docker - PC Stack Bringup

This containerizes architecture layers for running on a regular PC without
manually installing ROS 2, Gazebo, and dependencies. See
[../docs/architecture/SOLUTION_OVERVIEW.md](../docs/architecture/SOLUTION_OVERVIEW.md).
Profiles select which part of the stack to start.

| Profile | Containers | Purpose | Hardware |
|---|---|---|---|
| `sim`  | `sim` | full FLAT stack in simulation (Gazebo -> RTAB-Map -> Nav2 -> executive), one container | CPU |
| `edge` | `detector` + `orchestrator` | YOLOE detector + VLM orchestrator on the edge side | NVIDIA GPU |
| `all`  | all | sim + edge together | CPU + GPU (RAM-heavy) |

> The real **robot** layer (CAN/EPOS4/RealSense on Raspberry Pi) cannot be
> reproduced in Docker on a PC; physical hardware is required. See
> [../docs/HIL_BRINGUP_CHECKLIST.md](../docs/HIL_BRINGUP_CHECKLIST.md). On a PC,
> the `sim` container fills that role.

## Requirements

- **Docker** with **Compose v2** and **BuildKit** enabled. BuildKit is enabled by
  default in modern Docker and is required for `Dockerfile.dockerignore`.
- For the `edge` GPU profile: NVIDIA driver + **nvidia-container-toolkit** on the
  host (`--gpus`/`deploy.devices`). torch is installed from the `cu124` CUDA
  index (validated with torch 2.6.0+cu124); the CUDA runtime comes from the torch
  wheels, so a separate CUDA image is not required.

## Quick Start

```bash
cd ar_project/docker
cp .env.example .env                 # optional: ROS_DOMAIN_ID / RMW

# Option A: full FLAT simulation on the PC
docker compose --profile sim build
docker compose --profile sim up      # headless (gui:=false), no display required

# Option B: edge detector + orchestrator, GPU required
docker compose --profile edge build
docker compose --profile edge up
```

Or use `make` from `ar_project/docker/`: `make build`, `make sim`, `make edge`,
`make down`.

## Mission Trigger (inside the `sim` container)

After `up`, the clean FLAT stack is running. In a separate terminal:

```bash
# populate the map in a bounded world (SLAM unknown cells become frontiers)
docker compose exec sim bash -lc \
  'ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: 0.6}}"'  # Ctrl-C after ~5 s

# start a FLAT mission without VLM
docker compose exec sim bash -lc \
  "ros2 action send_goal /seek_object object_tracking_msgs/action/SeekObject \
   '{instruction: \"find bus\", request_id: \"m1\", mission_epoch: 0, allow_vlm: false}' --feedback"
```

Full scenarios (DETECT/APPROACH, VLM mode, monitoring) are documented in
[../docs/RUNBOOK.md](../docs/RUNBOOK.md).

## Volumes and Secrets

- **YOLOE weights** (~600 MB) are not baked into the image. They are mounted from
  `object_tracking/object_tracking/object_tracking/model_weights` into the
  package share path (see the `detector` service `volumes`). Put
  `yoloe-11s-seg.pt` and `mobileclip_blt.ts` there.
- **VLM credentials** are read from `object_tracking/planner_orchestrator/vlm.env`
  via `env_file` (`required: false`; without it, the orchestrator starts in mock
  mode). `*.env` files are not committed to git and are not copied into images.

## Network and Transport

Inside one `docker compose` project, all services share a network and DDS
discovery (`rmw_fastrtps_cpp`) works by default. The `sim` profile is
self-contained because all nodes run in one container. The production Pi-edge
Wi-Fi transport uses `rmw_zenoh` (see [../deploy/transport/](../deploy/transport/));
it is not required for a single-PC Docker setup.

## Limitations and Notes

- **RAM.** The full stack (Gazebo + RTAB-Map + Nav2 + YOLOE) is memory-hungry.
  On machines with about 4 GB of available RAM, start `sim` and `edge`
  **separately** rather than using `all`.
- **The `sim` build is the main risk.** The `ar_project` package (ament_cmake)
  pulls heavy dependencies. The CAN runtime (`canopen*`) and real RealSense
  driver are only needed on the robot, so the Dockerfile passes them through
  `--skip-keys`; the simulation uses `gz_ros2_control` and a Gazebo camera. If
  `rosdep` still fails on an unavailable key for your ROS version, add it to
  `--skip-keys`.
- **Without a robot or simulation**, `edge` containers start and wait for the ROS
  graph. This is useful together with the `sim` profile or with the real robot.
