# Pi / edge build + deploy

Near-zero-manual build of the two-tier stack: the **Pi** (robot) runs the executive
+ ros2_control hardware interface; the **edge** (GPU box) runs the detector + VLM
orchestrator + SLAM. `deploy.sh` (and the `Makefile` wrapper) handle source sync and
remote/local `colcon build` so you don't hand-build on each host.

## One-time setup
1. `make setup` → creates `deploy.env` from the example. Edit `PI_HOST` and `PI_USER`
   (the rest auto-detects). `deploy.env` is gitignored.
2. Passwordless SSH to the Pi: `ssh-copy-id $PI_USER@$PI_HOST` (so sync/build don't prompt).
3. On the **Pi** once: install ROS 2 + colcon + `rosdep`; `sudo rosdep init && rosdep update`.
   (The remote build runs `rosdep install` to pull any missing package deps.)

## Everyday use (from `deploy/build/`)
| command | what it does |
|---|---|
| `make edge`  | build `planner_orchestrator` + `object_tracking` (+deps) in `$EDGE_WS` on this box |
| `make pi`    | rsync both repos → Pi, then remote `colcon build` of `search_coordinator` + `ar_project` (+deps) |
| `make all`   | `edge` then `pi` |
| `make shell` | ssh into the Pi with `/opt/ros` + the workspace `install/` sourced |
| `make doctor`| verify repo paths + SSH reachability + ROS on both ends |

`make pi` is idempotent: rsync mirrors your working tree (uncommitted edits included),
excluding `build/ install/ log/ .git/ __pycache__/ model_weights/ *.pt *.ts *.env`, then
colcon `--symlink-install --packages-up-to` rebuilds only what changed. The 600 MB YOLOE
weights stay on the edge (the detector runs there) and are never shipped to the Pi.

## Tiers (deps resolved automatically by `--packages-up-to`)
- **Pi**: `search_coordinator`, `ar_project` → pulls `ar_project_msgs`, `object_tracking_msgs`,
  `fleet_comms`. C++ HW interface (`embodied_robot_system`) compiles natively on the Pi.
- **edge**: `planner_orchestrator`, `object_tracking` → pulls both msg packages + `fleet_comms`.

## Then bring up (see ../../docs/HIL_BRINGUP_CHECKLIST.md)
- edge: start `deploy/transport` zenoh router + `deploy/time_sync` chrony master.
- Pi: `ros2 launch ar_project hardware_bringup.launch.py` + the executive
  (`coordinator_node` + `frontier_extractor`).
- edge: `detect_target_server` (in the YOLOE venv) + `orchestrator_node` (VLM creds in env).
