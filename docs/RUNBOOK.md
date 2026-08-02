# RUNBOOK - запуск стека на симуляции и железе

Короткая рабочая инструкция для ветки `robust`. Safety-чеклист перед автономными
прогонами: `HIL_BRINGUP_CHECKLIST.md`. Логи экспериментов: `experiment_logs/vlm_missions/`.

## 0. Что где работает
- **Raspberry Pi:** моторы/CAN, RealSense, `/scan`, EKF, Nav2, `map_odom_relay`, `search_coordinator`.
- **Edge-ноутбук:** camera relay `/camera_edge/*`, RTAB-Map SLAM, dashboard/logger, detector, VLM-orchestrator.
- **FLAT:** миссией управляет executive на Pi, VLM не нужен.
- **VLM:** цель передается executive, затем executive делает handoff в `planner_orchestrator`.
- **Detector нужен обоим режимам:** FLAT тоже использует `detect_target_server` для DETECT/APPROACH.

## 1. Подготовка и сборка

Один раз на машине:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

Venv детектора на edge:

```bash
python3 -m venv --system-site-packages ~/.venvs/ros-jazzy-ml
source ~/.venvs/ros-jazzy-ml/bin/activate
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
pip install -r ~/ros2_ws/src/object_tracking/requirements.txt
```

VLM credentials, только для VLM-режима:

```bash
cp ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env.example \
   ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env
# заполнить VLM_BASE_URL / VLM_API_KEY / VLM_MODEL
```

## 2. Быстрый запуск на железе

В каждом терминале на **Pi** сначала:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh
```

В каждом терминале на **edge-ноутбуке** сначала:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh
```

### Raspberry Pi

**Pi T1 - hardware, motors, watchdog, twist mux, `/scan`**

```bash
ros2 launch ar_project hardware_bringup.launch.py
```

Если нужно включить collision monitor:

```bash
ros2 launch ar_project hardware_bringup.launch.py use_collision_monitor:=true
```

**Pi T2 - RealSense RGB-D + IMU**

```bash
ros2 launch ar_project realsense_rgbd_pi.launch.py \
  rgb_camera.color_profile:=640x480x6 \
  depth_module.depth_profile:=424x240x6
```

**Pi T3 - map->odom relay**

```bash
ros2 run search_coordinator map_odom_relay --ros-args \
  -p use_sim_time:=false
```

**Pi T4 - Nav2**

```bash
ros2 launch ar_project navigation_launch.py \
  use_sim_time:=false \
  odom_topic:=/odometry/filtered
```

**Pi T5 - executive / skill servers**

```bash
ros2 run search_coordinator coordinator_node --ros-args \
  -p use_sim_time:=false \
  -p approach_max_goal_step_m:=1.2 \
  -p approach_direct_clearance_m:=0.55 \
  -p approach_direct_if_goal_in_known_free_map:=true \
  -p approach_allow_unknown_bounded_goal:=true \
  -p approach_unknown_bounded_max_step_m:=0.6 \
  -p flat_initial_scan_forward_wait_s:=4.0 \
  -p flat_initial_scan_settle_s:=2.0 \
  -p flat_initial_scan_view_detect_wait_s:=2.0
```

В FLAT, если цель не найдена в стартовом кадре, coordinator делает фиксированный
несемантический обзор `forward -> right -> left`, затем переходит к `ExploreFrontier`.

### Edge-ноутбук

**Edge T1 - camera relay + RTAB-Map + dashboard/logger**

Для FLAT-экспериментов:

```bash
ros2 launch ar_project edge_bringup.launch.py \
  flat_log_run_id:=flat_scene_1 \
  start_vlm_logger:=false
```

Для VLM-экспериментов:

```bash
ros2 launch ar_project edge_bringup.launch.py \
  vlm_log_run_id:=vlm_scene_1 \
  start_flat_logger:=false
```

**Edge T2-FLAT - continuous detector (`/target_pixel`)**

FLAT-режим ждет поток `/target_pixel`, поэтому здесь нужен continuous tracker,
а не action-сервер VLM.

```bash
ros2 launch object_tracking sam_node.launch.py \
  model_mode:=dino_mobilesam \
  tracking_mode:=continuous \
  use_compressed_input:=false \
  image_topic:=/camera_edge/color/image_raw \
  use_depth_input:=true \
  depth_topic:=/camera_edge/aligned_depth_to_color/image_raw \
  input_reliability:=best_effort \
  target_publish_rate:=3.0 \
  enable_search_rotation:=false
```

Быстрая проверка после отправки FLAT-миссии:

```bash
ros2 topic info /target_pixel -v
timeout 8 ros2 topic echo /target_pixel --once
```

У `/target_pixel` должен быть publisher `rgb_tracker_node`.

**Edge T2-VLM - action detector / Set-of-Mark**

```bash
/home/user/.venvs/ros-jazzy-ml/bin/python -m object_tracking.detect_target_server \
  --ros-args \
  -p image_topic:=/camera_edge/color/image_raw \
  -p depth_topic:=/camera_edge/aligned_depth_to_color/image_raw \
  -p target_conf_default:=0.60
```

Defaults уже зашиты: `model_mode=dino`, `depth_point_strategy=nearest_mask`,
`use_compressed_input=false`.

**Edge T3 - VLM-orchestrator, только для VLM**

```bash
set -a
source ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env
set +a

/home/user/.venvs/ros-jazzy-ml/bin/python -m planner_orchestrator.orchestrator_node
```

**Edge T4 - RViz**

```bash
ros2 launch ar_project rviz_launch.py \
  use_sim_time:=false \
  config:=$(ros2 pkg prefix ar_project)/share/ar_project/config/rtabmap_rgbd.rviz
```

**Edge T5 - отправка миссии**

```bash
# FLAT
ros2 run fleet_comms send_mission "chair" false

# VLM
ros2 run fleet_comms send_mission "chair" true
```

Dashboard: `http://localhost:8088`.

## 3. Быстрая проверка перед миссией

Pi:

```bash
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
timeout 8 ros2 topic hz /scan
timeout 8 ros2 topic hz /odometry/filtered
```

Edge:

```bash
timeout 8 ros2 topic hz /camera_edge/color/image_raw
timeout 8 ros2 topic hz /camera_edge/aligned_depth_to_color/image_raw
timeout 8 ros2 topic hz /map_odom_correction
ros2 action list | grep detect
```

## 4. Быстрый запуск симуляции

VLM simulation:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export GZ_IP=127.0.0.1
set -a; source ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env; set +a

ros2 launch ar_project vlm_sim_bringup.launch.py \
  start_edge:=true \
  venv_python:=/home/user/.venvs/ros-jazzy-ml/bin/python \
  vlm_log_run_id:=sim_bus_001
```

Mission:

```bash
ros2 run fleet_comms send_mission "bus" true
```

FLAT simulation:

```bash
ros2 launch ar_project flat_sim_bringup.launch.py
ros2 run fleet_comms send_mission "bus" false
```

Если в FLAT нет frontier-ов, засей карту коротким вращением:

```bash
ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: 0.6}}" &
sleep 5
kill %1
ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: -0.6}}" &
sleep 5
kill %1
```

## 5. FLAT vs VLM

Операторская команда одна:

```bash
ros2 run fleet_comms send_mission "<цель>" <true|false>
```

- `false`: FLAT, `/seek_object allow_vlm=false`, миссией владеет executive; если цель не видна, выполняется фиксированный scan, затем `ExploreFrontier`.
- `true`: VLM, `/seek_object allow_vlm=true`, executive публикует instruction во внутренний `/vlm_mission`; Qwen выбирает действия по кадру, карте, context marks и памяти.
- `/vlm_mission` обычному оператору не нужен; это debug/internal topic.
- Для VLM нужен `Edge T3`; для FLAT `Edge T3` и `vlm.env` не нужны.

## 6. Как работает VLM-режим

На каждом шаге orchestrator отправляет в Qwen:

- target/instruction;
- `visible_marks`: строгие кандидаты цели, к ним можно делать `DRIVE_TO_VISIBLE`;
- `context_marks`: офисные объекты-подсказки, они не являются целями для подъезда;
- Set-of-Mark кадр с камеры;
- top-down SLAM map `/map` с позой робота;
- notes/memory: corridor scans, прошлые действия, причины отказов Nav2.

Действия VLM:

- `TURN`;
- `DRIVE_FORWARD`;
- `DRIVE_TO_VISIBLE`;
- `DETECT_ALL`, сейчас это refresh фиксированного DINO context-словаря;
- `DONE`.

Главное правило текущей логики: если цель не видна, робот исследует свободные
коридоры на карте; context-объекты помогают выбрать коридор, но не становятся
точками притяжения. Если цель уверенно найдена, orchestrator сохраняет ее map-точку
и продолжает подход к ней даже при временной потере объекта из кадра.

## 7. Важные параметры

| Параметр | Деф. | Смысл |
|---|---:|---|
| `target_conf_default` | `0.50` | Дефолт детектора; в HIL запускаем `0.60` для строгой target-детекции |
| `target_detect_conf` | `0.60` | Порог цели в orchestrator |
| `context_detect_conf` | `0.30` | Порог DINO context-объектов |
| `async_replan` | `false` | Дискретно: ехать -> стоп -> наблюдать -> думать |
| `turn_settle_s` | `2.0` | Пауза после TURN перед анализом кадра |
| `min_effective_turn_rad` | `0.60` | Малые TURN нормализуются, потому что Nav2 может засчитать их без движения |
| `initial_scan_when_target_absent` | `true` | Если цели нет, обзор: forward -> right -> left |
| `flat_initial_scan_enabled` | `true` | FLAT baseline тоже делает фиксированный обзор, но без VLM-выбора |
| `flat_initial_scan_forward_wait_s` | `1.5` | Сколько ждать стартовую target-детекцию перед FLAT-scan |
| `flat_initial_scan_settle_s` | `2.0` | Пауза после FLAT scan-TURN перед проверкой детекции |
| `approach_max_goal_step_m` | `1.2` | Bounded-step к далекой/плохо раскрытой цели |
| `approach_direct_clearance_m` | `0.55` | Радиус known-free вокруг direct standoff-точки |
| `approach_allow_unknown_bounded_goal` | `true` | Разрешить короткий cautious probe через unknown |
| `locked_target_approach_max_attempts` | `8` | Не забывать подтвержденную цель после потери кадра |
| `vlm_timeout_s` | `30.0` | Таймаут ответа VLM |
| `send_map` | `true` | Отправлять карту вторым изображением |

## 8. Мониторинг и логи

Dashboard:

```text
http://localhost:8088
```

CLI:

```bash
ros2 topic echo /robot_health
ros2 topic echo /vlm/activity
ros2 topic echo /mission/status
ros2 topic echo /planner/notes
ros2 topic echo /frontiers
ros2 node list
ros2 action list
```

Persistent mission logs пишутся автоматически из `edge_bringup.launch.py`.
Для осмысленных имён файлов добавляй `flat_log_run_id:=flat_scene_1` или
`vlm_log_run_id:=vlm_scene_1`.

```text
~/ros2_ws/experiment_logs/flat_missions/<run_id>.jsonl
~/ros2_ws/experiment_logs/flat_missions/<run_id>.csv
~/ros2_ws/experiment_logs/vlm_missions/<run_id>.jsonl
~/ros2_ws/experiment_logs/vlm_missions/<run_id>.csv
```

Timing поля: VLM CSV пишет `latency_ms` и `time_to_first_action_s`; FLAT CSV
пишет `time_to_detect_s` и `time_to_approach_s`.

Ручные loggers:

```bash
ros2 run fleet_comms flat_mission_logger --ros-args \
  -p output_dir:=~/ros2_ws/experiment_logs/flat_missions \
  -p run_id:=flat_scene_1

ros2 run fleet_comms vlm_mission_logger --ros-args \
  -p output_dir:=~/ros2_ws/experiment_logs/vlm_missions \
  -p run_id:=vlm_scene_1
```

## 9. Ожидаемая деградация

Если VLM недоступна или таймаутит, orchestrator открывает circuit-breaker и
переходит в DEGRADED/FLAT fallback. Миссия не должна аварийно останавливаться.
При потере edge/Wi-Fi executive на Pi сохраняет автономность.

## 10. Частые проблемы

- **Нет `/camera_edge/*`:** проверь RealSense на Pi и `edge_bringup` на edge.
- **Detector не стартует из-за `torch`:** запускай через `/home/user/.venvs/ros-jazzy-ml/bin/python -m object_tracking.detect_target_server`.
- **Detector есть, но нет глубины:** проверь `/camera_edge/aligned_depth_to_color/image_raw`.
- **Nav2 node not found:** перезапусти `navigation_launch.py`, затем проверь lifecycle nodes.
- **Робот не двигается:** проверь `/cmd_vel_out`, `/cmd_vel_collision_safe`, `/diff_cont/cmd_vel` и watchdog.
- **Collision monitor блокирует движение:** проверь timestamp `/scan`, TF `camera_link -> base_link`, lifecycle `/collision_monitor`.
- **VLM видит цель, но `DRIVE_TO_VISIBLE` не едет:** смотри в `search_coordinator` причину `ABORTED`: `clearance_occupied`, `clearance_unknown`, `outside_map`.
- **Ложные target-детекции:** подними `target_conf_default`/`target_detect_conf`; context-порог не трогай первым.
- **Карта не уходит в VLM (`map=no`):** нет `/map`, `send_map=false` или orchestrator запущен не из venv с cv2/numpy.
- **Gazebo падает SIGSEGV:** только симуляция, перед запуском `export GZ_IP=127.0.0.1`.
