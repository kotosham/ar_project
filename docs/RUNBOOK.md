# RUNBOOK — сборка, поднятие и запуск (симуляция + реальное железо)

Подробное руководство по эксплуатации стека `robust`. Аппаратные **safety**-барьеры (предохранительные шлюзы)
см. в `HIL_BRINGUP_CHECKLIST.md`; измеренные показатели — в `FLAT_BASELINE.md`; автоматизацию
сборки/развёртывания — в `deploy/build/README.md`.

## 0. Архитектура (кто что запускает)
- **Pi (робот):** executive (исполнительный слой) `search_coordinator` (SeekObject FSM + 5 skill-серверов +
  `frontier_extractor`) · аппаратный интерфейс ros2_control (`embodied_robot_system`, CAN/EPOS4) ·
  RealSense · локальный `/scan` (depthimage_to_laserscan) · облегчённый Nav2 · `map_odom_relay`.
- **Edge (GPU-машина):** RTAB-Map RGB-D SLAM · `detect_target_server` (YOLOE, в venv) ·
  `planner_orchestrator` (VLM). Сама VLM-модель — это внешний OpenAI-совместимый API.
- **Два режима:** FLAT (без VLM, executive автономен) и VLM (orchestrator управляет
  skill-ами executive, при потере связи деградирует обратно к FLAT).

---

## 1. Предварительные требования (один раз на машину)
- ROS 2 **Jazzy** + colcon + rosdep. Один раз `sudo rosdep init && rosdep update`.
- Venv детектора на edge (для него нужен torch; у системного shebang ноды torch нет):
  ```bash
  python3 -m venv --system-site-packages ~/.venvs/ros-jazzy-ml
  source ~/.venvs/ros-jazzy-ml/bin/activate
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
  pip install -r ~/ros2_ws/src/object_tracking/requirements.txt
  ```
  Веса YOLOE лежат в `object_tracking/object_tracking/model_weights/`
  (`yoloe-11s-seg.pt` + `mobileclip_blt.ts`).
- Учётные данные VLM (только для режима VLM): скопируйте `object_tracking/planner_orchestrator/vlm.env.example`
  → `vlm.env`, заполните `VLM_BASE_URL` / `VLM_API_KEY` / `VLM_MODEL`. Загрузите перед запуском
  orchestrator: `set -a; source vlm.env; set +a`.

## 2. Сборка
- **Симуляция (одна машина):** `colcon build` в вашем workspace, затем `source install/setup.bash`.
- **Pi + edge (реальный робот):** из `ar_project/deploy/build/`: `make setup` (заполните
  `deploy.env`), затем `make all` — собирает набор edge локально и через rsync+удалённую сборку
  собирает набор Pi. `make doctor` сначала проверяет SSH/ROS.

---

## 2.1. Быстрый запуск VLM по терминалам

Этот раздел — короткая карточка запуска именно **VLM-режима**. Подробные пояснения, параметры и
диагностика остаются ниже в разделах 3–8.

### Симуляция VLM на одной машине

**T1 — весь симуляционный стек + detector + VLM-orchestrator**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
export GZ_IP=127.0.0.1
set -a; source ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env; set +a

ros2 launch ar_project vlm_sim_bringup.launch.py \
  start_edge:=true \
  venv_python:=/home/user/.venvs/ros-jazzy-ml/bin/python
```

Что поднимается: Gazebo, RViz, SLAM, Nav2, `search_coordinator`, dashboard,
`detect_target_server` и `planner_orchestrator`.

**T2 — отправка VLM-миссии**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic pub --once /vlm_mission std_msgs/msg/String "{data: 'bus'}"
```

Dashboard: `http://localhost:8088`.

### Реальное железо: Pi + edge-ноутбук

#### Raspberry Pi

**Pi T1 — железо, моторы, watchdog/twist mux, `/scan`**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

ros2 launch ar_project hardware_bringup.launch.py
```

`collision_monitor` на железе по умолчанию выключен: при нестабильных timestamp
`/scan` он блокирует весь контур движения сообщениями `Robot to stop due to
invalid source`. Если `/scan` и TF проверены и нужны реактивные stop/slowdown
полигоны, включайте явно:

```bash
ros2 launch ar_project hardware_bringup.launch.py use_collision_monitor:=true
```

**Pi T2 — RealSense RGB-D + IMU**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

ros2 launch ar_project realsense_rgbd_pi.launch.py \
  rgb_camera.color_profile:=640x480x6 \
  depth_module.depth_profile:=424x240x6
```

Штатный режим для VLM-тестов на Wi-Fi: RGB `640x480x6` и depth `424x240x6`.
Edge-детектор масштабирует RGB-координаты в depth-сетку перед выборкой глубины.
Если нужно вернуться к старому профилю, запусти этот же launch без аргументов
или явно задай `640x480x15` / `424x240x15`.

**Pi T2a — сжатие камеры (Level-0, сразу после старта камеры)**

Единственный сжатый поток камеры через Wi-Fi кодируется плагинами `image_transport`
**внутри узла RealSense на Pi**. Два параметра публикатора решают почти всё:

- `compressedDepth.format = rvl` (вместо `png`) — PNG-кодирование 16-битной глубины
  это самая дорогая CPU-операция всего камерного тракта на малинке; RVL сделан под
  depth и радикально дешевле;
- `compressed.jpeg_quality = 75` (вместо 95) — примерно вдвое меньше байт цвета в
  эфир, на YOLOE/DINO/VLM при 640×480 не влияет.

Это параметры `image_transport`, а не RealSense, поэтому задаются в рантайме (rs_launch
не пробрасывает произвольные имена параметров). Применяются со следующего кадра и
мгновенно откатываются:

```bash
# применить (узел по умолчанию /camera/camera):
bash ~/ros2_ws/src/ar_project/deploy/tune_camera_compression.sh
# откатить к дефолтам:
bash ~/ros2_ws/src/ar_project/deploy/tune_camera_compression.sh /camera/camera revert
```

Скрипт сам находит точные имена параметров через `ros2 param list` (не зависит от
namespace). Проверить имена вручную:

```bash
ros2 param list /camera/camera | grep -iE 'compressedDepth|jpeg_quality'
```

Параметры `compressedDepth.*` объявляются лениво — только когда на топик подпишется
потребитель compressedDepth (edge-relay). Если скрипт пишет «no compressedDepth.format
param found», сначала подними `edge_camera_relay` на edge, затем запусти скрипт снова.

**Pi T3 — map->odom correction relay, запускать до Nav2**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

ros2 run search_coordinator map_odom_relay --ros-args \
  -p use_sim_time:=false
```

**Pi T4 — Nav2**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

ros2 launch ar_project navigation_launch.py \
  use_sim_time:=false \
  odom_topic:=/odometry/filtered
```

**Pi T5 — executive / skill servers для VLM**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

ros2 run search_coordinator coordinator_node --ros-args \
  -p use_sim_time:=false
```

#### Edge-ноутбук

**Edge T1 — camera relay + RTAB-Map + dashboard**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

ros2 launch ar_project edge_bringup.launch.py
```

Что поднимается: единственный Wi-Fi consumer камеры, локальные `/camera_edge/*`, RTAB-Map и
dashboard. Этот терминал **не** запускает detector и VLM-orchestrator.
Для RealSense 6 FPS внутри `edge_bringup` RTAB-Map запускается с расширенным
RGB-D sync-окном: `approx_sync_max_interval:=0.5`, `topic_queue_size:=120`,
`sync_queue_size:=120`, `detection_rate:=1`.

**Edge T2 — detector / Set-of-Mark**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

/home/user/.venvs/ros-jazzy-ml/bin/python -m object_tracking.detect_target_server \
  --ros-args \
  -p use_sim_time:=false \
  -p model_mode:=hybrid_dino_yoloe \
  -p image_topic:=/camera_edge/color/image_raw \
  -p depth_topic:=/camera_edge/aligned_depth_to_color/image_raw \
  -p use_compressed_input:=false \
  -p depth_point_strategy:=nearest_mask \
  -p target_conf_default:=0.50 \
  -p vocab_conf_default:=0.12
```

В `hybrid_dino_yoloe` конкретная цель (`chair`, `office chair`, `bus`) детектируется через
GroundingDINO+MobileSAM, а `DETECT_ALL` остается на YOLOE broad-vocab, чтобы обзор сцены не
становился слишком тяжелым.
YOLOE в hybrid-режиме грузится **лениво** — только при первом `DETECT_ALL` (в логе старта
`vocab_backend=yoloe (lazy)`), поэтому миссия без `DETECT_ALL` не тратит на него VRAM и время
старта. Первый `DETECT_ALL` в свежем процессе детектора платит разовую загрузку (~2–4 с);
если он из-за этого разово упрётся в `detect_timeout_s`, модель всё равно останется в памяти
и следующий вызов отработает штатно (при желании поднять `detect_timeout_s` для первого вызова).
`depth_point_strategy:=nearest_mask` означает, что пиксель для `DRIVE_TO_VISIBLE` выбирается
по ближайшей валидной глубине внутри маски объекта, а не по геометрическому центру маски.

**Edge T3 — VLM-orchestrator**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh
set -a
source ~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env
set +a

/home/user/.venvs/ros-jazzy-ml/bin/python -m planner_orchestrator.orchestrator_node \
  --ros-args \
  -p use_sim_time:=false \
  -p use_mock:=false \
  -p async_replan:=false \
  -p replan_every_n:=3 \
  -p max_steps:=40 \
  -p detect_conf:=0.0 \
  -p target_detect_conf:=0.50 \
  -p detect_all_conf:=0.12 \
  -p vlm_timeout_s:=30.0 \
  -p send_map:=true \
  -p motion_fallback_frame:=odom \
  -p camera_image_topic:=/camera_edge/color/image_raw
```

**Edge T4 — RViz**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

ros2 launch ar_project rviz_launch.py \
  use_sim_time:=false \
  config:=$(ros2 pkg prefix ar_project)/share/ar_project/config/rtabmap_rgbd.rviz
```

**Edge T5 — отправка VLM-миссии**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

ros2 topic pub --once /vlm_mission std_msgs/msg/String "{data: 'chair'}"
```

Dashboard: `http://localhost:8088` на edge-ноутбуке.

Минимальная проверка перед миссией:

```bash
# Pi
ros2 lifecycle get /planner_server
ros2 lifecycle get /controller_server
ros2 lifecycle get /bt_navigator
timeout 8 ros2 topic hz /scan
timeout 8 ros2 topic hz /odometry/filtered

# Edge
timeout 8 ros2 topic hz /camera_edge/color/image_raw
timeout 8 ros2 topic hz /camera_edge/aligned_depth_to_color/image_raw
timeout 8 ros2 topic hz /map_odom_correction
ros2 action list | grep detect
```

Перед автономным запуском на железе сначала пройти safety-проверки из `HIL_BRINGUP_CHECKLIST.md`,
раздел B: колёса над землёй, quick-stop, watchdog, collision monitor.

---

## 3. СИМУЛЯЦИЯ (полностью протестированный путь)

### 3a. FLAT-миссия (без VLM) — одна команда
```bash
# T1: bring up the whole FLAT stack (sim -> SLAM -> Nav2 -> executive)
ros2 launch ar_project flat_sim_bringup.launch.py
#   default world = flat_detect.world with the bus billboard target.
#   frontier-only world override:
#   world:=$(ros2 pkg prefix ar_project)/share/ar_project/worlds/oscillation.world
```
Подождите ~35 с, пока не увидите `search_coordinator up (Phase 2.2) ... epoch=0`.
```bash
# T2: bootstrap the map in a bounded world (gives SLAM unknown cells -> frontiers).
#     net-zero in-place rotation; robot ends where it started.
ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: 0.6}}" &  sleep 5
kill %1; ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: -0.6}}" &  sleep 5; kill %1
ros2 topic echo /frontiers --once          # expect a non-empty list

# T2: start the FLAT mission (allow_vlm:=false)
ros2 action send_goal /seek_object object_tracking_msgs/action/SeekObject \
  "{instruction: 'find bus', request_id: 'm1', mission_epoch: 0, allow_vlm: false}" --feedback
```
FSM проходит SEARCH (едет к frontier-ам) → при свежем `/target_pixel` → DETECT → APPROACH.

### 3b. Реальная детекция (этап DETECT) — запустите детектор в venv на edge
Используйте `flat_detect.world` (в нём есть билборд bus.jpg, который YOLOE надёжно детектирует):
```bash
/home/user/.venvs/ros-jazzy-ml/bin/python -m object_tracking.detect_target_server \
  --ros-args -p use_sim_time:=true
```
DETECT/APPROACH executive потребляет `/target_pixel`. (Без детектора для тестов можно
подать синтетический — см. шаблон `~/inject_pixel.py` в FLAT_BASELINE.)

### 3c. VLM-миссия
Для VLM-сценария используйте отдельный launch, чтобы не путать его с FLAT-миссией.
Он поднимает ту же безопасную базу (Gazebo → SLAM → Nav2 → executive/skill servers), но
по умолчанию открывает `flat_detect.world` с баннером `bus.jpg`, включает Gazebo GUI и RViz.

Если хотите, чтобы launch сразу поднял edge-часть (`detect_target_server` + `planner_orchestrator`),
сначала загрузите VLM-переменные в shell. Креды launch не печатает и не хранит:
```bash
set -a; source object_tracking/planner_orchestrator/vlm.env; set +a
ros2 launch ar_project vlm_sim_bringup.launch.py start_edge:=true
```

Если edge-часть запускаете руками, оставьте `start_edge:=false` или просто не указывайте его:
```bash
ros2 launch ar_project vlm_sim_bringup.launch.py
```

После запуска миссии командует именно VLM-orchestrator, а не FLAT `/seek_object`:
```bash
ros2 topic pub --once /vlm_mission std_msgs/msg/String "{data: bus}"
```

Ручной вариант edge-части:
```bash
# detector running (3b) + executive up. In the orchestrator shell:
set -a; source object_tracking/planner_orchestrator/vlm.env; set +a   # loads VLM_* (never printed)
/home/user/.venvs/ros-jazzy-ml/bin/python -m planner_orchestrator.orchestrator_node \
  --ros-args -p use_sim_time:=true -p use_mock:=false -p replan_every_n:=3 -p max_steps:=40 \
  -p async_replan:=false -p detect_conf:=0.0 \
  -p target_detect_conf:=0.50 -p detect_all_conf:=0.12
# expect: "planner_orchestrator up ... client=OpenAICompatibleClient creds=env"
ros2 topic pub --once /vlm_mission std_msgs/msg/String "{data: bus}"   # ЧИСТЫЙ лейбл (см. ниже)
```
Orchestrator забирает реальные Set-of-Mark-кандидаты из `detect_target_server`, шлёт VLM наблюдение
(см. ниже) и диспетчеризует выбранное действие в skill-ы executive. Для офлайн-прогона используйте
`-p use_mock:=true` (ключ API не нужен — это же FLAT-фоллбэк при деградации).

> Запускать орхестратор через `/home/user/.venvs/ros-jazzy-ml/bin/python -m planner_orchestrator.orchestrator_node`
> (а не `ros2 run`): ему
> нужны cv2/numpy для рендера карты и кодирования кадра; venv это гарантирует. Без cv2 карта молча
> отключается (`send_map` авто-off).

#### Словарь действий VLM (что модель может выбрать)
`TURN`(угол) · `DRIVE_FORWARD`(±метры, − = назад) · `DRIVE_TO_VISIBLE`(mark_id → ApproachDetection через
Nav) · `DETECT_ALL`(детект всех объектов + классы, в notes) · `DONE`. Высокоуровневого `GO_TO_FRONTIER`
больше нет — модель навигирует сырым движением (честное сравнение с FLAT). Stop — только safety-фоллбэк
исполнителя, не действие VLM.

#### Что уходит в VLM каждый ход
Текст (`target`, `visible_marks`=[mark_id, label, score, **distance_m** с RealSense], notes) + **1-е
изображение** (камера с номерами марок) + **2-е изображение** (top-down SLAM-карта `/map` с позой
робота) + описание карты. Карта рендерится из `/map` (RTAB-Map), edge-локально.

#### Ключевые параметры orchestrator
| Параметр | Деф. | Зачем |
|---|---|---|
| `async_replan` | `true` | `false` = дискретные шаги (едь→стоп→свежее наблюдение→думай). Для наблюдения/сравнения ставь `false` |
| `detect_conf` | `0.0` | Legacy override: если >0, одним числом переопределяет оба порога ниже |
| `target_detect_conf` | `0.50` | Порог конкретной цели для DINO+MobileSAM (`chair`, `drawer cabinet`) — как в базовой дипломной реализации |
| `detect_all_conf` | `0.12` | Порог `DETECT_ALL` для YOLOE broad-vocab — как в базовой дипломной реализации |
| `send_map` | `true` | `false` = не слать карту 2-м изображением (легче запрос; если эндпоинт таймаутит) |
| `map_max_px` | `384` | Макс. сторона рендера карты |
| `vlm_timeout_s` | `8.0` | На медленном эндпоинте/с картой подними до `30–60`, иначе circuit-breaker → DEGRADED |
| `replan_every_n` | `3` | Реальная VLM всё равно отдаёт 1 действие за вызов |

> **Цель — ЧИСТЫЙ лейбл объекта** (`bus`, НЕ `find a bus`/`ride to bus`): нормализации запроса нет,
> YOLOE матчит строку как один класс. `bus` → conf ~0.66; `ride to bus` → ~0.45 (слабее, ложные
> «доехал» у края кадра). Для target-детекции держите `target_detect_conf:=0.50`;
> если цель пропадает, временно снижайте до `0.35–0.40`. `DETECT_ALL` оставляйте мягче:
> `detect_all_conf:=0.12`, иначе обзор сцены станет слишком бедным.

> Замечание по RAM: gz + RTAB-Map + Nav2 + YOLOE вместе требуют >4 ГБ. На хосте с ≤4 ГБ запускайте
> либо детектор отдельно (мир из 3b), ЛИБО nav-стек, но не всё сразу.

> Замечание по симуляции (только gz): перед запуском sim-стека ставь `export GZ_IP=127.0.0.1` — иначе
> `gz sim` периодически падает с SIGSEGV в потоке gz-transport discovery (известный баг на WSL). На
> железе Gazebo нет — там GZ_IP не нужен. Лончеры `watch_vlm.sh`/`watch_flat.sh` уже это делают.

---

## 4. ЖЕЛЕЗО

Актуальная последовательность терминалов для реального робота находится в короткой карточке
запуска: **§2.1 → "Реальное железо: Pi + edge-ноутбук"**. Этот раздел намеренно не
дублирует команды, чтобы в RUNBOOK не было двух расходящихся наборов.

Перед автономной миссией сначала пройти safety-проверки из `HIL_BRINGUP_CHECKLIST.md`,
раздел B: колёса над землёй, quick-stop, watchdog, collision monitor. Полная
последовательность ввода в эксплуатацию (time-sync, transport, восприятие, FLAT→VLM,
деградация) — там же, разделы C–I.

---

## 5. Запуск миссий
- **FLAT:** action `/seek_object`, `allow_vlm: false` — миссией владеет executive.
- **VLM:** опубликуйте цель в `/vlm_mission` (std_msgs/String) — ею владеет orchestrator и
  управляет skill-ами executive. (`allow_vlm: true` в goal `/seek_object` — это флаг на стороне
  executive для интегрированного режима.)
- **Смена цели посреди миссии:** отправьте новый goal `/seek_object` (или новый `/vlm_mission`) —
  epoch инкрементируется, старая миссия получает PREEMPTED, незавершённые skill-goal-ы отклоняются как zombie.

## 6. Мониторинг

### 6a. Веб-дашборд миссии (основной инструмент)
`edge_bringup.launch.py` (железо) и `vlm_sim_bringup.launch.py` (симуляция) автоматически
поднимают **человекочитаемый веб-монитор**: откройте `http://<edge-host>:8088`
(или запустите вручную: `ros2 run fleet_comms mission_dashboard`). На нём:

- **Состояние компонентов робота** — по строке на каждый элемент (RealSense, EKF,
  приводы EPOS4/CAN, `/scan`, Nav2, SLAM-коррекция, детектор, VLM-оркестратор,
  executive FSM): что элемент принимает (возраст/частота сообщений) и что отказало
  (OK / ВНИМАНИЕ / НЕТ ДАННЫХ / ОТКАЗ). Источник — `/robot_health`
  (`robot_health_aggregator` на Pi, 1 Гц, входит в `hardware_bringup`).
- **Что думает и делает VLM** — живая лента `/vlm/activity`: что VLM увидела
  (детекции + уверенность + дистанция), что решила (действия + обоснование +
  латентность), что реально произошло (выполнено/провал + длительность), память
  (notes), деградация circuit-breaker.
- **Что видит робот** — последний Set-of-Mark кадр (`/vlm/setofmark`) и карта,
  отправленная VLM (`/vlm/map_view`).
- **FSM executive** — латченый `/mission/status` (состояние/подзадача/прогресс/итог).
- Heartbeat-строки — cpu / латентность / mission_epoch каждого продюсера; статусы
  теперь **реальные**: оркестратор публикует DEGRADED при открытом circuit-breaker,
  детектор — DOWN при упавшем бэкенде и DEGRADED без кадров камеры.

### 6b. CLI (как раньше)
- `ros2 topic echo /robot_health` — те же строки здоровья в сыром виде.
- `ros2 topic echo /vlm/activity` — JSON-события VLM; `ros2 topic echo /mission/status` — FSM.
- `ros2 topic echo /planner/notes` — компактные заметки VLM + token_estimate (вкл. результаты DETECT_ALL).
- `ros2 topic echo /frontiers` — список frontier-ов + закоммиченный id (для FLAT).
- `ros2 action list` / `ros2 node list` — подтвердить, что серверы подняты.
- Heartbeat-ы: executive логирует `/heartbeat deadline missed`, когда продюсер (edge) молчит.
- Логи orchestrator (3 строки на шаг) — видно, **что VLM видела и что решила**:
  ```
  observe@step N: 1 detection(s) best='bus' conf=0.66 @1.68m, notes=2, map=yes -> asking OpenAICompatibleClient
  plan@step N: VLM returned 1 action(s): DRIVE_FORWARD +0.50m
  step N: DRIVE_FORWARD +0.50m -- <rationale>
  ```
  `conf` низкая (≈0.45) → детекция слабая/краевая (см. `target_detect_conf`). `map=no` → карта не пришла
  (`/map` нет или `send_map:=false`). `DEGRADED: ran in FLAT fallback` в конце → VLM-эндпоинт упал
  (circuit-breaker), миссия доехала на mock — подними `vlm_timeout_s` / поставь `send_map:=false`.

## 7. Деградация (FMEA 5.1) — ожидаемое поведение
Если VLM потерян (таймаут/недоступность → circuit-breaker открывается), orchestrator **защёлкивается
на FLAT MockPlanner, и миссия ПРОДОЛЖАЕТСЯ (DEGRADED)** — она не останавливается. При реальной
потере edge/Wi-Fi executive на Pi сохраняет FLAT-автономность. ApproachDetection никогда не объявляет
ложный `reached` по устаревшему пикселю (возвращает STALE_DETECTION / LOST_TARGET).

## 8. Устранение неполадок
- **Relay на edge не получает кадры (`/camera_edge/*` пустые):** на Pi должны стоять плагины
  сжатия image_transport: `sudo apt install ros-jazzy-image-transport-plugins` (даёт
  `/camera/.../image_raw/compressed` и `.../compressedDepth`). Проверка на Pi:
  `ros2 topic list | grep compressed`. Сжатие ленивое — топик появляется под подписчиком.
- **Spawn зацикливается на "Requesting list of world names" / нет `/odom`:** завис старый сервер `gz sim` —
  `pkill -9 -f 'gz sim'; pkill -9 -f ruby` и перезапустите.
- **Робот не двигается в ограниченном мире:** ещё нет frontier-ов — выполните засев вращением (3a, T2),
  чтобы SLAM получил unknown-ячейки. Убедитесь, что `/frontiers` не пуст.
- **`explore_frontier: nav drive terminal=no_server`:** Nav2 ещё не активен — он больше не
  заносит в blacklist; он ждёт (`explore_nav_ready_timeout_s`) и повторяет попытку. Дайте Nav2 время активироваться.
- **Детектор: `ModuleNotFoundError: torch`:** вы запустили его системным python — используйте
  `/home/user/.venvs/ros-jazzy-ml/bin/python -m object_tracking.detect_target_server`.
- **DRIVE_TO_VISIBLE не едет:** кандидату нужна метрическая глубина — убедитесь, что топик глубины
  публикуется (`/camera/camera/aligned_depth_to_color/image_raw`) и задан `use_depth:=true`.
- **Неожиданно `client=MockVlmClient` у VLM:** не разрешён base_url — выполните `source vlm.env` (или передайте
  `-p vlm_base_url:=`) и не передавайте `use_mock:=true`.
- **Миссия уходит в `DEGRADED` (FLAT fallback):** реальный VLM-эндпоинт упал/таймаутил 3 раза подряд
  (circuit-breaker). Подними `-p vlm_timeout_s:=30..60` и/или `-p send_map:=false` (два изображения тяжелее).
- **VLM объявляет `DONE`/`reached`, а робот не у цели:** слабая/краевая детекция или неверная глубина.
	  Признак — низкий `conf` или `distance_m=null/unknown` в `observe@`. Используй ЧИСТЫЙ лейбл
	  (`bus`, не `ride to bus`). Если проблема именно в ложных детекциях — временно подними
	  `target_detect_conf`; если проблема в глубине — проверь `/camera_edge/aligned_depth_to_color/image_raw`.
- **VLM «видит» цель, хотя не смотрит на неё:** проверь `conf`/`distance_m` в `observe@`. FOV камеры
  всего ~62° (±31°); если цель реально вне кадра — детектор отдаёт `0 detection(s)` (проверено). Если
	  детекция есть — край баннера попал в кадр. Подними `target_detect_conf`, чтобы реагировать только на уверенные.
- **`gz sim` падает SIGSEGV на старте (ТОЛЬКО симуляция):** баг gz-transport discovery на WSL —
  `export GZ_IP=127.0.0.1` перед запуском + `pkill -9 -f 'gz sim'; pkill -9 -f ruby`. На железе Gazebo нет.
- **Карта не приходит в VLM (`map=no` в логе):** нет паблишера `/map` (RTAB-Map не поднят на edge) или
  `send_map:=false`, или у оркестратора нет cv2/numpy (запусти его из `~/.venvs/ros-jazzy-ml`).
