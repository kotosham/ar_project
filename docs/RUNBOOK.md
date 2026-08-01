# RUNBOOK — сборка, поднятие и запуск (симуляция + реальное железо)

Подробное руководство по эксплуатации стека `robust`. Аппаратные **safety**-барьеры (предохранительные шлюзы)
см. в `HIL_BRINGUP_CHECKLIST.md`; измеренные показатели — в `FLAT_BASELINE.md`; автоматизацию
сборки/развёртывания — в `deploy/build/README.md`.

## 0. Архитектура (кто что запускает)
- **Pi (робот):** executive (исполнительный слой) `search_coordinator` (SeekObject FSM + 5 skill-серверов +
  `frontier_extractor`) · аппаратный интерфейс ros2_control (`embodied_robot_system`, CAN/EPOS4) ·
  RealSense · локальный `/scan` (depthimage_to_laserscan) · облегчённый Nav2 · `map_odom_relay`.
- **Edge (GPU-машина):** RTAB-Map RGB-D SLAM · `detect_target_server` (DINO+MobileSAM, в venv) ·
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
  DINO веса берутся из локального Hugging Face cache; YOLOE веса остаются в репозитории только
  для старых/сравнительных запусков `model_mode:=yoloe`.
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
  venv_python:=/home/user/.venvs/ros-jazzy-ml/bin/python \
  vlm_log_run_id:=sim_bus_001
```

Что поднимается: Gazebo, RViz, SLAM, Nav2, `search_coordinator`, dashboard,
VLM mission logger, `detect_target_server` и `planner_orchestrator`.

**T2 — отправка VLM-миссии**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 run fleet_comms send_mission "bus" true
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
invalid source`. Если `/scan` и TF проверены и нужна реактивная защита,
включайте явно. Текущий штатный safety-режим — stop-only: `PolygonSlow`
отключён, потому что slowdown `30%` делает малые VLM/Nav2-команды слишком
слабыми и Nav2 может падать в `Failed to make progress`.

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
  -p use_sim_time:=false \
  -p approach_max_goal_step_m:=1.2 \
  -p approach_direct_clearance_m:=0.55 \
  -p approach_direct_if_goal_in_known_free_map:=true \
  -p approach_allow_unknown_bounded_goal:=true \
  -p approach_unknown_bounded_max_step_m:=0.6
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

ros2 launch ar_project edge_bringup.launch.py \
  vlm_log_run_id:=office_chair_001
```

Что поднимается: единственный Wi-Fi consumer камеры, локальные `/camera_edge/*`, RTAB-Map и
dashboard, а также VLM mission logger. Этот терминал **не** запускает detector и VLM-orchestrator.
Для RealSense 6 FPS внутри `edge_bringup` RTAB-Map запускается с расширенным
RGB-D sync-окном: `approx_sync_max_interval:=0.5`, `topic_queue_size:=120`,
`sync_queue_size:=120`, `detection_rate:=1`.

**Edge T2 — detector / Set-of-Mark**

Общий терминал для **FLAT** и **VLM**: executive в FLAT тоже использует
`detect_target_server` на этапе DETECT/APPROACH.

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
unset ROS_LOCALHOST_ONLY ROS_STATIC_PEERS ROS_AUTOMATIC_DISCOVERY_RANGE ROS_DISCOVERY_SERVER FASTDDS_BUILTIN_TRANSPORTS FASTRTPS_DEFAULT_PROFILES_FILE
export ROS_DISABLE_ROS2CLI_DAEMON=1
source ~/ros2_ws/src/ar_project/deploy/transport/transport_env.sh

/home/user/.venvs/ros-jazzy-ml/bin/python -m object_tracking.detect_target_server \
  --ros-args \
  -p image_topic:=/camera_edge/color/image_raw \
  -p depth_topic:=/camera_edge/aligned_depth_to_color/image_raw \
  -p target_conf_default:=0.60
```

По дефолту детектор запускается в `model_mode:=dino`: конкретная цель
(`chair`, `office chair`, `bus`) детектируется через
GroundingDINO+MobileSAM. Когда нужна обзорная семантика сцены, orchestrator отправляет
в тот же DINO не пустой `DETECT_ALL`, а фиксированный context-запрос по офисному словарю.
YOLOE в этом hardware/VLM режиме не используется.
Дефолтный `depth_point_strategy:=nearest_mask` означает, что пиксель для `DRIVE_TO_VISIBLE` выбирается
по ближайшей валидной глубине внутри маски объекта, а не по геометрическому центру маски.
Когда конкретная цель не найдена, orchestrator автоматически делает context-обзор сцены и
передает VLM `context_marks`: нецелевые объекты с `side=left/center/right`, расстоянием и
семантической релевантностью. Для офисных целей (`office chair`, `desk`, `cabinet`) context
идет через GroundingDINO по офисному словарю (`desk`, `table`, `drawer cabinet`, `cabinet`,
`shelf`, ...), потому что YOLOE в низком ракурсе часто путает офисную мебель с мусорными
классами. Эти объекты не являются финальной целью для `DRIVE_TO_VISIBLE`, но помогают выбрать
осмысленное направление исследования.

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

/home/user/.venvs/ros-jazzy-ml/bin/python -m planner_orchestrator.orchestrator_node
```

Для VLM hardware-режима рабочие значения уже зашиты дефолтами: edge-camera
`/camera_edge/color/image_raw`, `async_replan=false`, `vlm_timeout_s=30.0`,
`initial_scan_when_target_absent=true`, `context_detect_conf=0.30`,
`semantic_turn_max_streak=1`, `turn_settle_s=2.0`,
`approach_max_goal_step_m=1.2`, `locked_target_approach_max_attempts=8`.

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

ros2 run fleet_comms send_mission "chair" true
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
ros2 run fleet_comms send_mission "bus" false
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

После запуска миссии командует именно VLM-orchestrator, а `/seek_object`
используется как единая точка входа и bridge в `/vlm_mission`:
```bash
ros2 run fleet_comms send_mission "bus" true
```

Ручной вариант edge-части:
```bash
# detector running (3b) + executive up. In the orchestrator shell:
set -a; source object_tracking/planner_orchestrator/vlm.env; set +a   # loads VLM_* (never printed)
/home/user/.venvs/ros-jazzy-ml/bin/python -m planner_orchestrator.orchestrator_node
# expect: "planner_orchestrator up ... client=OpenAICompatibleClient creds=env"
ros2 run fleet_comms send_mission "bus" true
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
Nav) · `DETECT_ALL`(обновить фиксированный context-словарь в notes) · `DONE`. Высокоуровневого `GO_TO_FRONTIER`
больше нет — модель навигирует сырым движением (честное сравнение с FLAT). Stop — только safety-фоллбэк
исполнителя, не действие VLM.

#### Что уходит в VLM каждый ход
Текст (`target`, `visible_marks`=[mark_id, label, score, **distance_m** с RealSense],
`context_marks`=[mark_id, label, score, **distance_m**, side, relevance], notes) + **1-е
изображение** (камера с номерами марок) + **2-е изображение** (top-down SLAM-карта `/map` с позой
робота) + описание карты. Карта рендерится из `/map` (RTAB-Map), edge-локально.

`visible_marks` — это кандидаты финальной цели, к ним можно применять `DRIVE_TO_VISIBLE`.
`context_marks` — это объекты-подсказки для поиска: например `desk(left, office_context)`,
когда ищем `office chair`. Это **не цели для подъезда**. Они нужны только как ориентиры,
чтобы выбрать более перспективный свободный коридор/область карты: например “в правом
коридоре много офисной мебели, исследую этот проход”. При отсутствии строгой цели приоритет —
исследовать белые связные free-space коридоры на SLAM-карте, а не ехать носом к тумбе,
столу или шкафу. Если VLM все же ошибочно выберет `DRIVE_TO_VISIBLE mark_id` из
`context_marks`, orchestrator не валит план в fallback и не подъезжает к context-объекту:
он превращает это в `semantic_explore` — поворот/переоценку сцены для выбора коридора.
Если VLM уже выбрала `DRIVE_FORWARD` по свободному коридору, context-объекты больше не
подменяют это действие поворотом к мебели; исключение — близкое препятствие по центру,
когда прямой проезд небезопасен.

При `initial_scan_when_target_absent=true` начальный обзор работает как сбор описаний
коридоров: стартовый кадр записывается как `CORRIDOR_SCAN[forward]`, после правого
поворота — `CORRIDOR_SCAN[right]`, после левого обзора — `CORRIDOR_SCAN[left]`.
В каждую запись попадают найденные context-объекты этого направления. VLM получает эти
записи как `corridor_scan` и должна выбирать коридор так: сначала наличие свободного
белого/серого прохода на SLAM-карте, затем семантический вес объектов в этом проходе.
Если несколько коридоров одинаково проходимы, предпочтение получает тот, где больше
релевантных подсказок для цели. После выбора коридора нормальное действие —
короткие `DRIVE_FORWARD` шаги по проходу до появления строгой цели, отказа Nav2 или
близкого препятствия.

Если финальная цель видна, но `distance_m=null` / глубина неизвестна (например объект дальше
надежной зоны RealSense), VLM не должна завершать миссию и не должна вызывать
`DRIVE_TO_VISIBLE`: нужно выполнить `target_probe` — повернуться к стороне цели или коротко
проехать вперед, если цель по центру. Если VLM ошибочно выберет `DRIVE_TO_VISIBLE` для такой
цели, orchestrator автоматически превратит это в безопасный `target_probe`.

Даже когда глубина известна, `ApproachDetection` сначала проверяет финальную standoff-точку
по `/map`: если она уже лежит в известной свободной клетке SLAM-карты, Nav2 получает прямой
маршрут к цели. Если карты еще нет, точка вне карты, unknown/occupied или вокруг
standoff-точки нет свободного радиуса `approach_direct_clearance_m`, подход режется на
короткий шаг `approach_max_goal_step_m`. После первого уверенного `DRIVE_TO_VISIBLE`
orchestrator запоминает map-точку цели; если после bounded-step объект выпал из кадра,
он не возвращается к generic context-search, а продолжает подход к сохраненной точке
через `ApproachDetection` locked-target режим. Это защищает онлайн-SLAM от целей за
пределами текущей известной области карты и от прямой парковки слишком близко к
препятствию, но не забывает уже подтвержденный объект.

#### Ключевые параметры orchestrator
| Параметр | Деф. | Зачем |
|---|---|---|
| `async_replan` | `false` | `false` = дискретные шаги (едь→стоп→свежее наблюдение→думай). `true` включает overlap replan, но сложнее анализировать логи |
| `resolve_target_query` | `true` | Перед миссией VLM нормализует сырой instruction из `/seek_object`/`/vlm_mission`: прямые имена оставляет как есть, а загадки/описания переводит в короткую цель для детектора |
| `turn_settle_s` | `2.0` | Пауза после успешного `TURN` перед следующей детекцией/VLM-наблюдением; нужна, чтобы кадр RealSense не был смазан в хвосте поворота. При `async_replan=true` TURN всё равно требует свежего post-settle кадра |
| `min_effective_turn_rad` | `0.60` | Минимальный исполнимый `TURN`; маленькие повороты VLM нормализуются, потому что Nav2 может засчитать их внутри yaw tolerance без реального движения |
| `initial_scan_when_target_absent` | `true` | Если строгая цель не видна в начальном кадре, выполнить обзорный sweep перед VLM-поиском: сначала вправо ~90°, затем влево ~180° из правого положения |
| `initial_scan_left_rad` / `initial_scan_right_rad` | `3.14` / `1.57` | Углы начального обзора: правый кадр после `-1.57rad`, затем левый кадр после двух signed-поворотов `+1.57rad` + `+1.57rad`; так Nav2 не выбирает неоднозначное направление для 180° |
| `detect_conf` | `0.0` | Legacy override: если >0, одним числом переопределяет оба порога ниже |
| `target_detect_conf` | `0.60` | Порог конкретной цели для DINO+MobileSAM (`chair`, `drawer cabinet`); строгая цель должна быть увереннее context-подсказок |
| `detect_all_conf` | `0.08` | Legacy-порог для пустого broad `DETECT_ALL`; в DINO hardware/VLM режиме обычно не используется |
| `context_detect_conf` | `0.30` | Порог DINO office-context, когда цель не найдена, но нужно найти офисные объекты-подсказки |
| `context_target_promote_conf` | `0.35` | Legacy no-op: оставлен только чтобы старые команды запуска не падали; context_marks больше не становятся target-candidates |
| `auto_context_when_target_absent` | `true` | Если цель не найдена, автоматически собрать `context_marks` для semantic-explore |
| `semantic_turn_max_streak` | `1` | Сколько смысловых поворотов подряд разрешено до принудительного продвижения вперёд, если путь не блокирован; `1` не даёт роботу “зависать” на осмотре одного пятачка |
| `finish_on_approach_success` | `true` | Завершить VLM-миссию после успешного финального `DRIVE_TO_VISIBLE`; дальний bounded-step не считается финишем |
| `locked_target_approach_max_attempts` | `8` | После уверенной target-детекции продолжать ехать к сохраненной map-точке цели, даже если объект пропал из кадра после bounded-step; лимит защищает от бесконечной попытки |
| `approach_max_goal_step_m` | `1.2` | Лимит шага для дальнего `ApproachDetection`, когда финальная точка вне известной свободной карты |
| `approach_direct_if_goal_in_known_free_map` | `true` | Если финальная standoff-точка находится в known-free области `/map`, ехать к ней напрямую, без bounded-step |
| `approach_direct_clearance_m` | `0.35` | Минимальный known-free радиус вокруг direct standoff-точки; если рядом unknown/occupied, используется bounded-step |
| `approach_allow_unknown_bounded_goal` | `true` | Разрешить короткий bounded-step через `unknown`/`clearance_unknown`, когда цель видна, но онлайн-SLAM ещё не достроил карту; occupied/outside-map всё ещё запрещены |
| `approach_unknown_bounded_max_step_m` | `0.6` | Максимальная длина такого осторожного unknown-probe; если цель уже locked, следующий шаг продолжит тот же target-lock, а не generic поиск |
| `send_map` | `true` | `false` = не слать карту 2-м изображением (легче запрос; если эндпоинт таймаутит) |
| `map_max_px` | `384` | Макс. сторона рендера карты |
| `vlm_timeout_s` | `30.0` | На медленном эндпоинте/с картой можно поднять до `60`, иначе circuit-breaker → DEGRADED |
| `replan_every_n` | `3` | Реальная VLM всё равно отдаёт 1 действие за вызов |

> **Цель можно подавать как прямой лейбл или как загадку.** При `resolve_target_query:=true`
> оркестратор сначала делает короткий VLM-запрос нормализации: `office chair` остаётся
> `office chair`, `black office chair` может пойти в детектор с цветом, а описание вроде
> `the thing people sit on while working at a desk` нормализуется в `office chair`.
> Для target-детекции держите `target_detect_conf:=0.60`; для DINO office-context используйте
> `context_detect_conf:=0.30`. Пустой broad `DETECT_ALL`/YOLOE в текущем hardware/VLM режиме
> отключен как шумный для низкой офисной камеры.

> Замечание по RAM: gz + RTAB-Map + Nav2 + тяжёлый CV backend вместе требуют >4 ГБ. На хосте
> с ≤4 ГБ запускайте либо детектор отдельно (мир из 3b), ЛИБО nav-стек, но не всё сразу.

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
- **Операторская команда:** `ros2 run fleet_comms send_mission "<цель>" <true|false>`.
- **Общее для FLAT и VLM:** hardware/Nav2/executive, edge-camera/SLAM и detector должны быть подняты.
- **FLAT:** `false` — внутри отправляется action `/seek_object` с `allow_vlm: false`, миссией владеет executive.
- **Отличие FLAT по терминалам:** не нужен только `planner_orchestrator` и `vlm.env`.
- **VLM:** `true` — внутри отправляется тот же action `/seek_object`, но `allow_vlm: true`; executive делает handoff:
  публикует instruction во внутренний `/vlm_mission` и ждёт `mission_end` из `/vlm/activity`.
- **Отличие VLM по терминалам:** дополнительно нужен `planner_orchestrator` с загруженным `vlm.env`.
- **`/vlm_mission`:** внутренний/debug-топик для orchestrator; для обычных HIL-прогонов
  используйте `/seek_object`, чтобы терминал старта был одинаковым для обоих режимов.
- **Смена цели посреди миссии:** отправьте новый goal `/seek_object` —
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
  step N [semantic_explore]: DRIVE_FORWARD +0.50m -- <rationale>
  ```
  Роль в квадратных скобках показывает смысл действия: `target_approach` — едем к найденной цели,
  `semantic_explore` — цель не видна, но направление выбрано по объектам-подсказкам в кадре,
  `blind_scan` — полезных подсказок нет, робот просто сканирует.
  `conf` низкая (≈0.45) → детекция слабая/краевая (см. `target_detect_conf`). `map=no` → карта не пришла
  (`/map` нет или `send_map:=false`). `DEGRADED: ran in FLAT fallback` в конце → VLM-эндпоинт упал
  (circuit-breaker), миссия доехала на mock — подними `vlm_timeout_s` / поставь `send_map:=false`.

### 6c. Persistent VLM mission logger
`edge_bringup.launch.py` и `vlm_sim_bringup.launch.py` по умолчанию запускают
`fleet_comms/vlm_mission_logger`. Он пассивно слушает `/vlm/activity` и пишет
структурированные логи без изображений и без VLM-секретов:

```text
~/ros2_ws/experiment_logs/vlm_missions/vlm_activity_<run_id>.jsonl
~/ros2_ws/experiment_logs/vlm_missions/vlm_steps_<run_id>.csv
```

JSONL хранит все raw-события `/vlm/activity`, а CSV — компактные строки по миссиям,
шагам, результатам, задержкам, лучшим target/context-детекциям и причинам действий.
Чтобы имя файла было осмысленным, задавайте `vlm_log_run_id` при запуске edge/sim:

```bash
ros2 launch ar_project edge_bringup.launch.py vlm_log_run_id:=office_chair_001
```

Если нужен отдельный ручной logger:

```bash
ros2 run fleet_comms vlm_mission_logger --ros-args \
  -p output_dir:=~/ros2_ws/experiment_logs/vlm_missions \
  -p run_id:=office_chair_001
```

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
