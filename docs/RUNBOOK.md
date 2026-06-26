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
  python3 -m venv --system-site-packages ~/ot_venv
  source ~/ot_venv/bin/activate
  pip install torch torchvision --index-url https://download.pytorch.org/whl/cu124
  pip install -r object_tracking/requirements.txt
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
~/ot_venv/bin/python $(ros2 pkg prefix object_tracking)/lib/object_tracking/detect_target_server \
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
~/ot_venv/bin/python $(ros2 pkg prefix planner_orchestrator)/lib/planner_orchestrator/orchestrator_node \
  --ros-args -p use_sim_time:=true -p use_mock:=false -p replan_every_n:=3 -p max_steps:=40 \
  -p async_replan:=false -p detect_conf:=0.5
# expect: "planner_orchestrator up ... client=OpenAICompatibleClient creds=env"
ros2 topic pub --once /vlm_mission std_msgs/msg/String "{data: bus}"   # ЧИСТЫЙ лейбл (см. ниже)
```
Orchestrator забирает реальные Set-of-Mark-кандидаты из `detect_target_server`, шлёт VLM наблюдение
(см. ниже) и диспетчеризует выбранное действие в skill-ы executive. Для офлайн-прогона используйте
`-p use_mock:=true` (ключ API не нужен — это же FLAT-фоллбэк при деградации).

> Запускать орхестратор через `~/ot_venv/bin/python <...>/orchestrator_node` (а не `ros2 run`): ему
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
| `detect_conf` | `0.0` | Порог уверенности (0.0 = деф. детектора 0.25). `0.5` + чистый лейбл — игнор слабых/краевых детекций |
| `send_map` | `true` | `false` = не слать карту 2-м изображением (легче запрос; если эндпоинт таймаутит) |
| `map_max_px` | `384` | Макс. сторона рендера карты |
| `vlm_timeout_s` | `8.0` | На медленном эндпоинте/с картой подними до `30–60`, иначе circuit-breaker → DEGRADED |
| `replan_every_n` | `3` | Реальная VLM всё равно отдаёт 1 действие за вызов |

> **Цель — ЧИСТЫЙ лейбл объекта** (`bus`, НЕ `find a bus`/`ride to bus`): нормализации запроса нет,
> YOLOE матчит строку как один класс. `bus` → conf ~0.66; `ride to bus` → ~0.45 (слабее, ложные
> «доехал» у края кадра). При NL-запросе поднимай `detect_conf`.

> Замечание по RAM: gz + RTAB-Map + Nav2 + YOLOE вместе требуют >4 ГБ. На хосте с ≤4 ГБ запускайте
> либо детектор отдельно (мир из 3b), ЛИБО nav-стек, но не всё сразу.

> Замечание по симуляции (только gz): перед запуском sim-стека ставь `export GZ_IP=127.0.0.1` — иначе
> `gz sim` периодически падает с SIGSEGV в потоке gz-transport discovery (известный баг на WSL). На
> железе Gazebo нет — там GZ_IP не нужен. Лончеры `watch_vlm.sh`/`watch_flat.sh` уже это делают.

---

## 4. ЖЕЛЕЗО (СНАЧАЛА следуйте §B safety в HIL_BRINGUP_CHECKLIST.md — колёса над землёй)

### 4a. Edge-машина (GPU)
```bash
sudo systemctl start rmw-zenoh-router.service          # deploy/transport (transport)
sudo systemctl start chrony   # chrony-edge.conf master                (deploy/time_sync)
ros2 launch ar_project rtabmap_rgbd_launch.py use_sim_time:=false       # SLAM -> /map + MapOdomCorrection
~/ot_venv/bin/python $(ros2 pkg prefix object_tracking)/lib/object_tracking/detect_target_server \
  --ros-args -p use_sim_time:=false -p use_compressed_input:=true       # detector (YOLOE, venv)
# VLM-оркестратор (только для VLM-режима): creds из env, НИКОГДА не в параметрах/логах
set -a; source object_tracking/planner_orchestrator/vlm.env; set +a
~/ot_venv/bin/python $(ros2 pkg prefix planner_orchestrator)/lib/planner_orchestrator/orchestrator_node \
  --ros-args -p use_sim_time:=false -p async_replan:=false -p detect_conf:=0.5 \
  -p vlm_timeout_s:=30.0          # карта /map берётся edge-локально (RTAB-Map) -> в VLM 2-м изображением
```
Параметры VLM — см. таблицу в §3c. `detect_target_server` и `orchestrator_node` оба в `~/ot_venv`
(torch для детектора; cv2/numpy для рендера карты у оркестратора).

### 4b. Pi (робот)
Все ноды на Pi — с `use_sim_time:=false` (на железе НЕТ `/clock`; см. чек-лист A).
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ar_project hardware_bringup.launch.py        # ros2_control + CAN/EPOS4 + twist_mux
                                                         #   + collision_monitor + cmd_vel watchdog + /scan
ros2 launch ar_project realsense_rgbd_pi.launch.py       # RealSense (RGB+depth+IMU) + EKF
ros2 launch ar_project navigation_launch.py use_sim_time:=false odom_topic:=/odometry/filtered
ros2 run search_coordinator map_odom_relay --ros-args -p use_sim_time:=false      # применяет MapOdomCorrection с edge
ros2 run search_coordinator coordinator_node --ros-args -p use_sim_time:=false    # executive: SeekObject FSM + 5 skill-серверов
ros2 run search_coordinator frontier_extractor --ros-args -p use_sim_time:=false  # только для FLAT (VLM фронтиры не использует)
```
`coordinator_node` поднимает skill-серверы (`go_to_pose`, `approach_detection`, `explore_frontier`,
`get_observation`, `stop`), которыми и управляет orchestrator с edge в VLM-режиме. `frontier_extractor`
нужен только FLAT-режиму.

### 4c. Запуск миссии на железе (с любой машины графа ROS)
```bash
# FLAT (исполнителем владеет Pi):
ros2 action send_goal /seek_object object_tracking_msgs/action/SeekObject \
  "{instruction: 'bus', request_id: 'm1', mission_epoch: 0, allow_vlm: false}" --feedback

# VLM (исполнителем владеет orchestrator на edge -> гоняет skill-ы Pi):
#   нужен §4a orchestrator. Цель -- ЧИСТЫЙ лейбл.
ros2 topic pub --once /vlm_mission std_msgs/msg/String "{data: 'bus'}"
```
> **СНАЧАЛА пройдите раздел B (safety) в `HIL_BRINGUP_CHECKLIST.md` — колёса над землёй.** Не запускайте
> автономную миссию, пока B1–B7 не зелёные. Полная последовательность ввода в эксплуатацию (time-sync,
> transport, восприятие, FLAT→VLM, деградация) — там же, разделы C–I.

---

## 5. Запуск миссий
- **FLAT:** action `/seek_object`, `allow_vlm: false` — миссией владеет executive.
- **VLM:** опубликуйте цель в `/vlm_mission` (std_msgs/String) — ею владеет orchestrator и
  управляет skill-ами executive. (`allow_vlm: true` в goal `/seek_object` — это флаг на стороне
  executive для интегрированного режима.)
- **Смена цели посреди миссии:** отправьте новый goal `/seek_object` (или новый `/vlm_mission`) —
  epoch инкрементируется, старая миссия получает PREEMPTED, незавершённые skill-goal-ы отклоняются как zombie.

## 6. Мониторинг
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
  `conf` низкая (≈0.45) → детекция слабая/краевая (см. `detect_conf`). `map=no` → карта не пришла
  (`/map` нет или `send_map:=false`). `DEGRADED: ran in FLAT fallback` в конце → VLM-эндпоинт упал
  (circuit-breaker), миссия доехала на mock — подними `vlm_timeout_s` / поставь `send_map:=false`.

## 7. Деградация (FMEA 5.1) — ожидаемое поведение
Если VLM потерян (таймаут/недоступность → circuit-breaker открывается), orchestrator **защёлкивается
на FLAT MockPlanner, и миссия ПРОДОЛЖАЕТСЯ (DEGRADED)** — она не останавливается. При реальной
потере edge/Wi-Fi executive на Pi сохраняет FLAT-автономность. ApproachDetection никогда не объявляет
ложный `reached` по устаревшему пикселю (возвращает STALE_DETECTION / LOST_TARGET).

## 8. Устранение неполадок
- **Spawn зацикливается на "Requesting list of world names" / нет `/odom`:** завис старый сервер `gz sim` —
  `pkill -9 -f 'gz sim'; pkill -9 -f ruby` и перезапустите.
- **Робот не двигается в ограниченном мире:** ещё нет frontier-ов — выполните засев вращением (3a, T2),
  чтобы SLAM получил unknown-ячейки. Убедитесь, что `/frontiers` не пуст.
- **`explore_frontier: nav drive terminal=no_server`:** Nav2 ещё не активен — он больше не
  заносит в blacklist; он ждёт (`explore_nav_ready_timeout_s`) и повторяет попытку. Дайте Nav2 время активироваться.
- **Детектор: `ModuleNotFoundError: torch`:** вы запустили его системным python — используйте
  `~/ot_venv/bin/python <installed detect_target_server path>`.
- **DRIVE_TO_VISIBLE не едет:** кандидату нужна метрическая глубина — убедитесь, что топик глубины
  публикуется (`/camera/camera/aligned_depth_to_color/image_raw`) и задан `use_depth:=true`.
- **Неожиданно `client=MockVlmClient` у VLM:** не разрешён base_url — выполните `source vlm.env` (или передайте
  `-p vlm_base_url:=`) и не передавайте `use_mock:=true`.
- **Миссия уходит в `DEGRADED` (FLAT fallback):** реальный VLM-эндпоинт упал/таймаутил 3 раза подряд
  (circuit-breaker). Подними `-p vlm_timeout_s:=30..60` и/или `-p send_map:=false` (два изображения тяжелее).
- **VLM объявляет `DONE`/`reached`, а робот не у цели:** слабая/краевая детекция. Признак — `conf≈0.45`
  в `observe@`. Используй ЧИСТЫЙ лейбл (`bus`, не `ride to bus`) и `-p detect_conf:=0.5`.
- **VLM «видит» цель, хотя не смотрит на неё:** проверь `conf`/`distance_m` в `observe@`. FOV камеры
  всего ~62° (±31°); если цель реально вне кадра — детектор отдаёт `0 detection(s)` (проверено). Если
  детекция есть — край баннера попал в кадр. Подними `detect_conf`, чтобы реагировать только на уверенные.
- **`gz sim` падает SIGSEGV на старте (ТОЛЬКО симуляция):** баг gz-transport discovery на WSL —
  `export GZ_IP=127.0.0.1` перед запуском + `pkill -9 -f 'gz sim'; pkill -9 -f ruby`. На железе Gazebo нет.
- **Карта не приходит в VLM (`map=no` в логе):** нет паблишера `/map` (RTAB-Map не поднят на edge) или
  `send_map:=false`, или у оркестратора нет cv2/numpy (запусти его из `~/ot_venv`).
