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
#   default world = oscillation.world; override: world:=$(ros2 pkg prefix ar_project)/share/ar_project/worlds/flat_detect.world
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
```bash
# detector running (3b) + executive up. In the orchestrator shell:
set -a; source object_tracking/planner_orchestrator/vlm.env; set +a   # loads VLM_* (never printed)
ros2 run planner_orchestrator orchestrator_node --ros-args \
  -p use_sim_time:=true -p use_mock:=false -p replan_every_n:=3 -p max_steps:=40
# expect: "planner_orchestrator up ... client=OpenAICompatibleClient creds=env"
ros2 topic pub --once /vlm_mission std_msgs/msg/String "{data: bus}"   # start the VLM mission
```
Orchestrator забирает реальные Set-of-Mark-кандидаты из `detect_target_server`, VLM выбирает
`DRIVE_TO_VISIBLE(mark_id)` / `GO_TO_FRONTIER` и диспетчеризует их в skill-ы executive. Перепланирование
перекрывает выполнение (4.6). Для офлайн-прогона используйте `-p use_mock:=true` (ключ API не нужен).

> Замечание по RAM: gz + RTAB-Map + Nav2 + YOLOE вместе требуют >4 ГБ. На хосте с ≤4 ГБ запускайте
> либо детектор отдельно (мир из 3b), ЛИБО nav-стек, но не всё сразу.

---

## 4. ЖЕЛЕЗО (СНАЧАЛА следуйте §B safety в HIL_BRINGUP_CHECKLIST.md — колёса над землёй)

### 4a. Edge-машина
```bash
sudo systemctl start rmw-zenoh-router.service          # deploy/transport (transport)
sudo systemctl start chrony   # chrony-edge.conf master                (deploy/time_sync)
ros2 launch ar_project rtabmap_rgbd_launch.py use_sim_time:=false       # SLAM -> MapOdomCorrection
~/ot_venv/bin/python $(ros2 pkg prefix object_tracking)/lib/object_tracking/detect_target_server \
  --ros-args -p use_sim_time:=false -p use_compressed_input:=true       # detector
set -a; source vlm.env; set +a
ros2 run planner_orchestrator orchestrator_node --ros-args -p use_sim_time:=false  # VLM (optional)
```

### 4b. Pi (робот)
```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch ar_project hardware_bringup.launch.py        # ros2_control + CAN/EPOS4 + twist_mux
                                                         #   + collision_monitor + cmd_vel watchdog
ros2 launch ar_project realsense_rgbd_pi.launch.py       # RealSense + local /scan + EKF
ros2 launch ar_project navigation_launch.py use_sim_time:=false odom_topic:=/odometry/filtered
ros2 run search_coordinator map_odom_relay --ros-args -p use_sim_time:=false
ros2 run search_coordinator frontier_extractor --ros-args -p use_sim_time:=false
ros2 run search_coordinator coordinator_node --ros-args -p use_sim_time:=false
```
Затем запустите миссию точно так же, как в 3a (FLAT) / 3c (VLM), но с `use_sim_time:=false`.

---

## 5. Запуск миссий
- **FLAT:** action `/seek_object`, `allow_vlm: false` — миссией владеет executive.
- **VLM:** опубликуйте цель в `/vlm_mission` (std_msgs/String) — ею владеет orchestrator и
  управляет skill-ами executive. (`allow_vlm: true` в goal `/seek_object` — это флаг на стороне
  executive для интегрированного режима.)
- **Смена цели посреди миссии:** отправьте новый goal `/seek_object` (или новый `/vlm_mission`) —
  epoch инкрементируется, старая миссия получает PREEMPTED, незавершённые skill-goal-ы отклоняются как zombie.

## 6. Мониторинг
- `ros2 topic echo /planner/notes` — компактные заметки VLM + token_estimate.
- `ros2 topic echo /frontiers` — список frontier-ов + закоммиченный id.
- `ros2 action list` / `ros2 node list` — подтвердить, что серверы подняты.
- Heartbeat-ы: executive логирует `/heartbeat deadline missed`, когда продюсер (edge) молчит.
- Логи: каждая нода печатает свою фазу; orchestrator логирует `step N: <ACTION> (rationale)`.

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
