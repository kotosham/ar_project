# Phase 2 — ZERO-VLM FLAT baseline: план реализации

Гейт фазы 2. Получен из планировочного workflow (architect + adversarial
critic). Критик вернул вердикт **needs_revision**; перечисленные ниже
обязательные правки (must-fix) учтены в этом плане, поэтому источником истины
является именно этот документ, а не исходный план.

## Архитектура

Вся executive-логика живёт в одном rclpy-узле, **`search_coordinator`** (ar_project),
на `MultiThreadedExecutor` с раздельными `ReentrantCallbackGroup` (тик FSM,
каждый skill-сервер, подписки), чтобы внутрипроцессный loopback (FSM является
клиентом своих собственных skill-серверов) не приводил к взаимоблокировке.
**Ни один колбэк не должен блокироваться** — циклы движения и «ожидание нулевой
скорости» опрашиваются через таймер/асинхронно. **`map_odom_relay`** — отдельный
узел в том же пакете. Executive **никогда не публикует `cmd_vel`** — всё движение
проходит по цепочке Nav2 → `/cmd_vel_nav` → velocity_smoother → `/cmd_vel` →
watchdog → twist_mux → collision_monitor (цепочка безопасности не тронута).

- ENTRY: action-сервер `SeekObject` (object_tracking_msgs) — единственная точка
  входа в миссию; владеет mission_state, mission_epoch, FSM.
- SKILL-серверы (все в search_coordinator, ar_project_msgs): `ExploreFrontier`,
  `GoToPose`, `ApproachDetection`, `GetObservation`, `Stop`. FSM управляет ими
  через loopback action-клиентов (поэтому каждый тестируется через
  `ros2 action send_goal`).
- `GoToPose`/`ExploreFrontier`/`ApproachDetection` управляют движением через Nav2
  `navigate_to_pose`. Переиспользуют QoS `fleet_comms` + Heartbeat (фаза 1.3).

## Состояния FSM (общая константа `executive_fsm.STATE`)

`IDLE, SEARCH, DETECT, APPROACH, STOP, DEGRADED, DONE, FAILED` (DRIVE свёрнут в
skills; REPLAN зарезервирован для фазы 4 и в FLAT никогда не достигается).
Публикуется дословно в `SeekObject` feedback.state.

**Инварианты:** (1) committed-subgoal — ровно одна зафиксированная подцель
`{skill,args,step_id-UUID,epoch}` пока активна цель SeekObject, принимается только
в commit-точке; (2) default-productive-action — `_select_subgoal()` никогда не
возвращает None пока миссия активна; по умолчанию = EXPLORE_FRONTIER, с откатом
к STOP(HOLD) + завершению только когда frontier-ы исчерпаны. Никогда не крутиться
вхолостую, никогда не выдавать реактивный cmd_vel.

Проход: IDLE →(goal)→ SEARCH →(свежий target pixel)→ DETECT →(вычислима 3D-цель)→
APPROACH →(достигнута и свежая)→ DONE. APPROACH STALE_DETECTION/LOST_TARGET →
SEARCH (никогда ложно не «достигнуто»). ExploreFrontier NO_FRONTIER → FAILED.
Новая инструкция → STOP → сброс (epoch++).

## Контракты skill-серверов

- **ExploreFrontier** — разрешает `frontier_id` (-1 ⇒ лучший после гистерезиса;
  ≥0 ⇒ стабильный id) → PoseStamped → Nav2. Feedback: distance_remaining,
  selected_frontier_id, frontier_score. NO_FRONTIER если список пуст. Соблюдает
  max_travel_m.
- **GoToPose** — пробрасывает target_pose + допуски в Nav2; маппит результат.
- **ApproachDetection** (FMEA) — подписан на `/target_pixel`; контролирует возраст
  пикселя относительно `max_pixel_age_s` (1.5). SUCCEEDED **только** когда Nav2
  достиг поставленной мной позы **И** последний пиксель был свежим. Никогда не
  «достигнуто» при `detection_fresh==false`. STALE_DETECTION (пиксели приходят,
  но все устаревшие) / LOST_TARGET (пикселей нет) прерывают вместо движения или
  ложного успеха. Без защёлки goal_locked.
- **GetObservation** — функциональная заглушка фазы 2: захватывает один
  CompressedImage если есть реальный источник (иначе оставляет `view` пустым —
  см. must-fix #3), `candidates` пуст (детектор — это фаза 3).
- **Stop** — идемпотентен. SOFT_STOP: отменяет Nav2; остановка обеспечивается
  обнулением velocity_smoother по input-timeout + watchdog (НЕ рампа торможения —
  см. must-fix #4). HOLD: отмена + защёлка. QUICK_STOP_REQUEST: дополнительно
  срабатывает внешний аппаратный quick-stop-триггер (закрывает ROADMAP 0.7),
  исполняется независимо от epoch. `zero_velocity_confirmed` как только скорость
  по `/odometry/filtered` < eps. Строжайшая идемпотентность: повторная отправка
  того же request_id = no-op (кэшированный терминальный результат).

## Идемпотентность по mission-epoch + UUID (FMEA 2.5)

SeekObject — это authority по epoch. Новая инструкция → ABORT-AND-RESET: отменить
все skill- и Nav2-цели в полёте, `mission_epoch++` (uint32, безопасно к
переполнению), очистить committed-подцель + состояние commit frontier-а + таблицы
дедупликации, переввести в SEARCH; старый handle SeekObject финализируется как
PREEMPTED. Каждая отправленная skill-цель помечается текущим epoch + свежим UUID
step_id. Серверы выполняют epoch-gate на приёмке (отклоняют несовпадающие =
зомби). Результаты фильтруются путём пометки каждого ожидающего handle его
dispatch-epoch. Словарь дедупликации `{request_id: handle/result}` на сервер;
повторный request_id в том же epoch = no-op.

## Frontier-экстрактор (FMEA 2.3) — ИСПРАВЛЕННЫЙ ИСТОЧНИК

**Решение об источнике (must-fix #1):** НЕ `/local_costmap/costmap_raw`
(скользящий 3×3, без `track_unknown_space` → нет UNKNOWN-ячеек → ноль frontier-ов).
Использовать **SLAM occupancy grid** (RTAB-Map `/map`, `nav_msgs/OccupancyGrid`,
фрейм `map`, -1=unknown / 0=free / 100=occupied) — стандартный источник для
explore-lite. **Должно быть эмпирически проверено** в sim до начала сборки:
подтвердить, что `/map` публикуется с unknown-ячейками; если RTAB-Map не выдаёт
grid в `launch_sim`, включить его. Цели ExploreFrontier тогда задаются во фрейме
`map` (Nav2 global_frame=map). Frontier = свободная ячейка с unknown-соседом по
4-связности; кластер (≥ `min_frontier_cells`); оценка =
`w_size*size − w_dist*dist − w_turn*heading_change`; стабильные id через
квантование центроида; top-K в памяти + `/frontiers/markers` для RViz.

**Гистерезис:** не переключаться с зафиксированного frontier-а, пока конкурент не
превзойдёт его на `switch_margin` (15%) И он не был зафиксирован ≥ `min_dwell_s`
(4 с); обходится только когда зафиксированный frontier исчезает (прогресс, а не
осцилляция). Параметры объявлены на узле. Нужен мир с двумя почти равными
frontier-ами (must-fix #10).

## Detect-путь FLAT — ИСПРАВЛЕНО (must-fix #2/#3)

`/target_pixel` начинает течь только после того, как метка достигает
`/target_prompt`, а `reliable_prompt_sender` удаляется (2.9). Поэтому: **FSM
публикует `SeekObject.instruction` → `/target_prompt`** (небольшой мост внутри
executive), чтобы существующий трекер отслеживал запрошенный объект.
**Исправление QoS:** подписка-потребитель на `/target_pixel` НЕ должна запрашивать
deadline, который издатель с BEST_EFFORT/без deadline не может предложить (иначе
получится Request-vs-Offered несовместимость → тихие нулевые сэмплы). Либо добавить
вариант `detection_stream_nodeadline()`, либо добавить соответствующий offered
deadline на издателе трекера; добавить unit-тест `is_compatible` именно для этой
пары.

## map_odom_relay (2.6)

`search_coordinator/map_odom_relay.py`. Подписаться на `/map_odom_correction`
(QoS `correction_lowrate`); транслировать map→odom в /tf на ~10 Гц (< 0.2 с
transform_tolerance), удерживая последнее корректное значение (identity до первой
коррекции). Гейты: stale-by-seq, stale-by-stamp (`max_correction_age_s` 1.0),
covariance/fitness, скачок (`max_jump_m`/`max_jump_rad`; принимать только если
`relocalized==true`). **must-fix #7:** параметризовать RTAB-Map `publish_tf_map`
(жёстко прошитый `'true'` в `rtabmap_rgbd_launch.py:278`) в LaunchConfiguration,
выставлять `false` когда работает relay (без дублирующего broadcaster-а).

## Облегчение Nav2 (2.7)

`controller_frequency` 15→10; `expected_planner_frequency` 20→1; убрать
`smoother_server`+`waypoint_follower` (params + lifecycle_nodes + Nodes); урезать
behavior_plugins до spin/backup/wait; удалить мёртвый блок local static_layer.
Оставить NavFn(use_astar:false) + DWB, local costmap в odom, источник препятствий
`/scan`. НЕ прошивать жёстко use_sim_time (RewrittenYaml всё равно переписывает).
**must-fix #12:** перед удалением `map_server` подтвердить, что питает
static_layer `global_costmap` (RTAB-Map `/map`?), чтобы глобальный costmap
по-прежнему автозапускался.

## Удаления (2.9)

`reliable_prompt_sender.py` + его launch + запись install в CMakeLists; «суп»
target_pixel_to_goal (goal_locked, lock_goal_on_publish, final_approach_freeze,
nav_status_callback, prompt_ack, топик goal_locked), оставив только геометрию
(перенесена в `approach_geometry.py` в 2.8); реактивный cmd_vel в tracker_node +
конечный автомат target_found/reached. **must-fix #6:** также обработать осиротевших
потребителей (`home_pose_manager.py`, `experiment_metrics_logger.py` подписаны на
`/goal_pose`, `/target_prompt`) и launch-файлы, ссылающиеся на выпотрошенные
скрипты.

## Исправленный порядок задач

1. **T2.8** геометрия → `approach_geometry.py` (чистая, pytest). *нет зависимостей*
2. **T2.6** map_odom_relay + параметр RTAB-Map publish_tf_map (mock-проверка).
   *нет зависимостей*
3. **T2.7** облегчение Nav2 (+ проверить источник global_costmap).
   *нет зависимостей*
4. **Спайк по источнику frontier-ов** — эмпирически подтвердить, что `/map` (или
   выбранный grid) выдаёт unknown-ячейки в sim; затем **T2.3** frontier-экстрактор
   + мир для осцилляции.
5. **Prompt-мост + исправление QoS `/target_pixel`** (+ тест is_compatible).
6. **T2.4** skill-серверы — включить сюда authority по epoch `mission_state.py` +
   getter (разрешает цикличность T2.4↔T2.5; собирается с реальной epoch-инфраструктурой).
7. **T2.5** полное поведение mission-epoch + фильтрация по epoch в HeartbeatMonitor
   (`set_mission_epoch` + проверка epoch в `_on_msg` — новая работа, must-fix #8).
8. **T2.2** executive FSM + сервер SeekObject.
9. **T2.9** удаления (после переноса геометрии + запуска FSM в живую).
10. **T2.10** сквозной FLAT-сценарий + замер Pi-профиля → зафиксировать baseline.

## Ключевые риски (от критика)

- Источник frontier-ов должен быть эмпирически доказан до T2.3 (иначе SEARCH
  заходит в тупик).
- QoS `/target_pixel` + prompt-мост должны быть готовы до того, как любой APPROACH
  сможет завершиться успешно.
- Loopback action-серверы: раздельные callback-группы, никогда не блокироваться
  в колбэке.
- Pi-профилирование под WSL — это эмуляция через taskset/cpulimit → только
  относительный гейт; перемерить на железе в фазе 6.3.
