# Структура репозиториев, интерфейсы и оценки

Настоящий раздел фиксирует целевую раскладку кода на ветках `robust` обоих репозиториев (`ar_project` — сторона Raspberry Pi / executive; `object_tracking` — сторона edge / восприятие), полную инвентаризацию интерфейсов ROS 2 (`.action` / `.msg` / `.srv`), таблицу сложности и трудозатрат, а также пошаговую дорожную карту (ROADMAP) в порядке «сначала чиним, потом строим» (FIX-FIRST). Архитектура считается утверждённой по итогам двух проходов проектирования и FMEA; раздел её реализует, а не пересматривает. Целевая платформа — ROS 2 Jazzy, тестирование в Gazebo внутри образа WSL2 Ubuntu.

## 1. Раскладка пакетов и репозиториев на ветках `robust`

### 1.1 Принцип разделения

- **`ar_project` (Pi-сторона, executive-on-Pi).** Здесь живут все узлы реального времени и реактивный контур: EKF, облегчённый Nav2, `map_odom_relay`, Search Coordinator (executive FSM/BT), idempotent skill-серверы, `target_pixel_to_goal` (переиспользуется без изменений), `depthimage_to_laserscan`, драйвер RealSense, `ros2_control` + `EmbodiedRobotSystem` (EPOS4 CiA-402 поверх SocketCAN), слой безопасности. VLM на этой стороне нет никогда.
- **`object_tracking` (edge/PC-сторона).** Здесь живут SLAM (RTAB-Map), открытословарный детектор (YOLOE по умолчанию + GroundingDINO+MobileSAM как fallback, Set-of-Mark), Planner Orchestrator (лёгкий async HTTP-клиент к **внешнему OpenAI-совместимому VLM API**; саму модель здесь не хостим) и семантическая память / буфер заметок. На edge-GPU крутятся только детектор и SLAM; VLM — за API.
- **Транспорт между хостами** — `rmw_zenoh` (один systemd-роутер на edge), fallback Fast DDS LARGE_DATA + Discovery Server, multicast выключен, буферы сокетов 12 МБ, синхронизация часов `chrony` на всех хостах, QoS deadline/liveliness на кросс-линковых топиках. PointCloud2 / сырые depth-потоки по Wi-Fi не передаются никогда — `/scan` генерируется локально на Pi.

### 1.2 Новые пакеты интерфейсов (interface-only)

Чтобы не плодить циклические зависимости между кодом и сообщениями и чтобы оба хоста могли собрать только описание интерфейса без тяжёлых зависимостей, типы выносятся в два отдельных пакета `rosidl` (build_type `ament_cmake`, без логики):

- **`ar_project_msgs`** (НОВЫЙ, в репозитории `ar_project`) — интерфейсы Pi-стороны и кросс-линка, потребляемые executive: skill-actions (`ExploreFrontier`, `GoToPose`, `ApproachDetection`, `GetObservation`, `Stop`), `MapOdomCorrection.msg`, heartbeat-сообщения, типы планов/заметок, которые читает executive.
- **`object_tracking_msgs`** (НОВЫЙ, в репозитории `object_tracking`) — интерфейсы перцепции и планировщика: `DetectTarget.action`, `SeekObject.action` (высокоуровневая миссия), `PlanStep.msg` / `Notes.msg`, типы кандидатов Set-of-Mark.

Граница проведена так, чтобы Pi-пакеты зависели только от `ar_project_msgs`, а узлы, обслуживающие высокоуровневую миссию и восприятие, — от `object_tracking_msgs`. Если на практике executive начнёт зависеть и от `SeekObject`/`DetectTarget`, эти два типа допустимо продублировать ссылкой через `<depend>` на оба пакета — оба пакета чисто интерфейсные и легковесные.

### 1.3 Новые пакеты с логикой

- **`search_coordinator`** (НОВЫЙ, `ar_project`-репозиторий, отдельный `ament_python`/`ament_cmake` пакет). Координатор-executive: FSM/BehaviorTree, локальное извлечение фронтиров из costmap, владелец состояния миссии, единственный потребитель решений планировщика. Здесь же — skill-action-серверы (`ExploreFrontier` / `GoToPose` / `ApproachDetection` / `GetObservation` / `Stop`), `map_odom_relay`, локальный извлекатель фронтиров с гистерезисом, mission-epoch / UUID-идемпотентность, default-productive-action.
- **`planner_orchestrator`** (НОВЫЙ, `object_tracking`-репозиторий, `ament_python`). Лёгкий async **HTTP-клиент к внешнему OpenAI-совместимому VLM API** (`base_url` + ключ; модель не хостим, GPU не требует): single-in-flight, UUID-идемпотентность, timeout по измеренному p99, circuit-breaker, structured/enum tool-call, streaming; буфер заметок/суммаризации; anytime/async-replan с adoption в commit-point.

Причина выделить координатор и оркестратор в отдельные пакеты, а не складывать скрипты в `ar_project`/`object_tracking`: у них принципиально разный жизненный цикл сборки и зависимостей (координатор — Pi-only, реал-тайм; оркестратор — HTTP-клиент к внешнему VLM API на edge, без локальной модели/GPU), и каждый должен переживать удаление «soup» из старого пакета без регрессий.

### 1.4 Что ПЕРЕИСПОЛЬЗУЕТСЯ / что НОВОЕ / что УДАЛЯЕТСЯ

**REUSED (переиспользуется как есть или с минимальной правкой):**
- `scripts/target_pixel_to_goal.py` — переиспользуется без изменений по математике pixel + aligned-depth → метрическая 3D-цель. Планировщик НИКОГДА не выдаёт навигационные координаты; координаты рождаются только здесь. Из узла удаляются только хуки «soup» (см. ниже про `goal_locked`/`prompt_ack`), а сам бридж становится утилитой, вызываемой skill-сервером `ApproachDetection`.
- Сегментационные бэкенды в `object_tracking/`: `yoloe_image_segmentation.py` (default), `dino_mobilesam_image_segmentation.py` (fallback). `clip_image_segmentation.py` остаётся в репозитории, но из грудинга исключается (CLIPSeg для grounding отброшен).
- Nav2 (`config/nav2_params.yaml`, `launch/navigation_launch.py`, `launch/localization_launch.py`) — переиспользуется, но облегчается (Phase 2).
- EKF (`config/ekf_*.yaml`, `robot_localization`, odom→base_link 20 Гц) — переиспользуется.
- EPOS4/CAN `EmbodiedRobotSystem` (`src/embodied_robot_system.cpp`, `include/ar_project/embodied_robot_system.hpp`, `ar_project_hardware_plugins.xml`, `config/epos4_diffdrive/*`) — переиспользуется, но дорабатывается реальным quick-stop (Phase 0).
- RealSense (`launch/realsense_rgbd_pi.launch.py`), `description/*`, URDF/xacro, `ros2_control` diff_cont, `twist_mux` — переиспользуются.
- `depthimage_to_laserscan` — переиспользуется как локальный источник `/scan` для obstacle-слоя costmap.

**NEW (создаётся заново):**
- Пакеты `ar_project_msgs`, `object_tracking_msgs`, `search_coordinator`, `planner_orchestrator`.
- `map_odom_relay` (узел), локальный frontier-extractor с гистерезисом, skill-action-серверы, `cmd_vel`-watchdog, реальный CiA-402 quick-stop на пути `write()`, RTAB-Map online-localization → low-rate `MapOdomCorrection` (не TF-поток), Set-of-Mark рендеринг кандидатов, notes/summary-буфер.
- Bring-up для Gazebo-on-WSL и конфигурации `rmw_zenoh`/`chrony`.

**DELETED (удаляется):**
- `scripts/reliable_prompt_sender.py` + `launch/reliable_prompt_sender.launch.py` — узел повторной отправки промпта по латч-строке. Заменяется надёжной доставкой миссии через action `SeekObject` (UUID + feedback вместо retry-по-таймеру).
- «Soup» из латч-булей и латч-строк: топики `/target_goal_locked` (`Bool`, TRANSIENT_LOCAL), `/target_prompt_ack` (`String`, TRANSIENT_LOCAL), `/target_prompt` (`String`) и привязанная к ним логика `goal_locked`/`lock_goal_on_publish`/авто-`SUCCEEDED` по `nav_status` внутри `target_pixel_to_goal.py` и `tracker_node.py`. Владение состоянием и признак «достигли» переходят в executive и в результат skill-action `ApproachDetection`. В частности, удаляется авто-success по `nav_status` (status==4) на возможно устаревшем пикселе — см. FMEA-фикс в Phase 3.
- Реактивный `search_cmd_pub` → `/cmd_vel` прямо из `tracker_node.py` (раскрутка на месте при поиске) — удаляется; поиск становится `ExploreFrontier`-скиллом на Pi. Edge никогда не пишет в реактивный путь.

### 1.5 Где живёт документация

- Корневой `README.md` каждого репозитория — overview и точка входа.
- `ar_project/docs/architecture.md` — этот раздел + диаграммы 3T, карта топиков/TF, бюджет задержек (TF 0.2 с, depth-match 0.35 с, pixel-age 1.5 с, chrony-offset).
- `ar_project/docs/safety.md` — слой безопасности и FMEA-фиксы Pi-стороны (quick-stop, watchdog, Collision Monitor, bus-off).
- `object_tracking/docs/perception.md` и `object_tracking/docs/planner_orchestrator.md` — детектор/Set-of-Mark и VLM-оркестрация (контракт tool-call, circuit-breaker, notes-буфер, тайминг replan).
- `docs/roadmap.md` (в `ar_project`, как «головном» репо) — копия ROADMAP из раздела 4 как живой чек-лист.
- Каждый пакет — собственный `README.md` с интерфейсами и параметрами.

## 2. Инвентаризация интерфейсов

Ниже — определения, которые нужно создать. Имена полей даны на английском (идентификаторы), назначение — на русском. Все action’ы preemptable, несут feedback и UUID для идемпотентности; повторная цель с тем же `request_id` в рамках текущего `mission_epoch` не выполняется повторно, а присоединяется к идущему исполнению.

### 2.1 Высокоуровневая миссия

**`object_tracking_msgs/action/SeekObject.action`** — единая точка входа миссии (заменяет `reliable_prompt_sender`).
```
# Goal
string instruction          # естественно-языковая инструкция/цель
string request_id            # UUID идемпотентности
uint32 mission_epoch         # эпоха миссии; смена инструкции инкрементит эпоху
bool allow_vlm               # true=VLM mode, false=форсировать FLAT
---
# Result
uint8 outcome                # 0=SUCCEEDED 1=ABORTED 2=PREEMPTED 3=DEGRADED_SUCCESS
geometry_msgs/PoseStamped final_pose
string summary               # финальная заметка/итог
---
# Feedback
string state                 # текущее состояние FSM (SEARCH/DETECT/DRIVE/APPROACH/REPLAN/...)
string active_subtask        # человекочитаемое имя подзадачи
float32 progress             # 0..1
uint32 mission_epoch
```

### 2.2 Skill-actions (исполняются на Pi, владелец — executive)

Общие правила: каждый принимает `request_id` (UUID) и `mission_epoch`; в feedback идут таймстампы и индикатор «свежести» входных данных; result содержит enum `outcome` и причину.

**`ar_project_msgs/action/ExploreFrontier.action`** — локальное фронтир-исследование.
```
# Goal
string request_id
uint32 mission_epoch
int32 frontier_id            # -1 = выбрать лучший локально; >=0 = заданный из списка
float32 max_travel_m         # ограничение хода
---
# Result
uint8 outcome                # SUCCEEDED/ABORTED/PREEMPTED/NO_FRONTIER
geometry_msgs/PoseStamped reached_pose
---
# Feedback
float32 distance_remaining
int32 selected_frontier_id
float32 frontier_score
```

**`ar_project_msgs/action/GoToPose.action`** — поездка к позе через Nav2.
```
# Goal
string request_id
uint32 mission_epoch
geometry_msgs/PoseStamped target_pose
float32 xy_tolerance
float32 yaw_tolerance
---
# Result
uint8 outcome                # SUCCEEDED/ABORTED/PREEMPTED
geometry_msgs/PoseStamped reached_pose
---
# Feedback
float32 distance_remaining
builtin_interfaces/Time stamp
```

**`ar_project_msgs/action/ApproachDetection.action`** — финальный подъезд к детекции (обёртка над `target_pixel_to_goal`).
```
# Goal
string request_id
uint32 mission_epoch
string target_label          # что подтверждаем при подъезде
float32 approach_offset      # метры; по умолчанию 0.58
float32 max_pixel_age_s      # порог свежести пикселя; по умолчанию 1.5
bool use_locked_target       # продолжить к ранее подтвержденной map-точке
geometry_msgs/PointStamped locked_target_point
---
# Result
uint8 outcome                # SUCCEEDED/ABORTED/PREEMPTED/STALE_DETECTION/LOST_TARGET
geometry_msgs/PoseStamped reached_pose
geometry_msgs/PointStamped target_point
geometry_msgs/PoseStamped final_goal_pose
float32 final_distance_m
---
# Feedback
float32 distance_to_target
float32 detection_age_s      # возраст последнего валидного пикселя
bool detection_fresh         # false => НЕ объявлять reached (FMEA-фикс)
```

**`ar_project_msgs/action/GetObservation.action`** — собрать наблюдение/кадр(ы) для VLM/детектора в безопасной точке.
```
# Goal
string request_id
uint32 mission_epoch
bool with_setofmark          # отрисовать кандидатов Set-of-Mark
---
# Result
uint8 outcome
sensor_msgs/CompressedImage view   # сжатый кадр (через edge, не PointCloud2)
object_tracking_msgs/Candidate[] candidates
geometry_msgs/PoseStamped observed_from
---
# Feedback
string phase                 # ALIGNING/CAPTURING/RENDERING
```

**`ar_project_msgs/action/Stop.action`** — идемпотентная безопасная остановка / приведение в default-productive-action.
```
# Goal
string request_id
uint32 mission_epoch
uint8 mode                   # 0=SOFT_STOP 1=HOLD 2=QUICK_STOP_REQUEST
---
# Result
uint8 outcome
---
# Feedback
bool zero_velocity_confirmed
```

### 2.3 Восприятие

**`object_tracking_msgs/action/DetectTarget.action`** — открытословарная детекция по запросу (replace для непрерывного «трекера» при сервисном режиме).
```
# Goal
string request_id
uint32 mission_epoch
string query                 # описание цели (open-vocab)
bool render_setofmark
float32 conf_threshold
---
# Result
uint8 outcome                # FOUND/NOT_FOUND/ABORTED
object_tracking_msgs/Candidate[] candidates
sensor_msgs/CompressedImage annotated   # Set-of-Mark рендер
---
# Feedback
uint32 frames_processed
float32 best_confidence
```

**`object_tracking_msgs/msg/Candidate.msg`** — кандидат Set-of-Mark.
```
uint32 mark_id               # номер метки на Set-of-Mark рендере
string label
float32 confidence
geometry_msgs/Point pixel    # x=u, y=v, z=depth_m (как в target_pixel_to_goal)
string source_frame_id
builtin_interfaces/Time stamp
sensor_msgs/RegionOfInterest bbox
```

### 2.4 Локализация / коррекция карты

**`ar_project_msgs/msg/MapOdomCorrection.msg`** — низкочастотная коррекция map→odom от edge-SLAM (НЕ TF-поток). Применяется/перевещается локально `map_odom_relay`.
```
std_msgs/Header header              # stamp = время, к которому относится коррекция
geometry_msgs/TransformStamped map_to_odom   # сама поправка map->odom
float64[36] covariance              # ковариация поправки (для гейтинга по неопределённости)
float64 fitness                     # качество подгонки SLAM (0..1)
uint32 seq                          # монотонный счётчик для отбраковки stale/переупорядочивания
bool relocalized                    # true при скачке (перелокализация) -> гейтинг скачка
```

### 2.5 Планирование (VLM mode)

**`object_tracking_msgs/msg/PlanStep.msg`** — один шаг плана, всегда FLAT-решаемый скилл; VLM выбирает только из реального списка (enum tool-call), координат не порождает.
```
uint8 skill                  # 0=EXPLORE_FRONTIER 1=GO_TO_POSE 2=APPROACH_DETECTION 3=GET_OBSERVATION 4=STOP
int32 frontier_id            # для EXPLORE_FRONTIER: id из реального списка фронтиров
uint32 approach_target_mark  # для APPROACH_DETECTION: mark_id из Candidate-списка
string arg_label             # текстовый аргумент (например, имя комнаты/объекта)
string step_id               # UUID шага
string rationale             # краткое обоснование (для логов/notes)
```

**`object_tracking_msgs/msg/Notes.msg`** — компактные заметки/суммаризация (буфер контекста вместо хранения кадров).
```
std_msgs/Header header
uint32 mission_epoch
string summary               # текущая компактная сводка от модели
string[] facts               # дискретные факты (visited rooms, найденные/исключённые объекты)
uint32 token_estimate        # оценка токенов для бюджетирования контекста
```

### 2.6 Heartbeat / здоровье кросс-линка

**`ar_project_msgs/msg/Heartbeat.msg`** — публикуется каждым кросс-линковым продьюсером с QoS deadline/liveliness; пропадание → переход VLM mode → FLAT.
```
std_msgs/Header header
string node_name             # "slam"/"detector"/"planner_orchestrator"/...
uint8 status                 # 0=OK 1=DEGRADED 2=DOWN
float32 cpu_load
float32 last_latency_ms      # измеренная задержка последнего ответа (для p99-таймаутов)
uint32 mission_epoch
```

Опциональный сервис для ручного/тестового форсирования режима:
**`ar_project_msgs/srv/SetMode.srv`**
```
uint8 mode    # 0=FLAT 1=VLM
---
bool accepted
uint8 active_mode
```

## 3. Таблица сложности и трудозатрат

Относительная сложность: S ≈ 1–2 дня, M ≈ 3–5 дней, L ≈ 6–10 дней (один инженер, включая тесты в Gazebo). Дни — грубая оценка, риск — вероятность регресса/переделки.

| Компонент | Пакет | Сложность | Дни | Риск | Комментарий |
|---|---|---|---|---|---|
| Реальный CiA-402 quick-stop на пути `write()` (controlword 0x6040, без блокирующего 50 мс SDO) | ar_project (EmbodiedRobotSystem) | M | 4 | Высокий | RT-путь, требует hardware-in-the-loop; ошибка = небезопасность |
| Per-cycle fault poll + CAN bus-off recovery | ar_project | M | 3 | Высокий | Сейчас только логирование раз в 100 циклов |
| `cmd_vel` watchdog + Collision Monitor интеграция | ar_project / search_coordinator | S | 2 | Средний | Стандартные кирпичи Nav2 + watchdog-узел |
| `use_sim_time` параметризация (убрать hard-coded True) | ar_project (конфиги/launch) | S | 1 | Низкий | Механическая правка во всех yaml/launch |
| rmw_zenoh + chrony + 12МБ буферы + QoS deadline/liveliness | оба репо (bring-up) | M | 4 | Средний | Сетевые тонкости, кросс-хост отладка |
| Gazebo-on-WSL bring-up + миры | ar_project (worlds/launch) | M | 3 | Средний | GPU/passthrough в WSL2 капризен |
| Локальный `/scan` через depthimage_to_laserscan для costmap | ar_project | S | 1 | Низкий | Готовый пакет, настройка |
| `ar_project_msgs` + `object_tracking_msgs` (все .action/.msg/.srv) | оба репо | S | 2 | Низкий | Объёмно, но прямолинейно |
| Executive FSM/BT (Search Coordinator, mission state, default action) | search_coordinator | L | 9 | Высокий | Сердце системы; mission-epoch, owner состояния |
| Локальный frontier-extractor + гистерезис (margin + min dwell) | search_coordinator | M | 4 | Средний | Антиосцилляция критична |
| Skill-action серверы (Explore/GoTo/Approach/GetObs/Stop), UUID-идемпотентность, preemption | search_coordinator | L | 8 | Высокий | Preempt/idempotency легко сделать неверно |
| `map_odom_relay` (hold-last-good, gating скачка/ковариации, отбраковка stale, rebroadcast < 0.2 с) | search_coordinator | M | 5 | Высокий | TF-корректность, гонки по времени |
| Облегчение Nav2 (NavFn+DWB, local costmap в odom, controller 8–10 Гц, убрать лишнее) | ar_project | M | 3 | Средний | Сейчас 15 Гц; нужен профайлинг на Pi |
| Адаптация `target_pixel_to_goal` (снять «soup», обернуть в ApproachDetection) | ar_project | S | 2 | Средний | Не сломать переиспользуемую математику |
| RTAB-Map: offline mapping→.db + online localization→`MapOdomCorrection` (не TF) | object_tracking | M | 5 | Высокий | Переход от TF-стрима к сообщению-коррекции |
| `DetectTarget` action + Set-of-Mark рендер (YOLOE default, DINO+MobileSAM fallback) | object_tracking | M | 5 | Средний | Бэкенды есть, нужен сервисный контракт |
| Детектор: детект staleness потока (no auto-reached на устаревшем пикселе) | object_tracking / search_coordinator | S | 2 | Высокий | Прямой FMEA-фикс |
| Planner Orchestrator (single-in-flight, UUID, p99-timeout, circuit-breaker, enum tool-call, streaming) | planner_orchestrator | L | 10 | Высокий | Самый трудный edge-компонент |
| Notes/summary-буфер + бюджет токенов | planner_orchestrator | M | 4 | Средний | Контроль контекста/стоимости |
| Anytime/async replan + commit-point adoption (lead-time) | planner_orchestrator + search_coordinator | M | 5 | Высокий | Тайминг; FLAT должен продолжать исполнять текущую подзадачу |
| Деградация VLM→FLAT по heartbeat/circuit-breaker | оба репо | M | 3 | Высокий | Должно быть бесшовно |
| Instruction-change = ABORT-and-reset + mission-epoch инвалидация UUID | search_coordinator | S | 2 | Высокий | FMEA-фикс; легко получить «зомби»-цели |
| FMEA-тесты в Gazebo (инъекция отказов: stale TF, потеря edge, bus-off, скачок локализации) | оба репо (test) | M | 5 | Средний | Тестовая инфраструктура |
| Удаление `reliable_prompt_sender` + латч-«soup» | ar_project | S | 1 | Низкий | Чистка; проверить, что никто не подписан |
| Hardware bring-up (реальный робот, CAN, RealSense, Wi-Fi) | ar_project | L | 8 | Высокий | Интеграция «всё вместе» на железе |

Суммарно грубо ≈ 110–120 человеко-дней; критический путь идёт через executive FSM, skill-серверы, `map_odom_relay` и Planner Orchestrator.
