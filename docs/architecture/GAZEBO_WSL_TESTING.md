> **Поправки верификации (имеют приоритет над текстом ниже).**
> - **`tc netem` НЕ работает на стоковом ядре WSL2** (нет модуля `sch_netem`) — ровно как `vcan`. Для инъекции деградации Wi-Fi нужен **пересобранный WSL2-ядро** с `CONFIG_NET_SCH_NETEM` (+`CONFIG_NET_CLS_*`), либо второй физический хост / не-WSL VM. Без этого сетевые FMEA-строки в WSL2 **невоспроизводимы** (в матрице ниже — «только с кастомным ядром»).
> - **VRAM Qwen3-VL-30B-A3B:** FP16 ≈ 67 ГБ, **FP8 ≈ 33–34 ГБ** (комфортно на 80/48 ГБ, **тесно на 32 ГБ**, **не влезает в 24 ГБ**). На 24 ГБ dev-GPU реально только **4-битная** сборка (~17–23 ГБ) для коротких контекстов. Для sim предпочтителен **удалённый OpenAI-совместимый endpoint** или меньший dense-VLM.
> - **GUI gz под WSLg:** идти через **XWayland (X11)**; при чёрном экране/краше OGRE2 — **снять `WAYLAND_DISPLAY`** и/или `LIBGL_ALWAYS_SOFTWARE=1`. Возможны утечка памяти WSLg и зависание при закрытии 3D-окна.
> - **GPU-рендер сенсоров OGRE2 под WSL2 ненадёжен** (краши, чёрные кадры, откат на CPU). Симулированный RealSense RGB-D может не давать целевой FPS в WSL2 — валидировать на нативном Linux / edge-хосте.
> - **gz `DiffDrive` ждёт `geometry_msgs/Twist`, а Nav2/`diff_drive_controller` (Jazzy) шлёт `TwistStamped`** — согласовать мостом / `use_stamped_vel`, иначе sim-робот не поедет.
> - **vcan не воспроизводит настоящий bus-off** (виртуальный loopback) — его можно лишь скриптово сымитировать в fake-EPOS4; истинный bus-off — только на железе.

## Тестирование в Gazebo на WSL2 Ubuntu

Этот раздел описывает, как поднять и отлаживать всю систему `robust` (ветка `robust` в `ar_project` и `object_tracking`) в симуляции до выхода на реальное железо. Симуляция — **первый** обязательный этап: сначала измеряется чистый Pi-baseline (FLAT, без VLM), и только после него включаются VLM-режим и деградации. Все приведённые ниже команды выполняются внутри образа WSL2 Ubuntu (Ubuntu 24.04 + ROS 2 Jazzy).

### 1. Симулятор: Gazebo Sim (gz), не Classic

Репозиторий уже использует **Gazebo Sim** (новый `gz`, ранее «Ignition»), а не Gazebo Classic. Это подтверждается описанием робота: плагины с пространством имён `gz::sim::systems::*` (`gz-sim-diff-drive-system`, `gz-sim-joint-state-publisher-system` в `gazebo_control.xacro`), интеграция через `gz_ros2_control/GazeboSimSystem` (`ros2_control.xacro`), мост `ros_gz_bridge` (`config/gz_bridge.yaml`) и запуск симулятора командой `gz sim ... --force-version 8` в `launch_sim.launch.py`. Для ROS 2 Jazzy штатная и протестированная связка — **Gazebo Harmonic (gz-sim8)** + `ros_gz` + `gz_ros2_control`. Classic в документации и сборках не используем и не упоминаем как опцию.

Запуск симуляции (из `launch_sim.launch.py`):

```bash
# FLAT/baseline: управление колёсами штатным DiffDrive-плагином gz (use_ros2_control:=false)
ros2 launch ar_project launch_sim.launch.py world:=<path>/test_world.sdf gui:=true

# Полный стек, идентичный железу по интерфейсу управления (см. раздел 3)
ros2 launch ar_project launch_sim.launch.py use_ros2_control:=true gui:=false
```

### 2. Симуляция сенсорного набора (RealSense-подобный RGB + выровненный depth + IMU)

Цель — чтобы перцепция (`object_tracking`) и `target_pixel_to_goal` получали **те же топики и те же `frame_id`**, что и на реальном роботе с RealSense, и не знали, что под ними симуляция.

**RGB + depth.** Уже реализовано в `description/camera_gazebo_sensors.xacro`: два gz-сенсора (`type="camera"` и `type="depth"`), 640×480 @ 30 Гц, `horizontal_fov=1.089`, `clip near=0.05 far=8.0`, привязанные к оптическому кадру `camera_link_optical` (`gz_frame_id`). `depth_camera.xacro` сохраняет legacy-кадры `depth_camera_link*` как алиасы поверх того же физического `camera_link`, поэтому существующий код продолжает работать без правок. `camera_gazebo_sensors.xacro` намеренно вынесен отдельно — реальное описание робота (`robot_hardware.urdf.xacro`) содержит только геометрию и кадры, а sim-сенсоры подключаются только в симуляции.

**Эмуляция «aligned depth».** Реальная RealSense публикует `aligned_depth_to_color`. В симуляции отдельного выравнивания нет: depth-сенсор смонтирован на тот же оптический кадр, что и RGB, с тем же FOV и разрешением, поэтому пиксели уже совпадают. Совпадение топиков обеспечивает `config/gz_bridge.yaml`, который ремаппит один gz-топик глубины на все имена, ожидаемые стеком:

- `camera/camera/color/image_raw` и `camera/camera/color/camera_info` — RGB;
- `camera/camera/depth/image_rect_raw` → дополнительно публикуется как `camera/camera/aligned_depth_to_color/image_raw` (то самое имя, что использует реальный драйвер и потребляет `target_pixel_to_goal`);
- `camera/camera/aligned_depth_to_color/camera_info` берётся из **color/camera_info** (совпадение интринсиков выровненного потока).

Важно: `gz_bridge.yaml` уже содержит `camera/camera/depth/color/points` (PointCloud2). В симуляции этот топик допустимо использовать **только локально** для проверки; **по Wi-Fi его и raw-depth слать запрещено** (архитектурное правило) — для costmap локально генерируется `/scan` через `depthimage_to_laserscan`. Это правило одинаково в sim и на железе, и его нужно тестировать именно так: detector/SLAM на edge получают RGB и низкочастотные сообщения, а не облака точек.

**IMU.** В текущем описании gz-IMU **ещё нет** (в `camera_gazebo_sensors.xacro` его нет, упоминание `imu` встречается только в комментарии `depth_camera.xacro`). Для воспроизведения связки EKF (`robot_localization`) её нужно добавить — это работа ветки `robust`:

1. В мир (SDF) добавить системный плагин: `<plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>`.
2. На `base_link` (или отдельный `imu_link`) добавить gz-сенсор `type="imu"`, `always_on=true`, `update_rate` под вход EKF (например 50–100 Гц), `<topic>imu/data</topic>`, `gz_frame_id` = кадр IMU.
3. В `gz_bridge.yaml` добавить мост `sensor_msgs/msg/Imu` ↔ `gz.msgs.IMU`, `direction: GZ_TO_ROS`, с тем же ROS-именем топика, что у реального драйвера (то имя, на которое подписан `imu_orientation_filter`/`robot_localization`).

Так EKF в симуляции получает odom (от DiffDrive или диф-контроллера) + IMU и выдаёт `odom->base_link` ровно как на железе.

### 3. Эмуляция аппаратного интерфейса EPOS4/CAN

На железе колёсами управляет `ros2_control`-плагин `ar_project/EmbodiedRobotSystem` (CiA-402 поверх SocketCAN, описан в `ros2_control_hardware.xacro`). В симуляции этот плагин не используется — вместо него один из двух механизмов, выбираемых тем же аргументом `use_ros2_control`, который уже есть в `robot.urdf.xacro` и `launch_sim.launch.py`:

- **`use_ros2_control:=false`** — `description/gazebo_control.xacro`: штатный gz-плагин `gz-sim-diff-drive-system` (`/cmd_vel` → odom/TF). Самый простой путь для FLAT-baseline, но это **не** путь `ros2_control`, поэтому стек управления не идентичен железу.
- **`use_ros2_control:=true`** — `description/ros2_control.xacro`: `gz_ros2_control/GazeboSimSystem` + те же контроллеры (`diff_cont` = `diff_drive_controller`, `joint_broad` = `joint_state_broadcaster`, `config/my_controllers.yaml`). Это **mock-замена** реального hardware-интерфейса: весь стек выше `ros2_control` (Nav2, twist_mux, диф-контроллер, имена интерфейсов команд/состояний колёс) **идентичен** железу, меняется только нижний `<hardware>`-плагин (`GazeboSimSystem` вместо `EmbodiedRobotSystem`). Для верификации интеграции используем именно этот режим.

Принцип «sim-vs-hw switch»: один и тот же `use_ros2_control` переключает `<hardware>`-плагин в URDF (xacro `if/unless` в `robot.urdf.xacro`), не затрагивая остальной граф. На железе грузится `robot_hardware.urdf.xacro` с `EmbodiedRobotSystem`; в sim — `ros2_control.xacro` с `GazeboSimSystem`. Альтернатива `GazeboSimSystem` — generic `mock_components/GenericSystem` из `ros2_control` (петля «команда = состояние» без физики); полезна для unit-тестов контроллера без gz, но для интеграции предпочтителен `GazeboSimSystem` с реальной физикой колёс.

**Что НЕ тестируется в Gazebo (нужно железо или fake-EPOS4 на vcan):**

- **Реальный CiA-402 quick-stop** (controlword `0x6040`) на RT-пути `write()`. Ни `gz-diff-drive`, ни `GazeboSimSystem` не моделируют конечный автомат CiA-402, statusword, fault-реакцию и неблокирующий SDO. Это FMEA-must-fix (текущий код только логирует fault) и проверяется **только** на реальном EPOS4 или через **fake-EPOS4** поверх SocketCAN.
- **CAN bus-off, per-cycle fault poll, потеря кадров на шине.** Шины CAN в gz нет.

**Эмуляция CAN на WSL2 (vcan + fake-EPOS4).** SocketCAN/`vcan` в WSL2 из коробки нет: стандартный WSL-ядро не содержит модулей `can`, `can_raw`, `vcan` — нужен **пересобранный кастомный WSL-ядро** с включённым «CAN bus subsystem support» (через `.wslconfig`). После этого:

```bash
sudo modprobe can can_raw vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
candump vcan0   # из can-utils
```

Далее на `vcan0` поднимается **fake-EPOS4** — узел/скрипт, отвечающий на SDO/PDO как привод CiA-402 (statusword, переходы состояний, эмуляция fault и bus-off по запросу). Тогда `EmbodiedRobotSystem` можно тестировать с `can_interface_name:=vcan0` без gz-физики. Это отдельный тестовый стенд, а не часть запуска Gazebo: gz даёт физику движения, vcan+fake-EPOS4 — поведение привода и обработку отказов. Их можно сводить, но для большинства FMEA-сценариев привода gz не нужен.

### 4. Эмуляция разделения Pi↔PC на одной WSL-машине

Реальная система — два хоста (Pi и GPU-PC) поверх Wi-Fi с `rmw_zenoh` (один systemd-router на edge), multicast off, QoS deadline/liveliness на cross-link топиках. На одной WSL-машине это эмулируется так:

**Разделение графов.** Два варианта:
- **`ROS_DOMAIN_ID`** — запускаем «Pi-узлы» в одном терминале с `ROS_DOMAIN_ID=10`, «edge-узлы» — с `ROS_DOMAIN_ID=20`. Полная изоляция, но без cross-link связи — годится для проверки, что Pi-стек (FLAT) **полностью автономен** и переживает «исчезновение» edge.
- **Локальный zenoh-router** — поднять один `rmw_zenoh` router (как на edge) и запускать обе группы узлов с `RMW_IMPLEMENTATION=rmw_zenoh_cpp`, чтобы воспроизвести **реальный транспорт** cross-link. Это и есть рекомендуемый режим для VLM-mode и тестов QoS deadline/liveliness, circuit-breaker и адопции планов в commit-point.

**Инъекция сетевой деградации Wi-Fi (latency / loss / jitter).** Поскольку оба «хоста» на одной машине, трафик идёт через loopback и физических потерь нет — деградацию вносим искусственно:

- **`tc netem`** на интерфейсе zenoh-трафика (или на `lo` через классификацию по порту): задержка, джиттер, потери, дублирование, переупорядочивание. Примеры:
  ```bash
  sudo tc qdisc add dev <iface> root netem delay 200ms 80ms loss 10% duplicate 2% reorder 25%
  sudo tc qdisc change dev <iface> root netem loss 100%   # полный обрыв Wi-Fi
  sudo tc qdisc del dev <iface> root                      # снять
  ```
- **Network namespaces** (`ip netns`) + veth-пара дают более реалистичную картину «двух хостов»: каждая группа узлов в своём namespace, `tc netem` на veth между ними. Это точнее эмулирует обрыв линка (узлы реально перестают видеть друг друга на сетевом уровне), тогда как `ROS_DOMAIN_ID`/loopback изолируют только логически.

Эти инструменты — основной способ тестировать **деградированные режимы**: VLM/edge/Wi-Fi loss → переход VLM-mode в FLAT, hold-last-good в `map_odom_relay`, отбрасывание устаревших stamp'ов, поведение circuit-breaker в Planner Orchestrator.

### 5. GPU-нагрузки под WSL2 (CUDA-on-WSL2)

На edge/PC под WSL2 запускаются vLLM (Qwen3-VL) и сегментационные модели (YOLOE / GroundingDINO+MobileSAM).

**Драйвер и toolkit.** Под WSL2 CUDA-драйвер — это **Windows-драйвер NVIDIA**. Внутри Ubuntu-WSL **нельзя** ставить Linux-дисплей-драйвер: пакеты `cuda`, `cuda-drivers` из обычного репозитория тянут Linux-драйвер и ломают связку. Ставится только **WSL-specific CUDA toolkit** (вариант `Linux > x86_64 > WSL-Ubuntu` на сайте NVIDIA). Проверка: `nvidia-smi` внутри WSL должен видеть GPU.

**VRAM и помещается ли 30B.** Qwen3-VL-30B-A3B — MoE с ~3B активных параметров; качество близко к dense-32B, но **по VRAM это всё равно ~30B весов** (активны 3B на токен, но в память грузятся все эксперты). В FP8 веса ~30 ГБ, рантайм-футпринт с активациями/KV-кэшем/оверхедом фреймворка — ~37–40 ГБ, что комфортно на одном H100 80 ГБ, но **не помещается** на потребительские 24 ГБ (RTX 4090/3090) даже в 4-битной квантизации с учётом vision-энкодера и KV-кэша. Требуется vLLM ≥ 0.11.0 (поддержка Qwen3-VL).

**Fallback при нехватке VRAM (типично для dev-WSL2-машины):**
- меньший VLM (например dense Qwen3-VL-8B/4B) локально, либо
- удалённый **OpenAI-совместимый endpoint** — Planner Orchestrator уже спроектирован как async-клиент к OpenAI-совместимому API (single-in-flight, UUID-идемпотентность, timeout от измеренного p99, circuit-breaker, streaming, structured/enum tool-call). Для симуляции в `base_url` подставляется удалённый сервис вместо локального vLLM — остальной стек не меняется.

Сегментация (YOLOE и др.) по VRAM скромная (единицы ГБ) и спокойно соседствует с маленьким VLM на одной карте; с 30B-VLM на 80 ГБ — тоже.

### 6. GUI: WSLg vs headless gz

- **WSLg** (встроенный в современный WSL2) даёт GUI без X-сервера на Windows: окно Gazebo и RViz2 рендерятся через Wayland/`WAYLAND_DISPLAY`. `launch_sim.launch.py` уже учитывает особенность окружения: GUI-режим (`gui:=true`) стартует gz через `env -i` с whitelist'ом переменных (`DISPLAY`, `WAYLAND_DISPLAY`, `XDG_RUNTIME_DIR`, `XAUTHORITY` и т.д.), потому что запуск из snap-окружения VS Code ломает рантайм gz.
- **Headless** (`gui:=false`) запускает `gz sim -s --headless-rendering` — сервер без окна. Это режим для CI, для прогона FMEA-матрицы и для машин без WSLg/без рендера. Камеры/depth в gz при этом продолжают рендериться (нужен `--headless-rendering` для off-screen GPU-рендера сенсоров), а инспекция идёт через `ros2 topic`/`rviz2` отдельно или через запись rosbag.

Под WSL2 для GPU-рендера сенсоров gz желателен тот же NVIDIA-GPU (off-screen rendering); при проблемах с рендером — software-fallback (`LIBGL_ALWAYS_SOFTWARE=1`) ценой производительности.

### 7. Дисциплина `use_sim_time`, clock и TF

Жёсткое правило: **в симуляции `use_sim_time:=true` у ВСЕХ узлов, на железе — `false` у всех**. Источник времени в sim — gz, публикующий `/clock`; мост `clock` (`rosgraph_msgs/msg/Clock`, `GZ_TO_ROS`) уже есть первым в `gz_bridge.yaml`. Все узлы (EKF, Nav2, контроллеры, `target_pixel_to_goal`, перцепция, `map_odom_relay`) обязаны подписываться на `/clock` и брать stamp'ы оттуда — иначе окна валидности (TF `transform_tolerance` 0.2 с, depth-match 0.35 с, pixel-age 1.5 с) измеряются по неправильным часам.

В текущих launch-файлах `use_sim_time` зашит в значения для sim (например в `launch_sim.launch.py`: `rsp` с `use_sim_time:'true'`, `twist_mux`/`odom_to_tf` с `use_sim_time:True`; `my_controllers.yaml` имеет `use_sim_time: true`). Для ветки `robust` это нужно **параметризовать единым аргументом** `use_sim_time` на верхнем уровне launch, прокидываемым во **все** включаемые launch и узлы, со значением `true` в sim-bringup и `false` в `hardware_bringup.launch.py`. Не должно остаться узла с зашитым `True`, который случайно попадёт на железо.

**Clock skew (chrony).** На железе время хостов синхронизируется chrony; смещение обязано быть **много меньше** самого узкого окна (0.2 с TF). В симуляции на одной машине часы общие (`/clock`), поэтому реальный skew между Pi и edge **не воспроизводится** транспортом сам по себе — его нужно **инжектировать искусственно** (см. матрицу: сдвиг stamp'ов на edge-узле/в fake-публикаторе), чтобы проверить, что `map_odom_relay` отвергает устаревшие stamp'ы и что gating по covariance/jump работает.

**TF в sim.** `map->odom` от RTAB-Map на edge — это **низкочастотная коррекция-сообщение**, а НЕ поток TF. `map_odom_relay` на Pi применяет её, держит last-good, гейтит скачок/covariance, отвергает устаревшие stamp'ы и **сам** ребродкастит `map->odom` локально с частотой под `transform_tolerance < 0.2 с`. `odom->base_link` даёт EKF. В sim это проверяется: edge-SLAM питается симулированной RGB-D, выдаёт коррекцию, а на Pi-стороне TF-дерево остаётся валидным даже при обрыве линка (hold-last-good).

### 8. Матрица sim-тестов (FMEA → инъекция в Gazebo)

| FMEA-сценарий | Инъекция в sim | Ожидаемая реакция | Воспроизводимо в Gazebo? |
|---|---|---|---|
| **VLM медленный** | задержать ответ VLM (sleep в mock-endpoint или `tc netem delay` до vLLM); сжать timeout до p99 | single-in-flight держит план, FLAT продолжает текущий subtask, replan по lead-time, адопция в commit-point | Да (gz + mock/remote VLM) |
| **VLM/edge упал** | убить vLLM-процесс или edge-namespace; `tc netem loss 100%` к edge | circuit-breaker открывается, VLM-mode деградирует в FLAT, миссия продолжается | Да |
| **Обрыв Wi-Fi** | `tc netem loss 100%` / `ip link set down` veth между namespace'ами | `map_odom_relay` hold-last-good, Nav2 в `odom` продолжает, нет «фриза» Pi-стека | Да (netns точнее, чем loopback) |
| **map->odom залип/расходится** | заморозить или подать скачкообразную коррекцию из mock-SLAM | gating по jump/covariance отвергает плохую коррекцию, держит last-good, ребродкаст продолжается | Да |
| **Clock skew** | искусственно сдвинуть stamp'ы edge-публикатора относительно `/clock` | отвержение устаревших stamp'ов в relay; проверка границ 0.2/0.35/1.5 с | Частично — нужна ручная инъекция (общие часы в sim) |
| **OOM детектора** | ограничить VRAM (`CUDA_VISIBLE_DEVICES`/MPS-лимит) или убить detector; подать «нет детекций» | стек не падает, FLAT exploration продолжается, нет ложного «reached» | Да (на GPU-WSL2); реальный CUDA-OOM — частично |
| **Stale/дублирующий результат** | повторно подать тот же UUID-результат / устаревший по времени | UUID-идемпотентность отбрасывает дубль; stale по pixel-age (1.5 с) игнорируется | Да |
| **Изменение инструкции в полёте** | отправить новую инструкцию во время активного subtask | executive ABORT-and-reset, инкремент mission-epoch, инвалидизация in-flight UUID, preempt action-серверов | Да (чисто логика, gz даёт движение) |
| **Осцилляция фронтиров** | мир с двумя почти равными фронтирами | гистерезис (margin по score + min dwell) не даёт переключаться туда-обратно | Да (важен подбор world) |
| **Approach по устаревшему пикселю** | прекратить поток детектора во время ApproachDetection (детекция «замерла») | обнаружение staleness потока, **нет** авто-`SUCCEEDED` по goal_locked на старом пикселе, переход в re-detect/abort | Да |
| **CiA-402 quick-stop / CAN bus-off** | — | реальный controlword 0x6040, per-cycle fault poll, обработка bus-off | **Нет** — только железо или vcan+fake-EPOS4 (раздел 3) |
| **cmd_vel watchdog / Collision Monitor** | прекратить публикацию `/cmd_vel`; поставить препятствие перед роботом в gz | watchdog тормозит, Collision Monitor режет скорость/стопит до столкновения | Да |

**Сводно — что НЕ ловится в Gazebo:** реальная динамика CiA-402 (quick-stop, statusword, неблокирующий SDO), CAN bus-off и потери кадров на шине, истинный clock skew между физическими хостами (в sim общий `/clock`), реальная физика потерь Wi-Fi (эмулируется `tc netem`/netns), а также честный CUDA-OOM именно на целевой VRAM edge-бокса, если dev-машина мощнее/слабее. Всё остальное из FMEA воспроизводится комбинацией gz + mock/remote VLM + `tc netem`/netns + sim-инъекции stamp'ов и потоков.

---

Релевантные файлы (абсолютные пути):
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\description\camera_gazebo_sensors.xacro` — gz RGB+depth сенсоры (сюда добавлять gz-IMU)
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\description\camera.xacro`, `depth_camera.xacro` — кадры/алиасы RGB и depth
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\description\ros2_control.xacro` (`GazeboSimSystem`, sim) vs `ros2_control_hardware.xacro` (`EmbodiedRobotSystem`/EPOS4 CAN, hw)
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\description\gazebo_control.xacro` — gz DiffDrive (FLAT-baseline путь)
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\description\robot.urdf.xacro` — переключатель `use_ros2_control`
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\launch\launch_sim.launch.py` — запуск gz, `gui`/headless, `use_sim_time`
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\config\gz_bridge.yaml` — мост clock/camera/odom, алиасы aligned_depth
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\config\my_controllers.yaml` — `diff_cont`/`joint_broad`, `use_sim_time`
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\launch\hardware_bringup.launch.py` — hw-bringup (здесь `use_sim_time:=false`)

Примечания по правкам, требуемым в ветке `robust` (не сделаны в текущем коде): добавить gz-IMU-сенсор + плагин `gz-sim-imu-system` и мост IMU; вынести `use_sim_time` в единый прокидываемый аргумент во всех launch вместо зашитых `True`.