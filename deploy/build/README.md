# Сборка и развёртывание на Pi / edge

Практически без ручных операций собирает двухуровневый стек: на **Pi** (робот) работает executive
и аппаратный интерфейс ros2_control; на **edge** (машина с GPU) работают детектор, VLM-оркестратор
и SLAM. `deploy.sh` (и обёртка `Makefile`) берут на себя синхронизацию исходников и
удалённую/локальную `colcon build`, так что вручную собирать на каждом хосте не нужно.

## Однократная настройка
1. `make setup` → создаёт `deploy.env` из примера. Отредактируйте `PI_HOST` и `PI_USER`
   (остальное определяется автоматически). `deploy.env` добавлен в gitignore.
2. Беспарольный SSH до Pi: `ssh-copy-id $PI_USER@$PI_HOST` (чтобы синхронизация/сборка не запрашивали пароль).
3. На **Pi** однократно: установите ROS 2 + colcon + `rosdep`; `sudo rosdep init && rosdep update`.
   (Удалённая сборка запускает `rosdep install`, чтобы подтянуть недостающие зависимости пакетов.)

## Повседневное использование (из `deploy/build/`)
| команда | что она делает |
|---|---|
| `make edge`  | собирает `planner_orchestrator` + `object_tracking` (+зависимости) в `$EDGE_WS` на этой машине |
| `make pi`    | rsync обоих репозиториев → Pi, затем удалённая `colcon build` пакетов `search_coordinator` + `ar_project` (+зависимости) |
| `make all`   | сначала `edge`, затем `pi` |
| `make shell` | ssh на Pi с уже подключёнными (sourced) `/opt/ros` и `install/` рабочего пространства |
| `make doctor`| проверяет пути к репозиториям + доступность по SSH + ROS на обоих концах |

`make pi` идемпотентна: rsync зеркалирует ваше рабочее дерево (включая незакоммиченные правки),
исключая `build/ install/ log/ .git/ __pycache__/ model_weights/ *.pt *.ts *.env`, после чего
colcon `--symlink-install --packages-up-to` пересобирает только то, что изменилось. Веса YOLOE на 600 МБ
остаются на edge (детектор работает там) и никогда не передаются на Pi.

## Уровни (зависимости разрешаются автоматически через `--packages-up-to`)
- **Pi**: `search_coordinator`, `ar_project` → подтягивает `ar_project_msgs`, `object_tracking_msgs`,
  `fleet_comms`. C++-интерфейс оборудования (`embodied_robot_system`) компилируется нативно на Pi.
- **edge**: `planner_orchestrator`, `object_tracking` → подтягивает оба msg-пакета + `fleet_comms`.

## Затем поднимите систему (см. ../../docs/HIL_BRINGUP_CHECKLIST.md)
- edge: запустите zenoh-роутер `deploy/transport` + chrony-мастер `deploy/time_sync`.
- Pi: `ros2 launch ar_project hardware_bringup.launch.py` + executive
  (`coordinator_node` + `frontier_extractor`).
- edge: `detect_target_server` (в venv для YOLOE) + `orchestrator_node` (учётные данные VLM в переменных окружения).
