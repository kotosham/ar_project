# syntax=docker/dockerfile:1.7
# Многоцелевой образ для автоматического поднятия стека на ПК (симуляция + edge).
#
# Контекст сборки = корень mobile_robot_navigation (родитель ar_project), чтобы в один
# образ попали оба репозитория (ar_project + object_tracking). Запускать через compose:
#   docker compose -f ar_project/docker/docker-compose.yml --profile sim build
#
# Цели (target):
#   sim  — Gazebo Harmonic + executive (CPU). «Поднять весь FLAT на ПК».
#   edge — детектор YOLOE (torch/GPU) + VLM-оркестратор. Без Gazebo и CAN.
#   full — ВСЁ сразу: sim-пакеты + object_tracking + planner_orchestrator + operator_console
#          + ML-venv. Единственный образ, из которого консоль оператора реально способна
#          породить и нижний, и верхний слой (см. комментарий у самой цели).
ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# base — общий слой: colcon + rosdep + инструменты сборки
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
# rmw-zenoh-cpp ставится в БАЗОВЫЙ слой, потому что подключение к реальному роботу
# (docker-compose.robot.yml) идёт только через него: сессия zenoh в режиме client
# (deploy/transport/zenoh_session_config.json5:14) делает РОВНО ОДНО ИСХОДЯЩЕЕ
# TCP-соединение на tcp/<edge>:7447 и потому проходит через bridge+NAT Docker Desktop,
# где network_mode: host на Windows и macOS отсутствует. Без этого пакета
# RMW_IMPLEMENTATION=rmw_zenoh_cpp роняет ЛЮБОЙ узел ещё до первой подписки.
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git python3-pip python3-venv \
      python3-colcon-common-extensions python3-rosdep \
      ros-${ROS_DISTRO}-rmw-zenoh-cpp \
 && rm -rf /var/lib/apt/lists/*
RUN rosdep init 2>/dev/null || true && rosdep update
WORKDIR /ws
COPY ar_project/docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

# ---------------------------------------------------------------------------
# sim — симуляция Gazebo Harmonic + исполнитель (executive). CPU.
# ---------------------------------------------------------------------------
FROM base AS sim
ARG ROS_DISTRO
# Пакеты, нужные для симуляции и исполнителя.
COPY ar_project/ar_project/                 /ws/src/ar_project/
COPY ar_project/ar_project_msgs/            /ws/src/ar_project_msgs/
COPY ar_project/search_coordinator/         /ws/src/search_coordinator/
COPY ar_project/fleet_comms/                /ws/src/fleet_comms/
COPY object_tracking/object_tracking_msgs/  /ws/src/object_tracking_msgs/
RUN rm -rf /ws/src/*/build /ws/src/*/install /ws/src/*/log
# rosdep тянет nav2/rtabmap/robot_localization/twist_mux/lely и пр.
# --skip-keys: пропускаем ТОЛЬКО то, что не нужно на этапе сборки.
#   canopen              — метапакет CAN-рантайма, в симуляции не используется;
#   realsense2_camera    — реальная камера, в симуляции её заменяет камера Gazebo.
# canopen_402_driver и canopen_ros2_control СКИПАТЬ НЕЛЬЗЯ, хотя в package.xml они
# объявлены только как exec_depend: CMakeLists.txt:20-21 делает по ним
# find_package(... REQUIRED) на этапе configure, и без них colcon падает с
# «Could not find a package configuration file provided by canopen_402_driver».
# Проверено: с ними в skip-keys сборка образа не проходит вообще.
RUN apt-get update \
 && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
      --skip-keys "canopen realsense2_camera" \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gz-ros2-control \
 && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install
CMD ["ros2", "launch", "ar_project", "flat_sim_bringup.launch.py"]

# ---------------------------------------------------------------------------
# edge — детектор (YOLOE/torch, GPU) + VLM-оркестратор. Без Gazebo/CAN.
# ---------------------------------------------------------------------------
FROM base AS edge
ARG ROS_DISTRO
# Индекс колёс torch вынесен в аргумент: cu124 собирается только там, где есть
# NVIDIA-совместимые колёса. На Apple Silicon и любой машине без NVIDIA сборку надо
# запускать с TORCH_INDEX_URL=https://download.pytorch.org/whl/cpu — иначе pip не найдёт
# подходящее колесо и образ edge не соберётся вовсе.
ARG TORCH_INDEX_URL=https://download.pytorch.org/whl/cu124
COPY object_tracking/object_tracking/       /ws/src/object_tracking/
COPY object_tracking/object_tracking_msgs/  /ws/src/object_tracking_msgs/
COPY object_tracking/planner_orchestrator/  /ws/src/planner_orchestrator/
COPY ar_project/ar_project_msgs/            /ws/src/ar_project_msgs/
COPY ar_project/fleet_comms/                /ws/src/fleet_comms/
COPY object_tracking/requirements.txt       /tmp/requirements.txt
RUN rm -rf /ws/src/*/build /ws/src/*/install /ws/src/*/log
RUN apt-get update \
 && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
 && rm -rf /var/lib/apt/lists/*
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install
# venv детектора (зеркалит ~/ot_venv из RUNBOOK): torch из выбранного индекса + requirements
# + MobileSAM. --system-site-packages, чтобы venv видел rclpy/cv_bridge из /opt/ros.
RUN python3 -m venv --system-site-packages /opt/ot_venv \
 && /opt/ot_venv/bin/pip install --no-cache-dir --upgrade pip \
 && /opt/ot_venv/bin/pip install --no-cache-dir \
      torch torchvision --index-url ${TORCH_INDEX_URL} \
 && /opt/ot_venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
 && /opt/ot_venv/bin/pip install --no-cache-dir git+https://github.com/ChaoningZhang/MobileSAM.git
# По умолчанию — детектор; оркестратор запускается переопределением command в compose.
CMD ["/opt/ot_venv/bin/python", \
     "/ws/install/object_tracking/lib/object_tracking/detect_target_server", \
     "--ros-args", "-p", "use_sim_time:=true"]

# ---------------------------------------------------------------------------
# full — консоль оператора и ВЕСЬ стек в одном образе.
#
# ПОЧЕМУ ОТДЕЛЬНАЯ ЦЕЛЬ, А НЕ `FROM sim AS console`.
# Консоль не просто показывает страницу: она порождает подпроцессом верхний слой
# (edge_layer), который резолвит пути через get_package_prefix('object_tracking') и
# запускает детектор с планировщиком интерпретатором из параметра venv_python.
# В слое sim нет ни object_tracking, ни planner_orchestrator, ни venv — то есть
# наследование от sim давало бы образ, в котором преflight консоли НИКОГДА не станет
# зелёным, а шаг «отправить задание» недостижим в принципе. Поэтому full = base +
# ВСЕ пакеты обоих репозиториев + ros-gz + gz_ros2_control + /opt/ot_venv.
# Цели sim и edge оставлены: они по-прежнему нужны для РАЗДЕЛЬНОГО развёртывания
# (Gazebo на одной машине, GPU-детектор на другой), где тащить лишние 6 ГБ незачем.
# ---------------------------------------------------------------------------
FROM base AS full
ARG ROS_DISTRO
ARG TORCH_INDEX_URL=https://download.pytorch.org/whl/cu124
COPY ar_project/ar_project/                 /ws/src/ar_project/
COPY ar_project/ar_project_msgs/            /ws/src/ar_project_msgs/
COPY ar_project/search_coordinator/         /ws/src/search_coordinator/
COPY ar_project/fleet_comms/                /ws/src/fleet_comms/
COPY ar_project/operator_console/           /ws/src/operator_console/
COPY object_tracking/object_tracking/       /ws/src/object_tracking/
COPY object_tracking/object_tracking_msgs/  /ws/src/object_tracking_msgs/
COPY object_tracking/planner_orchestrator/  /ws/src/planner_orchestrator/
COPY object_tracking/requirements.txt       /tmp/requirements.txt
RUN rm -rf /ws/src/*/build /ws/src/*/install /ws/src/*/log
# См. подробный комментарий к тому же вызову в цели sim: canopen_402_driver и
# canopen_ros2_control нужны на этапе configure (CMakeLists.txt:20-21), скипать их
# нельзя, несмотря на то что в package.xml они помечены exec_depend.
RUN apt-get update \
 && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
      --skip-keys "canopen realsense2_camera" \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-ros-gz ros-${ROS_DISTRO}-gz-ros2-control \
 && rm -rf /var/lib/apt/lists/*
# Превью миров генерируются ДО colcon build и пишутся в исходный каталог
# ar_project/ar_project/worlds/previews — только оттуда их заберёт уже существующий
# install(DIRECTORY ... worlds ...) (CMakeLists.txt:64-67). После сборки писать поздно.
# `|| true` намеренно: отсутствие превью — это одна жёлтая строка в преflight консоли,
# а не повод завалить сборку шеститигабайтного образа.
# --force ОБЯЗАТЕЛЕН: без него скрипт видит уже закоммиченные worlds/previews/*.svg,
# печатает «пропущен (уже есть)» по каждому миру и выходит с кодом 0 — то есть шаг
# формально проходит, но не генерирует НИЧЕГО и не замечает, что превью устарело
# относительно изменённого .world. С --force содержимое образа всегда соответствует
# тем мирам, которые в него реально попали.
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && python3 /ws/src/ar_project/scripts/make_world_previews.py --force || true
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install
RUN python3 -m venv --system-site-packages /opt/ot_venv \
 && /opt/ot_venv/bin/pip install --no-cache-dir --upgrade pip \
 && /opt/ot_venv/bin/pip install --no-cache-dir \
      torch torchvision --index-url ${TORCH_INDEX_URL} \
 && /opt/ot_venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
 && /opt/ot_venv/bin/pip install --no-cache-dir git+https://github.com/ChaoningZhang/MobileSAM.git
# Дефолт параметра venv_python в конфиге консоли — '~/.venvs/ros-jazzy-ml/bin/python'
# (путь с рабочей станции разработчика). Симлинк делает этот дефолт рабочим и внутри
# образа: если оператор нажмёт «Запустить» до того, как compose передаст явный
# -p venv_python:=/opt/ot_venv/bin/python, детектор всё равно найдёт интерпретатор,
# а не упадёт с «нет такого файла».
RUN mkdir -p /root/.venvs && ln -sfn /opt/ot_venv /root/.venvs/ros-jazzy-ml
EXPOSE 8090 8088
CMD ["ros2", "run", "operator_console", "console_node", "--ros-args", \
     "-p", "bind:=0.0.0.0", "-p", "port:=8090", \
     "-p", "vlm_env_path:=/config/vlm.env", \
     "-p", "venv_python:=/opt/ot_venv/bin/python", \
     "-p", "vlm_env_template:=/ws/install/planner_orchestrator/share/planner_orchestrator/vlm.env.example"]
