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
ARG ROS_DISTRO=jazzy

# ---------------------------------------------------------------------------
# base — общий слой: colcon + rosdep + инструменты сборки
# ---------------------------------------------------------------------------
FROM ros:${ROS_DISTRO}-ros-base AS base
ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential git python3-pip python3-venv \
      python3-colcon-common-extensions python3-rosdep \
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
# --skip-keys: CAN-рантайм (canopen*) и реальная RealSense нужны ТОЛЬКО на роботе,
# в симуляции используется gz_ros2_control и камера Gazebo, поэтому их пропускаем
# (C++-интерфейс железа всё равно компилируется — для сборки хватает lely+pluginlib).
RUN apt-get update \
 && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
      --skip-keys "canopen canopen_402_driver canopen_ros2_control realsense2_camera" \
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
# venv детектора (зеркалит ~/ot_venv из RUNBOOK): torch из CUDA-индекса + requirements
# + MobileSAM. --system-site-packages, чтобы venv видел rclpy/cv_bridge из /opt/ros.
RUN python3 -m venv --system-site-packages /opt/ot_venv \
 && /opt/ot_venv/bin/pip install --no-cache-dir --upgrade pip \
 && /opt/ot_venv/bin/pip install --no-cache-dir \
      torch torchvision --index-url https://download.pytorch.org/whl/cu124 \
 && /opt/ot_venv/bin/pip install --no-cache-dir -r /tmp/requirements.txt \
 && /opt/ot_venv/bin/pip install --no-cache-dir git+https://github.com/ChaoningZhang/MobileSAM.git
# По умолчанию — детектор; оркестратор запускается переопределением command в compose.
CMD ["/opt/ot_venv/bin/python", \
     "/ws/install/object_tracking/lib/object_tracking/detect_target_server", \
     "--ros-args", "-p", "use_sim_time:=true"]
