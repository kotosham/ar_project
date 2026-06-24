# AR Project

Этот репозиторий содержит окружение для симуляции пользовательского робота в Gazebo с использованием SLAM Toolbox, Nav2 и RViz2. Робота можно запускать в разных мирах, строить карты с помощью SLAM и выполнять навигацию с помощью Nav2.

Проект основан на [этом](https://www.youtube.com/playlist?list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT) плейлисте на YouTube

## Robot Description

Робот представляет собой мобильную платформу с дифференциальным приводом, оснащённую двумя задними колёсами и одним всенаправленным передним колесом для пассивной балансировки. Он оборудован следующими датчиками:

- 2D-лидар для построения карт и обнаружения препятствий
- RGB-камера для получения визуальной информации
- Камера глубины для 3D-восприятия и оценки расстояний

## Project Structure

- `config/` — Конфигурационные файлы для SLAM, контроллеров, RViz и навигации.
- `description/` — Файлы URDF/XACRO, описывающие модель робота.
- `launch/` — Скрипты запуска для симуляции, SLAM, навигации и визуализации.
- `maps/` — Заранее построенные карты, соответствующие каждому миру.
- `worlds/` — Файлы миров Gazebo для тестирования симуляции.

## How to Run the Simulation

### 1. Launch Gazebo Simulation with a Specific World
Замените `<n>` на номер мира (1, 2 или 3):
```bash
ros2 launch ar_project launch_sim.launch.py world:=./src/ar_project/worlds/test_<n>.world
```

### 2. Launch SLAM Toolbox

Перед запуском обновите поле map_file_name в config/mapper_params_online_async.yaml:

```yaml
map_file_name: "home/<user>/<path_to_ROS_workspace>/src/ar_project/maps/test_map_<n>/test_world_<n>_map_serial"
```

Затем выполните:

```bash
ros2 launch slam_toolbox online_async_launch.py \
    slam_params_file:=./src/ar_project/config/mapper_params_online_async.yaml \
    use_sim_time:=true

```

### 3. Launch Nav2 Stack

```bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

### 4. Launch RViz2

```bash
rviz2
```

## Maps and Worlds

| World File     | Map Preview                          |
|----------------|---------------------------------------|
| `test_1.world` | ![test_1](./images/test_1_preview.png) |
| `test_2.world` | ![test_2](./images/test_2_preview.png) |
| `test_3.world` | ![test_3](./images/test_3_preview.png) |

## Dependencies

Убедитесь, что установлены следующие пакеты ROS 2:

- `ros-<distro>-gazebo-ros-pkgs`
- `ros-<distro>-slam-toolbox`
- `ros-<distro>-nav2-bringup`
- `ros-<distro>-robot-state-publisher`
- `ros-<distro>-joint-state-publisher`
- `ros-<distro>-xacro`
- `ros-<distro>-rviz2`
- `ros-<distro>-ros2-control`
- `ros-<distro>-ros2-controllers`

Замените `<distro>` на название вашего дистрибутива ROS 2 (например, `humble`, `foxy`, `galactic`).

Чтобы установить все зависимости, выполните:

```bash
sudo apt install ros-<distro>-gazebo-ros-pkgs ros-<distro>-slam-toolbox ros-<distro>-nav2-bringup \
ros-<distro>-robot-state-publisher ros-<distro>-joint-state-publisher ros-<distro>-xacro \
ros-<distro>-rviz2 ros-<distro>-ros2-control ros-<distro>-ros2-controllers
```

## Building the Workspace

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```