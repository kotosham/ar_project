"""ЕДИНАЯ точка входа стека: `mode:=sim|hardware`, `layer:=robot|edge|all`.

ЧТО ЭТОТ ФАЙЛ ДЕЛАЕТ И ЧЕГО НЕ ДЕЛАЕТ
=====================================
Он НЕ переписывает существующие launch-файлы. Он их включает с явными
аргументами. Основа взята с ЖЕЛЕЗНОГО пути: на роботе последовательность
подъёма сегодня набирается руками в пяти терминалах (`docs/RUNBOOK.md` §2.1,
Pi T1..T5), и именно там ошибка порядка стоит дороже всего — `map_odom_relay`,
запущенный ПОСЛЕ Nav2, оставляет костмапы без map->odom. Здесь эта
последовательность записана один раз и целиком.

Честная формулировка: общий у режимов ВЕРХНИЙ слой (детектор, оркестратор,
дашборд, протокол миссии — см. `edge_layer.launch.py`), а нижние слои разные по
существу: Gazebo против RealSense+CAN. «Общая логика» — это про верх, не про
низ.

ТАБЛИЦА СООТВЕТСТВИЯ (где какую команду запускать)
--------------------------------------------------
  layer=robot + mode=hardware  -> на Raspberry Pi (моторы, камера, Nav2, executive)
  layer=edge  + mode=hardware  -> на edge-боксе (реле камеры, RTAB-Map, детектор,
                                  оркестратор, дашборд)
  layer=all   + mode=sim       -> одна машина, весь стек целиком
  layer=all   + mode=hardware  -> только если Pi и edge физически одна машина;
                                  штатный железный сценарий — два разных хоста

ПРИМЕРЫ
-------
    # симуляция целиком, VLM-планировщик, мир по умолчанию
    ros2 launch ar_project mission_bringup.launch.py mode:=sim

    # симуляция с разрешением камеры «как на железе» и окном Gazebo
    ros2 launch ar_project mission_bringup.launch.py mode:=sim hw_parity:=true gui:=true

    # Raspberry Pi: весь нижний слой одной командой вместо пяти терминалов
    ros2 launch ar_project mission_bringup.launch.py mode:=hardware layer:=robot

    # edge-бокс
    ros2 launch ar_project mission_bringup.launch.py mode:=hardware layer:=edge \\
        venv_python:=/home/user/.venvs/ros-jazzy-ml/bin/python

ПРО ЗАДЕРЖКИ TimerAction
------------------------
Числа `delay_*_s` вынесены в аргументы, а не зашиты в код, потому что реальное
время подъёма `ros2_control` по CAN и RealSense на Pi на стенде НЕ ИЗМЕРЕНО.
Значения по умолчанию взяты по аналогии с ступенями симуляции
(`flat_sim_bringup.launch.py:114,124,137` — 10/20/28 с) и уменьшены, так как на
железе нет старта Gazebo. Выдавать их за проверенные было бы нечестно: если
Nav2 стартует раньше, чем поднялся `map_odom_relay`, увеличьте `delay_nav2_s`.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fleet_comms.mode_profiles import HARDWARE, MODES, PLANNERS, SIM, profile_for, urdf_canon

LAYERS = ('robot', 'edge', 'all')


def _arg(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _bool_value(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _tri_state(value, default):
    """Аргумент с ТРЕМЯ состояниями: 'true', 'false' и '' = «по режиму».

    Нужен там, где честного единого умолчания нет: `start_frontier_extractor`
    обязан быть true в симуляции и false на железе (DECISIONS §1.6), а
    DeclareLaunchArgument умеет только одно значение по умолчанию, ещё до того
    как известен mode.
    """
    if not str(value).strip():
        return default
    return _bool_value(value)


def _resolve_world(pkg, world):
    """Пусто -> house.sdf; id без разделителя пути -> <pkg>/worlds/<id>; иначе путь.

    Проверка существования обязательна и падает здесь, а не позже: gz на
    отсутствующем мире завершается, а `launch_sim.launch.py` вешает на этот
    процесс on_exit=Shutdown() — весь стек тогда молча сворачивается, и по логу
    не видно, что виноват путь к миру.
    """
    world = os.path.expanduser((world or '').strip())
    worlds_dir = os.path.join(pkg, 'worlds')
    if not world:
        return os.path.join(worlds_dir, 'house.sdf')
    if os.sep in world or (os.altsep and os.altsep in world):
        candidates = [world]
    else:
        candidates = [os.path.join(worlds_dir, world + suffix)
                      for suffix in ('', '.sdf', '.world')]
    for candidate in candidates:
        if os.path.isfile(candidate):
            return candidate
    raise RuntimeError(
        'mission_bringup: мир не найден: %r. Проверены пути: %s. Укажите world:= '
        'как id файла из ar_project/worlds (например world:=house) или как полный '
        'путь к .sdf/.world.' % (world, ', '.join(candidates))
    )


def _sim_robot_layer(context, pkg, launch_dir, prof, canon):
    """Нижний слой симуляции: существующий flat_sim_bringup + агрегатор здоровья."""
    world = _resolve_world(pkg, _arg(context, 'world'))

    # cam_* передаются ЯВНО и всегда. Без этого один и тот же
    # camera_gazebo_sensors.xacro даёт РАЗНЫЙ URDF в зависимости от точки входа:
    # flat_sim_bringup.launch.py:88 объявляет 320x240, а house_sim.launch.py:382
    # поднимает те же аргументы до 640x480. Канон живёт в mode_profiles.urdf_canon,
    # и переключается он ровно одним флагом hw_parity.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'flat_sim_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'world': world,
            'odom_topic': prof['odom_topic'],
            'gui': _arg(context, 'gui'),
            'rviz': _arg(context, 'rviz'),
            'spawn_x': _arg(context, 'spawn_x'),
            'spawn_y': _arg(context, 'spawn_y'),
            # spawn_z не выведен в аргументы: 0.1 — высота, на которой колёса не
            # проваливаются в пол, она одинакова для всех миров (launch_sim.launch.py).
            'spawn_z': '0.1',
            'spawn_yaw': _arg(context, 'spawn_yaw'),
            'cam_width': canon['cam_width'],
            'cam_height': canon['cam_height'],
            'cam_rate': canon['cam_rate'],
            'cam_far': canon['cam_far'],
            'depth_far': canon['depth_far'],
        }.items(),
    )

    actions = [
        LogInfo(msg='[mission_bringup] режим sim, мир: %s, камера URDF: %sx%s@%s Гц'
                    % (world, canon['cam_width'], canon['cam_height'], canon['cam_rate'])),
        sim,
    ]

    # flat_sim_bringup поднимает Gazebo, RTAB-Map, Nav2, frontier_extractor и
    # coordinator_node (:98-144), но robot_health_aggregator в нём НЕТ — его
    # добавлял только vlm_sim_bringup через start_monitor (:236-243). Без
    # /robot_health консоль оператора не может дать вердикт готовности, поэтому
    # здесь он поднимается явно; на железе он уже внутри
    # hardware_bringup.launch.py:194, и профиль это знает.
    if prof['starts_health_aggregator']:
        actions.append(Node(
            package='search_coordinator',
            executable='robot_health_aggregator',
            name='robot_health_aggregator',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))
    return actions


def _hardware_robot_layer(context, pkg, launch_dir, prof):
    """Нижний слой железа: дословная последовательность RUNBOOK §2.1, Pi T1..T5."""
    delay_realsense_s = float(_arg(context, 'delay_realsense_s'))
    delay_map_relay_s = float(_arg(context, 'delay_map_relay_s'))
    delay_nav2_s = float(_arg(context, 'delay_nav2_s'))
    delay_executive_s = float(_arg(context, 'delay_executive_s'))

    start_realsense = _bool_value(_arg(context, 'start_realsense'))
    start_nav2 = _bool_value(_arg(context, 'start_nav2'))
    start_executive = _bool_value(_arg(context, 'start_executive'))
    # DECISIONS §1.6: на железе frontier_extractor по умолчанию ВЫКЛЮЧЕН, потому
    # что в RUNBOOK §2.1 его нет ни в одном из пяти терминалов Pi, то есть на
    # стенде он не проверялся.
    start_frontier = _tri_state(_arg(context, 'start_frontier_extractor'), False)

    actions = [
        LogInfo(msg='[mission_bringup] режим hardware, порядок RUNBOOK §2.1: '
                    'hardware_bringup(0 с) -> realsense(%.1f с) -> map_odom_relay(%.1f с) '
                    '-> nav2(%.1f с) -> executive(%.1f с)'
                    % (delay_realsense_s, delay_map_relay_s, delay_nav2_s, delay_executive_s)),
    ]

    # T1: моторы по CAN, twist_mux, watchdog, /scan из depth, robot_health_aggregator.
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'hardware_bringup.launch.py')),
        launch_arguments={
            'can_interface_name': _arg(context, 'can_interface_name'),
            'use_collision_monitor': _arg(context, 'use_collision_monitor'),
        }.items(),
    ))

    # T2: RealSense. Профили понижены до 6 FPS — RUNBOOK §2.1 называет
    # 640x480x6 / 424x240x6 штатным режимом для VLM-тестов по Wi-Fi (дефолты
    # самого realsense_rgbd_pi.launch.py:36,41 — 15 FPS, для одного хоста).
    if start_realsense:
        actions.append(TimerAction(period=delay_realsense_s, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'realsense_rgbd_pi.launch.py')),
                launch_arguments={
                    'rgb_camera.color_profile': _arg(context, 'rgb_profile'),
                    'depth_module.depth_profile': _arg(context, 'depth_profile'),
                }.items(),
            )]))

    # T3: map->odom. Запускается ДО Nav2 не для красоты: костмапы, стартовавшие
    # без map->odom, поднимаются в global_costmap без трансформа и Nav2 уходит в
    # ошибку lifecycle-активации (RUNBOOK §2.1, «Pi T3 — запускать до Nav2»).
    actions.append(TimerAction(period=delay_map_relay_s, actions=[
        Node(
            package='search_coordinator',
            executable='map_odom_relay',
            name='map_odom_relay',
            parameters=[{'use_sim_time': False}],
            output='screen',
        )]))

    # T4: Nav2. params_file пиннится ЯВНО по той же причине, что и в
    # flat_sim_bringup.launch.py:130-134: collision_monitor.launch.py (включается
    # из hardware_bringup при use_collision_monitor:=true) объявляет собственный
    # аргумент params_file со значением collision_monitor.yaml, и эта
    # LaunchConfiguration протекает в этот include. Без строки ниже Nav2 читает
    # collision_monitor.yaml, DWB не находит критиков и bringup падает.
    if start_nav2:
        actions.append(TimerAction(period=delay_nav2_s, actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')),
                launch_arguments={
                    'use_sim_time': 'false',
                    'odom_topic': prof['odom_topic'],
                    'params_file': os.path.join(pkg, 'config', 'nav2_params.yaml'),
                }.items(),
            )]))

    # T5: executive + серверы навыков (GoToPose/ApproachDetection/Stop), которыми
    # пользуется и FLAT-цикл, и VLM-оркестратор.
    executive_nodes = []
    if start_frontier:
        executive_nodes.append(Node(
            package='search_coordinator',
            executable='frontier_extractor',
            name='frontier_extractor',
            parameters=[{'use_sim_time': False}],
            output='screen',
        ))
    if start_executive:
        executive_nodes.append(Node(
            package='search_coordinator',
            executable='coordinator_node',
            name='search_coordinator',
            parameters=[{'use_sim_time': False}],
            output='screen',
        ))
    if executive_nodes:
        actions.append(TimerAction(period=delay_executive_s, actions=executive_nodes))
    return actions


def _bringup(context, *args, **kwargs):
    pkg = get_package_share_directory('ar_project')
    launch_dir = os.path.join(pkg, 'launch')

    # Падать сразу и с русским текстом, а не молча делать не то — как это уже
    # сделано в house_sim.launch.py:203-207 для planner.
    mode = _arg(context, 'mode').lower()
    if not mode:
        raise RuntimeError(
            'mission_bringup: аргумент mode обязателен. Допустимые значения: %s. '
            'Умолчания у него нет намеренно: от mode зависит весь нижний слой '
            '(Gazebo против RealSense+CAN), и «угаданный» режим стоил бы часа '
            'диагностики.' % (' | '.join(MODES),))
    if mode not in MODES:
        raise RuntimeError('mission_bringup: mode:=%r недопустим. Допустимые значения: %s.'
                           % (mode, ' | '.join(MODES)))

    planner = _arg(context, 'planner').lower()
    if planner not in PLANNERS:
        raise RuntimeError('mission_bringup: planner:=%r недопустим. Допустимые значения: %s.'
                           % (planner, ' | '.join(PLANNERS)))

    layer = _arg(context, 'layer').lower()
    if layer not in LAYERS:
        raise RuntimeError('mission_bringup: layer:=%r недопустим. Допустимые значения: %s.'
                           % (layer, ' | '.join(LAYERS)))

    prof = profile_for(mode)
    canon = urdf_canon(_bool_value(_arg(context, 'hw_parity')))

    actions = []
    if layer in ('robot', 'all'):
        if mode == SIM:
            actions += _sim_robot_layer(context, pkg, launch_dir, prof, canon)
        elif mode == HARDWARE:
            actions += _hardware_robot_layer(context, pkg, launch_dir, prof)

    if layer in ('edge', 'all'):
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'edge_layer.launch.py')),
            launch_arguments={
                'mode': mode,
                'planner': planner,
                'venv_python': _arg(context, 'venv_python'),
                'vlm_env_file': _arg(context, 'vlm_env_file'),
                'vlm_model': _arg(context, 'vlm_model'),
                'start_dashboard': _arg(context, 'start_dashboard'),
                'dashboard_port': _arg(context, 'dashboard_port'),
                'max_steps': _arg(context, 'max_steps'),
                'replan_every_n': _arg(context, 'replan_every_n'),
                'rooms_spec': _arg(context, 'rooms_spec'),
                # Та же точка старта, что уходит в Gazebo: для планировщика это
                # начало кадра `map`, по нему он сдвигает комнаты в координаты
                # карты SLAM.
                'spawn_x': _arg(context, 'spawn_x'),
                'spawn_y': _arg(context, 'spawn_y'),
                'spawn_yaw': _arg(context, 'spawn_yaw'),
                'detect_memory_conf': _arg(context, 'detect_memory_conf'),
                'vlm_timeout_s': _arg(context, 'vlm_timeout_s'),
                # Агрегатор здоровья принадлежит НИЖНЕМУ слою: в sim его поднимает
                # ветка выше, на железе — hardware_bringup.launch.py:194. Второй
                # экземпляр публиковал бы в /robot_health наперегонки с первым.
                'start_health_aggregator': 'false',
            }.items(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='',
            description='sim | hardware. ОБЯЗАТЕЛЕН. Определяет НИЖНИЙ слой; верхний '
                        '(детектор, оркестратор, дашборд) в обоих режимах один и тот же.'),
        DeclareLaunchArgument(
            'layer', default_value='all',
            description='robot | edge | all. robot — то, что живёт на Pi; edge — «мозг» '
                        'на edge-боксе; all — обе половины на одной машине (штатно для sim).'),
        DeclareLaunchArgument(
            'planner', default_value='vlm',
            description='vlm | flat | mock. flat не поднимает детектор и оркестратор '
                        'вообще (FLAT-цикл живёт на роботе), mock — тот же оркестратор с '
                        'MockVlmClient и без кредов.'),
        DeclareLaunchArgument(
            'world', default_value='',
            description='Только mode=sim. Пусто = worlds/house.sdf. Значение без разделителя '
                        'пути трактуется как id файла в ar_project/worlds (world:=flat_detect), '
                        'иначе — как путь к .sdf/.world.'),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description='Окно Gazebo. Действует только при mode=sim. В контейнере без DISPLAY '
                        'включать нельзя.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='RViz. Действует только при mode=sim.'),
        DeclareLaunchArgument(
            'hw_parity', default_value='false',
            description='Только mode=sim: подставить в URDF разрешение железа (640x480 вместо '
                        '320x240). Платой служит RTF: репозиторий измерил, что 640x480 роняет '
                        'частоту кадров в WSL2 и RTAB-Map начинает отставать от одометрии '
                        '(camera_gazebo_sensors.xacro:10-14, house_sim.launch.py:69-76).'),
        DeclareLaunchArgument(
            'start_frontier_extractor', default_value='',
            description='Пусто = по режиму: в sim фронтиры нужны всегда, на железе выключено '
                        '(DECISIONS §1.6: в RUNBOOK §2.1 этого узла нет ни в одном терминале Pi, '
                        'то есть на стенде он не проверялся — включайте сознательно). При '
                        'mode=sim НЕ действует: flat_sim_bringup.launch.py:137-144 запускает '
                        'frontier_extractor безусловно.'),
        DeclareLaunchArgument(
            'can_interface_name', default_value='can0',
            description='Интерфейс SocketCAN к приводам EPOS4. Действует только при mode=hardware.'),
        DeclareLaunchArgument(
            'use_collision_monitor', default_value='false',
            description='Nav2 Collision Monitor после twist_mux. Действует только при '
                        'mode=hardware (в sim его включает launch_sim безусловно). По умолчанию '
                        'выключен: при нестабильных timestamp /scan он блокирует весь контур '
                        'движения (RUNBOOK §2.1).'),
        DeclareLaunchArgument(
            'start_realsense', default_value='true',
            description='Запустить RealSense на Pi. Действует только при mode=hardware.'),
        DeclareLaunchArgument(
            'rgb_profile', default_value='640x480x6',
            description='Профиль цвета RealSense. Действует только при mode=hardware. 6 FPS — '
                        'штатный режим VLM-тестов по Wi-Fi (RUNBOOK §2.1, Pi T2).'),
        DeclareLaunchArgument(
            'depth_profile', default_value='424x240x6',
            description='Профиль глубины RealSense. Действует только при mode=hardware.'),
        DeclareLaunchArgument(
            'start_nav2', default_value='true',
            description='Действует только при mode=hardware: в sim Nav2 поднимает '
                        'flat_sim_bringup.launch.py:124-135 безусловно.'),
        DeclareLaunchArgument(
            'start_executive', default_value='true',
            description='coordinator_node (серверы навыков GoToPose/ApproachDetection/Stop). '
                        'Действует только при mode=hardware: в sim его поднимает '
                        'flat_sim_bringup.launch.py:141-143 безусловно.'),
        DeclareLaunchArgument(
            'delay_realsense_s', default_value='3.0',
            description='Задержка старта RealSense после hardware_bringup, с. Только '
                        'mode=hardware. Значение НЕ ИЗМЕРЕНО на стенде — потому и вынесено '
                        'в аргумент.'),
        DeclareLaunchArgument(
            'delay_map_relay_s', default_value='5.0',
            description='Задержка старта map_odom_relay, с. Только mode=hardware. Обязан '
                        'подняться раньше Nav2 (RUNBOOK §2.1, Pi T3).'),
        DeclareLaunchArgument(
            'delay_nav2_s', default_value='8.0',
            description='Задержка старта Nav2, с. Только mode=hardware. Если в логе Nav2 видно '
                        'ожидание трансформа map->odom — увеличьте.'),
        DeclareLaunchArgument(
            'delay_executive_s', default_value='14.0',
            description='Задержка старта executive, с. Только mode=hardware. Ждёт активации '
                        'lifecycle-узлов Nav2: coordinator_node на старте ищет сервер '
                        'navigate_to_pose.'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Точка появления робота, X [м]. Только mode=sim.'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Точка появления робота, Y [м]. Только mode=sim.'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Курс робота при появлении [рад]. Только mode=sim.'),
        DeclareLaunchArgument(
            'venv_python', default_value='~/.venvs/ros-jazzy-ml/bin/python',
            description='Интерпретатор с torch/cv2/rclpy для детектора и оркестратора. '
                        'Передаётся в edge_layer.'),
        DeclareLaunchArgument(
            'vlm_env_file', default_value='',
            description='Путь к vlm.env с кредами VLM. Пусто = взять из переменной окружения '
                        'VLM_ENV_FILE, иначе не загружать. Значение ключа никуда не '
                        'печатается — см. edge_layer.launch.py.'),
        DeclareLaunchArgument(
            'vlm_model', default_value='',
            description='Переопределяет VLM_MODEL из файла. Имя модели секретом не является.'),
        DeclareLaunchArgument(
            'start_dashboard', default_value='true',
            description='Поднять mission_dashboard (http://<host>:8088). Передаётся в edge_layer.'),
        DeclareLaunchArgument(
            'dashboard_port', default_value='8088',
            description='Порт mission_dashboard.'),
        DeclareLaunchArgument(
            'max_steps', default_value='40',
            description='Потолок шагов миссии оркестратора.'),
        DeclareLaunchArgument(
            'rooms_spec', default_value=''),
        DeclareLaunchArgument(
            'detect_memory_conf', default_value='0.55',
            description='Порог, с которого детекция отмечается на карте планировщика '
                        'как найденный предмет. Подробности и предупреждение про '
                        'плоские билборды в симуляции — в edge_layer.launch.py.'),
        DeclareLaunchArgument(
            'replan_every_n', default_value='1',
            description='Через сколько шагов оркестратор перезапрашивает план у VLM.'),
        DeclareLaunchArgument(
            'vlm_timeout_s', default_value='30.0',
            description='Таймаут одного запроса к VLM, с.'),
        # Вся логика — в OpaqueFunction: ветвление по mode требует perform(context),
        # а PythonExpression на десяток узлов нечитаем и молча выдаёт пустую строку
        # при опечатке в значении.
        OpaqueFunction(function=_bringup),
    ])
