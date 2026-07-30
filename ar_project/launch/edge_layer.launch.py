"""ВЕРХНИЙ слой «мозг»: детектор + VLM-оркестратор + mission_dashboard.

Этот слой ОДИНАКОВ в симуляции и на железе. Различаются ровно две вещи — имена
топиков камеры и use_sim_time, — и обе берутся из
`fleet_comms.mode_profiles.as_launch_args`, а не пишутся здесь заново. В этом
весь смысл модуля профилей: пока дельта sim/железо жила в четырёх местах, эти
места уже успели разъехаться (см. докстроку mode_profiles.py).

ЖЁСТКОЕ ПРАВИЛО РЕЖИМА HARDWARE
===============================
**НИ ОДИН узел этого файла не имеет права подписываться на `/camera/camera/*`.**
Только на `/camera_edge/*`. Каждая прямая подписка на Pi-топик заново открывает
СОБСТВЕННЫЙ поток по Wi-Fi (`edge_bringup.launch.py:29-30`) и возвращает тот
самый веер потоков, ради устранения которого сделано реле единственного ingest.
Профиль режима (`mode_profiles.PROFILES[HARDWARE]:138-140`) обеспечивает это
автоматически — но если сюда когда-нибудь добавят узел с топиком, набранным
руками, инвариант рухнет молча.

БЕЗОПАСНОСТЬ КРЕДОВ
===================
`VLM_API_KEY` передаётся дочерним процессам ТОЛЬКО через окружение и никогда:
  * аргументом командной строки — его видно в `ps` любому пользователю хоста;
  * ROS-параметром — `ros2 param get` доступен кому угодно в том же графе.
Наружу (в лог) уходит максимум `token: задан|НЕ задан` через
`fleet_comms.vlm_env.public_view` — единственную функцию, которой это разрешено.

ПРИМЕРЫ
-------
    ros2 launch ar_project edge_layer.launch.py mode:=sim planner:=vlm
    ros2 launch ar_project edge_layer.launch.py mode:=hardware planner:=vlm \\
        vlm_env_file:=~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env
"""
import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from fleet_comms.mode_profiles import HARDWARE, MODES, PLANNERS, as_launch_args, profile_for
from fleet_comms.vlm_env import load_env_file, public_view

# Пакеты, чьи site-packages нужны дочернему интерпретатору из venv: он не видит
# оверлей ROS-воркспейса, потому что запускается не через `ros2 run`.
EDGE_PYTHON_PACKAGES = (
    'object_tracking',
    'planner_orchestrator',
    'object_tracking_msgs',
    'ar_project_msgs',
    'fleet_comms',
)


def _arg(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _bool_value(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _site_packages_for(package_name):
    prefix = get_package_prefix(package_name)
    return os.path.join(
        prefix,
        'lib',
        f'python{sys.version_info.major}.{sys.version_info.minor}',
        'site-packages',
    )


def _resolve_vlm_env_path(raw):
    """Путь к vlm.env: аргумент, иначе VLM_ENV_FILE, иначе пусто.

    Пусто — штатное состояние, а не ошибка: без кредов оркестратор работает с
    MockVlmClient (`vlm_client.py`, make_client). Именно поэтому ниже стоит
    отдельное громкое предупреждение — тихий уход в mock оператор замечает уже
    по странным результатам прогона.
    """
    raw = (raw or '').strip() or os.environ.get('VLM_ENV_FILE', '').strip()
    return os.path.expanduser(raw) if raw else ''


def _edge_env(vlm_env_path):
    env = dict(os.environ)
    env['PYTHONUNBUFFERED'] = '1'
    env.setdefault('HF_HOME', os.path.expanduser('~/.cache/huggingface'))
    env.setdefault('HF_HUB_DISABLE_PROGRESS_BARS', '1')
    env.setdefault('TRANSFORMERS_VERBOSITY', 'error')

    package_paths = [_site_packages_for(name) for name in EDGE_PYTHON_PACKAGES]
    env['PYTHONPATH'] = os.pathsep.join(
        path for path in package_paths + [env.get('PYTHONPATH', '')] if path
    )

    # Креды кладутся В ОКРУЖЕНИЕ дочернего процесса и больше никуда. Значения
    # файла ПЕРЕКРЫВАЮТ унаследованные переменные: оператор, который только что
    # правил vlm.env через консоль, вправе ожидать, что подействует файл, а не
    # то, что когда-то экспортировала оболочка.
    if vlm_env_path and os.path.isfile(vlm_env_path):
        env.update(load_env_file(vlm_env_path))
    return env


def _credentials_report(env, planner):
    """LogInfo о кредах: адрес и модель — можно, значение ключа — никогда."""
    view = public_view(env)
    actions = [LogInfo(msg='[edge_layer] VLM: base_url=%s, model=%s, token=%s'
                           % (view['base_url'] or '(пусто)',
                              view['model'] or '(пусто)',
                              'задан' if view['token_set'] else 'НЕ задан'))]
    if planner == 'vlm' and not view['base_url']:
        # make_client (planner_orchestrator/vlm_client.py) при пустом base_url
        # возвращает MockVlmClient БЕЗ ошибки и без предупреждения. Прогон
        # выглядит нормальным, но планирует не VLM, а детерминированная заглушка,
        # и в отчёт уходит не тот эксперимент.
        actions.append(LogInfo(msg=(
            '[edge_layer] ВНИМАНИЕ: planner:=vlm, но VLM_BASE_URL пуст. '
            'planner_orchestrator.vlm_client.make_client в этом случае МОЛЧА '
            'вернёт MockVlmClient — миссию будет планировать заглушка, а не VLM. '
            'Укажите vlm_env_file:=<путь к vlm.env> либо экспортируйте '
            'VLM_BASE_URL/VLM_API_KEY/VLM_MODEL перед запуском.')))
    return actions


def _slam_and_relay(context, mode, launch_dir):
    """Только hardware: реле камеры + RTAB-Map + /map_odom_correction.

    В симуляции возвращает []: RTAB-Map там уже внутри flat_sim_bringup
    (:114-122), а реле камеры не нужно вовсе — всё на одном хосте, копировать
    поток через сжатие незачем. Это же записано в профиле как
    starts_slam_in_robot_layer.
    """
    if mode != HARDWARE:
        return []
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'edge_bringup.launch.py')),
        launch_arguments={
            'start_slam': 'true',
            'database_path': os.path.expanduser(_arg(context, 'slam_database_path')),
            'delete_db_on_start': _arg(context, 'slam_delete_db_on_start'),
            # map->odom в /tf владеет Pi-шный map_odom_relay; вторая рука на том
            # же трансформе даёт дёргающийся TF (edge_bringup.launch.py:100-104).
            'publish_tf_map': 'false',
            'start_map_odom_correction': 'true',
            # Дашборд поднимаем МЫ, ниже: он обязан быть один и одинаковый в
            # обоих режимах, иначе порт 8088 займут два разных сервера.
            'start_dashboard': 'false',
        }.items(),
    )]


def _edge_processes(context, mode, planner, args):
    """Детектор и оркестратор — как ExecuteProcess из venv, а не Node.

    Оба нуждаются в torch/cv2, которых нет в системном python ROS; рабочий приём
    целиком скопирован из vlm_sim_bringup.launch.py:66-134 (в том числе ступени
    6 с и 34 с) — это единственная связка, реально проверенная на VLM-прогонах.
    """
    start_detector = _bool_value(_arg(context, 'start_detector')) and \
        _bool_value(args['start_detector'])
    start_orchestrator = _bool_value(_arg(context, 'start_orchestrator')) and \
        _bool_value(args['start_orchestrator'])
    if not (start_detector or start_orchestrator):
        # planner=flat: FLAT-цикл живёт на роботе, edge-узлы ему не нужны, и
        # отсутствие venv в этом случае не должно валить запуск.
        return [LogInfo(msg='[edge_layer] planner:=%s — детектор и оркестратор не '
                            'запускаются, поднят только дашборд.' % (planner,))]

    venv_python = os.path.expanduser(_arg(context, 'venv_python'))
    if not os.path.isfile(venv_python):
        # Fail-loud ровно как в vlm_sim_bringup.launch.py:70-75: молчаливый
        # пропуск edge-узлов выглядит как «стек поднялся», а миссия потом просто
        # не стартует, и причина ищется в VLM.
        raise RuntimeError(
            'edge_layer: интерпретатор ML-окружения не найден: %s. Передайте '
            'venv_python:=/путь/к/python (в контейнере обычно '
            '/opt/ot_venv/bin/python) или отключите узлы: start_detector:=false '
            'start_orchestrator:=false.' % (venv_python,))

    vlm_env_path = _resolve_vlm_env_path(_arg(context, 'vlm_env_file'))
    if vlm_env_path and not os.path.isfile(vlm_env_path):
        raise RuntimeError(
            'edge_layer: файл кредов не найден: %s. Создайте его из '
            'planner_orchestrator/vlm.env.example либо уберите vlm_env_file/'
            'VLM_ENV_FILE, чтобы креды брались из окружения.' % (vlm_env_path,))
    env = _edge_env(vlm_env_path)

    actions = _credentials_report(env, planner)

    if start_detector:
        actions.append(TimerAction(period=float(_arg(context, 'detector_start_delay_s')), actions=[
            ExecuteProcess(
                cmd=[
                    venv_python, '-m', 'object_tracking.detect_target_server',
                    '--ros-args',
                    '-p', f"use_sim_time:={args['use_sim_time']}",
                    '-p', f"image_topic:={args['detector_image_topic']}",
                    '-p', f"depth_topic:={args['detector_depth_topic']}",
                    # Реле на edge уже отдаёт raw, а в sim сжатия нет вовсе:
                    # включённый compressed открыл бы вторую подписку к Pi.
                    '-p', f"use_compressed_input:={args['detector_use_compressed_input']}",
                    '-p', f"model_mode:={args['detector_model_mode']}",
                    '-p', 'depth_point_strategy:=nearest_mask',
                    '-p', f"target_conf_default:={_arg(context, 'target_detect_conf')}",
                    '-p', f"vocab_conf_default:={_arg(context, 'detect_all_conf')}",
                ],
                env=env,
                name='detect_target_server',
                output='screen',
            )]))

    if start_orchestrator:
        # ЗАПРЕЩЕНО дописывать сюда '-p vlm_api_key:=...' и '-p vlm_base_url:=...'
        # с реальным значением: аргументы процесса видны в `ps` всем на хосте, а
        # ROS-параметры читаются любым узлом графа. Креды идут только через env
        # выше. vlm_model — не секрет, его передавать можно.
        cmd = [
            venv_python, '-m', 'planner_orchestrator.orchestrator_node',
            '--ros-args',
            '-p', f"use_sim_time:={args['use_sim_time']}",
            '-p', f"use_mock:={args['use_mock']}",
            '-p', f"camera_image_topic:={args['orchestrator_camera_image_topic']}",
            '-p', f"max_steps:={_arg(context, 'max_steps')}",
            '-p', f"replan_every_n:={_arg(context, 'replan_every_n')}",
            '-p', f"vlm_timeout_s:={_arg(context, 'vlm_timeout_s')}",
            '-p', f"async_replan:={_arg(context, 'async_replan')}",
            '-p', f"send_map:={_arg(context, 'send_map')}",
            '-p', 'motion_fallback_frame:=odom',
            '-p', f"detect_conf:={_arg(context, 'detect_conf')}",
            '-p', f"target_detect_conf:={_arg(context, 'target_detect_conf')}",
            '-p', f"detect_all_conf:={_arg(context, 'detect_all_conf')}",
            # Комнаты мира: планировщик подписывает их на карте, которую видит
            # модель. Пусто -> подписей нет, карта остаётся чистой SLAM-сеткой.
            '-p', f"rooms_spec:={_arg(context, 'rooms_spec')}",
            # Точка старта = начало кадра `map` у SLAM. Комнаты приходят в
            # МИРОВЫХ координатах, и без этого сдвига их подписи ложились мимо
            # плана здания ровно на вектор спавна (в house — на 7 метров).
            '-p', f"rooms_origin_x:={_arg(context, 'spawn_x')}",
            '-p', f"rooms_origin_y:={_arg(context, 'spawn_y')}",
            '-p', f"rooms_origin_yaw:={_arg(context, 'spawn_yaw')}",
            '-p', f"detect_memory_conf:={_arg(context, 'detect_memory_conf')}",
        ]
        vlm_model = _arg(context, 'vlm_model')
        if vlm_model:
            cmd += ['-p', f'vlm_model:={vlm_model}']
        actions.append(TimerAction(
            period=float(_arg(context, 'orchestrator_start_delay_s')),
            actions=[ExecuteProcess(cmd=cmd, env=env, name='planner_orchestrator',
                                    output='screen')]))
    return actions


def _edge_layer(context, *a, **k):
    launch_dir = os.path.join(get_package_share_directory('ar_project'), 'launch')

    mode = _arg(context, 'mode').lower()
    planner = _arg(context, 'planner').lower()
    try:
        profile_for(mode)
        args = as_launch_args(mode, planner)
    except ValueError as exc:
        # profile_for/planner_requirements уже дают русский текст; здесь только
        # добавляется, КТО именно упал, — иначе в логе десятка include-ов не
        # видно источника.
        raise RuntimeError(
            'edge_layer: %s. Допустимые значения: mode:=%s, planner:=%s.'
            % (exc, ' | '.join(MODES), ' | '.join(PLANNERS)))

    actions = [
        LogInfo(msg='[edge_layer] режим %s, планировщик %s, камера %s'
                    % (mode, planner, args['detector_image_topic'])),
    ]
    actions += _slam_and_relay(context, mode, launch_dir)

    if _bool_value(_arg(context, 'start_dashboard')):
        actions.append(Node(
            package='fleet_comms',
            executable='mission_dashboard',
            name='mission_dashboard',
            parameters=[{
                'port': int(_arg(context, 'dashboard_port')),
                # 0.0.0.0 внутри контейнера обязателен, иначе порт не виден
                # снаружи; ограничение доступа делает публикация docker на
                # 127.0.0.1, а не сам сервер (DECISIONS §1.4).
                'bind': '0.0.0.0',
                'use_sim_time': _bool_value(args['use_sim_time']),
                # Режим нужен дашборду, чтобы отличать «компонент отказал» от
                # «в этом режиме такого источника нет» (advisory_health).
                'mode': mode,
            }],
            output='screen',
        ))

    # Агрегатор здоровья по умолчанию НЕ здесь: в sim его поднимает
    # mission_bringup, на железе он уже внутри hardware_bringup.launch.py:194.
    # Флаг оставлен для запуска edge_layer в одиночку.
    if _bool_value(_arg(context, 'start_health_aggregator')):
        actions.append(Node(
            package='search_coordinator',
            executable='robot_health_aggregator',
            name='robot_health_aggregator',
            parameters=[{'use_sim_time': _bool_value(args['use_sim_time'])}],
            output='screen',
        ))

    actions += _edge_processes(context, mode, planner, args)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode', default_value='sim',
            description='sim | hardware. Влияет ровно на две вещи: имена топиков камеры и '
                        'use_sim_time; на железе дополнительно поднимается edge_bringup '
                        '(реле камеры + RTAB-Map).'),
        DeclareLaunchArgument(
            'planner', default_value='vlm',
            description='vlm | flat | mock. flat не поднимает ни детектор, ни оркестратор.'),
        DeclareLaunchArgument(
            'start_dashboard', default_value='true',
            description='Поднять mission_dashboard. Он один и тот же в обоих режимах, поэтому '
                        'edge_bringup включается с start_dashboard:=false.'),
        DeclareLaunchArgument(
            'dashboard_port', default_value='8088',
            description='HTTP-порт mission_dashboard.'),
        DeclareLaunchArgument(
            'start_detector', default_value='true',
            description='Разрешить запуск detect_target_server. Итоговое решение — И с '
                        'требованиями планировщика: при planner:=flat детектор не нужен.'),
        DeclareLaunchArgument(
            'start_orchestrator', default_value='true',
            description='Разрешить запуск planner_orchestrator. См. start_detector.'),
        DeclareLaunchArgument(
            'start_health_aggregator', default_value='false',
            description='По умолчанию false: в sim агрегатор поднимает mission_bringup, на '
                        'железе он уже внутри hardware_bringup. Два экземпляра публиковали бы '
                        'в /robot_health наперегонки.'),
        DeclareLaunchArgument(
            'venv_python', default_value='~/.venvs/ros-jazzy-ml/bin/python',
            description='Интерпретатор с torch/cv2/rclpy. Отсутствие файла — фатальная ошибка '
                        'с русским текстом, а не тихий пропуск узлов.'),
        DeclareLaunchArgument(
            'vlm_env_file', default_value='',
            description='Путь к vlm.env. Пусто = взять из VLM_ENV_FILE, иначе не загружать '
                        '(тогда креды берутся из окружения). Значение ключа не печатается '
                        'нигде и никогда — в лог уходит только «задан/НЕ задан».'),
        DeclareLaunchArgument(
            'vlm_model', default_value='',
            description='Переопределяет VLM_MODEL. Передаётся ROS-параметром: имя модели '
                        'секретом не является, в отличие от ключа.'),
        DeclareLaunchArgument(
            'max_steps', default_value='40',
            description='Потолок шагов миссии.'),
        DeclareLaunchArgument(
            'replan_every_n', default_value='1',
            description='Период перепланирования в шагах.'),
        DeclareLaunchArgument(
            'rooms_spec', default_value='',
            description='Комнаты мира: имя|x0,x1,y0,y1;имя|... — подписи на '
                        'для подписей на карте планировщика. АПРИОРНОЕ знание: '
                        'робот его не выводит, оно берётся из worlds.yaml.'),
        DeclareLaunchArgument(
            'spawn_x', default_value='0.0',
            description='Точка старта робота по X. Здесь она нужна как начало '
                        'кадра `map`, чтобы комнаты из rooms_spec (мировые '
                        'координаты) легли на карту SLAM без сдвига.'),
        DeclareLaunchArgument(
            'spawn_y', default_value='0.0',
            description='Точка старта робота по Y (см. spawn_x).'),
        DeclareLaunchArgument(
            'spawn_yaw', default_value='0.0',
            description='Курс робота на старте (см. spawn_x).'),
        DeclareLaunchArgument(
            'detect_memory_conf', default_value='0.55',
            description='Порог уверенности, с которого детекция ОТМЕЧАЕТСЯ на карте '
                        'планировщика как найденный предмет. Высокий намеренно: '
                        'отметка живёт до конца миссии, и модель ей верит, поэтому '
                        'ложная дороже пропущенной. Осторожно с симуляцией: цели в '
                        'house — плоские билборды, YOLOE даёт по ним 0.26..0.40, и '
                        'при 0.55 они на карту не попадут вовсе. Понижайте этим '
                        'аргументом, а не правкой кода.'),
        DeclareLaunchArgument(
            'vlm_timeout_s', default_value='30.0',
            description='Таймаут запроса к VLM, с.'),
        DeclareLaunchArgument(
            'async_replan', default_value='false',
            description='Перепланировать параллельно исполнению шага.'),
        DeclareLaunchArgument(
            'send_map', default_value='true',
            description='Прикладывать к запросу VLM миниатюру карты.'),
        DeclareLaunchArgument(
            'target_detect_conf', default_value='0.50',
            description='Порог уверенности по целевому классу (детектор и оркестратор).'),
        DeclareLaunchArgument(
            'detect_all_conf', default_value='0.12',
            description='Порог уверенности при открытом словаре.'),
        DeclareLaunchArgument(
            'detect_conf', default_value='0.0',
            description='Единый устаревший порог оркестратора; 0.0 = не переопределять.'),
        DeclareLaunchArgument(
            'detector_start_delay_s', default_value='6.0',
            description='Задержка старта детектора, с. 6 с — из проверенного '
                        'vlm_sim_bringup.launch.py:82.'),
        DeclareLaunchArgument(
            'orchestrator_start_delay_s', default_value='34.0',
            description='Задержка старта оркестратора, с. 34 с — из проверенного '
                        'vlm_sim_bringup.launch.py:105: он ждёт серверы навыков executive.'),
        DeclareLaunchArgument(
            'slam_database_path', default_value='~/.ros/rtabmap_rgbd.db',
            description='База RTAB-Map. Действует только при mode=hardware.'),
        DeclareLaunchArgument(
            'slam_delete_db_on_start', default_value='true',
            description='Удалять базу RTAB-Map при старте. Действует только при mode=hardware.'),
        OpaqueFunction(function=_edge_layer),
    ])
