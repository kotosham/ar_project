"""Верхнеуровневый запуск house-бенчмарка (квартира + сценарий + эпизод).

ЧТО ЭТО
=======
`house.sdf` — статичная двухкомнатная-плюс квартира 15 x 10 м с коридором длиной
14.8 м, пятью комнатами (bedroom, bathroom, kitchen, storage, living) и восемью
точечными светильниками, которые можно гасить по одному. Поверх этого мира
"сценарий" (`config/scenarios/<id>.yaml`) задаёт: стартовую позу робота, набор
дополнительных объектов (цель, ширма-окклюдер, таблички, ковёр, лужа), какие
светильники выключить, миссию для `/vlm_mission` и УПОРЯДОЧЕННЫЙ список
оракульных подцелей, по которым `house_scenario_runner.py` считает
OrderedProgress и итоговый успех.

Смысл бенчмарка — не "доехал / не доехал", а ДИАГНОСТИКА: каждый сценарий
изолирует одну конкретную причину отказа связки VLM-планировщик + FLAT-стек.
Поэтому запуск всегда параметризуется парой (scenario, perturbation): сценарий
описывает задачу, возмущение (`config/scenarios/perturbations/<pid>.yaml`) —
деградацию условий (темнота, грязный объектив, шум, смаз), которая
накладывается сверху и не меняет критерии успеха. Одна и та же задача при
`p_none` и при `p_dark_room` даёт две точки, разница между которыми и есть
измеряемая устойчивость.

СЦЕНАРИИ (mission -> что диагностирует)
=======================================
  s1_far_target       chair. Стул в 12.6 м по коридору: дальше предела
                      depth/scan (8 м), но в цвете виден (RGB far clip 30 м).
                      Декомпозиция большого подхода на несколько шагов вперёд
                      при отсутствии дальномерных данных о цели.
  s2_occlusion        bottle. Бутылка в гостиной за ширмой `occluder_screen`,
                      ровно на линии взгляда от дверного проёма. Смена ракурса
                      ради grounding'а, а не кружение на месте.
  s3_behind           sports ball. Мяч в 6.2 м ПОЗАДИ, приманка — впереди.
                      Активный поиск и поворот до поездки; устойчивость к
                      жадному "еду к тому, что вижу".
  s4_text_hint        cup. Кружка на кухне, ниоткуда не видна; в коридоре щит
                      "КРУЖКА — НА КУХНЕ / CUP -> KITCHEN". OCR и превращение
                      прочитанного в цель навигации.
  s5_arrow_signs      towel. Полотенце в ванной; маршрут размечен стрелками,
                      среди которых верный, но нерелевантный указатель
                      "КУХНЯ ->". Символьная разметка + устойчивость к
                      дистрактору.
  s6_toilet_implies_bathroom
                      bathroom. Цель — ПОМЕЩЕНИЕ, а не класс детектора.
                      DETECT_ALL вернёт "toilet"; планировщик обязан вывести,
                      что это и есть ванная.
  s7_neighbor_room    bathroom. Старт внутри кухни лицом в глухую перегородку;
                      ванная за ней, но пройти можно только через коридор.
                      Смежность комнат и выход до начала поиска.

ВОЗМУЩЕНИЯ
==========
  p_none  p_dark_room  p_dirty_lens  p_carpet  p_puddle  p_clutter  p_dropout
  p_hard (темнота + грязь + лужи одновременно)

ВАЖНО: id — это ровно имя файла `config/scenarios/<id>.yaml` (и
`config/scenarios/perturbations/<pid>.yaml`). Источник истины — то, что реально
установлено в share. Если передать несуществующий id, запуск падает СРАЗУ и
печатает полный список доступных сценариев. Полная таблица со стартовыми
позами, реквизитом и подцелями — в `config/scenarios/README.md`, разбор
метрик — в `docs/HOUSE_BENCHMARK.md`.

РАЗРЕШЕНИЕ КАМЕРЫ И ЕГО ЦЕНА
============================
Здесь по умолчанию `cam_width:=640 cam_height:=480`, тогда как во всех
остальных bring-up'ах 320x240. Причина: два OCR-сценария (s4, s5) требуют
читаемых табличек, а на 320x240 буквы с 2 м размазываются в кашу — сценарий
проваливается не из-за планировщика, а из-за пикселей.

Цена: пикселей в 4 раза больше, и это касается ОБОИХ сенсоров (RGB и depth
делят cam_width/cam_height, иначе развалится выравнивание depth->color).
В WSL2 программный OGRE2-рендер и так узкое место, так что 640x480 заметно
роняет RTF и частоту кадров, а RTAB-Map начинает отставать от одометрии.
Для сценариев без текста (s1, s2, s3, s6, s7) явно возвращайте
`cam_width:=320 cam_height:=240` — прогон пойдёт ощутимо быстрее и стабильнее,
а карта получится ровнее. НО: сравнивать между собой можно только эпизоды,
снятые при ОДНОМ разрешении — оно влияет и на детектор, и на VLM.

ПРИМЕРЫ ЗАПУСКА
===============
Быстрая проверка мира и сценария без VLM (mock-планировщик, окно Gazebo):

    ros2 launch ar_project house_sim.launch.py \
        scenario:=s1_far_target planner:=mock gui:=true \
        cam_width:=320 cam_height:=240

Полный прогон с VLM, без GUI, с записью эпизода:

    ros2 launch ar_project house_sim.launch.py \
        scenario:=s4_text_hint planner:=vlm start_edge:=true \
        gui:=false rviz:=false seed:=1 \
        out_dir:=~/ros2_ws/house_benchmark

Тот же сценарий, но с деградацией камеры (грязь + темнота + лужи):

    ros2 launch ar_project house_sim.launch.py \
        scenario:=s4_text_hint perturbation:=p_hard \
        perturb_camera:=true planner:=vlm gui:=false

Парная точка к предыдущей (та же задача, тот же seed, flat-базлайн вместо VLM) —
именно из таких пар считаются delta_progress и benefit/harm/neutral:

    ros2 launch ar_project house_sim.launch.py \
        scenario:=s4_text_hint perturbation:=p_hard planner:=mock \
        seed:=1 gui:=false

Только поднять квартиру в стартовой позе сценария и посмотреть глазами,
без запуска эпизода и без возмущений:

    ros2 launch ar_project house_sim.launch.py \
        scenario:=s6_toilet_implies_bathroom run_episode:=false \
        perturb_camera:=false gui:=true rviz:=true

Отчёт по накопленным прогонам строится отдельно:

    ros2 run ar_project house_benchmark_report.py --in ~/ros2_ws/house_benchmark
"""
import math
import os

import yaml

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

# Topic produced by sim_perturbations.py. When the perturbation node is in the
# loop BOTH consumers (detector and orchestrator) must read the degraded image,
# otherwise half the pipeline quietly keeps seeing a perfect picture and the
# experiment measures nothing.
PERTURBED_IMAGE_TOPIC = '/camera/camera/color/image_perturbed'
CLEAN_IMAGE_TOPIC = '/camera/camera/color/image_raw'


def _bool_value(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _available_scenarios(scenario_dir):
    if not os.path.isdir(scenario_dir):
        return []
    return sorted(
        os.path.splitext(name)[0]
        for name in os.listdir(scenario_dir)
        if name.endswith('.yaml') and os.path.isfile(os.path.join(scenario_dir, name))
    )


def _load_scenario(scenario_dir, scenario_id):
    """Read <scenario_dir>/<scenario_id>.yaml or die with a useful message.

    Failing loudly matters here: a typo in the id would otherwise silently spawn
    the robot at (0, 0) - inside the hallway but in the wrong place - and the
    whole episode would be scored against the wrong oracle.
    """
    # Accept either a bare id or a path to a YAML, exactly like house_scenario_runner.py's
    # `scenario` parameter does. Without this the launch file and the node it starts
    # disagree about what a valid scenario argument is, and a one-off variant in /tmp
    # (the natural way to try a tweak without touching the installed set) fails here while
    # working fine when the runner is started by hand.
    if scenario_id.endswith(('.yaml', '.yml')) or os.path.isabs(scenario_id) or os.sep in scenario_id:
        path = os.path.expanduser(scenario_id)
    else:
        path = os.path.join(scenario_dir, f'{scenario_id}.yaml')
    if not os.path.isfile(path):
        available = _available_scenarios(scenario_dir)
        listing = '\n  '.join(available) if available else '(none found)'
        raise RuntimeError(
            f'house_sim: scenario "{scenario_id}" not found.\n'
            f'Expected file: {path}\n'
            f'Available scenario ids in {scenario_dir}:\n  {listing}\n'
            'Pass one of the ids above as scenario:=<id>.'
        )
    with open(path, 'r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f'house_sim: {path} does not contain a YAML mapping.')
    return path, data


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('ar_project')
    launch_dir = os.path.join(pkg, 'launch')
    scenario_dir = os.path.join(pkg, 'config', 'scenarios')

    scenario_id = LaunchConfiguration('scenario').perform(context).strip()
    perturbation_id = LaunchConfiguration('perturbation').perform(context).strip()
    planner = LaunchConfiguration('planner').perform(context).strip().lower()
    seed = LaunchConfiguration('seed').perform(context).strip()
    perturb_camera = _bool_value(LaunchConfiguration('perturb_camera').perform(context))
    run_episode = _bool_value(LaunchConfiguration('run_episode').perform(context))
    out_dir = os.path.expanduser(LaunchConfiguration('out_dir').perform(context))
    runner_delay_s = float(LaunchConfiguration('runner_delay_s').perform(context))

    if planner not in ('vlm', 'mock'):
        raise RuntimeError(
            f'house_sim: planner:="{planner}" is not valid. Use planner:=vlm (real VLM) '
            'or planner:=mock (deterministic mock planner inside planner_orchestrator).'
        )
    # 'flat_mock' is the label the report groups mock runs under; keeping the
    # launch arg short ('mock') while the recorded label stays explicit.
    use_mock = 'true' if planner == 'mock' else 'false'
    planner_label = 'flat_mock' if planner == 'mock' else 'vlm'

    scenario_path, scenario = _load_scenario(scenario_dir, scenario_id)

    world = os.path.join(pkg, 'worlds', 'house.sdf')
    if not os.path.isfile(world):
        raise RuntimeError(
            f'house_sim: world not found: {world}. '
            'Build/install the ar_project package so worlds/house.sdf lands in share.'
        )

    # The spawn pose comes from the scenario, not from a launch argument: the
    # oracle subgoals are written relative to that start, so the two must never
    # drift apart. This also removes the need for the runner to teleport the
    # robot after the fact (teleport_robot:=false below).
    start = scenario.get('robot_start') or {}
    if not isinstance(start, dict):
        raise RuntimeError(f'house_sim: robot_start in {scenario_path} must be a mapping.')
    spawn_x = float(start.get('x', 0.0))
    spawn_y = float(start.get('y', 0.0))
    spawn_yaw = float(start.get('yaw', 0.0))

    image_topic = PERTURBED_IMAGE_TOPIC if perturb_camera else CLEAN_IMAGE_TOPIC

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'vlm_sim_bringup.launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'world': world,
            'gui': LaunchConfiguration('gui'),
            'rviz': LaunchConfiguration('rviz'),
            # The mission dashboard IS the "what is the planner thinking" view (Set-of-Mark
            # frame, the map the VLM was shown, the /vlm/activity trace). It defaulted to on
            # inside vlm_sim_bringup but was not reachable from here, so a batch sweep could
            # not turn it off and an interactive run could not be sure it was on.
            'start_monitor': LaunchConfiguration('start_monitor'),
            # The orchestrator's step budget. vlm_sim_bringup defaults it to 40, which was
            # sized for the one-room flat world; the house is 15 m long and every scenario
            # crosses at least one doorway. Measured on the first full sweep: s3..s7 all
            # ended on the 40-step ceiling having spent only ~90s of a 200-240s sim budget
            # and ~14m of a 45m path budget -- i.e. the ceiling, not the scenario, decided
            # the outcome, and the scenarios were unwinnable as configured.
            'max_steps': LaunchConfiguration('max_steps'),
            'start_edge': LaunchConfiguration('start_edge'),
            'venv_python': LaunchConfiguration('venv_python'),
            'use_mock': use_mock,
            'spawn_x': f'{spawn_x}',
            'spawn_y': f'{spawn_y}',
            'spawn_yaw': f'{spawn_yaw}',
            'cam_width': LaunchConfiguration('cam_width'),
            'cam_height': LaunchConfiguration('cam_height'),
            'detector_image_topic': image_topic,
            'orchestrator_camera_image_topic': image_topic,
        }.items(),
    )

    actions = [
        LogInfo(msg=(
            f'[house_sim] scenario={scenario_id} ({scenario.get("title", "no title")}) '
            f'perturbation={perturbation_id} planner={planner_label} seed={seed}'
        )),
        LogInfo(msg=(
            f'[house_sim] spawn x={spawn_x:.2f} y={spawn_y:.2f} '
            f'yaw={spawn_yaw:.2f} rad ({math.degrees(spawn_yaw):.0f} deg), '
            f'camera topic in use: {image_topic}'
        )),
        bringup,
    ]

    if perturb_camera:
        # Subscribes the clean RGB topic and republishes the degraded one; the
        # actual profile arrives later on /sim_perturbation/profile from the
        # scenario runner, so no profile parameters are set here.
        actions.append(Node(
            package='ar_project',
            executable='sim_perturbations.py',
            name='sim_perturbations',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ))

    if run_episode:
        # Delayed: the runner publishes /vlm_mission and immediately starts
        # scoring, so it must not start before Nav2, the detector and the
        # orchestrator are up (vlm_sim_bringup staggers those to ~34 s).
        actions.append(TimerAction(period=runner_delay_s, actions=[
            Node(
                package='ar_project',
                executable='house_scenario_runner.py',
                name='house_scenario_runner',
                output='screen',
                parameters=[{
                    'scenario': scenario_id,
                    'perturbation': perturbation_id,
                    'seed': int(seed),
                    'out_dir': out_dir,
                    'planner_label': planner_label,
                    # The robot was already spawned at the scenario pose above;
                    # a second teleport would only fight the physics settle.
                    'teleport_robot': False,
                    'use_sim_time': True,
                }],
            ),
        ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'scenario', default_value='s1_far_target',
            description='Scenario id = stem of config/scenarios/<id>.yaml. '
                        'An unknown id aborts the launch and prints the available ids.'),
        DeclareLaunchArgument(
            'perturbation', default_value='p_none',
            description='Perturbation overlay id = stem of '
                        'config/scenarios/perturbations/<id>.yaml. p_none is the clean baseline.'),
        DeclareLaunchArgument(
            'planner', default_value='vlm',
            description="'vlm' for the real VLM planner, 'mock' for the deterministic mock "
                        "(recorded as planner_label=flat_mock)."),
        DeclareLaunchArgument(
            'seed', default_value='0',
            description='Episode seed, recorded with the run and used by the runner for any '
                        'randomised choice.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='Show the Gazebo Sim window. Set false for batch runs.'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='Start RViz. Off by default: it competes with gz for the WSL2 GPU.'),
        DeclareLaunchArgument(
            'start_monitor', default_value='true',
            description='Start robot_health_aggregator + the mission dashboard on '
                        'http://localhost:8088 (component health, /mission/status, the VLM '
                        'activity trace, the Set-of-Mark frame and the map the VLM was shown). '
                        'Set false for headless sweeps to save a little CPU.'),
        DeclareLaunchArgument(
            'max_steps', default_value='120',
            description='Orchestrator step budget per mission. Raised from the flat-world '
                        'default of 40: the house needs ~30 steps just to cross the corridor, '
                        'and on the first sweep every multi-room scenario died on the ceiling '
                        'with most of its time and path budget unspent. The scenario YAML '
                        '(timeout_s, max_path_m) is meant to be the binding constraint, not '
                        'this. Lower it only to deliberately study step-limited behaviour.'),
        DeclareLaunchArgument(
            'start_edge', default_value='true',
            description='Start detect_target_server + planner_orchestrator from venv_python. '
                        'Needed for any real episode; set false to drive the stack by hand.'),
        DeclareLaunchArgument(
            'venv_python', default_value='~/.venvs/ros-jazzy-ml/bin/python',
            description='Python interpreter with torch/cv2/rclpy for the edge-side nodes.'),
        DeclareLaunchArgument(
            'perturb_camera', default_value='true',
            description='Run sim_perturbations.py and point BOTH the detector and the '
                        'orchestrator at /camera/camera/color/image_perturbed.'),
        DeclareLaunchArgument(
            'run_episode', default_value='true',
            description='Start house_scenario_runner.py. false = just bring the house up.'),
        DeclareLaunchArgument(
            'out_dir', default_value='~/ros2_ws/house_benchmark',
            description='Directory the runner writes episode records into.'),
        DeclareLaunchArgument(
            'runner_delay_s', default_value='45.0',
            description='Delay before the scenario runner starts [s]. Must exceed the slowest '
                        'stack layer (orchestrator at ~34 s) plus model load.'),
        # 640x480 here ONLY. See the module docstring: the two OCR scenarios need
        # legible signs, and every other bring-up stays at 320x240 because WSL2
        # gz rendering is the bottleneck.
        DeclareLaunchArgument(
            'cam_width', default_value='640',
            description='Sim RGB+depth width. 640 (not the usual 320) so the signs in the OCR '
                        'scenarios are readable; costs ~4x the render load.'),
        DeclareLaunchArgument(
            'cam_height', default_value='480',
            description='Sim RGB+depth height. Pair with cam_width to keep 4:3.'),
        OpaqueFunction(function=_setup),
    ])
