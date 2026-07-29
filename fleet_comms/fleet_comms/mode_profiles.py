"""Единственный источник истины по разнице между симуляцией и реальным роботом.

ЗАЧЕМ ЭТОТ МОДУЛЬ СУЩЕСТВУЕТ
============================
Дельта sim/железо сегодня размазана по четырём независимым местам, и они уже
разъехались:

  * имена топиков — `flat_sim_bringup.launch.py` даёт `/odom`, а железный путь
    `/odometry/filtered` (`hardware_bringup.launch.py:389`); камера в симуляции
    `/camera/camera/*`, а на edge ОБЯЗАНА быть `/camera_edge/*`
    (`edge_bringup.launch.py:29-30` — подписка edge-узла прямо на Pi-топик
    заново открывает поток по Wi-Fi);
  * разрешение камеры — один и тот же `camera_gazebo_sensors.xacro` рендерится
    как 320x240 из `flat_sim_bringup.launch.py:88` и как 640x480 из
    `house_sim.launch.py:382`, то есть URDF зависит от точки входа;
  * пороги свежести — `stale_s` проб в `robot_health_aggregator.py:125-155`,
    множитель 2.5 в `heartbeat.py:126`, дедлайны в `qos.py:50-81`;
  * список компонентов, которые обязаны быть живыми, не записан НИГДЕ — его
    держали в голове.

Пока эти четыре списка живут врозь, «единый интерфейс» неизбежно соврёт:
он будет проверять один набор топиков, а стек — публиковать другой. Здесь они
сведены в один словарь, который импортируют И launch-файлы, И правила
преflight-проверки консоли. Разъехаться они теперь физически не могут.

Модуль намеренно не импортирует ни rclpy, ни launch — он должен читаться
и юнит-тестироваться обычным python без ROS.
"""

SIM = 'sim'
HARDWARE = 'hardware'
MODES = (SIM, HARDWARE)

PLANNER_VLM = 'vlm'
PLANNER_FLAT = 'flat'
PLANNER_MOCK = 'mock'
PLANNERS = (PLANNER_VLM, PLANNER_FLAT, PLANNER_MOCK)


# Канонические аргументы xacro. Корневой launch передаёт их ВСЕГДА и явно,
# поэтому URDF перестаёт зависеть от того, каким файлом стек подняли.
#
# Почему два канона, а не один. Железная камера RealSense D435i поднимается в
# color-профиле 640x480 (`realsense_rgbd_pi.launch.py`), и для сопоставимости
# перцепции симуляция должна давать те же пиксели. Но репозиторий уже измерил
# цену этого в WSL2: `camera_gazebo_sensors.xacro:10-14` — «640x480@30 just
# overwhelms the pipeline (multi-second lag breaks the odom<->image sync ->
# crooked map)», а `house_sim.launch.py:69-76` — «640x480 заметно роняет RTF и
# частоту кадров, а RTAB-Map начинает отставать от одометрии». В Docker рендер
# ещё медленнее нативного WSL2. Поэтому дефолт симуляции — быстрый профиль, а
# тяжёлый включается сознательно галочкой «сопоставимо с железом», и оператор
# видит, чем платит.
URDF_CANON_SIM = {
    'cam_width': '320',
    'cam_height': '240',
    'cam_rate': '15',
    'cam_far': '30.0',
    'depth_far': '8.0',
}
URDF_CANON_HW = {
    'cam_width': '640',
    'cam_height': '480',
    'cam_rate': '15',
    'cam_far': '30.0',
    'depth_far': '8.0',
}

# Пороги возраста данных в секундах — единственное место, где они записаны.
# Взяты не с потолка: 'health' и 'heartbeat' — из robot_health_aggregator
# (пробы объявляют stale_s 1.5-5.0 с) и из heartbeat.py:126 (2.5 x период
# 0.5 с = 1.25 с, округлено вверх до 3.0 с, чтобы одиночная потеря пакета по
# Wi-Fi не красила индикатор). 'clock' жёстче остальных: если /clock встал,
# в симуляции встало ВСЁ, и знать об этом надо сразу.
FRESHNESS = {
    'health': 3.0,
    'heartbeat': 3.0,
    # Отдельный порог для симуляции, и вот почему он ВЧЕТВЕРО мягче.
    # Heartbeat публикуется таймером rclpy, а таймер не сработает, пока какой-то
    # питоновский колбэк того же процесса держит GIL. У детектора такой колбэк
    # есть: первый DETECT_ALL тянет текстовый энкодер YOLOE, и это ЗАМЕРЕНО как
    # ~8 с подряд без единого тика (журнал: STALE с 120.7 по 128.6 с, первая
    # детекция на 137.0 с). Отдельная callback-группа тут не спасает — она
    # решает конкуренцию очередей, а не GIL. На 3 с потребители объявляли живой
    # детектор мёртвым, и преflight консоли ронял готовый стек в «НЕ ГОТОВ»
    # прямо посреди работы.
    # На железе порог остаётся 3.0: там детектор живёт на edge-боксе отдельным
    # процессом, GIL общий с ним никто не делит, а вот реальную потерю связи
    # надо видеть быстро. Ровно та дельта sim/железо, ради которой существует
    # этот модуль.
    'heartbeat_sim': 12.0,
    'camera': 3.0,
    'scan': 3.0,
    'odom': 3.0,
    'clock': 2.0,
    'joint_states': 3.0,
    'map_correction': 10.0,
}

# Lifecycle-узлы Nav2 одинаковы в обоих режимах. robot_health_aggregator
# проверяет только ПРИСУТСТВИЕ этих узлов в графе (robot_health_aggregator.py:109),
# а узел может присутствовать в состоянии unconfigured и не навигировать —
# поэтому консоль дополнительно спрашивает их lifecycle-состояние.
NAV2_LIFECYCLE_NODES = ('planner_server', 'controller_server', 'bt_navigator')


PROFILES = {
    SIM: {
        'title_ru': 'Симуляция (Gazebo)',
        'use_sim_time': True,
        'odom_topic': '/odom',
        'camera_rgb_topic': '/camera/camera/color/image_raw',
        'camera_depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        'detector_use_compressed_input': False,
        'detector_model_mode': 'yoloe',
        # gz_bridge.yaml:73 отдаёт Twist на /diff_cont/cmd_vel_unstamped;
        # на железе ros2_control ждёт TwistStamped на /diff_cont/cmd_vel.
        'cmd_vel_final_topic': '/diff_cont/cmd_vel_unstamped',
        # flat_sim_bringup НЕ поднимает robot_health_aggregator (его добавляет
        # только vlm_sim_bringup через start_monitor), а на железе он уже внутри
        # hardware_bringup.launch.py:194 — иначе получим два экземпляра.
        'starts_health_aggregator': True,
        'starts_slam_in_robot_layer': True,
        # Обязательные строки /robot_health. В симуляции ekf_odometry и
        # wheel_odometry красные ВСЕГДА: gz_bridge.yaml не публикует ни
        # /odometry/filtered, ни /diff_cont/odom — EKF в sim не запускается.
        # Требовать их = никогда не дать зелёный свет.
        'required_health': ('realsense', 'scan', 'control_epos4', 'nav2',
                            'twist_mux', 'search_coordinator'),
        'advisory_health': ('ekf_odometry', 'wheel_odometry', 'slam_correction',
                            'detection_stream', 'cmd_vel', 'collision_monitor',
                            'cmd_vel_watchdog', 'slam_rtabmap'),
        # ЧЕСТНОСТЬ: в симуляции строка control_epos4 зелёная не потому, что
        # приводы живы, а потому что gz публикует /joint_states. Она НИЧЕГО не
        # доказывает про EPOS4/CAN, и интерфейс обязан это подписать.
        'misleading_health': {
            'control_epos4': 'в симуляции это /joint_states от Gazebo — '
                             'о реальных приводах EPOS4/CAN не говорит ничего',
        },
        'link_required_topics': ('/clock', '/scan', '/odom',
                                 '/camera/camera/color/camera_info'),
    },
    HARDWARE: {
        'title_ru': 'Реальный робот (Pi + edge)',
        'use_sim_time': False,
        'odom_topic': '/odometry/filtered',
        # ЕДИНСТВЕННЫЙ потребитель камеры Pi — реле на edge. Любой узел,
        # подписавшийся на /camera/camera/* напрямую, заново открывает поток по
        # Wi-Fi (edge_bringup.launch.py:29-30). Поэтому и детектор, и
        # оркестратор, и преflight-проверка консоли смотрят только на
        # /camera_edge/*.
        'camera_rgb_topic': '/camera_edge/color/image_raw',
        'camera_depth_topic': '/camera_edge/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera_edge/color/camera_info',
        'detector_use_compressed_input': False,
        'detector_model_mode': 'yoloe',
        'cmd_vel_final_topic': '/diff_cont/cmd_vel',
        'starts_health_aggregator': False,
        'starts_slam_in_robot_layer': False,
        'required_health': ('realsense', 'ekf_odometry', 'scan', 'control_epos4',
                            'wheel_odometry', 'nav2', 'twist_mux',
                            'search_coordinator'),
        'advisory_health': ('slam_correction', 'detection_stream', 'cmd_vel',
                            'collision_monitor', 'cmd_vel_watchdog',
                            'slam_rtabmap'),
        'misleading_health': {},
        # /robot_health публикуется НА PI (hardware_bringup.launch.py:194), поэтому
        # его приход сам по себе доказывает, что связь с роботом есть.
        # camera_info берётся с реле на edge, а не с Pi — см. выше про инвариант.
        'link_required_topics': ('/robot_health', '/scan', '/odometry/filtered',
                                 '/joint_states', '/camera_edge/color/camera_info'),
    },
}


def profile_for(mode):
    """Копия профиля режима. Копия, а не ссылка: вызывающий не должен иметь
    возможности испортить глобальный словарь для всех остальных."""
    if mode not in PROFILES:
        raise ValueError('неизвестный режим: %r, ожидается sim|hardware' % (mode,))
    src = PROFILES[mode]
    out = {}
    for key, value in src.items():
        if isinstance(value, tuple):
            out[key] = tuple(value)
        elif isinstance(value, dict):
            out[key] = dict(value)
        else:
            out[key] = value
    out['nav2_lifecycle_nodes'] = tuple(NAV2_LIFECYCLE_NODES)
    return out


def planner_requirements(planner):
    """Что обязано работать ради выбранного планировщика.

    flat вообще не поднимает edge-слой: FLAT-цикл живёт на роботе и
    единственная его сетевая зависимость — детектор на фазе DETECT, а без
    оркестратора он полностью самодостаточен (docs/architecture/MODES.md §2.5).
    mock — это тот же оркестратор, но с MockVlmClient: узлы нужны, креды нет.
    """
    if planner not in PLANNERS:
        raise ValueError('неизвестный планировщик: %r, ожидается vlm|flat|mock'
                         % (planner,))
    return {
        'needs_detector': planner in (PLANNER_VLM, PLANNER_MOCK),
        'needs_orchestrator': planner in (PLANNER_VLM, PLANNER_MOCK),
        'needs_vlm_creds': planner == PLANNER_VLM,
    }


def required_health_for(mode, planner):
    """Строки /robot_health, без которых нельзя объявлять готовность."""
    rows = list(profile_for(mode)['required_health'])
    need = planner_requirements(planner)
    if need['needs_detector']:
        rows.append('detector')
    if need['needs_orchestrator']:
        rows.append('planner_orchestrator')
    return tuple(rows)


def urdf_canon(hw_parity=False):
    """Канонические аргументы xacro. hw_parity=True даёт разрешение железа
    ценой скорости рендера — см. комментарий к URDF_CANON_SIM."""
    return dict(URDF_CANON_HW if hw_parity else URDF_CANON_SIM)


def as_launch_args(mode, planner, world_file='', hw_parity=False):
    """Всё, что нужно launch-файлам, уже строками — именно эта функция делает
    launch-файлы тонкими и не даёт им завести собственную копию дельты."""
    profile = profile_for(mode)
    need = planner_requirements(planner)
    args = {
        'mode': mode,
        'planner': planner,
        'use_sim_time': _b(profile['use_sim_time']),
        'odom_topic': profile['odom_topic'],
        'detector_image_topic': profile['camera_rgb_topic'],
        'detector_depth_topic': profile['camera_depth_topic'],
        'orchestrator_camera_image_topic': profile['camera_rgb_topic'],
        'detector_use_compressed_input': _b(profile['detector_use_compressed_input']),
        'detector_model_mode': profile['detector_model_mode'],
        'use_mock': _b(planner == PLANNER_MOCK),
        'start_detector': _b(need['needs_detector']),
        'start_orchestrator': _b(need['needs_orchestrator']),
        'start_health_aggregator': _b(profile['starts_health_aggregator']),
    }
    args.update(urdf_canon(hw_parity))
    if world_file:
        args['world'] = world_file
    return args


def freshness(key):
    """Порог возраста в секундах. Неизвестный ключ — 3.0 с, тот же порядок,
    что и у остальных: лучше слегка консервативно, чем молча без порога."""
    return FRESHNESS.get(key, 3.0)


def _b(value):
    return 'true' if value else 'false'
