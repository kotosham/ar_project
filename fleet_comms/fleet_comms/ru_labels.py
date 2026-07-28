"""Русские названия компонентов и уровней — один словарь на весь стек.

До этого модуля таблица «`control_epos4` -> Приводы EPOS4/CAN» жила внутри
JS-константы страницы `mission_dashboard.py`. Как только рядом появился второй
потребитель тех же строк `/robot_health` (консоль оператора), копия неизбежно
начала бы отставать: имена компонентов задаются в
`search_coordinator/robot_health_aggregator.py` (пробы + HEARTBEAT_ROSTER +
NODE_ROSTER), и добавление там новой строки должно приводить к появлению
подписи в ОБОИХ интерфейсах, а не в одном.

Ключи обязаны совпадать с `DiagnosticStatus.name`, который публикует
`robot_health_aggregator`.
"""

# Компоненты /robot_health и /heartbeat.
COMPONENT_RU = {
    'realsense': 'Камера RealSense',
    'ekf_odometry': 'EKF (одометрия)',
    'scan': 'Лазер-скан (/scan)',
    'control_epos4': 'Приводы EPOS4/CAN',
    'wheel_odometry': 'Колёсная одометрия',
    'slam_correction': 'SLAM-коррекция map→odom',
    'slam_map': 'SLAM-карта (/map)',
    'detection_stream': 'Детекции (/target_pixel)',
    'cmd_vel': 'Команды движения',
    'search_coordinator': 'Executive FSM (Pi)',
    'planner_orchestrator': 'VLM-оркестратор',
    'detector': 'Детектор YOLOE',
    'nav2': 'Nav2 (навигация)',
    'twist_mux': 'Twist mux',
    'collision_monitor': 'Collision Monitor',
    'cmd_vel_watchdog': 'Watchdog cmd_vel',
    'slam_rtabmap': 'RTAB-Map SLAM (процесс)',
}

# Уровни diagnostic_msgs/DiagnosticStatus: OK=0, WARN=1, ERROR=2, STALE=3.
LEVEL_RU = {
    0: 'OK',
    1: 'ВНИМАНИЕ',
    2: 'ОТКАЗ',
    3: 'НЕТ ДАННЫХ',
}


def component_ru(name):
    """Подпись компонента; неизвестный компонент показывается как есть —
    молча прятать новую строку /robot_health хуже, чем показать её по-английски."""
    return COMPONENT_RU.get(name, name)


def level_ru(level):
    try:
        return LEVEL_RU.get(int(level), str(level))
    except (TypeError, ValueError):
        return str(level)
