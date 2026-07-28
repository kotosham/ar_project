"""Ядро преflight консоли: snapshot -> вердикт готовности на русском языке.

ЗАЧЕМ ОТДЕЛЬНЫЙ ЧИСТЫЙ МОДУЛЬ
=============================
Всё, что здесь есть, — это функция от словаря. rclpy не импортируется, поэтому
правила проверяются обычным pytest без ROS, Gazebo и робота. Это не эстетика:
преflight — единственное место, которое решает, дать ли оператору нажать «Пуск»,
и ошибка в нём стоит либо ложного запрета, либо запуска в никуда. Такую логику
надо уметь прогонять на выдуманных словарях за миллисекунды.

ЧТО ЗДЕСЬ ПРИНЦИПИАЛЬНО
=======================
1. Формулировки КОНКРЕТНЫЕ. Не «ошибка связи», а «нет /scan уже 12 с — на Pi не
   поднят hardware_bringup, он публикует /scan из depth-изображения». Оператор
   должен узнать не факт беды, а следующее действие.
2. Разделение «не готов» и «ещё жду». Уровень `wait` в режиме реального робота
   означает «робот пока не подключился» — это нормальное состояние ожидания, а не
   поломка; красный экран в этот момент только пугает.
3. Советочные строки не блокируют. Множество ADVISORY берётся ЯВНО из
   `profile['advisory_health']`: в симуляции `ekf_odometry` и `wheel_odometry`
   красные ВСЕГДА (gz_bridge не публикует ни /odometry/filtered, ни
   /diff_cont/odom — EKF в sim не запускается), и наивное «все строки зелёные»
   не сложилось бы в симуляции никогда.
4. Зелёный индикатор, который ничего не доказывает, подписывается честно:
   `profile['misleading_health']` даёт текст для поля `note_ru` (в симуляции
   `control_epos4` зелёный из-за gz JointStatePublisher и о приводах EPOS4/CAN
   не говорит ничего).
"""
import os
import socket

from fleet_comms.mode_profiles import (
    HARDWARE,
    SIM,
    freshness,
    planner_requirements,
    profile_for,
    required_health_for,
)
from fleet_comms.ru_labels import component_ru

# Единый словарь русских подписей живёт в fleet_comms.ru_labels и используется
# И дашбордом, И консолью. Третьей копии здесь нет намеренно: имена компонентов
# задаются в search_coordinator/robot_health_aggregator.py, и новая строка должна
# появляться в обоих интерфейсах сразу.
ru_component = component_ru

LEVEL_OK = 'ok'
LEVEL_WARN = 'warn'
LEVEL_ERROR = 'error'
LEVEL_WAIT = 'wait'

PHASE_WAITING = 'waiting'
PHASE_PARTIAL = 'partial'
PHASE_CONNECTED = 'connected'

# Порт роутера zenoh. Зафиксирован в deploy/transport/transport_env.sh:12 и в
# zenoh_session_config.json5 — на edge слушает ровно он.
ZENOH_ROUTER_PORT = 7447

_LEVEL_FROM_DIAGNOSTIC = {0: LEVEL_OK, 1: LEVEL_WARN, 2: LEVEL_ERROR, 3: LEVEL_ERROR}

# Порог свежести для каждого «топика связи». Ключи — из FRESHNESS в mode_profiles,
# чтобы пороги не расползлись по файлам ещё раз.
_FRESHNESS_KEY = {
    '/clock': 'clock',
    '/scan': 'scan',
    '/odom': 'odom',
    '/odometry/filtered': 'odom',
    '/joint_states': 'joint_states',
    '/robot_health': 'health',
    '/camera/camera/color/camera_info': 'camera',
    '/camera_edge/color/camera_info': 'camera',
}

# Кто именно обязан публиковать топик. Без этого сообщение «нет /scan» бесполезно:
# оператор не знает, лезть ему на Pi, на edge или в Gazebo.
_TOPIC_SOURCE_RU = {
    '/clock': 'его публикует gz_bridge из Gazebo — значит, симулятор не запущен '
              'или встал',
    '/scan': 'на Pi должен быть поднят hardware_bringup — он публикует /scan из '
             'depth-изображения RealSense (depthimage_to_laserscan)',
    '/odom': 'в симуляции /odom публикует gz_bridge из плагина diff_drive',
    '/odometry/filtered': 'это выход EKF (robot_localization) на Pi',
    '/joint_states': 'это joint_state_broadcaster из ros2_control на Pi',
    '/robot_health': 'его публикует robot_health_aggregator на Pi '
                     '(hardware_bringup.launch.py:194)',
    '/camera/camera/color/camera_info': 'в симуляции это плагин камеры Gazebo',
    '/camera_edge/color/camera_info': 'это выход реле камеры на edge '
                                      '(edge_bringup); консоль намеренно смотрит '
                                      'на реле, а не на топик Pi — вторая '
                                      'подписка заново открыла бы поток по Wi-Fi',
}


class Check:
    """Одна строка преflight. Обычный класс, а не dataclass: нужен стабильный
    порядок полей в JSON и полная свобода в значениях по умолчанию."""

    __slots__ = ('id', 'title', 'level', 'message', 'hint', 'note_ru')

    def __init__(self, check_id, title, level, message, hint='', note_ru=''):
        self.id = check_id
        self.title = title
        self.level = level
        self.message = message
        self.hint = hint
        self.note_ru = note_ru

    def to_dict(self):
        return {
            'id': self.id,
            'title': self.title,
            'level': self.level,
            'message': self.message,
            'hint': self.hint,
            'note_ru': self.note_ru,
        }

    def __repr__(self):
        return 'Check(%r, %r)' % (self.id, self.level)


class Verdict:
    __slots__ = ('ready', 'link_phase', 'link_message', 'checks', 'blocking',
                 'topics')

    def __init__(self, ready, link_phase, link_message, checks, blocking, topics):
        self.ready = ready
        self.link_phase = link_phase
        self.link_message = link_message
        self.checks = tuple(checks)
        self.blocking = tuple(blocking)
        self.topics = dict(topics)

    def to_public(self):
        return {
            'ready': bool(self.ready),
            'link_phase': self.link_phase,
            'link_message': self.link_message,
            'checks': [c.to_dict() for c in self.checks],
            'blocking': list(self.blocking),
        }

    def link_public(self, mode):
        """Форма для GET /api/link — тот же вердикт, только про связь."""
        return {
            'phase': self.link_phase,
            'message': self.link_message,
            'topics': dict(self.topics),
            'mode': mode,
        }


# -- вспомогательное ----------------------------------------------------------

def _age(snapshot, topic):
    ages = snapshot.get('topic_ages') or {}
    value = ages.get(topic)
    if value is None:
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def _fresh(snapshot, topic, key=None):
    """(свежий ли, возраст). Возраст None означает «не приходило ни разу»."""
    age = _age(snapshot, topic)
    if age is None:
        return False, None
    limit = freshness(key or _FRESHNESS_KEY.get(topic, 'health'))
    return age <= limit, age


def _age_ru(age):
    if age is None:
        return 'не приходил ни разу'
    return 'молчит уже %.1f с' % age


def _topic_problem_ru(topic, age):
    return '%s: %s (%s)' % (topic, _age_ru(age),
                            _TOPIC_SOURCE_RU.get(topic, 'источник неизвестен'))


# -- отдельные проверки -------------------------------------------------------

def check_graph(snapshot):
    """Пустой ROS-граф — это не «нет робота», а «консоль вообще ни с кем не
    разговаривает». Разные причины и разные действия, поэтому отдельная строка."""
    nodes = snapshot.get('graph_nodes') or []
    domain = snapshot.get('ros_domain_id')
    rmw = snapshot.get('rmw') or 'по умолчанию'
    if len(nodes) <= 1:
        return Check(
            'graph', 'ROS-граф', LEVEL_ERROR,
            'Консоль не видит ни одного узла кроме себя. ROS_DOMAIN_ID = %s, '
            'транспорт = %s.' % (domain if domain is not None else 'не задан', rmw),
            hint='Под zenoh домен не при чём — важен адрес роутера '
                 '(ZENOH_CONFIG_OVERRIDE / connect/endpoints). Под Fast DDS, '
                 'наоборот, ROS_DOMAIN_ID обязан совпасть на всех хостах.')
    return Check('graph', 'ROS-граф', LEVEL_OK,
                 'Видно узлов: %d. Транспорт: %s.' % (len(nodes), rmw))


def check_router(mode, edge_host, tcp_probe):
    """TCP-connect на роутер zenoh. Только для режима реального робота.

    Это ПЕРВАЯ причина, по которой оператор не увидит робота, и она обязана
    называться прямо. Сессия поднимается в режиме client с выключенным scouting
    (deploy/transport/zenoh_session_config.json5:14,20-23), то есть узел делает
    ровно одно исходящее соединение на tcp/<edge>:7447 и, если там никто не
    слушает, не находит ничего вообще — при этом сообщение rmw об этом крайне
    невнятное, а ZENOH_ROUTER_CHECK_ATTEMPTS вместо ошибки просто блокирует
    создание узла (transport_env.sh:13-14).
    """
    if mode != HARDWARE:
        return Check('router', 'Роутер zenoh', LEVEL_OK,
                     'В симуляции роутер не нужен: все узлы в одном контейнере.')
    host = (edge_host or '').strip()
    if not host:
        return Check(
            'router', 'Роутер zenoh', LEVEL_WARN,
            'Адрес edge-бокса (EDGE_HOST) не задан, проверить роутер невозможно.',
            hint='EDGE_HOST задаётся в docker/.env и подставляется в '
                 'ZENOH_CONFIG_OVERRIDE; консоль его не редактирует.')
    ok, detail = tcp_probe(host, ZENOH_ROUTER_PORT)
    if ok:
        return Check('router', 'Роутер zenoh', LEVEL_OK,
                     'Роутер отвечает: tcp/%s:%d.' % (host, ZENOH_ROUTER_PORT))
    return Check(
        'router', 'Роутер zenoh', LEVEL_ERROR,
        'Роутер zenoh недоступен: tcp/%s:%d (%s). Пока он не поднят, консоль не '
        'увидит ни одного узла робота — сессия работает в режиме client и '
        'больше никуда не стучится.' % (host, ZENOH_ROUTER_PORT, detail),
        hint='На edge-боксе: ros2 run rmw_zenoh_cpp rmw_zenohd (или '
             'systemctl start zenoh-router). Проверьте также, что 7447 не '
             'закрыт файрволом и что EDGE_HOST указывает на нужный адрес.')


def check_link(snapshot, mode):
    """(проверка, фаза, сообщение).

    Фаза `connected` требует ИМЕННО свежести всех ключевых потоков, а не факта их
    присутствия в графе. Это и есть требование «сообщать об успешном подключении
    только когда робот реально виден»: узел может быть в графе и при этом ничего
    не публиковать (типовой случай — драйвер поднялся, а камера не открылась).
    """
    profile = profile_for(mode)
    topics = profile['link_required_topics']
    fresh_list = []
    stale_list = []
    ages = {}
    for topic in topics:
        ok, age = _fresh(snapshot, topic)
        ages[topic] = age
        (fresh_list if ok else stale_list).append((topic, age))

    if not stale_list:
        message = 'Подключено: все ключевые потоки свежие (%s).' % ', '.join(
            t for t, _ in fresh_list)
        return (Check('link', 'Связь с роботом', LEVEL_OK, message),
                PHASE_CONNECTED, message)

    # В симуляции стек поднимает сама консоль, поэтому молчание — это отказ.
    # На железе робот включается человеком отдельно, и ожидание нормально.
    waiting_level = LEVEL_WAIT if mode == HARDWARE else LEVEL_ERROR
    problems = '; '.join(_topic_problem_ru(t, a) for t, a in stale_list)

    if not fresh_list:
        message = ('Ожидаю подключения робота: не идёт ни один ключевой поток. %s'
                   % problems)
        hint = ('Проверьте питание и bringup на роботе; на железе также роутер '
                'zenoh на edge.' if mode == HARDWARE else
                'Нажмите «Запустить стек» и дождитесь Gazebo — /clock появляется '
                'первым.')
        return (Check('link', 'Связь с роботом', waiting_level, message, hint=hint),
                PHASE_WAITING, message)

    message = ('Связь частичная: идут %s. Не идут: %s'
               % (', '.join(t for t, _ in fresh_list), problems))
    return (Check('link', 'Связь с роботом', waiting_level, message),
            PHASE_PARTIAL, message)


def check_clock(snapshot, mode):
    """Только симуляция: если /clock встал, встало ВСЁ, что живёт с
    use_sim_time:=true, и это выглядит как «робот не едет», а не как «время»."""
    if mode != SIM:
        return None
    ok, age = _fresh(snapshot, '/clock', 'clock')
    if ok:
        return Check('clock', 'Симуляционное время', LEVEL_OK,
                     '/clock идёт (возраст %.1f с).' % age)
    return Check(
        'clock', 'Симуляционное время', LEVEL_ERROR,
        'Gazebo не публикует /clock (%s). Весь стек поднят с use_sim_time:=true '
        'и будет стоять: таймеры и TF не тикают.' % _age_ru(age),
        hint='Проверьте, что gz sim действительно запущен и не на паузе '
             '(в GUI — кнопка Play), и что gz_bridge поднят.')


def check_health(snapshot, mode, planner):
    """Строки /robot_health: обязательные — блокируют, советочные — нет."""
    checks = []
    profile = profile_for(mode)
    rows = snapshot.get('health_rows') or {}
    health_age = snapshot.get('health_age_s')
    misleading = profile.get('misleading_health') or {}

    if health_age is None:
        # Пока агрегатор не сказал ни слова, расписывать двенадцать красных строк
        # бессмысленно и страшно — это одно состояние «ещё не приехало».
        return [Check(
            'health', 'Здоровье робота', LEVEL_WAIT,
            '/robot_health ещё не приходил — по нему консоль и узнаёт состояние '
            'компонентов.',
            hint='В симуляции его поднимает vlm_sim_bringup (start_monitor), '
                 'на железе — hardware_bringup.launch.py:194.')]

    if float(health_age) > freshness('health'):
        checks.append(Check(
            'health', 'Здоровье робота', LEVEL_ERROR,
            '/robot_health устарел: последнее сообщение %.1f с назад (порог %.1f с) '
            '— агрегатор перестал публиковать.' % (health_age, freshness('health')),
            hint='Проверьте узел robot_health_aggregator: если он жив, а сообщений '
                 'нет, значит он завис на пробе.'))
    else:
        checks.append(Check('health', 'Здоровье робота', LEVEL_OK,
                            '/robot_health свежий (%.1f с).' % health_age))

    for name in required_health_for(mode, planner):
        checks.append(_health_row_check(name, rows.get(name), misleading,
                                        advisory=False))
    for name in profile.get('advisory_health') or ():
        if name in rows:
            checks.append(_health_row_check(name, rows.get(name), misleading,
                                            advisory=True))
    return checks


def _health_row_check(name, row, misleading, advisory):
    note = misleading.get(name, '')
    title = ru_component(name)
    check_id = 'health:%s' % name
    if not row:
        level = LEVEL_WARN if advisory else LEVEL_ERROR
        return Check(check_id, title, level,
                     '%s: строки нет в /robot_health — компонент не запущен либо '
                     'его проба не объявлена агрегатором.' % title,
                     hint='Список проб задаётся в '
                          'search_coordinator/robot_health_aggregator.py.',
                     note_ru=note)
    level = _LEVEL_FROM_DIAGNOSTIC.get(int(row.get('level', 3)), LEVEL_ERROR)
    if advisory and level == LEVEL_ERROR:
        # Советочная строка физически не может стать ошибкой: её красный цвет —
        # штатное состояние режима (см. ekf_odometry в симуляции).
        level = LEVEL_WARN
    message = '%s: %s' % (title, row.get('message') or 'без сообщения')
    age = row.get('age_s')
    if age is not None:
        try:
            message += ' (возраст %.1f с)' % float(age)
        except (TypeError, ValueError):
            pass
    return Check(check_id, title, level, message, note_ru=note)


def check_nav2(snapshot, mode):
    """Присутствие узла Nav2 в графе не значит, что он сконфигурирован.

    robot_health_aggregator.py:109 проверяет ровно присутствие, а lifecycle-узел
    может висеть в unconfigured и не навигировать — из-за этого «всё зелёное, а
    робот стоит». Поэтому спрашиваем состояние явно.
    """
    states = snapshot.get('nav2_lifecycle') or {}
    if not states or all(v == 'unknown' for v in states.values()):
        return Check(
            'nav2', 'Nav2', LEVEL_WAIT,
            'Nav2 ещё не отвечает на запрос состояния — серверы не подняты либо '
            'не видны в графе.',
            hint='Проверить руками: ros2 lifecycle get /planner_server')
    bad = ['%s=%s' % (node, state) for node, state in sorted(states.items())
           if state != 'active']
    if not bad:
        return Check('nav2', 'Nav2', LEVEL_OK,
                     'Все серверы Nav2 активны (%s).'
                     % ', '.join(sorted(states)))
    return Check(
        'nav2', 'Nav2', LEVEL_ERROR,
        'Nav2 не активирован: %s. Узлы в графе есть, но цели навигации они не '
        'примут.' % ', '.join(bad),
        hint='Смотрите lifecycle_manager: обычно он не смог дождаться карты '
             '(bond timeout) и остановил активацию всей цепочки.')


def check_planner_nodes(snapshot, planner):
    """Детектор и оркестратор — по heartbeat, а не по присутствию в графе:
    процесс может быть жив и при этом не считать (ML-venv не поднялся, модель не
    загрузилась). heartbeat даёт статус и возраст."""
    checks = []
    need = planner_requirements(planner)
    beats = snapshot.get('heartbeats') or {}
    wanted = []
    if need['needs_detector']:
        wanted.append(('detector', 'planner:detector'))
    if need['needs_orchestrator']:
        wanted.append(('planner_orchestrator', 'planner:orchestrator'))
    if not wanted:
        checks.append(Check(
            'planner:nodes', 'Планировщик', LEVEL_OK,
            'Планировщик FLAT работает на роботе: детектор и VLM-оркестратор для '
            'него не требуются.'))
        return checks

    limit = freshness('heartbeat')
    for name, check_id in wanted:
        title = ru_component(name)
        beat = beats.get(name)
        if not beat:
            checks.append(Check(
                check_id, title, LEVEL_ERROR,
                '%s: heartbeat не приходил — узел не запущен.' % title,
                hint='Верхний слой поднимается edge_layer; в симуляции это делает '
                     'та же кнопка «Запустить стек».'))
            continue
        age = float(beat.get('age_s', 1e9))
        status = str(beat.get('status', 'DOWN'))
        if age > limit:
            checks.append(Check(
                check_id, title, LEVEL_ERROR,
                '%s: heartbeat устарел (%.1f с при пороге %.1f с) — процесс жив, '
                'но не отчитывается.' % (title, age, limit)))
        elif status == 'OK':
            checks.append(Check(check_id, title, LEVEL_OK,
                                '%s: heartbeat OK (%.1f с).' % (title, age)))
        elif status == 'DEGRADED':
            checks.append(Check(
                check_id, title, LEVEL_WARN,
                '%s: работает в деградированном режиме (DEGRADED) — у оркестратора '
                'это означает открытый circuit breaker и откат в FLAT.' % title))
        else:
            checks.append(Check(check_id, title, LEVEL_ERROR,
                                '%s: статус %s.' % (title, status)))
    return checks


def check_vlm_creds(snapshot, planner):
    """Креды VLM. Только для planner=vlm.

    Пустой base_url блокирует ЖЁСТКО: `vlm_client.make_client` при отсутствии
    адреса молча возвращает MockVlmClient, и миссия «пойдёт», выдавая выдуманные
    планы. Молчаливый уход в mock — худший из возможных исходов, потому что
    выглядит он как успех.
    """
    if planner != 'vlm':
        return [Check('vlm_creds', 'Доступ к VLM', LEVEL_OK,
                      'Планировщик %s не обращается к VLM API — креды не нужны.'
                      % planner)]
    checks = []
    vlm = snapshot.get('vlm') or {}
    base_url = (vlm.get('base_url') or '').strip()
    model = (vlm.get('model') or '').strip()
    token_set = bool(vlm.get('token_set'))

    problems = []
    if not base_url:
        problems.append('не задан адрес API (VLM_BASE_URL) — без него оркестратор '
                        'молча уйдёт в offline-mock и будет выдавать выдуманные '
                        'планы')
    if not token_set:
        problems.append('не задан токен (VLM_API_KEY)')
    if not model:
        problems.append('не выбрана модель (VLM_MODEL)')

    if problems:
        checks.append(Check(
            'vlm_creds', 'Доступ к VLM', LEVEL_ERROR,
            'Креды VLM неполные: %s.' % '; '.join(problems),
            hint='Заполните шаг «VLM» в мастере. Значения пишутся в vlm.env; '
                 'токен обратно не показывается никогда и применяется только '
                 'после перезапуска стека.'))
    else:
        checks.append(Check('vlm_creds', 'Доступ к VLM', LEVEL_OK,
                            'Адрес, модель и токен заданы (%s, модель %s).'
                            % (base_url, model)))

    probe = vlm.get('probe')
    if not probe:
        checks.append(Check(
            'vlm_probe', 'Проверка связи с VLM', LEVEL_WARN,
            'Связь с VLM API не проверялась в этой сессии.',
            hint='Кнопка «Проверить связь» на шаге VLM: она делает GET /models и '
                 'один короткий запрос к модели.'))
    elif probe.get('ok'):
        checks.append(Check(
            'vlm_probe', 'Проверка связи с VLM', LEVEL_OK,
            'VLM API отвечает (%d мс), модель доступна.'
            % int(probe.get('latency_ms') or 0)))
    else:
        checks.append(Check(
            'vlm_probe', 'Проверка связи с VLM', LEVEL_ERROR,
            'Проверка связи с VLM не прошла: %s'
            % (probe.get('error_ru') or 'причина неизвестна'),
            hint=probe.get('hint_ru') or ''))
    return checks


def check_frontiers(snapshot, planner):
    """Советочная: в ограниченном мире SEARCH не стартует, пока карта не даёт
    фронтиров (RUNBOOK.md:346-349 — перед миссией карту «засевают» вращением на
    месте). Робот при этом стоит и выглядит исправным, что стоило не одного часа
    отладки.

    Проверяется по графу: наличие фронтиров из snapshot не видно (их нет в
    зафиксированной схеме), а вот отсутствие самого узла — видно и однозначно.
    """
    nodes = snapshot.get('graph_nodes') or []
    present = any('frontier_extractor' in str(n) for n in nodes)
    if not present:
        return Check(
            'frontiers', 'Фронтиры (исследование)', LEVEL_WARN,
            'Узел frontier_extractor не найден в графе — фаза SEARCH не получит '
            'ни одной цели и робот будет стоять на месте.',
            hint='В симуляции его поднимает flat_sim_bringup '
                 '(start_frontier_extractor:=true); на железе он по умолчанию '
                 'выключен и включается сознательно.')
    return Check(
        'frontiers', 'Фронтиры (исследование)', LEVEL_OK,
        'frontier_extractor в графе.',
        hint='Если робот всё равно стоит в SEARCH — карта ещё пуста: засейте '
             'карту вращением на месте (кнопка «Засеять карту»), иначе '
             'неизвестных клеток нет и фронтиров тоже.')


def check_mission_epoch(snapshot):
    """Рассинхрон эпохи миссии — реальная ловушка стека, а не гипотетическая:
    цели с чужой эпохой отбрасываются исполнителем как zombie, и внешне это
    выглядит как «оркестратор шлёт команды, робот их игнорирует»."""
    beats = snapshot.get('heartbeats') or {}
    epochs = {}
    for name, beat in beats.items():
        if not isinstance(beat, dict) or 'epoch' not in beat:
            continue
        try:
            epochs[name] = int(beat['epoch'])
        except (TypeError, ValueError):
            continue
    mission = snapshot.get('mission') or {}
    if 'mission_epoch' in mission:
        try:
            epochs['/mission/status'] = int(mission['mission_epoch'])
        except (TypeError, ValueError):
            pass

    distinct = set(epochs.values())
    if len(distinct) <= 1:
        shown = next(iter(distinct)) if distinct else 0
        return Check('mission_epoch', 'Эпоха миссии', LEVEL_OK,
                     'Эпоха миссии согласована: %d.' % shown)
    detail = ', '.join('%s=%d' % (k, v) for k, v in sorted(epochs.items()))
    return Check(
        'mission_epoch', 'Эпоха миссии', LEVEL_WARN,
        'Эпохи миссии разошлись (%s). Skill-цели с чужой эпохой отклоняются '
        'исполнителем как zombie: команды будут уходить, а робот — стоять.' % detail,
        hint='Лечится перезапуском отставшего узла (обычно оркестратора): эпоху '
             'штампует тот, кто начал миссию.')


def check_world(snapshot, mode):
    """Только симуляция: сломанный мир не загрузится, и Gazebo об этом скажет
    невнятно (чёрное окно вместо ошибки)."""
    if mode != SIM:
        return None
    world = snapshot.get('world') or {}
    world_id = world.get('id') or 'не выбран'
    status = world.get('status') or 'ok'
    note = world.get('status_note') or ''
    if status == 'broken':
        return Check('world', 'Мир симуляции', LEVEL_ERROR,
                     'Мир %s помечен как незагружаемый: %s' % (world_id, note),
                     hint='Выберите другой мир на шаге «Мир».')
    if status == 'limited':
        return Check('world', 'Мир симуляции', LEVEL_WARN,
                     'Мир %s пригоден с оговорками: %s' % (world_id, note))
    return Check('world', 'Мир симуляции', LEVEL_OK, 'Мир %s пригоден.' % world_id)


def check_catalog(snapshot):
    """По одной советочной строке на каждую проблему каталога миров."""
    checks = []
    for index, problem in enumerate(snapshot.get('catalog_problems') or []):
        checks.append(Check('catalog:%d' % index, 'Каталог миров', LEVEL_WARN,
                            str(problem)))
    return checks


# -- сборка вердикта ----------------------------------------------------------

def _default_tcp_probe(host, port, timeout_s=1.0):
    """TCP-connect с коротким таймаутом. Вынесен отдельно и инъектируем, чтобы
    evaluate() оставался тестируемым без сети."""
    try:
        with socket.create_connection((host, port), timeout=timeout_s):
            return True, 'соединение установлено'
    except socket.gaierror:
        return False, 'имя %s не разрешается' % host
    except (socket.timeout, TimeoutError):
        return False, 'таймаут соединения'
    except ConnectionRefusedError:
        return False, 'соединение отклонено — по адресу никто не слушает'
    except OSError as exc:
        return False, 'сеть недоступна (%s)' % exc.__class__.__name__


def evaluate(snapshot, mode, planner, *, now=None, edge_host=None, tcp_probe=None):
    """snapshot -> Verdict. Единственная точка входа преflight.

    `now` в схеме не используется: возрасты уже посчитаны построителем snapshot
    в console_node (там же, где живут монотонные метки прихода). Параметр
    сохранён в сигнатуре, потому что он зафиксирован интерфейсом.

    `edge_host` по умолчанию берётся из окружения: адрес edge-бокса задаётся в
    docker/.env при старте контейнера и консолью не редактируется.
    """
    del now  # см. докстроку: возрасты приходят готовыми
    profile = profile_for(mode)
    if edge_host is None:
        edge_host = os.environ.get('EDGE_HOST', '')
    if tcp_probe is None:
        tcp_probe = _default_tcp_probe

    link_check, phase, link_message = check_link(snapshot, mode)

    checks = [check_graph(snapshot),
              check_router(mode, edge_host, tcp_probe),
              link_check]
    for maybe in (check_clock(snapshot, mode),):
        if maybe is not None:
            checks.append(maybe)
    checks.extend(check_health(snapshot, mode, planner))
    checks.append(check_nav2(snapshot, mode))
    checks.extend(check_planner_nodes(snapshot, planner))
    checks.extend(check_vlm_creds(snapshot, planner))
    checks.append(check_frontiers(snapshot, planner))
    checks.append(check_mission_epoch(snapshot))
    world_check = check_world(snapshot, mode)
    if world_check is not None:
        checks.append(world_check)
    checks.extend(check_catalog(snapshot))

    # Множество советочных id задаётся ЯВНО. Без этого списка советочные строки
    # (в симуляции они красные всегда) заблокировали бы старт навсегда.
    advisory = set('health:%s' % name for name in profile.get('advisory_health') or ())
    advisory.update(('frontiers', 'mission_epoch'))
    advisory.update(c.id for c in checks if c.id.startswith('catalog:'))

    blocking = [c.id for c in checks
                if c.level in (LEVEL_ERROR, LEVEL_WAIT) and c.id not in advisory]

    topics = {}
    for topic in profile['link_required_topics']:
        topics[topic] = _age(snapshot, topic)

    return Verdict(ready=not blocking, link_phase=phase, link_message=link_message,
                   checks=checks, blocking=blocking, topics=topics)


def blocking_summary_ru(verdict):
    """Одна строка для сообщения об отказе запустить миссию: перечисляет ровно
    то, что мешает, а не абстрактное «стек не готов»."""
    by_id = dict((c.id, c) for c in verdict.checks)
    parts = []
    for check_id in verdict.blocking:
        check = by_id.get(check_id)
        if check is not None:
            parts.append('%s — %s' % (check.title, check.message))
    return '; '.join(parts) if parts else 'причина не определена'


# -- склейка вида миссии ------------------------------------------------------

def merge_mission_view(mission_status, activity):
    """Честный вид миссии из двух РАЗНЫХ источников.

    /mission/status публикует только SeekObjectServer, и в VLM-режиме он вечно
    IDLE: VLM-оркестратор ведёт свою миссию сам и рассказывает о ней в
    /vlm/activity (orchestrator_node.py:597,630,679). Если показывать только
    первый источник, оператор в VLM-режиме будет видеть «простой» во время
    работающей миссии.

    ВРЕМЕНА НЕ СМЕШИВАЮТСЯ: `/mission/status.stamp` — ROS-время (в симуляции это
    время Gazebo), а `/vlm/activity.stamp` — стенное `time.time()`. Здесь они не
    сравниваются между собой ни разу; VLM-состояние определяется исключительно
    ПОРЯДКОМ событий в списке.
    """
    events = [e for e in (activity or []) if isinstance(e, dict)]

    last_start = None
    last_end_index = -1
    last_start_index = -1
    for index, event in enumerate(events):
        name = event.get('event')
        if name == 'mission_start':
            last_start = event
            last_start_index = index
        elif name == 'mission_end':
            last_end_index = index

    if last_start is not None and last_start_index > last_end_index:
        step = None
        last_action = ''
        degraded = False
        outcome = ''
        for event in events[last_start_index:]:
            name = event.get('event')
            if name == 'degraded':
                degraded = True
            if name in ('step_start', 'step_result'):
                try:
                    value = int(event.get('step'))
                except (TypeError, ValueError):
                    value = None
                if value is not None:
                    step = value if step is None else max(step, value)
                if name == 'step_start':
                    last_action = str(event.get('action') or '')
            if name == 'plan_failed':
                outcome = 'план не построен: %s' % (event.get('error') or '')
        return {
            'source': 'vlm',
            'state': 'RUNNING',
            'instruction': str(last_start.get('target') or ''),
            'progress': None,
            'step': step,
            'last_action': last_action,
            'outcome': outcome,
            'degraded': degraded,
        }

    if isinstance(mission_status, dict) and str(mission_status.get('state') or 'IDLE') != 'IDLE':
        progress = mission_status.get('progress')
        try:
            progress = float(progress) if progress is not None else None
        except (TypeError, ValueError):
            progress = None
        return {
            'source': 'fsm',
            'state': str(mission_status.get('state') or ''),
            'instruction': str(mission_status.get('instruction') or ''),
            'progress': progress,
            'step': None,
            'last_action': str(mission_status.get('active_subtask') or ''),
            'outcome': str(mission_status.get('outcome') or ''),
            'degraded': False,
        }

    # Последняя завершённая VLM-миссия всё же должна оставить след: без этого
    # экран после успешного «Готово» выглядит так же, как до запуска.
    outcome = ''
    degraded = False
    if last_end_index >= 0:
        end = events[last_end_index]
        degraded = bool(end.get('degraded'))
        outcome = ('миссия отменена оператором' if end.get('cancelled')
                   else 'миссия завершена, шагов: %s' % end.get('steps'))
    return {
        'source': 'none',
        'state': 'IDLE',
        'instruction': '',
        'progress': None,
        'step': None,
        'last_action': '',
        'outcome': outcome,
        'degraded': degraded,
    }
