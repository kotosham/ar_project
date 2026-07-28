"""Единственный модуль консоли, который знает про rclpy.

ПОЧЕМУ ВЕБ-СЕРВЕР ПОДНИМАЕТСЯ РАНЬШЕ ROS
========================================
Штатный сценарий этой консоли — «включил и жду робота»: оператор открывает
страницу ДО того, как на Pi и edge что-то поднялось. Между тем
`deploy/transport/transport_env.sh:13-14` выставляет
`ZENOH_ROUTER_CHECK_ATTEMPTS` (по умолчанию 10), и под rmw_zenoh создание узла
БЛОКИРУЕТСЯ, пока роутер недоступен, а затем падает. То есть при наивном
порядке «rclpy.init() -> потом HTTP» процесс консоли умирал бы ровно в том
единственном случае, ради которого он написан, и оператор получал бы
ERR_CONNECTION_REFUSED вместо экрана ожидания с диагнозом.

Поэтому здесь:
  1. HTTP-сервер стартует ПЕРВЫМ, до `rclpy.init()`;
  2. вся ROS-часть живёт в фоновом потоке с бесконечным ретраем (RETRY_PERIOD_S);
  3. пока узла нет, backend честно отдаёт `ros_connected: false` и человеческую
     причину — её показывает шаг 4 мастера;
  4. ни одно исключение ROS-потока не роняет веб-сервер: поток ловит
     BaseException, кладёт причину в backend и повторяет попытку.

Даже импорт rclpy сделан ниже отказоустойчивым: если модуль запустили в
окружении без `source install/setup.bash`, консоль всё равно поднимется и
объяснит это по-русски, вместо ImportError в пустоту.

РАЗДЕЛЕНИЕ ОТВЕТСТВЕННОСТИ
==========================
`ConsoleBackend` — чистый python-объект (конфиг, каталог миров, супервизор
стека, проверка VLM). Он существует всегда и является тем самым duck-typed
backend, который потребляет `http_api`. `ConsoleRosNode` — узел ROS, который
появляется и исчезает; backend держит на него ссылку и деградирует до честного
«ROS ещё не подключён», когда её нет.
"""
import json
import math
import os
import shlex
import socket
import sys
import threading
import time
import uuid
from collections import deque
from urllib.parse import urlsplit

# ROS-импорты изолированы: без них модуль всё равно импортируется, и веб-часть
# работает. Node подменяется на object только чтобы объявление класса ниже не
# упало — экземпляр в этом состоянии никогда не создаётся (см. _ros_loop).
try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)

    from diagnostic_msgs.msg import DiagnosticArray
    from geometry_msgs.msg import Twist, TwistStamped
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, JointState, LaserScan
    from std_msgs.msg import Empty, String

    from ar_project_msgs.action import Stop
    from ar_project_msgs.msg import Heartbeat
    from object_tracking_msgs.action import SeekObject

    from fleet_comms.qos import liveliness_status
    _ROS_IMPORT_ERROR = ''
except Exception as _exc:                      # noqa: BLE001 — намеренно широко
    rclpy = None
    Node = object
    _ROS_IMPORT_ERROR = '%s: %s' % (type(_exc).__name__, _exc)

# Чистые модули: ни один из них не тянет rclpy, поэтому импортируются всегда.
from fleet_comms.mode_profiles import (HARDWARE, NAV2_LIFECYCLE_NODES,
                                       PLANNER_FLAT, SIM, profile_for)
from fleet_comms.vlm_env import KEY_API_KEY, load_env_file

from operator_console import http_api, preflight_rules, vlm_check
from operator_console.config_store import DEFAULT_CONFIG, ConfigStore
from operator_console.stack_runner import StackRunner, build_launch_argv
from operator_console.worlds_catalog import (default_spawn_args,
                                             load_catalog_file, resolve_paths,
                                             validate)

HEARTBEAT_PERIOD_S = 0.5        # обязан совпадать с продюсером: liveliness_status()
                                # у издателя и подписчика должны быть одинаковыми,
                                # иначе QoS несовместимы и /heartbeat молчит (qos.py:56).
RETRY_PERIOD_S = 3.0            # пауза между попытками поднять ROS-узел
LIFECYCLE_POLL_S = 5.0
# Срок жизни ОДНОГО запроса get_state. Нужен потому, что rclpy не отменяет
# call_async сам: если сервер Nav2 исчез между вызовом и ответом (перезапуск
# стека, падение controller_server), future не завершится НИКОГДА. Без срока имя
# навсегда оставалось бы в _lifecycle_pending, опрос его молча пропускал, и
# консоль до конца своей жизни показывала бы состояние, снятое до перезапуска —
# «Nav2 не активирован» поверх полностью активного Nav2.
LIFECYCLE_CALL_TIMEOUT_S = 12.0
# Кэш TCP-пробы роутера zenoh. 5 с: достаточно, чтобы поднятый роутер был
# замечен почти сразу, и достаточно, чтобы не блокировать поток кадров SSE
# (кадр раз в секунду, а connect к неподнятому роутеру висит до таймаута).
PROBE_CACHE_S = 5.0
MISSION_PUB_WAIT_S = 20.0
# Дольше этого без единого события /vlm/activity миссию считаем оборвавшейся:
# оркестратор пишет событие на каждом шаге, а самый долгий шаг ограничен
# vlm_timeout_s (по умолчанию 30 с) плюс исполнение навыка. 180 с — заведомо
# больше любого шага, но заметно меньше, чем терпение оператора перед
# неснимаемой блокировкой «миссия уже идёт».
MISSION_ACTIVITY_STALE_S = 180.0
# Засев карты: RUNBOOK.md:346-349 крутит робота на месте, чтобы SLAM получил
# unknown-ячейки и появились фронтиры. Там 5 с в одну сторону и 5 с обратно;
# здесь суммарные SEED_DURATION_S делятся пополам — вращение остаётся
# нулевым по сумме (робот заканчивает в исходном курсе), но занимает ~5 с.
SEED_DURATION_S = 5.0
SEED_YAW_RATE = 0.6
SEED_RATE_HZ = 10.0

_HB_STATUS = {0: 'OK', 1: 'DEGRADED', 2: 'DOWN'}

# Параметры узла и их значения по умолчанию. Значения ЗДЕСЬ же читаются до
# rclpy.init() (см. resolve_params) — узел объявляет ровно их, чтобы
# `ros2 param get` показывал то же самое, что реально использует веб-часть.
PARAM_DEFAULTS = {
    'bind': '0.0.0.0',
    'port': 8090,
    'config_path': '~/.ros/operator_console.json',
    'dashboard_url': '',        # пусто -> env DASHBOARD_URL -> http://127.0.0.1:8088
    'vlm_env_path': '',         # пусто -> env VLM_ENV_FILE -> share planner_orchestrator
    'vlm_env_template': '',     # пусто -> <share planner_orchestrator>/vlm.env.example
    'worlds_catalog': '',       # пусто -> <share ar_project>/config/worlds.yaml
    'console_token': '',        # пусто -> env CONSOLE_TOKEN
    'edge_host': '',            # пусто -> env EDGE_HOST
    'ros_launch_cmd': '',       # пусто -> ros2 launch (см. stack_start)
    # Интерпретатор с torch/cv2 для детектора и оркестратора. В контейнере это
    # /opt/ot_venv/bin/python, а заводское значение конфига — путь разработчика
    # ~/.venvs/ros-jazzy-ml/bin/python, которого в образе НЕТ. Без этого
    # параметра edge_layer падал бы fail-loud'ом при каждом «Запустить стек»,
    # то есть шаг «отправить задание» был бы недостижим в docker.
    'venv_python': '',
    # Куда дублировать журнал стека. Пусто -> <каталог конфига>/console_stack.log.
    # Без файла журнал живёт только в памяти и умирает вместе с контейнером —
    # разобрать упавший прогон потом нечем.
    'stack_log_path': '',
}
_PATH_PARAMS = ('config_path', 'vlm_env_path', 'vlm_env_template', 'worlds_catalog',
                'venv_python', 'stack_log_path')


# --------------------------------------------------------------------------
# Разбор параметров ДО rclpy
# --------------------------------------------------------------------------

def parse_cli_params(argv):
    """Вытащить `-p name:=value` из argv без rclpy.

    Нужна именно своя реализация: bind/port/config_path требуются, чтобы
    поднять HTTP, а сделать это надо ДО rclpy.init() (см. докстроку модуля).
    Разбирается только форма `-p`/`--param`; `--params-file` намеренно не
    поддержан — консоль запускается одной строкой из docker-compose и launch.
    """
    out = {}
    tokens = list(argv or [])
    for i, token in enumerate(tokens):
        if token not in ('-p', '--param') or i + 1 >= len(tokens):
            continue
        name, sep, value = tokens[i + 1].partition(':=')
        if sep and name:
            out[name.strip()] = value
    return out


def _share_dir(package):
    """Каталог share пакета; '' если пакет не установлен.

    ament_index_python — чистый python и НЕ тянет rclpy, поэтому вызывать её
    до rclpy.init() безопасно.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package)
    except Exception:
        return ''


def resolve_params(argv):
    """Итоговые значения параметров: CLI -> переменные окружения -> дефолт."""
    raw = parse_cli_params(argv)
    params = {}
    for name, default in PARAM_DEFAULTS.items():
        value = raw.get(name, default)
        params[name] = value
    try:
        params['port'] = int(params['port'])
    except (TypeError, ValueError):
        params['port'] = PARAM_DEFAULTS['port']

    # DECISIONS §2.G: переменная DASHBOARD_URL объявлена в compose, но раньше её
    # никто не читал. Пустой параметр означает «взять из окружения».
    if not str(params['dashboard_url']).strip():
        params['dashboard_url'] = os.environ.get('DASHBOARD_URL',
                                                 'http://127.0.0.1:8088')
    if not str(params['console_token']).strip():
        params['console_token'] = os.environ.get('CONSOLE_TOKEN', '')
    if not str(params['edge_host']).strip():
        params['edge_host'] = os.environ.get('EDGE_HOST', '')

    orch_share = _share_dir('planner_orchestrator')
    if not str(params['vlm_env_path']).strip():
        params['vlm_env_path'] = (os.environ.get('VLM_ENV_FILE', '').strip()
                                  or (os.path.join(orch_share, 'vlm.env')
                                      if orch_share else '~/.ros/vlm.env'))
    if not str(params['vlm_env_template']).strip() and orch_share:
        params['vlm_env_template'] = os.path.join(orch_share, 'vlm.env.example')
    if not str(params['worlds_catalog']).strip():
        ar_share = _share_dir('ar_project')
        params['worlds_catalog'] = (os.path.join(ar_share, 'config', 'worlds.yaml')
                                    if ar_share else '')

    # DECISIONS §2.R: пути из параметров прогоняются через expanduser — иначе
    # '~/.ros/...' создаст каталог с именем '~' рядом с рабочим (та же ловушка,
    # что закрыта в vlm_sim_bringup.launch.py:70).
    for name in _PATH_PARAMS:
        if params[name]:
            params[name] = os.path.expanduser(str(params[name]))

    # http_api читает токен либо через backend.console_token(), либо из
    # окружения; дублируем значение, чтобы порядок написания модулей не решал,
    # включится защита или нет. setdefault: заданный снаружи CONSOLE_TOKEN
    # приоритетнее и не затирается.
    if params['console_token']:
        os.environ.setdefault('CONSOLE_TOKEN', str(params['console_token']))
    return params


# --------------------------------------------------------------------------
# Backend: живёт всегда, ROS-узел — опционально
# --------------------------------------------------------------------------

class ConsoleBackend:
    """Реализация контракта backend для http_api (см. его докстроку).

    Ни один метод не возвращает значение VLM_API_KEY: наружу уходит только
    результат fleet_comms.vlm_env.public_view() (через ConfigStore) и результат
    vlm_check.probe(), в котором ключа нет по контракту.
    """

    def __init__(self, params):
        self.params = dict(params)
        self._lock = threading.RLock()
        self._node = None
        self._ros_phase = 'starting'
        self._ros_reason = 'ROS-часть консоли ещё не запускалась.'
        self._last_probe = None
        self._httpd = None
        self._seed_lock = threading.Lock()
        self.share_dir = _share_dir('ar_project')
        self._store = ConfigStore(self.params['config_path'],
                                  self.params['vlm_env_path'],
                                  self.params['vlm_env_template'])
        try:
            self._store.load()
        except Exception as exc:                       # noqa: BLE001
            _say('конфиг консоли не прочитан (%s) — работаю на значениях по умолчанию' % exc)
        self._runner = StackRunner()
        self._probe_cache = (0.0, None)                # (monotonic, результат TCP-пробы)
        self._catalog = None
        self._catalog_problems = []
        self._load_catalog()
        # После каталога: patch валидирует world по списку известных id.
        self._apply_venv_param()

    def _apply_venv_param(self):
        """Параметр venv_python перекрывает ЗАВОДСКОЕ значение конфига.

        Заводское значение — путь разработчика (~/.venvs/ros-jazzy-ml/bin/python),
        в контейнере такого нет, и edge_layer честно падает при попытке поднять
        детектор. Поэтому образ передаёт `-p venv_python:=/opt/ot_venv/bin/python`.
        Перекрываем ТОЛЬКО заводское значение: если оператор осознанно выбрал
        свой интерпретатор, параметр контейнера не должен его затирать при
        каждом перезапуске.
        """
        param = str(self.params.get('venv_python') or '').strip()
        if not param:
            return
        current = str(self._store.get().get('venv_python', '') or '').strip()
        if current and current != DEFAULT_CONFIG.get('venv_python'):
            return
        try:
            self._store.patch({'venv_python': param}, self._world_ids())
        except Exception as exc:                       # noqa: BLE001
            _say('venv_python из параметра не применён (%s)' % exc)

    # -- каталог миров ------------------------------------------------------

    def _load_catalog(self):
        path = self.params.get('worlds_catalog') or ''
        if not path:
            self._catalog_problems = [
                'каталог миров не найден: пакет ar_project не установлен '
                '(share-каталог не резолвится) — выбор мира недоступен']
            return
        try:
            self._catalog = load_catalog_file(path)
            self._catalog_problems = list(validate(self._catalog, self.share_dir))
        except FileNotFoundError:
            self._catalog_problems = [
                'каталог миров не найден: %s — пересоберите ar_project '
                '(файл ставится в share через install(DIRECTORY ... config ...))' % path]
        except Exception as exc:                       # noqa: BLE001
            self._catalog_problems = ['каталог миров не прочитан (%s): %s' % (path, exc)]

    def _world_ids(self):
        try:
            return list(self._catalog.ids()) if self._catalog else []
        except Exception:                              # noqa: BLE001
            return []

    def _world_entry(self, world_id):
        if self._catalog is None:
            raise KeyError('каталог миров не загружен')
        return self._catalog.get(world_id)

    # -- жизненный цикл ROS-части ------------------------------------------

    def attach_httpd(self, httpd):
        self._httpd = httpd

    def attach_node(self, node):
        with self._lock:
            self._node = node
            self._ros_phase = 'connected'
            self._ros_reason = 'ROS-узел консоли создан, слушаю граф.'

    def detach_node(self, reason_ru, phase='retry'):
        with self._lock:
            self._node = None
            self._ros_phase = phase
            self._ros_reason = reason_ru

    def set_ros_status(self, phase, reason_ru):
        with self._lock:
            if self._node is None or phase == 'connected':
                self._ros_phase = phase
                self._ros_reason = reason_ru

    def ros_status(self):
        with self._lock:
            return {'connected': self._ros_phase == 'connected',
                    'phase': self._ros_phase,
                    'reason_ru': self._ros_reason}

    def _require_node(self):
        with self._lock:
            node = self._node
        if node is None:
            raise ValueError('ROS-часть консоли ещё не подключена: %s'
                             % self._ros_reason)
        return node

    def close(self):
        try:
            self._runner.close()
        except Exception:                              # noqa: BLE001
            pass
        if self._httpd is not None:
            try:
                self._httpd.shutdown()
            except Exception:                          # noqa: BLE001
                pass

    # -- конфиг -------------------------------------------------------------

    def get_config(self):
        cfg = self._store.get()
        cfg['stack'] = self._runner.status()
        cfg['ros'] = self.ros_status()
        # edge_host и ros_domain_id фиксируются окружением контейнера при старте
        # (DECISIONS §3.6): показываем их только для чтения, чтобы оператор не
        # правил поле, которое консоль всё равно не применит.
        cfg['edge_host'] = self.params.get('edge_host', '')
        cfg['console_mode_locked'] = bool(self._runner.is_running())
        return cfg

    def set_config(self, patch):
        return self._store.patch(dict(patch or {}), self._world_ids())

    def console_token(self):
        """Токен доступа к консоли (НЕ токен VLM). Читается http_api для
        проверки заголовка X-Console-Token (DECISIONS §1, вопрос 4)."""
        return str(self.params.get('console_token', ''))

    def dashboard_base_url(self):
        cfg_url = str(self._store.get().get('dashboard_url', '') or '').strip()
        return cfg_url or str(self.params.get('dashboard_url', ''))

    # -- VLM ----------------------------------------------------------------

    def vlm_public(self):
        return self._store.vlm_public()

    def vlm_write(self, base_url=None, model=None, token=None):
        result = self._store.vlm_write(base_url=base_url, model=model, token=token)
        # Старый результат проверки относился к другим кредам — держать его
        # значило бы показывать зелёную галочку от предыдущего ключа.
        self._last_probe = None
        return result

    def _api_key(self):
        """Ключ читается ЛОКАЛЬНО на время запроса и не сохраняется в поле
        объекта: у объекта backend длинная жизнь и он попадает в трейсбеки."""
        return (load_env_file(self.params['vlm_env_path']).get(KEY_API_KEY) or '')

    def vlm_probe(self):
        pub = self._store.vlm_public()
        result = vlm_check.probe(pub.get('base_url', ''), self._api_key(),
                                 pub.get('model', ''))
        self._last_probe = result
        _say('проверка VLM: %s' % json.dumps(vlm_check.redact_for_log(result),
                                             ensure_ascii=False))
        return result

    def vlm_models(self):
        """Список моделей — это ТОТ ЖЕ probe, а не отдельный запрос.

        probe первым шагом и так делает GET /models (vlm_check.probe), поэтому
        второй сетевой путь означал бы второе место, где ключ ходит по сети, и
        второй набор веток разбора ошибок. Отдаём срез результата probe.
        """
        result = self.vlm_probe()
        return {'ok': bool(result.get('ok')),
                'models': list(result.get('models') or []),
                'error_kind': result.get('error_kind', ''),
                'error_ru': result.get('error_ru', '')}

    # -- миры ---------------------------------------------------------------

    def worlds(self):
        if self._catalog is None:
            return []
        return [entry.to_public(self.share_dir) for entry in self._catalog.list()]

    def world_preview(self, world_id, kind):
        entry = self._world_entry(world_id)
        paths = resolve_paths(entry, self.share_dir)
        path = paths.get('preview') if kind == 'svg' else paths.get('preview_png')
        if not path or not os.path.exists(path):
            return None
        with open(path, 'rb') as handle:
            body = handle.read()
        return ('image/svg+xml' if kind == 'svg' else 'image/png', body)

    # -- snapshot и преflight ------------------------------------------------

    def build_snapshot(self):
        """Ровно та схема, которую читает preflight_rules.evaluate."""
        with self._lock:
            node = self._node
        snap = node.build_snapshot() if node is not None else _empty_ros_snapshot()
        snap['vlm'] = dict(self._store.vlm_public(), probe=self._last_probe)
        try:
            entry = self._world_entry(self._store.get().get('world', ''))
            snap['world'] = {'id': entry.id, 'status': entry.status,
                             'status_note': entry.status_note}
        except Exception:                              # noqa: BLE001
            snap['world'] = {'id': str(self._store.get().get('world', '')),
                             'status': 'unknown', 'status_note': ''}
        snap['catalog_problems'] = list(self._catalog_problems)
        try:
            snap['ros_domain_id'] = int(os.environ.get('ROS_DOMAIN_ID', '0'))
        except ValueError:
            snap['ros_domain_id'] = None
        snap['rmw'] = os.environ.get('RMW_IMPLEMENTATION', '')
        return snap

    def _cached_tcp_probe(self, host, port, timeout_s=1.0):
        """TCP-проба роутера с кэшем на PROBE_CACHE_S.

        Без кэша это стоило бы секунду НА КАЖДЫЙ кадр SSE: _verdict() зовётся
        раз в секунду, а connect к неподнятому роутеру висит до таймаута —
        то есть ровно в режиме «жду робота» страница обновлялась бы рывками,
        а поток кадров фактически встал бы. Кэш держит диагноз свежим (5 с
        достаточно, чтобы заметить поднятый роутер) и не блокирует поток.
        """
        now = time.monotonic()
        stamp, cached = self._probe_cache
        if cached is not None and (now - stamp) < PROBE_CACHE_S:
            return cached
        sock = None
        try:
            sock = socket.create_connection((host, int(port)), timeout=timeout_s)
            result = (True, '')
        except OSError as exc:
            result = (False, exc.strerror or str(exc))
        finally:
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._probe_cache = (now, result)
        return result

    def _verdict(self):
        """(конфиг, snapshot, вердикт-в-публичной-форме) за один проход."""
        cfg = self._store.get()
        snap = self.build_snapshot()
        try:
            public = preflight_rules.evaluate(
                snap, cfg.get('mode', SIM), cfg.get('planner', 'vlm'),
                # Без edge_host проверка роутера всегда отвечала «адрес не
                # задан» и НИ РАЗУ не ходила в сеть — то есть самая частая
                # причина «не вижу робота» не диагностировалась вовсе.
                edge_host=self.params.get('edge_host', ''),
                tcp_probe=self._cached_tcp_probe).to_public()
        except Exception as exc:                       # noqa: BLE001
            public = {'ready': False, 'link_phase': 'waiting',
                      'link_message': 'Проверки не выполнены.',
                      'checks': [{'id': 'preflight', 'title': 'Преflight',
                                  'level': 'error',
                                  'message': 'Проверки не выполнены: %s' % exc,
                                  'hint': ''}],
                      'blocking': ['preflight']}
        ros = self.ros_status()
        if not ros['connected']:
            # Без этой строки оператор увидел бы «ROS-граф пуст» — формально
            # верно, но диагноз другой: узла консоли ещё нет вообще.
            public['ready'] = False
            public['checks'] = [{'id': 'ros', 'title': 'ROS-узел консоли',
                                 'level': 'wait', 'message': ros['reason_ru'],
                                 'hint': 'Веб-консоль работает без ROS намеренно; '
                                         'подключение повторяется раз в %.0f с.'
                                         % RETRY_PERIOD_S}] + list(public.get('checks', []))
            blocking = list(public.get('blocking', []))
            public['blocking'] = ['ros'] + [b for b in blocking if b != 'ros']
        public['ros_connected'] = ros['connected']
        return cfg, snap, public

    def preflight(self):
        return self._verdict()[2]

    def link(self):
        cfg, snap, public = self._verdict()
        return {'phase': public.get('link_phase', 'waiting'),
                'message': public.get('link_message', ''),
                'topics': dict(snap.get('topic_ages', {})),
                'mode': cfg.get('mode', SIM),
                'ros_connected': public.get('ros_connected', False)}

    def state(self):
        """Ответ на GET /api/state: доступен ВСЕГДА, в том числе когда ROS ещё
        не поднялся, — это и есть экран ожидания вместо ERR_CONNECTION_REFUSED."""
        cfg, snap, public = self._verdict()
        ros = self.ros_status()
        return {'ros_connected': ros['connected'],
                'ros_phase': ros['phase'],
                'reason_ru': ros['reason_ru'],
                'mode': cfg.get('mode', SIM),
                'planner': cfg.get('planner', 'vlm'),
                'edge_host': self.params.get('edge_host', ''),
                'ros_domain_id': snap.get('ros_domain_id'),
                'rmw': snap.get('rmw', ''),
                'link_phase': public.get('link_phase', 'waiting'),
                'link_message': public.get('link_message', ''),
                'ready': bool(public.get('ready')),
                'stack': self._runner.status(),
                'server_time': time.time()}

    def sse_snapshot(self):
        cfg, snap, public = self._verdict()
        try:
            mission = preflight_rules.merge_mission_view(snap.get('mission'),
                                                         list(snap.get('activity', [])))
        except Exception as exc:                       # noqa: BLE001
            mission = {'source': 'none', 'state': 'IDLE', 'instruction': '',
                       'progress': None, 'step': None, 'last_action': '',
                       'outcome': 'вид миссии не собран: %s' % exc, 'degraded': False}
        return {'preflight': public,
                'link': {'phase': public.get('link_phase', 'waiting'),
                         'message': public.get('link_message', ''),
                         'topics': dict(snap.get('topic_ages', {})),
                         'mode': cfg.get('mode', SIM)},
                'stack': self._runner.status(),
                'mission': mission,
                'config': cfg,
                'ros': self.ros_status(),
                'robot_pose': snap.get('robot_pose'),
                'server_time': time.time()}

    # -- стек ---------------------------------------------------------------

    def stack_start(self):
        cfg = self._store.get()
        if cfg.get('mode') == HARDWARE:
            raise PermissionError(
                'В режиме реального робота консоль не запускает стек — робот и '
                'edge поднимаются на своих хостах. Консоль только ждёт подключения.')
        world_file = ''
        extra = {}
        try:
            entry = self._world_entry(cfg.get('world', ''))
            world_file = resolve_paths(entry, self.share_dir).get('sdf', '') or ''
            extra.update(default_spawn_args(entry))
        except KeyError as exc:
            raise ValueError('Мир не выбран или отсутствует в каталоге: %s' % exc)
        for key in ('max_steps', 'replan_every_n', 'vlm_timeout_s'):
            if key in cfg:
                extra[key] = str(cfg[key])
        # DECISIONS §2.C: профиль console несовместим с edge/all, и владение
        # верхним слоем задаётся явным ключом, а не догадкой. Не владеем — верх
        # поднимает docker-compose, консоль запускает только нижний слой.
        layer = cfg.get('layer', 'all')
        if cfg.get('owns_edge_layer') is False:
            layer = 'robot'
        argv = build_launch_argv(
            mode=cfg.get('mode', SIM), planner=cfg.get('planner', 'vlm'),
            layer=layer, world_file=world_file,
            gui=bool(cfg.get('gui')), rviz=bool(cfg.get('rviz')),
            dashboard_port=self._dashboard_port(),
            venv_python=os.path.expanduser(str(cfg.get('venv_python', ''))),
            extra=extra)
        custom = str(self.params.get('ros_launch_cmd', '') or '').strip()
        if custom:
            # Подмена команды нужна для нестандартных развёртываний (обёртка
            # вроде `bash -lc 'source ...; ros2 launch ...'`). Аргументы вида
            # name:=value сохраняются от build_launch_argv, чтобы конфиг мастера
            # не потерялся при подмене.
            argv = shlex.split(custom) + [a for a in argv if ':=' in a]
        # Дублируем журнал в файл: кольцевой буфер в памяти умирает вместе с
        # контейнером, а разбирать упавший прогон потом нечем. Каталог тот же,
        # что у конфига, — он уже смонтирован томом console-state.
        log_path = str(self.params.get('stack_log_path') or '').strip()
        if not log_path:
            log_path = os.path.join(
                os.path.dirname(os.path.expanduser(self.params['config_path'])) or '.',
                'console_stack.log')
        return self._runner.start(argv, env=self._store.as_launch_env(),
                                  log_path=log_path)

    def _dashboard_port(self):
        try:
            port = urlsplit(self.dashboard_base_url()).port
        except ValueError:
            port = None
        return int(port or 8088)

    def stack_stop(self):
        return self._runner.stop()

    def stack_status(self):
        return self._runner.status()

    def stack_log(self, since=0):
        return self._runner.log(since=int(since or 0))

    # -- миссия -------------------------------------------------------------

    def mission_start(self, text):
        text = (text or '').strip()
        if not text:
            raise ValueError('Задание пустое.')
        node = self._require_node()
        # Проверка «миссия уже идёт» идёт ДО преflight: если задание отвергнуто
        # именно поэтому, оператору нужен этот диагноз, а не общий список
        # проверок, который во время нормально идущей миссии весь зелёный.
        node.ensure_idle()
        cfg, _snap, public = self._verdict()
        if not public.get('ready'):
            blocking = set(public.get('blocking', []))
            reasons = [c.get('message', '') for c in public.get('checks', [])
                       if c.get('id') in blocking]
            raise ValueError('Стек не готов: '
                             + ('; '.join(r for r in reasons if r)
                                or 'есть блокирующие проверки'))
        return node.mission_start(text, cfg.get('planner', 'vlm'))

    def mission_stop(self):
        cfg = self._store.get()
        return self._require_node().mission_stop(cfg.get('planner', 'vlm'))

    def seed_map(self):
        cfg = self._store.get()
        node = self._require_node()
        if not self._seed_lock.acquire(False):
            raise ValueError('Засев карты уже идёт — дождитесь окончания вращения.')
        try:
            return node.seed_map(cfg.get('mode', SIM))
        finally:
            self._seed_lock.release()


def _empty_ros_snapshot():
    """Snapshot, когда ROS-узла нет. Схема та же — preflight_rules не должен
    знать, что бывает состояние «узла ещё нет»."""
    return {'health_rows': {}, 'health_age_s': None, 'heartbeats': {},
            'topic_ages': {}, 'graph_nodes': [],
            'nav2_lifecycle': {name: 'unknown' for name in NAV2_LIFECYCLE_NODES},
            'mission': None, 'activity': []}


# --------------------------------------------------------------------------
# ROS-узел
# --------------------------------------------------------------------------

class ConsoleRosNode(Node):
    """Подписки, ActionClient'ы и lifecycle-опрос. Чистой логики здесь нет."""

    def __init__(self, backend, params):
        super().__init__('operator_console')
        self._backend = backend
        # Параметры объявляются УЖЕ разрешёнными (resolve_params прочитал те же
        # -p ещё до rclpy.init): так `ros2 param get` показывает ровно то, чем
        # пользуется веб-часть, а не пустые дефолты.
        for name, value in sorted(params.items()):
            try:
                self.declare_parameter(name, value)
            except Exception:                          # noqa: BLE001
                pass

        self._lock = threading.Lock()
        self._components = {}
        self._health_rx = 0.0
        self._heartbeats = {}
        self._mission = None
        self._activity = deque(maxlen=200)
        self._topic_rx = {}
        self._mission_running = False
        self._mission_step = 0
        self._mission_target = ''
        self._activity_rx = 0.0
        self._seek_handle = None
        self._odom_pose = None          # {'x','y','yaw','topic'} для плана мира
        self._odom_rx = 0.0
        self._lifecycle = {name: 'unknown' for name in NAV2_LIFECYCLE_NODES}
        # {имя: (future, монотонный дедлайн)} — не set: без дедлайна зависший
        # запрос заморозил бы состояние навсегда (см. LIFECYCLE_CALL_TIMEOUT_S).
        self._lifecycle_pending = {}

        self.create_subscription(DiagnosticArray, '/robot_health',
                                 self._on_health, 5)
        self.create_subscription(Heartbeat, '/heartbeat', self._on_heartbeat,
                                 liveliness_status(HEARTBEAT_PERIOD_S))
        self.create_subscription(String, '/mission/status', self._on_mission,
                                 _latched_qos())
        self.create_subscription(String, '/vlm/activity', self._on_activity,
                                 _latched_qos(depth=50))

        # Пробы связи. Каждая — ТОЛЬКО ради времени прихода, depth 1 и
        # BEST_EFFORT: консоль обязана быть бесплатной для канала. Изображений
        # в списке нет и быть не может — вторая подписка на /camera/camera/*
        # заново открывает поток камеры Pi по Wi-Fi (edge_bringup.launch.py:29-30),
        # то есть монитор ломал бы ровно то, что мониторит. camera_info — это
        # несколько сотен байт на кадр (матрица и искажения), а не картинка,
        # поэтому его подписывать можно.
        # Подписываемся на объединение обоих профилей: режим переключается
        # в мастере на лету, а пересоздавать подписки из HTTP-потока опаснее,
        # чем держать 5 простаивающих BEST_EFFORT-подписок.
        for topic in _probe_topics():
            if topic == '/robot_health':
                continue                    # его время прихода пишет _on_health
            msg_type = _link_topic_types().get(topic)
            if msg_type is None:
                continue
            self.create_subscription(msg_type, topic, self._probe_cb(topic),
                                     _probe_qos())

        # Одометрия ОБОИХ режимов: нужна для «реального вида сверху» — плана мира
        # с текущим положением робота. Это НЕ то же, что карта, уходящая в VLM:
        # там SLAM-сетка глазами модели, а здесь фактическая поза на известной
        # геометрии мира, то есть независимая точка отсчёта, по которой видно,
        # врёт ли SLAM. Подписки две по той же причине, что и у проб выше: режим
        # переключается мастером на лету.
        for _odom_topic in ('/odom', '/odometry/filtered'):
            self.create_subscription(Odometry, _odom_topic,
                                     self._odom_cb(_odom_topic), _probe_qos())

        # /vlm_mission: orchestrator_node.py:249 подписан обычным depth=1
        # (RELIABLE + VOLATILE). TRANSIENT_LOCAL на нашей стороне совместим и
        # переживает наш собственный рестарт, но НЕ доставит сообщение
        # подписчику, который появится позже, — отсюда ожидание подписчика
        # в mission_start.
        self._mission_pub = self.create_publisher(String, '/vlm_mission',
                                                  _latched_qos())
        # Операторская отмена (DECISIONS §1, вопрос 3): подписка уже есть в
        # orchestrator_node.py:251.
        self._cancel_pub = self.create_publisher(Empty, '/vlm_mission/cancel', 1)
        # Оба типа сразу: в sim gz_bridge ждёт Twist, на железе ros2_control —
        # TwistStamped (mode_profiles.py:104,143). Простаивающий publisher
        # ничего не стоит, а создавать его из HTTP-потока по требованию — нет.
        self._cmd_pub = {
            SIM: self.create_publisher(
                Twist, profile_for(SIM)['cmd_vel_final_topic'], 10),
            HARDWARE: self.create_publisher(
                TwistStamped, profile_for(HARDWARE)['cmd_vel_final_topic'], 10),
        }

        self._seek = ActionClient(self, SeekObject, 'seek_object')
        self._stop = ActionClient(self, Stop, 'stop')

        self._lifecycle_clients = {
            name: self.create_client(GetState, '/%s/get_state' % name)
            for name in NAV2_LIFECYCLE_NODES}
        self.create_timer(LIFECYCLE_POLL_S, self._poll_lifecycle)

        self.get_logger().info(
            'Консоль оператора: откройте http://%s:%d'
            % (params.get('bind') or '127.0.0.1', int(params.get('port', 8090))))

    # -- колбэки ------------------------------------------------------------

    def _probe_cb(self, topic):
        def _cb(_msg):
            self._topic_rx[topic] = time.monotonic()
        return _cb

    def _odom_cb(self, topic):
        """Поза робота из одометрии -> (x, y, yaw) для плана мира.

        yaw считаем прямо из кватерниона: тянуть ради одного угла зависимость
        (tf_transformations/scipy) в пакет, который гордится тем, что живёт на
        стандартной библиотеке, — несоразмерная цена за четыре умножения.
        """
        def _cb(msg):
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            with self._lock:
                self._odom_pose = {'x': float(p.x), 'y': float(p.y),
                                   'yaw': float(yaw), 'topic': topic}
                self._odom_rx = time.monotonic()
            self._topic_rx[topic] = time.monotonic()
        return _cb

    def _on_health(self, msg):
        now = time.monotonic()
        with self._lock:
            self._health_rx = now
            self._topic_rx['/robot_health'] = now
            for status in msg.status:
                level = status.level
                if isinstance(level, bytes):           # byte-поле в ROS 2
                    level = level[0] if level else 0
                self._components[status.name] = {
                    'level': int(level),
                    'message': status.message,
                    'values': {kv.key: kv.value for kv in status.values},
                    'rx': now,
                }

    def _on_heartbeat(self, msg):
        with self._lock:
            self._heartbeats[msg.node_name] = {
                'status': _HB_STATUS.get(int(msg.status), '?'),
                'cpu_load': round(float(msg.cpu_load), 2),
                'latency_ms': round(float(msg.last_latency_ms), 1),
                'epoch': int(msg.mission_epoch),
                'rx': time.monotonic(),
            }

    def _on_mission(self, msg):
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            data = {'state': msg.data}
        with self._lock:
            self._mission = data

    def _on_activity(self, msg):
        try:
            event = json.loads(msg.data)
        except (ValueError, TypeError):
            event = {'event': 'raw', 'detail': msg.data}
        event['rx'] = time.time()
        with self._lock:
            self._activity.append(event)
            self._activity_rx = time.monotonic()
            # Признак «миссия идёт» берётся отсюда, а не из /mission/status:
            # в VLM-режиме статус публикует только SeekObjectServer и он вечно
            # IDLE. Топик TRANSIENT_LOCAL depth 50, поэтому поздно
            # подключившаяся консоль получает историю В ПОРЯДКЕ прихода и
            # доигрывает флаг до правильного значения.
            name = event.get('event')
            if name == 'mission_start':
                self._mission_running = True
                self._mission_step = 0
                self._mission_target = str(event.get('target', ''))
            elif name == 'mission_end':
                self._mission_running = False
            elif name in ('step_start', 'step_result'):
                try:
                    self._mission_step = int(event.get('step', self._mission_step))
                except (TypeError, ValueError):
                    pass

    # -- lifecycle Nav2 ------------------------------------------------------

    _LIFECYCLE_LABELS = {0: 'unknown', 1: 'unconfigured', 2: 'inactive',
                         3: 'active', 4: 'finalized'}

    def _poll_lifecycle(self):
        """Опрос состояния трёх серверов Nav2 БЕЗ блокировки исполнителя.

        Присутствие узла в графе (robot_health_aggregator.py:109) не значит, что
        он сконфигурирован: unconfigured planner_server виден в графе и не
        навигирует. Ответ обрабатывается колбэком future — синхронный
        call() здесь заблокировал бы единственный поток spin.
        """
        # Сначала хороним просроченные запросы: сервер, которому мы послали
        # get_state, мог исчезнуть до ответа, и тогда future висит вечно.
        now = time.monotonic()
        for name, (future, deadline) in list(self._lifecycle_pending.items()):
            if now < deadline:
                continue
            del self._lifecycle_pending[name]
            self._lifecycle[name] = 'unknown'
            try:
                future.cancel()
                self._lifecycle_clients[name].remove_pending_request(future)
            except Exception:                          # noqa: BLE001
                # Клиента могло не остаться, а remove_pending_request есть не во
                # всех версиях rclpy: снятия имени из pending уже достаточно.
                pass

        for name, client in self._lifecycle_clients.items():
            if name in self._lifecycle_pending:
                continue
            if not client.service_is_ready():
                self._lifecycle[name] = 'unknown'
                continue
            future = client.call_async(GetState.Request())
            self._lifecycle_pending[name] = (
                future, time.monotonic() + LIFECYCLE_CALL_TIMEOUT_S)
            future.add_done_callback(self._lifecycle_done(name))

    def _lifecycle_done(self, name):
        def _cb(future):
            self._lifecycle_pending.pop(name, None)
            try:
                response = future.result()
                self._lifecycle[name] = str(response.current_state.label
                                            or self._LIFECYCLE_LABELS.get(
                                                int(response.current_state.id),
                                                'unknown'))
            except Exception:                          # noqa: BLE001
                self._lifecycle[name] = 'unknown'
        return _cb

    # -- snapshot ------------------------------------------------------------

    def build_snapshot(self):
        now = time.monotonic()
        with self._lock:
            health_rows = {
                name: {'level': item['level'], 'message': item['message'],
                       'values': dict(item['values']),
                       'age_s': round(now - item['rx'], 1)}
                for name, item in self._components.items()}
            heartbeats = {
                name: {'status': item['status'], 'cpu_load': item['cpu_load'],
                       'latency_ms': item['latency_ms'], 'epoch': item['epoch'],
                       'age_s': round(now - item['rx'], 1)}
                for name, item in self._heartbeats.items()}
            snapshot = {
                'health_rows': health_rows,
                'health_age_s': (round(now - self._health_rx, 1)
                                 if self._health_rx else None),
                'heartbeats': heartbeats,
                'topic_ages': {topic: round(now - rx, 1)
                               for topic, rx in self._topic_rx.items()},
                'mission': self._mission,
                'activity': list(self._activity),
            }
        try:
            snapshot['graph_nodes'] = list(self.get_node_names())
        except Exception:                              # noqa: BLE001
            snapshot['graph_nodes'] = []
        snapshot['nav2_lifecycle'] = dict(self._lifecycle)
        with self._lock:
            pose, rx = self._odom_pose, self._odom_rx
        snapshot['robot_pose'] = (dict(pose, age_s=round(time.monotonic() - rx, 1))
                                  if pose else None)
        return snapshot

    # -- миссия --------------------------------------------------------------

    def mission_running(self):
        with self._lock:
            if not self._mission_running:
                return False
            if not self._activity_rx:
                return False
            return (time.monotonic() - self._activity_rx) < MISSION_ACTIVITY_STALE_S

    def ensure_idle(self):
        """ValueError, если миссия уже идёт.

        orchestrator_node.py:310-315 при self._busy молча выбрасывает новое
        сообщение — без этой проверки оператор увидел бы «задание принято» и
        ждал бы реакции, которой не будет.
        """
        if not self.mission_running():
            return
        with self._lock:
            step = int(self._mission_step)
        raise ValueError(
            'Миссия уже идёт (шаг %d). Оркестратор не принимает новое '
            'задание до завершения — сначала нажмите «Стоп»' % step)

    def mission_start(self, text, planner):
        """Отправка задания одним из ДВУХ уже существующих входов стека.

        Консоль не изобретает новый протокол: `/vlm_mission` — это тот же вход,
        которым пользуется house_scenario_runner, а `/seek_object` — тот же
        action, что в RUNBOOK.md:352-354.
        """
        self.ensure_idle()

        if planner == PLANNER_FLAT:
            if not self._seek.wait_for_server(timeout_sec=5.0):
                raise ValueError(
                    'Сервер действия /seek_object не отвечает — executive '
                    '(search_coordinator) ещё не поднялся.')
            goal = SeekObject.Goal(instruction=text, request_id=uuid.uuid4().hex,
                                   mission_epoch=0, allow_vlm=False)
            future = self._seek.send_goal_async(goal)
            future.add_done_callback(self._on_seek_accepted)
            return {'ok': True, 'channel': 'seek_object', 'text': text,
                    'note_ru': 'Задание отправлено как цель /seek_object '
                               '(allow_vlm=false): исполняет FLAT-цикл на роботе, '
                               'VLM не участвует.'}

        # /vlm_mission у оркестратора VOLATILE (orchestrator_node.py:249):
        # опубликованное «в пустоту» сообщение не будет доставлено подписчику,
        # который появится позже, — поэтому ждём подписчика, а не публикуем
        # вслепую.
        deadline = time.monotonic() + MISSION_PUB_WAIT_S
        while self._mission_pub.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                raise ValueError(
                    'VLM-оркестратор не подписан на /vlm_mission (ждали %.0f с) — '
                    'он ещё не поднялся. Проверьте, что planner_orchestrator '
                    'запущен на edge.' % MISSION_PUB_WAIT_S)
            time.sleep(0.2)
        self._mission_pub.publish(String(data=text))
        return {'ok': True, 'channel': '/vlm_mission', 'text': text,
                'note_ru': 'Задание опубликовано в /vlm_mission. Оркестратор '
                           'начнёт цикл «наблюдение → план → шаг»; ход миссии '
                           'виден в мониторе ниже.'}

    def _on_seek_accepted(self, future):
        try:
            handle = future.result()
        except Exception:                              # noqa: BLE001
            self._seek_handle = None
            return
        self._seek_handle = handle if getattr(handle, 'accepted', False) else None

    def mission_stop(self, planner):
        """Операторский abort: остановка движения + отмена миссии планировщика.

        Возвращает то, что РЕАЛЬНО произошло. Врать здесь особенно дорого:
        оператор жмёт «Стоп», когда робот едет не туда.
        """
        notes = []
        stopped_motion = False
        if self._stop.wait_for_server(timeout_sec=2.0):
            # Stop — epoch-exempt навык: он принимается независимо от эпохи
            # миссии, поэтому mission_epoch=0 здесь безопасен и не будет
            # отброшен как zombie-цель.
            goal = Stop.Goal(request_id=uuid.uuid4().hex, mission_epoch=0,
                             mode=getattr(Stop.Goal, 'SOFT_STOP', 0))
            self._stop.send_goal_async(goal)
            stopped_motion = True
            notes.append('Движение остановлено навыком Stop.')
        else:
            notes.append('Сервер навыка Stop не отвечает — движение консолью '
                         'НЕ остановлено; жмите аппаратный стоп.')

        subscribers = self._cancel_pub.get_subscription_count()
        self._cancel_pub.publish(Empty())
        planner_cancelled = subscribers > 0
        if planner_cancelled:
            notes.append('В /vlm_mission/cancel отправлена отмена: оркестратор '
                         'проверяет флаг между шагами, поэтому текущий шаг '
                         'доигрывается, а новых не будет.')
        else:
            notes.append('На /vlm_mission/cancel никто не подписан — '
                         'оркестратор не запущен либо собран из версии без '
                         'поддержки отмены; миссия VLM продолжится сама.')

        if self._seek_handle is not None:
            try:
                self._seek_handle.cancel_goal_async()
                notes.append('Цель /seek_object отменена.')
                planner_cancelled = True
            except Exception as exc:                   # noqa: BLE001
                notes.append('Отменить цель /seek_object не удалось: %s' % exc)
            self._seek_handle = None

        return {'ok': stopped_motion or planner_cancelled,
                'stopped_motion': stopped_motion,
                'planner_cancelled': planner_cancelled,
                'note_ru': ' '.join(notes)}

    # -- засев карты ---------------------------------------------------------

    def seed_map(self, mode):
        """Вращение на месте ~SEED_DURATION_S секунд.

        Зачем: в ограниченном мире SEARCH не стартует, пока у SLAM нет
        unknown-ячеек и, значит, фронтиров (RUNBOOK.md:346-349, §«Робот не
        двигается в ограниченном мире» RUNBOOK.md:521-522). Вращение делится
        на две половины с противоположным знаком, чтобы курс робота вернулся
        к исходному — как в RUNBOOK.
        """
        publisher = self._cmd_pub.get(mode) or self._cmd_pub[SIM]
        topic = profile_for(mode if mode in self._cmd_pub else SIM)['cmd_vel_final_topic']
        period = 1.0 / SEED_RATE_HZ
        half = max(0.5, SEED_DURATION_S / 2.0)
        for sign in (1.0, -1.0):
            deadline = time.monotonic() + half
            while time.monotonic() < deadline:
                publisher.publish(self._twist(mode, sign * SEED_YAW_RATE))
                time.sleep(period)
        publisher.publish(self._twist(mode, 0.0))
        return {'ok': True, 'topic': topic, 'duration_s': round(2 * half, 1),
                'note_ru': 'Робот повернулся на месте и вернулся в исходный курс '
                           '(%.0f с в /%s). Проверьте, что /frontiers стал '
                           'непустым — без фронтиров SEARCH не поедет.'
                           % (2 * half, topic.lstrip('/'))}

    def _twist(self, mode, yaw_rate):
        if mode == HARDWARE:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.twist.angular.z = float(yaw_rate)
            return msg
        msg = Twist()
        msg.angular.z = float(yaw_rate)
        return msg

    # -- завершение ----------------------------------------------------------

    def destroy_node(self):
        try:
            self._backend.detach_node('ROS-узел консоли остановлен.', phase='stopped')
        except Exception:                              # noqa: BLE001
            pass
        super().destroy_node()


# --------------------------------------------------------------------------
# Вспомогательное для ROS-части
# --------------------------------------------------------------------------

def _latched_qos(depth=1):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _probe_qos():
    """depth 1 + BEST_EFFORT: проба должна стоить каналу ровно ничего и никогда
    не создавать очередь из устаревших сообщений."""
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def _link_topic_types():
    """Таблица типов проб. Изображений здесь нет намеренно — см. комментарий
    к созданию подписок."""
    return {
        '/clock': Clock,
        '/scan': LaserScan,
        '/odom': Odometry,
        '/odometry/filtered': Odometry,
        '/joint_states': JointState,
        '/camera/camera/color/camera_info': CameraInfo,
        # DECISIONS §2.S: на железе берётся latched-выход реле на edge, а НЕ
        # Pi-топик — иначе консоль стала бы вторым потребителем камеры.
        '/camera_edge/color/camera_info': CameraInfo,
        '/robot_health': DiagnosticArray,
    }


def _probe_topics():
    topics = ['/clock']
    for mode in (SIM, HARDWARE):
        for topic in profile_for(mode)['link_required_topics']:
            if topic not in topics:
                topics.append(topic)
    return topics


def _say(text):
    """Лог до появления узла: get_logger() ещё не существует."""
    sys.stderr.write('[operator_console] %s\n' % text)
    try:
        sys.stderr.flush()
    except Exception:                                  # noqa: BLE001
        pass


def _ros_init(argv):
    """rclpy.init без установки обработчиков сигналов.

    Обработчики ставит главный поток (см. main): ROS-часть живёт в фоновом
    потоке и может перезапускаться, а перехваченный ею SIGINT не дал бы
    остановить веб-сервер.
    """
    options = None
    try:
        from rclpy.signals import SignalHandlerOptions
        options = SignalHandlerOptions.NO
    except Exception:                                  # noqa: BLE001
        options = None
    if options is not None:
        try:
            rclpy.init(args=argv, signal_handler_options=options)
            return
        except TypeError:
            pass
    rclpy.init(args=argv)


def _ros_loop(backend, params, argv, stop_event):
    """Фоновый поток: поднять узел, крутить spin, при любой беде — повторить.

    Ни один выход отсюда не должен ронять процесс: веб-консоль остаётся
    единственным способом узнать, ПОЧЕМУ ROS не поднялся.
    """
    if _ROS_IMPORT_ERROR:
        backend.set_ros_status(
            'failed',
            'ROS-библиотеки не импортируются (%s). Веб-консоль работает, но '
            'данных из ROS не будет: запустите её из окружения, где выполнен '
            'source /opt/ros/jazzy/setup.bash и source install/setup.bash.'
            % _ROS_IMPORT_ERROR)
        return

    while not stop_event.is_set():
        node = None
        try:
            backend.set_ros_status(
                'connecting',
                'Подключение к ROS: создаётся узел. Под rmw_zenoh это ожидание '
                'роутера (ZENOH_ROUTER_CHECK_ATTEMPTS, '
                'deploy/transport/transport_env.sh:13-14) — консоль ждёт, а не падает.')
            if not rclpy.ok():
                _ros_init(argv)
            node = ConsoleRosNode(backend, params)
            backend.attach_node(node)
            rclpy.spin(node)
            backend.detach_node('ROS-узел остановлен (spin завершился). '
                                'Повтор через %.0f с.' % RETRY_PERIOD_S)
        except BaseException as exc:                   # noqa: BLE001
            backend.detach_node(
                'ROS-узел не поднялся: %s: %s. Чаще всего это недоступный '
                'zenoh-роутер на %s:7447 — консоль повторит попытку через %.0f с.'
                % (type(exc).__name__, exc,
                   params.get('edge_host') or 'edge-хосте', RETRY_PERIOD_S))
        finally:
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:                      # noqa: BLE001
                    pass
            try:
                rclpy.try_shutdown()
            except Exception:                          # noqa: BLE001
                pass
        if stop_event.wait(RETRY_PERIOD_S):
            break


def main(argv=None):
    """Порядок здесь — главное архитектурное требование, см. докстроку модуля."""
    argv = list(sys.argv if argv is None else argv)
    params = resolve_params(argv)
    backend = ConsoleBackend(params)

    # HTTP ПЕРВЫМ: если поднять его не удалось (занят порт), падать честно —
    # без веб-части консоль бессмысленна.
    httpd = http_api.serve(backend, str(params['bind']), int(params['port']))
    backend.attach_httpd(httpd)
    threading.Thread(target=httpd.serve_forever, name='console-http',
                     daemon=True).start()
    _say('Консоль оператора: откройте http://%s:%d'
         % (params['bind'] or '127.0.0.1', int(params['port'])))

    stop_event = threading.Event()
    threading.Thread(target=_ros_loop, args=(backend, params, argv, stop_event),
                     name='console-ros', daemon=True).start()
    try:
        while not stop_event.wait(1.0):
            pass
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        backend.close()
        try:
            if rclpy is not None and rclpy.ok():
                rclpy.try_shutdown()
        except Exception:                              # noqa: BLE001
            pass


if __name__ == '__main__':
    main()
