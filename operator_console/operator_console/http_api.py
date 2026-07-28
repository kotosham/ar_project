"""HTTP-слой консоли оператора: таблица маршрутов, обработчик и прокси дашборда.

Модуль не импортирует rclpy и вообще ничего не знает о ROS: он работает с
duck-typed объектом `backend`, поэтому проверяется обычным pytest без
установленного ROS 2. Так же устроен и `fleet_comms/mission_dashboard.py:179` —
там handler тоже создаётся замыканием вокруг источника данных.

КОНТРАКТ BACKEND (реализуется `console_node.OperatorConsole`)::

    get_config() -> dict          # + необязательный ключ 'console_token'
    set_config(patch: dict) -> dict                  # ValueError -> HTTP 400
    vlm_public() -> dict                             # ровно {base_url, model, token_set}
    vlm_write(base_url, model, token) -> dict
    vlm_probe() -> dict
    vlm_models() -> dict
    worlds() -> list[dict]
    world_preview(world_id: str, kind: str) -> tuple[str, bytes] | None
    preflight() -> dict
    link() -> dict
    stack_start() -> dict                            # PermissionError -> HTTP 409
    stack_stop() -> dict
    stack_status() -> dict
    stack_log(since: int) -> dict
    mission_start(text: str) -> dict                 # ValueError -> 400, PermissionError -> 409
    mission_stop() -> dict
    sse_snapshot() -> dict
    dashboard_base_url() -> str
    seed_map() -> dict                               # НЕОБЯЗАТЕЛЬНЫЙ, см. ROUTE_SEED_MAP

КАК ПОДСУНУТЬ FakeBackend (тест без ROS)::

    class FakeBackend:
        def get_config(self):   return {'mode': 'sim', 'planner': 'vlm', 'console_token': ''}
        def sse_snapshot(self): return {'preflight': {'ready': True, 'checks': []}}
        def dashboard_base_url(self): return 'http://127.0.0.1:8088'
        def mission_start(self, text):
            if not text.strip():
                raise ValueError('Задание пустое.')
            return {'ok': True, 'channel': '/vlm_mission', 'text': text}
        # остальные методы — по мере надобности теста

    import http.client, threading
    srv = serve(FakeBackend(), '127.0.0.1', 0)          # 0 = свободный порт
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    conn = http.client.HTTPConnection(*srv.server_address)
    conn.request('GET', '/api/config'); assert conn.getresponse().status == 200
    srv.shutdown()

Отсутствующий у backend метод даёт 501 с русским текстом, а не AttributeError в
трейсбеке, — тест может реализовать ровно те методы, которые проверяет.

ЯЗЫК ОШИБОК: все тела ошибок — ``{'error': '<русский текст>'}``, все ответы
идут с ``Cache-Control: no-store`` и ``ensure_ascii=False``.
"""
import json
import os
import re
import time
import urllib.error
import urllib.parse
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from operator_console.ui_page import render_page

# Период кадра SSE. Ровно как у дашборда (mission_dashboard.py:215): консоль и
# монитор миссии обновляются в одном ритме, иначе на одной странице видно два
# рассинхронизированных состояния одного и того же стека.
SSE_PERIOD_S = 1.0

# Чтение из дашборда при проксировании потока событий. Дашборд шлёт кадр раз в
# секунду, поэтому 30 с не могут сработать на живом потоке — зато мёртвое
# соединение не подвесит поток обработчика навсегда.
PROXY_TIMEOUT_S = 30.0

MAX_BODY_BYTES = 65536

ROUTE_SEED_MAP = 'seed_map'

# Точные маршруты: (метод, путь) -> имя. Константа используется и обработчиком,
# и тестом на полноту — новый маршрут невозможно забыть в одном из двух мест.
ROUTES = {
    ('GET', '/'): 'page',
    ('GET', '/index.html'): 'page',
    ('GET', '/api/state'): 'state',
    ('GET', '/api/events'): 'events',
    ('GET', '/api/config'): 'config_get',
    ('POST', '/api/config'): 'config_set',
    ('GET', '/api/vlm'): 'vlm_get',
    ('POST', '/api/vlm/token'): 'vlm_token',
    ('POST', '/api/vlm/test'): 'vlm_test',
    ('GET', '/api/vlm/models'): 'vlm_models',
    ('GET', '/api/worlds'): 'worlds',
    ('GET', '/api/preflight'): 'preflight',
    ('GET', '/api/link'): 'link',
    ('POST', '/api/stack/start'): 'stack_start',
    ('POST', '/api/stack/stop'): 'stack_stop',
    ('GET', '/api/stack/status'): 'stack_status',
    ('GET', '/api/stack/log'): 'stack_log',
    ('POST', '/api/mission/start'): 'mission_start',
    ('POST', '/api/mission/stop'): 'mission_stop',
    ('POST', '/api/seed_map'): ROUTE_SEED_MAP,
}

_PREVIEW_SVG = re.compile(r'^/api/worlds/([^/]+)/preview\.svg$')
_PREVIEW_PNG = re.compile(r'^/api/worlds/([^/]+)/preview\.png$')

# Проверка токена не распространяется на эти два маршрута. Причина конкретная:
# EventSource в браузере физически не умеет ставить заголовок X-Console-Token,
# и с проверкой на /api/events страница осталась бы навсегда пустой, а
# /api/state — тот же снимок для тех, кто опрашивает консоль скриптом.
NO_AUTH_ROUTES = frozenset({'page', 'state', 'events',
                            'dashboard', 'dashboard_redirect', 'not_found'})

_ERR_502 = ('Монитор миссии недоступен по адресу %s. В режиме симуляции он '
            'поднимается вместе со стеком (edge_layer) — дождитесь запуска. В '
            'режиме реального робота он живёт на EDGE-боксе: проверьте '
            'переменную DASHBOARD_URL и то, что edge-слой действительно запущен.')


# ---------------------------------------------------------------------------
# разбор маршрута и тел запросов
# ---------------------------------------------------------------------------

def parse_route(path, method='GET'):
    """(имя_маршрута, параметры) по пути и методу.

    Отдельная чистая функция, потому что таблицу маршрутов надо уметь
    проверить тестом, не поднимая сокет.
    """
    clean = path.split('?', 1)[0]
    name = ROUTES.get((method, clean))
    if name:
        return name, {}
    m = _PREVIEW_SVG.match(clean)
    if m and method == 'GET':
        return 'world_preview', {'id': urllib.parse.unquote(m.group(1)), 'kind': 'svg'}
    m = _PREVIEW_PNG.match(clean)
    if m and method == 'GET':
        return 'world_preview', {'id': urllib.parse.unquote(m.group(1)), 'kind': 'png'}
    if clean == '/dashboard':
        return 'dashboard_redirect', {}
    if clean.startswith('/dashboard/'):
        return 'dashboard', {'sub': clean[len('/dashboard/'):]}
    return 'not_found', {}


def _console_token(backend):
    """Токен доступа к КОНСОЛИ (не к VLM) из трёх источников по порядку.

    Три источника, а не один, потому что backend'ы разные:
    `console_node.ConsoleBackend.console_token()` (console_node.py:339) отдаёт
    значение ROS-параметра и дублирует его в CONSOLE_TOKEN
    (console_node.py:206), а простой FakeBackend в тесте обычно кладёт ключ
    прямо в `get_config()`. Переменная окружения — последний рубеж: она
    работает, даже если backend вообще не знает про токен.
    """
    fn = getattr(backend, 'console_token', None)
    if callable(fn):
        value = str(fn() or '')
        if value:
            return value
    get_cfg = getattr(backend, 'get_config', None)
    if callable(get_cfg):
        value = str((get_cfg() or {}).get('console_token') or '')
        if value:
            return value
    return os.environ.get('CONSOLE_TOKEN', '') or ''


def read_json_body(handler, limit=MAX_BODY_BYTES):
    """Тело запроса как dict. Пустое тело — это {}, а не ошибка."""
    raw = read_raw_body(handler, limit)
    if not raw:
        return {}
    try:
        data = json.loads(raw.decode('utf-8'))
    except (ValueError, UnicodeDecodeError):
        raise ValueError('Тело запроса не является корректным JSON.')
    if not isinstance(data, dict):
        raise ValueError('Тело запроса должно быть JSON-объектом.')
    return data


def read_raw_body(handler, limit=MAX_BODY_BYTES):
    try:
        length = int(handler.headers.get('Content-Length') or 0)
    except (TypeError, ValueError):
        raise ValueError('Некорректный заголовок Content-Length.')
    if length <= 0:
        return b''
    if length > limit:
        raise ValueError('Тело запроса больше %d байт — консоль такие не принимает.' % limit)
    return handler.rfile.read(length)


# ---------------------------------------------------------------------------
# ответы
# ---------------------------------------------------------------------------

def _send(handler, code, ctype, body, extra=None):
    handler.send_response(code)
    handler.send_header('Content-Type', ctype)
    handler.send_header('Cache-Control', 'no-store')
    # X-Frame-Options НЕ ставится сознательно: страница консоли встраивает
    # собственный /dashboard/ в iframe, и DENY сломал бы шаг «Работа».
    handler.send_header('Referrer-Policy', 'no-referrer')
    for k, v in (extra or {}).items():
        handler.send_header(k, v)
    handler.send_header('Content-Length', str(len(body)))
    handler.end_headers()
    if handler.command != 'HEAD':
        handler.wfile.write(body)


def json_response(handler, code, payload):
    body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
    _send(handler, code, 'application/json; charset=utf-8', body)


def error_response(handler, code, message_ru):
    json_response(handler, code, {'error': message_ru})


# ---------------------------------------------------------------------------
# обратный прокси к mission_dashboard
# ---------------------------------------------------------------------------

def proxy(handler, base_url, sub_path, method='GET'):
    """Проксировать запрос к дашборду.

    ЗАЧЕМ: снаружи нужен ОДИН порт консоли, а iframe обязан быть same-origin —
    иначе ни CORS, ни смешанный контент не дадут встроить монитор миссии. Плюс
    в режиме железа дашборд вообще живёт на другом хосте (edge-боксе), и
    браузер оператора туда может не иметь доступа.

    Работает это только потому, что страница дашборда переведена на
    ОТНОСИТЕЛЬНЫЕ пути (mission_dashboard.py:355-365): переписывать тело ответа
    не нужно. С корневыми "/events" и "/map.jpg" запросы ушли бы в корень
    КОНСОЛИ, где таких маршрутов нет, и iframe показывал бы пустой шаблон.
    """
    query = ''
    if '?' in handler.path:
        query = '?' + handler.path.split('?', 1)[1]
    target = base_url.rstrip('/') + '/' + sub_path.lstrip('/') + query

    body = read_raw_body(handler) if method == 'POST' else None
    req = urllib.request.Request(target, data=body, method=method)
    # Заголовки клиента НЕ пересылаются, кроме Accept: незачем тащить в чужой
    # процесс куки и X-Console-Token.
    accept = handler.headers.get('Accept')
    if accept:
        req.add_header('Accept', accept)
    if body is not None:
        req.add_header('Content-Type',
                       handler.headers.get('Content-Type') or 'application/json')

    try:
        resp = urllib.request.urlopen(req, timeout=PROXY_TIMEOUT_S)
    except urllib.error.HTTPError as exc:
        # 404 самого дашборда — это его ответ, а не отказ связи: отдаём как есть.
        payload = exc.read()
        _send(handler, exc.code,
              exc.headers.get('Content-Type') or 'text/plain; charset=utf-8', payload)
        return
    except (urllib.error.URLError, OSError):
        error_response(handler, 502, _ERR_502 % base_url)
        return

    ctype = resp.headers.get('Content-Type') or 'application/octet-stream'
    if ctype.split(';', 1)[0].strip() == 'text/event-stream':
        _proxy_stream(handler, resp, ctype)
        return
    try:
        payload = resp.read()
    except OSError:
        error_response(handler, 502, _ERR_502 % base_url)
        return
    finally:
        resp.close()
    _send(handler, getattr(resp, 'status', 200) or 200, ctype, payload)


def _proxy_stream(handler, resp, ctype):
    """Поток событий пересылается ПОСТРОЧНО и с немедленным flush.

    Буферизация здесь превратила бы живой поток в зависшую страницу: браузер
    ничего не получит, пока не наберётся буфер, а дашборд шлёт всего ~200 байт
    в секунду.
    """
    handler.send_response(200)
    handler.send_header('Content-Type', ctype)
    handler.send_header('Cache-Control', 'no-store')
    handler.send_header('Referrer-Policy', 'no-referrer')
    handler.end_headers()
    handler._stream_started = True
    try:
        for line in resp:
            handler.wfile.write(line)
            handler.wfile.flush()
    except (BrokenPipeError, ConnectionError, OSError):
        return
    finally:
        try:
            resp.close()
        except OSError:
            pass


# ---------------------------------------------------------------------------
# обработчик
# ---------------------------------------------------------------------------

def make_handler(backend):
    """Класс обработчика, замкнутый на backend (приём из mission_dashboard.py:179)."""

    def call(name, *args):
        """Вызвать метод backend, честно объяснив отсутствие метода."""
        fn = getattr(backend, name, None)
        if fn is None:
            raise NotImplementedError(
                'Консоль собрана без поддержки «%s»: backend не реализует этот '
                'метод.' % name)
        return fn(*args)

    class Handler(BaseHTTPRequestHandler):
        # HTTP/1.0 (умолчание BaseHTTPRequestHandler) выбран сознательно: при
        # HTTP/1.1 с keep-alive ответ без Content-Length считается незавершённым,
        # а у SSE и у прокси-стрима длины нет по определению — страница висела бы.
        server_version = 'OperatorConsole/1'
        _stream_started = False

        def log_message(self, *_a):        # тишина: журнал запросов тут не нужен
            pass

        # -- инфраструктура ------------------------------------------------

        def _authorized(self, route):
            """True, если запрос можно исполнять.

            Токен читается на КАЖДЫЙ запрос, а не кэшируется: иначе между
            заданием токена и перезапуском консоли осталось бы окно, в котором
            защиты фактически нет. Пустой токен = проверки нет — порты и так
            публикуются только на 127.0.0.1.
            """
            if route in NO_AUTH_ROUTES:
                return True
            token = _console_token(backend)
            if not token:
                return True
            return self.headers.get('X-Console-Token') == token

        def _dispatch(self, method):
            route, params = parse_route(self.path, method)
            if not self._authorized(route):
                error_response(self, 401,
                               'Требуется заголовок X-Console-Token: консоль '
                               'защищена значением CONSOLE_TOKEN.')
                return
            self._run(route, params, method)

        def _run(self, route, params, method):
            if route == 'page':
                _send(self, 200, 'text/html; charset=utf-8', render_page())
            elif route == 'state':
                # Маршрут нужен потому, что HTTP-сервер поднимается ДО
                # rclpy.init() (под zenoh создание узла блокируется, пока роутер
                # недоступен — transport_env.sh:13-14), и «включил консоль, жду
                # робота» обязан отвечать 200 с ros_connected:false, а не
                # ERR_CONNECTION_REFUSED. У console_node для этого есть
                # отдельный state() (console_node.py:459); у минимального
                # FakeBackend его может не быть — тогда отдаём кадр SSE.
                state = getattr(backend, 'state', None)
                json_response(self, 200, state() if callable(state)
                              else call('sse_snapshot'))
            elif route == 'events':
                self._sse()
            elif route == 'config_get':
                json_response(self, 200, call('get_config'))
            elif route == 'config_set':
                json_response(self, 200, call('set_config', read_json_body(self)))
            elif route == 'vlm_get':
                json_response(self, 200, call('vlm_public'))
            elif route == 'vlm_token':
                body = read_json_body(self)
                json_response(self, 200, call('vlm_write', body.get('base_url'),
                                              body.get('model'), body.get('token')))
            elif route == 'vlm_test':
                json_response(self, 200, call('vlm_probe'))
            elif route == 'vlm_models':
                json_response(self, 200, call('vlm_models'))
            elif route == 'worlds':
                json_response(self, 200, {'worlds': call('worlds')})
            elif route == 'world_preview':
                item = call('world_preview', params['id'], params['kind'])
                if not item:
                    error_response(self, 404,
                                   'Превью мира «%s» ещё не сгенерировано. '
                                   'Выполните: ros2 run ar_project '
                                   'make_world_previews.py' % params['id'])
                else:
                    ctype, payload = item
                    _send(self, 200, ctype, payload)
            elif route == 'preflight':
                json_response(self, 200, call('preflight'))
            elif route == 'link':
                json_response(self, 200, call('link'))
            elif route == 'stack_start':
                json_response(self, 200, call('stack_start'))
            elif route == 'stack_stop':
                json_response(self, 200, call('stack_stop'))
            elif route == 'stack_status':
                json_response(self, 200, call('stack_status'))
            elif route == 'stack_log':
                json_response(self, 200, call('stack_log', self._since()))
            elif route == 'mission_start':
                body = read_json_body(self)
                json_response(self, 200, call('mission_start', str(body.get('text') or '')))
            elif route == 'mission_stop':
                json_response(self, 200, call('mission_stop'))
            elif route == ROUTE_SEED_MAP:
                json_response(self, 200, call('seed_map'))
            elif route == 'dashboard_redirect':
                # Без завершающего слэша относительные пути страницы дашборда
                # (mission_dashboard.py:355-365) резолвились бы в корень
                # консоли: "events" рядом с "/dashboard" — это "/events".
                self.send_response(301)
                self.send_header('Location', '/dashboard/')
                self.send_header('Cache-Control', 'no-store')
                self.send_header('Content-Length', '0')
                self.end_headers()
            elif route == 'dashboard':
                proxy(self, str(call('dashboard_base_url') or ''), params['sub'], method)
            else:
                error_response(self, 404, 'Маршрут не найден: %s' % self.path)

        def _since(self):
            query = urllib.parse.urlparse(self.path).query
            raw = urllib.parse.parse_qs(query).get('since', ['0'])[0]
            try:
                return int(raw)
            except (TypeError, ValueError):
                raise ValueError('Параметр since должен быть целым числом.')

        def _sse(self):
            self.send_response(200)
            self.send_header('Content-Type', 'text/event-stream')
            self.send_header('Cache-Control', 'no-store')
            self.send_header('Referrer-Policy', 'no-referrer')
            self.end_headers()
            self._stream_started = True
            try:
                while True:
                    try:
                        snapshot = call('sse_snapshot')
                    except Exception as exc:          # noqa: BLE001 — см. ниже
                        # Исключение backend НЕЛЬЗЯ поднимать наружу: заголовки
                        # уже ушли, 500 отправить некуда, и страница просто
                        # замерла бы с живым на вид соединением. Отдаём кадр с
                        # ошибкой — оператор увидит причину в панели проверок.
                        snapshot = {'error': 'Внутренняя ошибка консоли: %s' % exc}
                    payload = json.dumps(snapshot, ensure_ascii=False)
                    # Префикс 'data: ' и ДВА перевода строки обязательны, ровно
                    # как в рабочем образце mission_dashboard.py:210-213: без
                    # него EventSource молча игнорирует кадр — соединение
                    # выглядит живым, onerror не срабатывает, страница пуста.
                    self.wfile.write(('data: %s\n\n' % payload).encode('utf-8'))
                    self.wfile.flush()
                    time.sleep(SSE_PERIOD_S)
            except (BrokenPipeError, ConnectionError, OSError):
                return

        # -- точки входа ---------------------------------------------------

        def _guarded(self, method):
            self._stream_started = False
            try:
                self._dispatch(method)
            except ValueError as exc:
                self._fail(400, str(exc))
            except PermissionError as exc:
                self._fail(409, str(exc))
            except KeyError as exc:
                self._fail(404, 'Не найдено: %s' % exc)
            except NotImplementedError as exc:
                self._fail(501, str(exc))
            except (BrokenPipeError, ConnectionError):
                return                      # клиент ушёл — это норма, не ошибка
            except Exception as exc:        # noqa: BLE001 — иначе поток обработчика умрёт молча
                self._fail(500, 'Внутренняя ошибка консоли: %s' % exc)

        def _fail(self, code, message_ru):
            if self._stream_started:
                # Заголовки уже отправлены (SSE/прокси-стрим) — второй ответ
                # поверх первого дал бы браузеру мусор вместо ошибки.
                return
            try:
                error_response(self, code, message_ru)
            except (BrokenPipeError, ConnectionError, OSError):
                pass

        def do_GET(self):
            self._guarded('GET')

        def do_POST(self):
            self._guarded('POST')

    return Handler


def serve(backend, bind, port):
    """ThreadingHTTPServer, НЕ запущенный: serve_forever вызывает потребитель.

    Возврат объекта без запуска нужен console_node: сервер обязан быть создан
    (а порт — занят и проверен) ДО rclpy.init(), а крутиться он должен в
    потоке-демоне, чтобы rclpy.spin() остался в главном потоке.
    """
    return ThreadingHTTPServer((bind, port), make_handler(backend))
