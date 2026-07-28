"""Проверка связи с VLM API на голом stdlib (`urllib`).

КОНТРАКТ БЕЗОПАСНОСТИ (обязателен к соблюдению при любой правке)
================================================================
Ни одна функция этого модуля не возвращает, не логирует и не помещает в текст
исключения значение `api_key`. Ключ принимается аргументом, живёт в локальной
переменной и используется ровно в одном месте — при формировании заголовка
`Authorization`. Всякий текст, полученный от сети или из исключения, перед
возвратом наружу проходит через `fleet_comms.vlm_env.scrub(text, api_key)`.

Это не перестраховка. `urllib` охотно вкладывает в текст исключения полное
описание запроса вместе с заголовками, а результат `probe()` уходит и в HTTP-ответ
консоли, и в SSE-поток, который открыт в браузере всё время работы. Одна забытая
подстановка `%s` с исключением — и токен оказывается в DevTools.

ПОЧЕМУ ОДНА ФУНКЦИЯ `probe`, А НЕ ДВЕ
=====================================
`probe()` первым же шагом делает `GET /models` — то есть список моделей у неё уже
есть, и отдельная `list_models()` была бы вторым сетевым вызовом ради данных,
которые только что получены. Список возвращается прямо в результате `probe`.

Диагностика намеренно подробная: «ошибка связи» ничего не даёт оператору, а
«адрес не найден (404): base_url должен оканчиваться на /v1 и НЕ содержать
/chat/completions» — даёт готовое действие. Ровно эта ошибка и встречается чаще
всего, потому что документация провайдеров показывает полный URL запроса.
"""
import json
import socket
import ssl
import time
import urllib.error
import urllib.request

from fleet_comms.vlm_env import scrub

# Русские объяснения по виду ошибки. Значение — (текст, что делать).
ERROR_RU = {
    'no_base_url': ('Не задан адрес VLM API (VLM_BASE_URL).',
                    'Заполните поле «Адрес API» — например, '
                    'https://dashscope-intl.aliyuncs.com/compatible-mode/v1 для Qwen '
                    'или http://localhost:8000/v1 для локального vLLM.'),
    'no_key': ('Не задан токен VLM (VLM_API_KEY).',
               'Введите токен на шаге «VLM». Он записывается в vlm.env и обратно '
               'не показывается никогда.'),
    'no_model': ('Не выбрана модель VLM (VLM_MODEL).',
                 'Нажмите «Проверить связь» — список доступных моделей придёт в '
                 'ответе, либо впишите идентификатор модели вручную.'),
    'bad_url': ('Адрес VLM API не похож на URL.',
                'Адрес обязан начинаться с http:// или https:// и оканчиваться '
                'путём версии API (обычно /v1).'),
    'dns': ('Имя хоста в адресе VLM API не разрешается (DNS).',
            'Проверьте написание адреса и наличие DNS в контейнере: из докера '
            'внешний DNS может быть недоступен, если сеть поднята без него.'),
    'refused': ('Соединение отклонено: по этому адресу никто не слушает.',
                'Для локального сервера (vLLM/Ollama) проверьте, что он запущен и '
                'что из контейнера виден host.docker.internal, а не 127.0.0.1 — '
                'локальная петля внутри контейнера ведёт в сам контейнер.'),
    'unreachable': ('Сеть до VLM API недоступна.',
                    'Проверьте интернет-доступ хоста и, если используется прокси, '
                    'переменные HTTP_PROXY/HTTPS_PROXY внутри контейнера.'),
    'timeout': ('Таймаут ожидания ответа VLM API.',
                'Увеличьте таймаут на шаге «VLM» либо проверьте канал: тяжёлые '
                'vision-модели отвечают дольше текстовых.'),
    'tls': ('Ошибка TLS: сертификат сервера не проверился.',
            'Обычно это устаревший набор корневых сертификатов в образе или '
            'перехватывающий корпоративный прокси.'),
    'auth_401': ('Ключ отклонён (401): токен неверный или истёк.',
                 'Перевыпустите токен у провайдера и введите заново — он '
                 'применится после перезапуска стека.'),
    'forbidden_403': ('Доступ запрещён (403): ключ принят, но прав на эту модель нет.',
                      'Проверьте, что у ключа есть доступ именно к vision-модели и '
                      'что регион ключа совпадает с адресом (у Qwen intl и cn — '
                      'разные адреса и разные ключи).'),
    'not_found_404': ('Адрес не найден (404).',
                      'base_url должен оканчиваться на /v1 и НЕ содержать '
                      '/chat/completions — этот путь клиент добавляет сам.'),
    'rate_limit_429': ('Превышен лимит запросов (429).',
                       'Подождите и повторите; при работе миссии увеличьте '
                       'replan_every_n, чтобы реже дёргать модель.'),
    'server_5xx': ('Ошибка на стороне VLM API.',
                   'Это не проблема стека: повторите позже или смените модель.'),
    'http_other': ('VLM API ответил неожиданным кодом.',
                   'Смотрите текст ответа: чаще всего это отвергнутый формат '
                   'запроса у конкретного провайдера.'),
    'bad_json': ('Ответ VLM API не является корректным JSON.',
                 'Похоже, по адресу отвечает не OpenAI-совместимый сервер '
                 '(например, страница прокси или портала авторизации).'),
    'model_missing': ('Запрошенная модель недоступна по этому ключу.',
                      'Выберите модель из списка, который вернул сервер.'),
}

_HTTP_KIND = {
    401: 'auth_401',
    403: 'forbidden_403',
    404: 'not_found_404',
    429: 'rate_limit_429',
}


def _error(kind, detail='', **extra):
    """Собрать отказ. `detail` уже должен быть очищен от секретов."""
    text, hint = ERROR_RU.get(kind, ('Неизвестная ошибка проверки VLM.', ''))
    if detail:
        text = '%s %s' % (text, detail)
    result = {
        'ok': False,
        'checked_at': time.time(),
        'latency_ms': 0,
        'http_status': None,
        'models': [],
        'model_ok': False,
        'error_kind': kind,
        'error_ru': text,
        'hint_ru': hint,
        'note_ru': '',
    }
    result.update(extra)
    return result


def normalize_base_url(base_url):
    """(нормализованный адрес, список русских предупреждений).

    Отрезание хвоста `/chat/completions` — не косметика: клиент оркестратора
    приклеивает этот путь сам, и с полным URL из документации провайдера
    запрос уходит на `/v1/chat/completions/chat/completions` и получает 404.
    """
    warnings = []
    url = (base_url or '').strip().rstrip('/')
    if not url:
        return '', warnings
    for tail in ('/chat/completions', '/completions'):
        if url.endswith(tail):
            url = url[:-len(tail)].rstrip('/')
            warnings.append('Из адреса убран хвост %s — клиент добавляет его сам.'
                            % tail)
            break
    return url, warnings


def _request(url, api_key, timeout_s, body=None):
    """(status, raw_bytes). Единственное место, где формируется Authorization."""
    data = None
    headers = {'Accept': 'application/json'}
    if api_key:
        headers['Authorization'] = 'Bearer ' + api_key
    if body is not None:
        data = json.dumps(body).encode('utf-8')
        headers['Content-Type'] = 'application/json'
    request = urllib.request.Request(url, data=data, headers=headers,
                                     method='POST' if data else 'GET')
    try:
        with urllib.request.urlopen(request, timeout=timeout_s) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as exc:
        # HTTPError — это ответ, а не сбой сети: код и тело нужны для диагноза.
        try:
            payload = exc.read()
        except Exception:
            payload = b''
        return exc.code, payload


def classify_exception(exc):
    """Вид сетевой ошибки по исключению. Отдельная функция — чтобы её можно было
    проверить тестом, не поднимая сервер."""
    if isinstance(exc, urllib.error.HTTPError):
        return classify_status(exc.code)
    if isinstance(exc, ssl.SSLError):
        return 'tls'
    if isinstance(exc, socket.gaierror):
        return 'dns'
    if isinstance(exc, ConnectionRefusedError):
        return 'refused'
    if isinstance(exc, (socket.timeout, TimeoutError)):
        return 'timeout'
    if isinstance(exc, urllib.error.URLError):
        reason = getattr(exc, 'reason', None)
        if reason is not None and not isinstance(reason, str):
            return classify_exception(reason)
        text = str(reason or '').lower()
        if 'timed out' in text or 'timeout' in text:
            return 'timeout'
        if 'name or service not known' in text or 'nodename' in text:
            return 'dns'
        if 'refused' in text:
            return 'refused'
        if 'certificate' in text or 'ssl' in text:
            return 'tls'
        return 'unreachable'
    if isinstance(exc, OSError):
        return 'unreachable'
    return 'unreachable'


def classify_status(status):
    """Вид ошибки по HTTP-коду. 2xx сюда не попадает."""
    if status in _HTTP_KIND:
        return _HTTP_KIND[status]
    if 500 <= status <= 599:
        return 'server_5xx'
    return 'http_other'


def _extract_models(raw):
    """Идентификаторы моделей из ответа /models или None, если формат чужой.

    Пустой/чужой ответ — НЕ ошибка: множество OpenAI-совместимых серверов
    (в том числе локальный vLLM в некоторых сборках) /models просто не
    реализуют, и валить на этом проверку было бы неправильно.
    """
    try:
        payload = json.loads(raw.decode('utf-8', 'replace'))
    except (ValueError, AttributeError):
        return None
    items = payload.get('data') if isinstance(payload, dict) else None
    if not isinstance(items, list):
        return None
    models = []
    for item in items:
        if isinstance(item, dict) and item.get('id'):
            models.append(str(item['id']))
        elif isinstance(item, str):
            models.append(item)
    return models


def probe(base_url, api_key, model, timeout_s=12.0):
    """Двухшаговая проверка доступа к VLM API.

    Шаг 1 — `GET /models`: доказывает, что адрес живой и ключ принимается, и
    заодно приносит список моделей (отдельного запроса за списком нет).
    Шаг 2 — минимальный `POST /chat/completions` с `max_tokens=1`: доказывает,
    что доступна именно ВЫБРАННАЯ модель. Список из шага 1 этого не доказывает —
    у многих провайдеров он общий для всего аккаунта, а права выданы на часть.
    Запрос отправляется ровно один раз и стоит доли копейки.

    Возвращает словарь без ключа и без единого фрагмента ключа в текстах.
    """
    api_key = api_key or ''
    url, warnings = normalize_base_url(base_url)
    note = ' '.join(warnings)

    if not url:
        return _error('no_base_url')
    if not (url.startswith('http://') or url.startswith('https://')):
        return _error('bad_url', 'Получено: %r.' % url[:120])
    if not api_key.strip():
        return _error('no_key', note_ru=note)
    if not (model or '').strip():
        return _error('no_model', note_ru=note)

    started = time.monotonic()

    # --- шаг 1: список моделей ---
    try:
        status, raw = _request(url + '/models', api_key, timeout_s)
    except Exception as exc:  # сеть: любое исключение превращается в диагноз
        kind = classify_exception(exc)
        detail = scrub(str(exc), api_key)
        return _error(kind, 'Подробности: %s.' % detail if detail else '',
                      latency_ms=_ms(started), note_ru=note)

    models = []
    if status == 200:
        parsed = _extract_models(raw)
        models = parsed if parsed is not None else []
    elif status in (401, 403):
        # На этих кодах дальше идти незачем: ключ уже отвергнут.
        return _error(classify_status(status),
                      _body_hint(raw, api_key),
                      http_status=status, latency_ms=_ms(started), note_ru=note)
    elif status == 404:
        # /models может быть не реализован — это не приговор адресу, поэтому
        # переходим ко второму шагу и судим по нему.
        note = (note + ' Сервер не отдаёт список моделей (404 на /models) — '
                       'проверка выполнена одним запросом к модели.').strip()
    else:
        note = (note + ' Список моделей недоступен (HTTP %d).' % status).strip()

    model_listed = (not models) or (model in models)

    # --- шаг 2: короткий запрос к самой модели ---
    body = {
        'model': model,
        'messages': [{'role': 'user', 'content': 'ping'}],
        'max_tokens': 1,
        'temperature': 0,
    }
    try:
        status2, raw2 = _request(url + '/chat/completions', api_key, timeout_s,
                                 body=body)
    except Exception as exc:
        kind = classify_exception(exc)
        detail = scrub(str(exc), api_key)
        return _error(kind, 'Подробности: %s.' % detail if detail else '',
                      models=models, latency_ms=_ms(started), note_ru=note)

    latency = _ms(started)

    if status2 == 200:
        if not model_listed:
            note = (note + ' Модель %s отсутствует в списке /models, но запрос к '
                           'ней прошёл — список у этого сервера неполный.'
                    % model).strip()
        return {
            'ok': True,
            'checked_at': time.time(),
            'latency_ms': latency,
            'http_status': 200,
            'models': models,
            'model_ok': True,
            'error_kind': '',
            'error_ru': '',
            'hint_ru': '',
            'note_ru': note,
        }

    # Модель не принята: 404/400 на этом шаге при исправном шаге 1 означает
    # именно неверный идентификатор модели, а не неверный адрес.
    if status2 in (400, 404) and not model_listed:
        text, hint = ERROR_RU['model_missing']
        return {
            'ok': False,
            'checked_at': time.time(),
            'latency_ms': latency,
            'http_status': status2,
            'models': models,
            'model_ok': False,
            'error_kind': 'model_missing',
            'error_ru': 'Модель %s недоступна по этому ключу (HTTP %d). %s'
                        % (model, status2, _body_hint(raw2, api_key)),
            'hint_ru': (hint + (' Доступны: %s.' % ', '.join(models[:12])
                                if models else '')).strip(),
            'note_ru': note,
        }

    kind = classify_status(status2)
    result = _error(kind, _body_hint(raw2, api_key), http_status=status2,
                    models=models, latency_ms=latency, note_ru=note)
    if kind == 'server_5xx':
        result['error_ru'] = result['error_ru'].replace(
            'Ошибка на стороне VLM API.', 'Ошибка на стороне VLM API (%d).' % status2)
    return result


def _body_hint(raw, api_key):
    """Короткая выжимка из тела ответа — обрезанная и очищенная от секрета."""
    try:
        text = (raw or b'').decode('utf-8', 'replace').strip()
    except Exception:
        return ''
    if not text:
        return ''
    try:
        payload = json.loads(text)
        error = payload.get('error') if isinstance(payload, dict) else None
        if isinstance(error, dict) and error.get('message'):
            text = str(error['message'])
        elif isinstance(error, str):
            text = error
    except ValueError:
        pass
    text = ' '.join(text.split())[:300]
    return 'Ответ сервера: %s' % scrub(text, api_key)


def _ms(started):
    return int((time.monotonic() - started) * 1000.0)


def redact_for_log(result):
    """То, что разрешено писать в лог консоли.

    Из результата выбрасываются ВСЕ текстовые поля, пришедшие от сети: даже
    пройдя scrub, они остаются чужим текстом, а лог узла ROS уходит в файл и в
    журнал контейнера, откуда его никто уже не вычистит.
    """
    result = result or {}
    return {
        'ok': bool(result.get('ok')),
        'http_status': result.get('http_status'),
        'latency_ms': result.get('latency_ms'),
        'error_kind': result.get('error_kind', ''),
        'model_ok': bool(result.get('model_ok')),
        'models_count': len(result.get('models') or []),
    }
