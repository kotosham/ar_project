"""Работа с `vlm.env` — файлом, в котором живут креды VLM.

ЖЁСТКИЙ КОНТРАКТ БЕЗОПАСНОСТИ
=============================
`public_view()` — ЕДИНСТВЕННАЯ функция во всём решении, результат которой
разрешено отдавать наружу (по HTTP, в лог, в отчёт). Значение `VLM_API_KEY` не
возвращается ни одной другой функцией, не попадает в ROS-параметры, не
передаётся аргументом командной строки (там его видно в `ps`) и не логируется.
Запись — есть; чтение наружу — нет.

Почему модуль вообще нужен, хотя `vlm_client.resolve_credentials` уже читает
переменные окружения: окружение заполняется ОДИН раз при старте процесса, а
консоли надо (а) показать оператору, задан ли ключ, не показывая ключ, и
(б) записать введённый ключ обратно в файл, не потеряв документацию, которой
`vlm.env.example` объясняет формат `VLM_BASE_URL`. Оба действия — файловые,
и ни одно из них не должно уметь вернуть секрет.

Формат файла — тот, что уже описан в `vlm.env.example`: строки `KEY=VALUE`,
пригодные для `set -a; source vlm.env; set +a` и для docker `env_file`.
"""
import os
import tempfile

KEY_BASE_URL = 'VLM_BASE_URL'
KEY_API_KEY = 'VLM_API_KEY'
KEY_MODEL = 'VLM_MODEL'
SECRET_KEYS = frozenset({KEY_API_KEY})


def parse_env(text):
    """Разбор `KEY=VALUE`-файла. Дубликаты — побеждает последний.

    Терпимость к формату здесь не роскошь: файл правят руками, и он уже
    приезжал с CRLF-окончаниями (тогда каждое значение получало хвостовой
    '\\r', а bash ругался `$'\\r': command not found`). Поэтому строка
    очищается от пробельных символов с обоих концов ДО разбора.
    """
    out = {}
    for raw in (text or '').splitlines():
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('export '):
            line = line[len('export '):].lstrip()
        if '=' not in line:
            continue
        key, _, value = line.partition('=')
        key = key.strip()
        if not key:
            continue
        value = value.strip()
        if len(value) >= 2 and value[0] == value[-1] and value[0] in ('"', "'"):
            value = value[1:-1]
        out[key] = value.strip()
    return out


def load_env_file(path):
    """Содержимое файла как словарь; {} если файла нет или он нечитаем.

    Исключений наружу нет намеренно: отсутствие vlm.env — штатное состояние
    (стек тогда работает в mock-режиме), а не ошибка, которую надо показывать.
    """
    try:
        with open(os.path.expanduser(path), 'r', encoding='utf-8',
                  errors='replace') as handle:
            return parse_env(handle.read())
    except OSError:
        return {}


def render_env(existing_text, updates):
    """Подменить значения ПО МЕСТУ, сохранив порядок строк и комментарии.

    Сохранять комментарии обязательно: `vlm.env.example` — это не просто список
    ключей, а инструкция (какой именно base_url подставлять для Qwen/OpenAI/
    локального vLLM). Переписав файл «начисто», консоль молча уничтожила бы
    единственное место, где это записано.

    В `updates`: None — не трогать ключ, '' — очистить значение.
    """
    updates = {k: v for k, v in (updates or {}).items() if v is not None}
    if not updates:
        return existing_text or ''
    lines = (existing_text or '').splitlines()
    seen = set()
    for i, raw in enumerate(lines):
        stripped = raw.strip()
        if not stripped or stripped.startswith('#') or '=' not in stripped:
            continue
        body = stripped[len('export '):].lstrip() if stripped.startswith('export ') else stripped
        key = body.partition('=')[0].strip()
        if key in updates:
            lines[i] = '%s=%s' % (key, _quote(updates[key]))
            seen.add(key)
    for key, value in updates.items():
        if key not in seen:
            lines.append('%s=%s' % (key, _quote(value)))
    return '\n'.join(lines) + '\n'


def write_env_file(path, updates, template_path=''):
    """Атомарно записать значения в vlm.env. Ничего не возвращает и не логирует.

    Временный файл ОБЯЗАН лежать в том же каталоге, что и целевой: `os.replace`
    через границу файловых систем даёт EXDEV, а внутри docker bind-mount это
    ровно тот случай. По той же причине в compose монтируется КАТАЛОГ
    `planner_orchestrator`, а не сам файл `vlm.env`: переименование поверх точки
    монтирования одиночного файла возвращает EBUSY, и смена inode всё равно не
    была бы видна хосту.
    """
    path = os.path.expanduser(path)
    directory = os.path.dirname(path) or '.'
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as handle:
            existing = handle.read()
    except OSError:
        existing = ''
        if template_path:
            try:
                with open(os.path.expanduser(template_path), 'r',
                          encoding='utf-8', errors='replace') as handle:
                    existing = handle.read()
            except OSError:
                existing = ''
    text = render_env(existing, updates)
    tmp_name = None
    try:
        with tempfile.NamedTemporaryFile('w', encoding='utf-8', delete=False,
                                         dir=directory, prefix='.vlm.env.') as tmp:
            tmp_name = tmp.name
            tmp.write(text)
        os.replace(tmp_name, path)
        tmp_name = None
    finally:
        if tmp_name:
            try:
                os.unlink(tmp_name)
            except OSError:
                pass
    # 0600 — best-effort: на Windows-bind-mount chmod не срабатывает, и это не
    # повод считать запись неудачной.
    try:
        os.chmod(path, 0o600)
    except OSError:
        pass


def public_view(env):
    """ЕДИНСТВЕННАЯ функция, результат которой разрешено отдавать наружу.

    Возвращает адрес, модель и ПРИЗНАК наличия ключа. Само значение ключа не
    возвращается никогда — ни целиком, ни хвостом, ни длиной: длина сужает
    перебор, а пользы оператору не даёт.
    """
    env = env or {}
    return {
        'base_url': (env.get(KEY_BASE_URL) or '').strip(),
        'model': (env.get(KEY_MODEL) or '').strip(),
        'token_set': bool((env.get(KEY_API_KEY) or '').strip()),
    }


def scrub(text, *secrets):
    """Вычистить секреты из текста перед показом человеку.

    Нужна для сообщений об ошибках: `urllib` охотно вкладывает в текст
    исключения весь запрос вместе с заголовком Authorization. Короче 8 символов
    не заменяем — такое «значение» скорее мусор, а глобальная замена коротких
    строк изуродует нормальный текст.
    """
    out = text or ''
    for secret in secrets:
        secret = (secret or '').strip()
        if len(secret) >= 8:
            out = out.replace(secret, '***')
    return out


def _quote(value):
    value = '' if value is None else str(value)
    if value == '' or any(ch in value for ch in ' \t#"\'$'):
        return "'%s'" % value.replace("'", "'\\''")
    return value
