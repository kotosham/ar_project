"""Сессионный конфиг консоли (JSON) и write-only доступ к `vlm.env`.

ГЛАВНАЯ ГАРАНТИЯ
================
Токен физически не может попасть ни в JSON-конфиг, ни в HTTP-ответ. Достигается
не аккуратностью вызывающего, а конструкцией:

  * `FORBIDDEN_KEYS` + эвристика по подстроке отвергают любой patch, в котором
    есть что-то похожее на секрет, — то есть JSON-файл просто не может получить
    такое поле;
  * креды читаются исключительно через `vlm_env.public_view()`, которая по
    контракту возвращает только `token_set: bool`;
  * `get()` перед возвратом прогоняет результат через `_assert_no_secret` —
    это дешёвая страховка от будущей правки, которая добавит поле не подумав.

ЧТО КОНСОЛЬ НЕ РЕДАКТИРУЕТ
==========================
`edge_host` и `ros_domain_id` убраны из редактируемых ключей намеренно. Оба
фиксируются окружением контейнера в момент старта процесса: `EDGE_HOST`
подставляется в `ZENOH_CONFIG_OVERRIDE` до создания сессии zenoh, а
`ROS_DOMAIN_ID` читается rmw при инициализации. Запись их в JSON создавала бы
поле, которое выглядит как настройка, но ни на что не влияет до пересоздания
контейнера, — а это худший вид интерфейса. Они отдаются read-only с пометкой
«задаётся в docker/.env».
"""
import json
import os
import tempfile

from fleet_comms.vlm_env import (
    KEY_API_KEY,
    KEY_BASE_URL,
    KEY_MODEL,
    load_env_file,
    public_view,
    write_env_file,
)

DEFAULT_CONFIG = {
    'mode': 'sim',
    'planner': 'vlm',
    'world': 'house',
    'gui': False,
    'rviz': False,
    'layer': 'all',
    'dashboard_url': 'http://127.0.0.1:8088',
    'max_steps': 40,
    'replan_every_n': 1,
    'vlm_timeout_s': 30.0,
    'venv_python': '~/.venvs/ros-jazzy-ml/bin/python',
    # Консоль поднимает верхний слой сама только тогда, когда его больше никто не
    # поднимает: docker-compose уже содержит сервисы detector/orchestrator в
    # профилях edge/all, и два экземпляра оркестратора дерутся за /vlm_mission.
    'owns_edge_layer': True,
    # Симуляция по умолчанию рендерит камеру 320x240 (тяжёлый профиль роняет RTF
    # в WSL2 и рвёт синхронизацию RTAB-Map). Флаг включает разрешение железа
    # ради сопоставимости метрик — сознательно и с видимой ценой.
    'hw_parity_camera': False,
}

ENUMS = {
    'mode': ('sim', 'hardware'),
    'planner': ('vlm', 'flat', 'mock'),
    'layer': ('robot', 'edge', 'all'),
}

BOOL_KEYS = ('gui', 'rviz', 'owns_edge_layer', 'hw_parity_camera')
INT_KEYS = {'max_steps': (1, 500), 'replan_every_n': (1, 100)}
FLOAT_KEYS = {'vlm_timeout_s': (1.0, 600.0)}
PATH_KEYS = ('venv_python',)

ALLOWED_KEYS = frozenset(DEFAULT_CONFIG)

# Явный чёрный список плюс эвристика ниже: явный список читается и проверяется
# тестом, эвристика ловит то, что придумают позже.
FORBIDDEN_KEYS = frozenset({
    'vlm_api_key', 'api_key', 'apikey', 'token', 'vlm_token', 'key', 'vlm_key',
    'secret', 'password', 'authorization', 'bearer',
})
_SECRET_SUBSTRINGS = ('token', 'secret', 'passw', 'api_key', 'apikey',
                      'authorization', 'bearer')
# Единственное поле, которому разрешено содержать 'token': признак наличия ключа.
_SECRET_ALLOWLIST = ('token_set',)

# Read-only значения, которые UI показывает рядом с настройками.
ENV_READONLY_NOTE_RU = 'задаётся в docker/.env и применяется при старте контейнера'


def _is_forbidden(key):
    lowered = str(key).strip().lower()
    if lowered in _SECRET_ALLOWLIST:
        return False
    if lowered in FORBIDDEN_KEYS:
        return True
    return any(part in lowered for part in _SECRET_SUBSTRINGS)


def _assert_no_secret(payload):
    """Страховка от будущей правки: ни одного «секретного» имени поля наружу."""
    def walk(node, path):
        if isinstance(node, dict):
            for key, value in node.items():
                if _is_forbidden(key):
                    raise RuntimeError(
                        'внутренняя ошибка консоли: поле %s%s похоже на секрет и '
                        'не должно отдаваться наружу' % (path, key))
                walk(value, '%s%s.' % (path, key))
        elif isinstance(node, list):
            for item in node:
                walk(item, path)
    walk(payload, '')
    return payload


def _as_bool(value, key, errors):
    if isinstance(value, bool):
        return value
    text = str(value).strip().lower()
    if text in ('true', '1', 'yes', 'on', 'да'):
        return True
    if text in ('false', '0', 'no', 'off', 'нет'):
        return False
    errors.append('%s: ожидается да/нет, получено %r' % (key, value))
    return None


def validate_patch(patch, world_ids):
    """(очищенный патч, список русских ошибок). Ничего не бросает сама."""
    clean = {}
    errors = []
    if not isinstance(patch, dict):
        return {}, ['тело запроса должно быть объектом JSON']

    for key, value in patch.items():
        if _is_forbidden(key):
            errors.append('Токен нельзя передавать через /api/config — для него '
                          'есть отдельный маршрут /api/vlm/token, который пишет '
                          'значение прямо в vlm.env и никогда не возвращает его '
                          'обратно (поле: %s).' % key)
            continue
        if key in ('edge_host', 'ros_domain_id'):
            errors.append('%s консолью не настраивается: %s. Изменение здесь '
                          'выглядело бы как настройка, но ни на что не влияло '
                          'бы до пересоздания контейнера.'
                          % (key, ENV_READONLY_NOTE_RU))
            continue
        if key not in ALLOWED_KEYS:
            errors.append('неизвестный параметр: %s' % key)
            continue

        if key in ENUMS:
            text = str(value).strip()
            if text not in ENUMS[key]:
                errors.append('%s: допустимо %s' % (key, ' | '.join(ENUMS[key])))
                continue
            clean[key] = text
        elif key in BOOL_KEYS:
            result = _as_bool(value, key, errors)
            if result is not None:
                clean[key] = result
        elif key in INT_KEYS:
            low, high = INT_KEYS[key]
            try:
                number = int(value)
            except (TypeError, ValueError):
                errors.append('%s: ожидается целое число' % key)
                continue
            if not low <= number <= high:
                errors.append('%s: допустимый диапазон %d..%d' % (key, low, high))
                continue
            clean[key] = number
        elif key in FLOAT_KEYS:
            low, high = FLOAT_KEYS[key]
            try:
                number = float(value)
            except (TypeError, ValueError):
                errors.append('%s: ожидается число' % key)
                continue
            if not low <= number <= high:
                errors.append('%s: допустимый диапазон %.1f..%.1f' % (key, low, high))
                continue
            clean[key] = number
        elif key == 'world':
            text = str(value).strip()
            if world_ids and text not in world_ids:
                errors.append('мир %s отсутствует в каталоге (доступны: %s)'
                              % (text, ', '.join(world_ids)))
                continue
            clean[key] = text
        elif key == 'dashboard_url':
            text = str(value).strip().rstrip('/')
            if not (text.startswith('http://') or text.startswith('https://')):
                errors.append('dashboard_url: адрес должен начинаться с http:// '
                              'или https://')
                continue
            clean[key] = text
        elif key in PATH_KEYS:
            # Тильда раскрывается ЗДЕСЬ, а не при запуске: путь уходит в argv
            # дочернего процесса, где никакой оболочки, которая её раскроет, нет
            # (vlm_sim_bringup.launch.py:70 наступил ровно на это).
            clean[key] = os.path.expanduser(str(value).strip())
        else:
            clean[key] = value
    return clean, errors


class ConfigStore:
    """JSON-конфиг сессии плюс единственная точка записи в vlm.env."""

    def __init__(self, config_path, vlm_env_path, vlm_env_template=''):
        self.config_path = os.path.expanduser(config_path or '')
        self.vlm_env_path = os.path.expanduser(vlm_env_path or '')
        self.vlm_env_template = os.path.expanduser(vlm_env_template or '')
        self.loaded_default = True
        self._cfg = dict(DEFAULT_CONFIG)
        self.load()

    # -- JSON ---------------------------------------------------------------

    def load(self):
        """Прочитать конфиг. Битый или отсутствующий файл — не исключение:
        консоль обязана подниматься всегда, иначе оператор остаётся без
        единственного инструмента, которым мог бы это починить."""
        cfg = dict(DEFAULT_CONFIG)
        self.loaded_default = True
        try:
            with open(self.config_path, 'r', encoding='utf-8') as handle:
                data = json.load(handle)
            if isinstance(data, dict):
                for key, value in data.items():
                    if key in ALLOWED_KEYS and not _is_forbidden(key):
                        cfg[key] = value
                self.loaded_default = False
        except (OSError, ValueError):
            pass
        self._cfg = cfg
        return dict(cfg)

    def save(self):
        """Атомарная запись: tmp в ТОМ ЖЕ каталоге, иначе os.replace через
        границу файловых систем (а bind-mount докера — ровно она) даст EXDEV."""
        if not self.config_path:
            return
        directory = os.path.dirname(self.config_path) or '.'
        try:
            os.makedirs(directory, exist_ok=True)
        except OSError:
            pass
        payload = _assert_no_secret(dict(self._cfg))
        tmp_name = None
        try:
            with tempfile.NamedTemporaryFile('w', encoding='utf-8', delete=False,
                                             dir=directory,
                                             prefix='.operator_console.') as tmp:
                tmp_name = tmp.name
                json.dump(payload, tmp, ensure_ascii=False, indent=2)
                tmp.write('\n')
            os.replace(tmp_name, self.config_path)
            tmp_name = None
        finally:
            if tmp_name:
                try:
                    os.unlink(tmp_name)
                except OSError:
                    pass

    # -- чтение наружу ------------------------------------------------------

    def get(self):
        """Полный вид конфига для HTTP. Гарантированно без токена."""
        out = dict(self._cfg)
        out['vlm'] = self.vlm_public()
        out['environment'] = self.environment_view()
        out['vlm_env_path'] = self.vlm_env_path
        out['vlm_env_exists'] = self.vlm_env_exists()
        return _assert_no_secret(out)

    def environment_view(self):
        """Read-only значения из окружения контейнера — показываются рядом с
        настройками с пометкой, что правятся они в docker/.env."""
        domain = os.environ.get('ROS_DOMAIN_ID')
        try:
            domain = int(domain) if domain not in (None, '') else None
        except ValueError:
            domain = None
        return {
            'edge_host': os.environ.get('EDGE_HOST', ''),
            'ros_domain_id': domain,
            'rmw': os.environ.get('RMW_IMPLEMENTATION', ''),
            'readonly': True,
            'note_ru': ENV_READONLY_NOTE_RU,
        }

    def patch(self, d, world_ids):
        clean, errors = validate_patch(d, world_ids)
        if errors:
            raise ValueError('; '.join(errors))
        self._cfg.update(clean)
        self.save()
        return self.get()

    # -- vlm.env ------------------------------------------------------------

    def vlm_public(self):
        """Ровно {base_url, model, token_set} — единственная разрешённая форма."""
        return public_view(load_env_file(self.vlm_env_path))

    def vlm_env_exists(self):
        return bool(self.vlm_env_path) and os.path.isfile(self.vlm_env_path)

    def vlm_write(self, base_url=None, model=None, token=None):
        """Записать креды в vlm.env. Токен не возвращается ни при каких условиях.

        None означает «не трогать ключ», пустая строка — «очистить»: без этого
        различения нельзя было бы сменить модель, не переписывая токен заново.
        """
        warnings = []
        updates = {}
        if base_url is not None:
            url = str(base_url).strip().rstrip('/')
            for tail in ('/chat/completions', '/completions'):
                if url.endswith(tail):
                    url = url[:-len(tail)].rstrip('/')
                    warnings.append(
                        'Из адреса убран хвост %s: клиент добавляет его сам, и с '
                        'полным URL из документации провайдера запрос уходит на '
                        '.../chat/completions/chat/completions и получает 404.'
                        % tail)
                    break
            if url and not (url.startswith('http://') or url.startswith('https://')):
                warnings.append('Адрес не начинается с http:// или https:// — '
                                'проверка связи почти наверняка не пройдёт.')
            updates[KEY_BASE_URL] = url
        if model is not None:
            updates[KEY_MODEL] = str(model).strip()
        if token is not None:
            updates[KEY_API_KEY] = str(token).strip()

        if updates:
            write_env_file(self.vlm_env_path, updates,
                           template_path=self.vlm_env_template)

        result = dict(self.vlm_public())
        result['restart_required'] = True
        result['note_ru'] = (
            'Значения записаны в vlm.env. Оркестратор читает их при СТАРТЕ '
            'процесса, поэтому изменения применятся только после перезапуска '
            'стека: в симуляции — кнопкой «Применить и перезапустить», на железе '
            '— перезапуском orchestrator_node на edge-боксе. До перезапуска '
            'оркестратор продолжит работать со старыми кредами.')
        result['warnings'] = warnings
        return _assert_no_secret(result)

    # -- окружение дочернего процесса --------------------------------------

    def as_launch_env(self):
        """Переменные для дочернего `ros2 launch`.

        Здесь передаётся ПУТЬ к файлу, а не его содержимое: значение ключа
        подхватит edge_layer.launch.py через load_env_file уже внутри своего
        процесса. Ключ в argv был бы виден в `ps` любому пользователю хоста, а
        ключ в env родителя пришлось бы держать в памяти консоли всё время её
        работы.
        """
        env = {}
        if self.vlm_env_path:
            env['VLM_ENV_FILE'] = self.vlm_env_path
        return env
