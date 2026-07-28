"""Каталог миров симуляции: чтение `config/worlds.yaml`, резолвинг путей, валидация.

ЗАЧЕМ ЭТОТ МОДУЛЬ
=================
Реестра миров в репозитории не было: `flat_sim_bringup.launch.py:68` умеет только
подставить путь к `.sdf`, а какие миры вообще пригодны — знание устное. При этом
часть миров в `ar_project/worlds/` физически не загружается (`demo1.sdf` ссылается
на абсолютные пути Fuel, `obstacles.world` — на модель `construction_barrel`,
которой нет в `ar_project/models`), и оператор узнаёт об этом только по пустому
чёрному окну Gazebo. Каталог делает пригодность машиночитаемой, а консоль —
способной честно сказать «этот мир сломан и вот почему».

ВТОРАЯ ЛОВУШКА, РАДИ КОТОРОЙ ЕСТЬ `world_name`
==============================================
gz-сервисы адресуются как `/world/<world name>/create|light_config|set_pose`, и
`house_scenario_runner.py` шлёт запросы именно туда. Внутри `.sdf` большинство
миров называются `default`, а вовсе не как файл. Ошибка в этом поле не ломает
запуск Gazebo — она молча ломает управление сценой (свет не гаснет, объекты не
появляются), поэтому `validate()` сверяет каталог с содержимым файла.

ФОРМАТ КОМНАТ
=============
Комнаты пишутся как `{x: [x0, x1], y: [y0, y1]}`, а не плоским списком из четырёх
чисел: источник истины — `house_scenario_runner.py:89`, где кортеж хранится в
порядке `(x_min, x_max, y_min, y_max)`. Плоский список `[x0, y0, x1, y1]` читается
«естественно» и потому неизбежно перепутает оси при первой же правке руками.

PyYAML импортируется ЛЕНИВО, внутри `load_catalog_file`: `parse_catalog()`
принимает готовый словарь, поэтому юнит-тесты и офлайн-разбор каталога не зависят
ни от одного внешнего пакета. Модуль не импортирует rclpy — он обязан работать
обычным python.
"""
import os
import re
from dataclasses import dataclass

CATALOG_VERSION = 1

STATUS_OK = 'ok'
STATUS_LIMITED = 'limited'
STATUS_BROKEN = 'broken'
STATUSES = (STATUS_OK, STATUS_LIMITED, STATUS_BROKEN)

# Русские подписи статусов — одна строка на бейдж в карточке мира.
STATUS_RU = {
    STATUS_OK: 'пригоден',
    STATUS_LIMITED: 'с оговорками',
    STATUS_BROKEN: 'не загружается',
}

_REQUIRED_FIELDS = ('id', 'title', 'file', 'world_name', 'status')

# Регулярка вместо полного XML-разбора намеренно: миры весят до 65 КБ, атрибут
# лежит в первых строках, а битый SDF (ради которого проверка и нужна) свалил бы
# ET.parse исключением ровно там, где нам нужно вернуть человеку объяснение.
_WORLD_NAME_RE = re.compile(r"<world\s+name=['\"]([^'\"]+)['\"]")
_HEAD_BYTES = 64 * 1024


@dataclass(frozen=True)
class WorldEntry:
    """Одна запись каталога. Неизменяемая: каталог читается один раз при старте
    консоли и раздаётся всем потребителям, случайная мутация испортила бы её всем."""

    id: str
    title: str
    file: str
    world_name: str
    status: str
    status_note: str = ''
    size_m: tuple = (0.0, 0.0)
    origin_m: tuple = (0.0, 0.0)
    default_spawn: dict = None
    rooms: dict = None
    preview: str = None
    preview_png: str = None
    scenarios_supported: bool = False
    notes: str = ''

    @property
    def usable(self):
        """Мир можно предлагать оператору. `limited` пригоден — просто с оговоркой,
        которую UI обязан показать рядом (например, серые стены test_1)."""
        return self.status != STATUS_BROKEN

    @property
    def status_ru(self):
        return STATUS_RU.get(self.status, self.status)

    def to_public(self, share_dir):
        """Форма для HTTP. `preview_url` подставляется только если файл РЕАЛЬНО
        существует: путь в каталоге — это намерение, а превью генерируется
        отдельным офлайн-скриптом, и до первого его прогона файлов нет."""
        paths = resolve_paths(self, share_dir)
        preview_url = None
        if paths['preview'] and os.path.exists(paths['preview']):
            preview_url = '/api/worlds/%s/preview.svg' % self.id
        elif paths['preview_png'] and os.path.exists(paths['preview_png']):
            preview_url = '/api/worlds/%s/preview.png' % self.id
        return {
            'id': self.id,
            'title': self.title,
            'world_name': self.world_name,
            'status': self.status,
            'status_ru': self.status_ru,
            'status_note': self.status_note,
            'size_m': list(self.size_m),
            'origin_m': list(self.origin_m),
            'default_spawn': dict(self.default_spawn or {}),
            'rooms': list((self.rooms or {}).keys()),
            'rooms_count': len(self.rooms or {}),
            'scenarios_supported': bool(self.scenarios_supported),
            'notes': self.notes,
            'preview_url': preview_url,
            'usable': self.usable,
        }


class Catalog:
    """Список записей в порядке файла плюс выбор мира по умолчанию."""

    def __init__(self, entries, default_id):
        self._entries = list(entries)
        self._by_id = dict((e.id, e) for e in self._entries)
        self._default_id = default_id

    def list(self):
        return list(self._entries)

    def usable(self):
        return [e for e in self._entries if e.usable]

    def get(self, world_id):
        try:
            return self._by_id[world_id]
        except KeyError:
            raise KeyError('мир не найден в каталоге: %s' % (world_id,))

    def ids(self):
        return [e.id for e in self._entries]

    def __len__(self):
        return len(self._entries)

    @property
    def default(self):
        entry = self._by_id.get(self._default_id)
        if entry is not None:
            return entry
        usable = self.usable()
        if usable:
            return usable[0]
        if self._entries:
            return self._entries[0]
        raise KeyError('каталог миров пуст')


def parse_catalog(data):
    """Разбор готового словаря в Catalog. PyYAML здесь не нужен намеренно.

    Валидация строгая и падает с русским текстом: молча проглоченная опечатка в
    `world_name` или `status` даст не ошибку, а неверное поведение сцены через
    несколько минут после запуска — отлаживать это несопоставимо дороже.
    """
    if not isinstance(data, dict):
        raise ValueError('каталог миров должен быть словарём, получено: %s'
                         % type(data).__name__)
    version = data.get('version')
    if version != CATALOG_VERSION:
        raise ValueError('неподдерживаемая версия каталога миров: %r' % (version,))

    raw_entries = data.get('worlds') or []
    if not isinstance(raw_entries, list):
        raise ValueError('поле worlds должно быть списком записей')

    entries = []
    seen = set()
    for index, raw in enumerate(raw_entries):
        if not isinstance(raw, dict):
            raise ValueError('запись каталога №%d не является словарём' % (index + 1,))
        world_id = str(raw.get('id') or '').strip()
        for field_name in _REQUIRED_FIELDS:
            value = raw.get(field_name)
            if value is None or str(value).strip() == '':
                raise ValueError('мир %s: не заполнено обязательное поле %s'
                                 % (world_id or '№%d' % (index + 1), field_name))
        if world_id in seen:
            raise ValueError('мир %s: повторяющийся id в каталоге' % (world_id,))
        seen.add(world_id)

        status = str(raw['status']).strip()
        if status not in STATUSES:
            raise ValueError('мир %s: недопустимый status %r, ожидается ok|limited|broken'
                             % (world_id, status))

        entries.append(WorldEntry(
            id=world_id,
            title=str(raw['title']),
            file=str(raw['file']),
            world_name=str(raw['world_name']),
            status=status,
            status_note=str(raw.get('status_note') or ''),
            size_m=_pair(raw.get('size_m'), world_id, 'size_m'),
            origin_m=_pair(raw.get('origin_m'), world_id, 'origin_m'),
            default_spawn=_spawn(raw.get('default_spawn'), world_id),
            rooms=_rooms(raw.get('rooms'), world_id),
            preview=_opt_str(raw.get('preview')),
            preview_png=_opt_str(raw.get('preview_png')),
            scenarios_supported=bool(raw.get('scenarios_supported', False)),
            notes=str(raw.get('notes') or ''),
        ))

    default_id = str(data.get('default') or '').strip()
    if not default_id:
        usable = [e for e in entries if e.usable]
        default_id = (usable or entries)[0].id if entries else ''
    return Catalog(entries, default_id)


def load_catalog_file(path):
    """Прочитать worlds.yaml с диска. FileNotFoundError пробрасывается наружу:
    отсутствие каталога — это ошибка развёртывания (файл ставится через
    install(DIRECTORY config ...)), а не штатное состояние, которое можно замять."""
    import yaml  # ленивый импорт: parse_catalog работает и без PyYAML
    full = os.path.expanduser(path)
    with open(full, 'r', encoding='utf-8') as handle:
        return parse_catalog(yaml.safe_load(handle))


def resolve_paths(entry, share_dir):
    """Абсолютные пути записи. Все пути в каталоге относительны share/ar_project —
    так их можно писать в YAML одинаково и для исходников, и для install-каталога."""
    share_dir = os.path.expanduser(share_dir or '')

    def _abs(rel):
        if not rel:
            return None
        return rel if os.path.isabs(rel) else os.path.join(share_dir, rel)

    return {
        'sdf': _abs(entry.file),
        'preview': _abs(entry.preview),
        'preview_png': _abs(entry.preview_png),
    }


def read_world_name(sdf_path):
    """Значение <world name=...> из файла или None."""
    try:
        with open(sdf_path, 'r', encoding='utf-8', errors='replace') as handle:
            head = handle.read(_HEAD_BYTES)
    except OSError:
        return None
    match = _WORLD_NAME_RE.search(head)
    return match.group(1) if match else None


def validate(catalog, share_dir):
    """Список русских проблем каталога; пустой список = каталог согласован.

    Вызывается консолью один раз при старте, результат уходит в преflight как
    предупреждения. Ничего не чинит и не бросает: неполный каталог не должен
    мешать поднять консоль — он должен быть ВИДЕН.
    """
    problems = []
    for entry in catalog.list():
        paths = resolve_paths(entry, share_dir)
        sdf = paths['sdf']
        if not sdf or not os.path.exists(sdf):
            problems.append('мир %s: файл не найден: %s' % (entry.id, sdf))
            continue
        actual = read_world_name(sdf)
        if actual is not None and actual != entry.world_name:
            problems.append(
                'мир %s: <world name> в файле = %r, а в каталоге указано %r — '
                'gz-сервисы /world/%s/* работать не будут (свет и расстановка '
                'объектов сценария уйдут в никуда)'
                % (entry.id, actual, entry.world_name, entry.world_name))
        if entry.status == STATUS_BROKEN:
            continue
        preview = paths['preview'] or paths['preview_png']
        if preview and not os.path.exists(preview):
            problems.append(
                'мир %s: превью не сгенерировано (%s) — запустите '
                'ros2 run ar_project make_world_previews.py' % (entry.id, preview))
    return problems


def default_spawn_args(entry):
    """Стартовая поза строками — ровно в том виде, в каком её ждёт launch."""
    spawn = entry.default_spawn or {}
    return {
        'spawn_x': '%.3f' % float(spawn.get('x', 0.0)),
        'spawn_y': '%.3f' % float(spawn.get('y', 0.0)),
        'spawn_yaw': '%.3f' % float(spawn.get('yaw', 0.0)),
    }


def rooms_aabb_xxyy(entry):
    """Комнаты в порядке `(x_min, x_max, y_min, y_max)` — ровно как в
    `house_scenario_runner.py:89`. Именно эта форма сверяется юнит-тестом с
    константой ROOMS раннера: сам раннер править нельзя (по нему уже сняты
    прогоны), поэтому сходимость доказывается тестом, а не правкой кода."""
    out = {}
    for name, box in (entry.rooms or {}).items():
        xs = box['x']
        ys = box['y']
        out[name] = (float(xs[0]), float(xs[1]), float(ys[0]), float(ys[1]))
    return out


# -- разбор необязательных полей ---------------------------------------------

def _opt_str(value):
    if value is None:
        return None
    text = str(value).strip()
    return text or None


def _pair(value, world_id, field_name):
    if value is None:
        return (0.0, 0.0)
    try:
        a, b = value
        return (float(a), float(b))
    except (TypeError, ValueError):
        raise ValueError('мир %s: поле %s должно быть парой чисел [x, y]'
                         % (world_id, field_name))


def _spawn(value, world_id):
    if value is None:
        return {'x': 0.0, 'y': 0.0, 'yaw': 0.0}
    if not isinstance(value, dict):
        raise ValueError('мир %s: default_spawn должен быть словарём {x, y, yaw}'
                         % (world_id,))
    try:
        return {
            'x': float(value.get('x', 0.0)),
            'y': float(value.get('y', 0.0)),
            'yaw': float(value.get('yaw', 0.0)),
        }
    except (TypeError, ValueError):
        raise ValueError('мир %s: значения default_spawn должны быть числами'
                         % (world_id,))


def _rooms(value, world_id):
    """Комнаты только в самодокументируемой форме {x: [x0, x1], y: [y0, y1]}.

    Плоский список из четырёх чисел отвергается ЯВНО, а не подгоняется: в
    источнике истины (`house_scenario_runner.py:89`) порядок
    `(x_min, x_max, y_min, y_max)`, и любая догадка о порядке плоского списка
    даёт молча перепутанные оси — комната «кухня» окажется в коридоре, а тест
    сценария просто не засчитает подцель.
    """
    if not value:
        return {}
    if not isinstance(value, dict):
        raise ValueError('мир %s: rooms должен быть словарём {имя: {x: [..], y: [..]}}'
                         % (world_id,))
    out = {}
    for name, box in value.items():
        if not isinstance(box, dict) or 'x' not in box or 'y' not in box:
            raise ValueError(
                'мир %s: комната %s задана не в формате {x: [x0, x1], y: [y0, y1]} — '
                'плоский список из четырёх чисел запрещён, потому что в '
                'house_scenario_runner.py:89 порядок (x_min, x_max, y_min, y_max) '
                'и оси неминуемо перепутаются' % (world_id, name))
        try:
            xs = (float(box['x'][0]), float(box['x'][1]))
            ys = (float(box['y'][0]), float(box['y'][1]))
        except (TypeError, ValueError, IndexError, KeyError):
            raise ValueError('мир %s: комната %s: x и y должны быть парами чисел'
                             % (world_id, name))
        out[str(name)] = {'x': xs, 'y': ys}
    return out
