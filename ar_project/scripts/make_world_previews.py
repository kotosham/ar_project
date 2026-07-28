#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Офлайн-генератор top-down превью миров Gazebo (SVG).

ПОЧЕМУ SVG, А НЕ PNG
PNG потребовал бы либо Pillow (внешний пакет, которого нет ни в rosdep-списке
ar_project, ни в образе `sim` из deploy/docker/Dockerfile), либо запуска самого
Gazebo с GPU — а превью нужны ровно тогда, когда симулятор ещё НЕ поднят
(оператор выбирает комнату до старта стека). SVG собирается строками из stdlib,
отображается браузером нативно тегом <img>, и весит единицы килобайт.

ПОЧЕМУ ВЫХОД ПО УМОЛЧАНИЮ — ИСХОДНЫЙ КАТАЛОГ, А НЕ share
`install(DIRECTORY config description launch models worlds ...)`
(CMakeLists.txt:64-67) копирует worlds/ целиком при КАЖДОЙ сборке colcon,
затирая всё, что кто-то положил в share/ar_project/worlds/previews.
Поэтому по умолчанию пишем в src-дерево — оттуда превью попадут в share сами.

ПОЧЕМУ ОБЯЗАТЕЛЬНО РЕЗОЛВИТСЯ <include><uri>model://X</uri></include>
В flat_detect.world вся полезная нагрузка — один include detect_billboard
(flat_detect.world:132-134); без резолвинга через models/X/model.sdf превью
этого мира было бы пустой коробкой, хотя мир существует ровно ради билборда.

Зависимости: только стандартная библиотека. PyYAML импортируется ЛЕНИВО и
только для чтения каталога — без него скрипт всё равно рисует все миры из
каталога worlds/, просто без русских заголовков и комнат.
"""

import argparse
import math
import os
import re
import sys
import xml.etree.ElementTree as ET


# Цвета в тон mission_dashboard.py (тёмная тема консоли) — превью показывается
# в той же странице, и светлый прямоугольник среди тёмных карточек слепит.
BG = '#101418'
GRID_MINOR = '#1e262e'
GRID_MAJOR = '#2a333c'
TEXT = '#dbe4ec'
TEXT_DIM = '#8a949e'
DEFAULT_BOX_COLOR = '#8a8f96'
DEFAULT_MARKER_COLOR = '#c8a15a'
SPAWN_COLOR = '#33c06e'
ROOM_STROKE = '#4a90d9'

# Модели-декорации, подпись которых только засоряет план: стены рисуются
# сплошным контуром и без того читаются как стены.
WALLISH = re.compile(
    r'wall|perim|chamber|divider|_upper|_lower|ground|floor', re.IGNORECASE)

# Ниже этой высоты объект лежит на полу (коврики floor_carpet, лужи
# floor_puddle) и препятствием для планировщика не является — на плане этажа
# он бы только маскировал настоящие препятствия.
MIN_OBSTACLE_TOP_M = 0.05


# ---------------------------------------------------------------- разбор SDF

def parse_pose(text):
    """'<pose>x y z roll pitch yaw</pose>' -> кортеж из шести float.

    Допускает 3 числа (Gazebo Classic так пишет), запись '-0' и экспоненту
    ('-9e-05' встречается в test_*.world после сохранения из GUI). Всё, что не
    разбирается, считается нулём: битая поза не должна ронять генерацию плана.
    """
    if not text:
        return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    vals = []
    for token in text.replace(',', ' ').split():
        try:
            vals.append(float(token))
        except ValueError:
            vals.append(0.0)
    while len(vals) < 6:
        vals.append(0.0)
    return tuple(vals[:6])


def _numbers(text, count, default=0.0):
    vals = []
    for token in (text or '').replace(',', ' ').split():
        try:
            vals.append(float(token))
        except ValueError:
            vals.append(default)
    while len(vals) < count:
        vals.append(default)
    return vals[:count]


def compose(base, local):
    """Складывает две позы в плоском приближении (поворот только вокруг Z).

    roll/pitch намеренно отбрасываются: на плане этажа наклонённый знак всё
    равно проецируется в тот же прямоугольник, а полная матрица потребовала бы
    кватернионов ради результата, неотличимого в масштабе 40 px/м.
    """
    bx, by, bz, _, _, byaw = base
    lx, ly, lz, _, _, lyaw = local
    cos_y = math.cos(byaw)
    sin_y = math.sin(byaw)
    return (bx + cos_y * lx - sin_y * ly,
            by + sin_y * lx + cos_y * ly,
            bz + lz, 0.0, 0.0, byaw + lyaw)


def _color_of(visual):
    """Цвет из <material><diffuse>r g b a</diffuse>.

    Только diffuse: в test_*.world материал задан через <script> в синтаксисе
    Gazebo Classic (test_1.world:65), который gz-sim не резолвит — цвет оттуда
    всё равно был бы враньём, лучше честный серый по умолчанию.
    """
    if visual is None:
        return None
    material = visual.find('material')
    if material is None:
        return None
    rgb = material.findtext('diffuse')
    if not rgb:
        return None
    vals = _numbers(rgb, 3)
    if not vals:
        return None
    return '#%02x%02x%02x' % tuple(
        max(0, min(255, int(round(v * 255.0)))) for v in vals)


def _geometry_shape(geom, pose, color, model_name):
    """Одна <geometry> -> словарь фигуры или None, если рисовать нечего."""
    if geom is None:
        return None
    x, y, z, _, _, yaw = pose

    box = geom.find('box')
    if box is not None:
        sx, sy, sz = _numbers(box.findtext('size'), 3)
        return {'kind': 'box', 'x': x, 'y': y, 'yaw': yaw,
                'sx': abs(sx), 'sy': abs(sy), 'z_top': z + abs(sz) / 2.0,
                'color': color or DEFAULT_BOX_COLOR, 'model': model_name}

    cyl = geom.find('cylinder')
    if cyl is not None:
        radius = _numbers(cyl.findtext('radius'), 1)[0]
        length = _numbers(cyl.findtext('length'), 1)[0]
        return {'kind': 'circle', 'x': x, 'y': y, 'yaw': yaw,
                'r': abs(radius), 'z_top': z + abs(length) / 2.0,
                'color': color or DEFAULT_BOX_COLOR, 'model': model_name}

    sph = geom.find('sphere')
    if sph is not None:
        radius = _numbers(sph.findtext('radius'), 1)[0]
        return {'kind': 'circle', 'x': x, 'y': y, 'yaw': yaw,
                'r': abs(radius), 'z_top': z + abs(radius),
                'color': color or DEFAULT_BOX_COLOR, 'model': model_name}

    # Пол. Плоскость 100x100 растянула бы bbox всего мира на сотню метров и
    # схлопнула бы саму квартиру в несколько пикселей — пропускаем всегда.
    if geom.find('plane') is not None:
        return None

    mesh = geom.find('mesh')
    if mesh is not None:
        uri = (mesh.findtext('uri') or '').strip()
        label = os.path.splitext(os.path.basename(uri))[0] or model_name
        return {'kind': 'marker', 'x': x, 'y': y, 'yaw': yaw,
                'z_top': z + 0.5, 'label': label,
                'color': color or DEFAULT_MARKER_COLOR, 'model': model_name}

    return None


def _shapes_of_link(link, base_pose, model_name):
    link_pose = compose(base_pose, parse_pose(link.findtext('pose')))
    visuals = link.findall('visual')
    collisions = link.findall('collision')
    # collision — это то, во что робот реально врежется и что видит /scan;
    # visual берётся только если collision у звена нет вовсе (билборд
    # detect_billboard имеет оба, а декорации иногда только visual).
    sources = collisions if collisions else visuals

    out = []
    for idx, elem in enumerate(sources):
        visual = visuals[idx] if idx < len(visuals) else (
            visuals[0] if visuals else None)
        pose = compose(link_pose, parse_pose(elem.findtext('pose')))
        shape = _geometry_shape(elem.find('geometry'), pose,
                                _color_of(visual), model_name)
        if shape is not None:
            out.append(shape)
    return out


def _shapes_of_model(model, parent_pose, name_override=None):
    name = name_override or model.get('name') or ''
    lowered = name.lower()
    if lowered == 'ground_plane' or lowered.startswith('sun') or 'light' in lowered:
        return []

    base = compose(parent_pose, parse_pose(model.findtext('pose')))
    out = []
    for link in model.findall('link'):
        out.extend(_shapes_of_link(link, base, name))
    # Вложенные <model> в SDF 1.7 легальны; в репозитории их нет, но рекурсия
    # стоит три строки и не даст будущему миру молча потерять половину мебели.
    for nested in model.findall('model'):
        out.extend(_shapes_of_model(nested, base))
    return out


def resolve_include(uri, models_dir):
    """model://X[/...] -> корневой <model> из models/X/model.sdf либо None."""
    if not uri.startswith('model://'):
        return None
    pkg = uri[len('model://'):].split('/')[0]
    if not pkg or not models_dir:
        return None
    path = os.path.join(models_dir, pkg, 'model.sdf')
    if not os.path.isfile(path):
        return None
    try:
        root = ET.parse(path).getroot()
    except ET.ParseError:
        return None
    return root.find('model') if root.tag == 'sdf' else root


def iter_shapes(world_elem, models_dir=None):
    """Все рисуемые фигуры мира в мировых координатах.

    Обходятся ТОЛЬКО прямые дети <world>: в test_*.world есть ещё блок <state>
    с копиями всех моделей (test_1.world:104-194) — обход через iter() удвоил
    бы каждый объект и сдвинул бы половину из них устаревшими позами.
    """
    shapes = []
    for model in world_elem.findall('model'):
        shapes.extend(_shapes_of_model(model, (0.0,) * 6))

    for inc in world_elem.findall('include'):
        uri = (inc.findtext('uri') or '').strip()
        pose = parse_pose(inc.findtext('pose'))
        name = (inc.findtext('name') or uri.rstrip('/').split('/')[-1] or 'include')
        resolved = resolve_include(uri, models_dir)
        if resolved is not None:
            shapes.extend(_shapes_of_model(resolved, pose, name_override=name))
        else:
            # Модель не нашлась (Fuel-URI, отсутствующий каталог) — рисуем
            # маркер с подписью, чтобы на плане было видно, что тут что-то есть.
            shapes.append({'kind': 'marker', 'x': pose[0], 'y': pose[1],
                           'yaw': pose[5], 'z_top': pose[2] + 0.5,
                           'label': name, 'color': DEFAULT_MARKER_COLOR,
                           'model': name})

    return [s for s in shapes if s.get('z_top', 1.0) >= MIN_OBSTACLE_TOP_M]


def bbox_of(shapes, margin=0.5):
    """(x0, y0, x1, y1) с запасом; для пустого мира — квадрат 10x10 м."""
    if not shapes:
        return (-5.0, -5.0, 5.0, 5.0)
    xs_min, ys_min, xs_max, ys_max = [], [], [], []
    for s in shapes:
        if s['kind'] == 'box':
            # Габарит повёрнутого прямоугольника: проекция полудиагоналей.
            cos_y = abs(math.cos(s['yaw']))
            sin_y = abs(math.sin(s['yaw']))
            hx = (s['sx'] * cos_y + s['sy'] * sin_y) / 2.0
            hy = (s['sx'] * sin_y + s['sy'] * cos_y) / 2.0
        elif s['kind'] == 'circle':
            hx = hy = s['r']
        else:
            hx = hy = 0.25
        xs_min.append(s['x'] - hx)
        xs_max.append(s['x'] + hx)
        ys_min.append(s['y'] - hy)
        ys_max.append(s['y'] + hy)
    return (min(xs_min) - margin, min(ys_min) - margin,
            max(xs_max) + margin, max(ys_max) + margin)


def check_world_name(sdf_path, expected):
    """Русское предупреждение о расхождении <world name> с каталогом, либо None.

    Это самая дорогая ошибка каталога: раннер и launch бьют в gz-сервисы
    /world/<world_name>/create|set_pose|light_config
    (house_scenario_runner.py адресует /world/house/*), и при неверном имени
    всё «работает», но ни одна команда не доходит.
    """
    try:
        with open(sdf_path, 'r', encoding='utf-8', errors='replace') as fh:
            head = fh.read(65536)
    except OSError as exc:
        return 'не удалось прочитать %s: %s' % (sdf_path, exc)
    match = re.search(r"<world\s+name=['\"]([^'\"]+)['\"]", head)
    if not match:
        return 'в файле %s не найден тег <world name=...>' % os.path.basename(sdf_path)
    actual = match.group(1)
    if expected and actual != expected:
        return ('<world name> в файле = %r, а в каталоге указано %r — '
                'сервисы /world/%s/* работать не будут'
                % (actual, expected, expected))
    return None


# ------------------------------------------------------------------- отрисовка

def esc(text):
    return (str(text).replace('&', '&amp;').replace('<', '&lt;')
            .replace('>', '&gt;').replace('"', '&quot;'))


def _fmt(value):
    return ('%.2f' % value).rstrip('0').rstrip('.') or '0'


def render_svg(shapes, bbox, title='', rooms=None, room_titles=None,
               spawn=None, px_per_m=40.0, max_px=900, subtitle=''):
    """Строит SVG-план этажа строкой.

    Ось Y экрана инвертирована относительно мировой (sy = (y1 - y) * k), чтобы
    +Y мира смотрел вверх — ровно как на ASCII-схеме в шапке house.sdf:6-21,
    иначе спальня и кладовка поменялись бы местами относительно документации.
    """
    rooms = rooms or {}
    room_titles = room_titles or {}
    x0, y0, x1, y1 = bbox
    w_m = max(x1 - x0, 0.5)
    h_m = max(y1 - y0, 0.5)
    k = min(px_per_m, max_px / w_m, max_px / h_m)

    pad_top = 34.0      # место под заголовок
    pad_bottom = 30.0   # место под масштабную линейку
    plot_w = w_m * k
    plot_h = h_m * k
    width = plot_w
    height = plot_h + pad_top + pad_bottom

    def sx(x):
        return (x - x0) * k

    def sy(y):
        return (y1 - y) * k + pad_top

    out = []
    out.append('<?xml version="1.0" encoding="UTF-8"?>')
    out.append('<svg xmlns="http://www.w3.org/2000/svg" role="img" '
               'viewBox="0 0 %.1f %.1f" width="%.0f" height="%.0f" '
               'font-family="DejaVu Sans, Segoe UI, sans-serif">'
               % (width, height, width, height))
    out.append('<title>%s</title>' % esc(title or 'план мира'))
    out.append('<rect x="0" y="0" width="%.1f" height="%.1f" fill="%s"/>'
               % (width, height, BG))

    # --- сетка 1 м; каждые 5 м линия ярче, чтобы считать расстояния глазом
    grid = []
    gx = math.floor(x0)
    while gx <= x1:
        color = GRID_MAJOR if abs(gx % 5.0) < 1e-6 else GRID_MINOR
        grid.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" '
                    'stroke="%s" stroke-width="1"/>'
                    % (sx(gx), pad_top, sx(gx), pad_top + plot_h, color))
        gx += 1.0
    gy = math.floor(y0)
    while gy <= y1:
        color = GRID_MAJOR if abs(gy % 5.0) < 1e-6 else GRID_MINOR
        grid.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" '
                    'stroke="%s" stroke-width="1"/>'
                    % (0.0, sy(gy), plot_w, sy(gy), color))
        gy += 1.0
    out.append('<g>%s</g>' % ''.join(grid))

    # --- комнаты (только у house): прямоугольник AABB + русская подпись
    for name in sorted(rooms):
        box = rooms[name]
        try:
            rx0, rx1 = float(box['x'][0]), float(box['x'][1])
            ry0, ry1 = float(box['y'][0]), float(box['y'][1])
        except (KeyError, TypeError, IndexError, ValueError):
            continue
        out.append('<rect x="%.1f" y="%.1f" width="%.1f" height="%.1f" '
                   'fill="%s" fill-opacity="0.06" stroke="%s" '
                   'stroke-opacity="0.35" stroke-dasharray="4 4"/>'
                   % (sx(rx0), sy(ry1), (rx1 - rx0) * k, (ry1 - ry0) * k,
                      ROOM_STROKE, ROOM_STROKE))
        label = room_titles.get(name, name)
        out.append('<text x="%.1f" y="%.1f" fill="%s" font-size="12" '
                   'text-anchor="middle" opacity="0.85">%s</text>'
                   % (sx((rx0 + rx1) / 2.0), sy((ry0 + ry1) / 2.0),
                      ROOM_STROKE, esc(label)))

    # --- геометрия
    labels = []
    seen_models = set()
    for s in shapes:
        cx, cy = sx(s['x']), sy(s['y'])
        if s['kind'] == 'box':
            w_px = s['sx'] * k
            h_px = s['sy'] * k
            # rotate на МИНУС yaw: экранная Y смотрит вниз, поэтому
            # положительный поворот мира выглядит на SVG как отрицательный.
            out.append('<rect x="%.2f" y="%.2f" width="%.2f" height="%.2f" '
                       'fill="%s" stroke="#0c0f12" stroke-width="1" '
                       'transform="rotate(%.3f %.2f %.2f)"/>'
                       % (cx - w_px / 2.0, cy - h_px / 2.0, w_px, h_px,
                          s['color'], -math.degrees(s['yaw']), cx, cy))
        elif s['kind'] == 'circle':
            out.append('<circle cx="%.2f" cy="%.2f" r="%.2f" fill="%s" '
                       'stroke="#0c0f12" stroke-width="1"/>'
                       % (cx, cy, max(s['r'] * k, 1.5), s['color']))
        else:
            out.append('<circle cx="%.2f" cy="%.2f" r="6" fill="%s" '
                       'stroke="#0c0f12" stroke-width="1"/>'
                       % (cx, cy, s['color']))
            labels.append('<text x="%.1f" y="%.1f" fill="%s" font-size="9" '
                          'text-anchor="middle">%s</text>'
                          % (cx, cy - 9.0, TEXT_DIM, esc(s.get('label', ''))))
            seen_models.add(s.get('model'))

        model = s.get('model') or ''
        if (model and model not in seen_models and not WALLISH.search(model)):
            seen_models.add(model)
            labels.append('<text x="%.1f" y="%.1f" fill="%s" font-size="9" '
                          'text-anchor="middle">%s</text>'
                          % (cx, cy - 6.0, TEXT_DIM, esc(model)))
    out.extend(labels)

    # --- точка старта робота
    if spawn:
        px, py = sx(float(spawn.get('x', 0.0))), sy(float(spawn.get('y', 0.0)))
        yaw = float(spawn.get('yaw', 0.0))
        pts = []
        for ang, rad in ((0.0, 11.0), (2.5, 7.0), (-2.5, 7.0)):
            a = yaw + ang
            pts.append('%.1f,%.1f' % (px + rad * math.cos(a),
                                      py - rad * math.sin(a)))
        out.append('<polygon points="%s" fill="%s" stroke="#0c0f12" '
                   'stroke-width="1"/>' % (' '.join(pts), SPAWN_COLOR))
        out.append('<text x="%.1f" y="%.1f" fill="%s" font-size="10" '
                   'text-anchor="middle">старт</text>'
                   % (px, py + 20.0, SPAWN_COLOR))

    # --- заголовок и оси
    out.append('<text x="10" y="20" fill="%s" font-size="13">%s</text>'
               % (TEXT, esc(title)))
    if subtitle:
        out.append('<text x="%.1f" y="20" fill="%s" font-size="10" '
                   'text-anchor="end">%s</text>'
                   % (width - 10.0, TEXT_DIM, esc(subtitle)))

    ax, ay = width - 46.0, pad_top + 34.0
    out.append('<g stroke="%s" stroke-width="1.5" fill="none">'
               '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
               '<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f"/>'
               '</g>' % (TEXT_DIM, ax, ay, ax, ay - 24.0, ax, ay, ax + 24.0, ay))
    out.append('<text x="%.1f" y="%.1f" fill="%s" font-size="9" '
               'text-anchor="middle">+Y</text>' % (ax, ay - 28.0, TEXT_DIM))
    out.append('<text x="%.1f" y="%.1f" fill="%s" font-size="9">+X</text>'
               % (ax + 27.0, ay + 3.0, TEXT_DIM))

    # --- легенда: отрезок длиной РОВНО 1 м мира
    bar_y = pad_top + plot_h + 16.0
    out.append('<line x1="10" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
               'stroke-width="2"/>' % (bar_y, 10.0 + k, bar_y, TEXT))
    out.append('<line x1="10" y1="%.1f" x2="10" y2="%.1f" stroke="%s" '
               'stroke-width="2"/>' % (bar_y - 4.0, bar_y + 4.0, TEXT))
    out.append('<line x1="%.1f" y1="%.1f" x2="%.1f" y2="%.1f" stroke="%s" '
               'stroke-width="2"/>' % (10.0 + k, bar_y - 4.0, 10.0 + k,
                                       bar_y + 4.0, TEXT))
    out.append('<text x="%.1f" y="%.1f" fill="%s" font-size="10">1 м</text>'
               % (14.0 + k, bar_y + 4.0, TEXT))
    out.append('<text x="%.1f" y="%.1f" fill="%s" font-size="10" '
               'text-anchor="end">%s x %s м, вид сверху</text>'
               % (width - 10.0, bar_y + 4.0, TEXT_DIM, _fmt(w_m), _fmt(h_m)))

    out.append('</svg>')
    return '\n'.join(out) + '\n'


def preview_for_world(sdf_path, entry=None, models_dir=None):
    """Полный цикл: SDF -> строка SVG. Возвращает (svg, кол_моделей, кол_фигур)."""
    entry = entry or {}
    root = ET.parse(sdf_path).getroot()
    world = root.find('world') if root.tag == 'sdf' else root
    if world is None:
        raise ValueError('в %s нет элемента <world>' % os.path.basename(sdf_path))

    shapes = iter_shapes(world, models_dir)
    models = len(world.findall('model')) + len(world.findall('include'))
    title = entry.get('title') or entry.get('id') or os.path.basename(sdf_path)
    subtitle = 'world name: %s' % (entry.get('world_name') or '?')
    svg = render_svg(shapes, bbox_of(shapes), title=title,
                     rooms=entry.get('rooms') or {},
                     room_titles=entry.get('room_titles') or {},
                     spawn=entry.get('default_spawn'),
                     subtitle=subtitle)
    return svg, models, len(shapes)


# ------------------------------------------------------------------------ CLI

def default_pkg_dir():
    """Каталог с worlds/ и config/: сначала исходное дерево, потом share.

    В исходниках скрипт лежит в <pkg>/scripts/, значит ../worlds существует —
    и это ровно тот каталог, куда надо писать (см. шапку модуля). После
    установки скрипт оказывается в lib/ar_project/, ../worlds нет, и тогда
    спрашиваем ament_index.
    """
    src = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if os.path.isdir(os.path.join(src, 'worlds')):
        return src
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('ar_project')
    except Exception:
        return src


def load_catalog(path):
    """worlds.yaml -> {id: запись}. Отсутствие файла или PyYAML не фатально."""
    if not path or not os.path.isfile(path):
        return {}, 'каталог не найден: %s' % path
    try:
        import yaml  # ленивый импорт: без каталога скрипт всё равно работает
    except ImportError:
        return {}, 'PyYAML не установлен — заголовки и комнаты не будут нарисованы'
    try:
        with open(path, 'r', encoding='utf-8') as fh:
            data = yaml.safe_load(fh) or {}
    except Exception as exc:
        return {}, 'каталог не разобран (%s): %s' % (path, exc)
    entries = {}
    for item in data.get('worlds') or []:
        if isinstance(item, dict) and item.get('id'):
            entries[item['id']] = item
    return entries, None


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Генерация top-down SVG-превью миров Gazebo без запуска симулятора.')
    pkg = default_pkg_dir()
    parser.add_argument('--worlds-dir', default=os.path.join(pkg, 'worlds'),
                        help='каталог с *.sdf / *.world (по умолчанию %(default)s)')
    parser.add_argument('--models-dir', default=os.path.join(pkg, 'models'),
                        help='каталог моделей для резолвинга model://X')
    parser.add_argument('--out-dir', default=None,
                        help='куда писать SVG (по умолчанию <worlds-dir>/previews)')
    parser.add_argument('--catalog', default=os.path.join(pkg, 'config', 'worlds.yaml'),
                        help='config/worlds.yaml для заголовков, комнат и статусов')
    parser.add_argument('--only', nargs='+', metavar='ID',
                        help='сгенерировать только указанные миры')
    parser.add_argument('--list', action='store_true',
                        help='только показать, что найдено, ничего не писать')
    parser.add_argument('--force', action='store_true',
                        help='перезаписывать уже существующие превью')
    args = parser.parse_args(argv)

    out_dir = args.out_dir or os.path.join(args.worlds_dir, 'previews')
    catalog, catalog_problem = load_catalog(args.catalog)
    if catalog_problem:
        print('ПРЕДУПРЕЖДЕНИЕ: %s' % catalog_problem, file=sys.stderr)

    if not os.path.isdir(args.worlds_dir):
        print('ОШИБКА: каталог миров не найден: %s' % args.worlds_dir,
              file=sys.stderr)
        return 2

    # Порядок: сначала как в каталоге (он отсортирован осмысленно), потом
    # всё остальное, что лежит в worlds/ и в каталог ещё не попало.
    files = sorted(f for f in os.listdir(args.worlds_dir)
                   if f.endswith(('.sdf', '.world')))
    by_id = {os.path.splitext(f)[0]: f for f in files}
    order = [wid for wid in catalog if wid in by_id]
    order += [wid for wid in sorted(by_id) if wid not in order]
    if args.only:
        unknown = [w for w in args.only if w not in by_id]
        if unknown:
            print('ОШИБКА: нет таких миров: %s' % ', '.join(unknown),
                  file=sys.stderr)
            return 2
        order = [w for w in order if w in args.only]

    failures = 0
    if not args.list:
        os.makedirs(out_dir, exist_ok=True)

    for wid in order:
        entry = dict(catalog.get(wid) or {})
        entry.setdefault('id', wid)
        sdf_path = os.path.join(args.worlds_dir, by_id[wid])

        warn = check_world_name(sdf_path, entry.get('world_name'))
        if warn:
            print('ПРЕДУПРЕЖДЕНИЕ [%s]: %s' % (wid, warn), file=sys.stderr)

        if entry.get('status') == 'broken' and not args.only:
            print('%-26s пропущен (broken): %s'
                  % (wid, entry.get('status_note') or 'мир помечен как нерабочий'))
            continue

        rel = os.path.join('worlds', 'previews', wid + '.svg').replace('\\', '/')
        dst = os.path.join(out_dir, wid + '.svg')

        try:
            svg, n_models, n_shapes = preview_for_world(
                sdf_path, entry, args.models_dir)
        except Exception as exc:
            failures += 1
            print('ОШИБКА [%s]: %s' % (wid, exc), file=sys.stderr)
            continue

        if args.list:
            print('%-26s %-34s %d моделей, %d фигур'
                  % (wid, by_id[wid], n_models, n_shapes))
            continue

        if os.path.exists(dst) and not args.force:
            print('%-26s пропущен (уже есть, нужен --force): %s' % (wid, rel))
            continue

        with open(dst, 'w', encoding='utf-8', newline='\n') as fh:
            fh.write(svg)
        print('%-26s -> %s (%d моделей, %d фигур)'
              % (wid, rel, n_models, n_shapes))

    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
