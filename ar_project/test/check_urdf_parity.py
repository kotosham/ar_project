#!/usr/bin/env python3
"""Проверка детерминизма URDF: одна геометрия, два нижних слоя, один канон камеры.

ЗАЧЕМ ЭТОТ СКРИПТ
=================
Заявление «URDF геометрически один, различается только слой драйверов» до сих
пор нигде не проверялось. Проверять его надо ровно в трёх местах, и первая
версия этой проверки была БЕСПОЛЕЗНОЙ: она нормализовала URDF, выбрасывая все
блоки <gazebo>, но канонические аргументы cam_width/cam_height/cam_rate/cam_far/
depth_far используются ТОЛЬКО внутри <gazebo> (description/camera_gazebo_sensors.xacro:16,
22, 23, 36, 56, 63, 64, 78). То есть нормализация стирала единственное место, на
которое канон вообще влияет, и сравнение было бы зелёным при любом значении.

Поэтому здесь три независимые проверки:

  (а) ПАРИТЕТ ГЕОМЕТРИИ. Рендерим robot.urdf.xacro (sim) и robot_hardware.urdf.xacro
      (железо) и сравниваем поддеревья <link>/<joint> БЕЗ <gazebo> и <ros2_control>.
      Эти два блока обязаны различаться — это и есть «два разных нижних слоя»;
      всё остальное обязано совпасть до шестого знака.

  (б) КАНОН КАМЕРЫ. Отдельно достаём из sim-URDF <gazebo>/<sensor>/<camera> —
      width, height, update_rate, clip/far — и сверяем с fleet_comms.mode_profiles.
      urdf_canon(). Сверка делается ДВАЖДЫ:
        б1 — рендер с явными аргументами канона: ловит момент, когда кто-то
             вписал число литералом мимо $(arg) или подключил аргумент только к
             RGB-сенсору, забыв про depth (их разрешения обязаны совпадать,
             camera_gazebo_sensors.xacro:49-54);
        б2 — рендер вообще без аргументов: ловит расхождение дефолтов
             robot.urdf.xacro:16-20 с каноном mode_profiles. Это разные файлы в
             разных репозиториях-пакетах, и разъехаться они могут молча.

  (в) WHEEL_SEPARATION. Печатается расхождение трёх источников как ЯВНОЕ
      предупреждение, а не как ошибка: железо уже откалибровано
      (wheel_separation_multiplier=1.0052), и правка чисел означает перекалибровку
      стенда. Смысл вывода — чтобы в журнале испытаний было записано, что метрики
      пути между режимами напрямую несравнимы.

Плюс проверяется, что в железный URDF не просочился симуляционный плагин
gz_ros2_control/GazeboSimSystem: controller_manager на Pi такой плагин не найдёт
и не поднимется вообще.

Это ИНСТРУМЕНТ РАЗРАБОТЧИКА, а не установленная нода: в install(PROGRAMS) он не
добавлен. Запуск из исходников:

    python3 ar_project/test/check_urdf_parity.py
    python3 ar_project/test/check_urdf_parity.py --hw-parity --verbose

КОДЫ ВОЗВРАТА
    0   паритет соблюдён
    1   расхождение (геометрия, канон камеры или gz_ros2_control на железе)
    2   ошибка рендера xacro
    77  пропуск: в системе нет xacro (стандартный код skip у automake/ctest)
"""

import argparse
import os
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

# Атрибуты URDF, которые содержат числа. Округляются до шести знаков, чтобы
# '0.17450000000000002' из питоновской арифметики xacro и '0.1745' считались
# одинаковыми, а '-0.0' не отличалось от '0.0'.
NUMERIC_ATTRS = (
    'xyz', 'rpy', 'value', 'mass', 'radius', 'length', 'size',
    'ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz',
    'lower', 'upper', 'effort', 'velocity', 'damping', 'friction',
)

# $(find ar_project) разворачивается в абсолютный путь, который различается на
# каждой машине (исходники против install/share). Такие значения обнуляются,
# иначе тексты никогда не совпадут в CI.
PKG_PATH_RE = re.compile(r'/[^\s"\']*(?:install|share)/ar_project')

# Блоки, которые ОБЯЗАНЫ различаться: это и есть «нижний слой».
LOWER_LAYER_TAGS = ('gazebo', 'ros2_control')

# Дублируется намеренно, чтобы скрипт работал в offline-CI без собранного
# рабочего пространства. Источник истины — fleet_comms.mode_profiles.
_FALLBACK_CANON_SIM = {
    'cam_width': '320', 'cam_height': '240', 'cam_rate': '15',
    'cam_far': '30.0', 'depth_far': '8.0',
}
_FALLBACK_CANON_HW = {
    'cam_width': '640', 'cam_height': '480', 'cam_rate': '15',
    'cam_far': '30.0', 'depth_far': '8.0',
}


def load_canon(hw_parity):
    """Канон камеры из fleet_comms, с падением на встроенную копию."""
    try:
        from fleet_comms.mode_profiles import urdf_canon
    except Exception:
        return dict(_FALLBACK_CANON_HW if hw_parity else _FALLBACK_CANON_SIM), False
    return urdf_canon(hw_parity=hw_parity), True


# --------------------------------------------------------------- пути

def default_share():
    """Каталог с description/ и config/.

    Сначала share собранного пакета (там же лежат сгенерированные master.dcf/
    master.bin, на которые ссылается ros2_control_hardware.xacro:9-10), затем —
    исходники рядом со скриптом, чтобы проверку можно было гонять до сборки.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('ar_project')
    except Exception:
        return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# --------------------------------------------------------------- рендер

def render(xacro_path, args):
    """Рендер одного корня xacro в текст URDF.

    Для mode=hardware нужен СОБРАННЫЙ пакет: ros2_control_hardware.xacro:9-10
    подставляет $(find ar_project)/config/epos4_diffdrive/master.dcf и master.bin,
    а их генерирует cogen_dcf на этапе сборки — в исходниках этих файлов нет.
    Сам xacro существование путей не проверяет, поэтому рендер пройдёт и без
    сборки; ошибка вылезет позже, уже в controller_manager.
    """
    if not os.path.isfile(xacro_path):
        raise RuntimeError('не найден файл xacro: %s' % xacro_path)
    argv = ['xacro', xacro_path] + ['%s:=%s' % (k, v) for k, v in sorted(args.items())]
    proc = subprocess.run(argv, capture_output=True, text=True, check=False)
    if proc.returncode != 0:
        raise RuntimeError('xacro завершился с кодом %d\n  команда: %s\n%s'
                           % (proc.returncode, ' '.join(argv), proc.stderr.strip()))
    return proc.stdout


# --------------------------------------------------------------- нормализация

def _round_numeric(value):
    out = []
    for token in value.split():
        try:
            number = round(float(token), 6)
        except ValueError:
            out.append(token)
            continue
        if number == 0.0:
            number = 0.0            # снимает знак у -0.0
        out.append(('%.6f' % number).rstrip('0').rstrip('.') or '0')
    return ' '.join(out)


def _clean(elem):
    for key in list(elem.attrib):
        value = elem.attrib[key]
        value = PKG_PATH_RE.sub('<PKG>', value)
        if key in NUMERIC_ATTRS:
            value = _round_numeric(value)
        elem.attrib[key] = value
    if elem.text is not None and not elem.text.strip():
        elem.text = None
    elif elem.text is not None:
        elem.text = _round_numeric(elem.text.strip())
    elem.tail = None
    for child in list(elem):
        _clean(child)


def _serialize(elem, depth=0):
    pad = '  ' * depth
    attrs = ''.join(' %s="%s"' % (k, elem.attrib[k]) for k in sorted(elem.attrib))
    children = list(elem)
    if not children and not elem.text:
        return ['%s<%s%s/>' % (pad, elem.tag, attrs)]
    lines = ['%s<%s%s>' % (pad, elem.tag, attrs)]
    if elem.text:
        lines.append('%s  %s' % (pad, elem.text))
    for child in children:
        lines.extend(_serialize(child, depth + 1))
    lines.append('%s</%s>' % (pad, elem.tag))
    return lines


def normalize(urdf_text):
    """Канонический текст ВЕРХНЕЙ части URDF (без слоя драйверов)."""
    root = ET.fromstring(urdf_text)
    for tag in LOWER_LAYER_TAGS:
        for node in root.findall(tag):
            root.remove(node)
    _clean(root)
    # Порядок включений в двух корнях разный (robot.urdf.xacro против
    # robot_hardware.urdf.xacro), поэтому детей сортируем: сравнивается
    # содержимое, а не порядок xacro:include.
    children = sorted(root, key=lambda e: (e.tag, e.get('name') or ''))
    for node in list(root):
        root.remove(node)
    for node in children:
        root.append(node)
    return '\n'.join(_serialize(root))


def diff_report(a, b, limit=80):
    import difflib
    lines = list(difflib.unified_diff(a.splitlines(), b.splitlines(),
                                      fromfile='sim', tofile='hardware', lineterm=''))
    if len(lines) > limit:
        lines = lines[:limit] + ['... (обрезано, всего строк diff: %d)' % len(lines)]
    return lines


# --------------------------------------------------------------- камера

def _text(elem, path):
    node = elem.find(path) if elem is not None else None
    return node.text.strip() if node is not None and node.text else None


def extract_gz_cameras(urdf_text):
    """{имя сенсора: {параметр: строка}} из блоков <gazebo>/<sensor>.

    Именно эти значения канон и обязан задавать; ничего другого он не трогает.
    """
    root = ET.fromstring(urdf_text)
    out = {}
    for gz in root.findall('gazebo'):
        for sensor in gz.findall('sensor'):
            name = sensor.get('name') or '?'
            cam = sensor.find('camera')
            out[name] = {
                'type': sensor.get('type') or '?',
                'update_rate': _text(sensor, 'update_rate'),
                'width': _text(cam, 'image/width'),
                'height': _text(cam, 'image/height'),
                'clip_far': _text(cam, 'clip/far'),
                'clip_near': _text(cam, 'clip/near'),
                'horizontal_fov': _text(cam, 'horizontal_fov'),
                'depth_clip_far': _text(cam, 'depth_camera/clip/far'),
            }
    return out


def _same_number(actual, expected):
    if actual is None:
        return False
    try:
        return abs(float(actual) - float(expected)) < 1e-6
    except ValueError:
        return str(actual) == str(expected)


def check_camera_canon(cameras, canon, label):
    """Список русских сообщений о расхождении с каноном (пустой = совпало)."""
    problems = []
    if 'camera' not in cameras:
        problems.append('%s: в sim-URDF нет сенсора <sensor name="camera">' % label)
    if 'depth_camera' not in cameras:
        problems.append('%s: в sim-URDF нет сенсора <sensor name="depth_camera">' % label)

    # Разрешение и частота обязаны совпадать у ОБОИХ сенсоров: эмуляция
    # aligned depth в gz_bridge держит пиксели RGB и depth совмещёнными только
    # при одинаковой сетке (camera_gazebo_sensors.xacro:49-54).
    for sensor_name in ('camera', 'depth_camera'):
        cam = cameras.get(sensor_name)
        if cam is None:
            continue
        for key, canon_key in (('width', 'cam_width'),
                               ('height', 'cam_height'),
                               ('update_rate', 'cam_rate')):
            if not _same_number(cam.get(key), canon[canon_key]):
                problems.append('%s: %s.%s = %r, канон %s = %r'
                                % (label, sensor_name, key, cam.get(key),
                                   canon_key, canon[canon_key]))

    cam = cameras.get('camera')
    if cam is not None and not _same_number(cam.get('clip_far'), canon['cam_far']):
        problems.append('%s: camera.clip/far = %r, канон cam_far = %r'
                        % (label, cam.get('clip_far'), canon['cam_far']))

    depth = cameras.get('depth_camera')
    if depth is not None:
        # Кусается именно <camera><clip><far>, а не <depth_camera><clip><far>:
        # измерено в gz-sim 8, что без первого depth-кадр возвращал до 14.13 м
        # вместо 8 (camera_gazebo_sensors.xacro:66-75). Проверяем оба.
        if not _same_number(depth.get('clip_far'), canon['depth_far']):
            problems.append('%s: depth_camera.clip/far = %r, канон depth_far = %r'
                            % (label, depth.get('clip_far'), canon['depth_far']))
        if not _same_number(depth.get('depth_clip_far'), canon['depth_far']):
            problems.append('%s: depth_camera.depth_camera/clip/far = %r, канон depth_far = %r'
                            % (label, depth.get('depth_clip_far'), canon['depth_far']))
    return problems


# --------------------------------------------------------------- wheel_separation

def _anchor(path, pattern, fallback_line):
    """'файл:строка' по первому совпадению; при неудаче — задокументированная строка."""
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            for number, line in enumerate(handle, 1):
                if re.search(pattern, line):
                    return '%s:%d' % (os.path.basename(path), number)
    except OSError:
        pass
    return '%s:%d' % (os.path.basename(path), fallback_line)


def wheel_separation_sources(sim_urdf, share):
    """Три независимых значения колеи с якорями файл:строка."""
    rows = []

    # 1. Геометрия: считаем прямо из отрендеренного URDF, а не из свойства
    #    xacro — так значение доказано, а не переписано.
    geometric = None
    try:
        root = ET.fromstring(sim_urdf)
        ys = {}
        for joint in root.findall('joint'):
            name = joint.get('name')
            if name in ('left_wheel_joint', 'right_wheel_joint'):
                origin = joint.find('origin')
                if origin is not None and origin.get('xyz'):
                    ys[name] = float(origin.get('xyz').split()[1])
        if len(ys) == 2:
            geometric = abs(ys['left_wheel_joint'] - ys['right_wheel_joint'])
    except Exception:
        geometric = None
    rows.append(('геометрия URDF (расстояние между осями колёс)',
                 geometric,
                 _anchor(os.path.join(share, 'description', 'robot_core.xacro'),
                         r'wheel_offset_y', 21)))

    # 2. Плагин DiffDrive симуляции: он и есть источник /odom в sim.
    gz_value = None
    try:
        root = ET.fromstring(sim_urdf)
        for gz in root.findall('gazebo'):
            for plugin in gz.findall('plugin'):
                node = plugin.find('wheel_separation')
                if node is not None and node.text:
                    gz_value = float(node.text.strip())
    except Exception:
        gz_value = None
    rows.append(('плагин gz DiffDrive (одометрия симуляции)',
                 gz_value,
                 _anchor(os.path.join(share, 'description', 'gazebo_control.xacro'),
                         r'<wheel_separation>', 8)))

    # 3. diff_drive_controller на железе. Читаем регуляркой, а не PyYAML:
    #    нужна ещё и строка для якоря, а зависимость тут ни к чему.
    yaml_path = os.path.join(share, 'config', 'epos4_diffdrive', 'ros2_controllers.yaml')
    hw_value = None
    multiplier = None
    try:
        with open(yaml_path, 'r', encoding='utf-8') as handle:
            text = handle.read()
        match = re.search(r'^\s*wheel_separation:\s*([0-9.]+)', text, re.M)
        if match:
            hw_value = float(match.group(1))
        match = re.search(r'^\s*wheel_separation_multiplier:\s*([0-9.]+)', text, re.M)
        if match:
            multiplier = float(match.group(1))
    except OSError:
        pass
    rows.append(('diff_drive_controller на железе',
                 hw_value,
                 _anchor(yaml_path, r'^\s*wheel_separation:', 22)))
    mult_anchor = _anchor(yaml_path, r'^\s*wheel_separation_multiplier:', 29)
    return rows, multiplier, mult_anchor


def print_wheel_separation_warning(rows, multiplier, mult_anchor):
    values = [v for _, v, _ in rows if v is not None]
    print('')
    print('ПРЕДУПРЕЖДЕНИЕ: колея (wheel_separation) задана в трёх местах и они РАЗЪЕХАЛИСЬ')
    for title, value, anchor in rows:
        shown = 'не определено' if value is None else ('%.4f м' % value)
        print('    %-46s %-14s (%s)' % (title, shown, anchor))
    if multiplier is not None and rows[2][1] is not None:
        print('    %-46s %-14s (%s)'
              % ('  ... с калибровочным множителем %.4f' % multiplier,
                 '%.4f м' % (rows[2][1] * multiplier),
                 mult_anchor))
    if len(values) >= 2 and (max(values) - min(values)) > 1e-6:
        print('    Разброс %.3f м (%.0f%% от наибольшего значения).'
              % (max(values) - min(values), 100.0 * (max(values) - min(values)) / max(values)))
    print('    ЧТО ЭТО ЗНАЧИТ: угловая скорость по одометрии считается по РАЗНОЙ колее в')
    print('    симуляции и на железе, поэтому метрики пути (длина траектории, число поворотов,')
    print('    накопленный курс) между режимами напрямую НЕСРАВНИМЫ.')
    print('    ЧТО ДЕЛАТЬ: ничего. Железо откалибровано под свои числа; правка любого из них')
    print('    требует повторной калибровки стенда. Расхождение зафиксировано сознательно.')


# --------------------------------------------------------------- main

def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Проверка паритета URDF между симуляцией и реальным роботом.')
    parser.add_argument('--share', default=None,
                        help='каталог с description/ и config/ (по умолчанию share пакета ar_project)')
    parser.add_argument('--hw-parity', action='store_true',
                        help='сверять с каноном 640x480 (режим «сопоставимо с железом»)')
    parser.add_argument('--verbose', action='store_true',
                        help='печатать извлечённые параметры камеры и нормализованные деревья')
    parser.add_argument('--write-dump', default=None,
                        help='каталог, куда сложить отрендеренные и нормализованные URDF')
    args = parser.parse_args(argv)

    if shutil.which('xacro') is None:
        print('ПРОПУСК: в системе нет команды xacro, отрендерить URDF нечем.')
        print('Установите ros-jazzy-xacro и повторите:  sudo apt install ros-jazzy-xacro')
        print('Либо запустите проверку внутри контейнера/WSL с окружением ROS 2.')
        return 77

    share = args.share or default_share()
    desc = os.path.join(share, 'description')
    sim_root = os.path.join(desc, 'robot.urdf.xacro')
    hw_root = os.path.join(desc, 'robot_hardware.urdf.xacro')

    canon, canon_live = load_canon(args.hw_parity)
    print('Каталог пакета: %s' % share)
    print('Канон камеры (%s): %s'
          % ('железный профиль 640x480' if args.hw_parity else 'профиль симуляции 320x240',
             ', '.join('%s=%s' % (k, canon[k]) for k in sorted(canon))))
    if not canon_live:
        print('ВНИМАНИЕ: fleet_comms.mode_profiles не импортируется, взята встроенная копия канона.')
        print('          Проверка б2 (совпадение дефолтов xacro с каноном) в этом режиме слабее.')

    # use_ros2_control:=false — именно так рендерит симуляционный launch:
    # в sim движение даёт плагин gz DiffDrive, а не ros2_control.
    sim_args = dict(canon)
    sim_args['use_ros2_control'] = 'false'
    hw_args = {'can_interface_name': 'can0'}

    try:
        sim_urdf = render(sim_root, sim_args)
        hw_urdf = render(hw_root, hw_args)
        sim_default_urdf = render(sim_root, {'use_ros2_control': 'false'})
    except RuntimeError as exc:
        print('')
        print('ОШИБКА РЕНДЕРА: %s' % exc)
        return 2

    if args.write_dump:
        os.makedirs(args.write_dump, exist_ok=True)
        for name, text in (('sim.urdf', sim_urdf),
                           ('hardware.urdf', hw_urdf),
                           ('sim_defaults.urdf', sim_default_urdf)):
            with open(os.path.join(args.write_dump, name), 'w', encoding='utf-8') as handle:
                handle.write(text)

    failures = []

    # --- (а) паритет геометрии -------------------------------------------
    norm_sim = normalize(sim_urdf)
    norm_hw = normalize(hw_urdf)
    if args.write_dump:
        for name, text in (('sim.normalized.xml', norm_sim),
                           ('hardware.normalized.xml', norm_hw)):
            with open(os.path.join(args.write_dump, name), 'w', encoding='utf-8') as handle:
                handle.write(text)

    print('')
    if norm_sim == norm_hw:
        print('[а] ПАРИТЕТ ГЕОМЕТРИИ: OK — link/joint без <gazebo> и <ros2_control> совпадают.')
    else:
        print('[а] РАСХОЖДЕНИЕ ГЕОМЕТРИИ sim vs железо:')
        for line in diff_report(norm_sim, norm_hw):
            print('    %s' % line)
        failures.append('геометрия')

    # --- (б) канон камеры -------------------------------------------------
    cameras = extract_gz_cameras(sim_urdf)
    default_cameras = extract_gz_cameras(sim_default_urdf)
    if args.verbose:
        print('')
        print('    извлечено из <gazebo>/<sensor>/<camera> (рендер с каноном):')
        for name in sorted(cameras):
            print('      %s: %s' % (name, cameras[name]))

    problems = check_camera_canon(cameras, canon, 'б1 (рендер с явным каноном)')
    # б2 сверяется ВСЕГДА с sim-каноном: дефолты в robot.urdf.xacro:16-20
    # описывают именно быстрый профиль симуляции, а не режим hw-parity.
    canon_sim, _ = load_canon(False)
    problems += check_camera_canon(default_cameras, canon_sim,
                                   'б2 (дефолты robot.urdf.xacro)')

    print('')
    if problems:
        print('[б] РАСХОЖДЕНИЕ С КАНОНОМ КАМЕРЫ:')
        for line in problems:
            print('    %s' % line)
        print('    Канон живёт в fleet_comms/mode_profiles.py (URDF_CANON_SIM / URDF_CANON_HW);')
        print('    дефолты xacro — в description/robot.urdf.xacro:16-20.')
        failures.append('канон камеры')
    else:
        print('[б] КАНОН КАМЕРЫ: OK — оба сенсора получают width/height/update_rate/clip из канона,')
        print('    и дефолты robot.urdf.xacro совпадают с URDF_CANON_SIM.')

    fov = (cameras.get('camera') or {}).get('horizontal_fov')
    print('    ЗАМЕЧАНИЕ: horizontal_fov=%s задан в xacro литералом и каноном НЕ управляется'
          % (fov or '?'))
    print('    (camera_gazebo_sensors.xacro:19,61). На железе intrinsics приходят из')
    print('    camera_info RealSense, поэтому перцепция sim и hw не идентична по построению.')

    # --- нижний слой на железе -------------------------------------------
    print('')
    r2c_count = hw_urdf.count('<ros2_control')
    if r2c_count == 1:
        print('[г] ЖЕЛЕЗНЫЙ URDF: ровно один блок <ros2_control> — OK.')
    else:
        print('[г] ЖЕЛЕЗНЫЙ URDF: блоков <ros2_control> найдено %d, должен быть ровно один.'
              % r2c_count)
        failures.append('ros2_control на железе')

    if 'gz_ros2_control' in hw_urdf:
        print('[д] В железный URDF просочился симуляционный плагин gz_ros2_control/GazeboSimSystem.')
        print('    controller_manager на Pi такой плагин не найдёт и не поднимется вообще.')
        failures.append('gz_ros2_control на железе')
    else:
        print('[д] ЖЕЛЕЗНЫЙ URDF: симуляционного gz_ros2_control нет — OK.')

    # --- (в) колея --------------------------------------------------------
    rows, multiplier, mult_anchor = wheel_separation_sources(sim_urdf, share)
    print_wheel_separation_warning(rows, multiplier, mult_anchor)

    # --- позиция камеры (для журнала испытаний) ---------------------------
    def camera_origin(urdf_text):
        try:
            root = ET.fromstring(urdf_text)
            for joint in root.findall('joint'):
                if joint.get('name') == 'camera_joint':
                    origin = joint.find('origin')
                    if origin is not None:
                        return origin.get('xyz')
        except Exception:
            pass
        return None

    print('')
    print('Позиция камеры (camera_joint origin xyz): sim=%s, железо=%s'
          % (camera_origin(sim_urdf), camera_origin(hw_urdf)))

    print('')
    if failures:
        print('ИТОГ: ПАРИТЕТ URDF НАРУШЕН — %s.' % ', '.join(failures))
        return 1
    print('ИТОГ: ПАРИТЕТ URDF OK — геометрия, кинематика и позиция камеры совпадают;')
    print('      различается только слой драйверов, а канон камеры соблюдён.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
