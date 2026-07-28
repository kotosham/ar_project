"""Юнит-тесты консоли оператора — БЕЗ ROS.

Весь смысл разделения пакета на «чистые» модули и один rclpy-модуль в том, что
эти тесты запускаются обычным pytest на машине без ROS 2 и без Gazebo:

    cd ar_project/operator_console && python -m pytest test -q

Поэтому первый же тест проверяет само это свойство: если кто-то вкатит
`import rclpy` в preflight_rules или http_api, набор покраснеет сразу, а не
через полгода на чужом ноутбуке.

Часть тестов закрывает КОНКРЕТНЫЕ дефекты, найденные при разборе, а не
гипотетические: CRLF в vlm.env (в прошлом прогоне давал `$'\\r': command not
found`), отсутствие префикса `data:` в SSE (страница молча остаётся пустой при
живом на вид соединении), расхождение AABB комнат с оракулом бенчмарка
(тихо перескорило бы все снятые эпизоды).
"""
import json
import os
import sys
import threading
import urllib.error
import urllib.request

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG = os.path.dirname(_HERE)                       # .../operator_console
_AR = os.path.dirname(_PKG)                         # .../ar_project
for path in (_PKG, os.path.join(_AR, 'fleet_comms')):
    if path not in sys.path:
        sys.path.insert(0, path)

from fleet_comms import mode_profiles as mp          # noqa: E402
from fleet_comms import vlm_env                      # noqa: E402
from operator_console import config_store            # noqa: E402
from operator_console import http_api                # noqa: E402
from operator_console import preflight_rules as pr   # noqa: E402
from operator_console import stack_runner            # noqa: E402
from operator_console import vlm_check               # noqa: E402
from operator_console import worlds_catalog          # noqa: E402

SHARE = os.path.join(_AR, 'ar_project')              # исходники = раскладка share
CATALOG_PATH = os.path.join(SHARE, 'config', 'worlds.yaml')

# Заведомо ненастоящий ключ: длиннее 8 символов, чтобы scrub его вообще брал.
FAKE_KEY = 'sk-НЕ-НАСТОЯЩИЙ-КЛЮЧ-ДЛЯ-ТЕСТА-0123456789'


# ---------------------------------------------------------------------------
# 1. базовое свойство пакета
# ---------------------------------------------------------------------------

def test_pure_modules_never_import_rclpy():
    """Ни один чистый модуль не тянет ROS — иначе тесты нельзя гонять в CI."""
    assert 'rclpy' not in sys.modules, (
        'rclpy оказался загружен при импорте чистых модулей: разделение '
        'сломано, и весь этот набор перестанет запускаться без ROS 2')


# ---------------------------------------------------------------------------
# 2. mode_profiles — единственный источник истины по дельте sim/железо
# ---------------------------------------------------------------------------

def test_profile_copy_cannot_corrupt_global():
    got = mp.profile_for(mp.SIM)
    got['odom_topic'] = 'СЛОМАНО'
    got['required_health'] = ()
    assert mp.profile_for(mp.SIM)['odom_topic'] == '/odom'
    assert mp.profile_for(mp.SIM)['required_health']


def test_hardware_camera_topics_go_through_the_edge_relay():
    """Инвариант единственного потребителя камеры: edge-узлы читают
    /camera_edge/*, а не /camera/camera/* напрямую с Pi
    (edge_bringup.launch.py:29-30 — каждая такая подписка заново открывает
    поток по Wi-Fi и перегружает робота)."""
    hw = mp.profile_for(mp.HARDWARE)
    for key in ('camera_rgb_topic', 'camera_depth_topic', 'camera_info_topic'):
        assert hw[key].startswith('/camera_edge/'), key
    for topic in hw['link_required_topics']:
        assert not topic.startswith('/camera/camera/'), (
            'преflight на железе подписался бы на Pi-топик %s' % topic)


def test_launch_args_are_all_strings():
    """launch подставляет значения как есть — не-строка падает уже в рантайме."""
    for mode in mp.MODES:
        for planner in mp.PLANNERS:
            args = mp.as_launch_args(mode, planner, world_file='/w.sdf')
            bad = {k: v for k, v in args.items() if not isinstance(v, str)}
            assert not bad, bad


def test_flat_planner_needs_no_edge_nodes():
    """FLAT самодостаточен на роботе (MODES.md §2.5) — требовать детектор и
    оркестратор значило бы никогда не дать зелёный свет чистому FLAT-прогону."""
    need = mp.planner_requirements(mp.PLANNER_FLAT)
    assert not any(need.values())
    rows = mp.required_health_for(mp.SIM, mp.PLANNER_FLAT)
    assert 'detector' not in rows and 'planner_orchestrator' not in rows
    assert 'detector' in mp.required_health_for(mp.SIM, mp.PLANNER_VLM)


def test_sim_does_not_require_ekf_rows():
    """В симуляции /odometry/filtered и /diff_cont/odom не публикуются вовсе
    (config/gz_bridge.yaml), поэтому ekf_odometry и wheel_odometry обязаны быть
    советочными — иначе готовность недостижима by design."""
    sim = mp.profile_for(mp.SIM)
    for row in ('ekf_odometry', 'wheel_odometry'):
        assert row in sim['advisory_health']
        assert row not in sim['required_health']
    assert 'ekf_odometry' in mp.profile_for(mp.HARDWARE)['required_health']


def test_unknown_mode_and_planner_fail_loudly():
    with pytest.raises(ValueError):
        mp.profile_for('квартира')
    with pytest.raises(ValueError):
        mp.planner_requirements('нейросеть')


# ---------------------------------------------------------------------------
# 3. vlm_env — контракт «записать можно, прочитать наружу нельзя»
# ---------------------------------------------------------------------------

def test_write_preserves_documentation_and_hides_key(tmp_path):
    template = tmp_path / 'vlm.env.example'
    template.write_text('# как заполнять base_url\nVLM_BASE_URL=\nVLM_API_KEY=\n'
                        'VLM_MODEL=\n', encoding='utf-8')
    target = tmp_path / 'vlm.env'
    vlm_env.write_env_file(str(target), {'VLM_BASE_URL': 'https://api/v1',
                                         'VLM_MODEL': 'qwen-vl',
                                         'VLM_API_KEY': FAKE_KEY},
                           template_path=str(template))
    text = target.read_text(encoding='utf-8')
    assert '# как заполнять base_url' in text, 'документация шаблона потеряна'

    public = vlm_env.public_view(vlm_env.load_env_file(str(target)))
    assert public == {'base_url': 'https://api/v1', 'model': 'qwen-vl',
                      'token_set': True}
    assert FAKE_KEY not in json.dumps(public, ensure_ascii=False)


def test_partial_write_keeps_existing_key(tmp_path):
    """Смена модели не должна стирать уже введённый токен."""
    target = tmp_path / 'vlm.env'
    vlm_env.write_env_file(str(target), {'VLM_API_KEY': FAKE_KEY,
                                         'VLM_MODEL': 'm1'})
    vlm_env.write_env_file(str(target), {'VLM_MODEL': 'm2',
                                         'VLM_API_KEY': None})
    env = vlm_env.load_env_file(str(target))
    assert env['VLM_MODEL'] == 'm2' and env['VLM_API_KEY'] == FAKE_KEY


def test_crlf_file_parses(tmp_path):
    """Регрессия на измеренный дефект: файл с CRLF давал каждому значению
    хвостовой \\r, и bash ругался `$'\\r': command not found`."""
    target = tmp_path / 'vlm.env'
    target.write_bytes(b'VLM_BASE_URL=https://api/v1\r\nVLM_API_KEY=abcdefgh\r\n')
    env = vlm_env.load_env_file(str(target))
    assert env['VLM_BASE_URL'] == 'https://api/v1'
    assert vlm_env.public_view(env)['token_set'] is True


def test_scrub_removes_key_from_error_text():
    text = 'HTTP 401: Authorization: Bearer %s отклонён' % FAKE_KEY
    assert FAKE_KEY not in vlm_env.scrub(text, FAKE_KEY)
    # короткий мусор не должен превращать нормальный текст в решето
    assert vlm_env.scrub('соединение отказано', 'abc') == 'соединение отказано'


def test_missing_file_is_not_an_error(tmp_path):
    assert vlm_env.load_env_file(str(tmp_path / 'нет.env')) == {}


# ---------------------------------------------------------------------------
# 4. config_store — токен физически не попадает в JSON на диске
# ---------------------------------------------------------------------------

def test_token_never_lands_in_config_json(tmp_path):
    cfg = tmp_path / 'console.json'
    env = tmp_path / 'vlm.env'
    store = config_store.ConfigStore(str(cfg), str(env))
    store.vlm_write(base_url='https://api/v1', model='m', token=FAKE_KEY)
    store.save()
    on_disk = cfg.read_text(encoding='utf-8') if cfg.exists() else ''
    assert FAKE_KEY not in on_disk
    assert FAKE_KEY not in json.dumps(store.get(), ensure_ascii=False)
    assert FAKE_KEY not in json.dumps(store.vlm_public(), ensure_ascii=False)
    assert store.vlm_public()['token_set'] is True


def test_patch_rejects_forbidden_and_unknown_keys(tmp_path):
    store = config_store.ConfigStore(str(tmp_path / 'c.json'),
                                     str(tmp_path / 'vlm.env'))
    for bad in ({'vlm_api_key': FAKE_KEY}, {'token': FAKE_KEY},
                {'какой_то_ключ': 1}):
        with pytest.raises(ValueError):
            store.patch(bad, world_ids=['house'])


def test_patch_rejects_unknown_world(tmp_path):
    store = config_store.ConfigStore(str(tmp_path / 'c.json'),
                                     str(tmp_path / 'vlm.env'))
    with pytest.raises(ValueError):
        store.patch({'world': 'нет_такого_мира'}, world_ids=['house'])
    store.patch({'world': 'house'}, world_ids=['house'])
    assert store.get()['world'] == 'house'


# ---------------------------------------------------------------------------
# 5. каталог миров — вторая копия AABB обязана совпадать с оракулом бенчмарка
# ---------------------------------------------------------------------------

def _catalog():
    if not os.path.isfile(CATALOG_PATH):
        pytest.skip('config/worlds.yaml не найден: %s' % CATALOG_PATH)
    return worlds_catalog.load_catalog_file(CATALOG_PATH)


def _runner_rooms():
    """ROOMS из house_scenario_runner.py — ИСТОЧНИК ИСТИНЫ. Читаем текстом,
    чтобы не тащить rclpy, который импортирует сам раннер."""
    import ast
    path = os.path.join(SHARE, 'scripts', 'house_scenario_runner.py')
    if not os.path.isfile(path):
        pytest.skip('house_scenario_runner.py не найден')
    tree = ast.parse(open(path, encoding='utf-8').read())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
                getattr(t, 'id', '') == 'ROOMS' for t in node.targets):
            return ast.literal_eval(node.value)
    pytest.skip('ROOMS не найден в house_scenario_runner.py')


def test_house_rooms_match_the_benchmark_oracle():
    """Каталог — ВТОРАЯ копия AABB комнат. Оракул сценариев считает in_room по
    house_scenario_runner.ROOMS; расхождение тихо перескорило бы каждый уже
    снятый эпизод, поэтому копия сверяется здесь, а не «на глаз»."""
    house = _catalog().get('house')
    assert house is not None, 'в каталоге нет мира house'
    catalog_rooms = worlds_catalog.rooms_aabb_xxyy(house)
    runner_rooms = {k: tuple(v) for k, v in _runner_rooms().items()}
    assert catalog_rooms == runner_rooms, (
        'AABB комнат разошлись с оракулом:\nкаталог: %s\nраннер:  %s'
        % (catalog_rooms, runner_rooms))


def test_catalog_declares_real_world_names():
    """world_name должен совпадать с <world name=...> в SDF: раннер бьёт в
    сервисы /world/<name>/*, и опечатка тут даёт молчаливый отказ спавна."""
    catalog = _catalog()
    checked = 0
    for entry in catalog.list():
        sdf = os.path.join(SHARE, entry.file)
        if not os.path.isfile(sdf):
            continue
        actual = worlds_catalog.read_world_name(sdf)
        if actual:
            assert actual == entry.world_name, (
                '%s: в каталоге world_name=%r, в файле %r'
                % (entry.id, entry.world_name, actual))
            checked += 1
    assert checked >= 5, 'проверено слишком мало миров: %d' % checked


def test_usable_worlds_have_previews_on_disk():
    """Иначе карточка мира в интерфейсе окажется пустым местом без объяснения."""
    for entry in _catalog().usable():
        public = entry.to_public(SHARE)
        assert public.get('preview_url'), '%s: нет ссылки на превью' % entry.id
        path = os.path.join(SHARE, entry.preview)
        assert os.path.isfile(path), '%s: файл превью отсутствует: %s' % (entry.id, path)
        assert os.path.getsize(path) > 1000, '%s: превью подозрительно пустое' % entry.id


def test_broken_worlds_are_not_offered():
    catalog = _catalog()
    usable_ids = {e.id for e in catalog.usable()}
    for entry in catalog.list():
        if entry.status == worlds_catalog.STATUS_BROKEN:
            assert entry.id not in usable_ids
            assert entry.status_note, '%s: не объяснено, почему мир непригоден' % entry.id


# ---------------------------------------------------------------------------
# 6. преflight — вердикт меняется от фактов, а не от настроения
# ---------------------------------------------------------------------------

def _snapshot(mode=mp.SIM, planner=mp.PLANNER_VLM, drop=(), levels=None):
    """Синтетический SNAPSHOT, в котором ВСЁ зелёное, минус то, что убрали."""
    profile = mp.profile_for(mode)
    rows = {}
    for name in mp.required_health_for(mode, planner):
        if name in drop:
            continue
        rows[name] = {'level': (levels or {}).get(name, 0), 'message': 'ok',
                      'age_s': 0.5, 'values': {}}
    beats = {n: {'status': 'OK', 'age_s': 0.4, 'epoch': 0, 'cpu_load': 0.1,
                 'latency_ms': 12.0}
             for n in ('search_coordinator', 'planner_orchestrator', 'detector')}
    return {
        'health_rows': rows,
        'health_age_s': 0.5,
        'heartbeats': beats,
        'topic_ages': {t: 0.3 for t in profile['link_required_topics']},
        'graph_nodes': ['planner_server', 'controller_server', 'bt_navigator',
                        'twist_mux', 'search_coordinator'],
        'nav2_lifecycle': {n: 'active' for n in profile['nav2_lifecycle_nodes']},
        'mission': None,
        'activity': [],
        'vlm': {'token_set': True, 'base_url': 'https://api/v1', 'model': 'm',
                'probe': None},
        'world': {'id': 'house', 'status': 'ok', 'status_note': ''},
        'catalog_problems': [],
        'ros_domain_id': 0,
        'rmw': 'rmw_fastrtps_cpp',
    }


def test_everything_green_is_ready():
    verdict = pr.evaluate(_snapshot(), mp.SIM, mp.PLANNER_VLM)
    public = verdict.to_public()
    assert public['ready'] is True, public['blocking']
    assert public['link_phase'] == pr.PHASE_CONNECTED


def test_missing_required_row_blocks_and_names_itself():
    verdict = pr.evaluate(_snapshot(drop=('scan',)), mp.SIM, mp.PLANNER_VLM)
    public = verdict.to_public()
    assert public['ready'] is False
    text = json.dumps(public, ensure_ascii=False)
    assert 'scan' in text.lower()


def test_stale_link_topic_is_not_reported_as_connected():
    """Главная ложь, которую интерфейс не имеет права сказать: «подключено»
    при мёртвом потоке."""
    snap = _snapshot(mode=mp.HARDWARE)
    snap['topic_ages']['/scan'] = 42.0
    verdict = pr.evaluate(snap, mp.HARDWARE, mp.PLANNER_VLM)
    public = verdict.to_public()
    assert public['ready'] is False
    assert public['link_phase'] != pr.PHASE_CONNECTED


def test_no_data_at_all_is_waiting_not_error():
    """Пустой граф — это «ещё жду робота», а не «всё сломалось»: иначе экран
    ожидания сразу красный и оператор идёт чинить исправное."""
    empty = {'health_rows': {}, 'health_age_s': None, 'heartbeats': {},
             'topic_ages': {}, 'graph_nodes': [], 'nav2_lifecycle': {},
             'mission': None, 'activity': [], 'catalog_problems': [],
             'vlm': {'token_set': False, 'base_url': '', 'model': '', 'probe': None},
             'world': {'id': 'house', 'status': 'ok', 'status_note': ''},
             'ros_domain_id': 0, 'rmw': 'rmw_zenoh_cpp'}
    public = pr.evaluate(empty, mp.HARDWARE, mp.PLANNER_VLM,
                         tcp_probe=lambda h, p: (False, 'соединение отказано')).to_public()
    assert public['ready'] is False
    assert public['link_phase'] == pr.PHASE_WAITING


def test_sim_marks_epos4_as_not_proving_anything():
    """control_epos4 в симуляции зелёный из-за /joint_states от Gazebo и о
    реальных приводах не говорит ничего — интерфейс обязан это подписать."""
    checks = pr.evaluate(_snapshot(), mp.SIM, mp.PLANNER_VLM).to_public()['checks']
    notes = ' '.join(str(c.get('note_ru') or '') for c in checks)
    assert 'симуляц' in notes.lower(), 'нет пометки про недостоверность control_epos4'


def test_vlm_planner_without_base_url_is_blocked():
    """Пустой base_url отправляет make_client в MockVlmClient МОЛЧА
    (vlm_client.py:make_client) — прогон был бы не тем, что думает оператор."""
    snap = _snapshot()
    snap['vlm'] = {'token_set': False, 'base_url': '', 'model': '', 'probe': None}
    public = pr.evaluate(snap, mp.SIM, mp.PLANNER_VLM).to_public()
    assert public['ready'] is False
    assert any('vlm' in str(c.get('id', '')) for c in public['checks'])
    # тот же снимок с planner=flat не должен спотыкаться о кредах
    flat = pr.evaluate(_snapshot(planner=mp.PLANNER_FLAT), mp.SIM,
                       mp.PLANNER_FLAT).to_public()
    assert flat['ready'] is True, flat['blocking']


def test_hardware_router_unreachable_is_named_explicitly():
    """Недоступный zenoh-роутер — первая причина «не вижу робота», и она должна
    называться прямо, а не прятаться за общим «нет данных»."""
    public = pr.evaluate(_snapshot(mode=mp.HARDWARE), mp.HARDWARE, mp.PLANNER_VLM,
                         edge_host='10.0.0.9',
                         tcp_probe=lambda h, p: (False, 'соединение отказано')).to_public()
    ids = [c['id'] for c in public['checks']]
    assert 'router' in ids
    router = [c for c in public['checks'] if c['id'] == 'router'][0]
    assert router['level'] in (pr.LEVEL_ERROR, pr.LEVEL_WARN, pr.LEVEL_WAIT)
    assert '10.0.0.9' in (router['message'] + router.get('hint', ''))


def test_mission_view_detects_a_running_mission():
    """Оркестратор молча отбрасывает новое задание, пока идёт миссия
    (orchestrator_node.py:310-315) — консоль обязана это увидеть заранее."""
    activity = [{'event': 'mission_start', 'target': 'chair', 'seq': 1},
                {'event': 'step_start', 'step': 4, 'seq': 2}]
    view = pr.merge_mission_view(None, activity)
    assert view.get('source') == 'vlm' and view.get('state') == 'RUNNING'
    assert view.get('step') == 4
    done = activity + [{'event': 'mission_end', 'steps': 5, 'seq': 3}]
    assert pr.merge_mission_view(None, done).get('state') != 'RUNNING'


# ---------------------------------------------------------------------------
# 7. stack_runner
# ---------------------------------------------------------------------------

def test_argv_targets_the_single_root_launch():
    argv = stack_runner.build_launch_argv(
        mode=mp.SIM, planner=mp.PLANNER_VLM, layer='all',
        world_file='/share/worlds/house.sdf', gui=False, rviz=False,
        dashboard_port=8088, venv_python='/opt/ot_venv/bin/python')
    joined = ' '.join(argv)
    assert argv[:2] == ['ros2', 'launch']
    assert 'mission_bringup.launch.py' in joined
    assert 'mode:=sim' in joined and 'planner:=vlm' in joined
    assert 'world:=/share/worlds/house.sdf' in joined
    assert not any('api_key' in a.lower() for a in argv), (
        'ключ в argv виден в ps — он обязан идти только через окружение')


class _FakePopen:
    """Подставной процесс: тесты не имеют права звать настоящий ros2 launch."""

    def __init__(self, argv, **kwargs):
        self.argv = argv
        self.pid = 4242
        self.returncode = None
        self._lines = ['[INFO] стек поднимается\n', '[INFO] готово\n']
        self.stdout = self
        self.terminated = False

    def readline(self):
        return self._lines.pop(0) if self._lines else ''

    def __iter__(self):
        while self._lines:
            yield self._lines.pop(0)

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.returncode = 0
        return 0

    def stay_alive(self):
        self._lines = []          # поток исчерпан, но процесс ещё не завершён

    def terminate(self):
        self.terminated = True
        self.returncode = 0

    def kill(self):
        self.terminated = True
        self.returncode = -9

    def send_signal(self, _sig):
        self.terminated = True
        self.returncode = 0


def test_runner_start_stop_and_log(tmp_path):
    runner = stack_runner.StackRunner(popen=_FakePopen)
    runner.start(['ros2', 'launch', 'ar_project', 'mission_bringup.launch.py'],
                 log_path=str(tmp_path / 'stack.log'))
    status = runner.status()
    assert status['running'] is True and status['pid'] == 4242
    runner.stop(grace_s=0.1)
    assert runner.is_running() is False
    runner.close()


def test_second_start_is_refused():
    """Два `ros2 launch` подряд -- это два coordinator_node и два
    detect_target_server в одном графе. Измерено ранее на этом стенде: выживший
    от прошлого эпизода узел даёт «Ignoring unexpected goal response», потерю
    целей движения и path_length_m 0.00 при внешне здоровых SLAM и детекторе.
    Поэтому повторный старт обязан ОТКАЗЫВАТЬ, а не молча плодить процессы."""
    class _Alive(_FakePopen):
        def poll(self):
            return None          # процесс жив, что бы ни случилось с потоком

    runner = stack_runner.StackRunner(popen=_Alive)
    runner.start(['ros2', 'launch', 'x'])
    with pytest.raises(RuntimeError):
        runner.start(['ros2', 'launch', 'x'])
    runner.stop(grace_s=0.1)
    runner.close()


# ---------------------------------------------------------------------------
# 8. vlm_check — ключ не должен вылезти ни в одном тексте
# ---------------------------------------------------------------------------

def test_probe_without_base_url_does_not_touch_network():
    result = vlm_check.probe('', FAKE_KEY, 'm')
    assert result['ok'] is False
    assert FAKE_KEY not in json.dumps(result, ensure_ascii=False)
    assert result.get('error_ru')


def test_probe_result_never_carries_the_key():
    """Неразрешимый хост: urllib охотно кладёт в текст исключения весь запрос."""
    result = vlm_check.probe('http://несуществующий.хост.тест.invalid/v1',
                             FAKE_KEY, 'm', timeout_s=2.0)
    assert result['ok'] is False
    assert FAKE_KEY not in json.dumps(result, ensure_ascii=False)


def test_status_classification_is_in_russian():
    cyr = set('абвгдеёжзийклмнопрстуфхцчшщъыьэюя')
    for code in (401, 403, 404, 500):
        kind = vlm_check.classify_status(code)
        assert kind, code
        # ERROR_RU: вид ошибки -> (что случилось, что делать). Подсказка
        # обязательна: сообщение без действия оператору бесполезно.
        entry = vlm_check.ERROR_RU.get(kind)
        assert entry, 'для вида %r нет русского текста' % kind
        message, hint = entry
        assert cyr & set(message.lower()), 'текст для %d не по-русски: %r' % (code, message)
        assert hint and cyr & set(hint.lower()), 'для %d нет подсказки' % code


# ---------------------------------------------------------------------------
# 9. http_api — поднимается по-настоящему и отвечает на каждый маршрут
# ---------------------------------------------------------------------------

class _FakeBackend:
    """Ровно интерфейс ConsoleBackend, без ROS."""

    def __init__(self, token=''):
        self._token = token
        self.missions = []
        self.stopped = 0

    def console_token(self):
        return self._token

    def get_config(self):
        return {'mode': 'sim', 'planner': 'vlm', 'world': 'house',
                'console_token': self._token}

    def set_config(self, patch):
        if 'плохой' in patch:
            raise ValueError('Неизвестный ключ настройки: плохой')
        return self.get_config()

    def vlm_public(self):
        return {'base_url': 'https://api/v1', 'model': 'm', 'token_set': True}

    def vlm_write(self, base_url=None, model=None, token=None):
        return dict(self.vlm_public(), restart_required=True, note_ru='ок')

    def vlm_probe(self):
        return {'ok': True, 'models': ['m'], 'latency_ms': 12}

    def vlm_models(self):
        return {'ok': True, 'models': ['m']}

    def worlds(self):
        return [{'id': 'house', 'title': 'Квартира', 'usable': True,
                 'preview_url': '/api/worlds/house/preview.svg'}]

    def world_preview(self, world_id, kind):
        if world_id != 'house':
            return None
        return ('image/svg+xml', b'<svg xmlns="http://www.w3.org/2000/svg"/>')

    def preflight(self):
        return {'ready': True, 'checks': [], 'blocking': [],
                'link_phase': 'connected', 'link_message': 'подключено'}

    def link(self):
        return {'phase': 'connected', 'message': 'подключено', 'topics': {},
                'mode': 'sim'}

    def state(self):
        return {'ros_connected': True}

    def sse_snapshot(self):
        return {'preflight': self.preflight(), 'server_time': 1.0}

    def stack_start(self):
        return {'running': True, 'pid': 1}

    def stack_stop(self):
        return {'running': False}

    def stack_status(self):
        return {'running': False, 'pid': None}

    def stack_log(self, since):
        return {'lines': [], 'next': since, 'total': 0}

    def mission_start(self, text):
        if not text.strip():
            raise ValueError('Задание пустое.')
        if text == 'занято':
            raise ValueError('Миссия уже идёт (шаг 3).')
        self.missions.append(text)
        return {'ok': True, 'channel': '/vlm_mission', 'text': text}

    def mission_stop(self):
        self.stopped += 1
        return {'ok': True, 'stopped_motion': True, 'planner_cancelled': True}

    def seed_map(self):
        return {'ok': True}

    def dashboard_base_url(self):
        return 'http://127.0.0.1:1'          # заведомо мёртвый -> ждём 502


class _Server:
    """Порт 0 = ядро выдаёт свободный и он занят НЕПРЕРЫВНО.

    Схема «найти свободный порт, закрыть сокет, потом слушать» даёт гонку:
    между закрытием и bind порт успевает занять соседний тест, и клиент
    получает ConnectionAbortedError на ровном месте.
    """

    def __init__(self, backend):
        self.httpd = http_api.serve(backend, '127.0.0.1', 0)
        self.port = self.httpd.server_address[1]
        threading.Thread(target=self.httpd.serve_forever, daemon=True).start()

    def url(self, path):
        return 'http://127.0.0.1:%d%s' % (self.port, path)

    def request(self, path, method='GET', body=None, headers=None):
        data = json.dumps(body).encode('utf-8') if body is not None else None
        req = urllib.request.Request(self.url(path), data=data, method=method,
                                     headers=headers or {})
        if data is not None:
            req.add_header('Content-Type', 'application/json')
        try:
            with urllib.request.urlopen(req, timeout=10) as resp:
                return resp.status, resp.read()
        except urllib.error.HTTPError as exc:
            return exc.code, exc.read()

    def close(self):
        self.httpd.shutdown()
        self.httpd.server_close()


@pytest.fixture()
def server():
    srv = _Server(_FakeBackend())
    yield srv
    srv.close()


def test_every_route_answers(server):
    """Каждый маршрут из таблицы отвечает не-500 с настоящим сервером."""
    cases = [
        ('GET', '/', None), ('GET', '/api/state', None),
        ('GET', '/api/config', None), ('POST', '/api/config', {'mode': 'sim'}),
        ('GET', '/api/vlm', None),
        ('POST', '/api/vlm/token', {'base_url': 'https://api/v1'}),
        ('POST', '/api/vlm/test', {}), ('GET', '/api/vlm/models', None),
        ('GET', '/api/worlds', None),
        ('GET', '/api/worlds/house/preview.svg', None),
        ('GET', '/api/preflight', None), ('GET', '/api/link', None),
        ('POST', '/api/stack/start', {}), ('POST', '/api/stack/stop', {}),
        ('GET', '/api/stack/status', None), ('GET', '/api/stack/log?since=0', None),
        ('POST', '/api/mission/start', {'text': 'найди стул'}),
        ('POST', '/api/mission/stop', {}), ('POST', '/api/seed_map', {}),
    ]
    for method, path, body in cases:
        status, payload = server.request(path, method, body)
        assert status < 500, '%s %s -> %d: %s' % (method, path, status, payload[:200])


def test_page_is_html_and_russian(server):
    status, payload = server.request('/')
    assert status == 200
    text = payload.decode('utf-8')
    assert '<html' in text.lower() or '<!doctype' in text.lower()
    assert 'Симуляц' in text and 'Реальн' in text


def test_unknown_route_is_404_with_russian_error(server):
    status, payload = server.request('/api/no-such-route')
    assert status == 404
    assert 'error' in json.loads(payload.decode('utf-8'))


def test_value_error_becomes_400_not_500(server):
    status, payload = server.request('/api/mission/start', 'POST', {'text': '   '})
    assert status == 400
    assert json.loads(payload.decode('utf-8'))['error']


def test_busy_mission_is_reported_not_swallowed(server):
    """Пользователь должен УВИДЕТЬ, что задание не принято, а не решить,
    что оно ушло (orchestrator_node.py:310-315 отбрасывает молча)."""
    status, payload = server.request('/api/mission/start', 'POST',
                                     {'text': 'занято'})
    assert status == 400
    assert 'уже идёт' in json.loads(payload.decode('utf-8'))['error']


def test_missing_preview_is_404_with_a_hint(server):
    status, payload = server.request('/api/worlds/no-such-world/preview.svg')
    assert status == 404
    assert 'make_world_previews' in json.loads(payload.decode('utf-8'))['error']


def test_dashboard_without_slash_redirects(server):
    """Без завершающего слэша относительные пути дашборда резолвятся в корень
    консоли, и встроенный монитор показал бы пустой шаблон."""
    req = urllib.request.Request(server.url('/dashboard'))

    class _NoRedirect(urllib.request.HTTPRedirectHandler):
        def redirect_request(self, *_a, **_kw):
            return None

    opener = urllib.request.build_opener(_NoRedirect)
    try:
        with opener.open(req, timeout=5) as resp:
            status, location = resp.status, resp.headers.get('Location')
    except urllib.error.HTTPError as exc:
        status, location = exc.code, exc.headers.get('Location')
    assert status in (301, 302, 308)
    assert location == '/dashboard/'


def test_dead_dashboard_gives_502_in_russian(server):
    status, payload = server.request('/dashboard/')
    assert status == 502
    assert json.loads(payload.decode('utf-8'))['error']


def test_sse_frame_carries_the_data_prefix():
    """Регрессия на реальный класс дефекта: без 'data: ' EventSource молча
    игнорирует кадр — соединение выглядит живым, onerror не срабатывает,
    страница остаётся пустой навсегда."""
    srv = _Server(_FakeBackend())
    try:
        with urllib.request.urlopen(srv.url('/api/events'), timeout=10) as resp:
            assert resp.headers.get('Content-Type', '').startswith('text/event-stream')
            first = resp.readline().decode('utf-8')
        assert first.startswith('data: '), repr(first)
        json.loads(first[len('data: '):])
    finally:
        srv.close()


def test_console_token_gate_blocks_api_but_not_the_page():
    srv = _Server(_FakeBackend(token='console-secret-42'))
    try:
        assert srv.request('/api/config')[0] == 401
        assert srv.request('/')[0] == 200          # страница обязана открыться
        assert srv.request('/api/state')[0] == 200  # иначе не показать причину
        ok = srv.request('/api/config', headers={'X-Console-Token': 'console-secret-42'})
        assert ok[0] == 200
    finally:
        srv.close()


def test_no_response_ever_contains_a_secret(server):
    """Сквозная проверка: обойти все GET-маршруты и убедиться, что ключа нет."""
    for path in ('/', '/api/state', '/api/config', '/api/vlm', '/api/worlds',
                 '/api/preflight', '/api/link', '/api/stack/status'):
        _status, payload = server.request(path)
        assert FAKE_KEY.encode('utf-8') not in payload
        assert b'VLM_API_KEY' not in payload
