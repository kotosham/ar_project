"""Страница мастера настройки консоли оператора: одна константа PAGE и render_page().

Модуль намеренно без импортов и без единого внешнего ресурса (шрифта, CDN,
картинки): консоль поднимается на стенде, где интернета может не быть вообще, а
`mission_dashboard.py` уже доказал, что офлайн-страница на голом stdlib+JS
покрывает задачу. Все цвета — те же переменные, что в
`fleet_comms/mission_dashboard.py:242-243`, чтобы встроенный в шаг «Работа»
iframe дашборда не выглядел чужой вставкой.

Разделение с `http_api.py` ровно одно: здесь нет ни одного знания о протоколе
сервера, кроме URL-ов маршрутов; здесь же не делается кодировка — за неё
отвечает `render_page()`, чтобы обработчик HTTP не занимался encode('utf-8').

Секретов на странице нет по построению: поле токена всегда пустое при загрузке,
сервер никогда не отдаёт значение обратно (`fleet_comms/vlm_env.py:public_view`
— единственная функция, которой разрешено что-то отдавать наружу), а состояние
показывается бейджем «токен задан / не задан» из `token_set`.
"""

# Один блок <script>; его можно выдернуть и прогнать `node --check` — так и
# проверялось при написании, потому что незакрытая скобка в JS не ловится
# `python -m py_compile` и страница молча осталась бы пустой.
PAGE = r"""<!DOCTYPE html>
<html lang="ru"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Консоль оператора</title>
<style>
:root{--bg:#101418;--card:#1a2026;--line:#2a333c;--fg:#dbe4ec;--dim:#8b98a5;
 --ok:#33c06e;--warn:#e2b93b;--stale:#e28f3b;--err:#e25555;--accent:#6ab7ff;
 color-scheme:dark}
*{box-sizing:border-box;margin:0}
body{background:var(--bg);color:var(--fg);font:14px/1.45 system-ui,Segoe UI,Roboto,sans-serif}
code,.mono{font-family:ui-monospace,Consolas,monospace}
h1{font-size:17px;margin-bottom:10px}
h2{font-size:16px;margin-bottom:4px}
h3{font-size:13px;color:var(--dim);text-transform:uppercase;letter-spacing:.06em;margin:16px 0 8px}
p.hint{color:var(--dim);font-size:12px;margin-top:4px}
.app{display:grid;grid-template-columns:300px 1fr;gap:14px;padding:14px;min-height:100vh}
@media(max-width:900px){.app{grid-template-columns:1fr}}
aside{display:flex;flex-direction:column;gap:12px}
.panel{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:10px}
.stepper{list-style:none;padding:0;display:flex;flex-direction:column;gap:4px}
.stepper li{display:flex;align-items:center;gap:8px;padding:8px 10px;border-radius:8px;
 border:1px solid transparent;cursor:pointer;color:var(--dim)}
.stepper li:hover{border-color:var(--line)}
.stepper li.on{background:var(--card);border-color:var(--accent);color:var(--fg);font-weight:600}
.stepper li.locked{opacity:.45;cursor:not-allowed}
.stepper .num{width:22px;height:22px;border-radius:50%;background:var(--line);
 display:flex;align-items:center;justify-content:center;font-size:12px;flex:0 0 auto}
.stepper li.on .num{background:var(--accent);color:#0b0f13}
.plate{border-radius:8px;padding:10px;font-weight:700;text-align:center;letter-spacing:.03em}
.plate.ok{background:rgba(51,192,110,.14);border:1px solid var(--ok);color:var(--ok)}
.plate.err{background:rgba(226,85,85,.14);border:1px solid var(--err);color:var(--err)}
.plate.wait{background:rgba(226,143,59,.14);border:1px solid var(--stale);color:var(--stale)}
.chk{display:flex;gap:8px;padding:6px 0;border-bottom:1px dashed var(--line)}
.chk:last-child{border-bottom:0}
.chk .dot{width:10px;height:10px;border-radius:50%;margin-top:5px;flex:0 0 auto;background:var(--dim)}
.chk.lv-ok .dot{background:var(--ok)}
.chk.lv-warn .dot{background:var(--warn)}
.chk.lv-error .dot{background:var(--err)}
.chk.lv-wait .dot{background:var(--stale)}
.chk .ct{font-weight:600}
.chk .ct small{color:var(--dim);font-weight:400;margin-left:4px}
.chk .cm{font-size:12px;color:var(--fg)}
.chk .ch{font-size:12px;color:var(--dim);margin-top:2px}
.chk .cn{font-size:12px;color:var(--dim);margin-top:2px;font-style:italic}
main{min-width:0}
.pane{display:none}
.pane.on{display:block}
.cards{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:10px;margin-top:8px}
.big{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-top:10px}
@media(max-width:700px){.big{grid-template-columns:1fr}}
.pick{background:var(--card);border:1px solid var(--line);border-radius:12px;padding:14px;cursor:pointer;text-align:left;color:inherit;font:inherit;width:100%}
.pick:hover{border-color:var(--accent)}
.pick.sel{border-color:var(--ok);box-shadow:inset 0 0 0 1px var(--ok)}
.pick.dead{opacity:.42;cursor:not-allowed;border-style:dashed}
.pick .t{font-size:16px;font-weight:700;margin-bottom:4px}
.pick .d{font-size:12px;color:var(--dim)}
.wcard img{width:100%;height:150px;object-fit:contain;background:#0b0f13;border-radius:8px;border:1px solid var(--line);display:block}
.wcard .meta{font-size:12px;color:var(--dim);margin-top:6px}
.badge{display:inline-block;padding:2px 8px;border-radius:20px;font-size:11px;font-weight:700;border:1px solid var(--line)}
.badge.ok{color:var(--ok);border-color:var(--ok)}
.badge.warn{color:var(--warn);border-color:var(--warn)}
.badge.err{color:var(--err);border-color:var(--err)}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin-top:8px}
label.f{display:block;margin-top:10px}
label.f span{display:block;font-size:12px;color:var(--dim);margin-bottom:3px}
input[type=text],input[type=password],select,textarea{background:#0d1216;color:var(--fg);
 border:1px solid var(--line);border-radius:8px;padding:8px 10px;font:inherit;width:100%}
input:focus,select:focus{outline:1px solid var(--accent)}
button{background:var(--card);color:var(--fg);border:1px solid var(--line);border-radius:8px;
 padding:8px 14px;font:inherit;cursor:pointer}
button:hover{border-color:var(--accent)}
button.primary{background:var(--accent);color:#0b0f13;border-color:var(--accent);font-weight:700}
button.danger{border-color:var(--err);color:var(--err)}
button:disabled{opacity:.45;cursor:not-allowed}
.chain{display:flex;gap:6px;flex-wrap:wrap;margin:10px 0}
.chain span{padding:3px 9px;border-radius:20px;border:1px solid var(--line);font-size:12px;color:var(--dim)}
.chain span.done{border-color:var(--ok);color:var(--ok)}
#log{background:#0b0f13;border:1px solid var(--line);border-radius:10px;padding:8px 10px;
 height:320px;overflow:auto;font-family:ui-monospace,Consolas,monospace;font-size:12px;white-space:pre-wrap}
#log .e{color:var(--err)}
.linkbox{border:1px solid var(--line);background:var(--card);border-radius:12px;padding:16px;margin-top:10px}
.linkbox.ok{border-color:var(--ok);background:rgba(51,192,110,.10)}
.linkbox .big-title{font-size:22px;font-weight:800;letter-spacing:.04em;display:flex;align-items:center;gap:10px}
.linkbox.ok .big-title{color:var(--ok)}
.pulse{width:14px;height:14px;border-radius:50%;background:var(--stale);animation:p 1.2s infinite}
.linkbox.ok .pulse{background:var(--ok);animation:none}
@keyframes p{0%{opacity:.25}50%{opacity:1}100%{opacity:.25}}
.trow{display:flex;justify-content:space-between;gap:10px;padding:3px 0;border-bottom:1px dashed var(--line);font-size:12px}
.trow .good{color:var(--ok)}.trow .warn{color:var(--warn)}.trow .bad{color:var(--err)}
ol.steps-hw{margin:8px 0 0 18px;font-size:13px}
ol.steps-hw li{margin-bottom:10px}
pre.cmd{background:#0b0f13;border:1px solid var(--line);border-radius:8px;padding:8px 10px;
 overflow-x:auto;font-size:12px;white-space:pre-wrap;word-break:break-word;margin-top:4px}
#frame{width:100%;min-height:640px;height:70vh;border:1px solid var(--line);border-radius:10px;background:#0b0f13}
.bigerr{display:none;margin-top:10px;padding:12px 14px;border-radius:10px;font-weight:700;
 border:1px solid var(--err);background:rgba(226,85,85,.14);color:var(--err)}
.bigerr.on{display:block}
.bigerr.good{border-color:var(--ok);background:rgba(51,192,110,.14);color:var(--ok)}
.toast{position:fixed;right:14px;bottom:14px;max-width:420px;padding:10px 14px;border-radius:10px;
 background:var(--card);border:1px solid var(--line);display:none;z-index:9;white-space:pre-wrap}
.toast.on{display:block}
.toast.err{border-color:var(--err);color:var(--err)}
.toast.ok{border-color:var(--ok);color:var(--ok)}
.mline{margin-top:8px;font-size:13px;color:var(--dim)}
.ro{color:var(--dim);font-size:12px;margin-top:6px}
</style></head><body>

<div id="toast" class="toast"></div>

<div class="app">
<aside>
  <div>
    <h1>Консоль оператора</h1>
    <ol class="stepper" id="stepper"></ol>
  </div>
  <div class="panel">
    <div id="readyPlate" class="plate wait">ПРОВЕРКИ ЕЩЁ НЕ ПРИХОДИЛИ</div>
    <div id="checks" style="margin-top:8px"></div>
  </div>
  <div class="panel ro">
    <div>Режим: <b id="sumMode">—</b> · планировщик: <b id="sumPlanner">—</b></div>
    <div>Мир: <b id="sumWorld">—</b></div>
    <div>Стек: <b id="sumStack">—</b></div>
    <div id="sumRos">ROS: —</div>
    <div style="margin-top:6px">версия страницы __CONSOLE_VERSION__</div>
  </div>
</aside>

<main>

<section class="pane" id="pane1">
  <h2>Шаг 1. Режим работы</h2>
  <p class="hint">Верхний слой (детектор, VLM-оркестратор, исполнительный протокол, монитор миссии) в обоих режимах ОДИН И ТОТ ЖЕ. Различается нижний слой: Gazebo против RealSense + приводов на CAN.</p>
  <div class="big">
    <button class="pick" id="modeSim" onclick="setMode('sim')">
      <div class="t">Симуляция (Gazebo)</div>
      <div class="d">Всё поднимается на этой машине одной кнопкой. Консоль сама запускает и останавливает стек, показывает журнал запуска. Подходит для проверки логики и для демонстрации без железа.</div>
    </button>
    <button class="pick" id="modeHw" onclick="setMode('hardware')">
      <div class="t">Реальный робот</div>
      <div class="d">Нижний слой живёт на Raspberry Pi, верхний — на edge-боксе. Консоль ничего не запускает: она ждёт, пока робот сам выйдет на связь, и показывает, каких потоков ещё нет.</div>
    </button>
  </div>
  <div class="row"><button class="primary" onclick="go(2)">Далее: планировщик</button></div>
</section>

<section class="pane" id="pane2">
  <h2>Шаг 2. Планировщик</h2>
  <p class="hint">Планировщик решает, КУДА ехать и что делать дальше. Ниже — что именно меняется.</p>
  <div class="cards">
    <button class="pick" id="plVlm" onclick="setPlanner('vlm')">
      <div class="t">VLM</div>
      <div class="d">Кадр с метками и карта уходят в облачную визуально-языковую модель, она возвращает список действий. Нужны детектор, VLM-оркестратор и рабочий токен API. Самый «умный» и самый требовательный к связи режим.</div>
    </button>
    <button class="pick" id="plFlat" onclick="setPlanner('flat')">
      <div class="t">FLAT (без VLM)</div>
      <div class="d">Классический поиск по фронтирам и наведение на детекцию, без обращений к внешней модели. Работает офлайн и без токена; задание можно писать фразой.</div>
    </button>
    <button class="pick" id="plMock" onclick="setPlanner('mock')">
      <div class="t">Mock-планировщик</div>
      <div class="d">Оркестратор поднимается, но вместо облака отвечает заглушка. Нужен, чтобы проверить весь путь «задание → действие → движение», не тратя запросы к API.</div>
    </button>
  </div>
  <div class="row"><button onclick="go(1)">Назад</button><button class="primary" onclick="go(3)">Далее: VLM</button></div>
</section>

<section class="pane" id="pane3">
  <h2>Шаг 3. Доступ к VLM</h2>
  <p class="hint">Значения пишутся в файл <code>vlm.env</code> (права 0600, в git не попадает). Токен НИКОГДА не отдаётся обратно: ни на страницу, ни в журнал, ни в ROS-параметры — поэтому поле токена при каждой загрузке страницы пустое.</p>
  <label class="f"><span>Адрес API (base URL)</span>
    <input type="text" id="vlmBase" placeholder="https://.../v1" autocomplete="off"></label>
  <p class="hint">Адрес должен оканчиваться на <code>/v1</code> и НЕ содержать <code>/chat/completions</code> — лишний хвост консоль отрежет сама и предупредит.</p>

  <label class="f"><span>Модель</span>
    <select id="vlmModel"><option value="">— нажмите «Проверить связь», чтобы получить список —</option></select></label>
  <label class="f"><span>…или впишите имя модели вручную (если сервер не отдаёт список)</span>
    <input type="text" id="vlmModelText" placeholder="например: qwen2.5-vl-72b-instruct" autocomplete="off"></label>

  <label class="f"><span>Токен API <span class="badge" id="tokBadge">не задан</span></span>
    <input type="password" id="vlmToken" placeholder="токен вводится один раз" autocomplete="new-password"></label>

  <label class="f"><span>Действий за один запрос к модели</span>
    <input type="number" id="replanN" min="1" max="100" step="1"
           onchange="setReplanN(this.value)"></label>
  <p class="hint">1 — модель спрашивается на КАЖДОМ шаге (по умолчанию): максимум реакции на новый кадр, максимум запросов к API. Значение N&gt;1 просит у модели сразу до N действий за один запрос и во столько же раз сокращает число обращений, но свежие кадр, карту и список меток видит только первое действие пачки — остальные исполняются вслепую. Применяется при следующем запуске стека.</p>

  <div class="row">
    <button onclick="vlmTest()">Проверить связь</button>
    <button class="primary" onclick="vlmSave()">Сохранить</button>
    <button id="btnApplyRestart" onclick="applyRestart()">Применить и перезапустить</button>
  </div>
  <div class="bigerr" id="vlmMsg"></div>
  <p class="hint">«Проверить связь» делает ровно два запроса: список моделей и один минимальный ответ на 1 токен — он же и наполняет выпадающий список. Оркестратор читает <code>vlm.env</code> при СТАРТЕ процесса, поэтому новые значения применяются только после перезапуска стека.</p>
  <div class="row"><button onclick="go(2)">Назад</button><button class="primary" onclick="go(4)">Далее</button></div>
</section>

<section class="pane" id="pane4">
  <h2>Шаг 4. Мир симуляции</h2>
  <p class="hint">Мир задаёт и геометрию, и точку старта. Непригодные миры показаны серыми: их нельзя выбрать, причина написана прямо в карточке.</p>
  <div class="cards" id="worlds"></div>
  <!-- Мир читается при СТАРТЕ процесса Gazebo. Без этой плашки выбор мира на
       работающем стеке выглядит как «кнопка не нажимается»: конфиг меняется,
       карточка подсвечивается, а сцена остаётся прежней — и понять, почему,
       из интерфейса было нельзя. -->
  <div id="worldRestart" class="bigerr" style="display:none">
    <div>Стек уже запущен и продолжает крутить мир <b id="worldRunning">—</b>.
      Выбор новой карточки сохранён, но применится только после перезапуска.</div>
    <div class="row" style="margin-top:8px">
      <button class="primary" onclick="applyRestart()">Применить и перезапустить</button>
    </div>
  </div>
  <p class="hint">Если превью пустое, сгенерируйте картинки командой:</p>
  <pre class="cmd">ros2 run ar_project make_world_previews.py</pre>
  <div class="row"><button onclick="go(3)">Назад</button><button class="primary" onclick="go(5)">Далее: подключение</button></div>
</section>

<section class="pane" id="pane5">
  <h2>Шаг 5. Подключение</h2>

  <div id="simConnect">
    <div class="row">
      <button class="primary" onclick="stackStart()">Запустить стек</button>
      <button class="danger" onclick="stackStop()">Остановить</button>
      <button onclick="applyRestart()">Перезапустить</button>
    </div>
    <div class="chain" id="chain"></div>
    <div id="log"></div>
    <p class="hint">Журнал тянется отдельным таймером раз в секунду через <code>/api/stack/log?since=N</code> — он не едет в каждом кадре событий, чтобы не раздувать поток.</p>
  </div>

  <div id="hwConnect">
    <div class="linkbox" id="linkBox">
      <div class="big-title"><span class="pulse"></span><span id="linkTitle">Ожидаю подключения робота…</span></div>
      <div class="mline" id="linkMsg">—</div>
      <div style="margin-top:10px" id="linkTopics"></div>
    </div>
    <h3>Как подключить робота</h3>
    <ol class="steps-hw">
      <li>На <b>edge-боксе</b> (тот, где живут детектор и VLM-оркестратор) — установить транспорт и поднять роутер:
        <pre class="cmd">cd ~/ros2_ws/src/ar_project/deploy/transport
bash install_transport.sh edge &lt;IP_EDGE&gt;</pre></li>
      <li>На <b>Raspberry Pi</b> — тот же скрипт и шина CAN:
        <pre class="cmd">cd ~/ros2_ws/src/ar_project/deploy/transport
bash install_transport.sh pi &lt;IP_EDGE&gt;
sudo ip link set can0 up type can bitrate 1000000</pre></li>
      <li>На <b>Pi</b>, в терминале с преамбулой, которую напечатал install_transport.sh — нижний слой:
        <pre class="cmd">ros2 launch ar_project mission_bringup.launch.py mode:=hardware layer:=robot</pre></li>
      <li>На <b>edge</b>, в терминале с той же преамбулой — верхний слой:
        <pre class="cmd">ros2 launch ar_project mission_bringup.launch.py mode:=hardware layer:=edge \
  venv_python:=~/ot_venv/bin/python \
  vlm_env_file:=~/ros2_ws/src/object_tracking/planner_orchestrator/vlm.env</pre></li>
      <li>На <b>edge</b>, ПОСЛЕ подъёма верхнего слоя (раньше нельзя: параметры сжатия объявляются лениво, при первом кадре реле):
        <pre class="cmd">bash ~/ros2_ws/src/ar_project/deploy/tune_camera_compression.sh</pre></li>
      <li>На <b>этом ПК</b> — указать консоли адрес edge и открыть её:
        <pre class="cmd">./run.sh robot &lt;IP_EDGE&gt;      # Windows: .\run.ps1 robot &lt;IP_EDGE&gt;</pre></li>
    </ol>
    <div class="row"><button onclick="copyHw()">Копировать инструкцию</button></div>
    <p class="hint">Надпись «РОБОТ ПОДКЛЮЧЁН» появляется только тогда, когда ВСЕ ключевые потоки реально свежие, а не когда узлы просто видны в ROS-графе: видимость графа ничего не говорит о том, идут ли данные.</p>
  </div>

  <div class="row"><button onclick="go(4)">Назад</button><button class="primary" id="btnGo6" onclick="go(6)">Далее: работа</button></div>
</section>

<section class="pane" id="pane6">
  <h2>Шаг 6. Работа</h2>
  <div class="row">
    <input type="text" id="mission" placeholder="например: chair" style="flex:1 1 320px">
    <button class="primary" onclick="sendMission()">Отправить задание</button>
    <button class="danger" onclick="stopMission()">Стоп</button>
    <button onclick="seedMap()">Засеять карту (вращение на месте)</button>
  </div>
  <p class="hint">Для VLM-режима нужен ЧИСТЫЙ лейбл объекта, без глаголов («chair», а не «найди стул»). Для FLAT можно фразу — её нормализует PromptBridge. «Засеять карту» крутит робота на месте несколько секунд: в замкнутом помещении поиск не стартует, пока на карте нет ни одной границы известного и неизвестного.</p>
  <div class="bigerr" id="missionMsg"></div>
  <div class="mline" id="missionLine">Миссия: —</div>

  <!-- ФАКТИЧЕСКОЕ положение робота на плане мира. Сознательно НЕ карта SLAM:
       ту строит робот и её же видит VLM, поэтому по ней нельзя судить, где робот
       на самом деле — если SLAM уехал, уедет и картинка. Здесь геометрия берётся
       из worlds.yaml (известна заранее), а поза — из одометрии, так что план
       остаётся системой отсчёта, независимой и от SLAM, и от модели. -->
  <h3>Где робот на самом деле</h3>
  <div id="realMapWrap" style="position:relative;display:inline-block;max-width:100%">
    <img id="realMap" alt="план мира" style="max-width:100%;border:1px solid var(--line);
         border-radius:8px;background:#0b0f13;display:block">
    <svg id="realMapOv" style="position:absolute;left:0;top:0;width:100%;height:100%;
         pointer-events:none" viewBox="0 0 100 100" preserveAspectRatio="none"></svg>
  </div>
  <div class="mline" id="realMapLine">Позиция: —</div>

  <h3>Монитор миссии</h3>
  <!-- iframe смотрит на СВОЙ путь /dashboard/, а не на чужой хост: консоль
       проксирует дашборд, поэтому нет ни CORS, ни смешанного контента. Работает
       это только потому, что страница дашборда переведена на ОТНОСИТЕЛЬНЫЕ
       пути (mission_dashboard.py:355-361) — с корневыми "/events" и "/map.jpg"
       она ушла бы в корень консоли и осталась бы пустой. -->
  <div class="bigerr" id="frameMsg"></div>
  <iframe id="frame" title="Монитор миссии"></iframe>
</section>

</main>
</div>

<script>
const STEPS=[{n:1,t:"Режим"},{n:2,t:"Планировщик"},{n:3,t:"VLM"},
             {n:4,t:"Мир"},{n:5,t:"Подключение"},{n:6,t:"Работа"}];
const LEVEL_RU={ok:"OK",warn:"ВНИМАНИЕ",error:"ОТКАЗ",wait:"ОЖИДАНИЕ"};
const PHASES=[["gazebo_up","Gazebo"],["rsp_up","Модель робота"],["slam_up","SLAM"],
              ["nav2_up","Nav2"],["executive_up","Исполнитель"],["detector_up","Детектор"],
              ["orchestrator_up","Оркестратор"]];
const SOURCE_RU={fsm:"исполнительный автомат",vlm:"VLM-оркестратор",none:"—"};
/* Кавычка задана как ": константа лежит внутри python-строки, открытой
   тремя кавычками, и три кавычки подряд оборвали бы её на середине. */
const ESC={"&":"&amp;","<":"&lt;",">":"&gt;","\u0022":"&quot;","'":"&#39;"};
const S={cfg:{},worlds:[],worldsDrawn:null,ready:false,step:1,logNext:0,frameLoaded:false};

function el(id){return document.getElementById(id)}
function esc(s){return String(s==null?"":s).replace(/[&<>"']/g,function(c){return ESC[c]})}

let toastTimer=null;
function toast(text,kind){
 const t=el("toast");
 t.textContent=text;
 t.className="toast on"+(kind?" "+kind:"");
 if(toastTimer)clearTimeout(toastTimer);
 toastTimer=setTimeout(function(){t.className="toast"},6000);
}

/* Токен консоли не хранится на сервере в открытом виде и не попадает в HTML:
   страница держит его в localStorage и добавляет заголовком. Поток событий и
   /api/state специально освобождены от проверки — иначе EventSource, который
   не умеет ставить заголовки, вообще не смог бы подключиться. */
function tok(){try{return localStorage.getItem("console_token")||""}catch(e){return ""}}
function askToken(){
 const v=window.prompt("Консоль защищена токеном. Введите значение CONSOLE_TOKEN:","");
 if(v!=null){try{localStorage.setItem("console_token",v)}catch(e){}}
}

async function api(method,url,body){
 const h={};
 const t=tok();
 if(t)h["X-Console-Token"]=t;
 if(body!==undefined)h["Content-Type"]="application/json";
 let r;
 try{
  r=await fetch(url,{method:method,headers:h,
                     body:body===undefined?undefined:JSON.stringify(body)});
 }catch(e){
  return {status:0,data:{error:"Консоль не отвечает: "+e}};
 }
 let d={};
 try{d=await r.json()}catch(e){d={}}
 if(r.status===401){askToken()}
 return {status:r.status,data:d};
}

/* ---- навигация по мастеру ------------------------------------------------ */
function stepVisible(n){
 if(n===4)return (S.cfg.mode||"sim")==="sim";
 return true;
}
function stepLocked(n){return n===6&&!S.ready}
function go(n){
 // Скрытый шаг перепрыгивается В ТУ ЖЕ сторону, куда шёл оператор: иначе
 // «Назад» с шага 5 в режиме железа (где шаг «Мир» скрыт) возвращало бы на
 // тот же шаг 5 и кнопка выглядела бы сломанной.
 const back=n<S.step;
 while(n>=1&&n<=6&&!stepVisible(n))n+=back?-1:1;
 if(n>6)n=6;
 if(n<1)n=1;
 if(stepLocked(n)){
  toast("Шаг «Работа» открывается, когда преflight зелёный: сейчас есть блокирующие проверки.","err");
  return;
 }
 S.step=n;
 for(let i=1;i<=6;i++){
  const p=el("pane"+i);
  if(p)p.className="pane"+(i===n?" on":"");
 }
 // iframe дашборда создаётся лениво: он держит собственный SSE-поток, и
 // открывать его до готовности стека значит просто копить 502 в консоли.
 if(n===6&&!S.frameLoaded){S.frameLoaded=true;mountDashboard()}
 renderStepper();
}
function renderStepper(){
 el("stepper").innerHTML=STEPS.filter(function(s){return stepVisible(s.n)}).map(function(s){
  const cls=(s.n===S.step?" on":"")+(stepLocked(s.n)?" locked":"");
  return '<li class="'+cls.trim()+'" onclick="go('+s.n+')"><span class="num">'+s.n+
         '</span><span>'+esc(s.t)+'</span></li>';
 }).join("");
}

/* ---- конфигурация -------------------------------------------------------- */
async function patchConfig(patch){
 const r=await api("POST","/api/config",patch);
 if(r.status!==200){toast(r.data.error||("Ошибка "+r.status),"err");return false}
 applyConfig(r.data);
 return true;
}
function setMode(m){patchConfig({mode:m}).then(function(ok){if(ok)renderStepper()})}
function setPlanner(p){patchConfig({planner:p})}
function setWorld(w){patchConfig({world:w})}
/* Пустое поле НЕ шлём: иначе стёртое оператором значение ушло бы как 0 и было бы
   отвергнуто валидатором (INT_KEYS: replan_every_n >= 1) с красной ошибкой на
   ровном месте. Перерисовываем из S.cfg — поле возвращается к текущему значению. */
function setReplanN(v){
 const n=parseInt(v,10);
 if(!isFinite(n)||n<1){applyConfig(S.cfg);return}
 patchConfig({replan_every_n:n});
}

function applyConfig(cfg){
 if(!cfg)return;
 S.cfg=cfg;
 const mode=cfg.mode||"sim", planner=cfg.planner||"vlm";
 el("modeSim").className="pick"+(mode==="sim"?" sel":"");
 el("modeHw").className="pick"+(mode==="hardware"?" sel":"");
 el("plVlm").className="pick"+(planner==="vlm"?" sel":"");
 el("plFlat").className="pick"+(planner==="flat"?" sel":"");
 el("plMock").className="pick"+(planner==="mock"?" sel":"");
 el("simConnect").style.display=mode==="sim"?"":"none";
 el("hwConnect").style.display=mode==="sim"?"none":"";
 // «Применить и перезапустить» существует только там, где консоль владеет
 // процессом стека. На железе стек поднят чужими руками на Pi и edge — кнопка
 // врала бы оператору, поэтому её просто нет.
 el("btnApplyRestart").style.display=mode==="sim"?"":"none";
 // Не перетираем поле, пока оператор в нём набирает, — иначе периодический
 // applyConfig из опроса состояния сбрасывал бы недонабранное число.
 if(document.activeElement!==el("replanN")){
  el("replanN").value=cfg.replan_every_n||1;
 }
 el("sumMode").textContent=mode==="sim"?"симуляция":"реальный робот";
 el("sumPlanner").textContent=planner.toUpperCase();
 el("sumWorld").textContent=cfg.world||"—";
 // Подсветку выбранного мира ставит renderWorlds, а он до этого звался ровно
 // один раз — из loadWorlds при старте страницы. Клик по карточке менял конфиг
 // на сервере, но зелёная рамка навсегда оставалась на том мире, который был
 // выбран в момент загрузки: снаружи это выглядит как «миры не выбираются».
 // Перерисовываем при смене выбора и только при ней: applyConfig зовётся ещё и
 // периодическим опросом состояния, а безусловный renderWorlds на каждом тике
 // ронял бы уже загруженные превью и мигал бы картинками.
 if(S.worlds.length&&S.worldsDrawn!==(cfg.world||""))renderWorlds();
 const v=cfg.vlm||{};
 if(document.activeElement!==el("vlmBase"))el("vlmBase").value=v.base_url||"";
 setTokenBadge(!!v.token_set);
 if(v.model)ensureModelOption(v.model);
 renderStepper();
}
function setTokenBadge(on){
 const b=el("tokBadge");
 b.textContent=on?"токен задан":"не задан";
 b.className="badge "+(on?"ok":"err");
}
function ensureModelOption(m){
 const sel=el("vlmModel");
 let found=false;
 for(let i=0;i<sel.options.length;i++){if(sel.options[i].value===m)found=true}
 if(!found){const o=document.createElement("option");o.value=m;o.textContent=m;sel.appendChild(o)}
 sel.value=m;
}

/* ---- шаг 3: VLM ---------------------------------------------------------- */
function vlmMsg(text,good){
 const b=el("vlmMsg");
 b.textContent=text;
 b.className="bigerr on"+(good?" good":"");
}
async function vlmTest(){
 vlmMsg("Проверяю связь…",true);
 const r=await api("POST","/api/vlm/test");
 if(r.status!==200){vlmMsg(r.data.error||("Ошибка "+r.status),false);return}
 const d=r.data;
 const sel=el("vlmModel");
 const models=d.models||[];
 if(models.length){
  const cur=sel.value;
  sel.innerHTML=models.map(function(m){return '<option value="'+esc(m)+'">'+esc(m)+"</option>"}).join("");
  if(cur)ensureModelOption(cur);
 }
 if(d.ok){
  vlmMsg("Связь есть: "+(d.latency_ms||0)+" мс, моделей в списке "+models.length+
         (d.model_ok?", выбранная модель отвечает.":", но выбранная модель не проверена."),true);
 }else{
  vlmMsg(d.error_ru||"Связь не установлена.",false);
 }
}
async function vlmSave(){
 const model=el("vlmModelText").value.trim()||el("vlmModel").value||"";
 const body={base_url:el("vlmBase").value.trim(),model:model};
 const t=el("vlmToken").value;
 if(t)body.token=t;
 const r=await api("POST","/api/vlm/token",body);
 if(r.status!==200){vlmMsg(r.data.error||("Ошибка "+r.status),false);return}
 el("vlmToken").value="";                 // поле очищается сразу: обратно значение не придёт никогда
 setTokenBadge(!!r.data.token_set);
 const warn=(r.data.warnings||[]).join(" ");
 vlmMsg((r.data.note_ru||"Сохранено.")+(warn?" "+warn:""),true);
}
async function applyRestart(){
 if((S.cfg.mode||"sim")!=="sim"){
  toast("На железе консоль не владеет процессами: перезапустите orchestrator_node на edge-боксе вручную.","err");
  return;
 }
 toast("Останавливаю стек…");
 await api("POST","/api/stack/stop");
 const r=await api("POST","/api/stack/start");
 if(r.status!==200){toast(r.data.error||("Ошибка "+r.status),"err");return}
 S.logNext=0;el("log").textContent="";
 toast("Стек перезапущен: новые значения из vlm.env прочитаны при старте процесса.","ok");
}

/* Реальное положение робота поверх плана мира.

   Пересчёт мировых метров в проценты картинки, а не в пиксели: превью тянется по
   ширине панели, и абсолютные пиксели разъезжались бы при любом изменении окна.
   Поэтому у SVG viewBox 0..100 и preserveAspectRatio="none" — он всегда лежит
   ровно поверх картинки. Ось Y инвертируем: у мира +Y вверх, у экрана вниз, и
   без инверсии робот ездил бы зеркально (та же причина, что в
   make_world_previews.render_svg). */
function renderRealMap(pose){
 const img=el("realMap"), ov=el("realMapOv"), line=el("realMapLine");
 if(!img||!ov)return;
 const w=(S.worlds||[]).filter(function(x){return x.id===(S.cfg.world||"")})[0];
 if(!w){line.textContent="Позиция: мир не выбран";ov.innerHTML="";return}
 const src="/api/worlds/"+encodeURIComponent(w.id)+"/preview.svg";
 if(img.getAttribute("data-src")!==src){img.setAttribute("data-src",src);img.src=src}
 const size=w.size_m, org=w.origin_m;
 if(!pose||!Array.isArray(size)||!Array.isArray(org)){
  ov.innerHTML="";
  line.textContent=pose?"Позиция: у мира не задана геометрия в worlds.yaml"
                       :"Позиция: одометрия ещё не пришла";
  return;
 }
 /* Одометрия отсчитывается ОТ ТОЧКИ СТАРТА (кадр odom), а не от начала мира:
    в house робот спавнится в (-7, 0), а /odom в этот момент выдаёт (0, 0).
    Складываем позу спавна с одометрией, иначе робот всегда рисовался бы в
    середине плана. Дрейф одометрии при этом накапливается — для «где робот
    примерно» этого достаточно, метрической точности здесь и не обещаем. */
 const sp=w.default_spawn||{x:0,y:0,yaw:0};
 const syaw=+sp.yaw||0, c=Math.cos(syaw), s=Math.sin(syaw);
 const wx=(+sp.x||0)+c*pose.x-s*pose.y;
 const wy=(+sp.y||0)+s*pose.x+c*pose.y;
 const wyaw=syaw+(pose.yaw||0);
 const px=(wx-org[0])/size[0]*100;
 const py=100-(wy-org[1])/size[1]*100;
 if(!isFinite(px)||!isFinite(py)){ov.innerHTML="";return}
 const deg=-wyaw*180/Math.PI;   /* экранная Y вниз -> знак меняется */
 ov.innerHTML='<g transform="translate('+px.toFixed(2)+','+py.toFixed(2)+')">'+
  '<g transform="rotate('+deg.toFixed(1)+')">'+
   '<polygon points="4,0 -2.2,2.4 -2.2,-2.4" fill="#e5484d" stroke="#0c0f13" stroke-width="0.5"/>'+
  '</g><circle r="1.1" fill="#e5484d" stroke="#0c0f13" stroke-width="0.4"/></g>';
 const stale=(pose.age_s!=null&&pose.age_s>3);
 line.textContent="Позиция: x="+wx.toFixed(2)+" м, y="+wy.toFixed(2)+" м, курс "+
  Math.round(wyaw*180/Math.PI)+"° · источник "+(pose.topic||"?")+" (+точка старта)"+
  (stale?(" · МОЛЧИТ "+pose.age_s+" с"):"");
 line.style.color=stale?"var(--err)":"";
}

/* Мир, с которым РЕАЛЬНО стартовал текущий процесс стека. Берём из argv запуска,
   а не из конфига: конфиг оператор уже мог переключить, и именно расхождение этих
   двух значений надо показать. */
function runningWorldId(st){
 const a=(st&&st.argv)||[];
 for(let i=0;i<a.length;i++){
  const m=/^world:=(.*)$/.exec(a[i]);
  if(!m)continue;
  const p=m[1];
  if(!p)return "";
  return p.split("/").pop().replace(/\.(sdf|world)$/,"");
 }
 return "";
}
function renderWorldRestart(st){
 const box=el("worldRestart");
 if(!box)return;
 const run=runningWorldId(st);
 const show=!!(st&&st.running)&&run&&run!==(S.cfg.world||"");
 // Именно "block", а не "": у класса .bigerr в CSS стоит display:none, и пустая
 // строка вернула бы элемент к этому правилу — плашка молча не показывалась бы.
 box.style.display=show?"block":"none";
 if(show)el("worldRunning").textContent=run;
}

/* ---- шаг 4: миры --------------------------------------------------------- */
async function loadWorlds(){
 const r=await api("GET","/api/worlds");
 if(r.status!==200)return;
 S.worlds=r.data.worlds||[];
 renderWorlds();
}
function renderWorlds(){
 const cur=S.cfg.world||"";
 S.worldsDrawn=cur;
 el("worlds").innerHTML=S.worlds.map(function(w){
  const dead=(w.usable===false)||(w.status==="broken");
  const bcls=w.status==="ok"?"ok":(w.status==="broken"?"err":"warn");
  const rooms=w.rooms||[];
  const roomNames=Array.isArray(rooms)?rooms:Object.keys(rooms);
  const size=Array.isArray(w.size_m)?(w.size_m[0]+" × "+w.size_m[1]+" м"):(w.size_m||"размер неизвестен");
  return '<button class="pick wcard'+(dead?" dead":"")+(w.id===cur?" sel":"")+'"'+
   (dead?" disabled":' onclick="setWorld(\''+esc(w.id)+'\')"')+'>'+
   '<img loading="lazy" alt="" data-wid="'+esc(w.id)+'" src="/api/worlds/'+
   encodeURIComponent(w.id)+'/preview.svg">'+
   '<div class="t" style="margin-top:8px">'+esc(w.title||w.id)+
   ' <span class="badge '+bcls+'">'+esc(w.status||"?")+"</span></div>"+
   '<div class="meta">'+esc(size)+" · комнат: "+roomNames.length+"</div>"+
   (roomNames.length?'<div class="meta">'+esc(roomNames.join(", "))+"</div>":"")+
   (w.status_note?'<div class="meta" style="color:var(--err)">'+esc(w.status_note)+"</div>":"")+
   (w.notes?'<div class="meta">'+esc(w.notes)+"</div>":"")+
   "</button>";
 }).join("")||'<div class="hint">Каталог миров пуст: проверьте config/worlds.yaml.</div>';
 // Обработчики вешаются кодом, а не строкой в атрибуте: id мира приходит из
 // worlds.yaml, и кавычка в нём разорвала бы атрибут.
 Array.prototype.forEach.call(el("worlds").querySelectorAll("img[data-wid]"),function(im){
  im.onerror=function(){previewFallback(im,im.getAttribute("data-wid"))};
 });
}
/* Если консоль защищена CONSOLE_TOKEN, обычный <img> получает 401: тег не умеет
   ставить заголовки. Тогда картинка догружается fetch-ом (заголовок ставится) и
   подставляется как blob. Токен при этом НЕ уезжает в URL — в адресной строке
   и в журналах прокси секретам не место. */
function previewFallback(img,id){
 if(img.getAttribute("data-retry"))return;
 img.setAttribute("data-retry","1");
 const h={};
 const t=tok();
 if(t)h["X-Console-Token"]=t;
 fetch("/api/worlds/"+encodeURIComponent(id)+"/preview.svg",{headers:h})
  .then(function(r){return r.ok?r.blob():null})
  .then(function(b){if(b)img.src=URL.createObjectURL(b)})
  .catch(function(){});
}

/* ---- шаг 5: стек и связь ------------------------------------------------- */
async function stackStart(){
 const r=await api("POST","/api/stack/start");
 if(r.status!==200){toast(r.data.error||("Ошибка "+r.status),"err");return}
 S.logNext=0;el("log").textContent="";
 toast("Стек запускается. Следите за журналом и цепочкой фаз.","ok");
}
async function stackStop(){
 const r=await api("POST","/api/stack/stop");
 if(r.status!==200){toast(r.data.error||("Ошибка "+r.status),"err");return}
 toast("Остановка отправлена.","ok");
}
async function pollLog(){
 if((S.cfg.mode||"sim")!=="sim")return;
 if(S.step!==5)return;
 const r=await api("GET","/api/stack/log?since="+S.logNext);
 if(r.status!==200)return;
 const lines=r.data.lines||[];
 if(lines.length){
  const box=el("log");
  const stick=box.scrollTop+box.clientHeight>=box.scrollHeight-24;
  const html=lines.map(function(l){
   const bad=/ERROR|Traceback|died/.test(l.text||"");
   return '<div'+(bad?' class="e"':"")+">"+esc(l.text)+"</div>";
  }).join("");
  box.insertAdjacentHTML("beforeend",html);
  if(stick)box.scrollTop=box.scrollHeight;
 }
 if(r.data.next!=null)S.logNext=r.data.next;
}
function renderChain(stack){
 const phase=(stack||{}).phase||"";
 // Незнакомую фазу не подсвечиваем вовсе: покрасить всю цепочку зелёным из-за
 // строки, которой нет в таблице, значит соврать «стек поднялся».
 let known=false;
 for(let i=0;i<PHASES.length;i++){if(PHASES[i][0]===phase)known=true}
 let seen=false;
 el("chain").innerHTML=PHASES.map(function(p){
  const done=known&&!seen;
  if(p[0]===phase)seen=true;
  return '<span class="'+(done?"done":"")+'">'+esc(p[1])+"</span>";
 }).join("");
}
function renderLink(link){
 const phase=(link||{}).phase||"waiting";
 const box=el("linkBox");
 box.className="linkbox"+(phase==="connected"?" ok":"");
 el("linkTitle").textContent=phase==="connected"?"РОБОТ ПОДКЛЮЧЁН"
   :(phase==="partial"?"Связь частичная — жду остальные потоки…":"Ожидаю подключения робота…");
 el("linkMsg").textContent=(link||{}).message||"—";
 const topics=(link||{}).topics||{};
 const keys=Object.keys(topics).sort();
 el("linkTopics").innerHTML=keys.map(function(t){
  const a=topics[t];
  const cls=a==null?"bad":(a>5?"warn":"good");
  const txt=a==null?"нет данных":(a+" с назад");
  return '<div class="trow"><code>'+esc(t)+'</code><span class="'+cls+'">'+esc(txt)+"</span></div>";
 }).join("")||'<div class="hint">Список ключевых потоков пуст.</div>';
}
function copyHw(){
 const text=Array.prototype.map.call(document.querySelectorAll("#hwConnect pre.cmd"),
   function(p){return p.textContent}).join("\n\n");
 if(navigator.clipboard&&navigator.clipboard.writeText){
  navigator.clipboard.writeText(text).then(function(){toast("Инструкция скопирована.","ok")},
                                           function(){toast("Скопировать не удалось — выделите текст вручную.","err")});
 }else{
  toast("Буфер обмена недоступен (страница открыта не по https/localhost) — выделите текст вручную.","err");
 }
}

/* ---- шаг 6: миссия ------------------------------------------------------- */
function missionMsg(text,good){
 const b=el("missionMsg");
 b.textContent=text;
 b.className="bigerr on"+(good?" good":"");
}
async function sendMission(){
 const t=el("mission").value.trim();
 if(!t){missionMsg("Задание пустое: впишите, что искать.",false);return}
 const r=await api("POST","/api/mission/start",{text:t});
 if(r.status===200){
  missionMsg("Задание отправлено в "+(r.data.channel||"стек")+". "+(r.data.note_ru||""),true);
  return;
 }
 // 409 = оркестратор занят. Раньше повторная отправка исчезала бесследно
 // (orchestrator_node.py:310-315 отбрасывает сообщение при self._busy), поэтому
 // ответ показывается КРУПНО, а не всплывашкой в углу.
 missionMsg(r.data.error||("Задание не принято, код "+r.status),false);
 if(r.status===409)toast(r.data.error||"Миссия уже идёт.","err");
}
async function stopMission(){
 const r=await api("POST","/api/mission/stop");
 if(r.status!==200){missionMsg(r.data.error||("Ошибка "+r.status),false);return}
 missionMsg("Стоп отправлен. "+(r.data.note_ru||""),true);
}
async function seedMap(){
 const r=await api("POST","/api/seed_map");
 if(r.status!==200){toast(r.data.error||("Ошибка "+r.status),"err");return}
 toast(r.data.note_ru||"Робот крутится на месте, чтобы на карте появились границы известного.","ok");
}
/* Прокси отдаёт машинное {"error": ...} — правильно для API, но внутри iframe
   оператор увидел бы голый JSON. Поэтому доступность проверяется до подстановки
   src: пока дашборда нет, на его месте стоит русская плашка, и попытка
   повторяется, чтобы iframe появился сам, как только поднимется edge-слой. */
function mountDashboard(){
 fetch("/dashboard/").then(function(r){
  if(r.status===502){
   return r.json().catch(function(){return {}}).then(function(d){
    const b=el("frameMsg");
    b.className="bigerr on";
    b.textContent=d.error||"Монитор миссии пока недоступен.";
    el("frame").style.display="none";
    setTimeout(mountDashboard,5000);
   });
  }
  el("frameMsg").className="bigerr";
  el("frame").style.display="";
  if(!el("frame").src)el("frame").src="/dashboard/";
 }).catch(function(){setTimeout(mountDashboard,5000)});
}

function renderMission(m){
 if(!m){el("missionLine").textContent="Миссия: —";return}
 const parts=["источник: "+(SOURCE_RU[m.source]||m.source||"—"),
              "состояние: "+(m.state||"—")];
 if(m.instruction)parts.push("задание: "+m.instruction);
 if(m.step!=null)parts.push("шаг: "+m.step);
 if(m.last_action)parts.push("последнее действие: "+m.last_action);
 if(m.outcome)parts.push("итог: "+m.outcome);
 if(m.degraded)parts.push("ДЕГРАДАЦИЯ (откат на FLAT)");
 el("missionLine").textContent="Миссия — "+parts.join(" · ");
}

/* ---- преflight ----------------------------------------------------------- */
function renderChecks(p){
 const checks=p.checks||[];
 el("checks").innerHTML=checks.map(function(c){
  const lv=c.level||"wait";
  return '<div class="chk lv-'+esc(lv)+'"><span class="dot"></span><div>'+
   '<div class="ct">'+esc(c.title||c.id)+" <small>"+esc(LEVEL_RU[lv]||lv)+"</small></div>"+
   '<div class="cm">'+esc(c.message||"")+"</div>"+
   (c.hint?'<div class="ch">'+esc(c.hint)+"</div>":"")+
   (c.note_ru?'<div class="cn">'+esc(c.note_ru)+"</div>":"")+
   "</div></div>";
 }).join("")||'<div class="cm">Проверки ещё не приходили.</div>';
 const plate=el("readyPlate");
 if(p.ready){plate.className="plate ok";plate.textContent="ГОТОВ К ИСПЫТАНИЮ"}
 else{
  const n=(p.blocking||[]).length;
  plate.className="plate "+(n?"err":"wait");
  plate.textContent=n?("НЕ ГОТОВ: "+n+" блокирующих"):"ЖДУ ДАННЫХ";
 }
 S.ready=!!p.ready;
 el("btnGo6").disabled=!S.ready;
}

/* ---- общий рендер кадра -------------------------------------------------- */
function render(s){
 if(s.config)applyConfig(s.config);
 renderChecks(s.preflight||{});
 renderLink(s.link||{});
 renderChain(s.stack||{});
 renderMission(s.mission);
 const st=s.stack||{};
 el("sumStack").textContent=st.running?("работает, pid "+st.pid+", "+Math.round(st.uptime_s||0)+" с")
                                      :(st.exit_code!=null?("остановлен, код "+st.exit_code):"не запущен");
 renderWorldRestart(st);
 renderRealMap(s.robot_pose);
 // ROS-часть может отсутствовать: HTTP-сервер консоли поднимается ДО
 // rclpy.init() (иначе под zenoh создание узла блокируется, пока роутер
 // недоступен — transport_env.sh:13-14 — и консоль падала бы ровно в сценарии
 // «включил и жду робота»). Пока поток не отчитался, честнее показать «—», чем
 // «да». Форму ответа console_node меняет по месту (объект s.ros в кадре SSE,
 // плоский ros_connected в /api/state), поэтому читаем обе.
 const ros=s.ros||{};
 const conn=(ros.connected!=null)?ros.connected
           :((s.ros_connected!=null)?s.ros_connected:(s.preflight||{}).ros_connected);
 const why=ros.reason_ru||s.reason_ru||"узел ещё не создан";
 el("sumRos").textContent="ROS: "+(conn===false?("нет — "+why):(conn===true?"да":"—"));
 if(stepLocked(S.step))go(S.step-1);
}

/* Переподключение — та же схема, что в mission_dashboard.py:360-365: закрыть
   поток и попробовать снова через 2 с. Другого способа нет: EventSource сам
   восстанавливается не во всех браузерах, а консоль обязана пережить полный
   перезапуск стека без перезагрузки страницы. */
function connect(){
 const es=new EventSource("/api/events");
 es.onmessage=function(e){try{render(JSON.parse(e.data))}catch(_){}};
 es.onerror=function(){es.close();setTimeout(connect,2000)};
}

async function boot(){
 const c=await api("GET","/api/config");
 if(c.status===200)applyConfig(c.data);
 await loadWorlds();
 go(1);
 connect();
 setInterval(pollLog,1000);
}
boot();
</script>
</body></html>
"""


def render_page(*, console_version='1'):
    """Готовая к отправке страница в bytes.

    Существует, чтобы `http_api` не занимался кодировкой: обработчику нужен
    ровно один вызов и `Content-Length` по длине результата. Версия печатается
    в углу — при отладке «почему кнопка не появилась» первым делом надо
    отличить закешированную браузером страницу от свежей.
    """
    return PAGE.replace('__CONSOLE_VERSION__', str(console_version)).encode('utf-8')
