# Режимы работы: `flat` и `vlm`

## 1. Общая идея: один исполнительный субстрат, два «верха»

Система имеет **ровно один исполнительный субстрат** — реактивный контур на Raspberry Pi 5 (EKF -> облегчённый Nav2 (NavFn+DWB, local costmap во фрейме `odom`, контроллер ~8-10 Гц) -> `ros2_control` `diff_cont` -> `EmbodiedRobotSystem` (EPOS4/CiA-402 поверх SocketCAN), плюс `SAFETY`-слой) и набор **идемпотентных skill-серверов** (`ExploreFrontier` / `GoToPose` / `ApproachDetection` / `GetObservation` / `Stop`). Этим субстратом всегда управляет **Search Coordinator (исполнительный уровень, FSM/BehaviorTree на Pi)**.

Разница между режимами — **только в источнике подцелей (subtask)**:

- **`flat`** — единственный «потребитель решений» это сам исполнитель. Цель задаётся одним описанием объекта; исполнитель сам ведёт цикл `search -> detect -> goal -> drive`. **Нулевая зависимость от сети/VLM.**
- **`vlm`** — над исполнителем работает **Planner Orchestrator** (на edge/PC, лёгкий async HTTP-клиент к **внешнему OpenAI-совместимому API** Qwen3-VL-30B-A3B; сам эндпоинт — отдельный сервис/облако, vLLM/SGLang или провайдер, мы его не хостим). Он **не управляет роботом напрямую и не выдаёт координат навигации**. Он лишь **декомпозирует высокоуровневую инструкцию в последовательность/дерево подзадач**, каждая из которых — это **обычный FLAT-skill**. Текущую подзадачу диспетчеризует FLAT-исполнителю и **периодически перепланирует** по накопленной истории.

Ключевой инвариант (закреплён двумя предыдущими проходами проектирования + FMEA, не пересматривается): **VLM никогда не находится на реактивном пути**. FLAT-цикл и safety-слой работают в реальном времени независимо от планировщика; новый план принимается только в безопасных **COMMIT POINT** (горизонт согласования AESOP). Отсюда прямое следствие: **`vlm` — это надстройка над `flat`, а `flat` — постоянный деградационный fallback `vlm`.**

```
                 ┌─────────────────────────────────────────────┐
   EDGE/PC       │  Planner Orchestrator  (VLM, async, seconds) │   ← только режим vlm
   (GPU)         │  notes/summary buffer · semantic memory       │
                 └───────────────┬──────────────────────────────┘
                                 │ subtask = FLAT-skill (enum tool-call),
                                 │ принимается ТОЛЬКО в commit point
   ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─│─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─  Wi-Fi (rmw_zenoh)
                                 ▼
   ROBOT (Pi)    ┌─────────────────────────────────────────────┐
                 │  Search Coordinator (executive FSM/BT)        │   ← общий для обоих
                 │  ExploreFrontier·GoToPose·ApproachDetection·  │
                 │  GetObservation·Stop  (идемпотентные actions) │
                 │  target_pixel_to_goal · Nav2 · EKF · SAFETY   │
                 └─────────────────────────────────────────────┘
```

---

## 2. Режим `flat` (базовый, VLM-free)

### 2.1. Назначение и поведение

Вход: **одно текстовое описание цели** (`/target_prompt`, доставка через `reliable_prompt_sender` с ack по `/target_prompt_ack`). Задача: найти и подъехать к объекту, описанному в prompt. Точный сквозной конвейер:

1. **SEARCH** — локальное исследование фронтиров. Search Coordinator извлекает фронтиры **локально из costmap** (не запрашивает их у edge), выбирает фронтир с **гистерезисом** (см. 2.3) и едет туда через `ExploreFrontier`.
2. **DETECT** — open-vocab детектор на edge (YOLOE по умолчанию; fallback GroundingDINO+MobileSAM; Set-of-Mark рендеринг кандидатов; CLIPSeg из grounding исключён) сопоставляет prompt с кадром и выдаёт **пиксель цели** (`/target_pixel`, опц. `/target_mask`).
3. **target_pixel_to_goal -> 3D-цель** — узел `target_pixel_to_goal` (переиспользуется без изменений) переводит `pixel + локальная aligned-depth` в **метрическую 3D-цель** во фрейме `map`/`base_link` и публикует `/goal_pose`. **Планировщик/детектор никогда не порождают координаты навигации сами** — координату всегда вычисляет этот узел из геометрии.
4. **DRIVE** — Nav2 ведёт робота к `/goal_pose`; obstacle-source costmap — локальный `/scan` от `depthimage_to_laserscan` (raw depth/PointCloud2 **никогда не уходят по Wi-Fi**).
5. **ARRIVE / LOST** — при достижении подцели — успех; при потере детекции/устаревании потока — возврат к SEARCH.

### 2.2. FSM состояния и переходы

```
                ┌──────┐
                │ IDLE │  (нет prompt)
                └──┬───┘
        prompt set │
                   ▼
              ┌─────────┐  frontier exhausted / timeout      ┌──────────┐
        ┌────▶│ SEARCH  │───────────────────────────────────▶│ FAILED   │
        │     │(Explore │                                     │(no target│
        │     │Frontier)│                                     │ found)   │
   lost │     └────┬────┘                                     └──────────┘
        │ detection│ detection valid (свежий /target_pixel)
        │          ▼
        │     ┌──────────┐  goal computed (target_pixel_to_goal)
        │     │ DETECT/  │──────────────────────────────────┐
        └─────│ CONFIRM  │  pixel stale / lost ─────────────┘ (→ SEARCH)
              │(N стаб.  │
              │ детекций)│
              └────┬─────┘
        goal_pose │ valid 3D goal
                  ▼
            ┌───────────┐  Nav2 SUCCEEDED + поток детекций СВЕЖИЙ
            │ APPROACH  │─────────────────────────────────────▶┌──────────┐
            │(GoToPose/ │                                       │ ARRIVED  │
            │ Approach  │  Nav2 ABORTED / поток детекций STALE  │(SUCCESS) │
            │ Detection)│──────────────────────────────────────└──────────┘
            └────┬──────┘                       │
                 │ pixel stale во время подхода │
                 └──────────────────────────────┘ (→ SEARCH, НЕ авто-успех)
   ─ из любого состояния: Stop / preempt / fault → SAFE_STOP → IDLE
```

**FMEA-критичный переход (закреплён):** в `APPROACH` исполнитель **обязан** проверять **свежесть потока детекций** (возраст последнего `/target_pixel` против окна `max_target_pixel_age_s = 1.5 s`). **Запрещено** объявлять `reached`/`SUCCEEDED` по схеме «`goal_locked` + авто-успех» на **устаревшем пикселе**: если поток детекций stale, переход в `ARRIVED` блокируется, состояние возвращается в `SEARCH`. Финальная фиксация цели использует `final_approach_freeze_distance` и `required_stable_detections`, но **только при свежем потоке**.

### 2.3. Гистерезис выбора фронтира (FMEA must-fix)

Чтобы исключить осцилляцию между двумя почти равноценными фронтирами:

- **score margin**: переключение на новый фронтир `f_new` происходит, только если `score(f_new) > score(f_current) + margin` (порог по информативности/стоимости пути).
- **min dwell**: после коммита к фронтиру действует **минимальное время удержания** `min_dwell`, в течение которого пересмотр выбора запрещён (кроме случая, когда текущий фронтир стал недостижим/исчерпан).

Это локальная, чисто Pi-логика и работает одинаково в обоих режимах.

### 2.4. Словарь skill-ов (общий субстрат)

| Skill (action server) | Назначение |
|---|---|
| `ExploreFrontier` | исследовать выбранный локальный фронтир costmap |
| `GoToPose` | доехать до заданной позы (через Nav2) |
| `ApproachDetection` | подъехать к детектированной цели (pixel -> `target_pixel_to_goal` -> `/goal_pose`), с проверкой свежести потока |
| `GetObservation` | получить наблюдение в точке (для VLM-режима — снять кадр/контекст для заметок) |
| `Stop` | безопасный останов / preempt текущей подзадачи |

Все серверы **preemptable, feedback-carrying, UUID-идемпотентные**. Search Coordinator — **единственный потребитель решений** и **всегда держит закоммиченную подцель + действие-по-умолчанию (default productive action)**, поэтому отсутствие новых команд сверху никогда не приводит к простою.

### 2.5. Нулевая зависимость и роль fallback

`flat` работает **полностью на Pi** (детектор на edge — единственная сетевая зависимость DETECT; при её потере исполнитель продолжает SEARCH с default productive action). Это и **baseline**, который **измеряется и собирается ПЕРВЫМ** (gate: всё остальное строится только после подтверждённого Zero-VLM Pi baseline), и **постоянный деградационный режим** для `vlm`.

---

## 3. Режим `vlm` (Planner Orchestrator над FLAT)

### 3.1. Цикл оркестратора

Planner Orchestrator (edge) реализует **anytime/async** цикл «декомпозиция -> диспетч -> периодический перепланинг»:

1. **Decompose**: высокоуровневая инструкция -> **последовательность/дерево подзадач**, где каждая подзадача = **один FLAT-skill** с аргументами из реального списка: `explore_room(Y)` (→ `ExploreFrontier`), `find(X)` (→ DETECT-cycle), `approach(Z)` (→ `ApproachDetection`), `observe()` (→ `GetObservation`).
2. **Dispatch**: текущая подзадача отправляется FLAT-исполнителю как обычная skill-цель (UUID-идемпотентная). Исполнитель её отрабатывает реактивно, **не дожидаясь VLM ни на одном цикле управления**.
3. **Replan**: периодически и асинхронно из накопленной **истории** (notes/summary buffer) вычисляется следующий план; он **принимается только в commit point**.

**Структурированный вывод (закреплено):** VLM возвращает **enum / tool-call** результат — выбирает **только** `frontier_id` или `approach_target` из **реального списка**, переданного ему. Он **не порождает координат**. Клиент: **single-in-flight**, UUID-идемпотентность, **timeout из измеренного p99**, circuit-breaker, streaming.

### 3.2. (a) Summarization buffer / notes-память

**Почему notes, а не кадры.** Хранить и переотправлять много кадров по Wi-Fi запрещено (полоса/латентность) и дорого по токенам. Поэтому модель ведёт **компактные текстовые заметки самой себе** — это и есть «память миссии». Кадры используются один раз (в момент `GetObservation`/DETECT), извлечённый смысл записывается в notes, кадр отбрасывается.

**Схема заметок (что пишет модель):**

```json
{
  "mission_epoch": 7,
  "instruction": "<высокоуровневая инструкция оператора>",
  "observations":        ["в kitchen виден стол, объект X не найден", ...],
  "visited_rooms":       ["hall", "kitchen"],
  "ruled_out":           ["X нет в kitchen (осмотрено 2 фронтира)"],
  "candidate_locations": [{"place":"bedroom","prior":0.6,"reason":"обычно там"}],
  "open_subtasks":       ["explore_room(bedroom)", "find(X)"],
  "last_result":         {"subtask":"explore_room(kitchen)","status":"done"}
}
```

- **observations** — что увидено в точке (из `GetObservation`/DETECT), в виде краткого текста.
- **visited_rooms** — пройденные зоны (исключают повторное исследование).
- **ruled_out** — отвергнутые гипотезы (где цели точно нет) — критично против зацикливания.
- **candidate_locations** — кандидаты с приоритетом/обоснованием — задают порядок следующих подзадач.

**Обновление:** append-only после завершения каждой подзадачи (`last_result`) и после каждого `GetObservation`. **Компакция (bound tokens):** при превышении мягкого порога буфер **суммаризируется самой моделью** — старые `observations` сворачиваются в `ruled_out`/`visited_rooms`, дубли кандидатов сливаются. Целевой бюджет — **порядка ~1.5-2k токенов** на notes (плюс инструкция и список доступных enum-опций), что держит латентность перепланинга стабильной и вписывает её в измеренный p99. Семантическая память (карта мест/объектов) хранится отдельно и подаётся в промпт выборочно.

### 3.3. (b) Тайминг перепланинга: lead-time, single-in-flight, commit horizon

Перепланинг **медленный (секунды)**, поэтому он **никогда не должен останавливать FLAT**. Модель тайминга:

- `replan_interval` — номинальный период перепланинга (например, `≈ 8 s`).
- `T_lead` — **запас опережения**: перепланинг **стартует за `T_lead` до ожидаемого завершения текущей подзадачи** (`T_lead ≥ измеренный p99 латентности VLM`, например `≈ 3 s`). Пока FLAT доигрывает текущую подзадачу, следующий план **считается в фоне**.
- **single-in-flight + cancel-when-busy**: одновременно допускается **ровно один** запрос к VLM. Если триггер перепланинга срабатывает, пока запрос ещё «в полёте», новый **не запускается** (либо текущий отменяется и перезапускается с более свежими notes — cancel-when-busy), что защищает от лавины запросов.
- **commit horizon / commit point**: готовый план **кладётся в «pending»** и **принимается исполнителем только в безопасной точке коммита** (завершение текущей подзадачи или явная safe-точка BT, AESOP consensus-horizon). До commit point FLAT продолжает закоммиченную подцель.

**Диаграмма тайминга (overlap, без stall):**

```
время →
FLAT:    [ subtask A: explore_room(kitchen) ........][ subtask B: find(X) .........][ C ...
                                          │                         │
                                          │ T_lead до конца A       │ T_lead до конца B
                                          ▼                         ▼
VLM:                                 ╭───replan #1───╮         ╭───replan #2───╮
(async)                              │ notes→план B' │         │ notes→план C' │
                                     ╰──────┬────────╯         ╰──────┬────────╯
                                            │ готово → pending        │ готово → pending
                                            ▼                         ▼
commit:  ──────────────────────────────────●  COMMIT B' (на границе A→B)  ●  COMMIT C'
                                            ↑                         ↑
                              FLAT НЕ ждёт VLM: к моменту границы план уже готов
```

Если к commit point план **не готов** (VLM медленнее p99 / circuit-breaker открыт), исполнитель **продолжает default productive action** (например, дальнейшее `ExploreFrontier` по гистерезису) — **простоя нет**, план примется на следующем commit point. Это и есть явный **real-time аргумент**: реактивный контур и FLAT **никогда не блокируются** на перепланинге; VLM влияет на поведение исключительно через отложенный, согласованный коммит.

### 3.4. (c) Переключение режимов и деградация

- **`vlm -> flat` (graceful degrade)**: при потере VLM (timeout > p99, circuit-breaker open), потере **edge** или **Wi-Fi** оркестратор перестаёт быть источником подзадач. Исполнитель **продолжает текущую закоммиченную подзадачу как чистый FLAT** и далее работает в `flat` (SEARCH/DETECT/DRIVE) — это бесшовно, т.к. подзадача и так была FLAT-skill-ом. Восстановление связи -> повторное включение перепланинга со следующего commit point.
- **`flat -> vlm`**: при наличии edge/VLM и enable-флага оркестратор подхватывает миссию на ближайшем commit point.
- **Смена инструкции оператора в середине миссии (FMEA must-fix)**: это **не** мягкий перепланинг, а **ABORT-and-reset**. Исполнительный уровень: (1) **инкрементирует `mission_epoch`**, что **инвалидирует все in-flight UUID** (результаты skill-ов и ответов VLM со старым epoch отбрасываются); (2) **preempt/Stop** текущей подзадачи -> `SAFE_STOP`; (3) сбрасывает/архивирует notes старого epoch и стартует декомпозицию новой инструкции. Это исключает исполнение «устаревших» подцелей от предыдущей инструкции.

### 3.5. Диаграмма состояний/последовательности режима `vlm`

```
Оператор          Orchestrator (edge)            Executive (Pi, FLAT)         VLM (внешний API)
   │ instruction        │                              │                          │
   ├───────────────────▶│ decompose                    │                          │
   │                    ├──────── subtask A (UUID,epoch)▶│ run A (FLAT skill)       │
   │                    │                              │   …reactive, no block…   │
   │                    │  T_lead до конца A:           │                          │
   │                    ├──────── notes+enum-options ──────────────────────────────▶│ (single-in-flight)
   │                    │◀─────── tool-call: план B' ──────────────────────────────┤ (≤ p99)
   │                    │  держим в pending             │                          │
   │                    │            A завершилась ─────│                          │
   │                    │            COMMIT POINT       │                          │
   │                    ├──────── subtask B' (UUID,epoch)▶│ run B'                   │
   │                    │                              │                          │
   │ NEW instruction    │                              │                          │
   ├───────────────────▶│ mission_epoch++  (ABORT)     │                          │
   │                    ├──────── Stop / preempt ──────▶│ SAFE_STOP, drop old epoch│
   │                    │  reset notes; decompose new   │                          │
   │                    │                              │                          │
   │   ── Wi-Fi/edge/VLM loss ──▶ circuit-breaker open │                          │
   │                    │  (источник подзадач молчит)   │ продолжает как FLAT       │
   │                    │                              │ (degrade vlm→flat)        │
```

---

## 4. Связь с safety и временными окнами (для обоих режимов)

Оба режима опираются на один safety-слой: `cmd_vel` watchdog + Nav2 Collision Monitor + **реальный CiA-402 quick-stop** (controlword `0x6040` на RT-пути `write()`, **без** блокирующего 50 ms SDO; текущий код только логирует — это must-fix), **поканальный poll отказов каждый цикл** и обработка **CAN bus-off**. Тайминговые окна, которые синхронизация часов (chrony) обязана перекрывать с большим запасом: **TF `transform_tolerance = 0.2 s`** (для `map->odom`-коррекции от edge SLAM через `map_odom_relay`), **`depth_match_tolerance = 0.35 s`**, **`max_target_pixel_age_s = 1.5 s`**. Поскольку VLM не на реактивном пути, его латентность (секунды) **не входит** в эти окна — она поглощается механизмом `T_lead`/commit point из раздела 3.3.

---

Релевантные файлы (абсолютные пути), на которые опирается описание:
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/scripts/target_pixel_to_goal.py` — параметры `max_target_pixel_age_s=1.5`, `depth_match_tolerance=0.35`, `lock_goal_on_publish`, `required_stable_detections`, `final_approach_freeze_distance`, топики `/target_pixel`, `/target_prompt`, `/target_goal_locked`, `/goal_pose`.
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/scripts/reliable_prompt_sender.py` — доставка prompt c ack (`/target_prompt_request` -> `/target_prompt` + `/target_prompt_ack`).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/src/embodied_robot_system.cpp` — место для реального CiA-402 quick-stop.
- `C:/Users/dende/code/mobile_robot_navigation/object_tracking/object_tracking/{yoloe_image_segmentation.py, dino_mobilesam_image_segmentation.py, clip_image_segmentation.py}` — open-vocab детектор (YOLOE default; GroundingDINO+MobileSAM fallback; CLIPSeg исключён из grounding).
- Обе репы на ветке `robust`.