> **Поправки верификации (имеют приоритет над числами в теле ниже).** По итогам состязательной проверки против кода:
> - **Aligned depth — 640×480, не 424×240.** При `align_depth.enable=true` нативный depth-профиль 424×240 репроецируется в color-сетку 640×480 (и `target_pixel_to_goal` берёт color-интринсики). Сырой кадр = 640·480·2 = 614 400 B.
> - **RVL даёт ~3:1 без потерь** (не 10–20×). Плотный 640×480 16UC1 RVL ≈ **50–70 КБ**; 10×+ — только на разрежённой глубине. Значит: depth-keyframe ≈ 50–70 КБ, полный `DetectTarget`-request (L4) ≈ **75–115 КБ**, пик/DETECT ≈ 75–115 КБ, совмещённый пик ≈ 80–130 КБ. Если полоса критична — слать нативный 424×240 с ре-выравниванием на EDGE.
> - **`/scan` через `depthimage_to_laserscan` — НОВЫЙ компонент.** Сейчас его в репозитории нет; `local_costmap` obstacle-source — PointCloud2 `/camera/camera/depth/color/points_rgbd` (локально). В `robust` он заменяется на локальный `/scan`.
> - **L6 `map→odom` — тип `ar_project_msgs/MapOdomCorrection`** (TransformStamped + covariance + seq + relocalized), не `PoseWithCovarianceStamped`.
> - **Самое узкое окно для chrony — EKF `transform_timeout` 0.1 c** (уже, чем TF 0.2 c / depth-match 0.35 c / pixel-age 1.5 c).
> - **Цель считается во фрейме `map`** → зависит от здоровья `map_odom_relay`: при длительной потере линка некорректируемый дрейф одометрии делает 3D-цель всё менее точной → ограничивать радиус/время исследования и уходить в SAFE_STOP по бюджету дрейфа.
> - JPEG q80 640×480 ≈ 25–45 КБ — это ~20–37× (не 15–25×).

# Контракты передачи данных Pi <-> ПК

Раздел описывает **все** каналы передачи данных целевой архитектуры, проходящие через Wi-Fi между ROBOT (Raspberry Pi 5, без GPU) и EDGE/PC (GPU-бокс). Транспорт — `rmw_zenoh` (один systemd-роутер `zenohd` на EDGE; fallback — Fast DDS LARGE_DATA + Discovery Server), multicast отключён, размер сокет-буферов 12 МБ, синхронизация часов через chrony на всех хостах. Базовый принцип, на котором построены все контракты: **реактивный контур (EKF, Nav2, /scan, control, safety) НИКОГДА не ждёт ответа по Wi-Fi**; через линк ходят только редкие, малые по объёму или событийные сообщения, а тяжёлые потоки (PointCloud2, сырые RGB/depth) через Wi-Fi не передаются принципиально.

Все идентификаторы (topic/action/service, имена узлов, параметры) даны на английском. Числа по размеру/полосе/задержке приведены для целевых профилей RealSense D435i: RGB `640x480x15`, depth `424x240x15` aligned-to-color (`16UC1`, миллиметры), как зафиксировано в `realsense_rgbd_pi.launch.py`.

---

## 1. Сводная таблица по каждому каналу (per-link)

Сокращения QoS: R=Reliable, BE=Best-Effort, TL=Transient-Local, V=Volatile, KL=Keep-Last(N), KA=Keep-All. Задержки — оценка end-to-end по Wi-Fi (включая сериализацию + RTT, исключая инференс/VLM, который указан отдельно). Часы синхронизированы chrony, оффсет << 0.2 c.

| # | Канал | Направление | Тип интерфейса | Тип сообщения / ключевые поля | Кодирование (ради скорости) | Размер | Частота | QoS | Полоса | Задержка линка | Поведение при деградации |
|---|-------|-------------|----------------|-------------------------------|------------------------------|--------|---------|-----|--------|----------------|--------------------------|
| **L1** | `SeekObject` | operator -> Pi | **action** (`/seek_object`) | goal: `string instruction`, `string mode{flat\|vlm}`, `string target_desc`, `uint32 mission_epoch`; feedback: `string phase`, `float32 progress`, `string committed_subgoal`; result: `uint8 outcome`, `string note` | CDR (текст), без сжатия | goal ~200–600 B, feedback ~150 B, result ~200 B | событийно (goal редко); feedback 1–2 Гц | R, V, KL(10), deadline 2 s (feedback), liveliness AUTOMATIC lease 5 s | < 1 КБ/с | 10–40 мс | потеря линка не отменяет миссию: executive на Pi автономен; feedback-таймаут — оператор видит «link lost», робот продолжает committed subgoal |
| **L2** | `plan-request` | Pi executive -> EDGE Planner Orchestrator | **action** (`/plan_request`, preemptable) | goal: `string instruction`, `uint32 mission_epoch`, `string history_digest`, `FrontierCandidate[] frontiers` (id, centroid x/y, size, info_gain), `DetectionSummary[] seen`, `string[] notes_ref`; feedback: `string status{queued\|inflight\|streaming}`; result: см. L3 | CDR; frontiers — компактный список (НЕ costmap), notes по reference-id, не текст | goal ~2–8 КБ (десятки frontiers + дайджест) | событийно, на commit-point; ~раз в 5–30 c | R, V, KL(1), deadline = p99_VLM (измеренный), liveliness lease 10 s | idle ~0; пик 2–8 КБ/цикл | линк 10–40 мс + **VLM secs** (вне реактивного пути) | single-in-flight + UUID idempotency; timeout по измеренному p99 + circuit-breaker; при отказе — executive остаётся в FLAT |
| **L3** | `plan-decision` | EDGE -> Pi | **action result/feedback** того же `/plan_request` (streaming) | `PlanDecision`: `uint32 mission_epoch`, `uuid plan_id`, `SubtaskNode[] tree` (skill enum, `frontier_id` ИЛИ `approach_target` — выбор ИЗ присланного списка, params), `string note_to_self`, `float32 confidence`, `builtin_interfaces/Time stamp` | CDR; **enum/structured tool-call**: VLM возвращает только индексы из реального списка, не координаты | ~0.5–4 КБ | событийно (ответ на L2) | R, V, KL(1) | пик 0.5–4 КБ/цикл | линк 10–40 мс (после VLM) | устаревший `mission_epoch` -> отбрасывается; адопция только в commit-point; при таймауте — продолжается текущий subtask, затем degrade в FLAT |
| **L4** | `DetectTarget request` | Pi -> EDGE | **action** (`/detect_target`) ИЛИ **service** (см. §2.2) | goal: `sensor_msgs/CompressedImage rgb`, `sensor_msgs/CompressedImage depth`, `sensor_msgs/CameraInfo info` (опц., один раз), `string prompt`, `builtin_interfaces/Time stamp`, `string frame_id`, `uuid req_id` | **RGB: JPEG q80** (640x480); **depth: RVL или PNG для 16UC1** (424x240, mm); intrinsics — **один раз** на сессию; CDR-обёртка | RGB ~25–45 КБ; depth ~8–25 КБ; info ~0.5 КБ (однократно) | **событийно**, 1 keyframe на цикл детекции (НЕ поток); ~0.2–2 Гц при поиске | R, V, KL(1), deadline 0.5 s | idle 0; пик ~35–70 КБ/детекцию | линк сериализация+RTT 30–90 мс + инференс на GPU | при stale/потере — детектор не выдаёт пиксель; executive продолжает explore (FLAT) |
| **L5** | `DetectTarget result` | EDGE -> Pi | **action result** `/detect_target` | `DetectionResult`: `float32 u`, `float32 v`, `float32 depth_m`, `float32 score`, `string class_label`, `string frame_id`, `builtin_interfaces/Time stamp`, `uuid req_id`, `bool found` | CDR, чистые числа | ~120–250 B | событийно (ответ на L4) | R, V, KL(1), deadline 0.5 s | < 1 КБ/с | линк 10–30 мс | `req_id` mismatch / stale stamp -> отбрасывается; **approach НЕ объявляет reached по устаревшему пикселю** (см. §2.4) |
| **L6** | `map->odom correction` | EDGE SLAM (RTAB-Map) -> Pi `map_odom_relay` | **topic** (`/map_odom_correction`) | `geometry_msgs/PoseWithCovarianceStamped` (поправка map->odom), `header.stamp`, `header.frame_id=map`, `child=odom` (в поле или конвенция) | CDR; **НЕ TF-поток**, низкочастотная поправка | ~350–500 B | 1–2 Гц (low-rate) | R, TL, KL(1), deadline 2 s, liveliness lease 5 s | ~0.5–1 КБ/с | 10–30 мс | `map_odom_relay` держит last-good, гейтит скачок/ковариацию, **отбрасывает stale stamps**, ребродкастит map->odom локально < `transform_tolerance` 0.2 s |
| **H1** | heartbeat Pi-alive | Pi -> EDGE | **topic** (`/hb/pi`) | `std_msgs/Header` (stamp) или `diagnostic_msgs/DiagnosticStatus` lite: `uint8 level`, `string node`, seq | CDR минимальный | ~60–120 B | 2 Гц | BE, V, KL(1), deadline 1 s, liveliness AUTOMATIC lease 2 s | < 0.5 КБ/с | < 10 мс | пропуск deadline -> EDGE метит Pi offline (логирование/диагностика), миссия не страдает |
| **H2** | heartbeat EDGE-alive | EDGE -> Pi | **topic** (`/hb/edge`) | как H1 | CDR минимальный | ~60–120 B | 2 Гц | BE, V, KL(1), deadline 1 s, liveliness lease 2 s | < 0.5 КБ/с | < 10 мс | пропуск -> Pi помечает edge недоступным -> VLM mode **degrade в FLAT**, SLAM-поправки замораживаются (last-good в relay) |
| **H3** | heartbeat VLM/Planner-ready | EDGE Planner Orchestrator -> Pi | **topic** (`/hb/planner`) | lite-status: `uint8 state{ready\|busy\|circuit_open}`, `float32 p99_ms`, seq | CDR минимальный | ~80–150 B | 1 Гц | BE, V, KL(1), deadline 2 s, liveliness lease 4 s | < 0.5 КБ/с | < 10 мс | `circuit_open`/пропуск -> executive не шлёт новые `plan-request`, остаётся в FLAT до восстановления |
| **L7** | semantic-memory / notes sync | (преим. LOCAL на EDGE) EDGE -> Pi только digest | **topic** (`/mission/notes_digest`) опц. | `string digest` (компактное summary, NOTES self-written), `uint32 mission_epoch`, seq | CDR текст; **полные frames/embeddings остаются на EDGE** | ~0.5–4 КБ | редко, на replan (≤ 0.2 Гц) | R, TL, KL(1) | < 1 КБ/с (всплески) | 10–40 мс | если не передаётся — executive использует только локальное mission-state; деградация прозрачна |
| **L8** | `/diagnostics` (aggregated) | Pi <-> EDGE (двунаправленно, подписка) | **topic** (`/diagnostics_agg`) | `diagnostic_msgs/DiagnosticArray`: status[] (level, name, message, key/value) | CDR; **агрегированный** (diagnostic_aggregator), НЕ сырой высокочастотный `/diagnostics` | ~1–4 КБ | 1 Гц | R, V, KL(5), deadline 5 s | ~2–4 КБ/с | 20–60 мс | потеря -> только мониторинг страдает; реактив не зависит |

> Примечание по L4: предпочтителен **action** (а не service), чтобы запрос был preemptable, нёс feedback и поддерживал UUID-идемпотентность при смене `mission_epoch`. Service допустим только в синхронном тестовом стенде.

---

## 2. Решения по keyframe / формату

### 2.1 RGB keyframe — JPEG q80, 640x480

- **Формат:** `sensor_msgs/CompressedImage`, `format="jpeg"`, качество **q80**. Источник — color-профиль `640x480x15`.
- **Почему:** JPEG q80 для типичной сцены 640x480 даёт **~25–45 КБ** (коэффициент сжатия ~15–25x против сырых 640·480·3 = 921 600 B). Визуального качества q80 более чем достаточно для open-vocab детектора (YOLOE / GroundingDINO). Поднимать до q90+ невыгодно: +50–80% байт ради нерелевантной для детекции точности.
- **Кодирование на Pi:** аппаратного JPEG-энкодера в пайплайне не предполагаем -> libjpeg-turbo (SIMD на ARM). Сложность **O(W·H)**; на Pi 5 ~**3–6 мс** на кадр 640x480. Это одноразовая операция на keyframe (НЕ на поток), поэтому нагрузка незаметна.
- **Decode на GPU-боксе:** O(W·H), ~1–2 мс (CPU) или nvJPEG на GPU < 1 мс.
- **Downscale:** до отправки кадр остаётся 640x480 (детектору нужен достаточный охват мелких целей); дальнейший даунскейл не делаем, т.к. q80 уже укладывается в бюджет полосы.

### 2.2 Depth keyframe — RVL (предпочтительно) или PNG для 16UC1, 424x240

- **Формат:** `sensor_msgs/CompressedImage` депт-варианта. Источник — aligned-to-color `424x240`, `16UC1` (миллиметры).
- **RVL (Run-length + Variable-length, алгоритм Wilson, как в `compressed_depth_image_transport`):** специально для 16UC1; **O(W·H)** энкод/декод, ~**0.5–1.5 мс** на Pi 5 для 424x240, размер **~8–18 КБ** в зависимости от сцены. Это предпочтительный путь: быстрее PNG и без потерь.
- **PNG (zlib) как fallback:** O(W·H) с большей константой (zlib deflate), на Pi 5 ~**3–6 мс**, размер **~12–25 КБ**. Используется, если RVL недоступен в транспортной цепочке.
- **Почему не сырой depth и не PointCloud2:** сырой 424x240·2 B = 203 520 B на кадр; PointCloud2 — сотни КБ–МБ. **Через Wi-Fi не передаём принципиально.** RVL/PNG даёт сжатие ~10–20x без потерь.
- **Согласование RGB/depth:** keyframe RGB и depth берутся из одного синхронизированного кадра (`enable_sync=true`), оба несут одинаковый `stamp`. На EDGE детектор сопоставляет их по stamp; локальный аналог допуска — `depth_match_tolerance=0.35 c` (как в `target_pixel_to_goal.py`).

### 2.3 Intrinsics — один раз за сессию

- `CameraInfo` (fx=`k[0]`, fy=`k[4]`, cx=`k[2]`, cy=`k[5]`) передаётся **однократно** при старте сессии детекции (или при смене профиля), а не с каждым keyframe. Это экономит ~0.5 КБ на каждую детекцию и совпадает с локальной логикой (`camera_info_callback` фиксирует интринсики один раз).
- На Pi восстановление 3D из пикселя делает **локальный** `target_pixel_to_goal` по той же модели pinhole: `x=(u−cx)·d/fx`, `y=(v−cy)·d/fy`, `z=d`. Планировщик и детектор **никогда не возвращают навигационные координаты** — только пиксель u,v (+ опц. depth).

### 2.4 Почему CDR против compressed, и event-driven против stream

- Управляющие/служебные сообщения (L1–L3, L5, L6, H1–H3, L7, L8) — **CDR без сжатия**: они малы (сотни байт–единицы КБ), сжатие добавило бы CPU-латентность без выигрыша.
- Только тяжёлые keyframes (L4) сжимаются (JPEG/RVL).
- **Всё, что идёт через линк, — событийное или низкочастотное.** Нет ни одного непрерывного высокочастотного потока через Wi-Fi. Это и есть гарантия, что реактивный контур не упирается в линк.
- **Anti-stale на приёме (FMEA must-fix):** L5 несёт `stamp` + `req_id`; `approach`-skill сверяет возраст пикселя (порог по аналогии с `max_target_pixel_age_s=1.5 c`) и **детектирует staleness потока детектора**. Запрещено авто-объявление `reached` по устаревшему пикселю (никаких `goal_locked`+SUCCEEDED как авто-успех). L6 в `map_odom_relay` отбрасывает кадры с устаревшим `stamp` и гейтит по ковариации/скачку.

---

## 3. Сводный бюджет полосы и задержек

### 3.1 Полоса

| Состояние | Каналы | Оценка полосы |
|-----------|--------|----------------|
| **Idle (без детекций/планов)** | H1+H2+H3 (~6·(60..150) B/с) + L6 correction (1–2 Гц·~450 B) + L8 diag (1 Гц·~2 КБ) + опц. L1 feedback | **~3–6 КБ/с (≈ 30–50 кбит/с)** |
| **Пик на одну DETECT-итерацию** | L4 (RGB ~35 КБ + depth ~15 КБ) + L5 (~0.2 КБ) поверх idle | **~50–70 КБ всплеском** (за ~30–90 мс на линке) |
| **Пик на один PLAN-цикл** | L2 (~2–8 КБ) + L3 (~0.5–4 КБ) + опц. L7 (~0.5–4 КБ) | **~3–16 КБ всплеском** |
| **Совмещённый пик** (детекция + replan одновременно) | L4+L5+L2+L3 | **~55–85 КБ всплеском**, мгновенная полоса << Wi-Fi capacity (десятки Мбит/с) |

Вывод: средняя нагрузка на линк в idle — десятки кбит/с; даже пиковые всплески (десятки КБ) на порядки ниже пропускной способности Wi-Fi и не конкурируют с реактивным трафиком (которого на линке нет — он локален).

### 3.2 Цепочка задержек, доказывающая, что реактивный контур не ждёт линк

Реактивный контур целиком **локален на Pi** и работает на своих частотах независимо от линка:

```
RealSense -> depthimage_to_laserscan -> /scan (local) ──┐
EKF odom->base_link @20Hz ──────────────────────────────┤
map_odom_relay: ребродкаст map->odom @ локально < 0.2s ──┼─> Nav2 local costmap (odom frame)
                                                          │   -> DWB controller @8-10Hz
                                                          │   -> cmd_vel -> safety (watchdog +
                                                          │      Collision Monitor + CiA-402 quick-stop)
                                                          └─> EPOS4 RT write()
```

- **Локальный цикл управления:** controller 8–10 Гц (период 100–125 мс), EKF 20 Гц (50 мс), все TF — на Pi. Ни один шаг не пересекает Wi-Fi.
- **L6 (SLAM correction)** приходит 1–2 Гц, но Nav2 НЕ ждёт его: `map_odom_relay` держит last-good и ребродкастит map->odom локально внутри `transform_tolerance=0.2 c`. Если поправка опоздала/потерялась — relay продолжает ребродкаст последней валидной (контур не блокируется).
- **L4/L5 (детекция):** линк 30–90 мс + GPU-инференс. Это **событийный** запрос; пока он считается, executive держит committed subgoal и продолжает продуктивное действие (explore). Пиксель приходит асинхронно и лишь обновляет цель — Nav2 продолжает ехать к текущей.
- **L2/L3 (VLM план):** секунды. **Вне реактивного пути по построению.** Зарезервирован lead-time/интервал: FLAT исполняет текущий subtask, пока считается следующий replan (anytime/async, адопция в commit-point — AESOP consensus-horizon).
- **chrony-бюджет (FMEA must-fix):** оффсет часов держим **существенно меньше** самых узких окон: TF `transform_tolerance` 0.2 c, depth-match 0.35 c, pixel-age 1.5 c. Целевой оффсет chrony единицы мс (<< 0.2 c), чтобы сверки stamp на приёме были корректны.

Итог латентной цепочки: **самый медленный участок линка (VLM, секунды) никогда не находится на пути от датчика к cmd_vel.** Реактивный контур имеет верхнюю границу задержки ~один период контроллера (≤125 мс) + EKF (50 мс), полностью локально.

---

## 4. Данные, остающиеся ЛОКАЛЬНО на Pi (НЕ пересекают Wi-Fi)

Эти потоки порождаются и потребляются только на Pi; передача их через Wi-Fi **запрещена** (объём и/или частота недопустимы, либо они критичны для реактивного контура):

| Данные | Топик/интерфейс | Частота | Почему локально |
|--------|------------------|---------|------------------|
| **/scan** | `/scan` (`sensor_msgs/LaserScan`) | ~15 Гц (из `depthimage_to_laserscan`) | obstacle source для local costmap; генерируется локально из depth — **не передаём depth-поток на EDGE ради /scan** |
| **Полное дерево TF** | `/tf`, `/tf_static` | 20–50 Гц | высокочастотный поток; реактивно критичен; только итоговая map->odom-поправка идёт через L6, и то как редкое PoseWithCovariance, НЕ как TF |
| **cmd_vel** | `/cmd_vel`, `/diff_cont/cmd_vel_unstamped` | 8–10+ Гц | команда привода; safety-критична; должна быть детерминированно локальной |
| **EKF** | `/diff_cont/odom`, `/odometry/filtered`, `odom->base_link` | 20 Гц | оценка позы реального времени; основа всего контура |
| **control** | ros2_control `diff_cont`, EmbodiedRobotSystem RT write(), EPOS4 CiA-402 SocketCAN | RT-цикл | жёсткий real-time; включая **реальный quick-stop (controlword 0x6040) на RT-пути без блокирующего 50 мс SDO**, per-cycle fault poll, обработку CAN bus-off |
| **RealSense raw streams** | `/camera/camera/color/image_raw`, `/.../aligned_depth_to_color/image_raw`, `/camera/camera/imu` | 15 Гц color/depth, IMU выше | сырые кадры/IMU; **только сжатый keyframe** уходит в L4 по событию; сырые потоки и **PointCloud2 — никогда** |

Дополнительно локально: local costmap, извлечение frontiers (executive делает это **локально из costmap**, наружу уходит лишь компактный список кандидатов в L2), все skill-action-серверы (ExploreFrontier / GoToPose / ApproachDetection / GetObservation / Stop), `target_pixel_to_goal` (пиксель + локальный aligned-depth -> 3D goal), twist_mux, Collision Monitor, cmd_vel watchdog.

---

## 5. Ключевые числа (резюме для проверки бюджета)

- RGB 640x480 JPEG q80 ≈ **25–45 КБ** (encode на Pi 5 ~3–6 мс, O(W·H)).
- Depth 424x240 16UC1 **RVL** ≈ **8–18 КБ** (encode ~0.5–1.5 мс) / **PNG** ≈ 12–25 КБ (~3–6 мс).
- DetectTarget request (L4) суммарно ≈ **35–70 КБ** на keyframe; result (L5) ≈ **120–250 B**.
- map->odom correction (L6) ≈ **350–500 B** @ 1–2 Гц.
- Heartbeats H1/H2/H3 ≈ **60–150 B** каждый @ 1–2 Гц.
- plan-request (L2) ≈ **2–8 КБ**; plan-decision (L3) ≈ **0.5–4 КБ**; notes digest (L7) ≈ **0.5–4 КБ**.
- **Idle cross-link:** ~3–6 КБ/с (≈30–50 кбит/с). **Пик/DETECT:** ~50–70 КБ. **Пик/PLAN:** ~3–16 КБ.
- Латентный бюджет реактива (локально): ≤ ~175 мс (1 период контроллера + EKF), **независимо от линка**; VLM секунды — всегда вне реактивного пути.

---

Релевантные файлы (абсолютные пути), на которых основаны контракты:
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\scripts\target_pixel_to_goal.py` — pixel->3D (pinhole), depth 16UC1 mm, `depth_match_tolerance=0.35`, `max_target_pixel_age_s=1.5`, TF timeout 0.2 c, интринсики из `k[0/4/2/5]`.
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\launch\realsense_rgbd_pi.launch.py` — профили RGB `640x480x15`, depth `424x240x15` aligned, `pointcloud.enable=false`.
- `C:\Users\dende\code\mobile_robot_navigation\ar_project\config\ekf_gyro.yaml` — EKF 20 Гц, `transform_timeout=0.1`, odom->base_link.
- `C:\Users\dende\code\mobile_robot_navigation\object_tracking\object_tracking\tracker_node.py` — backends (CLIP/GroundingDINO+MobileSAM/YOLOE), та же модель pixel+depth->goal на стороне perception.

Кастомных `.action/.srv/.msg` в репозиториях пока нет — типы `SeekObject`, `PlanRequest`/`PlanDecision`, `DetectTarget`, `DetectionResult`, `FrontierCandidate` нужно создать в новой ветке `robust` (предлагаемые поля приведены в таблице §1).