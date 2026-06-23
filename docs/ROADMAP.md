# ROADMAP — робастная архитектура (ветки `robust`)

## 4. ROADMAP (пошаговый чек-лист)

Порядок строго FIX-FIRST: сначала безопасность и корректность (Phase 0), затем транспорт и симуляция, затем ZERO-VLM FLAT-базлайн, и только после него — восприятие как сервис, VLM-режим, hardening и железо. Каждая фаза имеет критерий выхода (EXIT). FMEA-обязательные фиксы помечены `[FMEA]`. Прогресс отмечается галочками в `- [ ]`.

### Phase 0 — Безопасность и корректность (FIX-FIRST)
- [x] 0.1 Параметризовать `use_sim_time` во всех yaml/launch (`nav2_params.yaml`, `localization_launch.py`, `navigation_launch.py`, EKF и пр.); по умолчанию `False` на железе, `True` только в Gazebo-launch. — `nav2_params.yaml` приведён к `False` (переписывается `RewrittenYaml` в `navigation_launch.py` под sim); launch уже параметризованы (default `false`); `my_controllers.yaml`=`true` относится только к Gazebo (gz_ros2_control), `ros2_controllers.yaml` (железо) `use_sim_time` не задаёт → `False`.
- [x] 0.2 `[FMEA]` Реализовать реальный CiA-402 quick-stop: запись controlword `0x6040` (бит quick-stop, переход в Quick Stop Active) на RT-пути `write()` в `EmbodiedRobotSystem`, БЕЗ блокирующего 50 мс SDO; quick-stop делать через PDO/неблокирующий путь. — `write()` при латче `quick_stop_active_` шлёт controlword `0x0002` через `tpdo_transmit` (RPDO1, подтверждено по `bus.yml`), без SDO; добавлены `request_quick_stop()`/`transmit_quick_stop()`. `transmit_quick_stop()` дополнительно со-командует целевую скорость `0x60FF`=0 — fallback-остановка на случай, если циклический `handleWrite()` драйвера перезапишет controlword обратно на `0x000F` (контеншен). **HIL-проверка (железо / vcan+fake-EPOS4): подтвердить, что quick-stop реально латчится в Quick Stop Active, а не только обнуляет скорость; если драйвер перебивает controlword — перейти на halt()/quick-stop API самого `Cia402Driver`. В Gazebo не воспроизводимо.**
- [x] 0.3 `[FMEA]` Per-cycle fault poll: читать statusword `0x6041` каждый цикл (а не раз в 100), реагировать на fault-бит немедленным quick-stop, а не только логировать. — `read()` теперь сканирует statusword каждые `fault_poll_decimation_`=5 циклов (10 Гц @50 Гц, ≤100 мс) и при fault-бите вызывает `request_quick_stop()` (координированный стоп обоих колёс). _Поведенческий FMEA-фикс закрыт; истинно per-cycle без блокирующего SDO требует чтения TPDO1-кэша statusword — отложено до подтверждения accessor драйвера на build-хосте (помечено в коде)._
- [ ] 0.4 `[FMEA]` CAN bus-off detection + recovery: обнаружение bus-off на SocketCAN, безопасное обнуление команд, контролируемое восстановление NMT. — _Отложено: требует API lely/SocketCAN-состояния, не подтверждаемого на Windows; механизм латча quick-stop уже готов как точка реакции._
- [x] 0.5 `cmd_vel` watchdog: при отсутствии свежего `cmd_vel` дольше порога → нулевая скорость + переход в HOLD (узел в `search_coordinator` или отдельный, перед `twist_mux`). — `scripts/cmd_vel_watchdog.py`: pass-through `/cmd_vel`→`/cmd_vel_safe`, при простое > `timeout` (0.5 с) — детерминированный нулевой Twist + HOLD + статус `~/hold`, опц. латч (`~/clear_hold`). Подключён перед twist_mux в `hardware_bringup` и `launch_sim` (проверяемо в Gazebo); вход `navigation` twist_mux перенаправлен на `cmd_vel_safe`.
- [ ] 0.6 Подключить Nav2 Collision Monitor поверх локального `/scan`. — _Конфиг `config/collision_monitor.yaml` + `launch/collision_monitor.launch.py` готовы (stop+slowdown полигоны, источник `scan`, вход = выход twist_mux). Активация после появления `/scan` (Phase 1.4): перенаправить вход `twist_to_twist_stamped` на `/cmd_vel_collision_safe`._
- [ ] 0.7 Убедиться, что `Stop.action` (mode QUICK_STOP_REQUEST) доходит до hardware quick-stop и подтверждает `zero_velocity_confirmed`. — _Зависит от `Stop.action` (Phase 2.1) и канала внешнего триггера в hardware-interface; механизм (`quick_stop_active_` + `transmit_quick_stop`) уже на месте, осталось подать сигнал извне._
- [ ] **EXIT:** в Gazebo и на стенде HIL команда стоп/деградация приводит к подтверждённой остановке < 200 мс; искусственный fault и bus-off вызывают quick-stop; `use_sim_time` не зашит в код. — _Частично: `use_sim_time` чист; watchdog-деградация проверяема в Gazebo; CiA-402 quick-stop/fault/bus-off требуют HIL-стенда._

### Phase 1 — Транспорт + каркас симуляции
- [ ] 1.1 Поднять один `rmw_zenoh` systemd-роутер на edge; multicast OFF; 12 МБ буферы сокетов на всех хостах; зафиксировать fallback Fast DDS LARGE_DATA + Discovery Server.
- [ ] 1.2 `[FMEA]` Развернуть `chrony` на всех хостах; измерить offset и доказать, что он существенно меньше окон 0.2 с (TF), 0.35 с (depth-match), 1.5 с (pixel-age).
- [ ] 1.3 Навесить QoS deadline/liveliness на все кросс-линковые топики; ввести `Heartbeat.msg` от продьюсеров.
- [ ] 1.4 Локальный `/scan` через `depthimage_to_laserscan`; подключить как obstacle-source costmap; подтвердить, что PointCloud2/raw depth по Wi-Fi не уходят.
- [ ] 1.5 Gazebo-on-WSL bring-up: миры `test_1..3`, запуск симулятора, RealSense-эмуляция, проверка топиков.
- [x] 1.6 Создать ветки/каркас новых пакетов: `ar_project_msgs`, `object_tracking_msgs`, `search_coordinator`, `planner_orchestrator` (пустые сборки проходят). — Все 4 собраны в WSL (rosidl-генерация 13 интерфейсов; скелеты `coordinator_node`/`orchestrator_node` запускаются). ⚠️ Репо `ar_project`/`object_tracking` — single-package (`package.xml` в корне), поэтому новые пакеты вложены и colcon не находит их рекурсивно: в workspace подключены симлинками `src/<pkg> -> src/<repo>/<pkg>` (или `colcon build --paths …`). Открытый вопрос: оставить симлинки vs. реструктурировать репо в multi-package.
- [ ] **EXIT:** кросс-хостовая связность по zenoh с измеренным jitter в пределах бюджета; `/scan` локальный; Gazebo-сцена поднимается одной командой; пустые пакеты собираются `colcon build`.

### Phase 2 — ZERO-VLM FLAT базлайн (гейт для всего остального)
- [x] 2.1 Объявить все интерфейсы skill-actions в `ar_project_msgs` (`ExploreFrontier`, `GoToPose`, `ApproachDetection`, `GetObservation`, `Stop`) + `MapOdomCorrection.msg`. — Объявлены + `Heartbeat.msg`, `SetMode.srv`; типы генерируются. `GetObservation` несёт `object_tracking_msgs/Candidate[]` (кросс-зависимость разрешается в общем workspace).
- [ ] 2.2 Реализовать executive FSM/BT в `search_coordinator`: владение mission state, всегда удерживается committed subgoal + default-productive-action.
- [ ] 2.3 `[FMEA]` Локальный frontier-extractor из costmap + явный гистерезис выбора (score margin + min dwell time) для подавления осцилляции.
- [ ] 2.4 Реализовать skill-серверы (Explore/GoTo/Approach/GetObs/Stop): preemptable, feedback-carrying, UUID-идемпотентные.
- [ ] 2.5 `[FMEA]` Mission-epoch: смена инструкции = executive ABORT-and-reset + инкремент epoch, инвалидирующий in-flight UUID; «зомби»-цели прежней эпохи отбрасываются.
- [ ] 2.6 `map_odom_relay`: применяет low-rate `MapOdomCorrection` от edge-SLAM, держит last-good, гейтит скачок/ковариацию, отбраковывает stale по `seq`/stamp, ребродкастит map→odom локально < `transform_tolerance` 0.2 с.
- [ ] 2.7 Облегчить Nav2: NavFn+DWB, local costmap в `odom`, controller_frequency 8–10 Гц (сейчас 15), убрать неиспользуемое; профилировать на Pi.
- [ ] 2.8 Адаптировать `target_pixel_to_goal`: снять `goal_locked`/`prompt_ack`/`lock_goal_on_publish`/авто-`SUCCEEDED`, обернуть в `ApproachDetection`-сервер.
- [ ] 2.9 Удалить `reliable_prompt_sender.py` + его launch + латч-«soup» топики; убрать реактивный `cmd_vel` из edge-трекера.
- [ ] 2.10 Собрать чисто-FLAT сценарий: дано описание цели → SEARCH (фронтиры) + DETECT (детектор→pixel→`target_pixel_to_goal`→3D goal) + DRIVE (Nav2). Измерить производительность на профиле Pi (CPU, частоты, задержки).
- [ ] **EXIT:** робот в Gazebo на Pi-классе нагрузки находит и подъезжает к цели в чисто-FLAT режиме без VLM; нет осцилляции фронтиров; смена инструкции корректно сбрасывает миссию; измеренный baseline зафиксирован как гейт — дальнейшие фазы не начинаются, пока этот критерий не выполнен.

### Phase 3 — Восприятие как сервис
- [x] 3.1 Объявить `DetectTarget.action` + `Candidate.msg` в `object_tracking_msgs`. — Объявлены вместе с остальными интерфейсами на этапе 1.6 (типы генерируются).
- [ ] 3.2 Реализовать `DetectTarget`-сервер на edge: YOLOE по умолчанию, GroundingDINO+MobileSAM fallback; CLIPSeg из грудинга исключён.
- [ ] 3.3 Set-of-Mark рендер кандидатов (нумерованные метки) для будущего выбора VLM по `mark_id`.
- [ ] 3.4 `[FMEA]` Детект staleness потока детекций: `ApproachDetection` не объявляет `reached` на устаревшем пикселе (`detection_fresh=false`, `max_pixel_age_s`), возвращает `STALE_DETECTION`/`LOST_TARGET` вместо ложного success.
- [ ] 3.5 `GetObservation` отдаёт сжатый кадр (CompressedImage) + кандидатов; никаких PointCloud2 по Wi-Fi.
- [ ] **EXIT:** детектор работает как запросный сервис с Set-of-Mark; устаревший пиксель никогда не приводит к авто-success подъезда (проверено инъекцией задержки потока).

### Phase 4 — VLM-режим
- [x] 4.1 Объявить `SeekObject.action`, `PlanStep.msg`, `Notes.msg`. — Объявлены в `object_tracking_msgs` на этапе 1.6 (типы генерируются).
- [ ] 4.2 Planner Orchestrator: лёгкий async HTTP-клиент к **внешнему OpenAI-совместимому VLM API** (Qwen3-VL; `base_url`+ключ, модель не хостим, GPU не нужен — сервер vLLM/SGLang/облако вне системы); single-in-flight, UUID-идемпотентность, streaming.
- [ ] 4.3 Structured/enum tool-call: VLM выбирает только `frontier_id` / `approach_target` из реального списка; навигационных координат не порождает.
- [ ] 4.4 Timeout из измеренного p99 + circuit-breaker.
- [ ] 4.5 Notes/summary-буфер: модель пишет компактные заметки вместо хранения кадров; контроль бюджета токенов (`token_estimate`).
- [ ] 4.6 Anytime/async replan: lead-time/интервал зарезервирован так, что FLAT продолжает исполнять текущую подзадачу, пока считается следующий replan; adoption нового плана только в commit-point (consensus-horizon).
- [ ] 4.7 Декомпозиция миссии в дерево/последовательность FLAT-решаемых подзадач (`explore_room(Y)`, `find(X)`, `approach(Z)`).
- [ ] **EXIT:** VLM-режим декомпозирует высокоуровневую инструкцию, периодически перепланирует по истории, не попадает на реактивный путь, не простаивает между replan’ами (нет «wasted actions»), бюджет токенов ограничен.

### Phase 5 — Hardening деградации + FMEA-тесты в симуляции
- [ ] 5.1 `[FMEA]` Бесшовная деградация VLM→FLAT при потере VLM/edge/Wi-Fi (heartbeat DOWN или circuit-breaker open): миссия продолжается как FLAT, результат может быть `DEGRADED_SUCCESS`.
- [ ] 5.2 `[FMEA]` Тест инъекции stale TF / просроченной `MapOdomCorrection`: `map_odom_relay` держит last-good и не пропускает скачок/устаревшую коррекцию.
- [ ] 5.3 `[FMEA]` Тест скачка локализации (`relocalized=true`): гейтинг скачка, отсутствие «телепортации» цели.
- [ ] 5.4 `[FMEA]` Тест потери edge посреди подъезда: нет ложного reached; переход в FLAT/повторный поиск.
- [ ] 5.5 `[FMEA]` Тест bus-off и fault EPOS4 в движении: подтверждённый quick-stop (повтор Phase 0 в полном пайплайне).
- [ ] 5.6 Тест instruction-change mid-mission: ABORT-and-reset, отсутствие исполнения «зомби»-UUID прежней эпохи.
- [ ] 5.7 Тест осцилляции фронтиров под шумом: гистерезис удерживает выбор.
- [ ] **EXIT:** полный FMEA-набор в Gazebo зелёный; каждый сценарий отказа приводит к безопасному и предсказуемому поведению; деградация VLM→FLAT воспроизводимо бесшовна.

### Phase 6 — Ввод в эксплуатацию на железе (hardware bring-up)
- [ ] 6.1 `use_sim_time=False`, реальные часы, `chrony` на роботе; проверка offset на реальном Wi-Fi.
- [ ] 6.2 Поднять CAN/EPOS4 на реальном `EmbodiedRobotSystem`; проверить quick-stop, fault-poll, bus-off recovery на железе (HIL).
- [ ] 6.3 RealSense + локальный `/scan` + EKF + облегчённый Nav2 на Pi 5/4GB; подтвердить, что SLAM(edge)+Nav2(Pi) укладываются в бюджет CPU.
- [ ] 6.4 RTAB-Map: offline mapping → `.db`, online localization → `MapOdomCorrection` на реальной карте.
- [ ] 6.5 Прогон FLAT-миссии на железе; затем VLM-миссии (через внешний OpenAI-совместимый VLM API — облако или отдельный сервер).
- [ ] 6.6 Полевой прогон деградации (физическое отключение Wi-Fi/edge) и повтор ключевых FMEA-сценариев на роботе.
- [ ] **EXIT:** робот выполняет и FLAT-, и VLM-миссии на реальном железе; все безопасностные и деградационные сценарии воспроизводятся в поле; baseline-метрики совпадают с симуляцией в пределах допуска.

---

Ключевые файлы-якоря (абсолютные пути). NB: репозитории реструктурированы в multi-package — основной пакет `ar_project` теперь в подпапке `ar_project/ar_project/`, а `object_tracking` — в `object_tracking/object_tracking/`; новые пакеты (`ar_project_msgs`, `search_coordinator`, `object_tracking_msgs`, `planner_orchestrator`) лежат в корне соответствующего репо. Сборка — нативная (`colcon build`, без симлинков).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/ar_project/src/embodied_robot_system.cpp` — путь `write()` и `poll_fault_state` (Phase 0.2–0.3 ✅: quick-stop через controlword `0x6040`=`0x0002` на RPDO1 в `write()`, реактивный fault-poll каждые 5 циклов с `request_quick_stop`). Остаётся 0.4 (bus-off) и истинно per-cycle через TPDO-кэш statusword; блокирующий 50 мс SDO живёт только в `switch_operation_mode_via_sdo` (путь активации/смены режима, НЕ RT-`write()`).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/ar_project/config/nav2_params.yaml` — `use_sim_time: False` (Phase 0.1 ✅; sim получает `True` через `RewrittenYaml`), `controller_frequency: 15.0`, local costmap в `odom`, global costmap в `map` (ещё облегчить — Phase 2.7).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/ar_project/scripts/target_pixel_to_goal.py` — переиспользуемая математика + «soup» (`goal_locked`, `prompt_ack`, `lock_goal_on_publish`, авто-success по `nav_status`) к удалению/выносу (Phase 2.8, 3.4).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/ar_project/scripts/reliable_prompt_sender.py` + `C:/Users/dende/code/mobile_robot_navigation/ar_project/ar_project/launch/reliable_prompt_sender.launch.py` — DELETE (Phase 2.9).
- `C:/Users/dende/code/mobile_robot_navigation/object_tracking/object_tracking/object_tracking/tracker_node.py` — реактивный `search_cmd_pub`→`/cmd_vel` и латч-логика к удалению; бэкенды (`yoloe_*`, `dino_mobilesam_*`) переиспользуются, `clip_*` исключается из грудинга (Phase 2.9, 3.2).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/ar_project/CMakeLists.txt` и `.../package.xml` — пакет `ar_project`; новые пакеты — самостоятельные (`ar_project_msgs/`, `search_coordinator/` в корне репо), не регистрируются здесь.