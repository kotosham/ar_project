# ROADMAP — робастная архитектура (ветки `robust`)

## 4. ROADMAP (пошаговый чек-лист)

Порядок строго FIX-FIRST: сначала безопасность и корректность (Phase 0), затем транспорт и симуляция, затем ZERO-VLM FLAT-базлайн, и только после него — восприятие как сервис, VLM-режим, hardening и железо. Каждая фаза имеет критерий выхода (EXIT). FMEA-обязательные фиксы помечены `[FMEA]`. Прогресс отмечается галочками в `- [ ]`.

### Phase 0 — Безопасность и корректность (FIX-FIRST)
- [ ] 0.1 Параметризовать `use_sim_time` во всех yaml/launch (`nav2_params.yaml`, `localization_launch.py`, `navigation_launch.py`, EKF и пр.); по умолчанию `False` на железе, `True` только в Gazebo-launch.
- [ ] 0.2 `[FMEA]` Реализовать реальный CiA-402 quick-stop: запись controlword `0x6040` (бит quick-stop, переход в Quick Stop Active) на RT-пути `write()` в `EmbodiedRobotSystem`, БЕЗ блокирующего 50 мс SDO; quick-stop делать через PDO/неблокирующий путь.
- [ ] 0.3 `[FMEA]` Per-cycle fault poll: читать statusword `0x6041` каждый цикл (а не раз в 100), реагировать на fault-бит немедленным quick-stop, а не только логировать.
- [ ] 0.4 `[FMEA]` CAN bus-off detection + recovery: обнаружение bus-off на SocketCAN, безопасное обнуление команд, контролируемое восстановление NMT.
- [ ] 0.5 `cmd_vel` watchdog: при отсутствии свежего `cmd_vel` дольше порога → нулевая скорость + переход в HOLD (узел в `search_coordinator` или отдельный, перед `twist_mux`).
- [ ] 0.6 Подключить Nav2 Collision Monitor поверх локального `/scan`.
- [ ] 0.7 Убедиться, что `Stop.action` (mode QUICK_STOP_REQUEST) доходит до hardware quick-stop и подтверждает `zero_velocity_confirmed`.
- [ ] **EXIT:** в Gazebo и на стенде HIL команда стоп/деградация приводит к подтверждённой остановке < 200 мс; искусственный fault и bus-off вызывают quick-stop; `use_sim_time` не зашит в код.

### Phase 1 — Транспорт + каркас симуляции
- [ ] 1.1 Поднять один `rmw_zenoh` systemd-роутер на edge; multicast OFF; 12 МБ буферы сокетов на всех хостах; зафиксировать fallback Fast DDS LARGE_DATA + Discovery Server.
- [ ] 1.2 `[FMEA]` Развернуть `chrony` на всех хостах; измерить offset и доказать, что он существенно меньше окон 0.2 с (TF), 0.35 с (depth-match), 1.5 с (pixel-age).
- [ ] 1.3 Навесить QoS deadline/liveliness на все кросс-линковые топики; ввести `Heartbeat.msg` от продьюсеров.
- [ ] 1.4 Локальный `/scan` через `depthimage_to_laserscan`; подключить как obstacle-source costmap; подтвердить, что PointCloud2/raw depth по Wi-Fi не уходят.
- [ ] 1.5 Gazebo-on-WSL bring-up: миры `test_1..3`, запуск симулятора, RealSense-эмуляция, проверка топиков.
- [ ] 1.6 Создать ветки/каркас новых пакетов: `ar_project_msgs`, `object_tracking_msgs`, `search_coordinator`, `planner_orchestrator` (пустые сборки проходят).
- [ ] **EXIT:** кросс-хостовая связность по zenoh с измеренным jitter в пределах бюджета; `/scan` локальный; Gazebo-сцена поднимается одной командой; пустые пакеты собираются `colcon build`.

### Phase 2 — ZERO-VLM FLAT базлайн (гейт для всего остального)
- [ ] 2.1 Объявить все интерфейсы skill-actions в `ar_project_msgs` (`ExploreFrontier`, `GoToPose`, `ApproachDetection`, `GetObservation`, `Stop`) + `MapOdomCorrection.msg`.
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
- [ ] 3.1 Объявить `DetectTarget.action` + `Candidate.msg` в `object_tracking_msgs`.
- [ ] 3.2 Реализовать `DetectTarget`-сервер на edge: YOLOE по умолчанию, GroundingDINO+MobileSAM fallback; CLIPSeg из грудинга исключён.
- [ ] 3.3 Set-of-Mark рендер кандидатов (нумерованные метки) для будущего выбора VLM по `mark_id`.
- [ ] 3.4 `[FMEA]` Детект staleness потока детекций: `ApproachDetection` не объявляет `reached` на устаревшем пикселе (`detection_fresh=false`, `max_pixel_age_s`), возвращает `STALE_DETECTION`/`LOST_TARGET` вместо ложного success.
- [ ] 3.5 `GetObservation` отдаёт сжатый кадр (CompressedImage) + кандидатов; никаких PointCloud2 по Wi-Fi.
- [ ] **EXIT:** детектор работает как запросный сервис с Set-of-Mark; устаревший пиксель никогда не приводит к авто-success подъезда (проверено инъекцией задержки потока).

### Phase 4 — VLM-режим
- [ ] 4.1 Объявить `SeekObject.action`, `PlanStep.msg`, `Notes.msg`.
- [ ] 4.2 Planner Orchestrator: асинхронный клиент к Qwen3-VL через OpenAI-совместимый vLLM API; single-in-flight, UUID-идемпотентность, streaming.
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
- [ ] 6.5 Прогон FLAT-миссии на железе; затем VLM-миссии (локальный или удалённый vLLM).
- [ ] 6.6 Полевой прогон деградации (физическое отключение Wi-Fi/edge) и повтор ключевых FMEA-сценариев на роботе.
- [ ] **EXIT:** робот выполняет и FLAT-, и VLM-миссии на реальном железе; все безопасностные и деградационные сценарии воспроизводятся в поле; baseline-метрики совпадают с симуляцией в пределах допуска.

---

Ключевые файлы-якоря (абсолютные пути):
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/src/embodied_robot_system.cpp` — путь `write()` и `poll_and_log_fault_state` (Phase 0.2–0.4: сейчас quick-stop отсутствует, fault раз в 100 циклов, блокирующий 50 мс SDO в `switch_operation_mode_via_sdo`).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/config/nav2_params.yaml` — `use_sim_time: True` зашит, `controller_frequency: 15.0`, local costmap уже в `odom`, global costmap в `map` (Phase 0.1, 2.7).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/scripts/target_pixel_to_goal.py` — переиспользуемая математика + «soup» (`goal_locked`, `prompt_ack`, `lock_goal_on_publish`, авто-success по `nav_status`) к удалению/выносу (Phase 2.8, 3.4).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/scripts/reliable_prompt_sender.py` + `C:/Users/dende/code/mobile_robot_navigation/ar_project/launch/reliable_prompt_sender.launch.py` — DELETE (Phase 2.9).
- `C:/Users/dende/code/mobile_robot_navigation/object_tracking/object_tracking/tracker_node.py` — реактивный `search_cmd_pub`→`/cmd_vel` и латч-логика к удалению; бэкенды (`yoloe_*`, `dino_mobilesam_*`) переиспользуются, `clip_*` исключается из грудинга (Phase 2.9, 3.2).
- `C:/Users/dende/code/mobile_robot_navigation/ar_project/CMakeLists.txt` и `.../package.xml` — точки регистрации новых пакетов/таргетов и удаления установки `reliable_prompt_sender.py`.