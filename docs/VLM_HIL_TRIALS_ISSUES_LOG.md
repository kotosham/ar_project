# VLM HIL Trials: Issue Log

Краткий журнал проб и ошибок по HIL-тестам VLM/FLAT пайплайна.

Читать так: `симптом -> причина -> что сделали -> как проверить`.

## Текущий Контракт Поведения

- Strict target найден уверенно -> ехать к нему, не продолжать exploration.
- Target пропал после уверенной локализации -> `target_nav_lock` продолжает
  движение к последней подтверждённой точке.
- Goal внутри known-free карты -> direct approach.
- Goal вне known-free карты -> bounded approach.
- Bounded approach невозможен -> recovery и повтор locked target.
- Target не найден -> context objects не цели, а подсказки для выбора коридора.
- Приоритет поиска невидимой цели: свободный/unknown коридор, а не подъезд к
  мебели.
- VLM-повороты меньше примерно `0.60rad` не считаются полезными scan-действиями.
- Persistent logs: имя файла совпадает с `run_id`.

## Быстрые Проверки

```bash
# transport
cat /etc/zenoh/zenoh_session_config.json5 | grep -E 'mode:|gossip'

# health / motion
ros2 control list_controllers
timeout 8 ros2 topic hz /joint_states
timeout 8 ros2 topic hz /odometry/filtered
timeout 8 ros2 topic hz /scan
ros2 topic info /cmd_vel_out -v
ros2 topic info /cmd_vel_collision_safe -v
ros2 topic info /diff_cont/cmd_vel -v

# camera / SLAM
timeout 8 ros2 topic hz /camera_edge/color/image_raw
timeout 8 ros2 topic hz /camera_edge/aligned_depth_to_color/image_raw
timeout 8 ros2 topic hz /map
timeout 8 ros2 topic hz /map_odom_correction

# Nav2
ros2 node list | grep -E 'controller_server|planner_server|bt_navigator'
ros2 lifecycle get /controller_server
ros2 topic list | grep costmap
```

## 1. Transport / Infrastructure

### 1.1 Zenoh Peer/Gossip Ломал Стабильность Pi

- Симптом: `Unable to connect to any locator...`, `Unable to push non droppable network message`, задержки teleop.
- Причина: ROS-процессы пытались соединяться peer/direct путями, а не через один router.
- Фикс: `mode: "client"`, `gossip: false`, единый router endpoint, `transport_env.sh`.
- Проверка: `mode: "client"` и `gossip: { enabled: false }` на обеих машинах.

### 1.2 Страшные Zenoh/RMW Warnings Не Всегда Ошибка

- Симптом: `Watchdog Confirmator`, `Watchdog Validator`, `Scouting delay`, unsupported QoS callbacks.
- Причина: особенности `rmw_zenoh_cpp`/SHM/QoS.
- Фикс: считать нефатальным, если топики живые, lifecycle active, health OK.
- Проверка: `topic hz`, `ros2 control list_controllers`, dashboard health.

### 1.3 Logger Replay Дублировал Старую Миссию

- Симптом: после рестарта logger-а появлялся "новый" прогон без реального запуска.
- Причина: `/vlm/activity` был `TRANSIENT_LOCAL`, logger получал replay истории.
- Фикс: persistent logger подписывается как `VOLATILE`.
- Проверка: без новой миссии не появляется новый `mission_start`; `logger_rx_iso` близко к event `stamp`.

## 2. Camera / SLAM / Map

### 2.1 RealSense RGB/Depth Разного Размера

- Симптом: RGB `640x480`, depth `424x240`; `distance_m` неверная или `0`.
- Причина: RGB pixel использовался без пересчёта в depth pixel.
- Фикс: пересчёт RGB -> depth по реальным размерам кадров.
- Проверка: depth `encoding=16UC1`, `frame_id=camera_color_optical_frame`, стабильный `topic hz`.

### 2.2 Camera/SLAM Перегружали Wi-Fi/Pi

- Симптом: RViz image lag, depth stream отваливается, RTAB-Map теряет кадры, teleop тормозит.
- Причина: RGB-D + SLAM + RViz cloud тяжёлые для Wi-Fi/Pi.
- Фикс: SLAM на ноутбуке, RealSense около `6 fps`, тяжёлые RViz displays выключать.
- Проверка: `topic hz` для color/depth/map/map_odom_correction.

### 2.3 RTAB-Map Correction Stale

- Симптом: dashboard показывает stale `map->odom`, `/map_odom_correction age` растёт.
- Причина: RTAB-Map не должен сам публиковать `map` TF; correction держит relay.
- Фикс: `map_odom_relay`, RTAB-Map `publish_tf_map:=false`, last-good correction.
- Проверка: `/mapGraph` и `/map_odom_correction` публикуются.

### 2.4 RViz Мог Вводить В Заблуждение По Local Costmap

- Симптом: в RViz не видно local costmap/розовой зоны.
- Причина: display config, а не обязательно выключенная costmap.
- Фикс: проверять ROS graph/lifecycle/topic list, а не только RViz.
- Проверка: `controller_server`, `planner_server`, `bt_navigator`, `*costmap*` topics.

## 3. Detector / Perception

### 3.1 YOLOE Шумел На Офисных Сценах

- Симптом: `chair` на тумбе/панели/ручке; broad `DETECT_ALL` добавлял мусор.
- Причина: open-vocab YOLOE слишком шумный для низкой камеры и офисной мебели.
- Фикс: основной hardware/VLM режим переведён на `model_mode:=dino`.
- Статус: YOLOE оставлен только как legacy/comparison режим.

### 3.2 DINO Target Порог Поднят

- Симптом: ложные `office chair` с conf около `0.5`.
- Причина: одиночные слабые target detections слишком легко принимались.
- Фикс: `target_conf_default` / `target_detect_conf` подняты до `0.60`.
- Проверка: `/detect_target` с `conf_threshold: 0.60`.

### 3.3 Single-Frame False Target

- Симптом: одна ложная детекция становилась целью.
- Причина: single-frame detection считалась достаточно надёжной.
- Фикс: `target_confirm`: подтверждение по нескольким наблюдениям.
- Лог: `target_confirm[target]: raw -> confirmed`.

### 3.4 Context Noise Не Должен Становиться Целью

- Симптом: `desk`, `drawer cabinet`, слабый `office chair` из context-vocab становились destination.
- Причина: ранняя логика смешивала context и target candidates.
- Фикс: context не promote-ится в target candidates.
- Правило: `context_marks are not destinations`.

## 4. VLM Planning Logic

### 4.1 Context Objects Только Для Выбора Коридора

- Симптом: робот ехал к тумбе/столу "поискать стул".
- Причина: prompt поощрял подъезд к офисным объектам.
- Фикс: context objects задают релевантность направления, но не точку подъезда.
- Правило: если цели нет, выбирать свободный коридор; context помогает выбрать между коридорами.

### 4.2 Initial Scan Forward/Right/Left

- Симптом: без цели в первом кадре выбор направления выглядел случайным.
- Причина: VLM не имела сравнения направлений.
- Фикс: structured scan: `forward -> right -> left -> choose corridor`.
- Лог: `CORRIDOR_SCAN[forward/right/left]`.

### 4.3 Пауза После Поворота

- Симптом: DINO детектил шум на смазанном кадре после turn.
- Причина: observation начинался до стабилизации робота/камеры.
- Фикс: `turn settle: waiting 2.00s before next observation`.
- Проверка: после `TURN` есть `turn settle`.

### 4.4 Микроповороты Бессмысленны

- Симптом: `TURN +0.17rad` / `TURN +0.30rad`, Nav2 сразу завершает действие.
- Причина: действие попадает в tolerance и почти не меняет кадр.
- Фикс: semantic scan turns нормализуются до примерно `0.60rad+`.
- Важно: это про VLM scan-действия, не про внутренние коррекции controller-а.

### 4.5 Edge Target Не Должен Запускать Новый Initial Scan

- Симптом: target найден на краю, после recenter временно пропал, дальше начался full scan.
- Причина: `initial_scan` срабатывал раньше recovery по recent target.
- Фикс: `target_lock_recovery` поставлен перед `initial_scan`.
- Тест: `test_recent_edge_target_lock_recovery_preempts_initial_scan`.

### 4.6 Scene Exploration Должен Быть Активным

- Симптом: робот крутился на месте и рассматривал один пятачок.
- Причина: слишком сильная привязка к локальной мебели/context marks.
- Фикс: после scan и одного осмысленного turn предпочитать движение по свободному/unknown коридору.
- Проверка: после `CORRIDOR_SCAN` должны появляться `DRIVE_FORWARD`/safe-forward шаги, если коридор безопасен.

## 5. Approach / Nav2

### 5.1 Target Пропадал При Подъезде

- Симптом: `DRIVE_TO_VISIBLE`, затем `0 target detection(s)`, затем fallback/exploration.
- Причина: объект при приближении выходит из кадра, перекрывается или не помещается.
- Фикс: `target_lock` + `target_nav_lock`.
- Лог: `target_nav_lock: target was already confidently localized`.

### 5.2 Goal Вне Known-Free Карты

- Симптом: target виден, но Nav2 не строит путь.
- Причина: камера видит дальше, чем построена SLAM-карта.
- Фикс: bounded approach к ближайшей безопасной точке.
- Лог: `bounded_step=... bounded_goal=known_free`.

### 5.3 No Safe Bounded Approach

- Симптом: `ABORTED (no safe bounded approach; final_goal=... last_bounded=...)`.
- Причина: final и bounded точки заняты/unknown/слишком близко к препятствию.
- Фикс: `target_approach_blocked` recovery, затем retry locked target.
- Параметр: `target_approach_blocked_forward_m:=0.55`.

### 5.4 Safe-Forward Слишком Строго Требовал Known-Free

- Симптом: VLM выбирает `DRIVE_FORWARD`, но Pi пишет `safe_forward ABORTED ... clearance_unknown`.
- Причина: frontier-коридор ещё unknown на онлайн SLAM-карте.
- Фикс: короткий `clearance_unknown` разрешён для exploration.
- Параметры: `goto_safe_forward_allow_unknown:=true`, `goto_safe_forward_unknown_max_step_m:=0.60`.

### 5.5 Close Furniture В Swept Path

- Симптом: близкая мебель слева/справа, VLM едет forward, робот цепляет стол.
- Причина: image side не гарантирует, что объект вне footprint.
- Фикс: близкий `center <0.8m` и `left/right <0.55m` блокируют forward.
- Реакция: blocker слева -> искать вправо; blocker справа -> искать влево.

### 5.6 Nav2 Крутился У Цели

- Симптом: робот у объекта, но controller долго подкручивается.
- Причина: goal близко к occupied зоне/ножкам/столу, online map неполная.
- Фикс: direct/bounded approach, safe bounded point, прежний `approach_offset`.
- Важно: `approach_offset` считается от `base_link`, передний край имеет физический зазор.

## 6. Collision Monitor

### 6.1 Timestamp/TF Проблемы

- Симптом: `Lookup would require extrapolation into the future`, `Robot to stop due to invalid source`.
- Причина: skew между `/scan`, camera, odom и TF при нагрузке.
- Статус: на время VLM-отладки выключали по умолчанию; включать после стабилизации time/TF.
- Проверка: lifecycle active, `/scan` fresh, cmd_vel chain валидна.

### 6.2 Collision Monitor Режет Скорость

- Симптом: `/cmd_vel_out` есть, но `/cmd_vel_collision_safe` меньше.
- Причина: safety slow-down рядом с препятствием.
- Вывод: это нормально, если источники валидны; плохо, если invalid source спамит stop.

## 7. Logs / Metrics

### 7.1 Persistent Mission Logs

- `vlm_mission_logger`: `/vlm/activity` -> JSONL + CSV.
- `flat_mission_logger`: `/mission/status` -> JSONL + CSV.
- Имя файла равно `run_id`.

Файлы:

```text
~/ros2_ws/experiment_logs/vlm_missions/<run_id>.jsonl
~/ros2_ws/experiment_logs/vlm_missions/<run_id>.csv
~/ros2_ws/experiment_logs/flat_missions/<run_id>.jsonl
~/ros2_ws/experiment_logs/flat_missions/<run_id>.csv
```

### 7.2 FLAT Progress Rate

- `DONE -> 1.00`
- `FAILED after APPROACH/DETECT -> 0.66`
- `FAILED after SEARCH -> 0.33`
- `no useful progress -> 0.00`
- `success_auto=1` только для terminal `DONE`; `success_manual` оставлен для ручной разметки.

### 7.3 VLM Good-Run Pattern

```text
1. initial_scan: forward/right/left
2. CORRIDOR_SCAN для направлений
3. strict target found? -> DRIVE_TO_VISIBLE
4. no target? -> choose free/unknown corridor
5. safe-forward по коридору
6. target found -> target_lock + DRIVE_TO_VISIBLE
7. target lost -> target_nav_lock
8. approach blocked -> recovery + retry locked target
9. final approach reached -> auto_done
```

## Открытые Улучшения

- Сделать `target_approach_blocked` recovery более направленным.
- Структурно оценивать коридоры по карте, а не только prompt + map image.
- Сильнее фильтровать DINO context noise без потери полезных офисных подсказок.
- Вернуть collision monitor включённым по умолчанию после стабилизации TF/time.
- Добавить compact Nav2/ApproachDetection summary рядом с VLM mission log.
