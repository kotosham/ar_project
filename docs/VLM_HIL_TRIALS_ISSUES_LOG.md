# VLM HIL trials: issue log

Журнал проб и ошибок по тестам VLM-пайплайна на реальном роботе.

Формат: проблема -> как проявлялась -> причина -> что сделали -> как проверять.

## 1. Zenoh/RMW transport ломал стабильность Pi

Проблема: после запуска edge-компонентов на ноутбуке Pi-терминалы начинали сыпать transport warnings/errors, иногда падал `collision_monitor`, появлялись задержки управления.

Симптомы:

- `Unable to connect to any locator of scouted peer ... tcp/127.0.0.1 / fe80 / 10.x`
- `zenoh_transport ... Unable to push non droppable network message`
- `lifecycle_manager_collision_monitor: CRITICAL FAILURE`
- ручной teleop начинал идти с огромной задержкой

Причина: ROS-процессы пытались находить друг друга peer/gossip/direct-соединениями, а не через один edge-router. Pi начинала пытаться соединяться с лишними locator-ами.

Исправление:

- Pi ROS nodes переведены в `mode: "client"`.
- `gossip: { enabled: false }`.
- Все ROS-процессы подключаются к одному router endpoint на ноутбуке.
- Перед запуском терминалов чистим лишние ROS discovery env vars и используем `transport_env.sh`.

Проверка:

```bash
cat /etc/zenoh/zenoh_session_config.json5 | grep -E 'mode:|gossip'
```

Нормально:

```text
mode: "client"
gossip: { enabled: false }
```

## 2. Zenoh warnings выглядели страшно, но не были фатальными

Проблема: в логах постоянно появлялись warnings, которые сначала воспринимались как ошибки.

Симптомы:

- `Watchdog Confirmator: error setting scheduling priority`
- `Watchdog Validator: error setting scheduling priority`
- `heartbeat QoS event callbacks are not supported by this RMW`
- `Scouting delay elapsed before start conditions are met`

Причина: особенности `rmw_zenoh_cpp`, SHM watchdog и ROS QoS callbacks.

Решение: считаем эти строки нефатальными, если при этом:

- топики идут с нормальной частотой;
- Nav2 lifecycle nodes active;
- `/heartbeat` не stale постоянно;
- robot health cards OK.

Проверка:

```bash
ros2 control list_controllers
timeout 8 ros2 topic hz /joint_states
timeout 8 ros2 topic hz /odometry/filtered
timeout 8 ros2 topic hz /scan
```

## 3. RealSense depth/RGB имели разное разрешение

Проблема: RGB был `640x480`, depth был `424x240`. Детектор возвращал пиксель в RGB-координатах, а глубина бралась как будто размеры совпадают.

Симптомы:

- объект найден, но `distance_m` неверная или 0;
- робот едет не к тому месту;
- в логах RealSense:

```text
Depth Z16 Width: 424 Height: 240 FPS: 15
```

Причина: aligned depth topic сохранял depth-разрешение, а код должен был пересчитывать координаты RGB -> depth.

Исправление:

- Перестали требовать одинаковый размер окон.
- В детекторе/геометрии учитываем реальные размеры RGB/depth.
- Оставили дефолтный depth profile RealSense.

Проверка:

```bash
ros2 topic echo /camera_edge/aligned_depth_to_color/image_raw --once --field encoding
ros2 topic echo /camera_edge/aligned_depth_to_color/image_raw --once --field header
timeout 8 ros2 topic hz /camera_edge/aligned_depth_to_color/image_raw
```

Ожидаемо:

```text
16UC1
frame_id: camera_color_optical_frame
```

## 4. Камера и SLAM перегружали Wi-Fi/Pi

Проблема: цветной кадр/point cloud/RGB-D поток в RViz и RTAB-Map создавали задержки, иногда depth stream отваливался.

Симптомы:

- RViz color image сильно отстает.
- RTAB-Map не обновляется или теряет кадры.
- `/camera_edge/...` частота плавает.
- Pi начинает тормозить, teleop задерживается.

Причина: RGB-D поток + SLAM + RViz cloud тяжёлые для Wi-Fi и Pi.

Исправление:

- SLAM вынесен на ноутбук.
- RealSense FPS снижали до 6 fps для железных тестов.
- В RViz отключали тяжёлые RGB/PointCloud displays, если они не нужны прямо сейчас.
- Проверяли, что RTAB-Map получает `/camera_edge/color/image_raw`, `/camera_edge/aligned_depth_to_color/image_raw`, `/odometry/filtered`.

Проверка:

```bash
timeout 8 ros2 topic hz /camera_edge/color/image_raw
timeout 8 ros2 topic hz /camera_edge/aligned_depth_to_color/image_raw
timeout 8 ros2 topic hz /map
timeout 8 ros2 topic hz /map_odom_correction
```

## 5. RTAB-Map map->odom correction stale

Проблема: Dashboard показывал stale `map->odom`, хотя RTAB-Map визуально работал.

Симптомы:

- `SLAM-коррекция map->odom: stale`
- `/map_odom_correction age` растёт
- `/mapGraph` есть, но correction publisher не обновляет выход

Причина: QoS/MapGraph особенности и relay-логика. RTAB-Map publish_tf_map должен быть выключен, а `map_odom_relay` держит last-good correction.

Исправление:

- Запускаем `map_odom_relay`.
- RTAB-Map должен быть настроен так, чтобы не публиковать TF map сам.
- Отдельно проверяем `/mapGraph` и `/map_odom_correction`.

Проверка:

```bash
timeout 8 ros2 topic hz /mapGraph
timeout 8 ros2 topic hz /map_odom_correction
ros2 topic echo /map_odom_correction --once
```

## 6. YOLOE давал шум и плохо держал целевые офисные классы

Проблема: YOLOE в VLM-режиме часто путал офисные объекты, иногда видел стул там, где была тумба/полка/ручка.

Симптомы:

- `chair`/`office chair` на тумбе или чёрной панели.
- `drawer cabinet` разбивается на много кандидатов.
- `DETECT_ALL` добавляет шумные объекты, и VLM начинает рассуждать по мусору.

Причина: open-vocab/DETECT_ALL режим слишком широкий и шумный для низкой камеры в офисной сцене.

Исправление:

- Целевую детекцию перевели на `GroundingDINO + MobileSAM`.
- YOLOE оставлен для broad `DETECT_ALL`, но не как главный target detector.
- Для офисного контекста добавлен DINO office-vocab список.
- Пороги подняты ближе к дипломной реализации.

Текущая идея порогов:

```text
target_detect_conf >= 0.60 для строгой цели
context_detect_conf около 0.30 для офисных подсказок
detect_all_conf ниже, только для обзорного режима
```

Проверка:

```bash
ros2 action send_goal /detect_target object_tracking_msgs/action/DetectTarget \
  "{request_id: 'debug', mission_epoch: 0, query: 'office chair', render_setofmark: false, conf_threshold: 0.60}" \
  --feedback
```

## 7. Одна случайная детекция слишком легко становилась целью

Проблема: единичный ложный `office chair` мог стать кандидатом, и робот ехал к нему.

Симптомы:

- В dashboard виден один сомнительный bbox.
- Робот сразу переключается в `DRIVE_TO_VISIBLE`.
- Пример: ручка тумбы или стенка стола распознана как `office chair`.

Причина: принимали single-frame detection как достаточно надёжную.

Исправление:

- Добавлен `target_confirm`: цель должна подтвердиться в нескольких наблюдениях.
- В логах видно:

```text
target_confirm[target]: 1 raw -> 1 confirmed over 2 observation(s)
target_confirm[target]: 1 raw -> 0 confirmed over 2 observation(s)
```

Проверка: смотреть события `target_confirm` в `vlm_activity_*.jsonl`.

## 8. Context object ошибочно становился целью

Проблема: `context_marks` вроде `desk`, `drawer cabinet`, `office chair` из office-vocab стали использоваться как цель для подъезда.

Симптомы:

- VLM выбирает `DRIVE_TO_VISIBLE` к context mark.
- Робот едет к тумбе/столу вместо исследования коридора.
- `context_promote` продвигал слабый context-like объект в candidates.

Причина: сначала мы разрешили context-promoted candidates, чтобы не терять target-like подсказки. Потом стало понятно, что это загрязняет target approach.

Исправление:

- Context marks больше не promote-ятся в target candidates.
- Context objects теперь только подсказки для выбора коридора/направления обзора.
- Prompt уточнён: `context_marks are not destinations`.
- `DRIVE_TO_VISIBLE` разрешён только для strict `visible_marks`/target candidates.

Проверка:

```text
context_detect[dino_office]: ...
observe: 0 target detection(s) context=N
```

Если target detection(s)=0, робот не должен ехать к context object как к цели.

## 9. Робот слишком долго крутился на месте при поиске

Проблема: при отсутствии цели робот выбирал мелкие повороты или чередовал повороты вокруг одной мебели.

Симптомы:

- `TURN +0.17rad` или `TURN +0.30rad`
- Nav2 сразу считает поворот выполненным, кадр почти не меняется.
- Робот смотрит на тот же пятачок, а не исследует коридор.

Причина: VLM пыталась "уточнять обзор" микроповоротами, а Nav2 tolerance делал такие действия бессмысленными.

Исправление:

- Минимальный осмысленный VLM-поворот нормализован примерно до `0.60rad`.
- Малые повороты в semantic exploration больше не считаются полезными.
- Добавлен начальный structured scan: forward -> right -> left.

Проверка:

```text
TURN -1.57rad
TURN +1.57rad
TURN +1.57rad
```

Затем должен идти выбор коридора, а не бесконечный scan.

## 10. Нужно было явно сканировать forward/right/left перед выбором коридора

Проблема: если цели нет в первом кадре, VLM слишком рано выбирала случайный локальный объект или направление.

Симптомы:

- Сразу `DRIVE_FORWARD` или `TURN` без понимания правой/левой стороны.
- Выбор коридора выглядел случайным.

Причина: модель не имела "среза" трёх направлений.

Исправление:

- Добавлен `initial_scan`:

```text
step 0: forward view, if no target -> TURN right ~90deg
step 1: right view, if no target -> TURN left ~90deg back
step 2: continue left ~90deg
step 3: left view, then choose corridor
```

- В память пишутся:

```text
CORRIDOR_SCAN[forward]
CORRIDOR_SCAN[right]
CORRIDOR_SCAN[left]
```

Назначение: сравнить коридоры, выбрать свободный/релевантный, затем двигаться.

## 11. Кадры после поворота были смазанными

Проблема: сразу после поворота запускалась детекция, картинка была motion-blurred, DINO давал шум.

Симптомы:

- После поворота объект детектится странно.
- В dashboard виден смазанный кадр.
- VLM принимает решение по нестабильному изображению.

Причина: наблюдение начиналось до стабилизации камеры/робота.

Исправление:

- Добавлена пауза после turn:

```text
turn settle: waiting 2.00s before next observation
```

Проверка: после `TURN` в логах должен быть `turn settle`.

## 12. Если цель пропадала из кадра при подъезде, миссия разваливалась

Проблема: робот видел стул, начинал ехать, стул выходил из кадра из-за близости/окклюзии, система считала цель потерянной и возвращалась в exploration/fallback.

Симптомы:

- `target visible -> DRIVE_TO_VISIBLE`
- потом `0 target detection(s)`
- дальше `DETECT_ALL`, `TURN`, `DONE no target`, fallback

Причина: не было памяти о последней уверенно локализованной цели.

Исправление:

- Добавлен `target_lock`.
- Добавлен `target_nav_lock`.
- Если цель была уверенно найдена и локализована, робот продолжает идти к сохранённой координате даже если цель временно пропала из кадра.

Проверка:

```text
target_lock: remembered "office chair" conf=...
target_nav_lock: target was already confidently localized ...
DRIVE_TO_LOCKED_TARGET
```

## 13. Цель вне известной карты останавливала процесс

Проблема: объект виден, координата найдена, но Nav2 не строит маршрут, потому что финальная/промежуточная точка вне known-free области.

Симптомы:

- `DRIVE_TO_VISIBLE`
- `final_goal=clearance_occupied_100`
- `last_bounded=clearance_unknown`
- `ABORTED no safe bounded approach`

Причина: SLAM карта строится онлайн, а объект может быть за пределами known-free области.

Исправление:

- `approach_detection` стал ограничивать дальние цели bounded step-ом.
- Если final goal недоступен, пробует ближайшую safe bounded point.
- Если bounded тоже недоступен, оркестратор включает `target_approach_blocked` recovery.

Проверка:

```text
bounded_step=1.00m bounded_goal=known_free
target_approach_blocked: ... advance 0.55m ... then retry
```

## 14. ApproachDetection мог не найти безопасную bounded-точку

Проблема: цель есть, но costmap считает и финальную, и bounded-точку занятой.

Симптомы:

```text
ABORTED (no safe bounded approach;
target_range=...
final_goal=clearance_occupied_100
last_bounded=clearance_occupied_100)
```

Причина: линия подъезда к объекту проходит через occupied/unknown клетки, или объект стоит за мебелью/рядом с препятствием.

Исправление:

- Добавлен `target_approach_blocked` режим.
- После ABORTED робот делает короткие recovery forward steps, раскрывает карту/меняет геометрию, затем пробует locked target снова.

Параметр:

```text
target_approach_blocked_forward_m:=0.55
```

Это не выбирает VLM. Это фиксированный recovery параметр оркестратора.

## 15. Recovery был грубым: всегда ехал вперед

Проблема: при blocked target recovery робот делал `DRIVE_FORWARD +0.55m` независимо от того, цель слева/справа/по центру.

Симптомы:

- Цель справа, но recovery всё равно forward.
- Робот доезжает рывками.

Причина: recovery специально простой и deterministic: продвинуться по текущему free corridor и попробовать target снова.

Текущий статус:

- Работает как safety/progress fallback.
- Не VLM-controlled.
- Возможное улучшение: делать recovery направленным по side/locked target, но осторожно, чтобы не вернуться к хаотичным поворотам.

## 16. DRIVE_FORWARD мог вести в мебель, если DINO видел blocker не по центру

Проблема: guard запрещал forward только при близком `center` context object. Но низкая камера может видеть стенку стола/ножку как `left` или `right`, хотя физически это в swept path робота.

Симптомы:

- В кадре близкая мебель слева/справа:

```text
keyboard @0.45m left
drawer cabinet @0.42m left
```

- VLM пишет "no close obstacles blocking forward".
- Робот едет `DRIVE_FORWARD +0.55m` и цепляет стенку стола.

Причина: image side не всегда равен безопасной геометрии базы.

Исправление:

- `center <0.8m` блокирует `DRIVE_FORWARD`.
- `left/right <0.55m` тоже считается опасным swept-path blocker.
- Если blocker слева, робот поворачивает вправо.
- Если blocker справа, робот поворачивает влево.

Проверка:

```text
semantic_explore: ... close left context mark ... turn right ...
semantic_explore: ... close right context mark ... turn left ...
```

## 17. Edge target после recenter ошибочно уходил в initial_scan

Проблема: цель была уверенно найдена на краю кадра, робот правильно делал
короткий поворот для перецентровки, но если после этого цель временно пропадала
из кадра, оркестратор запускал panoramic `initial_scan` как будто цели никогда
не было.

Симптомы:

```text
ВИЖУ step 0: office chair 0.767 @ 5.15m, left edge
РЕШИЛ step 0: TURN +0.45rad
turn_guard -> TURN +0.60rad
ВИЖУ step 1: 0 object(s)
РЕШИЛ step 1: TURN +1.57rad
РЕШИЛ step 2: TURN +1.57rad
```

Поведение выглядело странно: робот сначала правильно повернулся к объекту, но
затем начал широкий обзор вправо/влево вместо того, чтобы продолжать работать с
уже найденным стулом.

Причина:

- `edge_target_guard` не давал сразу ехать к объекту на краю кадра и сначала
  делал recenter-turn.
- После recenter цель могла временно пропасть из `confirmed detections`.
- `target_lock` уже помнил, что цель была найдена, но `_compute_plan()` проверял
  `initial_scan` раньше, чем recovery по последней цели.
- Поэтому step 1 воспринимался как "цели нет с самого начала", и запускался
  `initial_scan`.

Исправление:

- Добавлен pre-VLM `target_lock_recovery` перед `initial_scan`.
- Если strict target уже был подтверждён ранее, но после recenter/probe временно
  исчез, оркестратор сначала пытается восстановить именно эту цель.
- `initial_scan` больше не должен перехватывать управление сразу после найденной
  edge-target цели.

Ожидаемый новый лог:

```text
observe@step 1: 0 target detection(s) -> planner=target_lock_recovery
plan@step 1: target lock recovery action: TURN/DRIVE_FORWARD ...
```

Проверка:

```bash
colcon test --packages-select planner_orchestrator --event-handlers console_direct+
```

Покрыто тестом:

```text
test_recent_edge_target_lock_recovery_preempts_initial_scan
```

## 18. Safe-forward слишком строго запрещал unknown frontier

Проблема: VLM выбирала `DRIVE_FORWARD` для исследования коридора, но робот
физически не двигался.

Симптомы:

- В orchestrator:

```text
plan@step N: VLM returned 1 action(s): DRIVE_FORWARD +0.55m
step N [semantic_explore]: DRIVE_FORWARD +0.55m
```

- На Pi в `search_coordinator`:

```text
go_to_pose: safe_forward ABORTED (no safe fan candidate;
requested=(...), step=0.55m, last_status=clearance_unknown)
```

Причина:

- После фикса от врезаний `DRIVE_FORWARD` стал проходить через `safe_forward`.
- `safe_forward` проверял веер коротких forward-точек и требовал, чтобы весь
  короткий сегмент был `known_free`.
- Для задачи активного исследования это оказалось слишком строго: перспективный
  коридор на онлайн SLAM-карте часто является `unknown/frontier`, а не
  `known_free`.
- В результате робот не ехал в коридор, хотя именно его и нужно было
  исследовать.

Исправление:

- `safe_forward` всё ещё предпочитает `known_free`.
- Но для короткого исследовательского шага теперь разрешён
  `clearance_unknown`.
- `occupied` и `outside_map` остаются жёстким запретом.
- Добавлены параметры:

```text
goto_safe_forward_allow_unknown:=true
goto_safe_forward_unknown_max_step_m:=0.60
```

Ожидаемое поведение:

- Если есть known-free точка в веере, выбирается она.
- Если known-free нет, но есть короткий unknown/frontier сегмент, робот может
  осторожно проехать туда.
- Если впереди occupied/outside-map, движение всё равно abort-ится.

Проверка:

```text
go_to_pose: safe_forward selected goal=(...)
status=known_free
```

или для frontier:

```text
go_to_pose: safe_forward selected goal=(...)
status=clearance_unknown
```

Покрыто тестами:

```text
test_select_safe_forward_goal_allows_short_unknown_frontier_when_enabled
test_select_safe_forward_goal_prefers_known_free_over_unknown_frontier
test_select_safe_forward_goal_still_rejects_occupied_when_unknown_enabled
```

## 19. Collision monitor был отключён из-за timestamp/TF проблем

Проблема: collision monitor спамил invalid source и останавливал робота.

Симптомы:

```text
Failed to get "camera_link"->"base_link" frame transform:
Lookup would require extrapolation into the future
Robot to stop due to invalid source
[scan]: timestamps differ ...
```

Причина: timestamp skew между scan/camera/odom/tf, плюс sensor timeout. При SLAM/transport нагрузке данные приходили с задержкой.

Что сделали:

- На время VLM-отладки выключали collision monitor по умолчанию, чтобы не блокировать все тесты.
- Позже проверили, что при корректном `/scan` и cmd_vel цепочке collision monitor может быть active.

Проверка:

```bash
ros2 lifecycle get /collision_monitor
timeout 8 ros2 topic hz /scan
ros2 topic echo /scan --once --field header
date +%s.%N
ros2 topic info /cmd_vel_out -v
ros2 topic info /cmd_vel_collision_safe -v
ros2 topic info /diff_cont/cmd_vel -v
```

## 20. Collision monitor снижал скорость/останавливал команды

Проблема: при включённом collision monitor движение иногда выглядело как будто "не происходит".

Симптомы:

- `/cmd_vel_out` есть.
- `/cmd_vel_collision_safe` меньше, например `0.04 -> 0.012`.
- `/diff_cont/cmd_vel` получает уже безопасную замедленную команду.

Причина: collision monitor режет скорость рядом с препятствием. Это нормальная функция, если источники валидны.

Вывод: collision monitor полезен для защиты, но требует стабильных `/scan`, TF и timestamp.

## 21. Nav2 иногда крутился у цели

Проблема: робот подъезжал к объекту, потом начинал подкручиваться/парковаться возле препятствий.

Симптомы:

- Визуально стоит на месте и крутится.
- В логах Nav2 получает goal, но долго не терминализируется.
- Локальная costmap/footprint рядом с occupied зонами.

Причина:

- Цель слишком близко к объекту/ножкам/столу.
- Costmap считает часть подхода occupied.
- SLAM карта онлайн и может быть неполной.

Что меняли:

- Смягчали Nav2 параметры для онлайн SLAM.
- Проверяли `approach_offset`, но вернули прежний offset по требованию: остановка должна быть от base_link так, чтобы передний край имел ожидаемый зазор.
- Добавили bounded/direct logic в ApproachDetection.

## 22. RViz вводил в заблуждение из-за отображения local costmap

Проблема: казалось, что local costmap выключена или не использует облако/scan.

Симптомы:

- В RViz не видно ожидаемой розовой зоны.
- Робот всё равно использует Nav2/costmap.

Причина: RViz config/display state и то, что SLAM на ноутбуке, а collision/local safety на Pi.

Проверка:

```bash
ros2 node list | grep -E 'controller_server|planner_server|bt_navigator'
ros2 lifecycle get /controller_server
ros2 topic list | grep costmap
```

## 23. Логи в терминале терялись, нужен persistent mission logger

Проблема: после закрытия терминала/перезапуска терялась история прогона.

Исправление:

- Добавлен `vlm_mission_logger`.
- Пишет подробный JSONL и компактный CSV.

Файлы:

```text
~/ros2_ws/experiment_logs/vlm_missions/vlm_activity_<run_id>.jsonl
~/ros2_ws/experiment_logs/vlm_missions/vlm_steps_<run_id>.csv
```

Что смотреть:

```bash
tail -40 ~/ros2_ws/experiment_logs/vlm_missions/vlm_activity_office_chair_001.jsonl
tail -30 ~/ros2_ws/experiment_logs/vlm_missions/vlm_steps_office_chair_001.csv
```

Ключевые события:

```text
mission_start
corridor_scan
observe
plan
step_start
step_result
target_confirm
target_lock
target_nav_lock
target_approach_blocked_recovery
auto_done
mission_end
```

## 23. Logger дублировал старую миссию при рестарте

Проблема: если перезапустить терминал с logger-ом, но не запускать новую миссию,
в файл мог записаться "второй прогон" из старых событий.

Симптомы:

- В CSV/JSONL появляется новая `mission_start`, хотя миссии не было.
- Все события "нового" прогона имеют одинаковый `logger_rx_iso`.
- Внутренний `activity.stamp` у этих событий старый.
- Пример: `logger_rx_iso=02:45:16`, но `activity.stamp` относится к `02:43`.

Причина: `/vlm/activity` публикуется с `TRANSIENT_LOCAL`, чтобы dashboard мог
получить последнюю историю. Logger тоже подписывался как `TRANSIENT_LOCAL`, поэтому
при новом подключении ROS replay-ил сохранённые события, а logger записывал их как
новый приём.

Исправление:

- `vlm_mission_logger` переведён на `VOLATILE` subscription.
- Dashboard может продолжать использовать replay/history.
- Persistent experiment logger теперь пишет только live-события.

Проверка:

```text
logger_rx_iso должен идти рядом с activity.stamp
при рестарте logger-а без новой миссии не должна появляться новая mission_start
```

## 24. Последний успешный шаблон поведения

Рабочая последовательность для сценария "office chair не виден изначально":

```text
1. initial_scan: forward/right/left
2. записать CORRIDOR_SCAN для трёх направлений
3. если strict target не найден, выбрать свободный коридор
4. DRIVE_FORWARD короткими шагами по коридору
5. при уверенной цели: DRIVE_TO_VISIBLE
6. если цель пропала: target_nav_lock продолжает к saved target
7. если ApproachDetection blocked: target_approach_blocked recovery
8. retry target
9. auto_done после успешного финального approach
```

Пример хорошего финала:

```text
target_nav_lock: ... bounded=False blocked=False final_distance=0.58m
auto_done: DRIVE_TO_VISIBLE succeeded; final approach pose reached
mission_end: steps=16 degraded=false
```

## 25. Что ещё остаётся улучшить

Открытые инженерные вопросы:

- Сделать `target_approach_blocked` recovery более направленным, а не только `DRIVE_FORWARD`.
- Лучше оценивать реальные free corridors структурно, а не только через prompt и карту-картинку.
- Усилить фильтрацию DINO context noise без потери полезных офисных подсказок.
- Вернуть collision monitor включённым по умолчанию, когда timestamp/TF стабильны.
- Логировать рядом с VLM mission ещё compact Nav2/ApproachDetection summary, чтобы проще сопоставлять причины blocked.
