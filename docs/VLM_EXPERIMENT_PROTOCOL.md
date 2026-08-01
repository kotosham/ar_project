# VLM experiment protocol

Документ для итоговой интерпретации HIL-прогонов VLM-пайплайна.

Raw artifacts (`*.jsonl`, `*.csv`) считаются исходными логами и прикладываются
отдельно. Здесь фиксируются условия сцены, ground truth, метрики и выводы.

Baseline convention:

- VLM mode uses Qwen to reason over target text, Set-of-Mark image, SLAM map,
  context marks and memory.
- FLAT mode does not use Qwen. For fairness it may perform one deterministic
  non-semantic overview scan (`forward -> right -> left`) before `ExploreFrontier`
  when the target is absent in the initial view. Semantic corridor selection
  remains VLM-only.

## Metric definitions

### Success rate

Бинарная метрика на один прогон:

```text
success = 1, если робот достиг правильного целевого объекта и миссия завершилась
            без degraded/fallback и без операторского аварийного прерывания.
success = 0 иначе.
```

Для серии:

```text
success_rate = sum(success) / N
```

### Time

Основное время:

```text
mission_time_s = activity.mission_end.stamp - activity.mission_start.stamp
```

Это включает perception, VLM latency, ожидания, Nav2/action execution и
служебную логику оркестратора.

Дополнительное время:

```text
motion_time_s = duration_s из step_result для ключевого action
```

Для прямого подъезда это длительность `DRIVE_TO_VISIBLE`.

Для сравнения скорости обработки/принятия решения между режимами:

```text
decision_time_s:
  VLM mode  = mean positive plan latency_ms / 1000
              deterministic initial_scan / target_lock / recovery calls excluded
  FLAT mode = mission_start -> first APPROACH state
```

Почему VLM не использует `mission_start -> first step_start`: в сценах поиска
первое действие может быть deterministic `initial_scan` с `latency_ms=0`, то
есть такая метрика искусственно делает сложные сцены быстрее простых. Для
диагностики это поле логируется как `time_to_first_action_s`, но в итоговой
таблице используется именно VLM planning latency.

### Progress rate

`progress_rate` оценивает не просто факт движения, а насколько далеко робот дошёл
по смысловой декомпозиции задачи.

Для простого сценария с видимой и достижимой целью:

```text
0.00 = нет осмысленного прогресса
0.25 = цель обнаружена/подтверждена в кадре
0.50 = выбрано правильное действие DRIVE_TO_VISIBLE без лишнего исследования
0.75 = подъезд к цели реально выполнен навигацией
1.00 = миссия завершена у правильного объекта
```

Для сценария, где цель видна, но начальная карта не позволяет сразу ехать в
финальную точку:

```text
0.00 = нет осмысленного прогресса
0.20 = цель обнаружена/подтверждена в стартовом кадре
0.40 = выбрана стратегия в сторону цели: recenter / DRIVE_TO_VISIBLE
0.60 = робот начал раскрывать карту/двигаться к цели bounded-подходами
0.80 = цель повторно локализована ближе или удержана через target_nav_lock
1.00 = миссия завершена у правильного объекта
```

Для сцен поиска, где цель изначально не видна, дополнительно будут учитываться:

```text
initial_scan выполнен
выбран правильный коридор по ground truth
робот начал исследовать выбранный коридор
цель найдена/подтверждена
цель достигнута
```

## Experiment section 1: target visible at start

Раздел проверяет случаи, где целевой объект уже присутствует в RGB-кадре при
старте миссии. Внутри раздела две сложности:

- `1.1`: объект виден и достижим по карте из стартовой точки.
- `1.2`: объект виден, но не достижим из стартовой точки из-за неполной/занятой
  карты; робот должен двигаться в сторону цели и уточнять карту, не теряя
  последнюю уверенную локализацию.

### 1.1 Scene 1: visible reachable drawer cabinet

#### Scene description

```text
experiment_label: vlm_scene_1
logger_run_id: scene_1
target: drawer cabinet / тумба
date: 2026-07-31
repeats: 5
```

Ground truth:

- Целевой объект находится прямо перед роботом.
- Цель видна из стартовой позиции.
- Цель достижима по карте из стартовой точки.
- Ожидаемое поведение: сразу `DRIVE_TO_VISIBLE`, без `initial_scan`,
  `semantic_explore`, `target_approach_blocked` или fallback.

Raw logs:

```text
~/ros2_ws/experiment_logs/vlm_missions/vlm_activity_scene_1.jsonl
~/ros2_ws/experiment_logs/vlm_missions/vlm_steps_scene_1.csv
```

Log sanity check:

- Найдено 5 реальных `mission_start -> mission_end` блоков.
- Replay-дубли после рестарта logger-а не попали в эти 5 прогонов.
- Для всех событий `logger_rx_stamp` почти совпадает с `activity.stamp`
  (разница около 0.000-0.002 s).

#### Per-run results

| Run | Start time | Detections | Selected target | VLM decision | Nav/approach | Steps | Mission time, s | Motion time, s | Success | Progress |
| --- | --- | ---: | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 02:43:22 | 1 | mark=1, conf=0.615, 2.70 m | `DRIVE_TO_VISIBLE mark=1` | direct, `bounded=false`, `blocked=false`, final=0.58 m | 1 | 14.75 | 10.55 | 1 | 1.00 |
| 2 | 02:52:36 | 1 | mark=1, conf=0.663, 2.66 m | `DRIVE_TO_VISIBLE mark=1` | direct, `bounded=false`, `blocked=false`, final=0.58 m | 1 | 13.31 | 10.30 | 1 | 1.00 |
| 3 | 02:54:56 | 2 | mark=1, conf=0.693, 2.68 m | `DRIVE_TO_VISIBLE mark=1` | direct, `bounded=false`, `blocked=false`, final=0.58 m | 1 | 13.66 | 10.62 | 1 | 1.00 |
| 4 | 02:56:22 | 1 | mark=1, conf=0.614, 2.77 m | `DRIVE_TO_VISIBLE mark=1` | direct, `bounded=false`, `blocked=false`, final=0.58 m | 1 | 13.57 | 11.16 | 1 | 1.00 |
| 5 | 02:57:46 | 2 | mark=2, conf=0.670, 2.75 m | `DRIVE_TO_VISIBLE mark=2` | direct, `bounded=false`, `blocked=false`, final=0.58 m | 1 | 14.10 | 11.11 | 1 | 1.00 |

#### Aggregate metrics

```text
N = 5
success_rate = 5/5 = 1.00
mean_progress_rate = 1.00
degraded_rate = 0/5 = 0.00
direct_approach_rate = 5/5 = 1.00
unnecessary_exploration_rate = 0/5 = 0.00
blocked_approach_rate = 0/5 = 0.00
mean_steps_per_run = 1.00
```

Timing:

```text
mission_time_s:
  mean = 13.88
  min = 13.31
  max = 14.75
  std = 0.50

motion_time_s:
  mean = 10.75
  min = 10.30
  max = 11.16
  std = 0.33

VLM plan latency_ms:
  mean = 1176.42
  min = 968.50
  max = 1802.10

decision_time_s_for_table = 1.18
```

Detection/geometry:

```text
selected_target_confidence:
  mean = 0.651
  min = 0.614
  max = 0.693

selected_target_distance_m:
  mean = 2.71
  min = 2.66
  max = 2.77

final_approach_distance_m:
  all runs = 0.58
```

#### Interpretation

Во всех пяти повторах VLM выполнила ожидаемую декомпозицию для простой сцены:

```text
видим цель -> выбираем DRIVE_TO_VISIBLE -> Nav2 напрямую доезжает -> auto_done
```

Не было лишнего поиска, начального сканирования, recovery или fallback. Это
хороший baseline для дальнейших сцен: если цель видна и достижима с начальной
позиции, пайплайн стабильно выбирает прямой подъезд и завершает миссию.

Особенность run 5: было две детекции `drawer cabinet`; VLM выбрала `mark=2`,
который был ближе (`2.75 m`) при сопоставимой уверенности. По ground truth это
считается корректным, если оба mark-а относятся к той же целевой тумбе/её части.

#### Result

```text
scene_1_result: PASS
main_claim: visible reachable target is handled reliably.
```

### 1.2 Scene 2: visible but initially unreachable office chair

#### Scene description

```text
experiment_label: vlm_scene_2
logger_run_id: scene_2
target: office chair / офисный стул
date: 2026-07-31
repeats: 5
```

Ground truth:

- Целевой объект виден из стартовой позиции.
- Объект находится в левой части кадра и далеко от робота.
- Финальная точка у объекта не должна считаться сразу надёжно достижимой по
  стартовой карте: карту нужно раскрывать по направлению к цели.
- Ожидаемое поведение: зафиксировать уверенную детекцию, при необходимости
  сделать небольшой `TURN` для центрирования, затем ехать к цели через
  `DRIVE_TO_VISIBLE` / `DRIVE_TO_LOCKED_TARGET`, не падая в общий
  context-search.

Raw logs:

```text
~/ros2_ws/experiment_logs/vlm_missions/vlm_activity_scene_2.jsonl
~/ros2_ws/experiment_logs/vlm_missions/vlm_steps_scene_2.csv
```

Log sanity check:

- Найдено 5 валидных `mission_start -> mission_end` блоков.
- Все 5 прогонов завершились `auto_done` и `degraded=False`.
- Отладочный/проблемный второй прогон со старой логикой был удалён из raw logs
  перед расчётом метрик; он зафиксирован отдельно в журнале проб и ошибок.

#### Per-run results

| Run | Start time | First confirmed target | Navigation pattern | Last target observation | Steps | Mission time, s | Motion time, s | Success | Progress |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 03:23:28 | `office chair`, conf=0.696, 5.68 m | recenter turn=1; bounded=5; lock=2 | 0.86 m | 7 | 45.48 | 32.49 | 1 | 1.00 |
| 2 | 03:42:09 | `office chair`, conf=0.778, 5.68 m | recenter turn=1; bounded=5; lock=1 | 1.04 m | 7 | 57.80 | 44.27 | 1 | 1.00 |
| 3 | 03:45:47 | `office chair`, conf=0.723, 5.78 m | recenter turn=1; bounded=4; lock=3; one Nav2 failed action recovered | 2.25 m | 7 | 144.78 | 132.49 | 1 | 1.00 |
| 4 | 03:52:51 | `office chair`, conf=0.798, 5.68 m | bounded=5; lock=3 | 1.46 m | 6 | 50.25 | 42.94 | 1 | 1.00 |
| 5 | 03:55:11 | `office chair`, conf=0.728, 5.68 m | recenter turn=1; bounded=5; lock=4; blocked-recovery forward=2 | 1.05 m | 10 | 88.98 | 72.99 | 1 | 1.00 |

#### Aggregate metrics

```text
N = 5
success_rate = 5/5 = 1.00
mean_progress_rate = 1.00
degraded_rate = 0/5 = 0.00
auto_done_rate = 5/5 = 1.00
```

Behavioral counters:

```text
bounded_approach_count = 24 total, mean = 4.8 per run
target_nav_lock_count = 13 total, mean = 2.6 per run
recenter_turn_runs = 4/5
blocked_recovery_forward_runs = 1/5
Nav2 failed action recovered = 2 runs
steps_per_run = [7, 7, 7, 6, 10], mean = 7.40
```

Timing:

```text
mission_time_s:
  mean = 77.46
  min = 45.48
  max = 144.78
  std = 36.92

motion_time_s:
  mean = 65.04
  min = 32.49
  max = 132.49
  std = 36.31

VLM plan latency_ms:
  mean = 1141.96
  min = 977.10
  max = 1482.80
  calls = 9

decision_time_s_for_table = 1.14
```

Detection/geometry:

```text
first_target_confidence:
  mean = 0.745
  min = 0.696
  max = 0.798

first_target_distance_m:
  mean = 5.70
  min = 5.68
  max = 5.78

last_seen_target_distance_m:
  mean = 1.33
  min = 0.86
  max = 2.25
```

#### Interpretation

Во всех пяти повторах робот видел целевой `office chair` уже в начале миссии,
но не мог просто один раз построить финальную навигацию к стулу: цель была
далеко, часто на краю кадра, а карта впереди раскрывалась постепенно. Поэтому
успешная декомпозиция выглядела так:

```text
видим цель -> при необходимости центрируем кадр -> едем bounded-подходами ->
если цель пропала из кадра, продолжаем к target_nav_lock -> при повторной
детекции обновляем locked point -> завершаем у целевого объекта
```

Ключевой результат `scene_2`: исправленная логика не забывает уверенно
обнаруженную цель после временной потери в кадре. Вместо возврата в общий
`semantic_explore` она использует сохранённую координату `target_nav_lock` и
продолжает движение в сторону объекта.

Главный остаточный риск - время и плавность. Run 3 занял 144.78 s из-за одного
долгого failed-действия Nav2, а run 5 потребовал blocked-recovery. Это не
сломало успех, но показывает, что для дальних видимых целей метрика времени
чувствительна к качеству локальной карты, costmap и контроллера.

#### Result

```text
scene_2_result: PASS
main_claim: visible but initially unreachable target is reached reliably after
            bounded map-expanding approach and target_nav_lock recovery.
```

## Experiment section 2: target not visible at start

Раздел проверяет случаи, где целевой объект отсутствует в стартовом RGB-кадре.
Пайплайн должен не крутиться на месте бесконечно, а выполнить короткий
структурированный обзор, выбрать полезное направление исследования и начать
активно раскрывать карту.

### 2.1 Scene 3: hidden office chair found by right scan

#### Scene description

```text
experiment_label: vlm_scene_3
logger_run_id: scene_3
target: office chair / офисный стул
date: 2026-07-31
repeats: 5
```

Ground truth:

- Целевого стула нет в стартовом ракурсе.
- Стул появляется в кадре после поворота вправо примерно на 90 градусов.
- Дальнего corridor exploration не требуется: после правого initial-scan
  ожидается немедленный переход в `target_approach`.
- Ожидаемое поведение: `TURN -1.57rad`, затем `DRIVE_TO_VISIBLE` /
  `DRIVE_TO_LOCKED_TARGET` до целевого стула.

Raw logs:

```text
~/ros2_ws/experiment_logs/vlm_missions/vlm_activity_scene_3.jsonl
~/ros2_ws/experiment_logs/vlm_missions/vlm_steps_scene_3.csv
```

Log sanity check:

- Найдено 5 реальных блоков `mission_start -> mission_end`.
- Все 5 прогонов завершились `auto_done`.
- Все 5 прогонов завершились с `degraded=False`.
- Во всех 5 прогонах первая осмысленная команда одинаковая:
  `initial_scan -> TURN -1.57rad`.

#### Per-run results

| Run | Start time | Detection after right scan | Approach pattern | Steps | Mission time, s | Action time, s | Success | Progress |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 06:19:49 | step 1: `office chair`, conf=0.907, 2.09 m | `DRIVE_TO_VISIBLE`, `DRIVE_TO_VISIBLE` | 3 | 21.12 | 13.37 | 1 | 1.00 |
| 2 | 06:23:11 | step 1: `office chair`, conf=0.914, 2.09 m | `DRIVE_TO_VISIBLE`, `DRIVE_TO_VISIBLE`, `DRIVE_TO_LOCKED_TARGET` | 4 | 38.77 | 31.07 | 1 | 1.00 |
| 3 | 06:25:22 | step 1: `office chair`, conf=0.877, 2.12 m | `DRIVE_TO_VISIBLE`, `DRIVE_TO_VISIBLE`, `DRIVE_TO_LOCKED_TARGET` | 4 | 31.27 | 23.11 | 1 | 1.00 |
| 4 | 06:41:30 | step 1: `office chair`, conf=0.911, 2.06 m | `DRIVE_TO_VISIBLE`, `DRIVE_TO_LOCKED_TARGET`, `DRIVE_TO_LOCKED_TARGET` | 4 | 22.05 | 15.18 | 1 | 1.00 |
| 5 | 06:42:57 | step 1: `office chair`, conf=0.910, 2.06 m | `DRIVE_TO_VISIBLE`, `DRIVE_TO_VISIBLE`, `DRIVE_TO_VISIBLE` | 4 | 24.34 | 15.78 | 1 | 1.00 |

#### Aggregate metrics

```text
N = 5
success_rate = 5/5 = 1.00
mean_progress_rate = 1.00
degraded_rate = 0/5 = 0.00
auto_done_rate = 5/5 = 1.00
initial_scan_completion_rate = 5/5 = 1.00
right_scan_target_found_rate = 5/5 = 1.00
unnecessary_semantic_explore_rate = 0/5 = 0.00
```

Behavioral counters:

```text
steps_per_run = [3, 4, 4, 4, 4]
mean_actions_per_run = 3.80

action_counts_total:
  TURN = 5
  DRIVE_TO_VISIBLE = 10
  DRIVE_TO_LOCKED_TARGET = 4

role_counts_total:
  initial_scan = 5
  target_approach = 14
  semantic_explore = 0
```

Timing:

```text
mission_time_s:
  mean = 27.51
  min = 21.12
  max = 38.77
  std = 6.66

action_execution_time_s:
  mean = 19.70
  min = 13.37
  max = 31.07
  std = 6.58

VLM plan latency_ms:
  mean = 1161.26
  median = 1170.50
  min = 1102.50
  max = 1227.10
  calls = 5

decision_time_s_for_table = 1.16
```

Detection/geometry:

```text
first_target_confidence:
  mean = 0.904
  min = 0.877
  max = 0.914

first_target_distance_m:
  mean = 2.08
  min = 2.06
  max = 2.12
```

#### Interpretation

Во всех пяти повторах VLM-пайплайн выполнил ожидаемую короткую декомпозицию:

```text
цели нет спереди -> initial_scan поворот вправо -> цель появилась в кадре ->
немедленный target_approach -> auto_done
```

Это важный промежуточный случай между `scene_1/scene_2` и более сложной
`scene_4`: объект не виден в стартовом кадре, но обнаруживается простым
структурированным обзором без настоящего коридорного исследования. Пайплайн не
уходит в лишний `semantic_explore`, не пытается подъезжать к контекстной мебели
и стабильно переключается в режим достижения цели сразу после уверенной
детекции `office chair`.

Run 2 был самым долгим (`38.77 s`) из-за длительного первого
`DRIVE_TO_VISIBLE` (`20.79 s`), но логически поведение осталось корректным:
цель была обнаружена после правого поворота и достигнута без fallback.

#### Result

```text
scene_3_result: PASS
main_claim: hidden target that becomes visible after one right scan is handled
            reliably without unnecessary corridor exploration.
```

### 2.2 Scene 4: hidden office chair in left corridor

#### Scene description

```text
experiment_label: vlm_scene_4
logger_run_id: scene_4
target: office chair / офисный стул
date: 2026-07-31
repeats: 5
```

Ground truth:

- Целевого стула нет в стартовом ракурсе.
- Стул находится в левом коридоре/левой исследуемой области.
- Простого прямого `DRIVE_TO_VISIBLE` из стартовой позиции быть не должно:
  сначала нужно осмотреть сцену и исследовать коридор.
- Ожидаемое поведение: `initial_scan` вправо/влево, затем движение в
  релевантный свободный коридор, обнаружение `office chair`, фиксация
  `target_nav_lock` и финальный подъезд.

Raw logs:

```text
~/ros2_ws/experiment_logs/vlm_missions/vlm_activity_scene_4.jsonl
~/ros2_ws/experiment_logs/vlm_missions/vlm_steps_scene_4.csv
```

Log sanity check:

- Найдено 5 реальных блоков `mission_start -> mission_end`.
- `mission_id` и `mission_index` повторяются из-за перезапуска logger-а, поэтому
  прогоны группируются только по порядку `mission_start -> mission_end`.
- Все 5 прогонов завершились с `degraded=False`.
- `auto_done` есть в 4 из 5 прогонов.

#### Progress scale for Scene 4

Для этой сцены используется детализированная шкала поиска:

```text
0.00 = нет осмысленного прогресса
0.20 = выполнен initial_scan
0.40 = выбран/начат правильный коридор исследования
0.60 = робот реально продвинулся по коридору
0.75 = целевой объект найден только как 2D/unknown-depth кандидат
0.85 = целевой объект локализован в 3D или создан target_nav_lock
1.00 = миссия завершена у правильного объекта
```

#### Per-run results

| Run | Start time | Key behavior | First target observation | Final target observation | Steps | Mission time, s | Action time, s | Success | Progress |
| --- | --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| 1 | 04:41:05 | initial scan -> left corridor exploration -> target approach | step 7: `office chair`, conf=0.798, 4.24 m | step 8: conf=0.660, 1.32 m | 9 | 65.37 | 40.20 | 1 | 1.00 |
| 2 | 05:20:52 | initial scan -> target seen without depth -> lost target -> repeated failed forward probes until max steps | step 3: `office chair`, conf=0.659, depth unknown | step 3: conf=0.659, depth unknown | 40 | 165.22 | 30.75 | 0 | 0.75 |
| 3 | 05:27:37 | corridor exploration -> target lock -> blocked recovery -> final approach | step 6: `office chair`, conf=0.605, 2.17 m | step 16: conf=0.629, 0.41 m | 17 | 75.00 | 44.34 | 1 | 1.00 |
| 4 | 05:31:27 | corridor exploration -> target lock -> blocked recovery -> locked target approach | step 5: `office chair`, conf=0.661, 2.52 m | step 5: conf=0.661, 2.52 m | 10 | 47.68 | 28.59 | 1 | 1.00 |
| 5 | 05:40:11 | longer semantic exploration -> target re-detection -> blocked recovery -> final approach | step 19: `office chair`, conf=0.755, depth unknown | step 25: conf=0.919, 0.99 m | 26 | 159.88 | 54.56 | 1 | 1.00 |

#### Aggregate metrics

```text
N = 5
success_rate = 4/5 = 0.80
mean_progress_rate = 0.95
degraded_rate = 0/5 = 0.00
auto_done_rate = 4/5 = 0.80
initial_scan_completion_rate = 5/5 = 1.00
target_found_rate = 5/5 = 1.00
target_localized_or_locked_rate = 4/5 = 0.80
```

Behavioral counters:

```text
steps_per_run = [9, 40, 17, 10, 26]
mean_actions_per_run = 20.40
mean_actions_per_successful_run = 15.50

action_counts_total:
  TURN = 25
  DRIVE_FORWARD = 57
  DRIVE_TO_VISIBLE = 10
  DRIVE_TO_LOCKED_TARGET = 5
  DETECT_ALL = 5

target_nav_lock_count = 15 total, mean = 3.00 per run
target_approach_blocked_recovery_count = 8 total, mean = 1.60 per run
failed_step_results = 30 total
```

Timing:

```text
mission_time_s, all runs:
  mean = 102.63
  min = 47.68
  max = 165.22
  std = 49.73

mission_time_s, successful runs only:
  mean = 86.98
  min = 47.68
  max = 159.88
  std = 43.21

action_execution_time_s, all runs:
  mean = 39.69
  min = 28.59
  max = 54.56
  std = 9.45

VLM plan latency_ms, deterministic initial_scan calls excluded:
  mean = 2032.85
  median = 1595.40
  min = 1044.00
  max = 31632.00
  p90 = 1877.00
  calls = 68

decision_time_s_for_table = 2.03
```

#### Interpretation

В четырёх из пяти повторов пайплайн успешно решил более сложную задачу:

```text
цели нет в стартовом кадре -> initial_scan -> исследование коридора ->
обнаружение office chair -> target_nav_lock / blocked recovery при необходимости
-> финальный DRIVE_TO_VISIBLE / DRIVE_TO_LOCKED_TARGET -> auto_done
```

Сильная сторона серии: `initial_scan` стабильно выполняется во всех пяти
прогонах, а целевой объект обнаруживается в каждом повторе. Это подтверждает
главную идею VLM-режима для скрытой цели: робот не просто крутится случайно, а
использует обзор сцены и коридорное исследование, чтобы довести цель до
детектора.

Слабое место серии - run 2. Там стул был найден как `office chair` с
`conf=0.659`, но без глубины (`depth unknown`). После короткого
`target_probe` цель была потеряна, `target_nav_lock` не появился, и система
перешла в неудачный цикл `semantic_explore`: много `DRIVE_FORWARD`, 26
`failed` step-result и завершение по `max_steps=40` без `auto_done`. Поэтому
run 2 получает высокий частичный progress (`0.75`), но `success=0`.

Run 5 показывает второй остаточный риск: успешный результат возможен даже после
длинного исследования, но цена высокая - 26 действий и 159.88 s. В этом прогоне
VLM/детектор долго удерживались в semantic exploration, затем цель была
переобнаружена, один `DRIVE_TO_VISIBLE` был заблокирован, после чего
`target_approach_blocked_recovery` сделал два коротких forward-probe и система
успешно обновила `target_nav_lock`.

#### Result

```text
scene_4_result: PARTIAL_PASS
main_claim: hidden target search works in 4/5 repeats; the remaining failure
            mode is target seen without depth, then lost before target_nav_lock.
next_fix_focus: keep a pending 2D target lock for unknown-depth target probes
                and prevent repeated failed forward probes near blockers.
```
