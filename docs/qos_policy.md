# Политика QoS и heartbeat для cross-link — ROADMAP Phase 1.3

Каждая ROS-точка обмена Pi↔edge берёт свой QoS из именованного профиля в
[`fleet_comms/qos.py`](../fleet_comms/fleet_comms/qos.py) — никогда не задаётся вручную.
Это единственный источник истины; от `fleet_comms` зависят
`search_coordinator` (Pi) и `planner_orchestrator` (edge).

## Профили

| Профиль | Reliability | Durability | Depth | Deadline | Liveliness (lease) | Назначение |
|---|---|---|---|---|---|---|
| `control_cmd` | RELIABLE | VOLATILE | 1 | 2.0 s | MANUAL_BY_TOPIC (3 s) | SeekObject goal, DetectTarget goal, PlanStep |
| `control_cmd_latched` | RELIABLE | TRANSIENT_LOCAL | 1 | — | AUTOMATIC | SeekObject result/status (переподключение оператора) |
| `liveliness_status(p)` | RELIABLE | VOLATILE | 1 | 1.5·p | MANUAL_BY_TOPIC (3·p) | Heartbeat + периодический health |
| `correction_lowrate` | RELIABLE | VOLATILE | 1 | 1.0 s | AUTOMATIC (3 s) | MapOdomCorrection (~1–2 Hz) |
| `detection_stream` | BEST_EFFORT | VOLATILE | 1 | 1.5 s | — | OFFER-сторона *периодического* потока детекций (publisher) |
| `detection_stream_nodeadline` | BEST_EFFORT | VOLATILE | 1 | — | — | /target_pixel **consumer** (спорадический поток; свежесть обеспечивается age-gate на уровне приложения) |
| `media_besteffort` | BEST_EFFORT | VOLATILE | 1 | — | — | только **сжатые** кадры/всплески |

Зачем deadline+liveliness: молчащий производитель должен становиться наблюдаемым в течение секунд.
Производитель и монитор `liveliness_status` ОБЯЗАНЫ использовать **один и тот же период**, чтобы
предлагаемый/запрашиваемый QoS оставались совместимыми — [`is_compatible()`](../fleet_comms/fleet_comms/qos.py)
кодирует правила DDS Request-vs-Offered, а модульный тест фиксирует их.

## Точка обмена cross-link → профиль

| Точка обмена | Тип | Производитель | Профиль | Статус |
|---|---|---|---|---|
| `/seek_object` | action | Pi exec | `control_cmd` goal / `control_cmd_latched` result+status | планируется (2.x) |
| `/detect_target` | action | edge | `control_cmd` goal / result `media_besteffort` для аннотированного кадра | планируется (3.2) |
| `/map_odom_correction` | topic | edge SLAM | `correction_lowrate` | планируется (2.6) |
| `/heartbeat` | topic | каждый производитель | `liveliness_status(0.5)` | **сейчас** |
| `PlanStep` | topic | edge planner | `control_cmd` (без фиксированного deadline) | планируется (4.x) |
| `Candidate[]` | sub-msg | edge | наследует от несущего action result | планируется (3.x) |
| `/target_pixel` | topic | edge tracker | publisher: BEST_EFFORT без deadline (спорадический); **consumer: `detection_stream_nodeadline`** — запрос deadline 1.5 s из `detection_stream` отбрасывает каждый сэмпл (must-fix #2, зафиксировано тестом `is_compatible`) | существует |
| `/target_prompt` | topic | Pi exec (`PromptBridge`) | `control_cmd_latched` — заменяет `reliable_prompt_sender`; подписка tracker должна стать TRANSIENT_LOCAL в 2.9 для повтора при позднем подключении | планируется (2.x) |
| GetObservation `result.view` | action payload | Pi | только CompressedImage; `media_besteffort` при ретрансляции | планируется (3.5) |

## Никакой raw depth / PointCloud2 по Wi-Fi (ROADMAP 1.4 / 3.5)

`media_besteffort` предназначен только для **сжатых** медиа. Raw depth и PointCloud2
никогда не должны пересекать линк.

- **LEAK (открыт, архитектурный — Phase 3.x):** `/tracker/aligned_depth_to_color/image_raw`
  — RAW несжатый aligned depth, публикуемый
  [`tracker_rgbd_bridge.py:56`](../ar_project/scripts/tracker_rgbd_bridge.py) с
  параметрами по умолчанию RELIABLE depth=10. Единственное настоящее нарушение «raw depth по Wi-Fi». Исправление —
  сжимать (`compressedDepth`) либо держать depth локально на edge и передавать только
  производную точку/Candidate — это часть переработки tracker→DetectTarget (Phase 3.2/3.5),
  а не настройка QoS.
- Сжатые медиа, которые легитимно пересекают линк, но сейчас используют параметры по умолчанию RELIABLE
  depth=10 (→ `media_besteffort` *если узел это переживёт*):
  `/tracker/color/image/compressed`, `/tracker/color/image_raw` (несмотря на имя — это CompressedImage).
  Тяжёлая mono8-маска `/target_mask` → заменяется на
  Candidate[]+CompressedImage в Phase 3.2.
- В исходниках нет ни одного publisher PointCloud2 (в Phase 1.4 costmap перенесли на
  локальный `/scan`).

## НЕ переназначайте профили этим (удалено в Phase 2.9)

`reliable_prompt_sender.py` и handshake из латч-мешанины (`/target_prompt`,
`/target_prompt_ack`, `/target_goal_locked`), а также реактивный путь `/cmd_vel`
edge-трекера. Работа над их QoS была бы выброшена впустую — `SeekObject`
(`control_cmd_latched`) является принципиальной заменой.

## Heartbeat (`fleet_comms/heartbeat.py`)

- **Производители** (`HeartbeatPublisher`) — edge SLAM (`slam`), детектор (`detector`),
  `planner_orchestrator`; Pi `search_coordinator` тоже эмитит heartbeat для симметрии. Один
  топик `/heartbeat`, различаемый по `node_name`, период **0.5 s** на весь флот.
  Заполняет `header.stamp` (синхронизация через chrony, Phase 1.2), `status`, `cpu_load`,
  `last_latency_ms` (питает circuit-breaker по p99, Phase 4.4), `mission_epoch`
  (биения с устаревшей epoch игнорируются, Phase 2.5).
- **Монитор** (`HeartbeatMonitor`, в `search_coordinator`) — отслеживает здоровье по каждому `node_name`
  из переданного status, события QoS deadline-missed / liveliness-lost
  и фолбэк по stale-timeout. В Phase 1.3 он только логирует / отдаёт
  `health_snapshot()`; обвязка FSM деградации VLM→FLAT относится к Phase 5.1.
