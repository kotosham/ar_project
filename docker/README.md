# Docker — поднятие стека на ПК

Контейнеризует уровни архитектуры (см. [../docs/architecture/SOLUTION_OVERVIEW.md](../docs/architecture/SOLUTION_OVERVIEW.md))
для запуска на обычном ПК — без ручной установки ROS 2, Gazebo и зависимостей.
Профили выбирают, что именно поднять.

| Профиль | Контейнеры | Что делает | Железо |
|---|---|---|---|
| `sim`  | `sim` | весь FLAT-стек в симуляции (Gazebo → RTAB-Map → Nav2 → executive), один контейнер | CPU |
| `edge` | `detector` + `orchestrator` | детектор YOLOE + VLM-оркестратор (edge-сторона) | NVIDIA GPU |
| `all`  | всё | sim + edge сразу | CPU + GPU (тяжело по RAM) |

> Реальный уровень **робота** (CAN/EPOS4/RealSense на Raspberry Pi) в Docker на ПК
> не воспроизводится — нужно физическое железо (см. [../docs/HIL_BRINGUP_CHECKLIST.md](../docs/HIL_BRINGUP_CHECKLIST.md)).
> На ПК его роль выполняет контейнер `sim`.

## Требования

- **Docker** с **Compose v2** и включённым **BuildKit** (по умолчанию в современном Docker —
  нужен для `Dockerfile.dockerignore`).
- Для профиля `edge` (GPU): драйвер NVIDIA + **nvidia-container-toolkit** на хосте
  (`--gpus`/`deploy.devices`). torch ставится из CUDA-индекса `cu124` (проверено на
  torch 2.6.0+cu124); рантайм CUDA приходит из колёс torch, отдельный CUDA-образ не нужен.

## Быстрый старт

```bash
cd ar_project/docker
cp .env.example .env                 # опционально: ROS_DOMAIN_ID / RMW

# Вариант A — весь FLAT в симуляции на ПК:
docker compose --profile sim build
docker compose --profile sim up      # headless (gui:=false), дисплей не нужен

# Вариант B — edge (детектор + оркестратор), нужен GPU:
docker compose --profile edge build
docker compose --profile edge up
```

Или через `make` (из `ar_project/docker/`): `make build`, `make sim`, `make edge`, `make down`.

## Триггер миссии (в контейнере `sim`)

После `up` поднимается чистый FLAT. В отдельном терминале:

```bash
# заполнить карту в ограниченном мире (даёт SLAM unknown-клетки → frontiers)
docker compose exec sim bash -lc \
  'ros2 topic pub -r 10 /diff_cont/cmd_vel_unstamped geometry_msgs/msg/Twist "{angular: {z: 0.6}}"'  # Ctrl-C через ~5 с

# запустить FLAT-миссию (без VLM)
docker compose exec sim bash -lc \
  "ros2 action send_goal /seek_object object_tracking_msgs/action/SeekObject \
   '{instruction: \"find bus\", request_id: \"m1\", mission_epoch: 0, allow_vlm: false}' --feedback"
```

Полные сценарии (DETECT/APPROACH, режим VLM, мониторинг) — в [../docs/RUNBOOK.md](../docs/RUNBOOK.md).

## Тома и секреты

- **Веса YOLOE** (~600 МБ) не пекутся в образ, а монтируются томом из
  `object_tracking/object_tracking/object_tracking/model_weights` в share-путь пакета
  (см. `volumes` сервиса `detector`). Положите туда `yoloe-11s-seg.pt` и `mobileclip_blt.ts`.
- **Креды VLM** берутся из `object_tracking/planner_orchestrator/vlm.env` через `env_file`
  (`required: false` — без него оркестратор стартует в mock-режиме). Файл `*.env`
  в git и в образ **не** попадает.

## Сеть и транспорт

Внутри одного `docker compose` все сервисы в общей сети; DDS-дискавери (`rmw_fastrtps_cpp`)
работает по умолчанию. Профиль `sim` самодостаточен (все узлы в одном контейнере — связь
внутрипроцессная). Боевой транспорт Pi↔edge по Wi-Fi — это `rmw_zenoh`
(см. [../deploy/transport/](../deploy/transport/)); в Docker на одном ПК он не требуется.

## Ограничения и заметки

- **RAM.** Полный стек (Gazebo + RTAB-Map + Nav2 + YOLOE) прожорлив. На машинах с ~4 ГБ
  доступной памяти поднимайте профили `sim` и `edge` **по отдельности**, не `all`.
- **Сборка `sim` — главный риск.** Пакет `ar_project` (ament_cmake) тянет тяжёлые
  зависимости. CAN-рантайм (`canopen*`) и реальная RealSense нужны только на роботе,
  поэтому в `Dockerfile` они в `--skip-keys` (симуляция использует gz_ros2_control и
  камеру Gazebo). Если `rosdep` всё равно споткнётся о недоступный ключ под вашу версию
  ROS — добавьте его в `--skip-keys`.
- **Без робота/симуляции** контейнеры `edge` поднимутся и будут ждать ROS-графа —
  полезны в паре с профилем `sim` или с реальным роботом.
