# Синхронизация времени (chrony) — ROADMAP Phase 1.2 [FMEA]

Pi и edge должны согласовывать время с запасом внутри окон сопоставления, на
которые опирается стек, иначе TF-запросы, сопоставление depth↔color и отсечение
по возрасту пикселей будут незаметно ломаться. chrony удерживает **относительное**
смещение Pi↔edge крошечным, делая edge единственным мастером времени для всего флота.

| Окно | Источник | Бюджет |
|---|---|---|
| TF `transform_tolerance` | `nav2_params.yaml`, EKF | **0.2 s** |
| сопоставление depth ↔ color | RealSense / RTAB-Map | **0.35 s** |
| возраст пикселей (детекции) | `ApproachDetection` (Phase 3.4) | **1.5 s** |

Цель: смещение и RMS **≤ 0.02 s** (10% от самого жёсткого окна 0.2 s). В LAN
с локальным сервером chrony обычно достигает субмиллисекундной точности, так что запас велик.

## Что куда

| Артефакт | Хост | Путь |
|---|---|---|
| `chrony-edge.conf` | edge | `/etc/chrony/chrony.conf` |
| `chrony-pi.conf` | Pi (правьте `EDGE_HOST`) | `/etc/chrony/chrony.conf` |
| `check_offset.sh` | оба | запуск после синхронизации |

```bash
# edge
sudo cp chrony-edge.conf /etc/chrony/chrony.conf && sudo systemctl restart chrony
# Pi (set EDGE_HOST to the edge IP first)
sudo cp chrony-pi.conf /etc/chrony/chrony.conf && sudo systemctl restart chrony
# proof (run on the Pi once synced)
bash check_offset.sh
```

`chronyc sources -v` должна показывать edge как выбранный источник (`*`), а
`chronyc tracking` — значение Last/RMS offset далеко ниже 0.02 s.

## Проверено vs. в ожидании

- **Проверено (один хост, WSL):** обе конфигурации разбираются без ошибок (`chronyd -p`).
- **В ожидании (нужны 2 хоста — Pi + edge):** фактическое доказательство смещения. Часы WSL2
  управляются хостом, поэтому реальный прогон дисциплинирования + PASS для `check_offset.sh`
  относятся к развёрнутой паре Pi+edge (с поднятой связью zenoh из Phase 1.1). Это и есть гейт
  для бюджета джиттера Phase 1 EXIT.

> NOTE: WSL2 синхронизирует свои часы с Windows-хостом через Hyper-V; **не** запускайте
> конкурирующий `chronyd` внутри WSL для разработки — эти конфигурации предназначены для
> развёрнутых Linux-хостов Pi/edge.
