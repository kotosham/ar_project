# Развёртывание транспорта — ROADMAP Phase 1.1

Единственный роутер `rmw_zenoh` на edge-узле, multicast **выключен**, сокетные
буферы по 12 МБ на каждом хосте, с резервным вариантом на Fast DDS (LARGE_DATA +
Discovery Server). Это межхостовая транспортная подложка для канала Pi↔edge; она
заменяет стандартное обнаружение DDS через multicast, которое перегружает общую
сеть Wi-Fi и плохо работает между подсетями.

## Что куда устанавливается

| Артефакт | Edge-хост | Pi (и любой хост с узлами) |
|---|---|---|
| `zenoh_router_config.json5` | ✅ `/etc/zenoh/` | — |
| `rmw-zenoh-router.service` | ✅ включить | — |
| `zenoh_session_config.json5` | ✅ `/etc/zenoh/` | ✅ `/etc/zenoh/` |
| `transport_env.sh` | ✅ source | ✅ source |
| `99-ros2-socket-buffers.conf` | ✅ `/etc/sysctl.d/` | ✅ `/etc/sysctl.d/` |
| `fastdds-discovery-server.service` | только для fallback | — |

Все ROS-узлы (включая роутер) должны использовать **одну и ту же** `RMW_IMPLEMENTATION`.

## Запуск (основной вариант: zenoh)

```bash
# every host: OS socket buffers
sudo cp 99-ros2-socket-buffers.conf /etc/sysctl.d/ && sudo sysctl --system

# edge: install + start the single router
sudo install -d /etc/zenoh
sudo cp zenoh_router_config.json5 zenoh_session_config.json5 /etc/zenoh/
sudo cp rmw-zenoh-router.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now rmw-zenoh-router

# every host: point sessions at the edge router and source the env
#   edit zenoh_session_config.json5 (EDGE_HOST) OR set ZENOH_CONFIG_OVERRIDE
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source <repo>/deploy/transport/transport_env.sh
```

## Резервный вариант (Fast DDS)

Если `rmw_zenoh` недоступен, переключите каждый хост на блок Fast DDS в файле
`transport_env.sh` (`rmw_fastrtps_cpp` + `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`
+ `ROS_DISCOVERY_SERVER`) и запустите `fastdds-discovery-server.service` на edge-узле.

## Дымовой тест (один хост)

`smoke_test_zenoh.sh` запускает роутер с этой конфигурацией, затем publisher и
subscriber как две отдельные сессии (multicast выключен, подключение к
локальному роутеру) и подтверждает, что сообщения проходят **через роутер** —
то есть обнаружение работает без multicast. Запускайте его в WSL:

```bash
bash deploy/transport/smoke_test_zenoh.sh
```

## Проверено и в ожидании

- **Проверено (один хост, WSL):** конфигурации загружаются; роутер стартует; при
  выключенном multicast + gossip доставка pub→sub работает только через роутер.
  Схема соответствует стандартным значениям установленного `rmw_zenoh_cpp`.
- **В ожидании (нужны 2 хоста — Pi + edge, ROADMAP Phase 1.2 / 6):** измеренный
  межхостовой джиттер в пределах бюджетов 0.2 s (TF) / 0.35 s (depth-match) /
  1.5 s (pixel-age); для этого требуются `chrony` (Phase 1.2) и реальный Wi-Fi.
