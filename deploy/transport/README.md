# Transport deployment — ROADMAP Phase 1.1

Single `rmw_zenoh` router on the edge, multicast **off**, 12 MB socket buffers on
every host, with a Fast DDS (LARGE_DATA + Discovery Server) fallback. This is the
cross-host transport substrate for the Pi↔edge link; it replaces default DDS
multicast discovery, which floods shared Wi-Fi and is fragile across subnets.

## What goes where

| Artifact | Edge host | Pi (and any node host) |
|---|---|---|
| `zenoh_router_config.json5` | ✅ `/etc/zenoh/` | — |
| `rmw-zenoh-router.service` | ✅ enable | — |
| `zenoh_session_config.json5` | ✅ `/etc/zenoh/` | ✅ `/etc/zenoh/` |
| `transport_env.sh` | ✅ source | ✅ source |
| `99-ros2-socket-buffers.conf` | ✅ `/etc/sysctl.d/` | ✅ `/etc/sysctl.d/` |
| `fastdds-discovery-server.service` | fallback only | — |

All ROS nodes (router included) must use the **same** `RMW_IMPLEMENTATION`.

## Bring-up (primary: zenoh)

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

## Fallback (Fast DDS)

If `rmw_zenoh` is unavailable, switch every host to the Fast DDS block in
`transport_env.sh` (`rmw_fastrtps_cpp` + `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`
+ `ROS_DISCOVERY_SERVER`) and run `fastdds-discovery-server.service` on the edge.

## Smoke test (single host)

`smoke_test_zenoh.sh` starts the router with this config, then a publisher and a
subscriber as two separate sessions (multicast off, connect to localhost router),
and confirms messages flow **through the router** — i.e. discovery works without
multicast. Run it in WSL:

```bash
bash deploy/transport/smoke_test_zenoh.sh
```

## Verified vs. pending

- **Verified (single host, WSL):** configs load; router starts; with multicast
  off + gossip, pub→sub delivery works only through the router. Schema matches
  the installed `rmw_zenoh_cpp` defaults.
- **Pending (needs 2 hosts — Pi + edge, ROADMAP Phase 1.2 / 6):** measured
  cross-host jitter within the 0.2 s (TF) / 0.35 s (depth-match) / 1.5 s
  (pixel-age) budgets; this requires `chrony` (Phase 1.2) and real Wi-Fi.
