# Transport Deployment - ROADMAP Phase 1.1

A single `rmw_zenoh` router runs on the edge host, multicast is **disabled**,
each host gets 12 MB socket buffers, and Fast DDS (LARGE_DATA + Discovery Server)
is kept as a fallback. This is the inter-host transport layer for the Pi-edge
link; it replaces standard DDS multicast discovery, which overloads shared Wi-Fi
and behaves poorly across subnets.

## Installation Targets

| Artifact | Edge host | Pi and any host running nodes |
|---|---|---|
| `zenoh_router_config.json5` | yes, `/etc/zenoh/` | - |
| `rmw-zenoh-router.service` | yes, enable it | - |
| `zenoh_session_config.json5` | yes, `/etc/zenoh/` | yes, `/etc/zenoh/` |
| `transport_env.sh` | source it | source it |
| `99-ros2-socket-buffers.conf` | yes, `/etc/sysctl.d/` | yes, `/etc/sysctl.d/` |
| `fastdds-discovery-server.service` | fallback only | - |

All ROS nodes, including the router, must use the **same** `RMW_IMPLEMENTATION`.

## Startup (Primary Path: zenoh)

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

## Fallback Path (Fast DDS)

If `rmw_zenoh` is unavailable, switch every host to the Fast DDS block in
`transport_env.sh` (`rmw_fastrtps_cpp` + `FASTDDS_BUILTIN_TRANSPORTS=LARGE_DATA`
+ `ROS_DISCOVERY_SERVER`) and start `fastdds-discovery-server.service` on the
edge host.

## Smoke Test (Single Host)

`smoke_test_zenoh.sh` starts the router with this configuration, then runs a
publisher and subscriber as two separate sessions (multicast disabled, both
connected to the local router). It confirms that messages pass **through the
router**, meaning discovery works without multicast. Run it in WSL:

```bash
bash deploy/transport/smoke_test_zenoh.sh
```

## Verified and Pending

- **Verified (single host, WSL):** configs load, the router starts, and with
  multicast + gossip disabled pub/sub delivery works only through the router.
  The setup matches the installed `rmw_zenoh_cpp` defaults.
- **Pending (requires two hosts: Pi + edge, ROADMAP Phase 1.2 / 6):** measured
  inter-host jitter within the 0.2 s (TF), 0.35 s (depth-match), and 1.5 s
  (pixel-age) budgets. This requires `chrony` (Phase 1.2) and real Wi-Fi.
