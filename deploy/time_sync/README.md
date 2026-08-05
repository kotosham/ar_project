# Time Synchronization (chrony) - ROADMAP Phase 1.2 [FMEA]

The Pi and edge host must keep their clocks synchronized well within the matching
windows used by the stack. Otherwise TF queries, depth/color association, and
pixel-age filtering can fail silently. chrony keeps the **relative** Pi-edge
offset tiny by making the edge host the single time master for the whole fleet.

| Window | Source | Budget |
|---|---|---|
| TF `transform_tolerance` | `nav2_params.yaml`, EKF | **0.2 s** |
| depth/color matching | RealSense / RTAB-Map | **0.35 s** |
| pixel age (detections) | `ApproachDetection` (Phase 3.4) | **1.5 s** |

Target: offset and RMS **<= 0.02 s** (10% of the tightest 0.2 s window). On a
LAN with a local chrony server, sub-millisecond accuracy is typical, leaving a
large margin.

## Installation Targets

| Artifact | Host | Path |
|---|---|---|
| `chrony-edge.conf` | edge | `/etc/chrony/chrony.conf` |
| `chrony-pi.conf` | Pi (edit `EDGE_HOST`) | `/etc/chrony/chrony.conf` |
| `check_offset.sh` | both | run after synchronization |

```bash
# edge
sudo cp chrony-edge.conf /etc/chrony/chrony.conf && sudo systemctl restart chrony
# Pi (set EDGE_HOST to the edge IP first)
sudo cp chrony-pi.conf /etc/chrony/chrony.conf && sudo systemctl restart chrony
# proof (run on the Pi once synced)
bash check_offset.sh
```

`chronyc sources -v` should show the edge host as the selected source (`*`), and
`chronyc tracking` should report Last/RMS offset well below 0.02 s.

## Verified vs. Pending

- **Verified (single host, WSL):** both configurations parse without errors (`chronyd -p`).
- **Pending (requires two hosts: Pi + edge):** measured offset proof. WSL2 time is
  managed by the Windows host, so the real disciplining run and `check_offset.sh`
  PASS apply to the deployed Pi-edge pair with the Phase 1.1 zenoh link running.
  This is the Phase 1 EXIT gate for the jitter budget.

> NOTE: WSL2 synchronizes its clock with the Windows host through Hyper-V. Do
> **not** run a competing `chronyd` inside WSL during development; these
> configurations are intended for deployed Linux Pi/edge hosts.
