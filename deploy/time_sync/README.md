# Time sync (chrony) — ROADMAP Phase 1.2 [FMEA]

The Pi and edge must agree on time well within the matching windows the stack
relies on, or TF lookups, depth↔color matching, and pixel-age gating silently
break. chrony keeps the **relative** Pi↔edge offset tiny by making the edge the
single fleet time master.

| Window | Source | Budget |
|---|---|---|
| TF `transform_tolerance` | `nav2_params.yaml`, EKF | **0.2 s** |
| depth ↔ color match | RealSense / RTAB-Map | **0.35 s** |
| pixel age (detections) | `ApproachDetection` (Phase 3.4) | **1.5 s** |

Target: offset and RMS **≤ 0.02 s** (10% of the tightest 0.2 s window). On a LAN
with a local server chrony normally reaches sub-millisecond, so the margin is large.

## What goes where

| Artifact | Host | Path |
|---|---|---|
| `chrony-edge.conf` | edge | `/etc/chrony/chrony.conf` |
| `chrony-pi.conf` | Pi (edit `EDGE_HOST`) | `/etc/chrony/chrony.conf` |
| `check_offset.sh` | both | run after sync |

```bash
# edge
sudo cp chrony-edge.conf /etc/chrony/chrony.conf && sudo systemctl restart chrony
# Pi (set EDGE_HOST to the edge IP first)
sudo cp chrony-pi.conf /etc/chrony/chrony.conf && sudo systemctl restart chrony
# proof (run on the Pi once synced)
bash check_offset.sh
```

`chronyc sources -v` should list the edge as the selected source (`*`), and
`chronyc tracking` should show a Last/RMS offset far below 0.02 s.

## Verified vs. pending

- **Verified (single host, WSL):** both configs parse cleanly (`chronyd -p`).
- **Pending (needs 2 hosts — Pi + edge):** the actual offset proof. WSL2's clock
  is host-managed, so a real disciplining run + `check_offset.sh` PASS belongs on
  the deployed Pi+edge pair (with the Phase 1.1 zenoh link up). This is the gate
  for the Phase 1 EXIT jitter budget.

> NOTE: WSL2 syncs its clock from the Windows host via Hyper-V; do **not** run a
> competing `chronyd` inside WSL for development — these configs are for the
> deployed Pi/edge Linux hosts.
