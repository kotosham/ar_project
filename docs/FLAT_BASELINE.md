# FLAT Baseline (Phase 2.10) - Frozen Gate

FLAT autonomy without VLM (zero-VLM) follows:

```text
instruction -> SEARCH (frontiers) -> DETECT (detector -> /target_pixel) -> APPROACH (Nav2)
```

The executive is the `search_coordinator` SeekObject FSM. This document records
the measured baseline used as a gate for later phases: no phase may trust the
robot with autonomous motion until FLAT is considered reliable.

Source tags:

- **[live-2026-06]**: current session
- **[prior]**: earlier seeded run with real YOLOE + Nav2
- **[unit]**: logic-only test

## Collected Scenario

- `flat_sim_bringup.launch.py` starts the whole FLAT stack in one command:
  simulation -> RTAB-Map -> Nav2 -> `frontier_extractor` + coordinator, all on
  simulation time. **[live-2026-06]**
- Executive starts cleanly with skills:
  `approach_detection`, `explore_frontier`, `get_observation`, `go_to_pose`,
  `stop`; epoch 0.
- Bounded worlds need one motion seed so SLAM produces unknown cells and
  frontiers. A zero-net rotation seed produced **22 frontiers**. **[live-2026-06]**

## Measured Action Baseline

| Action | Metric | Source |
|---|---|---|
| `t_detect` | YOLOE ~25 ms warm, ~5.6 s cold load, ~0.87 s on prompt switch | [prior] |
| `t_detect` | `DetectTarget('bus')`: FOUND, conf 0.916, depth 1.68 m, tens of ms warm | [live-2026-06] |
| `t_search` | ~13.4 s for one frontier transfer to centroid | [prior, Nav2 active] |
| `t_approach` | ~6 s reaction + ~12 s drive, about 20 s total; mission SUCCEEDED | [prior, seeded] |
| VLM planning tick, reference only | ~2 s/step with async replan | [live-2026-06] |

## EXIT Evidence

- **Find + approach in pure FLAT**: full SEARCH -> DETECT -> APPROACH -> DONE
  cycle succeeded in the seeded prior run with real YOLOE and active Nav2. In
  this session, SEARCH was validated live; motion was limited by a Nav2 activation
  issue described below.
- **No frontier oscillation**: executive selects frontiers by score, holds/blacklists
  them, and does not ping-pong. Unit hysteresis tests confirm no switch below
  margin and switch only when score beats margin and dwell time is satisfied.
- **Instruction change resets mission**: unit tests confirm epoch increment turns
  in-flight UUIDs into stale zombies and clears `RequestDedup`; prior smoke test
  confirms SeekObject preemption.
- **Baseline frozen**: this document is the gate record.

## Known Issue

In consolidated `flat_sim_bringup` on a resource-limited WSL host (3.5 GiB), the
Nav2 lifecycle did not finish activation before executive motion began. Each
`explore_frontier` transfer returned `terminal=no_server`, and the skill
incorrectly blacklisted several valid frontiers as unreachable. The executive
logic otherwise behaved correctly.

Fix direction: treat `no_server` as a temporary Nav2-not-ready failure and wait
or retry instead of blacklisting, and/or make mission start depend on Nav2
readiness. On hardware with enough resources, Nav2 should activate before the
mission.

## Note on Pi-Class Profiling

Real CPU/frequency/latency numbers for **Pi 5 / 4 GB** are measured during
hardware bringup (see `HIL_BRINGUP_CHECKLIST.md` section E). Numbers here are
Gazebo-on-WSL algorithmic timings, not Pi silicon timings.

## Embodied vs Robust

- **`t_search`**: embodied searches by rotating in place until the target enters
  the FOV. robust uses frontier transfers that move through space. A single
  embodied "look" is faster, but it cannot find targets outside the initial FOV
  or behind geometry. robust improves reliability.
- **`t_detect`**: same backend, about 25 ms warm.
- **`t_approach`**: comparable geometry and Nav2 stack, but robust adds a
  freshness gate to prevent false `reached` from stale pixels.
- **Summary**: robust trades some raw search speed for actual space coverage,
  stale-target protection, anti-oscillation, and seamless VLM-to-FLAT fallback.
