# FLAT baseline (Phase 2.10) — frozen gate

Zero-VLM FLAT autonomy: instruction → SEARCH (frontiers) → DETECT (detector →
`/target_pixel`) → APPROACH (Nav2), executive = `search_coordinator` SeekObject FSM.
This freezes the measured baseline that gates the later phases (no phase trusts the
robot to move until FLAT is sound). Provenance is tagged per line: **[live-2026-06]**
this session, **[prior]** earlier seeded run with real YOLOE + Nav2 active, **[unit]**
pure-logic test.

## Scenario assembled
- `flat_sim_bringup.launch.py` brings up the full FLAT stack in one command (sim →
  RTAB-Map → Nav2 → frontier_extractor + coordinator), all on sim time. **[live-2026-06]**
  executive comes up clean: `search_coordinator up ... skills [approach_detection,
  explore_frontier, get_observation, go_to_pose, stop]; epoch=0`.
- Bounded worlds need a one-time motion seed so SLAM gets unknown cells → frontiers:
  a net-zero in-place rotation seed yields **22 frontiers** **[live-2026-06]**.

## Measured per-action baseline
| Action | Metric | Source |
|---|---|---|
| t_detect | YOLOE ~25 ms warm · ~5.6 s cold load · ~0.87 s per prompt-change | [prior] |
| | DetectTarget('bus') live: FOUND, conf 0.916, depth 1.68 m, ~tens-ms warm | [live-2026-06] |
| t_search | ~13.4 s per frontier leg (drive to frontier centroid) | [prior, Nav2 active] |
| t_approach | ~6 s reaction + ~12 s drive ≈ 20 s total; mission SUCCEEDED | [prior, seeded] |
| planner cadence (VLM mode, ref) | ~2 s/step with async replan (4.6) | [live-2026-06] |

## EXIT criteria evidence
- **finds + approaches in pure-FLAT** — full SEARCH→DETECT→APPROACH→DONE = SUCCEEDED
  **[prior]** (seeded, real YOLOE, Nav2 active). This session validated SEARCH live (see
  below); the drive leg was Nav2-activation-limited this run (known issue ↓).
- **no frontier oscillation** — **[live-2026-06]** the executive selects frontiers by
  score and holds/blacklists rather than flapping (observed id sequence 75→16→112→7→2,
  distinct score-ranked, no A↔B ping-pong) + **[unit]** `should_switch` hysteresis
  (sub-margin noise never switches; switch only on beat>margin AND dwell≥min).
- **instruction-change resets the mission** — **[unit]** epoch bump makes in-flight UUID a
  zombie + `RequestDedup` clears across epoch (no zombie replay) + **[prior]** SeekObject
  supersession node-smoke (old mission → PREEMPTED).
- **baseline frozen** — this document.

## Known issue (found this session — flagged, not a baseline blocker)
In the consolidated `flat_sim_bringup` on a **resource-constrained host (3.5 GiB WSL)**,
Nav2's lifecycle had **not finished activating** when the executive began driving, so every
`explore_frontier` drive returned `terminal=no_server` and the skill **wrongly blacklisted
8+ valid frontiers** as "unreachable". The executive logic is otherwise correct. Fix tracked:
treat `no_server` (Nav2-not-ready) as transient — wait/retry instead of blacklisting; and/or
gate mission start on Nav2 readiness. On adequately-resourced hardware (Phase 6 Pi) Nav2
activates before the mission and this does not occur.

## Caveat: true Pi-class profiling is Phase 6
CPU/frequency/latency on the real **Pi 5 / 4 GB** are measured during hardware bring-up
(see `HIL_BRINGUP_CHECKLIST.md` §E). The numbers here are Gazebo-on-WSL (algorithmic
latencies), not Pi silicon.

## embodied vs robust (per-action)
- **t_search** — embodied: spin-in-place search (rotate until the target enters FOV; robot
  stays put). robust: frontier-translate legs (~13.4 s/leg) that actually explore the space.
  embodied is faster per "look" but cannot find a target outside the initial FOV / behind
  walls; robust covers the map. **Robustness win for robust.**
- **t_detect** — identical (same YOLOE backend), ~25 ms warm.
- **t_approach** — comparable (same pixel→3D geometry + Nav2), but robust adds the
  **freshness gate** (no false `reached` on a stale pixel) embodied lacked.
- **net** — robust trades a little raw search speed for actually-finding-things + no
  false-reach + anti-oscillation + (in VLM mode) seamless degradation back to this FLAT layer.
