# embodied-olympics-match-badminton-match-i01-standard-realtime

| | |
|---|---|
| Benchmark | Embodied Olympics - Match |
| Mode | standard-realtime (`eai-standard-realtime/1.0`) |
| Twin | `embodied-olympics-match-badminton-match-i01-privileged` |
| Simulator / robot | `mujoco` / `franka-panda` |
| Family / instance | `badminton_match` / seed 7 |
| Oracle | `partial` — screening round 16 (2026-10-13, revision 12, S = 20): our camera-only reference player beat level 1 in 17 of 20 seeds; on this instance at S = 20 it lost 10:12 (vanilla Harbor 0.23.0, 2026-10-13: a valid, replayed match, 78 min) |
| Agent time budget | 14400 s; tokens: protocol default (PROTOCOL §5) |
| Real time | slowdown S = 20 (the world runs at 1/S of real speed: one control step every S x control_dt s of wall time; `RO_SLOWDOWN`'s default) (PROTOCOL §4b) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-match-0.1.0` |

## Task
Robot badminton against the scripted opponent (level 1), program play: robot singles badminton on a small court (the BWF court's lengths x 0.4, the net at its BWF height 1.524 m) in real time (slowed down): a Franka Panda arm held to the real Panda's limits, with a racket, on an x-y gantry of ball-screw axes (2 m/s, 15 m/s^2), a point-mass shuttle with the real shuttle's high quadratic drag (terminal speed 6.8 m/s) and no bounce, a racket contact law of gain 4 (shuttles leave the racket at up to about 10 m/s); rally scoring, one game to 11 points; the players' robots serve (the rally winner serves next): the server tosses the shuttle (from rest or rising up to 0.5 m) at a point it chooses and hits it underhand, below 1.15 m, into the diagonal service court; format match-wallclock: you against a scripted computer opponent (levels 1-3, `game spec` match.opponent_level), format duel-wallclock: two players.

## Difficulty
Frozen development instance: seed 7, level 1, slowdown S = 20. Not one of
the sport's official (hidden) seeds, so results here are previews, not official Embodied Olympics results.

## Reference solution
`solution/`: our camera-only reference player through the same HTTP interface. screening round 16 (2026-10-13, revision 12, S = 20): our camera-only reference player beat level 1 in 17 of 20 seeds; on this instance at S = 20 it lost 10:12 (vanilla Harbor 0.23.0, 2026-10-13: a valid, replayed match, 78 min).

## Verification
Separate sandbox, no network. The match is graded live (owner, 2026-10-17): the sim service records the
match's scores, winner and reason as it plays it and nothing replays it to grade it (`final.json` `graded_by` live,
`replays` 0; the replay video is drawn after the verdict). The verifier (`tests/grade.py`, our core's grader) applies
the validity rule
(technical failures void the trial, a player's own problems are a valid loss), and writes reward.json; success = a
valid, finished match that the agent won. `tests/fexam_reward.py` adds the hub's fields (`infra_error`, `realtime`,
`slowdown`, `opponent_level`).

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 2 | 6 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 6 | 8 GiB | 256 | no GPU | — |
| verifier (as `main`) | 2 | 6 GiB | — | no GPU | — |

- GPU: not needed.
- Measured: Embodied Olympics' exported matches (scripts/embodied-olympics/exported.toml): the values of the matches it measured on the 2.3 adapter (tasks/embodied-olympics-fun/README.md): the sim takes up to 6 CPUs, the agent 2; memory: sim 0.7-1.4 GB, agent 2.2 GB, mem_guard (docker memory per container, an RTX 4090 workstation, 10-02..10); CPU-only, real time needs the cores
- Measured (caps): memory: agent container docker peak 2.2 GiB over 6 samples (mem_guard (docker memory incl. page cache, top-4 containers every 10 min, an RTX 4090 workstation, 10-02..10)) -> max(1.5 x peak, peak + 4, 8), at most the 4 GiB cap; whole trial: + the sim service's 8 GiB (docker peak 0.7 GiB, 5 samples); processes 512 and disk 20 GiB: design defaults, not yet measured

## Details

### Notes
Source: Embodied Olympics task `badminton-match-wallclock-program-standard`, revision `badminton-wallclock-12+duel-14+core-0.2`. The rules, the referee, the scripted
opponent, the observation, the review between points and the grader are ours, unchanged; only the seat is
unmetered (Harbor records the model's usage) and the instance is frozen.

### Change Log
- 2026-10-16: the replay video only. Badminton declares no spectator camera or scene, so `core.spectator` rendered nothing and every trial ended without a video. The collect hook now runs `environment/sim/spectator/sport_video.py` after the verdict: a fresh process replays `episode.jsonl` with the core's own replay and draws the players' two cameras (the sport's `Views`, its own MuJoCo data) at 10 frames per game second into `replay.mp4`, and `video.json` records whether that replay ended in the verdict replay's state. `RCB_RENDER_VIDEO` is 0 (the core's spectator step only recorded a misleading error), `RCB_SPORT_VIDEO` 1, `RCB_VIDEO_TIMEOUT_S` 1800 s; `[[verifier.collect]]` `timeout_sec` 2580 s (was 1080 s), `[verifier]` `timeout_sec` 2760 s (was 1260 s). `spectator/` is not part of the match's `source_sha` (`core/`, `sport/`): the match, the replay check, the grader and the pass rule are unchanged, and earlier results stand.
