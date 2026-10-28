# embodied-olympics-match-badminton-match-i02-standard-realtime

| | |
|---|---|
| Benchmark | Embodied Olympics - Match |
| Mode | standard-realtime (`eai-standard-realtime/1.0`) |
| Twin | `embodied-olympics-match-badminton-match-i02-privileged` |
| Simulator / robot | `mujoco` / `franka-panda` |
| Family / instance | `badminton_match` / seed 8 |
| Oracle | `partial` — screening round 16 (2026-10-13, revision 12): the truth-state reference beat level 2 in 10 of 20 seeds, the camera-only version did not win level 2; a model did (Opus 11:8, 11:6); this development seed was not screened |
| Agent time budget | 14400 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Real time | slowdown S = 20 (the world runs at 1/S of real speed: one control step every S x control_dt s of wall time; `RO_SLOWDOWN`'s default) (PROTOCOL §3) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-match-0.1.0` |

## Task
Robot badminton against the scripted opponent (level 2), program play: robot singles badminton on a small court (the BWF court's lengths x 0.4, the net at its BWF height 1.524 m) in real time (slowed down): a Franka Panda arm held to the real Panda's limits, with a racket, on an x-y gantry of ball-screw axes (2 m/s, 15 m/s^2), a point-mass shuttle with the real shuttle's high quadratic drag (terminal speed 6.8 m/s) and no bounce, a racket contact law of gain 4 (shuttles leave the racket at up to about 10 m/s); rally scoring, one game to 11 points; the players' robots serve (the rally winner serves next): the server tosses the shuttle (from rest or rising up to 0.5 m) at a point it chooses and hits it underhand, below 1.15 m, into the diagonal service court; format match-wallclock: you against a scripted computer opponent (levels 1-3, `game spec` match.opponent_level), format duel-wallclock: two players.

## Difficulty
Frozen development instance: seed 8, level 2, slowdown S = 20. Not one of
the sport's official (hidden) seeds, so results here are previews, not official Embodied Olympics results.

## Reference solution
`solution/`: our camera-only reference player through the same HTTP interface. screening round 16 (2026-10-13, revision 12): the truth-state reference beat level 2 in 10 of 20 seeds, the camera-only version did not win level 2; a model did (Opus 11:8, 11:6); this development seed was not screened.

## Verification
Separate sandbox, no network. The match is graded live (owner, 2026-10-17): the sim service records the
match's scores, winner and reason as it plays it and nothing replays it to grade it (`final.json` `graded_by` live,
`replays` 0; the replay video is drawn after the verdict). The verifier (`tests/grade.py`, our core's grader) applies
the validity rule
(technical failures void the trial, a player's own problems are a valid loss), and writes reward.json; success = a
valid, finished match that the agent won. `tests/fexam_reward.py` adds the hub's fields (`infra_error`, `realtime`,
`slowdown`, `opponent_level`).

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 2 | 5 GiB | 2048 | no GPU | 10 GiB |
| `sim` | 6 | 6 GiB | 256 | no GPU | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: the values Embodied Olympics measured for its exported matches (tasks/embodied-olympics-fun/README.md): the sim takes up to 6 CPUs, the agent 2; memory: sim 0.7-1.4 GB, agent 2.2 GB (a per-container memory sampler); CPU only, real time needs the cores
- Measured (caps): per-container peaks in 131 past trials: the agent's container memory 1.2 GiB (2.2 GiB in an earlier measurement), 593 threads, CPU at its cap (2), disk 1.3 GiB; the sim service memory 2.0 GiB, 58 threads, CPU 2.0 cores

## Details

### Notes
Source: Embodied Olympics task `badminton-match-wallclock-program-standard`, revision `badminton-wallclock-12+duel-14+core-0.2`. The rules, the referee, the scripted
opponent, the observation, the review between points and the grader are ours, unchanged; only the seat is
unmetered (Harbor records the model's usage) and the instance is frozen.
