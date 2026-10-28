# embodied-olympics-match-tabletennis-match-i03-standard-realtime

| | |
|---|---|
| Benchmark | Embodied Olympics - Match |
| Mode | standard-realtime (`eai-standard-realtime/1.0`) |
| Twin | — (none: a task Embodied Olympics exports with its own service, PROTOCOL §3) |
| Simulator / robot | `mujoco` / `franka-panda` |
| Family / instance | `tabletennis_match` / seed 11 |
| Oracle | `partial` — export-only top tier (sim/match.py TIERS top, 2026-10-16: level 4's serves, the strongest returns the real-limited arm makes, level 2's defence): our truth-state reference beat it in 12 of 40 seeds, our camera-only reference in 7 of 18 (D = 5), both on this seed (5:0 camera, 6:4 truth); every simple baseline lost all its matches; the live oracle run is in the export report |
| Agent time budget | 7200 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Real time | slowdown S = 10 (the world runs at 1/S of real speed: one control step every S x control_dt s of wall time; `RO_SLOWDOWN`'s default) (PROTOCOL §3) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-match-0.1.0` |

## Task
Table tennis against the scripted opponent (level 4, export-only tier top), program play: robot table tennis in a continuous world slowed by a factor S (MuJoCo Franka Panda + paddle on an x-y rail base behind the end line, both held to the real robot's limits: the Panda's joint velocity, acceleration, jerk, torque and torque rate, the rail's speed, acceleration and jerk; DeepMind ball and table model), seen only through two cameras of four frames each; one game to 5 points (lead by 2 from 4:4, at 6:6 the next point wins); the players' robots serve by the ITTF service laws (a toss device on the robot projects the ball at least 16 cm up from a point you choose behind your end line, you strike it as it falls; 2 serves each, from 4:4 one each); format match: you against a scripted opponent (level 1-4; you serve first), format duel: two players.

## Difficulty
Frozen development instance: seed 11, level 4, export-only tier top, slowdown S = 10. Not one of
the sport's official (hidden) seeds, so results here are previews, not official Embodied Olympics results.

## Reference solution
`solution/`: our camera-only reference player through the same HTTP interface. export-only top tier (sim/match.py TIERS top, 2026-10-16: level 4's serves, the strongest returns the real-limited arm makes, level 2's defence): our truth-state reference beat it in 12 of 40 seeds, our camera-only reference in 7 of 18 (D = 5), both on this seed (5:0 camera, 6:4 truth); every simple baseline lost all its matches; the live oracle run is in the export report.

## Verification
Separate sandbox, no network. The match is graded live (owner, 2026-10-17): the sim service records the
match's scores, winner and reason as it plays it and nothing replays it to grade it (`final.json` `graded_by` live,
`replays` 0; the replay video is drawn after the verdict). The verifier (`tests/grade.py`, our core's grader) applies
the validity rule
(technical failures void the trial, a player's own problems are a valid loss), and writes reward.json; success = a
valid, finished match that the agent won. `tests/fexam_reward.py` adds the hub's fields (`infra_error`, `realtime`,
`slowdown`, `opponent_level`).

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 2 | 6 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 6 | 8 GiB | 256 | no GPU | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: the values Embodied Olympics measured for its exported matches (tasks/embodied-olympics-fun/README.md): the sim takes up to 6 CPUs, the agent 2; memory: sim 0.7-1.4 GB, agent 2.2 GB (a per-container memory sampler); CPU only, real time needs the cores
- Measured (caps): processes and disk: defaults, not measured

## Details

### Notes
Source: Embodied Olympics task `tabletennis-match-wallclock-program-standard`, revision `wallclock-15+duel-wallclock-12+low-inloop-1+core-0.2`. The rules, the referee, the scripted
opponent, the observation, the review between points and the grader are ours, unchanged; only the seat is
unmetered (Harbor records the model's usage) and the instance is frozen.
