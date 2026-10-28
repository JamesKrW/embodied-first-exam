# embodied-olympics-match-soccerssl-match-i00-standard-realtime

| | |
|---|---|
| Benchmark | Embodied Olympics - Match |
| Mode | standard-realtime (`eai-standard-realtime/1.0`) |
| Twin | `embodied-olympics-match-soccerssl-match-i00-privileged` |
| Simulator / robot | `mujoco` / `ssl-robot` |
| Family / instance | `soccer_ssl_match` / seed 7 |
| Oracle | `partial` — screening (soccer_ssl-1): the camera reference's result at level 2 is not established for this development seed; a model won level 2 (Opus 4:3, camera only). The oracle run checks the pipeline |
| Agent time budget | 14400 s; tokens: protocol default (PROTOCOL §5) |
| Real time | slowdown S = 10 (the world runs at 1/S of real speed: one control step every S x control_dt s of wall time; `RO_SLOWDOWN`'s default) (PROTOCOL §4b) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-match-0.1.0` |

## Task
Wheeled robot soccer 5 v 5 (RoboCup SSL Division B) against the scripted opponent (level 2), program play: wheeled robot soccer 5 v 5 under the RoboCup Small Size League rules (Division B): ten omnidirectional SSL robots after TIGERs Mannheim's v2020 (kicker, chip kicker, dribbler; their real speed, acceleration, kick and dribbler limits) and an orange golf ball on the 9 x 6 m field, two halves of 300 s of playing time, a shoot-out when level, an automatic referee for every rule an SSL auto-referee judges; you command your robots with three skills (move to a pose, optionally by a time and at a speed; kick flat or chipped along the robot's heading, now or the moment the ball reaches it; spin the dribbler) and see the game through the four overhead cameras of the shared vision, 40 ms late; in the `match` format you command the blue team against a scripted team, in the `duel` format another player commands the yellow team, in the `team` format each robot has its own player.

## Difficulty
Frozen development instance: seed 7, level 2, slowdown S = 10. Not one of
the sport's official (hidden) seeds, so results here are previews, not official Embodied Olympics results.

## Reference solution
`solution/`: our camera-only reference player through the same HTTP interface. screening (soccer_ssl-1): the camera reference's result at level 2 is not established for this development seed; a model won level 2 (Opus 4:3, camera only). The oracle run checks the pipeline.

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
Source: Embodied Olympics task `soccer_ssl-match-wallclock-program-standard`, revision `soccer_ssl-1`. The rules, the referee, the scripted
opponent, the observation, the review between points and the grader are ours, unchanged; only the seat is
unmetered (Harbor records the model's usage) and the instance is frozen.
