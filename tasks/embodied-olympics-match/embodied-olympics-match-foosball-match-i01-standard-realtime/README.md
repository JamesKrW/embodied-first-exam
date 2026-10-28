# embodied-olympics-match-foosball-match-i01-standard-realtime

| | |
|---|---|
| Benchmark | Embodied Olympics - Match |
| Mode | standard-realtime (`eai-standard-realtime/1.0`) |
| Twin | — (none: a task Embodied Olympics exports with its own service, PROTOCOL §4c) |
| Simulator / robot | `mujoco` / `foosball-rods` |
| Family / instance | `foosball_match` / seed 7 |
| Oracle | `partial` — the hardest level our reference player wins (the rule: each sport at the hardest level the oracle beats): at level 4 (80 ms reactions) it lost 0:5 on this seed on vanilla Harbor and wins 10 of 100 in the screening; at level 3 (90 ms, 21 of 100 in the screening, docs/screening/foosball.md) it won this seed 5:4 on vanilla Harbor 0.23.0 (2026-10-16, an RTX 4090 workstation; seeds 7-9 tried: 1 won). Realtime play varies run to run, hence partial |
| Agent time budget | 14400 s; tokens: protocol default (PROTOCOL §5) |
| Real time | slowdown S = 20 (the world runs at 1/S of real speed: one control step every S x control_dt s of wall time; `RO_SLOWDOWN`'s default) (PROTOCOL §4b) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-match-0.1.0` |

## Task
Foosball against the scripted opponent (level 3), program play: table football on a physical table: you drive the four motorised rods of one team (goalkeeper rod with 3 figures, 2-rod 2, 5-rod 5, 3-rod 3) by slide and angle targets that the motors follow along a speed- and acceleration-limited path, and see the table only through an overhead camera; one game to 5 goals under the ITSF rules (opening sequence, possession time limits per rod, no spinning); in the `match` format the other team is a scripted computer team, in the `duel` format another player.

## Difficulty
Frozen development instance: seed 7, level 3, slowdown S = 20. Not one of
the sport's official (hidden) seeds, so results here are previews, not official Embodied Olympics results.

## Reference solution
`solution/`: our camera-only reference player through the same HTTP interface. the hardest level our reference player wins (the rule: each sport at the hardest level the oracle beats): at level 4 (80 ms reactions) it lost 0:5 on this seed on vanilla Harbor and wins 10 of 100 in the screening; at level 3 (90 ms, 21 of 100 in the screening, docs/screening/foosball.md) it won this seed 5:4 on vanilla Harbor 0.23.0 (2026-10-16, an RTX 4090 workstation; seeds 7-9 tried: 1 won). Realtime play varies run to run, hence partial.

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
Source: Embodied Olympics task `foosball-match-wallclock-program-standard`, revision `foosball-program-v2+core-0.2`. The rules, the referee, the scripted
opponent, the observation, the review between points and the grader are ours, unchanged; only the seat is
unmetered (Harbor records the model's usage) and the instance is frozen.
