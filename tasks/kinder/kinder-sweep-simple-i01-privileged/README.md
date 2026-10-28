# kinder-sweep-simple-i01-privileged

| | |
|---|---|
| Benchmark | KinDER |
| Mode | privileged |
| Twin | `kinder-sweep-simple-i01-standard` |
| Simulator / robot | `kinder` / `tidybot-plus-plus` |
| Family / instance | `sweep-simple` / seed 0 |
| Oracle | `none` — none yet: no positive example is known for this instance (human review pending) |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:kinder-0.1.6` |

## Task
KinDER `kinder/SweepSimple3D-o50-sweep_the_blocks_to_the_right_side_of_the_kitchen_island-v0` (Dynamic3D, TidyBot++), seed 0, privileged mode: the agent has the simulator and its state and
hands in one trajectory, replayed open-loop in a fresh process. Its standard-mode twin is `kinder-sweep-simple-i01-standard`.

Task: get the fifty small dark-red cubes scattered on the floor into the translucent grey box marked on the floor at the end of the kitchen island (the free-standing white-topped counter) nearest the robot; a wiper stands on the floor near the cubes. The task succeeds the moment all fifty cubes' centres are inside that box.

## Difficulty
Nonprehensile, tool use: fifty small cubes spread over 2 m² of floor into a box marked at the kitchen island's end, with a wiper standing about 1.7 m from where the robot starts. The variant is upstream's o50 (both cube families use theirs, tasks/kinder/README.md); of its three goals, the one upstream's README gives as the family's example.

## Reference solution
None, and no positive example yet: the task is unvalidated (`validated = false`, docs/protocol/README.md §2, §4) until a human review records one.

## Verification
Two open-loop replays of `/app/output/trajectory.npz` from `tests/instance.json` in a fresh process, in a container of
their own (`tests/Dockerfile`, no network): pass iff both reach the goal within 1000 control steps and end in
the same state. No oracle; no positive example yet (human review pending). `nop` scores 0.

Benchmark notes: `tasks/kinder/README.md`.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 16 | 20 GiB | 8192 | no GPU | 10 GiB |
| verifier | 4 | 8 GiB | no cap | no GPU | — |

- GPU: not needed.
- Measured: GPU memory 1.2-1.3 GB (the oracle 1.2 GB on an RTX 4090; per-trial peaks of past trials); memory: privileged container p90 2.8 GB, sim 1.1 GB (a per-container memory sampler); CPU: an estimate

## Details

### Change Log
- 2026-10-16: first version, a new instance replacing `kinder-sweep-simple-i00-privileged` (upstream's o10 variant: ten cubes, the left side of the kitchen island): the scene changed, so the task has a new name (PROTOCOL §1, H.8). Agent time limit 1800 s.
