# kinder-dynamo-i00-privileged

| | |
|---|---|
| Benchmark | KinDER |
| Mode | privileged |
| Twin | `kinder-dynamo-i00-standard` |
| Simulator / robot | `kinder` / `tidybot-plus-plus` |
| Family / instance | `dynamo` / seed 0 |
| Oracle | `full` |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:kinder-0.1.6` |

## Task
KinDER `kinder/Dynamo3D-o3-v0` (Dynamic3D, TidyBot++), seed 0, privileged mode: the agent has the simulator and its state and
hands in one trajectory, replayed open-loop in a fresh process. Its standard-mode twin is `kinder-dynamo-i00-standard`.

Task: drive the robot into the translucent green square marked on the floor; three chairs stand in the way and can be pushed. The task succeeds the moment the centre of the robot's base is inside the square. (The red, blue and yellow squares only mark where the chairs started.)

## Difficulty
Nonprehensile: three chairs stand between the robot and the floor square, and the base can push them. The benchmark's fixture: the pipeline check, with an oracle in both modes.

## Reference solution
`solution/oracle.py` steers the base from the state around the chairs' north side (three waypoints, every move inside a 0.1 m disc) into the square: success at step 101.

## Verification
Two open-loop replays of `/app/output/trajectory.npz` from `tests/instance.json` in a fresh process, in a container of
their own (`tests/Dockerfile`, no network): pass iff both reach the goal within 1000 control steps and end in
the same state. The oracle (`solution/solve.sh`) scores 1; `nop` scores 0.

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
- 2026-10-16: agent time limit 3600 s -> 1800 s (`[agent] timeout_sec`, both twins; author decision, tasks/kinder/README.md). The scene, instance, pass rule and verifier logic are unchanged (`tests/rcb_limits.json` carries the new cutoff), so the task keeps its name; runs recorded before this date are reported separately.
