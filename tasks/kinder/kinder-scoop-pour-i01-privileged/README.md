# kinder-scoop-pour-i01-privileged

| | |
|---|---|
| Benchmark | KinDER |
| Mode | privileged |
| Twin | `kinder-scoop-pour-i01-standard` |
| Simulator / robot | `kinder` / `tidybot-plus-plus` |
| Family / instance | `scoop-pour` / seed 0 |
| Oracle | `none` — none yet: no positive example is known for this instance (human review pending) |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:kinder-0.1.6` |

## Task
KinDER `kinder/ScoopPour3D-o50-v0` (Dynamic3D, TidyBot++), seed 0, privileged mode: the agent has the simulator and its state and
hands in one trajectory, replayed open-loop in a fresh process. Its standard-mode twin is `kinder-scoop-pour-i01-standard`.

Task: move all 50 small dark-red cubes from the yellow bin into the green bin; a scoop is in the scene. The task succeeds the moment every cube's centre is inside the green bin, at least 1 cm from each of its inner walls and at least 0.5 cm below its rim.

## Difficulty
Nonprehensile, tool use, dynamics: move 50 small cubes from one bin to another with a scoop, each ending well inside the target bin. The variant is upstream's o50 (both cube families use theirs, tasks/kinder/README.md).

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
| verifier | 4 | 8 GiB | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: memory: privileged container p90 2.8 GB (a per-container memory sampler); CPU only; CPU: an estimate
- Measured (caps): processes and disk: defaults, not measured

## Details

### Change Log
- 2026-10-16: first version, a new instance replacing `kinder-scoop-pour-i00-privileged` (upstream's o30 variant: 30 cubes): the scene changed, so the task has a new name (PROTOCOL §1, H.8). Agent time limit 1800 s.
