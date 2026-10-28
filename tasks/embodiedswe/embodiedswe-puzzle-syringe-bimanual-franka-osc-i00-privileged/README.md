# embodiedswe-puzzle-syringe-bimanual-franka-osc-i00-privileged

| | |
|---|---|
| Benchmark | EmbodiedSWE |
| Mode | privileged |
| Twin | — (privileged only) |
| Simulator / robot | `embodiedswe` / `franka-panda-bimanual` |
| Family / instance | `puzzle.syringe.bimanual_franka.osc` / seed 0 |
| Oracle | `none` — No reference solution in the repository, and none written for this scene yet; the build, the start-state hash and a do-nothing run scoring 0 are checked. |
| Agent time budget | 14400 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:embodiedswe-0.3.0` |

## Task
Draw and dose liquid with a syringe (two Frankas, operational-space control). EmbodiedSWE's `puzzle.syringe.bimanual_franka.osc` task (robobench at commit `67f44e8a2ca5`), privileged mode, with a program as the
deliverable: `solution/solve.py` exposing `solve(env)`.

## Difficulty
Bimanual tool use with two arms under torque control (OSC): draw a full load of liquid from a reservoir with a syringe, dispense a dose inside a set band into each of three test tubes without spilling, then lay the syringe back down at its spot.

## Reference solution
None in the repository (`oracle = "none"`). No scripted reference has been written for this scene yet.

## Verification
`tests/grade.py`, as EmbodiedSWE's harbor verifier: every checkpoint the agent froze with `submit` (at most 64) and
the final `solution/` are candidates, and each distinct one is graded in a fresh process. There a trusted simulator
builds the env from the frozen instance (seed 0, GPU PhysX), compares its start state and attaches EmbodiedSWE's grader
for the task (`tests/grader/`); the candidate's `solve()` runs in a separate process as `nobody` (no network, no GPU,
process limit) behind a proxy that answers reads with copies, executes `env.step(action)` and refuses everything else.
The run ends when `solve` returns, raises or dies, or 30 min after the process started (start-up and import count);
the verdict is read on the state then: success = the grader's end predicate, `progress` = its rubric score (its peak
recorded progress). The result is the candidate with the highest progress (ties: higher success, then the earliest);
`reward.txt` = its success. Its trajectory (`trajectory.npz`: actions and recorded states) is rendered afterwards by
posing the recorded states (`replay.mp4`, `video.json`); the video does not change the verdict.

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 4 | 4 |
| Memory | 13 GB | 13 GB |
| GPU memory | 8 GB | 8 GB |

- GPU: required, an RTX-class NVIDIA GPU (ray-tracing cores, for Isaac Sim / Omniverse RTX rendering); recommended: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: GPU PhysX, measured on an L40S (nvidia-smi every 2 s): a graded rollout process 3.2 to 4.6 GiB of GPU memory, the video process (cameras) 6.6 to 6.9 GiB; GPU minimum = the video peak plus headroom; memory: RoboLab's privileged figures (the same Isaac Sim 5.1 stack) as the estimate
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), every container of the trial: 24 GiB GPU memory, 24 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 24 GiB GPU memory, 24 GiB memory. Limits measured: not yet measured: RoboLab's privileged limits (the same Isaac Sim stack) as the estimate.
