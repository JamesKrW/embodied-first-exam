# embodiedswe-assembly-bulb-franka-osc-i00-privileged

| | |
|---|---|
| Benchmark | EmbodiedSWE |
| Mode | privileged |
| Twin | — (privileged only) |
| Simulator / robot | `embodiedswe` / `franka-panda` |
| Family / instance | `assembly.bulb.franka.osc` / seed 0 |
| Oracle | `none` — No reference solution in the repository. Pilot (board 0265): a scripted reference solve() written for this pilot is kept outside the task (internal only); its result on this version is recorded in the pilot report. |
| Agent time budget | 14400 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:embodiedswe-0.1.0` |

## Task
Screw a light bulb into its socket (Franka, operational-space control). EmbodiedSWE's `assembly.bulb.franka.osc` task (robobench at commit `67f44e8a2ca5`), privileged mode, with a program as the
deliverable: `solution/solve.py` exposing `solve(env)`.

## Difficulty
Contact-rich assembly: a grasp of a small round bulb, alignment on a threaded socket and a turn-while-pressing motion under torque control (OSC); EmbodiedSWE's GPT-6-class agents solve it within a few hours in their own runs.

## Reference solution
None in the repository (`oracle = "none"`). The pilot's scripted reference `solve()` is kept outside the task.

## Verification
`tests/grade.py`: a trusted simulator process in the verifier container builds the env from the frozen instance
(seed 0, PhysX on the CPU), checks its state hash and attaches EmbodiedSWE's grader for the task (`tests/grader/`); the
agent's `solve()` runs in a separate process as `nobody` (no network, no GPU, process limit) behind a proxy that
answers reads with copies, executes `env.step(action)` and refuses everything else. The rollout ends when `solve`
returns, raises or dies, or after 30 min; the verdict is read on the state then: success = the grader's end
predicate, `progress` = its rubric score. A fresh process then re-executes the recorded actions; `reward.txt` = 1 iff
the rollout and the replay both succeed and reach the same final-state hash (`deterministic`). The video is rendered
afterwards from the recorded actions.

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 4 | 4 |
| Memory | 13 GB | 13 GB |
| GPU memory | 6.5 GB | 6.5 GB |

- GPU: required, an RTX-class NVIDIA GPU (ray-tracing cores, for Isaac Sim / Omniverse RTX rendering); recommended: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: not yet measured: RoboLab's privileged figures (the same Isaac Sim 5.1 stack and recipe, CPU PhysX) as the estimate
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), every container of the trial: 24 GiB GPU memory, 24 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 24 GiB GPU memory, 24 GiB memory. Limits measured: not yet measured: RoboLab's privileged limits (the same Isaac Sim stack) as the estimate.
