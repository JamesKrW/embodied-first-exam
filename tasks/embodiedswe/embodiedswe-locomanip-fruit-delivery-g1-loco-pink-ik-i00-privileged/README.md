# embodiedswe-locomanip-fruit-delivery-g1-loco-pink-ik-i00-privileged

| | |
|---|---|
| Benchmark | EmbodiedSWE |
| Mode | privileged |
| Twin | — (privileged only) |
| Simulator / robot | `embodiedswe` / `unitree-g1-dex3` |
| Family / instance | `locomanip.fruit_delivery.g1.loco_pink_ik` / seed 0 |
| Oracle | `none` — No reference solution in the repository. A scripted reference solve() exists outside the repository (internal only, used to check the task is solvable through the grading path); it is not shipped. |
| Agent time budget | 14400 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:embodiedswe-0.3.0` |

## Task
Deliver the fruit to the plate (Unitree G1, walking, Pink IK). EmbodiedSWE's `locomanip.fruit_delivery.g1.loco_pink_ik` task (robobench at commit `67f44e8a2ca5`), privileged mode, with a program as the
deliverable: `solution/solve.py` exposing `solve(env)`.

## Difficulty
Loco-manipulation: the G1 walks under a frozen locomotion policy (base velocity commands) while its arms and hands pick fruit and carry it to a plate elsewhere in the room.

## Reference solution
None in the repository (`oracle = "none"`). A scripted reference `solve()` is kept outside the repository (internal only).

## Verification
`tests/grade.py`: a trusted simulator process in the verifier container builds the env from the frozen instance
(seed 0, PhysX on the CPU), checks its state hash and attaches EmbodiedSWE's grader for the task (`tests/grader/`); the
agent's `solve()` runs in a separate process as `nobody` (no network, no GPU, process limit) behind a proxy that
answers reads with copies, executes `env.step(action)` and refuses everything else. The rollout ends when `solve`
returns, raises or dies, or after 30 min; the verdict is read on the state then: success = the grader's end
predicate, `progress` = its rubric score (its peak recorded progress), and `reward.txt` = that success, as
EmbodiedSWE grades. A fresh process then re-executes the recorded actions as a diagnostic (`deterministic` = the same
final-state hash, `replay_success`, `replay_progress` in `reward.json`); it does not change the verdict. The video is
rendered afterwards from the recorded actions (`video.json`: `video_mismatch` when that replay misses the run's
final state).

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 4 | 4 |
| Memory | 13 GB | 13 GB |
| GPU memory | 5 GB | 5 GB |

- GPU: required, an RTX-class NVIDIA GPU (ray-tracing cores, for Isaac Sim / Omniverse RTX rendering); recommended: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: measured on an L40S with a scripted reference, one graded run alone on the GPU (docker stats + nvidia-smi every 2 s): rollout and replay through the proxy 3.0 GiB RAM / 0.56 GiB GPU, video render 11.9 / 4.4; minimums = the video peak plus headroom
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), every container of the trial: 24 GiB GPU memory, 24 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 24 GiB GPU memory, 24 GiB memory. Limits measured: not yet measured: RoboLab's privileged limits (the same Isaac Sim stack) as the estimate.

## Details

### Review notes
- Contact artifact (reported, not changed): at a high grip closure (1.3 on the hand's closure scale) the middle
  fingertip of the G1's right hand stays attached to the lemon after the fingers open, and the lemon rises with the
  hand (3 of 3 trials; 0 of 4 at closure 0.9). It looks like a penetration artifact of the contact model. It makes
  the release harder than it is physically, and a solution could use it as glue while carrying.
- Device: PhysX and the frozen locomotion policy run on the CPU only (the tooling refuses another device). On a GPU
  the policy is deterministic but its bits differ from the CPU's, the start state differs from the frozen instance,
  and a trajectory recorded on the CPU fails there.
