# bigym2-saucepan-to-hob-i00-standard

| | |
|---|---|
| Benchmark | BiGym 2.0 |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `bigym2-saucepan-to-hob-i00-privileged` |
| Simulator / robot | `bigym2` / `unitree-g1-dex1` |
| Family / instance | `saucepan-to-hob` / seed 0 |
| Oracle | `none` — no reference solution (BiGym 2.0 ships no expert or planner; its demonstrations and its agents' programs stay off the agent's network); no positive example yet; human review in PR #123 |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:bigym2-0.1.1` |

## Task
Take the saucepan out of the cabinet and gently set it on the hob. (BiGym 2.0 `saucepan_to_hob`; the robot as a service: its three onboard cameras and its proprioception only, one
episode.)

## Difficulty
Two stages low and high: open the cabinet under the counter, take the saucepan out, set it on the hob, released. In standard mode the agent sees only the robot's cameras (the head camera looks down at the hands at the
start) and proprioception, acts through the client, and gets one episode, no reset.

## Reference solution
None (as `bigym2-saucepan-to-hob-i00-privileged`).

## Verification
The sim sidecar judges the live episode with BiGym's own success check (`is_success`: the task's success held for 1 s,
no fall) and writes that verdict to `/data/final.json` (`graded_by = "live"`); nothing is replayed to grade it (owner,
2026-10-17). The scene's start is checked against the frozen instance (`instance.json`'s `t0`) when an
episode begins (a mismatch is `instance_mismatch`, infrastructure). `tests/grade.py` reads `final.json`: pass iff the
live episode succeeded. The video (`replay.mp4`) is rendered from `trajectory.npz` in one pass after the verdict and
never changes it. Success condition: the saucepan touches the hob and is held by neither gripper (it starts inside the closed cabinet under the counter). No oracle; human review in PR #123 (PROTOCOL §2.7).

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 16 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 4 | 10 GiB | 8192 | 6 GiB | — |
| verifier | no cap | no cap | no cap | no GPU | — |

- GPU: any NVIDIA GPU, held by `sim`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU memory 0.7-1.2 GB (per-trial peaks of past trials); memory: privileged container 3.0 GB (a per-container memory sampler); CPU: an estimate (MuJoCo and two ONNX policies on the CPU)
- Measured (caps): GPU memory (the sim service): a per-trial peak of 1.2 GiB -> max(1.5 x peak, peak + 4), rounded up: 6 GiB

## Details
| | |
|---|---|
| Service | `environment/sim/server.py` (the BiGym 2.0 adapter) on `rcb_service.py` (eai-standard/2.1), `rcb_bigym2.py`, `instance.json` (= the privileged twin's) |
| Observations | three onboard RGB cameras (`RCB_IMAGE_HW`, default 84 px square, as upstream renders them, 60° field of view); upstream's low-dimensional observation (joints, grippers, tracked pelvis pose) |
| Horizon | 4650 control steps (93 s at 50 Hz) |

### Change Log
- 2026-10-11: `RCB_RESET_BUDGET`'s default 0 -> 10 (owner decision; PROTOCOL §4.5,
  tasks/bigym2/README.md); the service already had `POST /reset`. The scene, instance, pass rule and verifier are
  unchanged, so the task keeps its name; runs recorded before this date had no reset and are reported separately.
- 2026-10-16: `RCB_RESET_BUDGET`'s default 10 -> 0 (author decision; PROTOCOL §2, the protocol's default,
  tasks/bigym2/README.md): one episode, which the practice and the graded attempt share; the instruction says there
  is no reset. The scene, instance, pass rule and verifier are unchanged, so the task keeps its name; runs recorded
  with 10 resets are reported separately.
- 2026-10-17 (owner): graded by the live episode: `success` = BiGym's own `is_success` in the episode
  the sim service ran (the same rule, judged as it ran), and no grading replay runs (no `replay_success` /
  `deterministic` any more; `final.json` has `graded_by = "live"`, `replays = 0`, `reward.json` `graded_by_live = 1`).
  The video is one render pass over the trajectory after the verdict, as before. The scene's start is checked when an
  episode begins: a scene that does not rebuild the frozen instance's starting state (exactly, or within upstream's
  tolerance on another CPU) gives up as infrastructure (`instance_mismatch`). The scene, instance, pass rule and the
  agent's interface are unchanged, so the task keeps its name; every archived verdict is the same under the live rule.
