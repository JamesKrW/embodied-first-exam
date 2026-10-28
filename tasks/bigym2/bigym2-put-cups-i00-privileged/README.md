# bigym2-put-cups-i00-privileged

| | |
|---|---|
| Benchmark | BiGym 2.0 |
| Mode | privileged |
| Twin | `bigym2-put-cups-i00-standard` |
| Simulator / robot | `bigym2` / `unitree-g1-dex1` |
| Family / instance | `put-cups` / seed 0 |
| Oracle | `none` — no reference solution (BiGym 2.0 ships no expert or planner; its demonstrations and its agents' programs stay off the agent's network); no positive example yet; human review in PR #123 |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:bigym2-0.1.1` |

## Task
Take the cups from the counter and gently put them away in the wall cabinet. (BiGym 2.0 `put_cups`, a Unitree G1 humanoid whose legs and waist NVIDIA's GR00T whole-body controller drives.)

## Difficulty
Two stages: open the wall cabinet's glass door, then place two cups on its bottom shelf, released.

## Reference solution
None. BiGym 2.0 has no scripted expert or planner; its human demonstrations and its coding agents' programs are public
on the Hugging Face Hub and never reach the agent (no network but the model APIs; seed 0 is none of their seeds).

## Verification
`tests/grade.py` replays `/app/output/trajectory.npz` from the frozen instance in two fresh processes (`rcb_bigym2.py
replay`: upstream's official env, one step per row until upstream ends the episode). Pass iff upstream's `is_success`
holds in both (the task's success held for 1 s, no fall) and both end in the same state. Success condition: both cups touch the bottom shelf of the wall cabinet (its glass doors start closed) and neither is held by a gripper.
No oracle; human review in PR #123 (PROTOCOL §2.7).

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 20 GiB | 8192 | 6 GiB | 10 GiB |
| verifier | 4 | 16 GiB | no cap | 6 GiB | — |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU memory 0.7-1.2 GB (per-trial peaks of past trials); memory: privileged container 3.0 GB (a per-container memory sampler); CPU: an estimate (MuJoCo and two ONNX policies on the CPU)
- Measured (caps): GPU memory: a per-trial peak of 1.2 GiB -> max(1.5 x peak, peak + 4), rounded up: 6 GiB

## Details
| | |
|---|---|
| Upstream | BiGym 2.0 (swirl-uk/BiGym2 at 93ad4b3, bigym 1.0.0, Apache-2.0), task `put_cups` ("kitchen counter"), official configuration |
| Instance | `environment/instance.json` = `tests/instance.json`: the task reset with seed 0, t = 0 state hash `5f4cdf3a89e1fa8ae86ad094e8b0ca9d1f281017` |
| Horizon | 3000 control steps (60 s at 50 Hz), upstream's budget for this task |
| Action | 21 floats per control step (`instance.json` `t0.layout`) |
| Tooling | `environment/rcb_bigym2.py` = `tests/rcb_bigym2.py` (scripts/bigym2/shared/) |
