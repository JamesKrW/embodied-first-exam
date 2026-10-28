# bigym2-wall-cupboard-open-i00-privileged

| | |
|---|---|
| Benchmark | BiGym 2.0 |
| Mode | privileged |
| Twin | `bigym2-wall-cupboard-open-i00-standard` |
| Simulator / robot | `bigym2` / `unitree-g1-dex1` |
| Family / instance | `wall-cupboard-open` / seed 0 |
| Oracle | `none` — no reference solution (BiGym 2.0 ships no expert or planner; its demonstrations and its agents' programs stay off the agent's network); positive example graded 1 by the separate verifier: bigym2-wall-cupboard-open-i00-pr__mcuR6nu (run codex-gpt6_luna-medium, 2026-10-08, on an earlier text of the privileged task; regraded by its current verifier); human review in PR #123 |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:bigym2-0.1.1` |

## Task
Open both doors of the wall cabinet. (BiGym 2.0 `wall_cupboard_open`, a Unitree G1 humanoid whose legs and waist NVIDIA's GR00T whole-body controller drives.)

## Difficulty
Two hinged doors above the counter, both opened to within about 9 degrees of fully open at the same time.

## Reference solution
None. BiGym 2.0 has no scripted expert or planner; its human demonstrations and its coding agents' programs are public
on the Hugging Face Hub and never reach the agent (no network but the model APIs; seed 0 is none of their seeds).

## Verification
`tests/grade.py` replays `/app/output/trajectory.npz` from the frozen instance in two fresh processes (`rcb_bigym2.py
replay`: upstream's official env, one step per row until upstream ends the episode). Pass iff upstream's `is_success`
holds in both (the task's success held for 1 s, no fall) and both end in the same state. Success condition: both doors of the wall cabinet are open to within 10% of their travel (within about 9 degrees of fully open), at the same time.
No oracle; human review in PR #123 (PROTOCOL §2.7).

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 20 GiB | 8192 | 6 GiB | 10 GiB |
| verifier | 4 | 16 GiB | — | 6 GiB | — |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU 0.7-1.2 GB, evaluation-pool GPU profile (per-trial GPU peaks, 10-03..10); memory: privileged container 3.0 GB, mem_guard (docker memory per container, an RTX 4090 workstation, 10-02..10); CPU author estimate (MuJoCo + two ONNX policies on the CPU)
- Measured (caps): GPU: 1.2 GiB (the evaluation pool's GPU profile (per-trial GPU peaks by card, 10-03..10)) -> max(1.5 x peak, peak + 4); memory: the [environment] cap 16 GiB, which every past trial ran under (docker peak 3.0 GiB over 1 samples, mem_guard (docker memory incl. page cache, top-4 containers every 10 min, an RTX 4090 workstation, 10-02..10)); processes 512 and disk 10 GiB: design defaults, not yet measured

## Details
| | |
|---|---|
| Upstream | BiGym 2.0 (swirl-uk/BiGym2 at 93ad4b3, bigym 1.0.0, Apache-2.0), task `wall_cupboard_open` ("kitchen counter"), official configuration |
| Instance | `environment/instance.json` = `tests/instance.json`: the task reset with seed 0, t = 0 state hash `21069c16aff06dbfd19e9b64345d1a90328352e1` |
| Horizon | 1450 control steps (29 s at 50 Hz), upstream's budget for this task |
| Action | 21 floats per control step (`instance.json` `t0.layout`) |
| Tooling | `environment/rcb_bigym2.py` = `tests/rcb_bigym2.py` (scripts/bigym2/shared/) |
