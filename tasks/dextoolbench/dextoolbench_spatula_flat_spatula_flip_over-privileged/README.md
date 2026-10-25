# dextoolbench_spatula_flat_spatula_flip_over-privileged

| | |
|---|---|
| Benchmark | DexToolBench |
| Mode | privileged |
| Twin | `dextoolbench_spatula_flat_spatula_flip_over-standard` |
| Simulator / robot | `mujoco` / `kuka-iiwa14-sharpa-ha4` |
| Family / instance | `dextoolbench/spatula/flat_spatula/flip_over` / seed 0 |
| Oracle | `partial` — SimToolReal's pretrained policy, replayed in this scene, reaches 22/45 goals (solution/; README of dextoolbench_spatula_flat_spatula_flip_over-privileged) |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:mujoco-0.1.2` |

## Task
DexToolBench task `spatula / flat_spatula / flip_over` (SimToolReal, MIT) ported to plain MuJoCo for this benchmark.

## Difficulty
Dexterous tool use: a KUKA iiwa 14 arm with a 22-DoF Sharpa HA4 hand (29 position-controlled joints) must carry the tool along DexToolBench's goal trajectory, every goal within 1.5 cm on all keypoints, and success needs every goal. SimToolReal's pretrained policy reaches 22/45 goals here.

## Reference solution
`solution/`: SimToolReal's pretrained policy run in this exact scene, recorded targets replayed: **22/45 goals** in 1829 control steps (the policy's own sim2sim progress; a task's reference, not a ceiling); nop 0/45

## Verification
DexToolBench task progress: goal reached when all 8 grasp-box keypoints are within 1.5 cm; 30 s per goal (upstream 10 s); `reward.txt` = 1 iff every goal is reached (both replays agreeing); `progress` = goals / 45 in `reward.json`

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 2 | 2 |
| Memory | 6 GB | 3 GB |
| GPU memory | 1.5 GB | 1.5 GB |

- GPU: required, any NVIDIA GPU; recommended: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU 0.3-0.9 GB on 24 GB cards, evaluation-pool GPU profile (per-trial GPU peaks, 10-03..10); memory: sim p90 0.9 GB, privileged container p90 4.4 GB, mem_guard (docker memory per container, an RTX 4090 workstation, 10-02..10); CPU author estimate
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), every container of the trial: 7 GiB GPU memory, 16 GiB memory, 512 processes, 10 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 7 GiB GPU memory, 16 GiB memory. Limits measured: GPU: 2.1 GiB on a 48 GB card (the evaluation pool's GPU profile (per-trial GPU peaks by card, 10-03..10)) -> max(1.5 x peak, peak + 4); memory: the [environment] cap 16 GiB, which every past trial ran under (docker peak 13.1 GiB over 26 samples, mem_guard (docker memory incl. page cache, top-4 containers every 10 min, an RTX 4090 workstation, 10-02..10)); processes 512 and disk 10 GiB: design defaults, not yet measured.

## Details
| | |
|---|---|
| source | [SimToolReal / DexToolBench](https://github.com/tylerlum/simtoolreal) @ 313d5ae: object meshes, goal trajectory (45 goals); MuJoCo scene rebuilt from its sim2sim `deployment/mujoco/mujoco_sim.py` |
| robot | KUKA iiwa 14 (MuJoCo Menagerie @ 1b86ece) + left Sharpa HA4 hand, 29 position-controlled joints |
| physics | MuJoCo 3.3.7, 600 Hz, implicitfast, elliptic cones; control 60 Hz (10 substeps) |
| deliverable | `(T, 29)` joint targets per control step; bit-exact replay |
| table props | bowl + plate (table_narrow_bowl_plate; primitive stand-ins, the YCB meshes are not in the SimToolReal repo) |
| egress | allowlist (the model APIs and the package registries): the pretrained policy is publicly downloadable |

### Change Log
- 2026-10-25: **new start pose** (`instance.json` `start_pose`; the goals are unchanged). The demo's start pose lays the spatula flat on the bare table, but this scene has the bowl and plate stand-ins under it: the blade went 3.4 cm into the bowl wall and the handle 1.25 cm into the plate, so at the first physics steps (t ≈ 0.017 s, before the hand moves) contact pushed the spatula up 17 cm at about 2 m/s and it landed 9 cm away, turned over (178°). The new start is where the spatula comes to rest when it is dropped from just above the old pose: on the plate and the bowl rim, 2.0 cm higher, 0.3 cm aside, tilted 13°; from it the spatula settles by under 1 mm and 0.4°. The oracle (SimToolReal's pretrained policy, re-recorded in the new scene) reaches 22/45 goals (before: 7/45); nop 0. Results from before this date ran from the old start and are not comparable.
