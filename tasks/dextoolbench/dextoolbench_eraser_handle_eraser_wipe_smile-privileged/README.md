# dextoolbench_eraser_handle_eraser_wipe_smile-privileged

| | |
|---|---|
| Benchmark | DexToolBench |
| Mode | privileged |
| Twin | `dextoolbench_eraser_handle_eraser_wipe_smile-standard` |
| Simulator / robot | `mujoco` / `kuka-iiwa14-sharpa-ha4` |
| Family / instance | `dextoolbench/eraser/handle_eraser/wipe_smile` / seed 0 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:mujoco-0.1.2` |

## Task
DexToolBench task `eraser / handle_eraser / wipe_smile` (SimToolReal, MIT) ported to plain MuJoCo for this benchmark.

## Difficulty
Dexterous tool use: a KUKA iiwa 14 arm with a 22-DoF Sharpa HA4 hand (29 position-controlled joints) must carry the tool along DexToolBench's goal trajectory, every goal within 1.5 cm on all keypoints, and success needs every goal. SimToolReal's pretrained policy reaches 29/29 goals here.

## Reference solution
`solution/`: SimToolReal's pretrained policy run in this exact scene, recorded targets replayed: **29/29 goals** in 1542 control steps (the policy's own sim2sim progress; a task's reference, not a ceiling); nop 0/29

## Verification
DexToolBench task progress: goal reached when all 8 grasp-box keypoints are within 1.5 cm; 30 s per goal (upstream 10 s); `reward.txt` = 1 iff every goal is reached (both replays agreeing); `progress` = goals / 29 in `reward.json`

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 20 GiB | 8192 | 7 GiB | 10 GiB |
| verifier | no cap | no cap | no cap | 7 GiB | no cap |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU memory: per-trial peaks of past trials 0.3-0.9 GB on 24 GB cards and up to 2.1 GiB on a 48 GB card; memory: privileged container p90 4.4 GB (a per-container memory sampler); CPU: an estimate
- Measured (caps): GPU memory: a per-trial peak of 2.1 GiB on a 48 GB card -> max(1.5 x peak, peak + 4), rounded up: 7 GiB; processes and disk: defaults, not measured

## Details
| | |
|---|---|
| source | [SimToolReal / DexToolBench](https://github.com/tylerlum/simtoolreal) @ 313d5ae: object meshes, goal trajectory (29 goals); MuJoCo scene rebuilt from its sim2sim `deployment/mujoco/mujoco_sim.py` |
| robot | KUKA iiwa 14 (MuJoCo Menagerie @ 1b86ece) + left Sharpa HA4 hand, 29 position-controlled joints |
| physics | MuJoCo 3.3.7, 600 Hz, implicitfast, elliptic cones; control 60 Hz (10 substeps) |
| deliverable | `(T, 29)` joint targets per control step; bit-exact replay |
| table props | whiteboard (table_narrow_whiteboard) |
| egress | allowlist (the model APIs and the package registries): the pretrained policy is publicly downloadable |
