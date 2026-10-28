# mujoco_playground_aloha_hand_over-privileged

| | |
|---|---|
| Benchmark | MuJoCo Playground |
| Mode | privileged |
| Twin | `mujoco_playground_aloha_hand_over-standard` |
| Simulator / robot | `mujoco` / `aloha-2` |
| Family / instance | `mujoco_playground/AlohaHandOver` / seed 60417 |
| Oracle | `full` |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:mujoco-0.1.2` |

## Task
MuJoCo Playground's `AlohaHandOver` ported to plain MuJoCo (CPU, deterministic) for this benchmark.

## Difficulty
MuJoCo Playground defines only a shaped RL reward; this task scores a binary success defined from it (at the end the box is within 3 cm of the target, touched by the right gripper's fingers only, off the table), so a trajectory has to actually finish the manipulation rather than collect partial reward.

## Reference solution
`solution/oracle_ik.py`: left top-down grasp, lift to the handover point, right horizontal grasp of the lower half, left release, carry with correction (box 2.5 mm from the target, 650 control steps)

## Verification
Playground has only a shaped RL reward; defined here from it: at the end the box is within 3 cm of the target, touched by the right gripper's fingers only, off the table

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 10 GiB | 8192 | 6 GiB | 10 GiB |
| verifier | no cap | no cap | no cap | 6 GiB | no cap |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU memory 1.0-1.3 GB (per-trial peaks of past trials); memory: privileged container p90 2.6 GB (a per-container memory sampler); CPU p90 1.9 cores (a per-container sampler every 30 s)
- Measured (caps): GPU memory: a per-trial peak of 1.3 GiB -> max(1.5 x peak, peak + 4), rounded up: 6 GiB; processes and disk: defaults, not measured

## Details
| | |
|---|---|
| source | [MuJoCo Playground](https://github.com/google-deepmind/mujoco_playground) @ 4057c14 (Apache-2.0): `manipulation/aloha/xmls/mjx_hand_over.xml`; ALOHA 2 from MuJoCo Menagerie @ 1b86ece |
| physics | MuJoCo 3.3.7, 200 Hz (Playground's sim_dt), implicitfast; control 50 Hz (4 substeps) |
| instance | Playground's reset randomization (box xy ± 5 / 10 cm; target (0.2, 0, 0.25) ± 15 cm, x ≥ 0.15) drawn with seed 60417; initial-state hash in instance.json |
| success | Playground has only a shaped RL reward; defined here from it: at the end the box is within 3 cm of the target, touched by the right gripper's fingers only, off the table |
| deliverable | `(T, 14)` actuator targets per control step, T ≤ 3000; bit-exact replay |

### Change Log
- 2026-10-10 (owner decision): the agent's network egress is restricted to the model APIs (`[agent] network_mode = "allowlist"`, the hosts of `scripts/set_agent_egress.py`; since 2026-10-11 these include the package registries, PyPI and the Debian / Ubuntu archives, so that `pip install` and `apt-get install` work); it was open before (Harbor's default), and agents could download upstream sources, reference solutions and robot models from their own shell. The scene, the frozen instance, the tooling and the pass rule are unchanged, so the task keeps its name; results from before this date were obtained with open egress.
- 2026-10-13: `environment/rcb_aloha.py` and `tests/rcb_aloha.py` change with the tooling they share with `mujoco_playground_aloha_single_peg_insertion` (that task's success rule got its rear bound corrected). The hand-over scene, its success rule and the grader's verdicts are unchanged.
