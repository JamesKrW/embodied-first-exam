# tactilelab-dough-press-thickness-i00-privileged

| | |
|---|---|
| Benchmark | TactileLab |
| Mode | privileged |
| Twin | — (privileged only) |
| Simulator / robot | `tactilelab` / `xtrainer-gsmini` |
| Family / instance | `dough_press_thickness` / seed 0 |
| Oracle | `full` — solution/: the scripted demonstration policy of this task, recorded through the Session; graded 1 by two fresh-process replays on an L40S (both solved; their final states also agreed in that run). |
| Agent time budget | 3600 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe-private:tactilelab-0.2.0` |

## Task
Pinch the dough ridge to the target thickness at two places and lift the hand.

## Difficulty
Contact-rich manipulation with a tactile gripper, driven through end-effector targets only.

## Reference solution
`solution/oracle_dough_press_thickness.py`: the scripted demonstration policy, recorded through the Session; `solution/solve.sh` lays the policy code over the package first (the agent's image does not carry it).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from `tests/instance.json` in two fresh processes (`tests/grade.py`); pass iff the task's success predicate holds in both replays (the Newton sidecar's replays are not compared bit for bit; `deterministic` and `replay_spread` report how close they ended), and the rebuilt frozen scene matches the recorded one (robot, rigid objects and a sample of the deformable body's particles within tolerance). A simulator error on the agent's rows scores 0 (`sim_error`); the video is rendered in a third replay after the verdict.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 16 GiB | 16384 | 34 GiB | 20 GiB |
| verifier | 8 | 19 GiB | no cap | 34 GiB | 20 GiB |

- GPU: an RTX-class NVIDIA GPU (ray-tracing cores), one of the models in `gpu_types`: L40S, L40, RTX 6000 Ada, held by `main`.
- Recommended GPU: L40S 48GB.
- Measured: on an L40S with efe-private:tactilelab-0.2.0, cgroup memory.peak and the GPU memory of the container's own processes (nvidia-smi, 1 s samples); the agent's container = the oracle with the self-check CLI replay running beside it (two Isaac Sim processes, and two Newton sidecars for a deformable task), the verifier = two fresh-process replays and the video replay, one after the other. Agent's container (memory / GPU memory / mean cores): insert_hdmi 7.3 GiB / 5.7 GiB / 7, cloth_fold_half 8.2 / 6.7 / 5, cable_needle_thread 8.2 / 6.7 / 5, dough_press_thickness 9.1 / 22.3 / 7; verifier: insert_hdmi 12.2 / 5.6 / 6, cloth_fold_half 12.1 / 6.1 / 4, dough_press_thickness 12.4 / 13.9 / 6; the verifier's own caps ([verifier.environment], set in each task): 8 CPUs, 19 GiB
- Measured (caps): GPU memory: the implicit-MPM dough sidecar holds about 10 GiB per Session; 22.3 GiB with the oracle and its self-check replay -> max(1.5 x peak, peak + 4), rounded up: 34 GiB, so a 48 GB card; memory: 9.1 GiB -> 14 GiB, set to 16 GiB; processes and disk: defaults, not measured

## Details
| | |
|---|---|
| Simulator | TactileLab tasks on Isaac Lab 2.3.2 / Isaac Sim 5.1 (PhysX on the GPU, RTX rendering), X-Trainer with two GelSight Mini sensors (taxel raycast + Taxim look-up table) |
| Task | `dough_press_thickness`; the frozen instance (the scene state after reset, with its hash) is `environment/instance.json` |
| Image | built locally on every host (`images/tactilelab/BUILD_ONLY`: it contains Isaac Sim); needs an RTX-class NVIDIA GPU (`environment/docker-compose.yaml`) |
| Budget | the agent's cutoff is the `Agent time budget` row above (this task's `[agent] timeout_sec`); every Python process pays ~1 min of Isaac Sim launch + scene build (tasks/tactilelab/README.md) |
| Benchmark notes | tasks/tactilelab/README.md |
