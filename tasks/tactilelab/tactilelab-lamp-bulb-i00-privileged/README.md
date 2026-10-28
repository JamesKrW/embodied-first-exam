# tactilelab-lamp-bulb-i00-privileged

| | |
|---|---|
| Benchmark | TactileLab |
| Mode | privileged |
| Twin | — (privileged only) |
| Simulator / robot | `tactilelab` / `xtrainer-gsmini` |
| Family / instance | `lamp_bulb` / seed 0 |
| Oracle | `full` — solution/: the scripted demonstration policy of this task, recorded through the Session; graded 1 by two fresh-process replays on an L40S, which ended in the same state. |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:tactilelab-0.2.0` |

## Task
Seat the bulb in the lamp base's socket and the hood on top.

## Difficulty
Contact-rich manipulation with a tactile gripper, driven through end-effector targets only.

## Reference solution
`solution/oracle_lamp_bulb.py`: the scripted demonstration policy, recorded through the Session; `solution/solve.sh` lays the policy code over the package first (the agent's image does not carry it).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from `tests/instance.json` in two fresh processes (`tests/grade.py`); pass iff the task's success predicate holds in every replay and the final-state hashes agree, and the rebuilt frozen scene matches the recorded one. A simulator error on the agent's rows scores 0 (`sim_error`); the video is rendered in a third replay after the verdict.

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 8 | 6 |
| Memory | 13 GB | 19 GB |
| GPU memory | 11 GB | 10.5 GB |

- GPU: required, an RTX-class NVIDIA GPU (ray-tracing cores, for Isaac Sim / Omniverse RTX rendering); recommended: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: agent container = the oracle with the self-check CLI replay running beside it in the same container (two Isaac Sim processes, and two Newton sidecars for a deformable task), verifier = two fresh-process replays and the video replay; cgroup memory.peak and the summed GPU memory of the container's own processes (nvidia-smi, 1 s samples) on an L40S, efe-private:tactilelab-0.2.0: agent insert_hdmi 7.3 GiB / 5.7 GiB GPU / 7 cores, cloth_fold_half 8.2 / 6.7 / 5, cable_needle_thread 8.2 / 6.7 / 5, dough_press_thickness 8.5 / 22.2 / 7; verifier insert_hdmi 12.2 / 5.6 / 6, cloth_fold_half 12.1 / 6.1 / 4, dough_press_thickness 11.5 / 11.1 / 5 (GiB / GiB GPU / cores); rounded up from max(1.5 x peak, peak + 4)
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), every container of the trial: 24 GiB GPU memory, 24 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 24 GiB GPU memory, 24 GiB memory. Limits measured: agent container = the oracle with the self-check CLI replay running beside it in the same container (two Isaac Sim processes, and two Newton sidecars for a deformable task), verifier = two fresh-process replays and the video replay; cgroup memory.peak and the summed GPU memory of the container's own processes (nvidia-smi, 1 s samples) on an L40S, efe-private:tactilelab-0.2.0: agent insert_hdmi 7.3 GiB / 5.7 GiB GPU / 7 cores, cloth_fold_half 8.2 / 6.7 / 5, cable_needle_thread 8.2 / 6.7 / 5, dough_press_thickness 8.5 / 22.2 / 7; verifier insert_hdmi 12.2 / 5.6 / 6, cloth_fold_half 12.1 / 6.1 / 4, dough_press_thickness 11.5 / 11.1 / 5 (GiB / GiB GPU / cores); rounded up from max(1.5 x peak, peak + 4).

## Details
| | |
|---|---|
| Simulator | TactileLab tasks on Isaac Lab 2.3.2 / Isaac Sim 5.1 (PhysX on the GPU, RTX rendering), X-Trainer with two GelSight Mini sensors (taxel raycast + Taxim look-up table) |
| Task | `lamp_bulb`; the frozen instance (the scene state after reset, with its hash) is `environment/instance.json` |
| Image | built locally on every host (`images/tactilelab/BUILD_ONLY`: it contains Isaac Sim); needs an RTX-class NVIDIA GPU (`environment/docker-compose.yaml`) |
| Budget | the agent's cutoff is the `Agent time budget` row above (this task's `[agent] timeout_sec`); every Python process pays ~1 min of Isaac Sim launch + scene build (tasks/tactilelab/README.md) |
| Benchmark notes | tasks/tactilelab/README.md |
