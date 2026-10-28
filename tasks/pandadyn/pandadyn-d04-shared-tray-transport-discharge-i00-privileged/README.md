# pandadyn-d04-shared-tray-transport-discharge-i00-privileged

| | |
|---|---|
| Benchmark | Panda Dynamic Scenes |
| Mode | privileged |
| Twin | `pandadyn-d04-shared-tray-transport-discharge-i00-standard` |
| Simulator / robot | `maniskill` / `franka-panda-bimanual` |
| Family / instance | `d04_shared_tray_transport_discharge` / seed 1 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:pandadyn-0.2.0` |

## Task
The two Pandas hold one tray between them. Each arm's small open fork cradles a tab on one end of the tray, and an orange ball lies loose in the tray. Working together, the arms must first hold the tray steadily. They then carry it through two waypoints that make a turn. When the environment pushes the ball, they must bring it back to rest near the tray centre. Next they bring the tray to an exit position beside a fixed basin, and finally they tip the ball out so that it settles in the basin. Both forks must support the tray during the transport phases, the ball must not leave the tray before the discharge, and the tray must never drop low. The deliverable is one replayable trajectory of 100 Hz joint-target actions (`/app/output/trajectory.npz`,
16 values per row), graded by the task's own staged success check: 5 phases (`shared_support`, `transport_turn`, `disturbance_recovery`, `exit_alignment`, `natural_discharge`), in order,
with no failure event and no safety-limit violation, within the task's horizon of 25 s.

## Difficulty
The two arms form a closed chain through a tray that is only loosely cradled, so they must move in lockstep or the tray slips out of a fork. Meanwhile the ball rolls freely in a shallow tray and is pushed once on purpose. The final tilt has to pour the ball into the basin without letting the tray sink below 0.17 m. Privileged mode gives the full simulator state (every object's pose and velocity, the fixtures, the
announced goals), the parameters, unlimited exact resets and snapshots, and self-replay; the tooling offers only joint
targets and kinematics. This mode is the control for the standard twin.

## Reference solution
`solution/oracle_actions.npy` (560 control steps, run by `solution/oracle.py` through the agent's own tooling): a
recorded rollout of this instance by the benchmark's reference controller, which reads the simulator's true state in closed
loop, replayed open-loop. The reference controllers and their recordings are not in the image, and the agent's network
reaches only the model APIs and the package registries.

## Verification
`tests/grade.py` in a separate sandbox: two replays, each in a fresh process, from the verifier's own copy of the instance
(`tests/instance.json`; the rebuilt start is checked against its hash); pass iff the task's success check holds in both
(every phase complete, no failure) and both end in the same state (`deterministic`), plus `/logs/verifier/replay.mp4`
(overview camera). Oracle: 560 control steps (5.6 s simulated), success at the last one; the oracle
run and two fresh-process replays end in the same state (hash `faaa539627e671af`), and the rollout reproduces the physical
state the benchmark recorded for it after every control step, bit for bit. `nop`: no trajectory, reward 0; a trajectory that
holds the arm still fails (the benchmark's parked controls never succeeded).

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 20 GiB | 8192 | 5 GiB | 10 GiB |
| verifier | 4 | 8 GiB | no cap | 5 GiB | — |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: any 8 GB+ NVIDIA GPU with Vulkan (RTX 4090 / L4).
- Measured: 50 oracle trials, 25 per mode (the trial's resource monitor every 5 s, RTX 4090): privileged trial peak 0.79 GiB (anon + shmem), GPU memory 0.05 GiB; standard sim service peak 0.94 GiB, GPU memory 0.35 GiB, agent container 0.03 GiB; CPU p90 about 1 core per container; one filmed scene (render_video, S12 and D07): RSS 1.2-1.3 GiB, GPU memory 0.25 GiB; CPU: one core per scene (CPU PhysX, single-threaded) plus one for the agent
- Measured (caps): GPU memory: 0.25 GiB per scene -> max(1.5 x peak, peak + 4), rounded up: 5 GiB, room for a few scenes at once

## Details
| | |
|---|---|
| Benchmark source | Panda Dynamic Scenes (the environment package `images/pandadyn/panda_dynamics/`, from `images/pandadyn/sync_env.py`); task `PandaMultiStage/D04-SharedTrayTransportDischarge-v0` |
| Instance | difficulty `nominal`, seed 1, the resolved parameters and the start's state hash in `environment/instance.json` (`initial_sha` 6c202dbdc4b8f2a6) |
| Actions | 16 per control step (10 ms): per arm 7 absolute joint targets (rad) + the total aperture (m); a Ruckig + torque servo per arm |
| Physics | CPU PhysX at 1000 Hz with enhanced determinism (single thread); SAPIEN renders on the GPU only when a frame is asked for |
| Image | `images/pandadyn/` (public); needs an NVIDIA GPU with Vulkan for SAPIEN (`environment/docker-compose.yaml`) |
| Budget | the agent's cutoff is the `Agent time budget` row above (this task's `[agent] timeout_sec`); a scene builds in ~5-15 s |

### Provenance
Scene description `D04`, frozen instance `D04-SharedTrayTransportDischarge-i00`, chosen and frozen in `ghcr.io/mll-lab-nu/efe:pandadyn-0.2.0`.

### Change Log
- 2026-10-23 (owner): pandadyn 0.2.0, base image `efe:pandadyn-0.2.0`. Unused configuration parameters are gone;
  scene, physics and checks are unchanged. Frozen instance: seed 1, `initial_sha` 6c202dbdc4b8f2a6; reference rows:
  560 control steps (the same start and reference rows as before). Results from before are under the earlier digest
  and are not ranked with these.
