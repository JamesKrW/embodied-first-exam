# pandadyn-d05-moving-ramp-relay-i00-privileged

| | |
|---|---|
| Benchmark | Panda Dynamic Scenes |
| Mode | privileged |
| Twin | `pandadyn-d05-moving-ramp-relay-i00-standard` |
| Simulator / robot | `maniskill` / `franka-panda-bimanual` |
| Family / instance | `d05_moving_ramp_relay` / seed 0 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:pandadyn-0.2.0` |

## Task
An orange ball rests in the left Panda's cup, and the right Panda holds a straight channel that is open at both ends. The left arm must pass the ball into the inlet part of the channel. The ball must then leave the channel by its outlet end while the outlet is moving (at least 0.10 m/s) and be caught again in free flight by the left cup. The relay is then repeated with the channel held at a clearly different orientation from the first time. After the second catch, the left cup must deliver the ball into a fixed basin, where it must come to rest. The deliverable is one replayable trajectory of 100 Hz joint-target actions (`/app/output/trajectory.npz`,
16 values per row), graded by the task's own staged success check: 6 phases (`first_channel_entry`, `first_moving_exit`, `first_catch`, `second_channel_entry`, `second_moving_exit`, `second_catch_and_delivery`), in order,
with no failure event and no safety-limit violation, within the task's horizon of 25 s.

## Difficulty
Each relay hands the ball from cup to a hand-held channel. The ball then rolls out by itself while the channel's outlet has to be moving, and the same cup that gave it away has to catch it in free flight off the outlet. The second relay must use a channel orientation at least 0.30 rad different from the first, and the basin delivery counts only after the second catch has been held. Privileged mode gives the full simulator state (every object's pose and velocity, the fixtures, the
announced goals), the parameters, unlimited exact resets and snapshots, and self-replay; the tooling offers only joint
targets and kinematics. This mode is the control for the standard twin.

## Reference solution
`solution/oracle_actions.npy` (2393 control steps, run by `solution/oracle.py` through the agent's own tooling): a
recorded rollout of this instance by the benchmark's reference controller, which reads the simulator's true state in closed
loop, replayed open-loop. The reference controllers and their recordings are not in the image, and the agent's network
reaches only the model APIs and the package registries.

## Verification
`tests/grade.py` in a separate sandbox: two replays, each in a fresh process, from the verifier's own copy of the instance
(`tests/instance.json`; the rebuilt start is checked against its hash); pass iff the task's success check holds in both
(every phase complete, no failure) and both end in the same state (`deterministic`), plus `/logs/verifier/replay.mp4`
(overview camera). Oracle: 2393 control steps (23.93 s simulated), success at the last one; the oracle
run and two fresh-process replays end in the same state (hash `93320a39eab8394e`), and the rollout reproduces the physical
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
| Benchmark source | Panda Dynamic Scenes (the environment package `images/pandadyn/panda_dynamics/`, from `images/pandadyn/sync_env.py`); task `PandaMultiStage/D05-MovingRampRelay-v0` |
| Instance | difficulty `nominal`, seed 0, the resolved parameters and the start's state hash in `environment/instance.json` (`initial_sha` 211481317fc25fe9) |
| Actions | 16 per control step (10 ms): per arm 7 absolute joint targets (rad) + the total aperture (m); a Ruckig + torque servo per arm |
| Physics | CPU PhysX at 1000 Hz with enhanced determinism (single thread); SAPIEN renders on the GPU only when a frame is asked for |
| Image | `images/pandadyn/` (public); needs an NVIDIA GPU with Vulkan for SAPIEN (`environment/docker-compose.yaml`) |
| Budget | the agent's cutoff is the `Agent time budget` row above (this task's `[agent] timeout_sec`); a scene builds in ~5-15 s |

### Provenance
Scene description `D05`, frozen instance `D05-MovingRampRelay-i00`, chosen and frozen in `ghcr.io/mll-lab-nu/efe:pandadyn-0.2.0`.

### Change Log
- 2026-10-23 (owner): pandadyn 0.2.0, base image `efe:pandadyn-0.2.0`. Stricter check: the channel's outlet must be
  moving at 0.10 m/s or more when the ball leaves it (the threshold was 0). The ball's restitution is a named
  parameter (same value); unused parameters are gone. Frozen instance: seed 0, `initial_sha` 211481317fc25fe9;
  reference rows: 2393 control steps (the same start; new reference rows). Results from before are under the earlier
  digest and are not ranked with these.
