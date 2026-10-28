# robocasa365-beverage-sorting-i00-standard

| | |
|---|---|
| Benchmark | RoboCasa365 |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `robocasa365-beverage-sorting-i00-privileged` |
| Simulator / robot | `robosuite` / `franka-panda-omron` |
| Family / instance | `robocasa365/BeverageSorting` / seed 0 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:robocasa365-0.1.0` |

## Task
Standard mode (robot as a service, `eai-standard/2.1`) of `robocasa365-beverage-sorting-i00-privileged`: the same frozen instance and success check, served by the `sim` sidecar (`embodied_first_exam/benchmarks/robocasa/sim/server_casa.py`, tool module `casa_tool`), which records the episode and judges it live (nothing is replayed); the verifier grades the sidecar's record in a container of its own. The agent sees the robot's cameras (RGB-D, calibrated), its own joints and end effector, and the instruction.

## Difficulty
The agent has to find the fixtures and objects in the robot's camera images, drive a mobile manipulator with its native controller, and finish in one episode without a reset.

## Reference solution
`solution/`: the privileged reference's 1816 actions submitted through the client, one row per control step.

## Verification
The sim service judges the live episode when the agent's time is up (the environment's success check on the state after the last executed control step; nothing is replayed) and writes the verdict, final at once; `tests/grade.py` (separate sandbox, no network) reads its `final.json` (`graded_by = live`): `success = live_success ∧ not solved at the start ∧ no simulator error`. The episode's start is checked against the frozen instance before the agent is served (`RCB_INSTANCE_SHA`, `RCB_START_SHA`; a mismatch is an infrastructure failure, `instance_mismatch`). The video is rendered from the trajectory after the verdict (one render pass) and never decides it.

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 6 GiB | 1024 | no GPU | 10 GiB |
| `sim` | 4 | 10 GiB | 1024 | no GPU | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: memory: sim p90 5.6 GB (a per-container memory sampler); CPU only (OSMesa cameras); CPU: an estimate
- Measured (caps): per-container peaks in 124 past trials: the agent's container memory 3.9 GiB, 278 threads, CPU at its cap (4), disk 3.2 GiB; the sim service memory 6.4 GiB, 110 threads, CPU 3.1 cores

## Details
| | |
|---|---|
| Action type | `joint_position` (13 channels, `control_dt` 0.05 s) |
| Cameras | `robot0_agentview_center`, `robot0_agentview_left`, `robot0_agentview_right`, `robot0_eye_in_hand` (robot-mounted; calibration in every response) |
