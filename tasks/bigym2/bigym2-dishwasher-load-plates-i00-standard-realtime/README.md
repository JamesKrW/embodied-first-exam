# bigym2-dishwasher-load-plates-i00-standard-realtime

| | |
|---|---|
| Benchmark | BiGym 2.0 |
| Mode | standard-realtime (`eai-standard/2.3`) |
| Twin | `bigym2-dishwasher-load-plates-i00-privileged` |
| Simulator / robot | `bigym2` / `unitree-g1-dex1` |
| Family / instance | `dishwasher-load-plates` / seed 0 |
| Oracle | `none` — no reference solution (BiGym 2.0 ships no expert or planner; its demonstrations and its agents' programs stay off the agent's network); no positive example yet; human review in PR #123 |
| Agent time budget | 3600 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Real time | slowdown S = 1 (the world runs at 1/S of real speed: one control step every S x control_dt s of wall time; `RCB_SLOWDOWN`'s default); reset budget 0 (`RCB_RESET_BUDGET`'s default) (PROTOCOL §3) |
| Base image | `ghcr.io/mll-lab-nu/efe:bigym2-0.1.1` |

## Task
Take the plates from the dish rack and gently slot them into the dishwasher's lower rack. (BiGym 2.0 `dishwasher_load_plates`; the robot as a service on a clock that does not stop: its three onboard cameras and
its proprioception only, one episode.)

## Difficulty
Two plates moved from a drainer into the dishwasher's low rack and stood on edge; a dropped plate ends the episode. In standard-realtime mode the robot keeps running at 50 Hz while the agent thinks, holding its last
command (standing still), so the agent drives it from a controller program of its own.

## Reference solution
None (as `bigym2-dishwasher-load-plates-i00-privileged`).

## Verification
As `bigym2-dishwasher-load-plates-i00-standard`: the sidecar judges the live episode with BiGym's own success check and writes that verdict to
`/data/final.json` (`graded_by = "live"`); nothing is replayed to grade it (owner, 2026-10-17). The
trajectory holds the commands the simulator executed at each control step, so it reproduces the live run whatever the
timing was; the video is rendered from it in one pass after the verdict. `tests/grade.py` reads `final.json`. After
the agent's first command, a simulator error raised by a control step and a simulator crash inside a control step (a
fatal signal) are the agent's 0; before it, a kill, a failed resume and a reset that does not rebuild the scene are infrastructure
(`infra_error = 1`; owner, 2026-10-17, docs/protocol/standard_mode.md). Success condition: each of the two plates stands on edge in the dishwasher's lower rack: its face toward your left or right (its axis within 20 degrees of the robot's left or right at the start), touching the lower rack, held by neither gripper; both plates at once.
A plate touching the floor ends the episode as a failure. No oracle; human review in PR #123 (PROTOCOL §4).

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 4 GiB | 1024 | no GPU | 10 GiB |
| `sim` | 2 | 6 GiB | 1024 | 4 GiB | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: any NVIDIA GPU, held by `sim`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU memory 0.7-1.2 GB (per-trial peaks of past trials); memory: no samples of the sim service or the agent's container (the privileged twin's container, which runs the same simulator, 3.0 GB; a per-container memory sampler); CPU: an estimate (MuJoCo and two ONNX policies on the CPU)
- Measured (caps): per-container peaks in 60 past trials: the agent's container memory 0.4 GiB, 261 threads, CPU at its cap (4), disk 1.0 GiB; the sim service memory 0.9 GiB (3.0 GiB in an earlier measurement), 50 threads, CPU 0.9 cores, GPU memory 0.6 GiB (1.2 GiB in an earlier measurement)

## Details
| | |
|---|---|
| Service | `environment/sim/rcb_realtime.py` (eai-standard/2.3) with the BiGym 2.0 adapter `server.py`, `rcb_bigym2.py`, `instance.json` (= the privileged twin's) |
| Clock | real time (`RCB_SLOWDOWN`, default 1) from the first `submit` |
| Horizon | 3150 control steps (63 s at 50 Hz) |
