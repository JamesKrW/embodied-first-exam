# molmospaces-navigate-to-i00-standard

| | |
|---|---|
| Benchmark | MolmoSpaces |
| Mode | standard (`eai-standard/2.2`) |
| Twin | `molmospaces-navigate-to-i00-privileged` |
| Simulator / robot | `molmospaces` / `rainbow-rby1` |
| Family / instance | `navigate-to` / seed 0 |
| Oracle | `none` — no reference solution (MolmoSpaces' planners and grasp files are not shipped); positive example graded 1 by the separate verifier: molmospaces-navigate-to-i00-priv__YWMd22o (run codex-gpt6_1_sol-high, batch molmo-v2-sol-rby1, 2026-10-16); human review in PR #276 |
| Agent time budget | 3600 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:molmospaces-0.2.5` |

## Task
Standard-mode twin of `molmospaces-navigate-to-i00-privileged` (the same frozen MolmoSpaces episode): Navigate to any tablelamp (2 available). The agent gets only the
`eai-standard/2.2` client (docs/protocol/standard_mode.md); the simulator runs in the `sim` sidecar
(`environment/docker-compose.yaml`), which owns the episode, serves cameras, proprioception and upstream's kinematic
model, and records every executed row. The collect hook (`environment/sim/finalize.sh`) ends the episode, lets the
service exit and writes `final.json` from the live episode's result (nothing is replayed to grade it), then renders
`replay.mp4`; the verifier grades those artifacts in a separate sandbox (`tests/Dockerfile`).

## Difficulty
That of `molmospaces-navigate-to-i00-privileged` (its README), without resets, object poses or a success check: every move is planned from camera
images, depth and the robot's own proprioception, within the benchmark's 100 s of simulated time.

## Reference solution
None is shipped (see `molmospaces-navigate-to-i00-privileged`).

## Verification
`success` = the live episode's success, judged by the sim service (graded live, no replay: 2026-10-17). The episode
follows upstream's evaluation: one upstream `Task.step` per executed row; it ends at upstream's `is_done()`
(500 control steps), at the agent's `done`, or when the agent's time is up; then upstream's
`judge_success()` on the final state: for the one of the 2 target objects nearest to the robot base (horizontal distance), the horizontal distance from the base to its position (the origin of its body, not its nearest surface) is less than 1.5 m and at least one of its pixels shows in head_camera. The service checks the frozen start (the instance's t = 0 state hash)
when it builds the scene, before the agent is served. `replay.mp4` is rendered by one video-only pass from
`trajectory.npz` after the verdict, which it never changes. `infra_error = 1` (no verdict on the agent): no, an
unreadable or an unfinished `final.json`, a service that gave up after crashes between calls or never ended the
episode, a frozen start the image no longer builds (`instance_mismatch`). No oracle; human review in PR #276 (PROTOCOL §4: per family, this task
and its twin, a positive example and the `nop` result).

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 16 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 4 | 12 GiB | 8192 | 7 GiB | — |
| verifier (as `main`) | 8 | 16 GiB | no cap | no GPU | 20 GiB |

- GPU: any NVIDIA GPU, held by `sim`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4 / L40S).
- Measured: GPU memory 2.7 GB on an L40S (per-trial peaks of past trials); memory and CPU estimated (MuJoCo, CPU physics, EGL rendering)
- Measured (caps): GPU memory (the sim service): a per-trial peak of 2.7 GiB on a 48 GB card -> max(1.5 x peak, peak + 4), rounded up: 7 GiB; one trial on a 24 GB card read 18.4 GiB, above the cap (a whole-trial reading on that card, not split by container, so which container held it is not recorded); processes and disk: defaults, not measured

## Details
| | |
|---|---|
| Upstream | molmo-spaces 0.2.9, benchmark `molmospaces-bench-v2/20260415`, package `holodeck-objaverse/NavToObjDataGenConfig/NavToObjHolodeckBench_20260115_json_benchmark` (Navigation, Holodeck houses (MolmoSpaces benchmarks; the ProcTHOR-10k package was built on scene version 20251121, 0.2.9 loads 20251217 and its episodes miss objects)), episode 237 |
| World-frame information | none in the texts: object names and the goal only; the robot's own pose and the camera calibration come from the service (tasks/molmospaces/README.md) |
| Benchmark notes | tasks/molmospaces/README.md |
