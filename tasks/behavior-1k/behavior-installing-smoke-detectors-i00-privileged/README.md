# behavior-installing-smoke-detectors-i00-privileged

| | |
|---|---|
| Benchmark | BEHAVIOR-1K |
| Mode | privileged |
| Twin | `behavior-installing-smoke-detectors-i00-standard` |
| Simulator / robot | `behavior` / `r1pro` |
| Family / instance | `installing_smoke_detectors` / seed 301 |
| Oracle | `none` — No reference solution and no verified positive example yet: BEHAVIOR-1K ships no expert or planner that replays on this frozen instance, and no model trial has solved this task. Solvable by a human with this robot: the 2026 BEHAVIOR Challenge publishes 200 teleoperated demonstrations of this activity on the R1Pro in this scene model (86 s on average); demonstration video (68 s): https://www.youtube.com/embed/gRtLcTllXqQ (challenge task list https://behavior.stanford.edu/challenge/tasks/index.html). Supplementary task (added 2026-10-07); human review in PR #75 (protocol §4). |
| Agent time budget | 7200 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe-private:behavior-0.1.2` |

## Task
Install the smoke detector. A fire alarm lies on the table in the living room; attach it to the wall nail in the living room.

**Success**, all at the same time, held for 1 s with no goal object in a gripper and every goal object at rest:
- the fire alarm (`fire_alarm.n.02_1`) is attached to the wall nail (`wall_nail.n.01_1`)

## Difficulty
First-pass rating on the task-list site: medium. The 2026 challenge's 200 teleoperated demonstrations took 86 s on average (2,569 control steps at 30 Hz) and drove 8.61 m; the scene loads 4 rooms, and the goal uses `attached`.

## Reference solution
None (`oracle = "none"`): no reference solution was written (the challenge ships teleop demos, not scripts), and no positive example has been graded by this verifier yet. Evidence that the task is solvable: the 2026 BEHAVIOR Challenge's 200 human teleoperation demonstrations of this activity on the R1Pro (86 s on average; video https://www.youtube.com/embed/gRtLcTllXqQ); `task.toml` `oracle_evidence`.

## Verification
Open-loop replay of `/app/output/trajectory.npz` from `tests/instance.json` in two fresh processes; pass iff the BDDL goal
holds after every replay, the final-state hashes agree and the rebuilt frozen instance matches the recorded one
(`tests/grade.py`), plus `/logs/verifier/replay.mp4`. A fresh process reproduces the live run bit for bit (the replay
starts from the loaded state, no reset); an in-process reset is only approximate, hence two fresh processes. A file the
grader cannot read or with the wrong shape is `bad_trajectory`; a replay that dies or runs out of time, and a frozen
instance that does not rebuild in the verifier (`instance_ok` 0), are `grader_error` + `infra_error` (no verdict on the agent).

No oracle; human review in PR #75 (PROTOCOL §4), on the official human demonstrations and a `nop` run per mode; the task is marked unvalidated (PROTOCOL §2 `validated`).

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 29 GiB | 4096 | 22 GiB | 32 GiB |
| verifier | no cap | no cap | no cap | 22 GiB | no cap |

- GPU: an RTX-class NVIDIA GPU (ray-tracing cores), one of the models in `gpu_types`: L40S, L40, L4, RTX 4090, RTX 6000 Ada, held by `main`.
- Recommended GPU: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: GPU memory 8.3-9.0 GB with RCB_TEXTURE_BUDGET=0.1, the tasks' default (a per-container sampler every 30 s, trials on 14 GPU hosts); CPU 6-9 cores per trial; 13-16 GB of memory per Isaac process
- Measured (caps): per-container peaks in 143 past trials: the agent's container memory 29.0 GiB (OOM-killed at the cap in 2 trials), 1093 threads, CPU at its cap (8), GPU memory 18.6 GiB, disk 30.2 GiB

## Details
| | |
|---|---|
| Instance | 2026 BEHAVIOR Challenge public_test instance 301 (scene `house_double_floor_lower`), state hash in `environment/instance.json` |
| Image | `ghcr.io/mll-lab-nu/efe-private:behavior-0.1.2` = `stanfordvl/behavior:3.9.2` + ffmpeg (`images/behavior/`): the task builds it from public sources, or reuses our prebuilt copy when you can pull it |
| Dataset | the licensed BEHAVIOR dataset, mounted read-only from `$RCB_BEHAVIOR_DATA` (a precondition), never baked in |
| GPU | an RTX-class NVIDIA GPU (`environment/docker-compose.yaml`, `count: 1`) |
| Budget | the agent's time (the `Agent time budget` row; the number is this task's `[agent] timeout_sec`): every Python process pays ~3 min of Isaac Sim + scene load out of it; ~7-11 control steps/s (tasks/behavior-1k/README.md) |
| Verifier time | at most 40,000 rows accepted; each graded replay min(28000 s, max(3600 s, 1200 s + steps / 1.5)), and the video replay after the verdict the same; `[verifier] timeout_sec` 85800 s (three capped replays + 30 min) |
