# behavior-sorting-household-items-i00-privileged

| | |
|---|---|
| Benchmark | BEHAVIOR-1K |
| Mode | privileged |
| Twin | `behavior-sorting-household-items-i00-standard` |
| Simulator / robot | `behavior` / `r1pro` |
| Family / instance | `sorting_household_items` / seed 301 |
| Oracle | `none` — No reference solution and no verified positive example yet: BEHAVIOR-1K ships no expert or planner that replays on this frozen instance, and no model trial has solved this task. Solvable by a human with this robot: the 2026 BEHAVIOR Challenge publishes 200 teleoperated demonstrations of this activity on the R1Pro in this scene model (527 s on average); demonstration video (527 s): https://player.vimeo.com/video/1114059574 (challenge task list https://behavior.stanford.edu/challenge/tasks/index.html). Supplementary task (added 2026-10-04); human review per family (protocol §4). |
| Agent time budget | 7200 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe-private:behavior-0.1.2` |

## Task
Sort the household items into the bathroom. Two baskets stand on the bedroom floor: one holds two bottles of detergent and a box of sanitary napkins, the other a soap dispenser, a tube of toothpaste and a toothbrush; a cup stands on the bathroom sink. Put both detergent bottles under the bathroom sink, next to each other; the box of sanitary napkins on the bathroom shelf; the soap dispenser on the sink; the toothbrush into the cup, which stays on the sink; and the toothpaste next to the cup.

**Success**, all at the same time, held for 1 s with no goal object in a gripper and every goal object at rest:
- both detergent bottles (`bottle__of__detergent.n.01_1`, `_2`) are under the sink (`sink.n.01_1`) and next to each other
- the box of sanitary napkins (`box__of__sanitary_napkin.n.01_1`) is on top of the shelf (`shelf.n.01_1`)
- the soap dispenser (`soap_dispenser.n.01_1`) and the cup (`cup.n.01_1`) are on top of the sink
- the toothbrush (`toothbrush.n.01_1`) is inside the cup and the tube of toothpaste (`tube__of__toothpaste.n.01_1`) is next to the cup

## Difficulty
First-pass rating on the task-list site: hard. The 2026 challenge's 200 teleoperated demonstrations took 527 s on average (15,808 control steps at 30 Hz) and drove 50.91 m; the scene loads 7 rooms, and the goal uses `inside`, `ontop`, `nextto`, `under`.

## Reference solution
None (`oracle = "none"`): no reference solution was written (the challenge ships teleop demos, not scripts), and no positive example has been graded by this verifier yet. Evidence that the task is solvable: the 2026 BEHAVIOR Challenge's 200 human teleoperation demonstrations of this activity on the R1Pro (527 s on average; video https://player.vimeo.com/video/1114059574); `task.toml` `oracle_evidence`.

## Verification
Open-loop replay of `/app/output/trajectory.npz` from `tests/instance.json` in two fresh processes; pass iff the BDDL goal
holds after every replay, the final-state hashes agree and the rebuilt frozen instance matches the recorded one
(`tests/grade.py`), plus `/logs/verifier/replay.mp4`. A fresh process reproduces the live run bit for bit (the replay
starts from the loaded state, no reset); an in-process reset is only approximate, hence two fresh processes. A file the
grader cannot read or with the wrong shape is `bad_trajectory`; a replay that dies or runs out of time, and a frozen
instance that does not rebuild in the verifier (`instance_ok` 0), are `grader_error` + `infra_error` (no verdict on the agent).

No oracle and no verified positive example: the family is reviewed by a maintainer (PROTOCOL §4) in the pull request that adds it, on the official human demonstrations and a `nop` run per mode; the task is marked unvalidated (PROTOCOL §2 `validated`).

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 29 GiB | 16384 | 24 GiB | 20 GiB |
| verifier | no cap | no cap | no cap | 24 GiB | no cap |

- GPU: an RTX-class NVIDIA GPU (ray-tracing cores), one of the models in `gpu_types`: L40S, L40, L4, RTX 4090, RTX 6000 Ada, held by `main`.
- Recommended GPU: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: GPU memory 8.3-9.0 GB with RCB_TEXTURE_BUDGET=0.1, the tasks' default (a per-container sampler every 30 s, trials on 14 GPU hosts); CPU 6-9 cores per trial; 13-16 GB of memory per Isaac process
- Measured (caps): GPU memory: the reference solution alone holds 15.1 GiB for most of the episode on a 48 GB card with RCB_TEXTURE_BUDGET=0.1, the tasks' default (three oracle runs of two tasks on two hosts; trials without the budget reached 29.0 GiB on 48 GB cards) -> max(1.5 x peak, peak + 4), rounded up: 24 GiB; processes and disk: defaults, not measured

## Details
| | |
|---|---|
| Instance | 2026 BEHAVIOR Challenge public_test instance 301 (scene `house_single_floor`), state hash in `environment/instance.json` |
| Image | `ghcr.io/mll-lab-nu/efe-private:behavior-0.1.2` = `stanfordvl/behavior:3.9.2` + ffmpeg (`images/behavior/`): the task builds it from public sources, or reuses our prebuilt copy when you can pull it |
| Dataset | the licensed BEHAVIOR dataset, mounted read-only from `$RCB_BEHAVIOR_DATA` (a precondition), never baked in |
| GPU | an RTX-class NVIDIA GPU (`environment/docker-compose.yaml`, `count: 1`) |
| Budget | the agent's time (the `Agent time budget` row; the number is this task's `[agent] timeout_sec`): every Python process pays ~3 min of Isaac Sim + scene load out of it; ~7-11 control steps/s (tasks/behavior-1k/README.md) |
| Verifier time | at most 40,000 rows accepted; each graded replay min(28000 s, max(3600 s, 1200 s + steps / 1.5)), and the video replay after the verdict the same; `[verifier] timeout_sec` 85800 s (three capped replays + 30 min) |
