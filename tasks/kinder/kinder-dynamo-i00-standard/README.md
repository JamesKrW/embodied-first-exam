# kinder-dynamo-i00-standard

| | |
|---|---|
| Benchmark | KinDER |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `kinder-dynamo-i00-privileged` |
| Simulator / robot | `kinder` / `tidybot-plus-plus` |
| Family / instance | `dynamo` / seed 0 |
| Oracle | `full` |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:kinder-0.1.6` |

## Task
Standard mode (robot as a service, `eai-standard/2.1`) of `kinder-dynamo-i00-privileged`: the same KinDER instance (`kinder/Dynamo3D-o3-v0`, seed
0) runs in the `sim` sidecar; the agent gets a client and the robot's own sensors -- two
cameras, joint encoders, the base's odometry -- and
never the scene's state or the goal check. No reset: one episode, which is graded.

Task: drive the robot into the translucent green square marked on the floor; three chairs stand in the way and can be pushed. The task succeeds the moment the centre of the robot's base is inside the square. (The red, blue and yellow squares only mark where the chairs started.)

## Difficulty
Nonprehensile: three chairs stand between the robot and the floor square, and the base can push them. The benchmark's fixture: the pipeline check, with an oracle in both modes.

## Reference solution
`solution/oracle_replay.py` sends the privileged oracle's 101 moves, turned into the odometry frame (`solution/oracle_actions.npy`), through the client: success at step 101, no privileged information at run time.

## Verification
The sidecar judges the episode as it runs (KinDER's goal check at every control step; the episode ends there) and,
when the collect hook ends the trial, writes that verdict to `/data/final.json` (`graded_by = "live"`): nothing is
replayed to grade it (owner, 2026-10-17). The scene's start is checked against the frozen instance's
recorded state (`scene_sha`) when the episode begins (a mismatch is `instance_mismatch`, infrastructure).
`tests/grade.py` grades `final.json` in a container of its own with no network: success = live success. The video
(`replay.mp4`) is rendered from `trajectory.npz` in one pass after the verdict and never changes it.
The oracle (`solution/solve.sh`) scores 1; `nop` scores 0.

Benchmark notes: `tasks/kinder/README.md`.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 16 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 4 | 8 GiB | 8192 | 6 GiB | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: any NVIDIA GPU, held by `sim`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU memory 1.2-1.3 GB (the oracle 1.2 GB on an RTX 4090; per-trial peaks of past trials); memory: sim 1.1 GB (a per-container memory sampler); CPU: an estimate
- Measured (caps): GPU memory (the sim service): a per-trial peak of 1.3 GiB -> max(1.5 x peak, peak + 4), rounded up: 6 GiB; processes and disk: defaults, not measured

## Details

### Change Log
- 2026-10-11: `RCB_RESET_BUDGET`'s default 0 -> 10 (owner decision; PROTOCOL §4.5, tasks/kinder/README.md); the service already had `POST /reset`. The scene, instance, pass rule and verifier are unchanged, so the task keeps its name; runs recorded before this date had no reset and are reported separately.
- 2026-10-16: agent time limit 3600 s -> 1800 s (`[agent] timeout_sec`, both twins; author decision, tasks/kinder/README.md), and `RCB_RESET_BUDGET`'s default 10 -> 0 (author decision): one episode, which the practice and the graded attempt share; the instruction now states where the green square is at the start, measured in the odometry frame. The scene, instance, pass rule and verifier logic are unchanged (`tests/rcb_limits.json` carries the new cutoff), so the task keeps its name; runs recorded before this date are reported separately.
- 2026-10-17 (owner): graded by the live episode: `success` = KinDER's goal check reached during the episode the sim service ran (the same rule, judged as it ran), and no grading replay runs (no `replay_success` / `deterministic` any more; `final.json` has `graded_by = "live"`, `replays = 0`, `reward.json` `graded_by_live = 1`). The video is one render pass over the trajectory after the verdict, as before. The scene's start is checked when the episode begins: a scene that does not build to the frozen instance's recorded state (`scene_sha`) gives up as infrastructure (`instance_mismatch`). The scene, instance, pass rule and the agent's interface are unchanged, so the task keeps its name; every archived verdict is the same under the live rule.
