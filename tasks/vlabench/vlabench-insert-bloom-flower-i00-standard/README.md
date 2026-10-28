# vlabench-insert-bloom-flower-i00-standard

| | |
|---|---|
| Benchmark | VLABench |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `vlabench-insert-bloom-flower-i00-privileged` |
| Simulator / robot | `vlabench` / `franka-panda` |
| Family / instance | `vlabench/insert_bloom_flower` / seed 0 |
| Oracle | `full` |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:vlabench-0.2.0` |

## Task
VLABench's `insert_bloom_flower` as a robot service: a Franka Panda arm at a table, instructed **"Insert the tulip into the vase_seen."**
(a direct command), on the same frozen instance as `vlabench-insert-bloom-flower-i00-privileged`. The simulator runs in the
`sim` sidecar (`environment/sim/server.py` on main's `rcb_service.py`); the agent only has the client: four cameras
(RGB 320×320, depth on request, calibrated), the robot's own state, and joint-position chunks. One episode, no reset,
no object poses; nothing to hand in.

## Difficulty
The agent has to find the objects in the images, close the loop through the cameras with joint-position targets, and
finish in one episode without a reset, on top of the task's own reasoning (a direct command).

## Reference solution
`solution/`: the privileged oracle's 205 control steps (recorded offline; see the twin's README) submitted
through the client; the episode ends with `reason = "success"` at the last of them.

## Verification
The sim service ends the episode at the task's success (or at the 9000-step technical cap, when the agent's
time is up, or on a simulator error) and writes that live verdict to `/data/final.json` (`graded_by = "live"`);
nothing is replayed to grade it (owner, 2026-10-17). The scene is checked against the frozen instance (its
initial state and compiled model hashes) when it is built, and its start state when the episode begins (a mismatch is
`instance_mismatch`, infrastructure). `tests/grade.py` (separate sandbox, no network) grades `final.json`:
`success = live_success`. The video (`replay.mp4`) is rendered from `trajectory.npz` in one pass after the verdict and
never changes it.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 16 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 4 | 15 GiB | 8192 | no GPU | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: memory: sim p90 6.9 GB (a per-container memory sampler); CPU only; CPU: an estimate
- Measured (caps): processes and disk: defaults, not measured

## Details
| | |
|---|---|
| Actions | `joint_position` (9, float64): absolute targets for the 7 arm joints (rad) and the 2 fingers (m), 0.1 s per control step |
| Episode cap | 9000 control steps (900 s of robot time; `spec.max_control_steps`, PROTOCOL §4.5 MAY) |
| Resources | 4 CPUs, no GPU (CPU physics, OSMesa rendering) |

### Change Log
- 2026-10-16: the sim collect hook waits for the replay video until 2880 s after the episode ends (was 1470 s; `timeout_sec` 1500 -> 2910); the verdict keeps its 1440 s bound. The video replays the graded episode once more after the verdict, so a slow verdict left it no time. Video only: the verdict, the physics and the pass rule are unchanged, so results from before this date stay valid.
- 2026-10-17 (owner): graded by the live episode: `success` = VLABench's own success check reached in the episode the sim service ran (the same rule, judged as it ran), and no grading replay runs (no `replay_success` / `deterministic` any more; `final.json` has `graded_by = "live"`, `replays = 0`, `reward.json` `graded_by_live = 1`). The video is one render pass over the trajectory after the verdict, as before. The scene's start is checked when the episode begins: a scene that does not rebuild the frozen instance gives up as infrastructure (`instance_mismatch`). The scene, instance, pass rule, step cap and the agent's interface are unchanged, so the task keeps its name; archived verdicts are the same under the live rule, except 5 trials whose old video render ran out of time (recorded `infra_error`), which the live rule grades (all 0).
- 2026-10-19 (in place: a new DIGEST, PROTOCOL H.8): the VLABench image is built by the task itself when the runner cannot pull it. The sim service's Dockerfile (`environment/sim/Dockerfile`) start with its recipe (`images/vlabench/Dockerfile`, stage `vlabench-base`) and compose `build.cache_from` names `ghcr.io/mll-lab-nu/efe-private:vlabench-0.2.0`, the same recipe prebuilt (Debian packages from a dated snapshot, the same package versions as before); the environment's build time limit is 3600 s (was 1800 s). The scene, the frozen instance, VLABench's code and assets, the grading and the time limit are unchanged.
