# labutopia-shake-i00-standard

| | |
|---|---|
| Benchmark | LabUtopia |
| Mode | standard (`eai-standard/2.2`) |
| Twin | `labutopia-shake-i00-privileged` |
| Simulator / robot | `labutopia` / `franka-panda` |
| Family / instance | `shake` / seed 0 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:labutopia-0.1.0` |

## Task
LabUtopia's `shake` (Shake (LabUtopia's level-2 ShakeBeaker is the same task)) on the same frozen instance as `labutopia-shake-i00-privileged`: Pick up the glass beaker, shake it, and hold it still. Nothing is counted while the beaker's centre is less than 5 cm above where it stood. Success the moment the second of these is met: (1) five times, the beaker's centre moves 5 cm or more (horizontally) from where it was when that count began; (2) after the fifth, the centre stays within 1 cm in each horizontal direction over one second (60 control steps in a row).
On the bench in front of the arm: one small glass beaker, about 6.5 cm across and 10 cm tall. Standard mode (eai-standard/2.2): the agent drives the robot through a client in one episode, with only its proprioception and the scene's cameras.

## Difficulty
Same instance and success rule as `labutopia-shake-i00-privileged` (LabUtopia's level 1). A small beaker has to survive five quick moves of 5 cm in the grip and then be held within 1 cm for a second: the grasp has to be firm enough for the accelerations. In standard mode the agent loses resets, the simulator and the state of the objects: it sees camera images (glassware is nearly transparent in them and absent from the depth maps) and the robot's proprioception, acts in joint target chunks, and gets one episode, so it has to find the things it handles in the images and recover from its own mistakes without starting over.

## Reference solution
`solution/oracle_stream.py` streams the privileged reference's rows (`solution/oracle.npz`, 1087 rows, float32) through the client as `joint_position_velocity` chunks; it uses no privileged information at run time. The service's recorded trajectory is then exactly the privileged reference: success after 1087 control steps.

## Verification
Graded by the live episode (docs/protocol/README.md H.4): `success` = the live episode's success, judged by the sim service on the episode the agent ran, with no grading replay; an episode that ended in a simulator error scores 0. The service records every executed control step; the collect hook (`environment/sim/finalize.sh`) ends the service, writes `final.json` from the live episode's record and, after the verdict is on disk, replays the recorded trajectory once from the frozen instance to render `replay.mp4` (a failed video replay costs the video only). `tests/grade.py` (separate sandbox, no network) writes the reward fields (`graded_by_live` = 1, `replays` = 0, `infra_error`, the crash-recovery fields). A missing or unfinished `final.json` (the hook never wrote its verdict) and a live episode that did not start in the instance's recorded state are `infra_error = 1`, never the agent's 0. `nop`: the episode times out, reward 0.

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 32 GiB | 16384 | no GPU | 20 GiB |
| `sim` | 16 | 32 GiB | 16384 | 16 GiB | — |
| verifier (as `main`) | 8 | 32 GiB | — | no GPU | — |

- GPU: an RTX-class NVIDIA GPU (ray-tracing cores, for Isaac Sim / Omniverse RTX rendering; `gpu_types`: RTX 4090, L40S), held by `sim`.
- Recommended GPU: RTX 4090 24GB / L40S 48GB.
- Measured: oracle checks on RTX 4090 (driver 580), lab_001 pick: rendering container peak 17 GB RAM, about 7 GB GPU, 8 CPUs (start-up bursts to 16 cores for about 70 s); author: lab_003 families 21 GB at the peak (8 vCPU host)
- Measured (caps): GPU: about 7 GiB, oracle on an RTX 4090 (author); no trial data -> max(1.5 x peak, peak + 4), at least 16 for Isaac Sim; memory: no samples of the agent's container; floor 8 GiB (at most the 32 GiB cap); whole trial: + the sim service's 20 GiB (docker peak 12.7 GiB, 1 samples); processes 512 and disk 20 GiB: design defaults, not yet measured

## Details
| | |
|---|---|
| Upstream | LabUtopia (github.com/Rui-li023/LabUtopia @ 8df7278; code MIT, assets CC BY-NC 4.0) on NVIDIA Isaac Sim 6.1.0 (LabUtopia pins 5.1.0; the image is built locally, images/labutopia/BUILD_ONLY) |
| Service | `environment/sim/server.py` (the adapter) on the shared 2.2 layer `rcb_limited.py` (`embodied_first_exam/standard/`); cameras `camera_1`, `camera_2`, `wrist_cam` (512 x 512); action types `joint_position` (9) and `joint_position_velocity` (18); forward and inverse kinematics of the arm |
| Resources | an RTX-class NVIDIA GPU for the `sim` sidecar (Isaac Sim), 8 CPUs, 32 GB of RAM |

### Change Log
- 2026-10-10: how a trial that could not be graded is recorded, and request checks; the scene, the frozen instance and the pass rule are unchanged, so the task keeps its name. `tests/grade.py`: a missing, unreadable or unfinished `final.json` (the collect hook never wrote its verdict) is `infra_error = 1` (was the agent's 0, or no reward at all); a reward is always written. `environment/sim/finalize.sh`: each verdict replay has a time limit (2400 s; a replay without a result was and is `infra_error`), the video replay gets what is left, `final.json` is written atomically, and the collect hook's `timeout_sec` is 9000 s (was 5400 s). `environment/sim/server.py`: a malformed request field (null or a wrong type for `dt`, `seconds`, `cameras`, `note`) is a 400 before anything is executed (was a 500 that ended the episode). A pass before this change is a pass after it; results recorded before it may hold infrastructure failures counted as the agent's 0.
- 2026-10-10 (review round 2): texts and the client only. The renderer's first start is stated as measured (5 to 27 minutes by stage and host: 26-27 minutes for the `lab_003` stage on an 8-vCPU L40S host, where the texts said about 15); `Robot()` waits up to 3600 s for the service (was 600 s, so the instruction said `Robot(timeout=2400)`) and prints its progress once a minute; `cameras="name"` (a string) is one camera, no longer its characters. `pour` only: the instruction says that the last condition counts 84 control steps in all, not in a row (what the rule always did). The service, the collect hook, the grader, the scene and the pass rule are unchanged.
- 2026-10-10 (review round 3): the shared service layer and the client only (`environment/sim/rcb_limited.py`, `environment/rcb_client.py`). A camera named more than once in `cameras` is rendered and returned once (N copies of one name made the service build N x N images within the frame budget, which could exhaust its memory). A service that has given up on the episode (it could not start it, or could not resume it after its crashes) answers every request with 409 and the reason, and `Robot()` / `wait_ready()` raise `EpisodeOver` at once; before, nothing answered and they waited out their timeout (up to an hour). The adapter, the collect hook, the grader, the scene and the pass rule are unchanged.
- 2026-10-13 (owner): the sim service's clock (`environment/sim/rcb_limited.py`, `environment/sim/server.py`, the docstring of `environment/rcb_client.py`). `meta["time_s"]` and `meta["time_left_s"]` count from the agent's first request to the service (any endpoint), and a restarted service keeps that moment; before, they counted from the service's first start in its container (`time_left_s`) and from when its scene was ready (`time_s`). Where the service is ready before the agent's phase begins (the harness is installed in between: a median of 79 s over the recorded standard trials), that clock ran ahead of the agent's and `time_left_s` reached 0 while the agent still had that time. `/spec` says what the clock counts from (`budget.clock`), and the instruction's `## Time limit` tells the agent to plan by `time_left_s`. The service never ended an episode on this clock: the scene, the frozen instance, the pass rule and the grader are unchanged, so the task keeps its name and earlier results stand.
- 2026-10-10 (review round 2, shared framework): the `sim` sidecar's docker memory cap is 32 GiB (was 24 GiB, below the 27 GB this task declares as the verifier phase's minimum: a simulator killed at that cap inside a call would have been counted as the agent's failure). Nothing the agent sees or the grader decides changes (`scripts/task_resources.toml`, `[metadata.resources.caps]`).
- 2026-10-22: graded by the live episode (docs/protocol/README.md H.4): `success` = the live episode's success (was live ∧ both fresh-process replays ∧ deterministic); the collect hook no longer replays for the verdict, only once for the video. The scene, the frozen instance and the success rule are unchanged, so the task keeps its name.
- 2026-10-25: an episode that ended in a simulator error scores 0 (docs/protocol/README.md); a live success followed by a simulator error passed before. No recorded result changes: none of them ended in an error.
