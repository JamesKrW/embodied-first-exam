# autobio-pipette-i00-standard

| | |
|---|---|
| Benchmark | AutoBio |
| Mode | standard (`eai-standard/2.2`) |
| Twin | `autobio-pipette-i00-privileged` |
| Simulator / robot | `autobio` / `ur5e-dexhand-2f85-bimanual` |
| Family / instance | `pipette` / seed 0 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:autobio-0.1.0` |

## Task
AutoBio's `pipette` (Aspirate with pipette) on the same frozen instance as `autobio-pipette-i00-privileged`: Aspirate from the 50 mL centrifuge tube with the pipette. The test is a sequence: (1) with the plunger pressed (the thumb's target, action entry `arm1_dh_rh_thj3`, above 0.70 rad), the pipette tip is within 6.5 mm (horizontally) of the tube's axis and more than 5 mm below the liquid surface; (2) the plunger is released (the thumb's target below 0.45 rad) while the tip is still below the surface; (3) success the moment, after (1) and (2), the tip is more than 5 cm above the liquid surface. The tube is part-filled (31.5 mL); the cameras show the liquid as a blue surface inside the tube. No liquid actually moves: the test is on positions and commands.
On the bench, between the arms: a rack with an open, part-filled 50 mL tube standing in it. AutoBio's prompt for this task: "one arm lifts centrifuge tube, the other aligns pipette tip and aspirates liquid"; the test itself does not ask for the tube to be lifted. Standard mode (eai-standard/2.2): the agent drives the robot through a client in one episode, with only its proprioception and the scene's cameras.

## Difficulty
Same instance and success rule as `autobio-pipette-i00-privileged` (AutoBio's level: medium). Two arms. The pipette tip has to enter a 50 mL tube within 6.5 mm of its axis and reach below the liquid surface, with the plunger pressed before and released there. In standard mode the agent loses resets, the simulator and the state of the tubes and instruments: it sees camera images and the robot's proprioception, acts in joint-position chunks, and gets one episode, so it has to find the things it handles in the images and recover from its own mistakes without starting over.

## Reference solution
`solution/oracle_stream.py` streams the privileged reference's rows (`solution/oracle.npz`, 6973 rows, float32-exact) through the client in `hold` chunks; it uses no privileged information at run time. The service's recorded trajectory is then exactly the privileged reference: success after 6973 control steps.

## Verification
Graded by the live episode (docs/protocol/README.md H.4): `success` = the live episode's success, judged by the sim service on the episode the agent ran, with no grading replay; an episode that ended in a simulator error scores 0. The service records every executed control step; the collect hook (`environment/sim/finalize.sh`) ends the service, writes `final.json` from the live episode's record and, after the verdict is on disk, replays the recorded trajectory once from the frozen instance to render `replay.mp4` (a failed video replay costs the video only). `tests/grade.py` (separate sandbox, no network) writes the reward fields (`graded_by_live` = 1, `replays` = 0, `infra_error`, the crash-recovery fields). A missing or unfinished `final.json` (the hook never wrote its verdict) and a live episode that did not start in the instance's recorded state are `infra_error = 1`, never the agent's 0. `nop`: the episode times out, reward 0.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 10 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 8 | 15 GiB | 8192 | no GPU | — |
| verifier (as `main`) | 4 | 10 GiB | no cap | no GPU | — |

- GPU: not needed.
- Measured: CPU only; 4 CPUs and 8 GB per trial, the verifier the same (the benchmark's estimate)

## Details
| | |
|---|---|
| Upstream | AutoBio (github.com/autobio-bench/AutoBio @ 0850421, no licence stated: the image is built locally, images/autobio/BUILD_ONLY) on MuJoCo 3.3.0 |
| Service | `environment/sim/server.py` (the adapter) on the shared 2.2 layer `rcb_limited.py` (`embodied_first_exam/standard/`); cameras `table_cam_front`, `table_cam_left`, `arm1_wrist_cam`, `arm2_wrist_cam` (640 x 480); action types `joint_position` and `done`; forward and inverse kinematics of the arm joints |
| Resources | 4 CPUs, no GPU (CPU physics, OSMesa rendering) |

### Change Log
- 2026-10-10: how a trial that could not be graded is recorded, and request checks; the scene, the frozen instance and the pass rule are unchanged, so the task keeps its name. `tests/grade.py`: a missing, unreadable or unfinished `final.json` (the collect hook never wrote its verdict) is `infra_error = 1` (was the agent's 0, or no reward at all); a reward is always written. `environment/sim/finalize.sh`: each verdict replay has a time limit (2400 s; a replay without a result was and is `infra_error`), the video replay gets what is left, `final.json` is written atomically, and the collect hook's `timeout_sec` is 7200 s (was 5400 s). `environment/sim/server.py`: a malformed request field (null or a wrong type for `dt`, `seconds`, `cameras`, `note`) is a 400 before anything is executed (was a 500 that ended the episode). The instruction and `/spec` `episode` now state that a MuJoCo physics warning (an unstable simulation) ends the episode as a failure; the rule itself is unchanged (AutoBio's evaluator's, in force since the task was added). A pass before this change is a pass after it; results recorded before it may hold infrastructure failures counted as the agent's 0.
- 2026-10-10 (review round 2): the client only (`environment/rcb_client.py`). `Robot()` waits up to 3600 s for the service (was 600 s) and prints its progress to stderr once a minute, so the first `Robot()` outlasts the simulator's start (no effect on this benchmark, whose service is up within a minute); `cameras="name"` (a string) is one camera, no longer its characters. The service, the collect hook, the grader, the scene and the pass rule are unchanged.
- 2026-10-10 (review round 3): the shared service layer and the client only (`environment/sim/rcb_limited.py`, `environment/rcb_client.py`). A camera named more than once in `cameras` is rendered and returned once (N copies of one name made the service build N x N images within the frame budget, which could exhaust its memory). A service that has given up on the episode (it could not start it, or could not resume it after its crashes) answers every request with 409 and the reason, and `Robot()` / `wait_ready()` raise `EpisodeOver` at once; before, nothing answered and they waited out their timeout (up to an hour). The adapter, the collect hook, the grader, the scene and the pass rule are unchanged.
- 2026-10-13 (owner): the sim service's clock (`environment/sim/rcb_limited.py`, `environment/sim/server.py`, the docstring of `environment/rcb_client.py`). `meta["time_s"]` and `meta["time_left_s"]` count from the agent's first request to the service (any endpoint), and a restarted service keeps that moment; before, they counted from the service's own start of the episode (when its scene was ready). Where the service is ready before the agent's phase begins (the harness is installed in between: a median of 79 s over the recorded standard trials), that clock ran ahead of the agent's and `time_left_s` reached 0 while the agent still had that time. `/spec` says what the clock counts from (`budget.clock`), and the instruction's `## Time limit` tells the agent to plan by `time_left_s`. The service never ended an episode on this clock: the scene, the frozen instance, the pass rule and the grader are unchanged, so the task keeps its name and earlier results stand.
- 2026-10-22: graded by the live episode (docs/protocol/README.md H.4): `success` = the live episode's success (was live ∧ both fresh-process replays ∧ deterministic); the collect hook no longer replays for the verdict, only once for the video. The scene, the frozen instance and the success rule are unchanged, so the task keeps its name.
- 2026-10-25: an episode that ended in a simulator error scores 0 (docs/protocol/README.md); a live success followed by a simulator error passed before. No recorded result changes: none of them ended in an error.
