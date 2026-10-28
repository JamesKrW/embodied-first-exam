# ambench-press-button-fahexa-i00-standard

| | |
|---|---|
| Benchmark | AM-Bench |
| Mode | standard (`eai-standard/2.2`) |
| Twin | `ambench-press-button-fahexa-i00-privileged` |
| Simulator / robot | `ambench` / `ambench-fahexa` |
| Family / instance | `press_button_fahexa` / seed 0 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:ambench-0.3.1` |

## Task
Standard-mode twin of `ambench-press-button-fahexa-i00-privileged` (the same frozen instance: AM-Bench environment `PressButton-Am-FAHexa-Abs-PID-Direct-v0`, its seeded reset with seed
0): A wall stands in front of the hovering aerial manipulator, and a push button is mounted flat on the wall's front face, facing the robot: a red circular plunger in a dark-grey circular housing. Press the button, i.e. push its spring-loaded plunger into the housing, along the wall's normal, until it is depressed by at least 4 mm (40 % of its 10 mm travel). The agent gets only the `eai-standard/2.2` client (docs/protocol/standard_mode.md); the simulator runs in
the `sim` sidecar (`environment/docker-compose.yaml`, GPU on the sidecar), which owns the scene and records the episode as a
(T, 8) trajectory of AM-Bench's own end-effector pose actions. The collect hook (`environment/sim/finalize.sh`) ends the
episode, lets the service exit, writes `final.json` from the live episode's record (no replay) and then films the
trajectory once (the video only); the verifier runs in a separate sandbox (`tests/Dockerfile`) over those artifacts. Compare against the privileged task to measure the cost of losing
resets and privileged state.

## Difficulty
That of `ambench-press-button-fahexa-i00-privileged` (its README), without resets, object poses or the success signal: The button's pose and its pressed state are not given: the red disc on the concrete wall has to be found with the forward-looking onboard cameras and related to the robot through proprioception (base and end-effector poses). A press shows only as a few millimetres of plunger travel, and success is not reported during the episode.

## Reference solution
`solution/oracle_replay.py` streams the privileged oracle's rows (`solution/oracle_qpos.npy`: AM-Bench's scripted expert for
this task, recorded on this instance, then 2 s of holding its last command; one row per control step) as one-second `ee_pose`
chunks (`hold`) through the client; after a service crash it waits for the resumed episode and continues from the control
step the service reports.

## Verification
Graded by the live episode (owner 2026-10-17): `success` = the live episode's success, judged by the sim service on the episode the agent ran, with no grading replay (until 2026-10-17: live success ∧ two fresh-process replays that succeed and agree on the outcome). The live service runs camera-free physics (its cameras are drawn by a separate render process), the same computation as a fresh-process replay of its rows. `tests/grade.py` reads `final.json` with no network and writes the reward fields (`graded_by_live` = 1,
`replays` = 0, `infra_error`, the crash-recovery fields), plus `/logs/verifier/replay.mp4` (the recorded rows drawn by the
render worker in one film pass after the verdict, an illustration: its failure costs the video only). AM-Bench's success
check is evaluated after every control step; the episode ends at the first step where it holds. Before serving, the
service checks the rebuilt scene's state hash against the frozen instance's (`instance_mismatch`: an infrastructure
failure), and GPU faults in the live episode's physics (`live_gpu_faults`) make it no verdict on the agent
(`infra_error`). Oracle: success at control step 1027; the live
episode ended in the privileged oracle's end state (hash `fcd4de9b60a77009`; under the earlier rule both replays did too). `nop`: the episode ends at the
agent's time-out with the check never met, reward 0.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 10 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 16 | 32 GiB | 16384 | 16 GiB | — |
| verifier (as `main`) | 4 | 10 GiB | no cap | no GPU | 20 GiB |

- GPU: an RTX-class NVIDIA GPU (ray-tracing cores), one of the models in `gpu_types`: L40S, L40, L4, RTX 4090, RTX 6000 Ada, RTX PRO 6000 Blackwell, held by `sim`.
- Recommended GPU: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: an RTX 4090 (conformance run and oracle trials, nvidia-smi every 15 s): the sim sidecar runs the camera-free physics process (2.0 GiB of memory, 1.2 GiB of GPU memory with one environment's PhysX buffers; 2.65 GiB with Isaac Lab's) and the render worker (11.9 GiB, 3.4 GiB): 14 GiB and 4.6 GiB; the agent's container holds the client only; the collect hook films through the render worker (4.6 GiB)
- Measured (caps): GPU memory (the sim service): the physics process and the render worker, 4.6 GiB at most -> at least 16 GiB for Isaac Sim: 16 GiB

## Details
| | |
|---|---|
| Episode | one episode, no reset, no step budget: it ends at the first control step where AM-Bench's success check holds, at the task's own time-out (episode length 20 s: control step 2399, reason `episode_timeout`), when the agent's time is up, or on a simulator error; simulated time advances only with the agent's calls |
| Action space | `submit` chunks of `ee_pose` (8: AM-Bench's own action, the tool frame's target pose + the gripper command, streamed at 120 Hz with the agent's `dt`, a multiple of 1/120 s, `hold` or `linear`); `observe` frames or a watch window; `/kinematics` (FK / IK of the arm on its airframe, stateless); binary RCB1 responses |
| Cameras | `ee_camera` (on the end effector), `base_camera` (on the airframe, looking forward), 384×384 RGB, z-depth on request; drawn by a render-only Isaac process from the physics state |
| Images | the `sim` sidecar runs efe-private:ambench (BUILD_ONLY, `images/ambench/BUILD_ONLY`); the agent container and the verifier run the shared client image with its pip pins (`embodied_first_exam/standard/client.env`) |
| Crash recovery | resume class `approx` (GPU PhysX replays the journal to the checkpointed state almost always; a rare race can leave the last bits different, so a resume's deviation is recorded, `resume_state_mismatch`, and the episode continues; the live state after it is graded): every control step journaled, a checkpoint with the state hash after every call; `restart: on-failure:6`, the service resumes the same episode (docs/protocol/standard_mode.md §13) |
| Benchmark notes | tasks/ambench/README.md |

### Change Log
- 2026-10-17 (owner, boards 0150/0151; in place: a new DIGEST, PROTOCOL v2.0 §1 and H.8): standard modes graded by the live
  episode; no grading replay; video by one film pass after the verdict (`replay.mp4`, the render worker drawing the recorded
  rows; it never decides anything and is not run again). `success` is the live episode's success as the service judged it
  (AM-Bench's check at a control step); it was live success ∧ two camera-free fresh-process replays that succeed ∧ agree on
  the outcome (a task graded by verdict: the live success already, its replays recorded). The collect hook's limit 7200 s ->
  4800 s. The start-state check is made by the service before it serves the agent: a rebuilt scene whose state hash is not
  the frozen instance's ends the trial as an infrastructure failure (`instance_mismatch`, a give-up; before, the strict check
  made the service crash and restart until the hook timed out, and the replays checked it again). GPU faults logged by the
  live episode are an infrastructure failure for every task (before: for a task graded by verdict; for the others when the
  live outcome differed from the replays' or the live episode failed). A collect hook on `main` (`kill -9 -1`) now stops
  whatever the agent left running in its container before the service ends the episode (PROTOCOL H.4). reward.json:
  `graded_by_live` = 1, `replays` = 0, no `replay_success` / `deterministic`. The scene, the frozen instance, AM-Bench's
  success check, the agent's interface and the time limit are unchanged; a trial recorded before this date is re-scored by
  its live episode (tasks/ambench/README.md, Grading).
