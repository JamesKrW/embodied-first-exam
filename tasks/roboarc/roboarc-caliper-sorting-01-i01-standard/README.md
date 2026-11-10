# roboarc-caliper-sorting-01-i01-standard

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | standard (`eai-standard/2.4`) |
| Twin | `roboarc-caliper-sorting-01-i01-privileged` |
| Simulator / robot | `roboarc` / `aloha-2` |
| Family / instance | `caliper_sorting` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0@sha256:96a0a895c0e4934adca39f7a370e67bf82bfb4db7ef049131a3ac0dc9d0f9057` |

## Task
Standard-mode twin of `roboarc-caliper-sorting-01-i01-privileged` (same frozen instance): the agent gets only the `eai-standard/2.4` client
(docs/protocol/standard_mode.md); the simulator runs in the `sim` sidecar (`environment/docker-compose.yaml`, GPU on the
sidecar), which owns the scene and records the episode as a (T, 14) trajectory; its collect hook
(`environment/sim/finalize.sh`) writes the live episode's verdict, then renders the video in one video-only
replay of the recorded rows (no grading replay). The verifier runs in a separate sandbox (`tests/Dockerfile`) over the sidecar's artifacts. Compare against
the privileged task to measure the cost of losing resets, privileged state and the tooling's primitives.

## Difficulty
Same scene, instance and success rule as `roboarc-caliper-sorting-01-i01-privileged` (see its Difficulty). In standard mode the agent loses resets, privileged state and planning primitives: it sees camera images and proprioception only, acts through the client, and gets one episode, so it has to estimate the scene from images and recover from its own mistakes without starting over.

## Reference solution
`solution/solve.sh` streams the recorded rows of the standard-mode oracle episode (`solution/oracle_qpos.npy`, one `joint_position` row per 25 ms control step, `hold`; the same rows as the privileged reference) through the client (`solution/oracle_replay.py`). That episode scored 1 (live and fresh replay) on the project's RoboARC 1.3.0 release image (sha256 `499d6f69bd81`), seeds 0, 1 and 2; harbor run -a oracle with rendering off (2026-10-24; simulator source 4d2cba21, tooling 412cbe02).

## Verification
See `tests/grade.py`.

`nop` scored 0 on this base in both modes (the project's RoboARC 1.3.0 release image (sha256 `499d6f69bd81`), seeds 0, 1 and 2; harbor run -a nop with rendering off; 2026-10-24).

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 16 GiB | 4096 | no GPU | 10 GiB |
| `sim` | 4 | 9 GiB | 1024 | 4 GiB | — |
| verifier (as `main`) | 4 | 16 GiB | no cap | no GPU | 10 GiB |

- GPU: any NVIDIA GPU, held by `sim`.
- Recommended GPU: L40S 48GB / RTX 4090 24GB / L4 24GB (sm_86 or newer).

## Details

### Goal
Measure the sleeves with the caliper and put every sleeve into the bin whose label gives its two caliper readings, then briefly hold everything still.

### Success requirements
- Four sleeves stand in the rack. They look alike and differ only in their inside and outside diameters. Each of the four bins carries a label 'ID r1 / OD r2' with two readings of this caliper: r1 with its knife-edge inside jaws spread in a sleeve's bore, r2 with its outside jaws closed on the sleeve's outside. A sleeve belongs in the bin whose two readings both match its own (within 0.1). A bin may stay empty; each bin holds two sleeves standing.
- The caliper is fixed face up on a stand, its jaws out over the table: it cannot be picked up. Bring each sleeve to its jaws, set the jaws with the slider and read the vernier.
- The caliper is not graduated in millimetres. Its scale is in its own unit (one main-scale division; the vernier reads tenths of it); the size of that unit and the reading with the jaws closed are this caliper's own and are not given, so a size measured in millimetres any other way cannot be matched to a label: the readings must come from the scale.
- The slider is locked at the start. Its locking screw is the knurled knob on the slider's front face, left of the vernier on the jaws' side; a white line on its top shows its angle. It turns between two stops 45 degrees apart, and clockwise (seen from the scale side) tightens. Within 10 degrees of the loose stop the slider moves freely (about 1.5 N of friction); from 20 degrees on it is locked where it stands and does not move however hard it is pushed. Turn the knob by pinching it and rotating the gripper about its axis: it turns by friction only and is not a grasp target.
- Every sleeve's centre must end inside the bin of its class: within 40 mm of the bin's centre in both horizontal directions (the bin's 80 mm x 80 mm cavity) and 3 mm to 40 mm above the tabletop.
- Every sleeve must be released: no sleeve may still be held by an assisted grasp, and no part of the robot (arm, gripper or finger) may press on a sleeve with more than 0.05 N. Whether the caliper is locked at the end does not matter.
- Keep every sleeve's linear speed at most 1 mm/s and angular speed at most 0.02 rad/s.
- Keep all final conditions satisfied together for at least 0.5 s of simulation time. If any final condition stops holding, the confirmation interval starts again. A brief qualifying pose is not enough; let the simulation advance through the full interval.

### Task failure conditions
- A task piece falling more than 100 mm below the tabletop ends the episode in failure. Contact penetration never ends the episode by itself: a success with a task contact deeper than 3 mm, or deeper than 2 mm for at least 0.25 s (100 consecutive physics ticks), is not credited automatically but reviewed, with the recorded evidence and frames (for a finger contact with a held movable piece: 4 mm, or 2 mm for 0.25 s; holding requires both fingers of the same hand to contact the same piece, or an active assisted grasp by that hand); a clean success is credited automatically. A task contact deeper than 10 mm is a simulator breakdown (Grader details).
- A sleeve left in a bin that is not its own (its centre inside that bin as above, released as above and still at the speeds above) for 0.5 s ends the episode in failure. When you set a sleeve down to take it again, set it on the table, not in another bin.

### Grader details (in addition to the requirements above)
- Contact penetration is read from the contacts at the end of every physics tick (2.5 ms; 10 ticks per control step). "Held" is decided anew at every tick: a zero-force touch counts, and if one finger loses contact (with no active assisted grasp), the other finger is back at the non-held limits in the same tick.
- Robot safety (this benchmark's tooling): no gripper target goes lower than 15 mm below the tabletop; a commanded target that would bring any part of a gripper lower is stopped there, on the commanded path. The fingertips reach about 16 mm beyond the tool frame (TCP) along its approach axis, open or closed. Arm drive torques are capped (the effort limits in robot_spec), so an arm pushing against an obstacle stalls at that torque. An arm with any part more than 5 mm deep in a fixed object (a fixture, the table) is stopped where it is: its joint targets become its measured joints for that control step, so it stops pushing (the safety readings report these stops as static_guard).
- Forcing is recorded, not judged: an arm drive (not a gripper drive) at 95% of its torque cap or more while that arm's gripper, or a part it holds, presses on anything else with 20 N or more in total is forcing; 0.5 s of it on every physics tick is a forcing event in the evidence (`meta["safety"]`), shown to the reviewer when a success is reviewed. Forcing does not end the episode and does not change the grade by itself.
- Simulator breakdown (a non-finite state or a task contact deeper than 10 mm): the episode ends as an infrastructure error, not your failure, unless an arm was forcing (even for one tick) within the 0.25 s before it: then it is your failure (forced_breakdown: what broke and what forced it).
- Grasp regions (assisted grasping, see Grippers above; one sphere per graspable part): caliper: radius 20 mm; sleeve_0: radius 40 mm; sleeve_1: radius 40 mm; sleeve_2: radius 40 mm; sleeve_3: radius 40 mm. Both finger contacts must lie inside the part's sphere; a part touched elsewhere is grasped physically.

| | |
|---|---|
| mode | standard (RGB, optional depth, proprioception, calibrated cameras, no reset) |
| action space | `eai-standard/2.4`: `submit` chunks of `joint_position` (14, streamed at the 40 Hz control rate with the agent's `dt`) or `ee_pose` (16, IK per row); `observe` frames or a watch window; binary RCB1 responses |
| oracle | `full` (`solution/`) |
| verification | `success` = the live success: the task predicate on the live simulator with the instance's own thresholds (its confirmation time included), as the sim service recorded it (`final.json`: `graded_by` live, `replays` 0); no grading replay. The frozen instance is checked when the service builds it (`instance_mismatch`: infrastructure). After the verdict is written, one video-only fresh-process replay in the sidecar's collect hook renders `/logs/verifier/replay.mp4` (and, for a success held for review, the frames around its live review events); a failed video never changes the verdict |
| crash recovery | `eai-standard/2.4` §13: journal + per-call checkpoints, `restart: on-failure:6`, resume class `approx` |

### Provenance
- Frozen instance: a MuJoCo instance (`simulator.physics_engine` "mujoco", its scene frozen under `instance_assets/mujoco/`), built by TopoGym source `cacc831d` (MuJoCo instance made before this image was built (TopoGym source cacc831d; its bytes are bound into the release image efe:roboarc-2.2.0)); bound to this release's runtime (source `1552f29d`, tooling `f2320e4c`).
- Frozen instance files (`instance.json`, `instance_assets/`): not in this directory. The image carries them in `/opt/robotangle/instances/caliper-sorting-01-i01/` (`images/roboarc/assets.json` pins each file's sha256), and `/opt/robotangle/stage_instance.py` copies this task's instance next to its `runtime_binding.json`, which names the instance's canonical sha256 and every asset's sha256. The tooling checks them again at every session start.
- Release: RoboARC 2.2.1 (release base `e0a930a38`), image `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0` (images/roboarc/).
- Name: RoboARC is the published name of this benchmark. Code identifiers keep the simulator's name: the simulator package `topogym`, the tooling `rcb_topogym.py`, `TOPOGYM_*` variables.
