# roboarc-clock-assembling-01-i01-standard

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | standard (`eai-standard/2.4`) |
| Twin | `roboarc-clock-assembling-01-i01-privileged` |
| Simulator / robot | `roboarc` / `aloha-2` |
| Family / instance | `clock_assembling` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0@sha256:96a0a895c0e4934adca39f7a370e67bf82bfb4db7ef049131a3ac0dc9d0f9057` |

## Task
Standard-mode twin of `roboarc-clock-assembling-01-i01-privileged` (same frozen instance): the agent gets only the `eai-standard/2.4` client
(docs/protocol/standard_mode.md); the simulator runs in the `sim` sidecar (`environment/docker-compose.yaml`, GPU on the
sidecar), which owns the scene and records the episode as a (T, 14) trajectory; its collect hook
(`environment/sim/finalize.sh`) writes the live episode's verdict, then renders the video in one video-only
replay of the recorded rows (no grading replay). The verifier runs in a separate sandbox (`tests/Dockerfile`) over the sidecar's artifacts. Compare against
the privileged task to measure the cost of losing resets, privileged state and the tooling's primitives.

## Difficulty
Same scene, instance and success rule as `roboarc-clock-assembling-01-i01-privileged` (see its Difficulty). In standard mode the agent loses resets, privileged state and planning primitives: it sees camera images and proprioception only, acts through the client, and gets one episode, so it has to estimate the scene from images and recover from its own mistakes without starting over.

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
Assemble the watch movement's gear train: the four wheels on their pivots in the main plate, the train bridge over them on its pillars with every upper pivot in its hole, the two bridge screws tight; then wind the barrel through its ratchet, let go and watch the fourth wheel run.

### Success requirements
- Seat the barrel, the center wheel, the third wheel and the fourth wheel on their lower pivots in the main plate (each wheel no more than 1 mm above its seat); a wheel seats once its lower pivot is down in its hole: the wheel at most 0.8 mm above its seat, within 0.6 mm of the hole's axis and tilted less than 3 degrees. Each wheel's pinion must mesh with the previous wheel's teeth (barrel, center, third, fourth); a wheel lowered with its pinion's teeth tip to tip on the other wheel's teeth stops until it is turned a little.
- Every upper pivot (the barrel: its arbor) inside its hole in the bridge, at least 1 mm deep and within the hole's radius of its axis.
- The bridge seated on its pillars: within 0.5 mm of its seat height, 0.5 mm of its seat position and tilted less than 0.5 degrees.
- Both screws in the bridge's two screw holes (either screw in either hole, one to a hole) and turned clockwise by 2 full turns there (no thread is simulated: the turns are counted only while the screw's head sits on the bridge over its hole, its underside between 0.3 mm below and 0.8 mm above the bridge's top and its axis within 0.6 mm of the hole's; counterclockwise turns count back; a screw moved to the other hole starts again). Once both are tight, every wheel is seated, every upper pivot is in its bridge hole and the bridge is seated (height and tilt), the bridge is fixed: screw the bridge down after the wheels, not before.
- After the bridge is fixed, wind the barrel: turn its square (the ratchet under it turns with it) counterclockwise seen from above; the click lets it turn that way only and holds it one ratchet tooth (18 degrees) behind the furthest point reached. Wind it at least 120 degrees (as held by the click), then stop winding and let go of the square: the mainspring drives the barrel and the train, and any further advance of the square counts as winding again. Half a second after the last winding the rules time a 5 s run: the fourth wheel must turn at least 3.5 times in it, spend at most 0.2 s slower than a tenth of its free-run speed (the speed the mainspring's present windup gives the train against its own damping and friction), and turn 450 times as much as the barrel in the opposite direction, within 2 %. The run counts only for the mainspring alone: from the last winding to the end of the run neither the hands (any part of the robot) nor the probe may touch the barrel or a wheel, and the barrel must turn the way the mainspring drives it; otherwise the run does not count until the barrel is wound again. Winding again restarts the run.
- Holding parts is allowed, apart from the run above.
- Keep all final conditions satisfied together for at least 0.5 s of simulation time. If any final condition stops holding, the confirmation interval starts again. A brief qualifying pose is not enough; let the simulation advance through the full interval.

### Task failure conditions
- A task piece falling more than 100 mm below the tabletop ends the episode in failure. Contact penetration never ends the episode by itself: a success with a task contact deeper than 3 mm, or deeper than 2 mm for at least 0.25 s (100 consecutive physics ticks), is not credited automatically but reviewed, with the recorded evidence and frames (for a finger contact with a held movable piece: 4 mm, or 2 mm for 0.25 s; holding requires both fingers of the same hand to contact the same piece, or an active assisted grasp by that hand); a clean success is credited automatically. A task contact deeper than 10 mm is a simulator breakdown (Grader details).

### Grader details (in addition to the requirements above)
- Contact penetration is read from the contacts at the end of every physics tick (2.5 ms; 10 ticks per control step). "Held" is decided anew at every tick: a zero-force touch counts, and if one finger loses contact (with no active assisted grasp), the other finger is back at the non-held limits in the same tick.
- Robot safety (this benchmark's tooling): no gripper target goes lower than 15 mm below the tabletop; a commanded target that would bring any part of a gripper lower is stopped there, on the commanded path. The fingertips reach about 16 mm beyond the tool frame (TCP) along its approach axis, open or closed. Arm drive torques are capped (the effort limits in robot_spec), so an arm pushing against an obstacle stalls at that torque. An arm with any part more than 5 mm deep in a fixed object (a fixture, the table) is stopped where it is: its joint targets become its measured joints for that control step, so it stops pushing (the safety readings report these stops as static_guard).
- Forcing is recorded, not judged: an arm drive (not a gripper drive) at 95% of its torque cap or more while that arm's gripper, or a part it holds, presses on anything else with 20 N or more in total is forcing; 0.5 s of it on every physics tick is a forcing event in the evidence (`meta["safety"]`), shown to the reviewer when a success is reviewed. Forcing does not end the episode and does not change the grade by itself.
- Simulator breakdown (a non-finite state or a task contact deeper than 10 mm): the episode ends as an infrastructure error, not your failure, unless an arm was forcing (even for one tick) within the 0.25 s before it: then it is your failure (forced_breakdown: what broke and what forced it).
- The rules (seating, the tooth-on-tooth gates, meshing, the screws' turns, the bridge's fixing, the click and the run test's timing) are applied after every physics tick (2.5 ms); the success conditions are checked at the end of every tick. The mainspring is a spring between the barrel's arbor and drum; the click is a limit on the arbor's turn that the rules move up as it is wound.
- Grasp regions (assisted grasping, see Grippers above; one sphere per graspable part): barrel: radius 65.8 mm; center: radius 42 mm; third: radius 33.8 mm; fourth: radius 33.8 mm; bridge: radius 120.6 mm; screw1: radius 20.9 mm; screw2: radius 20.9 mm; probe: radius 56.1 mm; barrel_arbor: radius 30.6 mm. Both finger contacts must lie inside the part's sphere; a part touched elsewhere is grasped physically.

| | |
|---|---|
| mode | standard (RGB, optional depth, proprioception, calibrated cameras, no reset) |
| action space | `eai-standard/2.4`: `submit` chunks of `joint_position` (14, streamed at the 40 Hz control rate with the agent's `dt`) or `ee_pose` (16, IK per row); `observe` frames or a watch window; binary RCB1 responses |
| oracle | `full` (`solution/`) |
| verification | `success` = the live success: the task predicate on the live simulator with the instance's own thresholds (its confirmation time included), as the sim service recorded it (`final.json`: `graded_by` live, `replays` 0); no grading replay. The frozen instance is checked when the service builds it (`instance_mismatch`: infrastructure). After the verdict is written, one video-only fresh-process replay in the sidecar's collect hook renders `/logs/verifier/replay.mp4` (and, for a success held for review, the frames around its live review events); a failed video never changes the verdict |
| crash recovery | `eai-standard/2.4` §13: journal + per-call checkpoints, `restart: on-failure:6`, resume class `approx` |

### Provenance
- Frozen instance: a MuJoCo instance (`simulator.physics_engine` "mujoco", its scene frozen under `instance_assets/mujoco/`), built by TopoGym source `4163626d` (MuJoCo instance made before this image was built (TopoGym source 4163626d; its bytes are bound into the release image efe:roboarc-2.2.0)); bound to this release's runtime (source `1552f29d`, tooling `f2320e4c`).
- Frozen instance files (`instance.json`, `instance_assets/`): not in this directory. The image carries them in `/opt/robotangle/instances/clock-assembling-01-i01/` (`images/roboarc/assets.json` pins each file's sha256), and `/opt/robotangle/stage_instance.py` copies this task's instance next to its `runtime_binding.json`, which names the instance's canonical sha256 and every asset's sha256. The tooling checks them again at every session start.
- Release: RoboARC 2.2.1 (release base `e0a930a38`), image `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0` (images/roboarc/).
- Name: RoboARC is the published name of this benchmark. Code identifiers keep the simulator's name: the simulator package `topogym`, the tooling `rcb_topogym.py`, `TOPOGYM_*` variables.
