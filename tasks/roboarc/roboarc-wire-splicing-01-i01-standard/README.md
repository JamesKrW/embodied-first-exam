# roboarc-wire-splicing-01-i01-standard

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | standard (`eai-standard/2.4`) |
| Twin | `roboarc-wire-splicing-01-i01-privileged` |
| Simulator / robot | `roboarc` / `aloha-2` |
| Family / instance | `wire_splicing` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0@sha256:96a0a895c0e4934adca39f7a370e67bf82bfb4db7ef049131a3ac0dc9d0f9057` |

## Task
Standard-mode twin of `roboarc-wire-splicing-01-i01-privileged` (same frozen instance): the agent gets only the `eai-standard/2.4` client
(docs/protocol/standard_mode.md); the simulator runs in the `sim` sidecar (`environment/docker-compose.yaml`, GPU on the
sidecar), which owns the scene and records the episode as a (T, 14) trajectory; its collect hook
(`environment/sim/finalize.sh`) writes the live episode's verdict, then renders the video in one video-only
replay of the recorded rows (no grading replay). The verifier runs in a separate sandbox (`tests/Dockerfile`) over the sidecar's artifacts. Compare against
the privileged task to measure the cost of losing resets, privileged state and the tooling's primitives.

## Difficulty
Same scene, instance and success rule as `roboarc-wire-splicing-01-i01-privileged` (see its Difficulty). In standard mode the agent loses resets, privileged state and planning primitives: it sees camera images and proprioception only, acts through the client, and gets one episode, so it has to estimate the scene from images and recover from its own mistakes without starting over.

## Reference solution
`solution/solve.sh` streams the rows of the standard reference (`solution/oracle_qpos.npy`, one `joint_position` row per control step, `hold`) through the client (`solution/oracle_replay.py`): the privileged reference moved an arm joint target more than the standard service's joint-speed rule (0.04 rad per control step) in 785 rows (190 runs); each run is re-timed inside a window of nearby rows of that arm: the arm follows the same joint path at an even speed within the rule and arrives at the window's last row on time; no row is added, every other row and every gripper entry is unchanged (meta speed_rule; the privileged reference is unchanged). That episode scored 1 (live and fresh replay, end state `164567f6effde5b0`) on the project's RoboARC 2.1.0 release image (sha256 `c813b33cff3f`), seed 0 (seeds 1 and 2: RoboARC 1.3.0, before the grader fix); harbor run -a oracle with rendering off (2026-10-26; simulator source cf508cee, tooling 412cbe02).

## Verification
See `tests/grade.py`.

`nop` scored 0 on this base in both modes (the project's RoboARC 2.1.0 release image (sha256 `c813b33cff3f`), seed 0; harbor run -a nop with rendering off; 2026-10-26).

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 16 GiB | 1024 | no GPU | 10 GiB |
| `sim` | 2 | 9 GiB | 1024 | 4 GiB | — |
| verifier (as `main`) | 4 | 16 GiB | no cap | no GPU | 10 GiB |

- GPU: any NVIDIA GPU, held by `sim`.
- Recommended GPU: L40S 48GB / RTX 4090 24GB / L4 24GB (sm_86 or newer).

## Details

### Goal
Splice the red and the blue wire: bend the red wire's bare end into an eye round the yellow rod of the bending post, thread the blue wire's bare end through the eye, wind the blue wire's bare tail round the red wire's doubled end at least 1.5 turns, at least 1 full turn of them round both strands of the doubled end together, fold the red wire's leftover end back over the coil, then let go.

### Success requirements
- The red wire's bare end is turned back on itself by more than 150 degrees into an eye that is closed at its neck: the two legs touch where they first meet below the bend.
- The blue wire passes through the closed eye exactly once (its linking number with the eye is +1 or -1).
- The blue wire's bare tail winds at least 1.5 turns round the red wire's doubled section. The turns are counted about the doubled section's axis: the line fitted to the red wire's stem (its bare part from the strip to the neck) and the return-leg points that lie beside it, within 8.9 mm (2.5 core diameters; the core is 3.56 mm thick) of the stem's own line. The counted coil is the longest run of the blue wire's bare points within 8.9 mm of that axis and between the neck and the red wire's strip, each widened by one core diameter.
- At least 1 full turn of the coil goes round both strands of the red wire's doubled end together, not only round the line between them. The strands: the stem side, from the strip to the eye's apex (its point nearest the eye end of the axis), and the return side, from the apex to the return leg's point farthest toward the strip (a leftover end folded back from there is not one of them). For each point of the counted coil, each strand's point at the same station along the axis is taken, where the strand crosses the plane through the coil point perpendicular to the axis, and the coil point's angle about it in that plane; along the coil these angles accumulate into turns about each strand, a step counting only where both strands have their points. Some contiguous part of the coil must wind at least 1 turn about each strand, both the same way as the coil winds about the axis: a loop round one strand only, or one passing between the strands, does not count.
- The coil is tight: its points lie on average at most 7.1 mm (2 core diameters) from the axis.
- The red wire's leftover end is folded back: the direction from its fourth-last to its last point points back toward the eye (its dot product with the axis, which runs from the eye toward the strip, is at most -0.5), and its tip is at least 4.3 mm (1.2 core diameters) from the axis.
- Let go: no hand holds a wire. The blue wire's turns may drop by at most 0.25 and the threading may not change while the conditions are held, and the splice must be still: every bare point's speed relative to the bare points' mean motion at most 5 mm/s.
- Keep all final conditions satisfied together for at least 2 s of simulation time. If any final condition stops holding, the confirmation interval starts again. A brief qualifying pose is not enough; let the simulation advance through the full interval.

### Task failure conditions
- A task piece falling more than 100 mm below the tabletop ends the episode in failure. Contact penetration never ends the episode by itself: a success with a task contact deeper than 3 mm, or deeper than 2 mm for at least 0.25 s (100 consecutive physics ticks), is not credited automatically but reviewed, with the recorded evidence and frames (for a finger contact with a held movable piece: 4 mm, or 2 mm for 0.25 s; holding requires both fingers of the same hand to contact the same piece, or an active assisted grasp by that hand); a clean success is credited automatically. A task contact deeper than 10 mm is a simulator breakdown (Grader details).

### Grader details (in addition to the requirements above)
- Contact penetration as this benchmark grades it: penetration is recorded, it never ends the episode by itself. A success with deep interpenetration (a task contact deeper than 3 mm, or deeper than 2 mm for at least 0.25 s: 100 consecutive physics ticks; for a finger contact with a held piece 4 mm, or 2 mm for 0.25 s) is not credited automatically but reviewed, with the recorded evidence and frames; a clean success is credited automatically. A task contact deeper than 10 mm is a simulator breakdown (below).
- Contact penetration is read from the contacts at the end of every physics tick (2.5 ms; 10 ticks per control step). "Held" is decided anew at every tick: a zero-force touch counts, and if one finger loses contact (with no active assisted grasp), the other finger is back at the non-held limits in the same tick.
- Robot safety (this benchmark's tooling): no gripper target goes lower than 15 mm below the tabletop; a commanded target that would bring any part of a gripper lower is stopped there, on the commanded path. The fingertips reach about 16 mm beyond the tool frame (TCP) along its approach axis, open or closed. Arm drive torques are capped (the effort limits in robot_spec), so an arm pushing against an obstacle stalls at that torque. An arm with any part more than 5 mm deep in a fixed object (a fixture, the table) is stopped where it is: its joint targets become its measured joints for that control step, so it stops pushing (the safety readings report these stops as static_guard).
- Forcing is recorded, not judged: an arm drive (not a gripper drive) at 95% of its torque cap or more while that arm's gripper, or a part it holds, presses on anything else with 20 N or more in total is forcing; 0.5 s of it on every physics tick is a forcing event in the evidence (`meta["safety"]`), shown to the reviewer when a success is reviewed. Forcing does not end the episode and does not change the grade by itself.
- Simulator breakdown (a non-finite state or a task contact deeper than 10 mm): the episode ends as an infrastructure error, not your failure, unless an arm was forcing (even for one tick) within the 0.25 s before it: then it is your failure (forced_breakdown: what broke and what forced it).
- Grasp regions (assisted grasping, see Grippers above; one or more spheres per graspable part): wire_a: 63 spheres of radii 26.1 mm, 5.1 mm; wire_b: 97 spheres of radii 25.1 mm, 5.5 mm. Both finger contacts must lie inside one of the part's spheres; a part touched elsewhere is grasped physically.

| | |
|---|---|
| mode | standard (RGB, optional depth, proprioception, calibrated cameras, no reset) |
| action space | `eai-standard/2.4`: `submit` chunks of `joint_position` (14, streamed at the 40 Hz control rate with the agent's `dt`) or `ee_pose` (16, IK per row); `observe` frames or a watch window; binary RCB1 responses |
| oracle | `full` (`solution/`) |
| verification | `success` = the live success: the task predicate on the live simulator with the instance's own thresholds (its confirmation time included), as the sim service recorded it (`final.json`: `graded_by` live, `replays` 0); no grading replay. The frozen instance is checked when the service builds it (`instance_mismatch`: infrastructure). After the verdict is written, one video-only fresh-process replay in the sidecar's collect hook renders `/logs/verifier/replay.mp4` (and, for a success held for review, the frames around its live review events); a failed video never changes the verdict |
| crash recovery | `eai-standard/2.4` §13: journal + per-call checkpoints, `restart: on-failure:6`, resume class `approx` |

### Provenance
- Frozen instance: a MuJoCo instance (`simulator.physics_engine` "mujoco", its scene frozen under `instance_assets/mujoco/`), built by TopoGym source `4d2cba21` (MuJoCo instance made before this image was built (TopoGym source 4d2cba21; its bytes are bound into the release image efe:roboarc-2.2.0)); bound to this release's runtime (source `1552f29d`, tooling `f2320e4c`).
- Frozen instance files (`instance.json`, `instance_assets/`): not in this directory. The image carries them in `/opt/robotangle/instances/wire-splicing-01-i01/` (`images/roboarc/assets.json` pins each file's sha256), and `/opt/robotangle/stage_instance.py` copies this task's instance next to its `runtime_binding.json`, which names the instance's canonical sha256 and every asset's sha256. The tooling checks them again at every session start.
- Release: RoboARC 2.2.1 (release base `e0a930a38`), image `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0` (images/roboarc/).
- Name: RoboARC is the published name of this benchmark. Code identifiers keep the simulator's name: the simulator package `topogym`, the tooling `rcb_topogym.py`, `TOPOGYM_*` variables.
