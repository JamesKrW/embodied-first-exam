# roboarc-wire-splicing-01-i01-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-wire-splicing-01-i01-standard` |
| Simulator / robot | `roboarc` / `aloha-2` |
| Family / instance | `wire_splicing` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0@sha256:96a0a895c0e4934adca39f7a370e67bf82bfb4db7ef049131a3ac0dc9d0f9057` |

## Task
RoboARC wire_splicing (ALOHA 2, MuJoCo): produce one replayable trajectory. Splice the red and the blue wire: bend the red wire's bare end into an eye round the yellow rod of the bending post, thread the blue wire's bare end through the eye, wind the blue wire's bare tail round the red wire's doubled end at least 1.5 turns, at least 1 full turn of them round both strands of the doubled end together, fold the red wire's leftover end back over the coil, then let go. Frozen scene instance (variation wire_splicing_eye_wrap, seed 0).

## Difficulty
Two 6-dof aloha (viperx) arms with parallel-jaw grippers on a fixed base with a 14-D joint-target action space at 40 Hz; success needs the task's predicate (above) held for 0.5 s. Grasping: assisted grasping (contact_weld_v1).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `3a547a8849503c21…`) to `/app/output/`: the MuJoCo reference (this design's oracle, recorded on MuJoCo), which the verifier scored 1 in two fresh-process replays on the project's RoboARC 2.1.0 release image (sha256 `c813b33cff3f`), seed 0 (seeds 1 and 2: RoboARC 1.3.0, before the grader fix); harbor run -a oracle with rendering off (2026-10-26; simulator source cf508cee, tooling 412cbe02).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the frozen instance (`/tests/instance.json`, staged from the image and checked against `tests/runtime_binding.json`) in one fresh process; pass iff the
task's predicate holds after the replay (`tests/grade.py`). `/logs/verifier/replay.mp4` is rendered during that
replay (one pass, no separate video replay). The physics runs on the CPU and is bit-deterministic on one host; with one replay
no final-state hashes are compared (`determinism_checked` 0); the rebuilt frozen scene must match (`instance_ok`)
(Embodied First Exam's GPU-backend rule).
A trajectory may have at most 28000 rows (`RCB_MAX_STEPS`: the replay must fit the verifier's timeout; a longer one is
`bad_trajectory`).

`nop` scored 0 on this base in both modes (the project's RoboARC 2.1.0 release image (sha256 `c813b33cff3f`), seed 0; harbor run -a nop with rendering off; 2026-10-26).

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 7 | 23 GiB | 1024 | 4 GiB | 10 GiB |
| verifier | no cap | no cap | no cap | 4 GiB | no cap |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: L40S 48GB / RTX 4090 24GB / L4 24GB (sm_86 or newer).

## Details
| | |
|---|---|
| Grasping | assisted grasping (contact_weld_v1) |
| Success rule | task-native success (goal_complete True; frozen parameters; terminal revision wire_splice_eye_wrap_v5) |

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
- Forcing is recorded, not judged: an arm drive (not a gripper drive) at 95% of its torque cap or more while that arm's gripper, or a part it holds, presses on anything else with 20 N or more in total is forcing; 0.5 s of it on every physics tick is a forcing event in the evidence (`s.safety()`, `s.status()["forcing"]`), shown to the reviewer when a success is reviewed. Forcing does not end the episode and does not change the grade by itself.
- Simulator breakdown (a non-finite state or a task contact deeper than 10 mm): the episode ends as an infrastructure error, not your failure, unless an arm was forcing (even for one tick) within the 0.25 s before it: then it is your failure (forced_breakdown: what broke and what forced it).
- Grasp regions (assisted grasping, see Grippers above; one or more spheres per graspable part): wire_a: 63 spheres of radii 26.1 mm, 5.1 mm; wire_b: 97 spheres of radii 25.1 mm, 5.5 mm. Both finger contacts must lie inside one of the part's spheres; a part touched elsewhere is grasped physically. Each region's centre (in its link's frame) is in `/app/instance.json` under `simulator.grasp_assist.targets`.
- A failure is reported in `s.status()["episode_end"]` as "A task body fell below the table" or "forced_breakdown: <what>" (<what>: the readable reason); `s.safety()` reports forcing, contact depths, volume ratios and table-guard stops against these limits, and `s.status()["review"]` the deep task contacts that would hold a success for review.

### Provenance
- Frozen instance: a MuJoCo instance (`simulator.physics_engine` "mujoco", its scene frozen under `instance_assets/mujoco/`), built by TopoGym source `4d2cba21` (MuJoCo instance made before this image was built (TopoGym source 4d2cba21; its bytes are bound into the release image efe:roboarc-2.2.0)); bound to this release's runtime (source `1552f29d`, tooling `f2320e4c`).
- Frozen instance files (`instance.json`, `instance_assets/`): not in this directory. The image carries them in `/opt/robotangle/instances/wire-splicing-01-i01/` (`images/roboarc/assets.json` pins each file's sha256), and `/opt/robotangle/stage_instance.py` copies this task's instance next to its `runtime_binding.json`, which names the instance's canonical sha256 and every asset's sha256. The tooling checks them again at every session start.
- Release: RoboARC 2.2.1 (release base `e0a930a38`), image `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0` (images/roboarc/).
- Name: RoboARC is the published name of this benchmark. Code identifiers keep the simulator's name: the simulator package `topogym`, the tooling `rcb_topogym.py`, `TOPOGYM_*` variables.
