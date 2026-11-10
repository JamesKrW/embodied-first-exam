# roboarc-battery-replacement-01-i01-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-battery-replacement-01-i01-standard` |
| Simulator / robot | `roboarc` / `aloha-2` |
| Family / instance | `battery_replacement` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0@sha256:96a0a895c0e4934adca39f7a370e67bf82bfb4db7ef049131a3ac0dc9d0f9057` |

## Task
RoboARC battery_replacement (ALOHA 2, MuJoCo): produce one replayable trajectory. Replace the two old AA cells in the game controller's battery compartment with the two new ones, with the right polarity, put the old cells in the recycling box and close the battery cover. Frozen scene instance (variation battery_replacement_xbox_aa, seed 0).

## Difficulty
Two 6-dof aloha (viperx) arms with parallel-jaw grippers on a fixed base with a 14-D joint-target action space at 40 Hz; success needs the task's predicate (above) held for 0.5 s. Grasping: assisted grasping (contact_weld_v1).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `472848140a225398…`) to `/app/output/`: the MuJoCo reference (this design's oracle, recorded on MuJoCo), which the verifier scored 1 in two fresh-process replays on the project's RoboARC 1.3.0 release image (sha256 `499d6f69bd81`), seeds 0, 1 and 2; harbor run -a oracle with rendering off (2026-10-24; simulator source 4d2cba21, tooling 412cbe02).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the frozen instance (`/tests/instance.json`, staged from the image and checked against `tests/runtime_binding.json`) in one fresh process; pass iff the
task's predicate holds after the replay (`tests/grade.py`). `/logs/verifier/replay.mp4` is rendered during that
replay (one pass, no separate video replay). The physics runs on the CPU and is bit-deterministic on one host; with one replay
no final-state hashes are compared (`determinism_checked` 0); the rebuilt frozen scene must match (`instance_ok`)
(Embodied First Exam's GPU-backend rule).
A trajectory may have at most 200000 rows (`RCB_MAX_STEPS`: the replay must fit the verifier's timeout; a longer one is
`bad_trajectory`).

`nop` scored 0 on this base in both modes (the project's RoboARC 1.3.0 release image (sha256 `499d6f69bd81`), seeds 0, 1 and 2; harbor run -a nop with rendering off; 2026-10-24).

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 16 | 30 GiB | 8192 | 12 GiB | 10 GiB |
| verifier | no cap | no cap | no cap | 12 GiB | no cap |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: L40S 48GB / RTX 4090 24GB / L4 24GB (sm_86 or newer).

## Details
| | |
|---|---|
| Grasping | assisted grasping (contact_weld_v1) |
| Success rule | task-native success (goal_complete True; frozen parameters; terminal revision battery_replacement_swap_v1) |

### Goal
Replace the two old AA cells in the game controller's battery compartment with the two new ones, with the right polarity, put the old cells in the recycling box and close the battery cover.

### Success requirements
- Each of the two slots holds a new cell (black body, copper band), fully seated: its axis within 5 degrees of the slot's, its centre within 1.2 mm sideways, 1 mm vertically and 1.5 mm along the axis of the seated position, and the slot's spring contact compressed at least 1 mm.
- Polarity: a cell's + end (the small nub, and the '+' printed on its coloured band) faces that slot's + contact plate, marked with a red '+' on the compartment floor; its - end presses the spring contact.
- The cover is closed and latched: within 0.5 mm of its closed position along and across the controller (so pushed all the way back along its slide, where the detent plunger sits in the cover's groove), between 0.2 mm below and 0.6 mm above it, tilted at most 1 degree, and the detent plunger not held down more than 0.25 mm below its upper stop.
- Both old cells (grey body, tan band) lie inside the recycling box (their centres within its inner walls, below its rim).
- The four cells and the cover move at most 5 mm/s and 0.1 rad/s. Holding parts is allowed; the pry stick may end anywhere (on the table or in a hand).
- Keep all final conditions satisfied together for at least 0.5 s of simulation time. If any final condition stops holding, the confirmation interval starts again. A brief qualifying pose is not enough; let the simulation advance through the full interval.

### Task failure conditions
- A task piece falling more than 100 mm below the tabletop ends the episode in failure. Contact penetration never ends the episode by itself: a success with a task contact deeper than 3 mm, or deeper than 2 mm for at least 0.25 s (100 consecutive physics ticks), is not credited automatically but reviewed, with the recorded evidence and frames (for a finger contact with a held movable piece: 4 mm, or 2 mm for 0.25 s; holding requires both fingers of the same hand to contact the same piece, or an active assisted grasp by that hand); a clean success is credited automatically. A task contact deeper than 10 mm is a simulator breakdown (Grader details).

### Grader details (in addition to the requirements above)
- Contact penetration is read from the contacts at the end of every physics tick (2.5 ms; 10 ticks per control step). "Held" is decided anew at every tick: a zero-force touch counts, and if one finger loses contact (with no active assisted grasp), the other finger is back at the non-held limits in the same tick.
- Robot safety (this benchmark's tooling): no gripper target goes lower than 15 mm below the tabletop; a commanded target that would bring any part of a gripper lower is stopped there, on the commanded path. The fingertips reach about 16 mm beyond the tool frame (TCP) along its approach axis, open or closed. Arm drive torques are capped (the effort limits in robot_spec), so an arm pushing against an obstacle stalls at that torque. An arm with any part more than 5 mm deep in a fixed object (a fixture, the table) is stopped where it is: its joint targets become its measured joints for that control step, so it stops pushing (the safety readings report these stops as static_guard).
- Forcing is recorded, not judged: an arm drive (not a gripper drive) at 95% of its torque cap or more while that arm's gripper, or a part it holds, presses on anything else with 20 N or more in total is forcing; 0.5 s of it on every physics tick is a forcing event in the evidence (`s.safety()`, `s.status()["forcing"]`), shown to the reviewer when a success is reviewed. Forcing does not end the episode and does not change the grade by itself.
- Simulator breakdown (a non-finite state or a task contact deeper than 10 mm): the episode ends as an infrastructure error, not your failure, unless an arm was forcing (even for one tick) within the 0.25 s before it: then it is your failure (forced_breakdown: what broke and what forced it).
- Grasp regions (assisted grasping, see Grippers above; one sphere per graspable part): cover: radius 45.1 mm; stick: radius 32.4 mm, around the stick's handle; old_A: radius 31.3 mm; old_B: radius 31.3 mm; new_1: radius 31.3 mm; new_2: radius 31.3 mm. Both finger contacts must lie inside the part's sphere; a part touched elsewhere is grasped physically. Each region's centre (in its link's frame) is in `/app/instance.json` under `simulator.grasp_assist.targets`.
- A failure is reported in `s.status()["episode_end"]` as "A task body fell below the table" or "forced_breakdown: <what>" (<what>: the readable reason); `s.safety()` reports forcing, contact depths, volume ratios and table-guard stops against these limits, and `s.status()["review"]` the deep task contacts that would hold a success for review.

### Provenance
- Frozen instance: a MuJoCo instance (`simulator.physics_engine` "mujoco", its scene frozen under `instance_assets/mujoco/`), built by TopoGym source `cacc831d` (MuJoCo instance made before this image was built (TopoGym source cacc831d; its bytes are bound into the release image efe:roboarc-2.2.0)); bound to this release's runtime (source `1552f29d`, tooling `f2320e4c`).
- Frozen instance files (`instance.json`, `instance_assets/`): not in this directory. The image carries them in `/opt/robotangle/instances/battery-replacement-01-i01/` (`images/roboarc/assets.json` pins each file's sha256), and `/opt/robotangle/stage_instance.py` copies this task's instance next to its `runtime_binding.json`, which names the instance's canonical sha256 and every asset's sha256. The tooling checks them again at every session start.
- Release: RoboARC 2.2.1 (release base `e0a930a38`), image `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0` (images/roboarc/).
- Name: RoboARC is the published name of this benchmark. Code identifiers keep the simulator's name: the simulator package `topogym`, the tooling `rcb_topogym.py`, `TOPOGYM_*` variables.
