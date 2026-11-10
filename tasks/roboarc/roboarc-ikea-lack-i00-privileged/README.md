# roboarc-ikea-lack-i00-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-ikea-lack-i00-standard` |
| Simulator / robot | `mujoco` / `franka-panda-bimanual` |
| Family / instance | `ikea_lack` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b` |

## Task
IKEA LACK side table (AA-207276-4, four legs) in plain MuJoCo 3.14.0 (CPU, float64) with two 7-DoF Franka Panda arms, each with the MuJoCo Menagerie Panda hand (two-finger, tendon-coupled): produce one replayable trajectory. Screw all four legs onto the four threaded studs of the tabletop.

## Difficulty
Bimanual furniture assembly with physical grasps (no assist), 16-D actuator action at 400 Hz
(4 physics steps of 0.625 ms per row); the reference needs 67697 rows
(169.24 s of simulated time).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `21764c5a84d77fba…`) to `/app/output/`: the task's one reference, shared with the standard twin (its `solution/oracle_actions.npz`, the same rows): the robot demo of the IKEA MuJoCo tree (`demos/lack/actions.npz`), time-dilated to the robots' real joint-speed limits (each row whose gripper command steps faster than the Franka Hand's speed held until the hand reaches it, `--grip wait`), so the speed clamp never changes a row. The demo itself commands faster than the limits and fails under the clamp.

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the scene's initial state in one fresh process
(`tests/grade.py`, `tests/rcb_ikea.py replay`: per row `data.ctrl[:] = row; S.step(4)` with the scene's mechanism, then
`final_check()`: `success.py`, released, at rest and no breakdown at the end of the rows and again after a 1.0 s hold of
the last row). Pass iff the replay passes. The
scene in the verifier image must match `tests/runtime_binding.json` file by file (`instance_exact`) and build to its
initial-state hash (`instance_ok`). Replay video (evidence only, never the verdict): another process replays the same rows and renders `/logs/verifier/replay.mp4` (CPU OSMesa, 512 x 288, at most 750 frames over the episode and the 1.0 s hold; `video_rendered`).

Harbor checks (2026-10-18, a Ryzen 9 7950X workstation (shared, heavily loaded: load 40-90 on 32 threads), Docker, Harbor 0.23.0, image 0.3.1 sha256:b9552184c2de, the replay-video verifier, the Franka Hand slew, image `ghcr.io/mll-lab-nu/eai-ikea-mujoco:0.3.1` id `b9552184c2de`): `oracle` scored 1 (two replays, 772.1 s wall); `oracle` scored 1 (two replays, 626.2 s wall); `nop` scored 0. Image 0.3.2 (kyrre's native-cylinder scene; this scene byte-identical to 0.3.1), 2026-10-19: a fresh replay of the reference ends and settles on the same hashes (`684fe3c488e0` / `a4bc18f244c3`, on a Ryzen 9 7950X workstation). Reference timing: one unobserved replay of the hand-slew reference 372 s (3.4 %; 8.6 % of the 4320 s bar) on a quiet i9-14900HX host (one process at a time, image 0.3.1's scene); 1342 s on a loaded Ryzen 9 7950X workstation (load 40-90); 1127 s on a Xeon Gold 6230R core including the re-time step (reported only), end and settled hashes equal on every host and in the Harbor replays.

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 4 GiB | 1024 | no GPU | 10 GiB |
| verifier | 4 | 8 GiB | no cap | no GPU | no cap |

- GPU: not needed.

## Details
| | |
|---|---|
| Grasping | physical (finger friction), no assist |
| Success rule | the scene's `success.py` + the terminal rules (released, at rest, no breakdown), at the end of the rows and after a 1.0 s hold |
| Scene | `/opt/ikea/scenes/lack/` in the image; digest `e669195502062a45…` (images/ikea-mujoco/assets.json) |
| Initial state | sha256 `0bddce57c9bcaf34…` of the mjSTATE_INTEGRATION vector |

### Goal
Screw all four legs onto the four threaded studs of the tabletop.

### Success requirements
- The tabletop lies studs up. Each leg (nut end down) is screwed onto a stud; any leg may go on any stud, one leg per stud.
- Every leg is seated and latched on a distinct stud, and at the end, measured in the tabletop frame: the leg origin is 0 to 12 mm above the slab top, within 20 mm laterally of its stud, and the leg axis is within 10 degrees of the tabletop axis.

### Mechanism
The M16 x 2 right-hand threads are exact convex pieces: a leg advances only by being turned (about six turns from first thread contact to the seat). A leg that is seated on a vacant stud and descends slower than 0.05 m/s is latched to the tabletop at its current pose (a weld, the source's fixed joint).

### Provenance
- Scene and demo: the IKEA MuJoCo tree (`scenes/lack/`, `demos/lack/actions.npz`), MuJoCo 3.14.0, RoboARC rigid settings
  (timestep 0.625 ms, Newton, 100 iterations, elliptic cones).
- The scene files are not in this repository: the image carries them (`images/ikea-mujoco/README.md`, build
  context from `images/ikea-mujoco/stage_context.py`, pinned by `images/ikea-mujoco/assets.json`).
