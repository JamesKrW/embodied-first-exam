# roboarc-ikea-svallet-i00-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-ikea-svallet-i00-standard` |
| Simulator / robot | `mujoco` / `aloha-2` |
| Family / instance | `ikea_svallet` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b` |

## Task
IKEA SVALLET work lamp (AA-2140039-1), unpowered steps 1-2 in plain MuJoCo 3.14.0 (CPU, float64) with two 6-DoF ViperX 300 s arms (MuJoCo Menagerie ALOHA), each with its own parallel gripper: produce one replayable trajectory. Press the lamp shade's key down into the socket on top of the stand column until the spring jaw snaps over it, then screw the E12 bulb up into the shade's socket until it seats.

## Difficulty
Bimanual furniture assembly with physical grasps (no assist), 14-D actuator action at 400 Hz
(4 physics steps of 0.625 ms per row); the reference needs 69414 rows
(173.5 s of simulated time).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `699f86e5075b74eb…`) to `/app/output/`: the task's one reference, shared with the standard twin (its `solution/oracle_actions.npz`, the same rows): the robot demo of the IKEA MuJoCo tree (`demos/svallet/actions.npz`), time-dilated to the robots' real joint-speed limits, so the speed clamp never changes a row. The demo itself commands faster than the limits and fails under the clamp.

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the scene's initial state in one fresh process
(`tests/grade.py`, `tests/rcb_ikea.py replay`: per row `data.ctrl[:] = row; S.step(4)` with the scene's mechanism, then
`final_check()`: `success.py`, released, at rest and no breakdown at the end of the rows and again after a 1.0 s hold of
the last row). Pass iff the replay passes. The
scene in the verifier image must match `tests/runtime_binding.json` file by file (`instance_exact`) and build to its
initial-state hash (`instance_ok`). Replay video (evidence only, never the verdict): another process replays the same rows and renders `/logs/verifier/replay.mp4` (CPU OSMesa, 512 x 288, at most 750 frames over the episode and the 1.0 s hold; `video_rendered`).

Harbor checks (2026-10-16, a Ryzen 9 7950X workstation (shared, loaded), Docker, Harbor 0.23.0, image 0.3.0 sha256:ed950d079b63, the replay-video verifier, image `ghcr.io/mll-lab-nu/eai-ikea-mujoco:0.3.0` id `ed950d079b63`): `oracle` scored 1 (two replays, 495.2 s wall); `nop` scored 0. Image 0.3.1 (the shared welded-part contact switch-off, off in this scene): a fresh replay of the reference in it ends and settles on the same hashes as in 0.3.0 (`19d049469045` / `6413de2d8af8`, on a Ryzen 9 7950X workstation, 2026-10-17). Merged head with the Franka Hand slew (not applied to the ViperX): a fresh replay in 0.3.1 ends and settles on `19d049469045` / `6413de2d8af8` (on the same workstation, 2026-10-18). Image 0.3.2 (kyrre's native-cylinder scene; this scene byte-identical to 0.3.1), 2026-10-19: a fresh replay of the reference ends and settles on the same hashes (`19d049469045` / `6413de2d8af8`, on the same workstation). Reference timing: one unobserved replay 478 s (4.4 %, inside the 40 % bar) on a dedicated Xeon Gold 6230R core (2.1 GHz, a slow trial-class CPU), end and settled hashes equal to the Harbor runs'.

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
| Scene | `/opt/ikea/scenes/svallet/` in the image; digest `3c8ceadcfe226264…` (images/ikea-mujoco/assets.json) |
| Initial state | sha256 `22360c69fb8a50ef…` of the mjSTATE_INTEGRATION vector |

### Goal
Press the lamp shade's key down into the socket on top of the stand column until the spring jaw snaps over it, then screw the E12 bulb up into the shade's socket until it seats.

### Success requirements
- Both retention joints are connected (shade on the stand, bulb in the shade) and, at the end, each connector is within its gates (axial gap, lateral offset and axis tilt).
- The stand rests on the bench (vertical contact force above 0.1 N) and is within 3 degrees of its upright start orientation.
- No robot contact with any lamp part at the end.

### Mechanism
The shade's barb cams a spring jaw (a slide joint with a spring, 1200 N/m) open and it snaps back over the barb. The bulb thread (11.9 mm x 1.5 mm right-hand, convex thread pieces) advances only by turning and seats on the socket stop. Retention welds switch on with the current relative pose when the seating gates hold. The prewired cable is not built.

### Provenance
- Scene and demo: the IKEA MuJoCo tree (`scenes/svallet/`, `demos/svallet/actions.npz`), MuJoCo 3.14.0, RoboARC rigid settings
  (timestep 0.625 ms, Newton, 100 iterations, elliptic cones).
- The scene files are not in this repository: the image carries them (`images/ikea-mujoco/README.md`, build
  context from `images/ikea-mujoco/stage_context.py`, pinned by `images/ikea-mujoco/assets.json`).
