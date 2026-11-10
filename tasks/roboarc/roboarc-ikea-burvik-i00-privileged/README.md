# roboarc-ikea-burvik-i00-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-ikea-burvik-i00-standard` |
| Simulator / robot | `mujoco` / `franka-panda-bimanual` |
| Family / instance | `ikea_burvik` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b` |

## Task
IKEA BURVIK side table (AA-2445102-1), seven operations in plain MuJoCo 3.14.0 (CPU, float64) with two 7-DoF Franka Panda arms, each with the MuJoCo Menagerie Panda hand (two-finger, tendon-coupled): produce one replayable trajectory. Assemble the BURVIK side table: screw two legs into the upside-down top, turn the top on its side, push the handle through the two remaining top holes, screw the other two legs into the handle's sleeves, and turn the table upright.

## Difficulty
Bimanual furniture assembly with physical grasps (no assist), 16-D actuator action at 400 Hz
(4 physics steps of 0.625 ms per row); the reference needs 162534 rows
(406.34 s of simulated time).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `6445eb0f7dc19ca2…`) to `/app/output/`: the task's one reference, shared with the standard twin (its `solution/oracle_actions.npz`, the same rows): a closed-loop reference built on the final physics and `move_from_take.py`: the demo (time-dilated to the joint-speed limits) as the phase guide, each row held while the slewed Franka Hand is more than two ticks from its command; the top seated in the jig's slot by a two-hand servo (the demo perched it on the guards, leaning 6 degrees, so the hand threading leg_3 met the handle rack); the handle and the two handle legs servoed onto their axes in the top's frame, each leg spun by the wrist with a bounded axial press until its weld engages; for the upright turn the right hand re-takes the handle in the demo's arm configuration and the left hand takes leg_2 on its centre, then the demo's rigid two-hand motion is executed on these grasps through joint-limited IK; the table set down level on its feet, clear of the stand, before the hands open; the rows are the targets executed within the joint-speed and hand-speed limits. It never trips the speed clamp (arm joints and Franka Hand).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the scene's initial state in one fresh process
(`tests/grade.py`, `tests/rcb_ikea.py replay`: per row `data.ctrl[:] = row; S.step(4)` with the scene's mechanism, then
`final_check()`: `success.py`, released, at rest and no breakdown at the end of the rows and again after a 1.0 s hold of
the last row). Pass iff the replay passes. The
scene in the verifier image must match `tests/runtime_binding.json` file by file (`instance_exact`) and build to its
initial-state hash (`instance_ok`). Replay video (evidence only, never the verdict): another process replays the same rows and renders `/logs/verifier/replay.mp4` (CPU OSMesa, 512 x 288, at most 750 frames over the episode and the 1.0 s hold; `video_rendered`).

Harbor checks (2026-10-18, a Ryzen 9 7950X workstation (shared, loaded), Docker, Harbor 0.23.0, fresh per-run file mtimes, image 0.3.0 sha256:ed950d079b63, the replay-video verifier, image `ghcr.io/mll-lab-nu/roboarc-ikea:1.0.0` id `5d5090ea1910`): `oracle` scored 1 (two replays, 1646.1 s wall); `nop` scored 0. Image 0.3.1 (the shared welded-part contact switch-off, off in this scene) on the merged head: a fresh replay of the reference ends and settles on the same hashes as in 0.3.0 (`60878a5edc5d` / `02732bf5d33e`, on a Ryzen 9 7950X workstation, 2026-10-18). Image 0.3.2 (kyrre's native-cylinder scene; this scene byte-identical to 0.3.1), 2026-10-19: a fresh replay of the reference ends and settles on the same hashes (`60878a5edc5d` / `02732bf5d33e`, on the same workstation). Reference timing: one unobserved replay 3780-3783 s (35.0 %, inside the 40 % bar: 87.6 % of its 4320 s) on a dedicated Xeon Gold 6230R core (2.1 GHz, a slow trial-class CPU), end and settled hashes equal to the Harbor runs'; 1694 s (15.7 %) on a quiet i9-14900HX host; 1679-1684 s on a Ryzen 9 7950X workstation (Docker, image 0.3.0, two replays at once).

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
| Scene | `/opt/ikea/scenes/burvik/` in the image; digest `49dbcc4041d4f2be…` (images/ikea-mujoco/assets.json) |
| Initial state | sha256 `d563d833bd219a22…` of the mjSTATE_INTEGRATION vector |

### Goal
Assemble the BURVIK side table: screw two legs into the upside-down top, turn the top on its side, push the handle through the two remaining top holes, screw the other two legs into the handle's sleeves, and turn the table upright.

### Success requirements
- All five retention joints (the four legs and the handle to the top) are latched.
- Every leg and the handle is at its assembled pose in the top's frame: gap within 0.8 mm, lateral within 1 mm, tilt within 2 degrees (legs: thread axis; handle: full rotation, both rod ends within 1 mm of the two top holes), relative speed below 0.025 m/s.
- The top ends upright: its normal within 2 degrees of world +z.

### Mechanism
Each leg end is a right-hand M8 thread (1 mm lead, SDF mesh) in an SDF nut: about 12.5 turns from first engagement to the seat. Retention welds (legs and handle to the top) switch on once the scene's gates hold for 0.15 s.

### Provenance
- Scene and demo: the IKEA MuJoCo tree (`scenes/burvik/`, `demos/burvik/actions.npz`), MuJoCo 3.14.0, RoboARC rigid settings
  (timestep 0.625 ms, Newton, 100 iterations, elliptic cones).
- The scene files are not in this repository: the image carries them (`images/ikea-mujoco/README.md`, build
  context from `images/ikea-mujoco/stage_context.py`, pinned by `images/ikea-mujoco/assets.json`).
