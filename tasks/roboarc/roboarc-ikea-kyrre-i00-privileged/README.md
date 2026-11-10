# roboarc-ikea-kyrre-i00-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-ikea-kyrre-i00-standard` |
| Simulator / robot | `mujoco` / `franka-panda-bimanual` |
| Family / instance | `ikea_kyrre` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b` |

## Task
IKEA KYRRE stool (AA-2118871-4), manual step 1: three legs, six screws in plain MuJoCo 3.14.0 (CPU, float64) with two 7-DoF Franka Panda arms, each with the MuJoCo Menagerie Panda hand (two-finger, tendon-coupled): produce one replayable trajectory. Fasten the three legs to the upside-down seat of the KYRRE stool: place each leg and drive its two screws, in the scene's order (leg 1, its two screws, leg 2, its two screws, leg 3, its two screws), and leave the stool upside-down on the table.

## Difficulty
Bimanual furniture assembly with physical grasps (no assist), 16-D actuator action at 400 Hz
(4 physics steps of 0.625 ms per row); the reference needs 733558 rows
(1833.89 s of simulated time).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `0808ce7829703cda…`) to `/app/output/`: the task's one reference, shared with the standard twin (its `solution/oracle_actions.npz`, the same rows): a reference built on the final physics, the scene with MuJoCo's native cylinders (kyrre is the one exception to the prism rule, approved on 2026-10-19): the author's demo (every row within the arm limits), each row held until the slewed Franka Hand reaches its command (`standard/std_reference.py --grip wait`); screw 3-1's second grasp re-aimed at where the screw really lies (`patch_grasp.py`: damped-least-squares IK on the TCP, the demo's grasp pose relative to the screw); the hex key set down 12 mm along its rest, off the rest's knife-edge pose where it never comes to rest; the rows are the targets executed within the joint-speed and hand-speed limits. It never trips the speed clamp (arm joints and Franka Hand).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the scene's initial state in one fresh process
(`tests/grade.py`, `tests/rcb_ikea.py replay`: per row `data.ctrl[:] = row; S.step(4)` with the scene's mechanism, then
`final_check()`: `success.py`, released, at rest and no breakdown at the end of the rows and again after a 1.0 s hold of
the last row). Pass iff the replay passes. The
scene in the verifier image must match `tests/runtime_binding.json` file by file (`instance_exact`) and build to its
initial-state hash (`instance_ok`). Replay video (evidence only, never the verdict): another process replays the same rows and renders `/logs/verifier/replay.mp4` (CPU OSMesa, 512 x 288, at most 750 frames over the episode and the 1.0 s hold; `video_rendered`).

Harbor checks (2026-10-19, a Ryzen 9 7950X workstation (shared, load ~10 first run, 30-65 final), Docker, Harbor 0.23.0, fresh per-run file mtimes, image 0.3.2 sha256:5d5090ea1910 (final; first run on the candidate's local 0.3.1-kyrre-cyl), the replay-video verifier, image `ghcr.io/mll-lab-nu/roboarc-ikea:1.0.0` id `5d5090ea1910`): `oracle` scored 1 (two replays, 4528.2 s wall); `nop` scored 0; `oracle` scored 1 (two replays, 2745.6 s wall); `nop` scored 0. kyrre is the one scene with MuJoCo's native cylinders (scene.py, the prism rule off; decided on 2026-10-19): end 0a9f1a9e6d77 / settled e242979c6712 on AMD Zen 4 (7950X), Intel AVX2 (i9-14900HX) and Intel AVX-512 (Xeon Gold 6338, Xeon Gold 6230R). Reference timing: 2740 s (25.4 % of the 10800 s budget, 63 % of the 4320 s bar) on a quiet i9-14900HX host (one process at a time); 2694 s (24.9 %) on a Ryzen 9 7950X workstation (Docker, host load 5-10); one unobserved replay 8283 s (76.7 %, OVER the 40 % bar) on a dedicated Xeon Gold 6230R core (2.1 GHz, a slow trial-class CPU) and 7953 s (73.6 %) on a Xeon Gold 6338 core, end and settled hashes equal to the Harbor runs', as gladom / variera / stenerik / uppsta on the slow class.

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
| Scene | `/opt/ikea/scenes/kyrre/` in the image; digest `27de0629369b8f2d…` (images/ikea-mujoco/assets.json) |
| Initial state | sha256 `ecd23f8f42cd96e0…` of the mjSTATE_INTEGRATION vector |

### Goal
Fasten the three legs to the upside-down seat of the KYRRE stool: place each leg and drive its two screws, in the scene's order (leg 1, its two screws, leg 2, its two screws, leg 3, its two screws), and leave the stool upside-down on the table.

### Success requirements
- All nine retention welds (three legs, six screws) are engaged, each by its operation's acceptance gates, strictly in the scene's operation order.
- Every operation is still within its gates: worst connector gap below 0.6 mm (a screw's seat gap 0.7 mm), lateral within 0.8 mm, axis within 2 degrees; every screw head is seated on its leg's bearing pad.
- The assembly stayed rigid after the last weld: every part within 1 mm and 1 degree of its pose relative to the seat at that moment.
- At the end it is released (no robot contact with any part), and all of the above held for 1.5 s with the seat drifting less than 1 cm and tilting less than 3 degrees over the hold.

### Mechanism
One retention weld per operation, switched on with the current relative pose once its gates hold for 0.15 s: the previous operation latched, gap, lateral and tilt near the seat, at least 4 mm of travel, contact with the counterpart, slow, and for a screw more than a quarter turn of tightening. The screw threads (0.5 mm pitch, right-hand) are contact geometry: a screw advances only by being turned. Once a part's weld latches, it stops colliding with the seat and with the other parts welded to it (it still collides with the robot, the table and every unwelded part), and a latched screw and its insert's thread stop colliding altogether (sw_cutover.json: from the first operation on). Each leg still carries the idle spring jaw of its removed press-in pin.

### Provenance
- Scene and demo: the IKEA MuJoCo tree (`scenes/kyrre/`, `demos/kyrre_vlim/actions.npz`), MuJoCo 3.14.0, RoboARC rigid settings
  (timestep 0.625 ms, Newton, 100 iterations, elliptic cones).
- The scene files are not in this repository: the image carries them (`images/ikea-mujoco/README.md`, build
  context from `images/ikea-mujoco/stage_context.py`, pinned by `images/ikea-mujoco/assets.json`).
