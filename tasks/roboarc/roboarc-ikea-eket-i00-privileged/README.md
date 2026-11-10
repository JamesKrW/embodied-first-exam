# roboarc-ikea-eket-i00-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-ikea-eket-i00-standard` |
| Simulator / robot | `mujoco` / `franka-panda-bimanual` |
| Family / instance | `ikea_eket` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b` |

## Task
IKEA EKET cabinet (AA-1914763-5), manual steps 1-4 in plain MuJoCo 3.14.0 (CPU, float64) with two 7-DoF Franka Panda arms, each with the MuJoCo Menagerie Panda hand (two-finger, tendon-coupled): produce one replayable trajectory. Build the EKET cabinet frame: slide the two side panels into the top panel, then the back panel into its grooves, then the bottom panel onto the sides, and leave the cabinet standing on the table on its own.

## Difficulty
Bimanual furniture assembly with physical grasps (no assist), 16-D actuator action at 400 Hz
(4 physics steps of 0.625 ms per row); the reference needs 83181 rows
(207.95 s of simulated time).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `eb838b01a6df5fcd…`) to `/app/output/`: the task's one reference, shared with the standard twin (its `solution/oracle_actions.npz`, the same rows): a closed-loop reference built on the final physics: the demo as the phase guide, each held panel servoed onto the unclamped demo's recorded relative-pose track (damped-least-squares Jacobian IK on the TCP) and paused at each slide until its gate accepts; each demo row held (the guide tracks still running) until the slewed Franka Hand reaches its command; the rows are the targets executed after the joint-speed and hand-speed clamp. It never trips the speed clamp (arm joints and Franka Hand).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the scene's initial state in one fresh process
(`tests/grade.py`, `tests/rcb_ikea.py replay`: per row `data.ctrl[:] = row; S.step(4)` with the scene's mechanism, then
`final_check()`: `success.py`, released, at rest and no breakdown at the end of the rows and again after a 1.0 s hold of
the last row). Pass iff the replay passes. The
scene in the verifier image must match `tests/runtime_binding.json` file by file (`instance_exact`) and build to its
initial-state hash (`instance_ok`). Replay video (evidence only, never the verdict): another process replays the same rows and renders `/logs/verifier/replay.mp4` (CPU OSMesa, 512 x 288, at most 750 frames over the episode and the 1.0 s hold; `video_rendered`).

Harbor checks (2026-10-18, a Ryzen 9 7950X workstation (shared, heavily loaded: load 40-90 on 32 threads), Docker, Harbor 0.23.0, image 0.3.1 sha256:b9552184c2de, the replay-video verifier, the Franka Hand slew, image `ghcr.io/mll-lab-nu/eai-ikea-mujoco:0.3.1` id `b9552184c2de`): `oracle` scored 1 (two replays, 1454.4 s wall); `nop` scored 0. Image 0.3.2 (kyrre's native-cylinder scene; this scene byte-identical to 0.3.1), 2026-10-19: a fresh replay of the reference ends and settles on the same hashes (`25be8deaa1fe` / `1d2906d52c38`, on a Ryzen 9 7950X workstation). Reference timing: one unobserved replay of the hand-slew reference 220 s (2.0 %; 5.1 % of the 4320 s bar) on a quiet i9-14900HX host (one process at a time, image 0.3.1's scene); 652 s on a loaded Ryzen 9 7950X workstation (load 40-90); 575 s on a dedicated Xeon Gold 6230R core (2.1 GHz, a slow trial-class CPU; reported only), end and settled hashes equal on every host and in the Harbor replays.

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
| Scene | `/opt/ikea/scenes/eket/` in the image; digest `cd515879e4eee6da…` (images/ikea-mujoco/assets.json) |
| Initial state | sha256 `9815e6dc4b4a9856…` of the mjSTATE_INTEGRATION vector |

### Goal
Build the EKET cabinet frame: slide the two side panels into the top panel, then the back panel into its grooves, then the bottom panel onto the sides, and leave the cabinet standing on the table on its own.

### Success requirements
- The seven operations happen in this order, each measured against the top panel: side A entry and slide, side B entry and slide, back slide, bottom entry and slide. Each one must reach its goal pose within the scene's gates (goal error 0.25 to 0.8 mm, angle 0.1 to 1 degree, nearly at rest, with contact with its counterpart panels).
- The four frame joints (side A and side B to the top, the bottom to both sides) are latched, and the five panels are finite.
- At the end the panels are released (no robot contact, no applied force), rest on the table (at least one panel touches the table top, none touches the floor) and move slower than 0.01 m/s and 0.05 rad/s.
- At the end every panel, the back included (it has no frame joint), is still in its assembled place: within 10 mm and 2 degrees of its goal pose relative to the top panel (the benchmark's end-pose rule, rcb_ikea.END_POSE_RULES).

### Mechanism
Mitres, keyholes, dowels and back grooves are contact. The four frame joints (welds) latch with the measured relative pose when the side A slide, the side B slide and the bottom slide pass their gates.

### Provenance
- Scene and demo: the IKEA MuJoCo tree (`scenes/eket/`, `demos/eket_vlim/actions.npz`), MuJoCo 3.14.0, RoboARC rigid settings
  (timestep 0.625 ms, Newton, 100 iterations, elliptic cones).
- The scene files are not in this repository: the image carries them (`images/ikea-mujoco/README.md`, build
  context from `images/ikea-mujoco/stage_context.py`, pinned by `images/ikea-mujoco/assets.json`).
