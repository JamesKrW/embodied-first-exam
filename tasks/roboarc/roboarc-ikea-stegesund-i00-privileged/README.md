# roboarc-ikea-stegesund-i00-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-ikea-stegesund-i00-standard` |
| Simulator / robot | `mujoco` / `franka-panda-bimanual` |
| Family / instance | `ikea_stegesund` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b` |

## Task
IKEA STEGESUND toilet roll stand (AA-2652783-1), manual steps 1-3 in plain MuJoCo 3.14.0 (CPU, float64) with two 7-DoF Franka Panda arms, each with the MuJoCo Menagerie Panda hand (two-finger, tendon-coupled): produce one replayable trajectory. Build the STEGESUND roll stand: press half B onto half A at the two leg crossings, push the shelf in from the foot end, drive the four bolts through the legs into the shelf's rail ends with the hex key, stand the frame up and drop the top rod into the two crotches.

## Difficulty
Bimanual furniture assembly with physical grasps (no assist), 16-D actuator action at 400 Hz
(4 physics steps of 0.625 ms per row); the reference needs 379529 rows
(948.82 s of simulated time).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `0bfcaf1ce04871fe…`) to `/app/output/`: the task's one reference, shared with the standard twin (its `solution/oracle_actions.npz`, the same rows): a closed-loop reference built on the final physics: the author's speed-limited demo as the phase guide, each row held until the slewed hands reach it, each grasp approach steered onto the part where it really is, each carried bolt steered onto its hole and kept on its axis while it is screwed in by hand, each hex-key drive moved onto the true bolt axis, the frame turned on the guide's path and the top rod steered into the crotches; tiny target shifts searched where the demo itself is chaotic (the half-lap press); the rows are the targets executed after the joint-speed and hand-speed clamp. It never trips the speed clamp (arm joints and Franka Hand).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the scene's initial state in one fresh process
(`tests/grade.py`, `tests/rcb_ikea.py replay`: per row `data.ctrl[:] = row; S.step(4)` with the scene's mechanism, then
`final_check()`: `success.py`, released, at rest and no breakdown at the end of the rows and again after a 1.0 s hold of
the last row). Pass iff the replay passes. The
scene in the verifier image must match `tests/runtime_binding.json` file by file (`instance_exact`) and build to its
initial-state hash (`instance_ok`). Replay video (evidence only, never the verdict): another process replays the same rows and renders `/logs/verifier/replay.mp4` (CPU OSMesa, 512 x 288, at most 750 frames over the episode and the 1.0 s hold; `video_rendered`).

Harbor checks (2026-10-18, a Ryzen 9 7950X workstation (shared, loaded), Docker, Harbor 0.23.0, image 0.3.0 sha256:ed950d079b63, the replay-video verifier, image `ghcr.io/mll-lab-nu/eai-ikea-mujoco:0.3.0` id `ed950d079b63`): `oracle` scored 1 (two replays, 1552.9 s wall); `nop` scored 0. Image 0.3.1 (the shared welded-part contact switch-off, off in this scene) on the merged head: a fresh replay of the reference ends and settles on the same hashes as in 0.3.0 (`71688676098e` / `a7da23fa3988`, on a Ryzen 9 7950X workstation, 2026-10-18). Image 0.3.2 (kyrre's native-cylinder scene; this scene byte-identical to 0.3.1), 2026-10-19: a fresh replay of the reference ends and settles on the same hashes (`71688676098e` / `a7da23fa3988`, on the same workstation). Reference timing: one unobserved replay of the closed-loop reference 1164 s (10.8 %) on a quiet i9-14900HX host; 1546-1554 s on a Ryzen 9 7950X workstation (shared, load ~22); 2450-2473 s on a dedicated Xeon Gold 6338 core (2.0 GHz, a slow trial-class CPU); end and settled hashes equal on all three CPU classes and in the Harbor replays.

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
| Scene | `/opt/ikea/scenes/stegesund/` in the image; digest `d13192b82ad1037a…` (images/ikea-mujoco/assets.json) |
| Initial state | sha256 `917b2bc63ab1ec62…` of the mjSTATE_INTEGRATION vector |

### Goal
Build the STEGESUND roll stand: press half B onto half A at the two leg crossings, push the shelf in from the foot end, drive the four bolts through the legs into the shelf's rail ends with the hex key, stand the frame up and drop the top rod into the two crotches.

### Success requirements
- Every part except half A and the hex key is at its assembled pose relative to half A (a bolt: relative to the shelf): half B and the shelf within 0.6 mm and 1 degree; the top rod within 3 mm and 1 degree (up to the rod's own symmetry: it may lie end for end, or turned by a facet); each bolt within 0.6 mm along and across its bore axis, its axis within 2 degrees (spin free).
- The stand is upright: half A's z axis within 1 degree of world +z.
- success.py checks only the end state, and the hex key may be anywhere at the end; the terminal rules below add the release, rest and no-breakdown checks.

### Mechanism
Each bolt's coarse right-hand thread (2.5 mm pitch) is a capsule helix against a matching helix in the shelf's rail end: a bolt advances only by being turned, by the hex key in its 4 mm socket. The half-lap crossings are press fits (contact and friction, no weld). A bolt's seat weld is armed at its first thread contact; once its head touches and it has stopped relative to the shelf for 0.05 s, the bolt and the half it clamps are welded to the shelf at their current pose, and the bolt and its rail-end thread stop colliding.

### Provenance
- Scene and demo: the IKEA MuJoCo tree (`scenes/stegesund/`, `demos/stegesund_vlim/actions.npz`), MuJoCo 3.14.0, RoboARC rigid settings
  (timestep 0.625 ms, Newton, 100 iterations, elliptic cones).
- The scene files are not in this repository: the image carries them (`images/ikea-mujoco/README.md`, build
  context from `images/ikea-mujoco/stage_context.py`, pinned by `images/ikea-mujoco/assets.json`).
