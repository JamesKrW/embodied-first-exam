# roboarc-ikea-uppsta-i00-privileged

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | privileged |
| Twin | `roboarc-ikea-uppsta-i00-standard` |
| Simulator / robot | `mujoco` / `franka-panda-bimanual` |
| Family / instance | `ikea_uppsta` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b` |

## Task
IKEA UPPSTÅ toy hammering block (AA-2300462-4), all manual steps and the 8 pegs in plain MuJoCo 3.14.0 (CPU, float64) with two 7-DoF Franka Panda arms, each with the MuJoCo Menagerie Panda hand (two-finger, tendon-coupled): produce one replayable trajectory. Build the UPPSTÅ hammering block: set two dowels into one board end, press end block b onto them and drive screw b with the hex key; turn it over, set the other two dowels, press end block a on and drive screw a; stand the block on its feet and push the eight pegs into their holes.

## Difficulty
Bimanual furniture assembly with physical grasps (no assist), 16-D actuator action at 800 Hz
(4 physics steps of 0.3125 ms per row); the reference needs 430225 rows
(537.78 s of simulated time).

## Reference solution
`solution/solve.sh` copies `solution/trajectory.npz` (sha256 `69cb5194277c444e…`) to `/app/output/`: the task's one reference, shared with the standard twin (its `solution/oracle_actions.npz`, the same rows): a closed-loop reference built on the final physics (`uppsta_oracle.py`): the guide is the author's demo (each 2.5 ms tick reached over its two rows) run in the scene without the prism rule, where it assembles the block; segment by segment (the arms' releases), each grasp approach offset to where the part really is, each carried part held on its guide pose (relative to the board near it) by an integral loop through damped-least-squares IK, the hex key's tip servoed onto the hole's axis, a screw's spin matched modulo 60 degrees, a part pressed a little deeper before its release where it would tip over; away from every grasp and release the demo is followed at every 4th row where the segment's gate allows (`ref/tempo.npz`); the rows are the targets executed within the joint-speed limits and the Franka Hand's speed. It never trips the speed clamp (arm joints and Franka Hand).

## Verification
Open-loop replay of `/app/output/trajectory.npz` from the scene's initial state in one fresh process
(`tests/grade.py`, `tests/rcb_ikea.py replay`: per row `data.ctrl[:] = row; S.step(4)` with the scene's mechanism, then
`final_check()`: `success.py`, released, at rest and no breakdown at the end of the rows and again after a 1.0 s hold of
the last row). Pass iff the replay passes. The
scene in the verifier image must match `tests/runtime_binding.json` file by file (`instance_exact`) and build to its
initial-state hash (`instance_ok`). Replay video (evidence only, never the verdict): another process replays the same rows and renders `/logs/verifier/replay.mp4` (CPU OSMesa, 512 x 288, at most 750 frames over the episode and the 1.0 s hold; `video_rendered`).

Harbor checks (2026-10-18, a Ryzen 9 7950X workstation (shared, load ~15-20), Docker, Harbor 0.23.0, fresh per-run file mtimes, image 0.3.1 sha256:b9552184c2de, the replay-video verifier, image `ghcr.io/mll-lab-nu/roboarc-ikea:1.0.0` id `5d5090ea1910`): `oracle` scored 1 (two replays, 4157.2 s wall); `nop` scored 0. Merged head with the Franka Hand slew in the shared rule (2026-10-18): a fresh replay in image 0.3.1 ends and settles on the same hashes (`a748a42dc36d` / `a2d8a234abc3`, on a Ryzen 9 7950X workstation, 7676 s at load 40-90): the reference was built within the slew. Image 0.3.2 (kyrre's native-cylinder scene; this scene byte-identical to 0.3.1), 2026-10-19: a fresh replay of the reference ends and settles on the same hashes (`a748a42dc36d` / `a2d8a234abc3`, on the same workstation). Reference timing: 3199 s (29.6 % of the 10800 s budget, 74 % of the 4320 s bar) on a quiet i9-14900HX host (one process at a time); 3519 s (32.6 %) on a Ryzen 9 7950X workstation (Docker, image 0.3.1, host load 12-17); one unobserved replay 8414-8425 s (78.0 %, OVER the 40 % bar) on a dedicated Xeon Gold 6230R core (2.1 GHz, a slow trial-class CPU), end and settled hashes equal to the Harbor runs', as gladom / variera / stenerik on that CPU.

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
| Scene | `/opt/ikea/scenes/uppsta/` in the image; digest `8c884957c9be172b…` (images/ikea-mujoco/assets.json) |
| Initial state | sha256 `3b69cfd57f5a5330…` of the mjSTATE_INTEGRATION vector |

### Goal
Build the UPPSTÅ hammering block: set two dowels into one board end, press end block b onto them and drive screw b with the hex key; turn it over, set the other two dowels, press end block a on and drive screw a; stand the block on its feet and push the eight pegs into their holes.

### Success requirements
- Both screws are seated and their seat welds engaged, each in its own hole within 0.6 mm along and across the hole axis, its axis within 2 degrees (spin free).
- Measured in the board frame against the scene's 'assembled' keyframe: both end blocks within 0.6 mm and 1 degree; every dowel position holds a dowel within 0.6 mm, its axis within 2 degrees (the four dowels are interchangeable).
- Every peg hole holds a peg (the eight are interchangeable) within 1 mm of the hole axis, its axis within 2 degrees, pushed down to the up position (at most 0.5 mm above it).
- The block stands on its feet (the board's assembled up direction within 1 degree of world +z) and no robot body touches any part at the end.

### Mechanism
Each screw's coarse right-hand thread (2.5 mm pitch) runs in a helical thread in the board end: a screw advances only by being turned, by the hex key in its socket. Dowels and pegs hold by compliant flutes (spring-loaded slide joints, an interference fit) and friction. A screw's seat weld is armed once it is on its hole's axis within 3 mm of its seat; once its head touches and it has stopped relative to the board for 0.05 s, the screw and the end block it clamps are welded to the board at their current pose, and the screw and its hole's thread stop colliding. The scene's tick is 8 physics steps (2.5 ms); a row here is 4 of them (1.25 ms, 800 Hz).

### Provenance
- Scene and demo: the IKEA MuJoCo tree (`scenes/uppsta/`, `demos/uppsta_vlim/actions.npz`), MuJoCo 3.14.0, RoboARC rigid settings
  (timestep 0.3125 ms, Newton, 100 iterations, elliptic cones).
- The scene files are not in this repository: the image carries them (`images/ikea-mujoco/README.md`, build
  context from `images/ikea-mujoco/stage_context.py`, pinned by `images/ikea-mujoco/assets.json`).
