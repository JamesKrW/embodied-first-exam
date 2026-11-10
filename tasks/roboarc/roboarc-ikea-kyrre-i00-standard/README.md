# roboarc-ikea-kyrre-i00-standard

| | |
|---|---|
| Benchmark | RoboARC |
| Mode | standard (`eai-standard/2.4`) |
| Twin | `roboarc-ikea-kyrre-i00-privileged` |
| Simulator / robot | `mujoco` / `franka-panda-bimanual` |
| Family / instance | `ikea_kyrre` / seed 0 (allowed seeds: [0, 1, 2]; PROTOCOL §2) |
| Oracle | `full` |
| Agent time budget | 10800 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe:roboarc-ikea-sim-1.1.0@sha256:f22a1e9042c7877d33703960313cf548f2af7f15d6fcfa61db83315bfdacba1e` |

## Task
IKEA KYRRE stool (AA-2118871-4), manual step 1: three legs, six screws in plain MuJoCo 3.14.0 (CPU, float64) with two 7-DoF Franka Panda arms, each with the MuJoCo Menagerie Panda hand (two-finger, tendon-coupled), through the robot service (eai-standard/2.4; 2.1 / 2.2
requests are served unchanged): RGB from four ALOHA-style cameras (optional depth), proprioception, `joint_position`
chunks, stateless kinematics, every pose in the robot's frame; no reset and no privileged state. Fasten the three legs to the upside-down seat of the KYRRE stool: place each leg and drive its two screws, in the scene's order (leg 1, its two screws, leg 2, its two screws, leg 3, its two screws), and leave the stool upside-down on the table.

## Difficulty
The privileged twin's bimanual assembly with physical grasps, from cameras and proprioception only, in one episode
without reset; 16-D actuator action at 400 Hz, joint targets within the robot's joint
velocity limits (joints 1-4 2.175 rad/s, joints 5-7 2.61 rad/s, the hand command 182.143 units/s). The reference needs 733558 control steps (1833.89 s of simulated
time).

## Reference solution
`solution/solve.sh` runs `solution/oracle_replay.py`: it submits the control steps of `solution/oracle_actions.npz` through
the client as `joint_position` chunks of 10 s, `hold`, one control step per row, as float64 (the client's transport;
`Robot.submit` sends float32), so the service executes exactly those rows. The rows are the task's one reference, the same file as the privileged twin's `solution/trajectory.npz`: a reference built on the final physics, the scene with MuJoCo's native cylinders (kyrre is the one exception to the prism rule, approved on 2026-10-19): the author's demo (every row within the arm limits), each row held until the slewed Franka Hand reaches its command (`standard/std_reference.py --grip wait`); screw 3-1's second grasp re-aimed at where the screw really lies (`patch_grasp.py`: damped-least-squares IK on the TCP, the demo's grasp pose relative to the screw); the hex key set down 12 mm along its rest, off the rest's knife-edge pose where it never comes to rest; the rows are the targets executed within the joint-speed and hand-speed limits,
so the service's speed limiter never has to act (`n_clipped` 0).

## Verification
The sim service (`environment/sim/server.py`) steps the scene's own runtime (`rcb_ikea.Session.step`: `data.ctrl[:] =
row; S.step(4)`), applies the joint-speed rule, evaluates the grader's terminal rules (`rcb_ikea.Session.status()`:
success.py, released, at rest, no breakdown) after every call and, when they hold, `Session.final_check()` (the same after
a 1.0 s hold of the last row), both on a snapshot restored bit-exactly afterwards, ends the episode at the first settled
success or at a breakdown, and journals every executed control step; at the end it writes
`/data/trajectory.npz` (all executed control steps, float64: the replayable deliverable) and the final state hash; an
episode that ends otherwise (timeout) is judged as it stands by the same two checks. Before the episode the service checks
the scene file by file against `runtime_binding.json` and by its initial-state hash (`instance_mismatch`, infrastructure).
The verifier (`tests/grade.py`, separate sandbox, no network, the shared client base) transcribes `final.json`: pass iff
the live episode succeeded (`graded_by_live` 1, `replays` 0). No grading replay runs.

Harbor oracle / nop checks: pending.

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 10 GiB | 1024 | no GPU | 10 GiB |
| `sim` | 2 | 7 GiB | 1024 | 4 GiB | — |
| verifier | 2 | 2 GiB | no cap | no GPU | no cap |

- GPU: any NVIDIA GPU, held by `sim`.
- Recommended GPU: RTX 4090 24GB / L4 24GB (an NVIDIA GPU with Vulkan and EGL).

## Details
| | |
|---|---|
| Grasping | physical (finger friction), no assist |
| Success rule | the scene's `success.py` + the terminal rules (released, at rest, no breakdown), on the live episode: after every call and at its end, at the last row and again after a 1.0 s hold of it (on a copy) |
| Frame | every pose the agent gets or gives is in the robot frame: origin at the left arm's base link, +z up (`spec.world`); the scene's own world frame never leaves the sim service |
| Speed rule | targets within the robot's real limits (joints 1-4 2.175 rad/s, joints 5-7 2.61 rad/s, the hand command 182.143 units/s; the table and its sources: `rcb_ikea.JOINT_SPEED_LIMITS_RAD_S` / `GRIPPER_WIDTH_SPEED_M_S`), clamped per control step, as the privileged twin's Session; faster rows take the control steps they need (`meta.clipped`), as RoboARC's rule |
| Cameras | `cam_high`, `cam_low` (static), `cam_left_wrist`, `cam_right_wrist` (on the hands); 848 x 480, fovy 58 deg; added at service start (`environment/sim/ikea_rig.py`, `rig.json`), the model is unchanged; RGB: Nyx, RoboARC's renderer and look (path tracer, 32 spp, denoised; studio HDRI and point lights; `environment/sim/ikea_nyx.py` with RoboARC's `nyx_direct.py`); depth: MuJoCo's raster; on the sidecar's GPU |
| Video | `/data/live.mp4` (cam_high, 10 frames per simulated second, recorded during the live episode), copied to the verifier logs; never affects the verdict |
| Scene | `/opt/ikea/scenes/kyrre/` in the images; digest `27de0629369b8f2d…` |
| Initial state | sha256 `ecd23f8f42cd96e0…` of the mjSTATE_INTEGRATION vector |

### Budget
The wall clock only, the privileged twin's cutoff (PROTOCOL H.7): `[agent] timeout_sec` = 10800 s. The service
counts the same limit from the agent's first request (`spec.budget.wall_clock_s`, `meta.time_left_s`) and ends the
episode with reason `timeout` when it runs out; that count never runs out before Harbor's.

### Deviations
- `ee_pose` is not offered (only `joint_position`, the scene's actuator layout); `/kinematics` gives FK / IK.
- Graded by the live episode (the settled check on the live simulator); no grading replay, in the sidecar or the
  verifier.

### Goal
Fasten the three legs to the upside-down seat of the KYRRE stool: place each leg and drive its two screws, in the scene's order (leg 1, its two screws, leg 2, its two screws, leg 3, its two screws), and leave the stool upside-down on the table.

### Success requirements
- All nine retention welds (three legs, six screws) are engaged, each by its operation's acceptance gates, strictly in the scene's operation order.
- Every operation is still within its gates: worst connector gap below 0.6 mm (a screw's seat gap 0.7 mm), lateral within 0.8 mm, axis within 2 degrees; every screw head is seated on its leg's bearing pad.
- The assembly stayed rigid after the last weld: every part within 1 mm and 1 degree of its pose relative to the seat at that moment.
- At the end it is released (no robot contact with any part), and all of the above held for 1.5 s with the seat drifting less than 1 cm and tilting less than 3 degrees over the hold.

The benchmark's terminal rules, the same in every IKEA MuJoCo task, come on top of these requirements. When your rows end: (1) the requirements above hold; (2) the parts are released: no robot link touches any part; (3) every part is at rest: at most 0.01 m/s and 0.05 rad/s; (4) the simulation never broke down: no non-finite state, no MuJoCo BADQACC / BADQPOS / BADQVEL warning, no contact of a part deeper than 10 mm, at any time. The grader then holds your last row for 1.0 s more and requires (1)-(4) again at its end.

### Mechanism
One retention weld per operation, switched on with the current relative pose once its gates hold for 0.15 s: the previous operation latched, gap, lateral and tilt near the seat, at least 4 mm of travel, contact with the counterpart, slow, and for a screw more than a quarter turn of tightening. The screw threads (0.5 mm pitch, right-hand) are contact geometry: a screw advances only by being turned. Once a part's weld latches, it stops colliding with the seat and with the other parts welded to it (it still collides with the robot, the table and every unwelded part), and a latched screw and its insert's thread stop colliding altogether (sw_cutover.json: from the first operation on). Each leg still carries the idle spring jaw of its removed press-in pin.
