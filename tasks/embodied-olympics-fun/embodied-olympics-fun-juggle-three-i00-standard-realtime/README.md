# embodied-olympics-fun-juggle-three-i00-standard-realtime

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | standard-realtime (`eai-standard/2.3`) |
| Twin | `embodied-olympics-fun-juggle-three-i00-privileged` |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `juggle_three` / seed 0 |
| Oracle | `none` — no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_juggle_three.py: positive by injection (10 catches: the machine's red lob and every catch are real, the jaws closing on the arriving ball; each throw injected as an arc from inside, and each arriving ball recentred 0.15 m above the jaws); the red lob comes down within 5 mm of the left jaws' centre but at 1.1 m/s horizontally; negatives: nop, the waiting ball pushed in the machine, a low throw, the wrong ball, a fall; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Real time | slowdown S = 1 (the world runs at 1/S of real speed: one control step every S x control_dt s of wall time; `RCB_SLOWDOWN`'s default); reset budget 10 (`RCB_RESET_BUDGET`'s default) (PROTOCOL §4b) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Juggle three balls in a cascade: throw each ball from one gripper in an arc to the other, the hands in turn, catching each between the jaws, until 10 catches have been made in a row. (Embodied Olympics `juggle-three`; the robot as a service on a clock that does not stop, slowdown S = 1: its head cameras, joint sensors and IMUs, and the venue's fixed camera only.)

## Difficulty
Each throw has to come down between jaws that leave 10 mm on each side of the ball, 0.4 m away, about 0.6 s later, the jaws must close on it within a few hundredths of a second before it bounces off, and the catching gripper must have thrown its own ball before: three balls, two hands, never two in one. A throw is the gripper opening while the arm moves, and errors add up from throw to throw, because a ball is thrown from wherever it was caught. The scene is dynamic (`scripts/task_dynamics.toml`: A small ball machine lobs the first ball into the robot's left hand at launch_time_s.), and in standard-realtime mode time does not stop: once an attempt's clock starts the venue's timed events happen whatever the agent does, so it has to plan or stream the robot's rows from a program of its own, with no simulator and no state, and 10 resets (only the last attempt is graded).

## Reference solution
None (as `embodied-olympics-fun-juggle-three-i00-privileged`).

## Verification
The sim sidecar (`rcb_realtime.py` + `rcb_service.py`, the shared eai-standard/2.3 layer, with the standard twin's
`server.py`) runs the attempt on its clock and its referee judges it live; every control step the simulator applied is
recorded, and the live referee's call on the last attempt is the verdict (`final.json`: `graded_by` live, `replays` 0; no
grading replay, owner 2026-10-17); the video is rendered from the recorded attempt afterwards.
`/tests/grade.py` reads `/data/final.json`: pass iff the live attempt succeeded. The clock's record
(`stalls`, `max_lag_s`, ...) is copied into `reward.json` and decides nothing. `nop`: the attempt never starts (its
clock starts at the first submit), reward 0. The
rules: Three balls (0.07 m across, 0.13 kg). Both your hands are two-jaw grippers (90 mm stroke, jaws 80 mm long with rubber pads on their inner faces, at most 120 N of grip) that hold things only by friction. At the start your forearms point forwards and your grippers' jaws point up; the blue ball is held between the jaws of your left gripper, the yellow ball between those of your right gripper, and the red ball rests in a small ball machine on the ground 1.0 m in front of you: it stands for the second ball of the right hand and is thrown for you at about t = 2.0 s (it is thrown when it has left the machine's wheels), on a lob about 0.4 m above your jaws that comes down on where the centre of your left gripper's jaws is at the start. Before that it may not be touched (by you or by a ball you hold). A catch: the pads of both jaws of a gripper press on a ball from its two sides (each with a contact whose direction is within 45.0 degrees of the jaws' closing direction: a ball lying on the jaws' tips or edges is not caught) for 0.05 s without a break, on a ball that has been thrown by the other hand (the red ball's first throw counts as the right hand's), the clamp beginning at the latest 0.15 s after the ball first touched you again. A ball is in a hand from its catch (the blue and the yellow ball from the start) until it has touched nothing of that gripper for 0.02 s: its release, at its last contact with the gripper; a ball in a hand may touch nothing of you but that gripper. A throw: after the release the ball, touching nothing but air, rises at least 0.3 m above the point where it left the gripper; only the rise before the ball first touches you again counts. The cascade's order: the catches alternate between the hands, the left hand first, and take the balls in the order red, blue, yellow, red, ... You succeed at the 10th catch. You fail if a ball touches the ground; if a ball in a hand touches any part of you other than that gripper (the other gripper included); if a ball in flight touches any part of you other than a gripper, or still touches the gripper that released it later than 0.1 s after the release; if a ball is caught by the gripper that released it, or before it has risen 0.3 m; if a ball that has touched a gripper again is not clamped within 0.15 s (it rests on the gripper or bounces off it); if a catch is out of the cascade's order (the wrong hand or the wrong ball); if the waiting ball is touched before its launch; if you fall (your pelvis lower than 0.35 m above the ground, or any part of you other than your feet on the ground); or after 20 s. No oracle; human review in the pull request that adds the benchmark (PROTOCOL §2.7).

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 3 | 2 |
| Memory | 5 GB | 3 GB |
| GPU memory | 1 GB | 1 GB |

- GPU: required, any NVIDIA GPU; recommended: any NVIDIA GPU with EGL (RTX 4090 / L4).
- Measured: as the standard twin; the GPU renders the cameras over EGL (2026-10-14): author estimate (an RTX 4090 workstation, 2026-10-11): CPU-only; the sim sidecar steps and renders on one or two cores (under 1 GB), the agent's container holds the harness and the client
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), the agent's container: no GPU, 8 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 4 GiB GPU memory, 16 GiB memory. Limits measured: author estimate: no trials yet; the agent's container 8 GiB under its 10 GiB cap; whole trial: + the sim service's 8 GiB cap; disk 20 GiB for downloaded tools and controllers; processes 512: design default.

## Details
| | |
|---|---|
| Category | throwing-catching-striking |
| Rule book | No rule book: the three-ball cascade as jugglers define and count it (siteswap 3; the International Jugglers' Association counts a run in catches): each ball is thrown from one hand and caught by the other, the hands throw in turn, the balls follow one another in a fixed order; a run ends with a drop; deviations: The hands are Unitree's Dex1-1 two-jaw grippers (90 mm stroke, jaws 80 mm long and 34 mm wide with rubber pads, at most 120 N of grip, opening and closing at most 375 mm/s); nothing is welded. A gripper holds one ball between its jaws, so "two balls in one hand at the start" is modelled: the third ball (the red one) is thrown into the left gripper by a small two-wheel ball machine on the ground 1.0 m in front of you at about t = 2.0 s, on a lob about 0.4 m above the jaws that comes down on where the left gripper's jaw centre is at the start; its first catch counts as a catch of a ball thrown by the right hand. Until it is thrown it may not be touched. · A catch is the pads of both jaws of the catching gripper pressing on the ball from its two sides (each with a contact within 45.0 degrees of the jaws' closing direction) for 0.05 s without a break, the clamp beginning at the latest 0.15 s after the ball first touched you again; a ball is in a hand from its catch until it has touched nothing of that gripper for 0.02 s (released, at its last contact: a ball that rattles in the jaws is not released), and it may touch nothing of you but that gripper. · A throw is a release after which the ball, touching nothing, rises at least 0.3 m above where it left the gripper; a hand-to-hand pass is not a throw. So that a ball laid on the other hand and lifted is not one, the rise counts only until the ball first touches you again. · The balls are spheres of a juggling ball's size and mass with a soft bounce; they fly in air (MuJoCo's ellipsoid fluid model: drag, a quasi-steady model), still air. |
| Stages | first catch → half way → ten catches |
| Service | `environment/sim/rcb_realtime.py` (the shared realtime layer, eai-standard/2.3) with the standard twin's `server.py` on `rcb_service.py`, `rcb_rfo.py`, `rfo/`, `rfo_task.py`, `instance.json` (= the privileged twin's); EGL rendering on the GPU |
| Slowdown | S = 1 (`RCB_SLOWDOWN`'s default): RTF 2.39 with the clock flat out (RCB_SLOWDOWN = 0.01, physics only, 4 attempts; 4090-ws1, 4 CPUs, 2026-10-20); PROTOCOL §4b: S = 1 when RTF >= 3, else ceil(1.5 / RTF) |
| Observations | `head_left`, `head_right` (RGB 270 × 480, depth on `head_left`), `venue` (fixed); proprioception: joints, torques, targets, two IMUs, grip state |
| Episode | 2000 control steps (20 s at 100 Hz); 10 resets, the last attempt graded |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; tasks/embodied-olympics-fun/README.md) |
