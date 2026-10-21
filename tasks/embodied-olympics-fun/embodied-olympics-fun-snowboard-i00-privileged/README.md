# embodied-olympics-fun-snowboard-i00-privileged

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | privileged |
| Twin | `embodied-olympics-fun-snowboard-i00-standard` |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `snowboard` / seed 0 |
| Oracle | `none` — no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_snowboard.py: positive by injection (straight runs re-aimed by hand: 6 gates passed, finished in 15.61 s). The ski controller adapted to the board (heel and toe edge by leg IK) passes no gate: an edge command without the matching lean tips the robot over the other edge (static test, flat snow); human review in the pull request that adds the benchmark (PROTOCOL §2.7) |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Ride the parallel slalom on your snowboard: one run down the course, the board and both feet through each of the 6 gates in order, then over the finish line. (Embodied Olympics `snowboard`: one Unitree G1 on the Embodied Olympics simulator, judged by a referee of events, geometry and contact rules.)

## Difficulty
A snowboard turns on its toe edge and its heel edge: the robot must tip the board by leaning forwards or back over it, its feet locked to one plank, with no walking policy; toe-side turns ask it to balance over the narrow board's front edge at speed, heel-side turns to sit back without falling.

## Reference solution
None. No reference controller exists for this family yet: positive by injection (straight runs re-aimed by hand: 6 gates passed, finished in 15.61 s). The ski controller adapted to the board (heel and toe edge by leg IK) passes no gate: an edge command without the matching lean tips the robot over the other edge (static test, flat snow).

## Verification
`/tests/grade.py` replays `/app/output/trajectory.npz` from the frozen start in two fresh processes (`rcb_rfo.py replay`:
one sim row per control step until the referee ends the attempt). Pass iff the referee's verdict is a success in both
and both end in the same state. The rules: FIS snowboard parallel slalom, one run on one course (a shortened course). The course is a groomed slope falling at 8.0 degrees along +x (the fall line), 40.0 m wide, centred on y = 0. Both your feet are bound to one snowboard, the left foot forward (regular), across the board. You stand still at the start at t = 0, sideways to the fall line with the board's nose pointing down it, and are released. There are 6 gates: gate 1's line is 10.0 m down the slope from your start (measured along the slope), each next one 12.0 m further. A gate is two flexible poles across the slope, red and blue in turn: its turning pole stands 1.0 m from the centre line, on the left (y > 0) for gate 1 and then alternately right and left, its outside pole 4.0 m further out on the same side; its gate line is the line on the snow between them. A gate is passed when the board's nose and tail and both feet have crossed its gate line going down the slope; a point that comes back up across a gate line must cross it again. Gates are passed in order: if any of those four points goes down past the next gate's line, or the finish line, while a gate is not passed, you missed it and the run fails. Hitting or folding a pole is allowed. The finish line is 82.0 m down the slope from the start, 12.0 m wide and centred on y = 0: you succeed when, after every gate, both feet cross it between its posts, within 60 s. The run fails if you fall (a part of you other than your feet and your hands touches the snow, or your pelvis is less than 0.35 m above the snow, measured along the slope's normal), if you stop (your pelvis slower than 0.1 m/s for 1.5 s), if a foot crosses the finish line outside its posts, or at 60 s. No oracle; human review in the pull request that adds the benchmark (PROTOCOL §2.7).

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 2 | 2 |
| Memory | 4 GB | 3 GB |
| GPU memory | 0 GB | 0 GB |

- GPU: not needed.
- Measured: author estimate (an RTX 4090 workstation, 2026-10-11): CPU-only MuJoCo with OSMesa rendering; one world steps 5 times faster than real time on one core and holds about 0.4 GB; the verifier runs two replays and a filmed one
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), every container of the trial: no GPU, 13 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 0 GiB GPU memory, 13 GiB memory. Limits measured: author estimate: no trials yet; memory under the 16 GiB cap by the margin the monitor needs (an agent may run several worlds at once or train a controller); disk 20 GiB for downloaded tools and controllers; processes 512: design default.

## Details
| | |
|---|---|
| Category | track-and-jumps |
| Rule book | FIS International Competition Rules (ICR), Snowboard, edition 2024/25, parallel slalom (PSL): Art. 2507.2 (turning poles of consecutive gates 10-14 m apart; vertical drop 80-120 m, at least 18 gates), the gate passed when the board and both feet pass outside the gate's turning pole in the direction of the turn (as alpine ICR 661.4.2 for parallel events), flex turning poles (alpine ICR 680), no continuing after a missed gate, a fall; deviations: Parallel format: one rider on one course against the clock (no opponent, no heats, no blue and red courses side by side). · Vertical drop and gates (80-120 m, at least 18 gates): the course is 82.0 m long on a slope of 8.0 degrees with 6 gates, so that the robot's run fits the episode. · Gates: a turning pole and an outside pole 4.0 m further out (PSL gates carry a panel on their poles; drawn as plain poles here); a flex pole is a 28 mm pole on a sprung two-axis hinge at the snow (stiffness and mass are estimates). The gate is judged like an alpine gate: the board's nose and tail and both feet must cross the gate line from the turning pole to the outside pole. · Snowboard ICR (the start gate: the rider pushes out of it, the gate starts the clock): the robot stands still on the slope at the start and is released at t = 0; the run's time is the episode's. · A rider who falls may get up and go on: a fall ends the run here (the families' fall rule): a part of the robot other than its feet and its hands on the snow, or its pelvis less than 0.35 m above the snow. · Snow and air: the board-snow contact is the freeski ski-snow model of the CHIGUI0/roboolympics project (MIT, ported): kinetic friction along the board after Wolfsperger et al. 2021, the edge's hold across it after Mossner et al. 2013 with race-piste snow, the sidecut's arc after Howe 2001; the snow keeps no ruts. Still air at 1500 m (1.052 kg/m^3), drag on every link; no lift. · Equipment: a children's all-mountain board (not a hard-boot race board); bindings at 0 / 0 degrees with a 0.4 m stance, rigid (no boots: the feet are bound directly; a binding never releases); the board is rigid. |
| Stages | gate 1 → gate 2 → gate 3 → gate 4 → gate 5 → gate 6 → finish |
| Instance | `environment/instance.json` = `tests/instance.json`: the family's parameters, t = 0 state hash `ea22dd10aabce2676960ace1fca6faae47e918f626461ebf23a4065bd32faf5a` |
| Episode | 6000 control steps (60 s at 100 Hz) |
| Action | sim rows of 151 floats per control step (`rfo/world.py`); six action types are views of it |
| Tooling | `/opt/tools/rcb_rfo.py`, `/opt/tools/rfo/` (the world, the action layer, the referee's toolkit; from the files-only image images/embodied-olympics-fun-files/, built from embodied_first_exam/embodied-olympics-fun/) and `environment/rfo_task.py` (this family's rules), the same files in the verifier's `/tests/` |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; tasks/embodied-olympics-fun/README.md) |
