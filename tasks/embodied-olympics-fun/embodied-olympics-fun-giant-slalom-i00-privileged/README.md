# embodied-olympics-fun-giant-slalom-i00-privileged

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | privileged |
| Twin | `embodied-olympics-fun-giant-slalom-i00-standard` |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `giant_slalom` / seed 0 |
| Oracle | `none` — no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_giant_slalom.py: positive by injection (straight runs re-aimed by hand at the gates: 4 gates passed, finished in 16.55 s). The slalom controller re-searched for this course reaches gate 3 of 4 (it falls at about 7 m/s on the 14.5 m-radius skis); human review in the pull request that adds the benchmark (PROTOCOL §2.7) |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Ski the giant slalom: one run down the course on your skis, both ski tips and both feet through each of the 4 gates in order, then over the finish line. (Embodied Olympics `giant-slalom`: one Unitree G1 on the Embodied Olympics simulator, judged by a referee of events, geometry and contact rules.)

## Difficulty
Giant-slalom skis carve long arcs: to turn, the robot must roll them onto their edges and lean its body into the turn so that the snow's force passes through its centre of mass, with no walking or running policy to help. The gates are far apart, so speed builds between them, and a lean that does not match the turn throws it on the snow.

## Reference solution
None. No reference controller exists for this family yet: positive by injection (straight runs re-aimed by hand at the gates: 4 gates passed, finished in 16.55 s). The slalom controller re-searched for this course reaches gate 3 of 4 (it falls at about 7 m/s on the 14.5 m-radius skis).

## Verification
`/tests/grade.py` replays `/app/output/trajectory.npz` from the frozen start in two fresh processes (`rcb_rfo.py replay`:
one sim row per control step until the referee ends the attempt). Pass iff the referee's verdict is a success in both
and both end in the same state. The rules: FIS alpine giant slalom, one run (a shortened course). The course is a groomed slope falling at 8.0 degrees along +x (the fall line), 40.0 m wide, centred on y = 0. Each of your feet is bound to a giant-slalom ski. You stand still at the start at t = 0 and are released. There are 4 open gates: gate 1's line is 12.0 m down the slope from your start (measured along the slope), each next one 22.0 m further. A gate is two pairs of flexible poles across the slope, each pair joined by a panel, red and blue in turn: the inner pole of its turning pair stands 2.0 m from the centre line, on the left (y > 0) for gate 1 and then alternately right and left; the inner pole of its outside pair stands 5.0 m further out on the same side. Its gate line is the line on the snow between the two inner poles. A gate is passed when both ski tips (the front ends of the skis) and both feet have crossed its gate line going down the slope; a point that comes back up across a gate line must cross it again. Gates are passed in order: if any of those four points goes down past the next gate's line, or the finish line, while a gate is not passed, you missed it and the run fails. Hitting a pole or a panel is allowed. The finish line is 90.0 m down the slope from the start, 14.0 m wide and centred on y = 0: you succeed when, after every gate, both feet cross it between its posts, within 60 s. The run fails if you fall (a part of you other than your feet and your hands touches the snow, or your pelvis is less than 0.35 m above the snow, measured along the slope's normal), if you stop (your pelvis slower than 0.1 m/s for 1.5 s; ICR 614.2.3), if a foot crosses the finish line outside its posts, or at 60 s. No oracle; human review in the pull request that adds the benchmark (PROTOCOL §2.7).

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
| Rule book | FIS International Competition Rules (ICR), Book IV, Alpine, edition 2025: Art. 614.2 (gates, no continuing after a missed gate or a stop), 661.4.1 and 661.4.1.1 (a gate is passed when both ski tips and both feet cross the gate line, the shortest line on the snow between the inner poles of its two pole pairs), 680 (flex turning poles), 690 and 901.2.2 (panels of about 75 x 50 cm, their lower edge at least 1 m above the snow), 901 (giant slalom technical data: a gate of four poles and two panels, alternately red and blue, 4-8 m wide, at least 10 m between the nearest poles of consecutive gates, direction changes 11-15 % of the vertical drop), 902-903 (the course); deviations: ICR 901.1 (vertical drop 250-450 m, U16-U14 200-350 m): the course is 90.0 m long on a slope of 8.0 degrees with 4 gates, a small fraction of the official drop, so that the robot's run fits the episode; 901.2.4's direction changes (11-15 % of the vertical drop) are not kept: they would allow only one change of direction on this drop, so the course keeps the gate spacing and widths instead. · ICR 902 (the terrain, a course about 40 m wide): a uniform groomed slope of 8.0 degrees, 40.0 m wide. · ICR 903 (the course setter's free choice of long, medium and short turns): open gates at a regular 22.0 m, their turning poles alternately 2.0 m left and right of the centre line. · ICR 805 / 905 (the start: the racer pushes out of the start gate, the start wand starts the clock): the robot stands still on the slope at the start and is released at t = 0; the run's time is the episode's. · ICR 614.2.4 and 628 (a racer who falls may get up and go on): a fall ends the run here (the families' fall rule): a part of the robot other than its feet and its hands on the snow, or its pelvis less than 0.35 m above the snow. · Snow and air: the ski-snow contact is the freeski model of the CHIGUI0/roboolympics project (MIT, ported): kinetic friction along the plank after Wolfsperger et al. 2021, the edge's hold across it after Mossner et al. 2013 with race-piste snow, the sidecut's arc after Howe 2001; the snow keeps no ruts. Still air at 1500 m (1.052 kg/m^3), drag on every link; no lift. · Equipment: no ski poles (the robot's hands are bare pads that cannot hold one), no ski boots (the feet are bound directly); the binding is rigid and never releases; the skis are rigid. · Gates: each pole pair and its panel is one body on a sprung two-axis hinge at the snow (a real pair has a flex hinge per pole and a panel that tears off in a collision; ICR 690.1); stiffness and mass are estimates. |
| Stages | gate 1 → gate 2 → gate 3 → gate 4 → finish |
| Instance | `environment/instance.json` = `tests/instance.json`: the family's parameters, t = 0 state hash `dae220d6001870ac83c4b95d1403a5d1fe273052ebe9f08e2cc9722049617bcc` |
| Episode | 6000 control steps (60 s at 100 Hz) |
| Action | sim rows of 151 floats per control step (`rfo/world.py`); six action types are views of it |
| Tooling | `/opt/tools/rcb_rfo.py`, `/opt/tools/rfo/` (the world, the action layer, the referee's toolkit; from the files-only image images/embodied-olympics-fun-files/, built from embodied_first_exam/embodied-olympics-fun/) and `environment/rfo_task.py` (this family's rules), the same files in the verifier's `/tests/` |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; tasks/embodied-olympics-fun/README.md) |
