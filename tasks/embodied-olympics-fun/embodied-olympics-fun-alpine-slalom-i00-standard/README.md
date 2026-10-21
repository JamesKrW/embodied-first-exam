# embodied-olympics-fun-alpine-slalom-i00-standard

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `embodied-olympics-fun-alpine-slalom-i00-privileged` |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `alpine_slalom` / seed 0 |
| Oracle | `none` — no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_alpine_slalom.py: positive by a real controller (leg IK for lean and edge on the snow, carved arcs, slip feedback; gains from a search) that passes all 6 gates and finishes in 20.84 s at its exact gains; an injection run (the heading re-aimed by hand) also finishes. It is fragile: 0 of 10 succeed with its gains scaled by 0.98 to 1.02 (typically 4-5 of 6 gates, falls at 5-6 m/s), so it is not shipped as a reference solution; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Ski the slalom: one run down the course on your skis, both ski tips and both feet through each of the 6 gates in order, then over the finish line. (Embodied Olympics `alpine-slalom`; the robot as a service: its head cameras, joint sensors and IMUs, and the venue's fixed camera only.)

## Difficulty
Skis glide and carve: to turn, the robot must roll its skis onto their edges and lean its body into the turn so that the snow's force passes through its centre of mass, with no walking or running policy to help; each change of turn shifts its whole body across the skis. Speed builds on the slope, and a lean that does not match the turn throws it on the snow. In standard mode the agent has no simulator and no state: it sees through the robot's cameras, acts through the client, and has 50 resets (only the last attempt is graded).

## Reference solution
None (as `embodied-olympics-fun-alpine-slalom-i00-privileged`).

## Verification
The sim sidecar's referee judges the live attempt, and that is the verdict (`final.json`: `graded_by` live, `replays` 0;
no grading replay, owner 2026-10-17); the frozen start is checked on the live simulator (`instance_mismatch`:
infrastructure), and the video is rendered from the recorded attempt afterwards. `/tests/grade.py` reads
`/data/final.json`: pass iff the live attempt succeeded. The
rules: FIS alpine slalom, one run (a shortened course). The course is a groomed slope falling at 6.0 degrees along +x (the fall line), 40.0 m wide, centred on y = 0. Each of your feet is bound to a slalom ski. You stand still at the start at t = 0 and are released. There are 6 open gates: gate 1's line is 9.0 m down the slope from your start (measured along the slope), each next one 11.0 m further. A gate is two flexible poles across the slope, red and blue in turn: its turning pole stands 0.8 m from the centre line, on the left (y > 0) for gate 1 and then alternately right and left, its outside pole 4.0 m further out on the same side. Its gate line is the line on the snow from the turning pole to the outside pole. A gate is passed when both ski tips (the front ends of the skis) and both feet have crossed its gate line going down the slope; a point that comes back up across a gate line must cross it again. Gates are passed in order: if any of those four points goes down past the next gate's line, or the finish line, while a gate is not passed, you missed it and the run fails. Hitting or folding a pole is allowed. The finish line is 70.0 m down the slope from the start, 12.0 m wide and centred on y = 0: you succeed when, after every gate, both feet cross it between its posts, within 60 s. The run fails if you fall (a part of you other than your feet and your hands touches the snow, or your pelvis is less than 0.35 m above the snow, measured along the slope's normal), if you stop (your pelvis slower than 0.1 m/s for 1.5 s; ICR 614.2.3), if a foot crosses the finish line outside its posts, or at 60 s. No oracle; human review in the pull request that adds the benchmark (PROTOCOL §2.7).

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 3 | 2 |
| Memory | 5 GB | 3 GB |
| GPU memory | 0 GB | 0 GB |

- GPU: not needed.
- Measured: author estimate (an RTX 4090 workstation, 2026-10-11): CPU-only; the sim sidecar steps and renders on one or two cores (under 1 GB), the agent's container holds the harness and the client
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), the agent's container: no GPU, 8 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 0 GiB GPU memory, 16 GiB memory. Limits measured: author estimate: no trials yet; the agent's container 8 GiB under its 10 GiB cap; whole trial: + the sim service's 8 GiB cap; disk 20 GiB for downloaded tools and controllers; processes 512: design default.

## Details
| | |
|---|---|
| Category | track-and-jumps |
| Rule book | FIS International Competition Rules (ICR), Book IV, Alpine, edition 2025: Art. 614.2 (gates, no continuing after a missed gate or a stop), 661.4.1 and 661.4.1.2 (a gate is passed when both ski tips and both feet cross the gate line, the shortest line between the turning pole and the outside pole), 680 (flex poles), 801 (slalom technical data: gates of two poles, alternately red and blue, 4.0 m wide (4-6 m), turning poles of consecutive gates 6-13 m apart, direction changes 30-35 % of the vertical drop +/- 3), 802 (the slope: about 33-45 %, may be gentler); deviations: ICR 801.1 (vertical drop: men 140-220 m, U16-U14 100-160 m, at least 50 m in a three-run slalom): the course is 70.0 m long on a slope of 6.0 degrees, a vertical drop of about a tenth of the shortest official one, with 6 gates, so that the robot's run (about 20 s) fits the episode; the 6 gates' direction changes stay within 801.2.4's 30-35 % of the vertical drop, plus or minus 3. · ICR 802.1.1 (a slope of about 33-45 %; a gentler one is allowed): 6.0 degrees, a gentle slope, chosen for the shortened course: the robot has no poles to push and no speed to lose in the few turns of a short course. · ICR 803.2-803.4 (a slalom sets open and vertical gates, hairpins, vertical and delayed combinations): only open gates, at a regular 11.0 m (801.2.3: 6-13 m), their turning poles alternately 0.8 m left and right of the course's centre line (the course setter's choice; ICR sets no offset). · ICR 805 (the start: the racer pushes out of the start gate, the start wand starts the clock): the robot stands still on the slope at the start, on its skis, and is released at t = 0; the run's time is the episode's. · ICR 614.2.4 and 628 (a racer who falls may get up and go on): a fall ends the run here (the families' fall rule): a part of the robot other than its feet and its hands on the snow, or its pelvis less than 0.35 m above the snow. · Equipment: no ski poles (the robot's hands are bare pads that cannot hold one), no ski boots (the feet are bound directly); the binding is rigid and never releases; the skis are rigid (no bending; the sidecut's arc is the snow model's, below). · Snow and air: the ski-snow contact is the freeski model of the CHIGUI0/roboolympics project (MIT, ported): kinetic friction along the ski after Wolfsperger et al. 2021, the edge's hold across it after Mossner et al. 2013 with race-piste snow, the sidecut's arc after Howe 2001; the snow keeps no ruts. Still air at 1500 m (1.052 kg/m^3), drag on every link; no lift. · Gates: a flex pole is a 28 mm pole on a sprung two-axis hinge at the snow (return stiffness and mass are estimates; ICR 680 requires flex turning poles but publishes no stiffness); it can be hit and folds. |
| Stages | gate 1 → gate 2 → gate 3 → gate 4 → gate 5 → gate 6 → finish |
| Service | `environment/sim/server.py` (the Embodied Olympics adapter) on `rcb_service.py` (eai-standard/2.1), `rcb_rfo.py`, `rfo/`, `rfo_task.py`, `instance.json` (= the privileged twin's) |
| Observations | `head_left`, `head_right` (RGB 270 × 480, depth on `head_left`), `venue` (fixed); proprioception: joints, torques, targets, two IMUs, grip state |
| Episode | 6000 control steps (60 s at 100 Hz); 50 resets, the last attempt graded |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; docs/benchmarks/embodied-olympics.md) |
