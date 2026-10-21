# embodied-olympics-fun-ski-jumping-i00-privileged

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | privileged |
| Twin | `embodied-olympics-fun-ski-jumping-i00-standard` |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `ski_jumping` / seed 0 |
| Oracle | `none` — no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_ski_jumping.py: positive by a real controller (an in-run crouch, extension 1.13 m before the edge, torso-pitch and ski-pitch control in the air, a landing posture; parameters from a search): 10.0 m (K = 10 m), 0.85 s in the air, past the fall line standing. It is fragile: 2 of 10 succeed with its parameters scaled by 0.98 to 1.02 (9.5-10.0 m every time; the failures come after the touchdown), so it is not shipped as a reference solution; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Jump the K 10 hill: ski down the in-run, take off from the table, land on your skis at or beyond the K-point (10.0 m) and ski through the transition past the fall line without falling. (Embodied Olympics `ski-jumping`: one Unitree G1 on the Embodied Olympics simulator, judged by a referee of events, geometry and contact rules.)

## Difficulty
The robot reaches about 10 m/s on the in-run and must jump at the right instant at the edge: too late and it rotates forward, too early and it loses speed; with no lift from the air, only the push at the edge carries it to K. In the air it must set its skis and body for a landing on a slope of about 31 degrees at 13 m/s and hold its balance through the transition's g-load.

## Reference solution
None. No reference controller exists for this family yet: positive by a real controller (an in-run crouch, extension 1.13 m before the edge, torso-pitch and ski-pitch control in the air, a landing posture; parameters from a search): 10.0 m (K = 10 m), 0.85 s in the air, past the fall line standing. It is fragile: 2 of 10 succeed with its parameters scaled by 0.98 to 1.02 (9.5-10.0 m every time; the failures come after the touchdown), so it is not shipped as a reference solution.

## Verification
`/tests/grade.py` replays `/app/output/trajectory.npz` from the frozen start in two fresh processes (`rcb_rfo.py replay`:
one sim row per control step until the referee ends the attempt). Pass iff the referee's verdict is a success in both
and both end in the same state. The rules: FIS ski jumping, one jump on a small hill (K 10.0 m) built to the FIS Construction Norm 2018. Your feet are bound to jumping skis. You stand still at the top of the in-run, which falls at 30.0 degrees and bends (radius 17.0 m) into the take-off table (2.6 m long, falling at 7.73 degrees); the table ends at its edge (x = 0) 0.3 m above the start of the landing hill. You are released at t = 0. The take-off: both skis leave the in-run with your feet past the edge. The landing: the first moment both skis touch the landing hill; the jump's distance is 1.005 times the straight distance from the edge to the point of the hill's profile under the middle of your feet, in half metres (rounded down). The landing hill: the knoll, the landing area from P to L (the K-point at 10.0 m, the hill size HS at 10.0 m / 0.9), the transition (radius 26.0 m) to the flat outrun; the fall line is where the transition ends and the outrun starts. You succeed when, after a landing at a distance of at least 10.0 m, both feet pass the fall line without a fall, within 20 s. You fail: on a fall (a part of you other than your feet on the snow at any time, or your pelvis less than 0.35 m above the snow, measured along the hill's normal, outside the flight, or after the landing your torso tilted more than 80.0 degrees from the hill's normal; a body part that touches the hill before both skis do is a fall, and the distance is measured there), if the flight lasted less than 0.1 s, if the distance is short of 10.0 m when your feet reach the fall line, if you stop (your pelvis slower than 0.1 m/s for 1.5 s), or at 20 s. No oracle; human review in the pull request that adds the benchmark (PROTOCOL §2.7).

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
| Rule book | FIS International Competition Rules (ICR), Book III, Ski Jumping, edition 2024: Art. 411 (jumping hills to the FIS standards), 432 (the distance: where both feet are in full contact with the landing surface; in a fall, where a body part first touches it), the fall line (the end of the landing hill's transition, where the outrun starts) and a fall before it; FIS Jumping Hills Construction Norm 2018 (implementing Art. 411): table angle alpha = w/30 + 7.4 deg, h/n between w/800 + 0.4 and w/1000 + 0.48, in-run angle gamma at most 32 deg on a beginners' hill (w < 30 m), HS = w / 0.9, beta_P = beta + alpha/2 - 2.5 deg, beta_0 = beta_P / 6, at most 0.7 g of centrifugal load at the end of the in-run's transition and 0.8 g in the landing's, distances w = 1.005 x the straight distance from the edge; deviations: Art. 411 and the Construction Norm 2018 (a hill certificate from FIS's JUMP program): the hill is built from the norm's published formulas for w = 10.0 m (K 10, HS 11) with h/n = 0.451, alpha = 7.73 deg, gamma = 30.0 deg; what JUMP computes from data it does not publish is our estimate: the take-off speed 10.52 m/s that carries a point mass with the robot's drag and a take-off push of 1.0 m/s across the table to K, beta = 30.7 deg from an equivalent landing height of 0.3 m at K, the table's height 0.3 m, the knoll a cubic from beta_0 to beta_P, the landing circle's P 2.0 m before K, the transitions as circles (17.0 m in the in-run, the norm's 0.7 g at 10.52 m/s; 26.0 m to the outrun, its 0.8 g), the table 2.6 m long (about a quarter second at take-off speed). · In-run length: the start is where the robot, standing still, reaches about 10.52 m/s at the edge in a crouch (12.3 m of straight in-run above the transition); there is one start (no start gates, no gate changes, no wind), and no in-run track: the skis run on the groomed in-run, held straight by the snow model's edge grip. · The start: no start bench and no push-off; the robot stands still on the in-run, perpendicular to it, and is released at t = 0. · Equipment: jumping skis at the robot's size (see the module's constants); the binding holds the foot rigidly (a jumping binding lets the heel rise; the robot's ankle joint takes that role) and never releases; no ski boots (the feet are bound directly); rigid skis. · Air and snow: still air at 1500 m (1.052 kg/m^3) with drag on every link but no lift (the ported snow-and-air model has no lift: a V-style flight gains nothing aerodynamically); the ski-snow contact is the freeski model of the CHIGUI0/roboolympics project (MIT, ported): kinetic friction after Wolfsperger et al. 2021, the edge's hold after Mossner et al. 2013, a hard prepared snow track. · Judging: no style judges; the success line is the hill's construction point (a jump that reaches K, the distance a hill is built for) with a valid landing, judged by geometry at every 1 ms physics step: the landing is the first instant both skis touch the landing hill, its distance 1.005 x the straight distance from the edge to the profile point under the middle of the feet (the norm's approximation of Art. 432), in half metres; a fall (a body part other than the feet on the snow, or the pelvis less than 0.35 m above the snow after the landing) before the fall line voids the jump; a telemark is not required. |
| Stages | take-off → landing → fall line |
| Instance | `environment/instance.json` = `tests/instance.json`: the family's parameters, t = 0 state hash `a592b9cdba7e08c49d2a49d9c191c027f5668b8d07f67809ab83c08600360df5` |
| Episode | 2000 control steps (20 s at 100 Hz) |
| Action | sim rows of 151 floats per control step (`rfo/world.py`); six action types are views of it |
| Tooling | `/opt/tools/rcb_rfo.py`, `/opt/tools/rfo/` (the world, the action layer, the referee's toolkit; from the files-only image images/embodied-olympics-fun-files/, built from embodied_first_exam/embodied-olympics-fun/) and `environment/rfo_task.py` (this family's rules), the same files in the verifier's `/tests/` |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; tasks/embodied-olympics-fun/README.md) |
