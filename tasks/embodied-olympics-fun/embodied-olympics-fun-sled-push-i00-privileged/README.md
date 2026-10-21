# embodied-olympics-fun-sled-push-i00-privileged

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | privileged |
| Twin | `embodied-olympics-fun-sled-push-i00-standard` |
| Simulator / robot | `mujoco` / `unitree-h1-2` |
| Family / instance | `sled_push` / seed 0 |
| Oracle | `full` — Reference: the walking policy walks into the sled while inverse kinematics holds both palm pads flat on the 1.04 m poles and a differential press steers; the 5 kg sled (declared: a light frame with the shape and poles of a 57 lb push sled, about 22 N to start on turf at friction 0.45) crosses the 12.5 m line in 46 s. 17 of 20 variations succeed (44-53 s of 60). On the same turf 8 kg moves 1.9 m, 10 kg 0.22 m and 25.9 kg 0.01 m in 60 s: the walking policies push 20-35 N on average. Measured (make_oracles.py): success after 45.99 s of the 60 s limit, live and in a fresh replay, ending in the same state. |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Push the weighted sled with your palms on its two upright handles until the whole sled is over the line 12.5 m ahead, within 60 s. (Embodied Olympics `sled-push`: one Unitree H1-2 on the Embodied Olympics simulator, judged by a referee of events, geometry and contact rules.)

## Difficulty
The sled weighs 5.0 kg on high-friction turf: keeping it moving takes a steady push of about 22 N, against which the walking policy (trained for walking, not pushing) has to keep its balance on 67 kg. Bare palms can only push: if the sled drifts sideways or the hands slip off a handle, nothing holds it.

## Reference solution
`solution/oracle.py` drives the robot through the task's own tooling, closed loop on the privileged state, and writes the trajectory (`solution/solve.sh`). Reference: the walking policy walks into the sled while inverse kinematics holds both palm pads flat on the 1.04 m poles and a differential press steers; the 5 kg sled (declared: a light frame with the shape and poles of a 57 lb push sled, about 22 N to start on turf at friction 0.45) crosses the 12.5 m line in 46 s. 17 of 20 variations succeed (44-53 s of 60). On the same turf 8 kg moves 1.9 m, 10 kg 0.22 m and 25.9 kg 0.01 m in 60 s: the walking policies push 20-35 N on average. Measured (make_oracles.py): success after 45.99 s of the 60 s limit, live and in a fresh replay, ending in the same state.

## Verification
`/tests/grade.py` replays `/app/output/trajectory.npz` from the frozen start in two fresh processes (`rcb_rfo.py replay`:
one sim row per control step until the referee ends the attempt). Pass iff the referee's verdict is a success in both
and both end in the same state. The rules: A prowler-type push sled stands on the turf in front of you, its two upright handles towards you: two steel runners 0.919 m long, 0.62 m apart outside, the handles 0.05 m thick and 1.041 m high at its rear corners; it carries 5.0 kg in all and its runners slide on the turf with a friction coefficient of 0.45. The sled's front edge stands on the start line; the finish line is 12.5 m beyond it, and the lane is 1.0 m to each side of its centre line. Push the sled: only your hands may touch it, and only on the two upright handles (they cannot grip: push, do not pull). The sled has moved once its rear has gone 0.1 m forward. You succeed when the whole sled, its rear included, is over the finish line. You fail if any part of you other than a hand touches the sled, or a hand touches it anywhere but on a handle; if any part of the sled leaves the lane; if you fall (your pelvis lower than 0.5 m above the ground, or any part of you other than your feet on the ground); or after 60 s.

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
| Category | parkour-apparatus-balance |
| Rule book | HYROX Rulebook 2026/27 (the Sled Push station: the sled pushed 50 m in four 12.5 m lengths on the competition turf, each length counted when the whole sled is over the line; the athlete stays in the lane; the loads, the sled included: 102 kg Women Open, 152 kg Men Open and Women Pro, 202 kg Men Pro). There is no rule book for a sled push alone; HYROX's station is the closest official event; deviations: HYROX Sled Push (load): a light sled frame of 5.0 kg with the dimensions and the two 1.04 m push poles of a real weight sled (a 57 lb sled, the lightest with upright poles found: 25.9 kg, 114 N to start on this turf), so that the start takes about 22 N, within what this robot's walking pushes. 5.0 kg in all (0.0 kg of plates) is 7 % of the robot's 67.0 kg. HYROX's lightest official load, 102 kg (Women Open), takes about 450 N to start on the turf: far beyond a 67 kg robot pushing at hip height. · HYROX Sled Push (distance): one 12.5 m length, not four: one action, not the station. · HYROX Sled Push (technique): the sled is pushed with the palms only, on the two upright handles; hands cannot grip (the robot's hands are bare pads), and no other part of the body may touch the sled. · Turf friction: MuJoCo has one friction coefficient per contact: the runners slide on the turf at mu 0.45, the measured push-sled-on-AstroTurf static range (0.37-0.53, Hoffmann 2014) mid value, above the measured kinetic range (0.28-0.35): starting is as hard as on real turf, keeping it moving harder. |
| Stages | moved → line |
| Instance | `environment/instance.json` = `tests/instance.json`: the family's parameters, t = 0 state hash `ce2b8a1521e39128433446247897d2579aab8ef37595260ce69116416cb213fb` |
| Episode | 6000 control steps (60 s at 100 Hz) |
| Action | sim rows of 141 floats per control step (`rfo/world.py`); six action types are views of it |
| Tooling | `environment/rcb_rfo.py`, `environment/rfo/` (the world, the action layer, the referee's toolkit) and `environment/rfo_task.py` (this family's rules), the same files as in `tests/` (scripts/embodied-olympics/shared/) |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; docs/benchmarks/embodied-olympics.md) |
