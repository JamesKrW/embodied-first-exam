# embodied-olympics-fun-rugby-conversion-i00-standard

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `embodied-olympics-fun-rugby-conversion-i00-privileged` |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `rugby_conversion` / seed 0 |
| Oracle | `full` — Reference: a standing power kick through lowcmd; the ball passes the goal plane at 4.5 m (the bar is at 3.0 m); 12 of 13 gain and tee-offset perturbations score. The robot falls after the kick, which is allowed (Law 8 judges the ball). The privileged twin's reference rows (433 control steps, success after 4.33 s) streamed through the client. |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Kick a conversion: with one place kick of a foot, send the rugby ball standing on its tee over the crossbar and between the posts. (Embodied Olympics `rugby-conversion`; the robot as a service: its head cameras, joint sensors and IMUs, and the venue's fixed camera only.)

## Difficulty
To clear a 3 m bar 5 m away the ball must leave the foot at about 10 m/s or more, rising at 45 to 60 degrees: a hard, steep kick under the ball's lower end. The robot has no kicking skill; a hard swing on one leg throws the 33 kg body round, and the robot must stay on its feet until the ball has scored. In standard mode the agent has no simulator and no state: it sees through the robot's cameras, acts through the client, and has 50 resets (only the last attempt is graded).

## Reference solution
`solution/oracle_replay.py` streams the privileged twin's reference solution, row by row, through the client (`solution/rows.npz`, recorded by scripts/embodied-olympics/make_oracles.py). Reference: a standing power kick through lowcmd; the ball passes the goal plane at 4.5 m (the bar is at 3.0 m); 12 of 13 gain and tee-offset perturbations score. The robot falls after the kick, which is allowed (Law 8 judges the ball). The privileged twin's reference rows (433 control steps, success after 4.33 s) streamed through the client.

## Verification
The sim sidecar's referee judges the live attempt, and that is the verdict (`final.json`: `graded_by` live, `replays` 0;
no grading replay, owner 2026-10-17); the frozen start is checked on the live simulator (`instance_mismatch`:
infrastructure), and the video is rendered from the recorded attempt afterwards. `/tests/grade.py` reads
`/data/final.json`: pass iff the live attempt succeeded. The
rules: A size-5 rugby ball (an oval 0.29 m long and 0.19 m across, 0.435 kg, in air) stands upright on a kicking tee (0.06 m high to its cup, 0.08 kg, standing free) on the grass in front of your right foot, on the 5-metre line: its centre 0.3 m ahead of your pelvis and -0.17 m to your left (negative: to the right). The goal line is 5.0 m beyond the ball, square to the direction you face; the goal posts stand on it, 5.6 m apart between their inner faces, centred on the line through the ball square to the goal line, 8.0 m high and 0.12 m thick, padded up to 2.0 m (the padding 0.25 m round each post's axis); the crossbar's top edge is 3.0 m above the ground. The kick is a contact of a foot of yours with the ball: the foot below the ankle, or your lower leg where it lies over the foot (from 0.08 m behind to 0.15 m ahead of the ankle joint) and no more than 0.12 m above the ankle joint; every such contact within 0.15 s of the first belongs to the one kick. You score when the ball's centre passes the goal line's vertical plane above the crossbar's top edge and between the posts' inner faces (extended upwards), and goes on 0.4 m beyond it; it may touch the bar or a post on the way. You fail if any other part of you touches the ball at any time, if you touch it again after the kick, if the ball touches the ground (not the tee) after the kick before it has scored, if it passes the goal line's plane under the bar or outside the posts, if you fall before the kick (your pelvis lower than 0.35 m above the ground, or any part of you other than the feet on the ground), or after 60 s. A fall after the kick does not end the attempt (Law 8 judges the ball, not the kicker; the setting fall_after_kick_fails = 0, where 1 would make any fall a failure).

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
| Category | football-and-volleyball |
| Rule book | World Rugby Laws of the Game (2025): Law 1 (the goal posts 5.6 m apart, the crossbar's top edge 3.0 m above the ground, the posts at least 3.4 m high, padding on the posts; the 5-metre line), Law 2 (the ball: oval, 280-300 mm long, 740-770 mm round end to end, 580-620 mm round across, 410-460 g), Law 8 (8.4: a goal is scored when the ball is kicked over the crossbar and between the goal posts without first touching a team-mate or the ground; 8.7: a conversion is a place-kick or a drop-kick; 8.8: the kick is taken on a line through the place where the try was awarded, parallel to the touchlines, the ball on the ground or on a kicking tee, within 60 seconds); deviations: Law 8.8 (where): the try is taken as awarded under the posts, and the conversion is kicked from the 5-metre line (5.0 m from the goal line) on that line: a choice the law allows, made for this robot, whose measured hard kicks leave the foot at 7 to 11 m/s (a professional's at 25 m/s from 20-40 m). A ball over the bar from there must still rise more than 3 m within 5 m. · Law 2 (the ball's behaviour): a rigid prolate spheroid (a thin shell's mass distribution); its drag in air from wind-tunnel measurements (side-on), its bounce a stand-in (no published rebound test for a rugby ball: a size-5 football's, at the same inflation pressure). It does not deform. · Law 8.8 (the tee): a low plastic kicking tee (0.06 m to the cup), a free body standing on the grass; the ball stands upright in its cup. No placer. · Law 8.4 (between the posts): judged as the ball's centre passing the goal line's vertical plane above the crossbar's top and between the posts' inner faces extended upwards; a ball that hits the bar and goes on over counts, one that comes back does not. A touch judge's flag is replaced by this geometry. · Law 1: only the goal end of the field is there; the posts are 8.0 m high; no wind, no opponents (a conversion is not charged down after the kicker's first movement in this setting). |
| Stages | kick → goal |
| Service | `environment/sim/server.py` (the Embodied Olympics adapter) on `rcb_service.py` (eai-standard/2.1), `rcb_rfo.py`, `rfo/`, `rfo_task.py`, `instance.json` (= the privileged twin's) |
| Observations | `head_left`, `head_right` (RGB 270 × 480, depth on `head_left`), `venue` (fixed); proprioception: joints, torques, targets, two IMUs, grip state |
| Episode | 6000 control steps (60 s at 100 Hz); 50 resets, the last attempt graded |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; docs/benchmarks/embodied-olympics.md) |
