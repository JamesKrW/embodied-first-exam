# embodied-olympics-fun-round-off-i00-standard

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `embodied-olympics-fun-round-off-i00-privileged` |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `round_off` / seed 0 |
| Oracle | `none` — no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_round_off.py: positive by injection (a straight fist handstand placed with the chest facing +x, hand support with real contacts for 0.05 s, then 3.4 m/s up and 3.2 rad/s about the left-right axis: it flies, turns upright and lands on both feet facing -x; the rotation removed at touchdown, the walk at zero command stands it); negatives: nop, landing facing +x (a tinsica), sideways, one foot then the other, hands down at the landing, a jumped half-turn without hands, a crouch hop on the hands (not inverted), a landing that falls, outside the area; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Do a round-off: go from your feet onto your hands, turn half round through the handstand and come down onto both feet together facing back the way you came, then stand. (Embodied Olympics `round-off`; the robot as a service: its head cameras, joint sensors and IMUs, and the venue's fixed camera only.)

## Difficulty
The round-off passes your whole weight over two small fists while the body turns half round in the air above them, and ends in a two-foot landing that must stop the rotation: there is no built-in action for any of it and the arms' motors give 25 N m. In standard mode the agent has no simulator and no state: it sees through the robot's cameras, acts through the client, and has 5 resets (only the last attempt is graded).

## Reference solution
None (as `embodied-olympics-fun-round-off-i00-privileged`).

## Verification
The sim sidecar's referee judges the live attempt, and that is the verdict (`final.json`: `graded_by` live, `replays` 0;
no grading replay, owner 2026-10-17); the frozen start is checked on the live simulator (`instance_mismatch`:
infrastructure), and the video is rendered from the recorded attempt afterwards. `/tests/grade.py` reads
`/data/final.json`: pass iff the live attempt succeeded. The
rules: You face +x at the start. The hand support counts when only your hands touch the floor and your torso is upside down, its long axis (from the pelvis up to the neck) within 45.0 degrees of pointing straight down. After it, the landing: your feet come down with no hand on the floor, the second foot within 0.1 s of the first (both feet together), and when both feet are down your pelvis and your torso face back, -x, each within 45.0 degrees (landing facing +x or sideways is a cartwheel or a Tinsica, not a round-off). Then you stand for 1.0 s without a break: only your feet on the floor, your torso within 20.0 degrees of vertical, your pelvis slower than 0.35 m/s. You fail if any part of you other than your hands and feet touches the floor or your pelvis is lower than 0.25 m (a fall); if after the hand support one foot lands and the other does not follow within 0.1 s, or a hand is on the floor when both feet are down, or you land not facing back; if a hand touches the floor after the landing; if any part of you touches the floor outside the 12.0 m x 12.0 m floor area; or after 30 s. Before the hand support anything is free (steps, a hop, a hurdle). No oracle; human review in the pull request that adds the benchmark (PROTOCOL §2.7).

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
| Category | parkour-apparatus-balance |
| Rule book | FIG Men's Artistic Gymnastics Code of Points 2025-2028: floor exercise, Art. 10.1 (the floor area, 12 m x 12 m), Art. 10.2.1.2a (inside the floor area, the lines included), Art. 10.2.2.6l (a round-off with a 1/4 turn to land facing forward, the Tinsica, is not permitted: the round-off lands facing back), Art. 9.2.12-13 (an element through the handstand deviating more than 45 degrees from it is not recognised), Art. 9.2.17 (landings on the feet, the feet at most 10 cm apart to bring the heels together); the round-off itself as gymnastics defines it: a half turn with hand support, the legs joined over the top, both feet landing together facing the direction you came from; deviations: FIG Apparatus Norms, floor: the floor is rigid and level (friction 1.0), not the norm's sprung floor, because the simulator's ground is rigid and no sourced stiffness of the sprung floor is used here · the hands: the G1 has rubber fists without fingers; any part of the hand may touch the floor · Art. 9.2.17 (feet together at the landing): the robot's legs are 0.24 m apart in its stance and its hips are not built to close them; a two-foot landing here is both feet touching down within 0.1 s of each other, not the feet within 10 cm · a round-off is judged pass or fail, as one element without a run-up requirement or judges: hand support with the torso within 45.0 degrees of upside down, a two-foot landing facing back within 45.0 degrees, and a stand; height, speed and form are not scored |
| Stages | hand support → landing → stand |
| Service | `environment/sim/server.py` (the Embodied Olympics adapter) on `rcb_service.py` (eai-standard/2.1), `rcb_rfo.py`, `rfo/`, `rfo_task.py`, `instance.json` (= the privileged twin's) |
| Observations | `head_left`, `head_right` (RGB 270 × 480, depth on `head_left`), `venue` (fixed); proprioception: joints, torques, targets, two IMUs, grip state |
| Episode | 3000 control steps (30 s at 100 Hz); 5 resets, the last attempt graded |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; docs/benchmarks/embodied-olympics.md) |
