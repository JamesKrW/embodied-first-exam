# embodied-olympics-fun-high-bar-swing-i00-standard

| | |
|---|---|
| Benchmark | Embodied Olympics - Fun |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `embodied-olympics-fun-high-bar-swing-i00-privileged` |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `high_bar_swing` / seed 0 |
| Oracle | `none` — no reference solution; the referee is tested by scripts/roboolympics/tests/test_high_bar_swing.py: positive by a declared magic action (the G1 on a 1.0 m block; two Dex1-1 grippers with the clamp force raised 4.4 times to 528 N per jaw, declared; no weld): the jump is one injected velocity (1.82 m/s up, a 0.17 m rise, aimed so the bar enters the half-closed jaws), the jaws close in flight and catch the bar (1.6 body weights at the catch), then an injected spin as from 60 deg and a gravity-like drive of at most 400 N m about the bar (no joint loaded by it), limp wrists: a full extended circle in 4.58 s, bottom load 5.2 body weights (100 ms mean); the same state hash twice. The clamp force: the device's 120 N holds 1.41 body weights with both hands, a giant swing pulls at least 4.2 (gymnasts 3.5-3.7); 240, 360, 420 and 480 N per jaw all lose the bar in the same magic-action swing, 528 N completes it. Negatives: nop (time), no catch (jaws open: back on the block, time), starting already hanging (held the bar without jumping to it), grippers opened (let go), a fall from the block (the mats), a tucked circle and a circle that turns back (referee-only rotations: no count); stress: 4 s of random joint targets on the bar held (2.9 body weights), then the grippers opened let go in 0.06 s; the jaw pads press at most 5.8 mm into the bar.; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |
| Agent time budget | 7200 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1` |

## Task
Standing on the block under the high bar, jump up and catch the bar with both grippers, then do one giant swing: take your extended body in a full circle around the bar, without letting go. (Embodied Olympics `high-bar-swing`; the robot as a service: its head cameras, joint sensors and IMUs, and the venue's fixed camera only.)

## Difficulty
A real jump: the legs must lift the whole robot about 0.15 m so that the bar enters the open jaws, and the jaws must close in flight. Then nothing pushes you: the swing has to be pumped up from a hang by the legs, the waist and the shoulders, against motors of 88 N m at the hips and 25 N m in the arms, until the body goes over the top of the bar extended; the bottom of a giant swing pulls more than four times your weight through the grippers and the arms, and the grippers slide round the fixed bar against their own friction. In standard mode the agent has no simulator and no state: it sees through the robot's cameras, acts through the client, and has 50 resets (only the last attempt is graded).

## Reference solution
None (as `embodied-olympics-fun-high-bar-swing-i00-privileged`).

## Verification
The sim sidecar's referee judges the live attempt, and that is the verdict (`final.json`: `graded_by` live, `replays` 0;
no grading replay, owner 2026-10-17); the frozen start is checked on the live simulator (`instance_mismatch`:
infrastructure), and the video is rendered from the recorded attempt afterwards. `/tests/grade.py` reads
`/data/final.json`: pass iff the live attempt succeeded. The
rules: The bar is a competition horizontal bar: 0.028 m thick, 2.8 m above the floor, 2.4 m long between its two uprights, along the y axis; it is fixed and does not turn. Your hands are two-finger grippers (each jaw clamps with up to 528.0 N); they hold the bar by the friction of their jaws only, so the bar can slide along the jaws and out of them. A hand holds the bar while the bar touches both of that gripper's jaws. You start standing on the block and must jump to the bar: the catch is the moment both hands hold the bar after you have left the block (holding it before leaving the block fails). Before the catch you fail if you touch the mats or the floor; you may land back on the block and jump again. After the catch, when for more than 0.05 s a hand has not held the bar you have let go (whether you opened the gripper or the bar slipped out), and there is no regrip; only the grippers may touch the bar; touching the block, the uprights, the mats or the floor fails. Your swing angle is the angle between straight down and the line from the bar to your centre of mass, seen from the side. The giant swing is a full circle of that line after the catch: the angle must travel 360.0 degrees in one direction (forwards or backwards) without turning back; a turn back of more than 5.0 degrees starts the count again from where you turned. The body must be extended over the top: whenever your centre of mass is higher than the bar it must be at least 0.565 m from the midpoint of your two wrists (nine tenths of the dead hang's 0.628 m: a slight pike or arch passes, a tuck or bent arms do not); if it comes closer there, the count starts again from that point. You succeed the moment the count reaches 360.0 degrees. Below the bar you may bend as you like to build up the swing. You fail after 60 s. No oracle; human review in the pull request that adds the benchmark (PROTOCOL §2.7).

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
| Rule book | FIG Apparatus Norms, men's artistic gymnastics, horizontal bar: the bar 2.80 m above the floor (2.60 m above the 0.20 m landing mats), 2.40 m between the uprights, 28 mm in diameter, fixed in the uprights; FIG Code of Points (men's artistic gymnastics), horizontal bar: the giant swing (a full circle round the bar with the body extended, in hang); deviations: the grip (the hands on the bar): each hand is a two-finger gripper, Unitree's Dex1-1 (90 mm stroke, 80 mm jaws with rubber pads; sold for the G1), closed across the bar; it holds the bar by the jaws' friction alone, nothing is welded, and the bar can slide along the jaws and out of them; the bar is fixed, as the norm's is, and a hand turns on it only by its pads sliding round it or by the wrist's joints · the grip force: each jaw clamps with up to 528.0 N, 4.4 times the Dex1-1's 120 N, because the device's force holds only 1.41 body weights with both hands (measured) while the bottom of a giant swing pulls 4.2 body weights or more (4.75 to 5.8 measured in this venue; gymnasts 3.5 to 3.7, Kopp & Reid 1980): 533 N per jaw is that load's need with a margin of 1.5. The device's own rating is a 5 kg friction-grasp load: this hand is far stronger than any Unitree hand · the grip (the hands stay on the bar): defined by contact: a hand holds the bar while the bar touches both of its gripper's jaws; after the catch a break of up to 0.05 s (the pads' contact flickering under a load) is not letting go · the mount: the robot (1.32 m) starts standing on a 1.0 m mounting block under the bar and jumps to it (a rise of about 0.15 m), because it cannot reach a 2.80 m bar from the mats (a gymnast is lifted to the bar or jumps from a springboard) · FIG Code of Points (the body extended, judged by eye): the extension is measured from the wrists: whenever the centre of mass is above the bar it must be at least 0.565 m from the midpoint of the wrists (nine tenths of the dead hang's 0.628 m), because the referee has positions, not a judge's eye, and the grippers may fold at the wrists on the fixed bar · FIG Apparatus Norms, horizontal bar (elasticity): the bar is rigid, not the norm's elastic steel bar on tensioned cables, because the simulator has rigid bodies only (the real bar's spring helps the gymnast: this is not easier) · FIG Code of Points (a routine is scored): one element is judged pass or fail, because the task is a single action |
| Stages | jumped → caught the bar → over the top → full circle |
| Service | `environment/sim/server.py` (the Embodied Olympics adapter) on `rcb_service.py` (eai-standard/2.1), `rcb_rfo.py`, `rfo/`, `rfo_task.py`, `instance.json` (= the privileged twin's) |
| Observations | `head_left`, `head_right` (RGB 270 × 480, depth on `head_left`), `venue` (fixed); proprioception: joints, torques, targets, two IMUs, grip state |
| Episode | 6000 control steps (60 s at 100 Hz); 50 resets, the last attempt graded |
| Network | open: agents may search for and download public tools, robot models and pretrained controllers (owner, 2026-10-11; tasks/embodied-olympics-fun/README.md) |
