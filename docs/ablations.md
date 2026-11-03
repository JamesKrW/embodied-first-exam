# Ablations: reset + save/load, object poses, both

These runs sit between privileged and standard mode. Each one gives the standard-mode agent one thing that only the
privileged mode has. They are ablations, not official results, and they are not ranked with the official runs. Each
one runs on its own branch:

| Branch | `sim` service | Instruction section | Run key |
|---|---|---|---|
| `ablation/unlimited-reset` | `RCB_STATE_OPS: "1"`, `RCB_RESET_BUDGET` 1000 | `## This run: unlimited resets, save and load` | `codex-gpt6_astra_resetinf-high` |
| `ablation/object-pose` | `RCB_EXPOSE_OBJECT_POSES: "1"` | `## This run: object poses` | `codex-gpt6_astra_objpose-high` |
| `ablation/object-pose-unlimited-reset` | both | both, the reset section first | `codex-gpt6_astra_objpose_resetinf-high` |

- **Base run**: `codex-gpt6_astra-high`, with the same task contents except the switches and the instruction section.
- **Run metadata** (`base_run`, `ablation`, `setting`): the reset run is labelled "reset + save/load".
- **Task set**: the core set's tasks whose default mode is standard or standard-realtime, minus the excluded ones below.
- **Switches**: both are off by default. Every branch carries the code for both, and only the task dirs set them.
- **Rules**: `docs/protocol/standard_mode.md`, Part 1, "Reset, save and load" and "Object poses".
- **Task dirs**: `scripts/ablation_tasks.py apply reset|pose|both <task dirs>` writes them, then `normalize`.

## Reset + save/load: coverage

Snapshots are exact unless marked approximate.

| Coverage | Benchmarks |
|---|---|
| reset, save and load | BEHAVIOR-1K (approximate: serialized OmniGibson state; tasks whose transition rules create or remove objects log `object_set_changed`), EO Fun, DexHand, DexToolBench and MuJoCo Playground ALOHA (server_mj), MyoSuite, BiGym2, PandaDyn, HumanoidBench, iSpatialGym (approximate), RoboLab (approximate), RoboCasa / RoboCasa-GR1 / RoboCasa365 (approximate), KinDER, RoboTwin 2.0 (a load replays the graded rows from a rebuilt start), RoboWits (approximate), MuJoCo Playground Go1 / Spot / Barkour, DexJoCo, MetaWorld+, MolmoSpaces, VLABench, YAMLab (approximate), RoboARC IKEA, RoboPaint calligraphy (approximate) |
| reset only (no `state_ops` in `/spec`) | RoboARC TopoGym (the tooling keeps only the start as a rewind target), ManiGuard (the state dump leaves out grasp constraints), RoboDojo and AM-Bench (no physics snapshot), RoboPaint acrylic / oil (the MyPaint brush engine's state cannot be copied) |
| not in this ablation | Embodied Safety Exam (one graded episode by design: harm is latched per episode), LIBERO-Max (one simulator per process), EmbodiedSWE (no reset to the frozen start) |

## Object poses: coverage

| Coverage | Objects and frame |
|---|---|
| poses given | the privileged twin's objects (BEHAVIOR-1K, iSpatialGym, RoboLab, ManiGuard, RoboDojo, RoboWits, YAMLab, AM-Bench, PandaDyn, VLABench, DexJoCo, MolmoSpaces, RoboARC) or the scene's non-robot free bodies; world frame, or the frame the service gives the robot in (EO Fun: the robot at t = 0; HumanoidBench: the pelvis; KinDER, iSpatialGym: odometry; RoboARC: the robot base) |
| not in this ablation | scenes with no object: RoboPaint, MuJoCo Playground Go1 / Spot / Barkour, MyoSuite osl-run and chase-tag (the opponent is an agent), HumanoidBench hurdle and door, the 14 EO Fun families with no free body (backflip, balance-beam, boxing, high-bar-swing, long-jump, platform-10m, springboard-3m, sprint, standing-long-jump, steeplechase, toes-to-bar, trampoline, triple-jump, vault), RoboCasa turn-off-sink-faucet (distractors only); Embodied Safety Exam |

## Recorded per trial

`final.json` / `reward.json`:
- `resets`, `saves`, `loads`, `episodes`;
- on a success, `resets_at_success` and `loads_at_success`;
- `first_success`: 1 only when the graded success needed no reset and no load;
- `final.json` also has `episodes_log`: per episode, when it began (start / reset / load), its agent-clock start and
  end, its control steps, and how it ended.



## Embodied Olympics match

The match tasks (one agent seat against a scripted opponent or a built-in learned seat) run their own match service
(Embodied Olympics core 0.2; table tennis G1: core 0.5), not an eai-standard one. Their ablation dirs come from
`scripts/ablation_eomatch.py apply nosignal|query|reset|both <task dirs>` (then `normalize`): the `sim` service runs
`python3 /srv/rcb_ablation/launch.py`, a layer (`scripts/ablation_eomatch/rcb_ablation/`, copied into the task's
`environment/sim/`) around the unchanged core that adds the calls when its switch is set (with none set it serves what
the core serves), and `tests/rcb_ablation_reward.py` adds the counts below to `reward.json`. Every ablation has the 13
core match tasks.

| Ablation | What the layer does |
|---|---|
| no success signal (`RCB_HIDE_SUCCESS`) | every reply but `/spec` leaves out the score, a point's, rally's or phrase's result, who scored a goal, the black card and the winner; `POST /finish` ends the agent's session, decided as the session's end is: before the start a no-show, during the match abandoned (a valid loss), after the match its result stands. The match's end by its rules (phase `over`) stays visible. |
| state queries (`RCB_STATE_QUERY`) | in the player's own frame, every physical body of the match: its robot (base or pelvis pose, joints), paddle / racket / rods / blade, the opponent (its robot, player, paddle, rods or blade), the ball or shuttle, and the table, net, court, goals, playfield or piste; the eai-standard state table's kinds (`robot`, `robot_link`, `object`, `articulation`, `scene_feature`) and fields only (`pos`, `quat`, `lin_vel`, `ang_vel`, `joint_pos`, `joint_vel`, `joint_range`, `size`, `bbox`). Velocities are the current ones (a drawn opponent's: a backward difference). The fencing bout (phrases measured in MuJoCo, fenced at once) gives the on-guard layout it stands in between phrases. No score, phase, clock, referee state, opponent target, plan or strategy. Read under the world's lock, no step. |
| reset + save/load (`RCB_STATE_OPS`) | reset: the instance built again; save: a copy of the whole match (world, scripted opponent, built-in seat players, generators, score, clocks), exact (a copy and its original agree on the state digest after further steps); load: a copy of it, as a new episode. Table tennis G1's opponent program runs in a process of its own (core 0.5 platform seat): its whole state is pickled there after every observation already sent (its replies received first), restored on a load (checked: the restored program's canonical state hash and its own digest equal the save's) and built new on a reset; the commands it sent that are not yet in effect are part of the save. After either the world is frozen until `start` / `ready` (at most the review timeout); a turn match resumes with its chess clock. A loss by the rules is held at its end (the agent may reset or load); a win ends the run. The graded episode is the last one; `episode.<n>.jsonl` keeps the earlier ones. |

`final.json` / `reward.json` add `resets`, `saves`, `loads`, `episodes`, `first_success`, `resets_at_success`,
`loads_at_success` (reset runs), `state_queries` (query runs), and `success_signal`, `ended_by`, `finished` (no
success signal).
