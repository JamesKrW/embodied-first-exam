# Ablations: reset + save/load, privileged state queries, both

These runs sit between privileged and standard mode. Each one gives the standard-mode agent one thing that only the
privileged mode has. They are ablations, not official results, and they are not ranked with the official runs. Each
one runs on its own branch:

| Branch | `sim` service | Instruction section | Run key |
|---|---|---|---|
| `ablation/unlimited-reset` | `RCB_STATE_OPS: "1"`, `RCB_RESET_BUDGET` 1000 | `## This run: unlimited resets, save and load` | `codex-gpt6_astra_resetinf-high` |
| `ablation/object-pose` | `RCB_STATE_QUERY: "1"` | `## This run: privileged state queries` | `codex-gpt6_astra_objpose-high` |
| `ablation/object-pose-unlimited-reset` | both | both sections | `codex-gpt6_astra_objpose_resetinf-high` |

- **Base run**: `codex-gpt6_astra-high`, with the same task contents except the switches and the instruction section.
- **Run metadata** (`base_run`, `ablation`, `setting`): the reset run is labelled "reset + save/load", the query run
  "standard + privileged state queries".
- **Task set**: the core set's tasks whose default mode is standard or standard-realtime, minus the excluded ones below.
- **Switches**: both are off by default. Every branch carries the code for both, and only the task dirs set them.
- **Rules**: `docs/protocol/standard_mode.md`, Part 1, "Reset, save and load" and "Privileged state queries".
- **Task dirs**: `scripts/ablation_tasks.py apply reset|query|both <task dirs>` writes them, then `normalize`.

## Reset + save/load: coverage

Snapshots are exact unless marked approximate.

| Coverage | Benchmarks |
|---|---|
| reset, save and load | BEHAVIOR-1K (approximate: serialized OmniGibson state; tasks whose transition rules create or remove objects log `object_set_changed`), EO Fun, DexHand, DexToolBench and MuJoCo Playground ALOHA (server_mj), MyoSuite, BiGym2, PandaDyn, HumanoidBench, iSpatialGym (approximate), RoboLab (approximate), RoboCasa / RoboCasa-GR1 / RoboCasa365 (approximate), KinDER, RoboTwin 2.0 (a load replays the graded rows from a rebuilt start), RoboWits (approximate), MuJoCo Playground Go1 / Spot / Barkour, DexJoCo, MetaWorld+, MolmoSpaces, VLABench, YAMLab (approximate), RoboARC IKEA, RoboPaint calligraphy (approximate), roboEngineer (its policy interface: a reset is a deployment round, save/load of a development episode, re-simulated and digest-checked; tasks/roboengineer/README.md "Ablations"), EmbodiedSWE (the steps after a restore can differ slightly: GPU PhysX solver caches), Embodied Safety Exam (a load replays the saved path from the frozen start and checks its state hash; a violation committed in a discarded episode is logged and still fails the trial; an exit ends the run), LIBERO-Max (the simulator runs in a worker process: a reset starts a new worker, a load replays the path in one and checks its digest) |
| reset only (no `state_ops` in `/spec`) | RoboARC TopoGym (the tooling keeps only the start as a rewind target), ManiGuard (the state dump leaves out grasp constraints), RoboDojo and AM-Bench (no physics snapshot), RoboPaint acrylic / oil (the MyPaint brush engine's state cannot be copied) |
| not in this ablation | Embodied Olympics match table tennis G1 (its section below) |

## Privileged state queries: coverage

Every task of the set (the robot itself is always an entity). An entity's fields are only `pos`, `quat`, `size`,
`bbox`, `lin_vel`, `ang_vel`, `joint_pos`, `joint_vel`, `joint_range` (the robot's and articulated objects' joints) and
`height_above_ground`; RoboPaint adds its sheets' paint `coverage` and its wells' `rgb`. Names and notes are neutral
(no target / goal / distractor or hazard roles, no hidden facts); goal markers and goal zones are not listed; particle
systems and objects that do not exist at the episode's start are not listed, and a body the simulator removes keeps
its last pose. Per benchmark, what `GET /state/entities` lists:

Every table has the robot (`robot`: base or root pose, velocities, height above the ground, joints) and its key links
(`robot_link`: end effectors / TCPs, hands, fingertips, feet, pelvis, torso, head where the robot has them). Then:

| Benchmark | Frame | Objects, articulations, scene features |
|---|---|---|
| BEHAVIOR-1K | OmniGibson world, xyzw | the BDDL scope objects present at the start (no particle systems); jointed ones as articulations, fixed ones as scene features |
| iSpatialGym | odometry (the base at the episode's start), xyzw | every object of the scene except the room shell, as `<category>_<k>` |
| ManiGuard | world, xyzw | every scene object except the goal marker, as `<category>_<k>` (no particle systems) |
| RoboLab, RoboDojo, EmbodiedSWE | world (RoboLab, RoboDojo: the environment's own origin) | the scene's objects (deformables and fluids: centroid and bbox), articulations with their joints, the table |
| RoboPaint | world, wxyz | brush (holder), brush tip, board, plate, paint wells (pose, size, colour), the sheets and their 8 x 8 regions with paint coverage |
| DexHand, DexJoCo, MolmoSpaces, BiGym2, MetaWorld+, DexToolBench, MuJoCo Playground, LIBERO-Max | MuJoCo world | free bodies (pose, velocities, size / bbox), jointed bodies as articulations, static colliding bodies as scene features (MuJoCo Playground Go1 / Spot / Barkour: trunk, IMU, feet, floor) |
| KinDER | odometry (the base at the episode's start) | movable objects with KinDER's bounding boxes, drawers as articulations, fixtures |
| VLABench, PandaDyn, RoboTwin 2.0, RoboWits, YAMLab, AM-Bench | the simulator's world (AM-Bench: the environment origin) | the task's actors / objects, articulations, the table and other static actors |
| RoboCasa, RoboCasa-GR1, RoboCasa365 | MuJoCo world | the episode's objects as `<category>_<k>`; jointed fixtures (faucets, microwave and cabinet doors) as articulations; the other fixtures |
| RoboARC, RoboARC IKEA | the robot frame (the table's height above the floor is never given: heights are above the table top) | the scene's colliding bodies (deformables: centroid and bbox; multi-link: the root link), lock discs and hinged parts as articulations, the table / bench / pedestals as their top faces |
| EO Fun | the venue (floor z = 0) | free bodies (props, balls; a chain of alike bodies as one rope), jointed fixtures (springboard, bag, bed), the field's geometry (track, lanes, beam, bar, water, sand, gates); ball machines and other launchers as pose and box only |
| HumanoidBench | MuJoCo world | props, the door's hinges, the hurdles and barriers |
| MyoSuite | MyoSuite world | the objects and balls, the opponent and goalkeeper (pose and box), bimanual's pillars, the terrain |
| Embodied Safety Exam | the bench or room frame | every scene object as `<category>_<i>` (pose, size, bbox, velocities), the table, floor and walls |
| roboEngineer | MuJoCo world, wxyz | of a completed round's development episodes at their telemetry times: the robot (joints as its encoders read them), the weave poles, the parking slot, parked cars and curb, the landing pad (tasks/roboengineer/README.md "Ablations") |

Never in a table: task predicates, success flags, progress, rewards, distances to a goal, thresholds, referee or hold
counters, goal markers or zones, contents or device states, hazard labels.

## Recorded per trial

`final.json` / `reward.json`:
- `resets`, `saves`, `loads`, `episodes`;
- on a success, `resets_at_success` and `loads_at_success`;
- `first_success`: 1 only when the graded success needed no reset and no load;
- `state_queries`: the `/state/query` calls answered (the query run);
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
