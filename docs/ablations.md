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
| reset, save and load | BEHAVIOR-1K (approximate: serialized OmniGibson state; tasks whose transition rules create or remove objects log `object_set_changed`), EO Fun, DexHand, DexToolBench and MuJoCo Playground ALOHA (server_mj), MyoSuite, BiGym2, PandaDyn, HumanoidBench, iSpatialGym (approximate), RoboLab (approximate), RoboCasa / RoboCasa-GR1 / RoboCasa365 (approximate), KinDER, RoboTwin 2.0 (a load replays the graded rows from a rebuilt start), RoboWits (approximate), MuJoCo Playground Go1 / Spot / Barkour, DexJoCo, MetaWorld+, MolmoSpaces, VLABench, YAMLab (approximate), RoboARC IKEA, RoboPaint calligraphy (approximate), EmbodiedSWE (the steps after a restore can differ slightly: GPU PhysX solver caches) |
| reset only (no `state_ops` in `/spec`) | RoboARC TopoGym (the tooling keeps only the start as a rewind target), ManiGuard (the state dump leaves out grasp constraints), RoboDojo and AM-Bench (no physics snapshot), RoboPaint acrylic / oil (the MyPaint brush engine's state cannot be copied) |
| not in this ablation | Embodied Safety Exam (one graded episode by design: harm is latched per episode), LIBERO-Max (one simulator per process) |

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
