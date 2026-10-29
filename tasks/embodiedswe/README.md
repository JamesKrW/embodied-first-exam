# EmbodiedSWE

EmbodiedSWE's robot benchmark (`robobench`, commit `67f44e8a2ca5`) on Isaac Lab 2.3.2 / Isaac Sim 5.1, PhysX on the
GPU: the agent writes a program, `solution/solve.py` exposing `solve(env)`, that completes a manipulation or
loco-manipulation task by stepping the env. `simulator = "embodiedswe"`. 24 tasks (assembly, packing, puzzle, cutting
and loco-manipulation scenes, one frozen instance each), privileged mode, `task_type = "policy"`, in the full set only:
EmbodiedSWE's 28 published tasks without its 4 deformable ones (they need Newton physics and a networked VLM judge).

| Task | Preset | Robot |
|---|---|---|
| `embodiedswe-assembly-bulb-franka-osc-i00-privileged` | `assembly.bulb.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-puzzle-stack-blocks-g1-pink-ik-i00-privileged` | `puzzle.stack_blocks.g1.pink_ik` | Unitree G1 (fixed pelvis), Pink IK |
| `embodiedswe-locomanip-fruit-delivery-g1-loco-pink-ik-i00-privileged` | `locomanip.fruit_delivery.g1.loco_pink_ik` | Unitree G1 (walking), Pink IK |
| `embodiedswe-assembly-allen-bolt-franka-osc-i00-privileged` | `assembly.allen_bolt.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-assembly-ikea-table-bimanual-franka-osc-i00-privileged` | `assembly.ikea_table.bimanual_franka.osc` | two Franka Pandas, operational-space control |
| `embodiedswe-assembly-nut-thread-franka-osc-i00-privileged` | `assembly.nut_thread.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-assembly-pc-gpu-franka-osc-i00-privileged` | `assembly.pc_gpu.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-assembly-pc-gpu-ram-franka-osc-i00-privileged` | `assembly.pc_gpu_ram.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-assembly-pc-motherboard-franka-osc-i00-privileged` | `assembly.pc_motherboard.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-assembly-pc-ram-franka-osc-i00-privileged` | `assembly.pc_ram.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-packing-pen-holder-franka-osc-i00-privileged` | `packing.pen_holder.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-packing-tool-packing-franka-osc-i00-privileged` | `packing.tool_packing.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-packing-egg-carton-g1-pink-ik-i00-privileged` | `packing.egg_carton.g1.pink_ik` | Unitree G1 (fixed pelvis), Pink IK |
| `embodiedswe-packing-clear-organic-objects-g1-pink-ik-i00-privileged` | `packing.clear_organic_objects.g1.pink_ik` | Unitree G1 (fixed pelvis), Pink IK |
| `embodiedswe-puzzle-coffee-franka-osc-i00-privileged` | `puzzle.coffee.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-puzzle-syringe-bimanual-franka-osc-i00-privileged` | `puzzle.syringe.bimanual_franka.osc` | two Franka Pandas, operational-space control |
| `embodiedswe-puzzle-push-shapes-g1-pink-ik-i00-privileged` | `puzzle.push_shapes.g1.pink_ik` | Unitree G1 (fixed pelvis), Pink IK |
| `embodiedswe-puzzle-classify-objects-g1-pink-ik-i00-privileged` | `puzzle.classify_objects.g1.pink_ik` | Unitree G1 (fixed pelvis), Pink IK |
| `embodiedswe-cutting-slice-franka-osc-i00-privileged` | `cutting.slice.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-cutting-dice-franka-osc-i00-privileged` | `cutting.dice.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-locomanip-box-to-bin-g1-loco-pink-ik-i00-privileged` | `locomanip.box_to_bin.g1.loco_pink_ik` | Unitree G1 (walking), Pink IK |
| `embodiedswe-locomanip-wheel-carry-g1-loco-pink-ik-i00-privileged` | `locomanip.wheel_carry.g1.loco_pink_ik` | Unitree G1 (walking), Pink IK |
| `embodiedswe-assembly-so101-bimanual-franka-osc-i00-privileged` | `assembly.so101.bimanual_franka.osc` | two Franka Pandas, operational-space control |
| `embodiedswe-puzzle-spatula-franka-osc-i00-privileged` | `puzzle.spatula.franka.osc` | Franka Panda, operational-space control |

## Grading

The verifier grades as EmbodiedSWE's harbor verifier (`eval/harbor_support/verifier.py` at the pinned commit) does,
one seed (the frozen instance, seed 0). The candidates are the checkpoints the agent froze with `submit` and the final
`solution/` (see Checkpoints). Each distinct candidate runs in a fresh process: the trusted simulator builds the env
from the frozen instance (EmbodiedSWE's own build and seeded reset, GPU PhysX), attaches the task's grader
(EmbodiedSWE's, unchanged) and runs the candidate's `solve(env)` in a separate process as an unprivileged user,
without network or GPU, behind a proxy env (`embodied_first_exam/benchmarks/embodiedswe/esw_proxy.py`): reads return
copies, `env.step(action)` steps the simulator, everything else is refused. A candidate's process has EmbodiedSWE's
rollout limit, 1800 s of wall clock from its start (the simulator's start-up and the import of `solve.py` count); then
it gets SIGTERM, the solve is ended and the verdict is read on the state at that moment; 30 s later, SIGKILL. A process
that ends without a verdict while solving scores 0; in any other phase it is the verifier's error (`infra_error`).
The verdict is EmbodiedSWE's: success = the grader's end predicate (the cutting and SO101 scenes have no `success()`:
their graders' predicates, enough pieces and all of them on the board, and every screw fastened with the servo and
the forearm seated, are also the tooling's `cut_success` and `so101_success` for the development `check`);
`progress` = its rubric score, the peak recorded progress, so progress made before an exception or the time limit
counts. The spatula scene's tool-only rule (the hand or a finger touching the bread disqualifies the episode) is
enforced by the verifier: PhysX contact reports on the bread, read after every physics substep against the hand and
finger links; a contact with a nonzero force sets success and progress to 0 (`disqualified`, `tool_contact` in
`reward.json`). An exception from `env.step` is raised in `solve`, as EmbodiedSWE's `GradedEnv` lets it propagate.
Actions are applied as given: as in EmbodiedSWE, a Franka's operational-space controller scales the 6 pose deltas by
`pos_scale` / `rot_scale` without clipping them and clamps the joint torques (100 N·m); only an action's shape and
finite values are checked. A run in which no action reached the simulator scores 0.

The result is one candidate, selected as EmbodiedSWE selects: the highest `progress` (their score), ties to higher
success, then to the earliest (`final` is last); a candidate that ran wins a tie against one that never did (an
invalid checkpoint, a missing `solve.py`). `reward.txt` and `reward.json` give that candidate's success and progress
(`selected_submission`, `selected_final`); `results.json` lists every candidate with its status, score and success.
The selected candidate's trajectory, `trajectory.npz` (every action, and Isaac Lab's scene state, each asset's root
pose and velocity and joint positions and velocities, every 0.1 s of simulated time, at most 3,000 states: a longer
run is kept at a coarser spacing), is the run's record; with it come its `rollout.json` and `solve.log`. The video,
`replay.mp4`, is rendered after the verdict in a fresh process: each recorded state is posed on the bodies (no physics
step) and drawn, so the video shows the graded run's own states (`video_source = "recorded_states"`; a scene's
non-physical visuals, such as a liquid level, are not part of the state). Before it, the same process re-executes the
selected trajectory's actions with the camera drawing, as a live recording would, and records whether that reaches
the graded run's final state (`camera_replay_match`, `camera_replay_success` in `video.json`; at most 1800 s). Neither
changes the verdict.

### Checkpoints (a benchmark-specific exception)

As in EmbodiedSWE, the agent may freeze up to 64 checkpoints of its deliverable while it works: `submit "note"`
(`esw_submit.py`, in the image as `/usr/local/bin/submit`) copies `/app/output/solution/` to
`/app/output/submissions/NNNNNN/solution/` with a receipt, `submission.json` (the sha256 of every file and of that
inventory), serialized with flock. The verifier checks every receipt and freezes every candidate before any of them
runs; a checkpoint whose files differ from its receipt, or a directory that is not a checkpoint, scores 0 and is not
run. Byte-identical candidates (the same content hash) are graded once; a later copy takes the earlier one's result.
The other benchmarks of this repository grade one deliverable once; this one keeps EmbodiedSWE's checkpoint
protocol, so its result is comparable with EmbodiedSWE's own. The instruction states the limits.

The verifier's time limit (`task.toml` `[verifier] timeout_sec`) covers the worst case, 64 checkpoints and the final
folder, each distinct and each using its whole limit, then the video:

    (64 + 1) x (1800 + 30) + (900 + 1800 + 600) + 300 = 122,550 s

1800 + 30 is a candidate's limit and grace; 900 + 1800 + 600 the video process's (start-up with cameras, the camera
replay's budget, posing and encoding); 300 the margin. A typical trial grades a few candidates and takes minutes.

Differences from EmbodiedSWE's own grading, on purpose:
- `solve` runs in a sandbox (a separate process as an unprivileged user, no network, no GPU) behind a proxy env, not
  in the simulator's process.
- One frozen instance per task, seed 0 (EmbodiedSWE's harbor protocol averages over its seeds).
- The video poses the selected run's recorded states after the verdict; EmbodiedSWE renders live during the run when
  asked to. Drawing frames between physics steps can change GPU PhysX's later bits (measured: in the stack scene a
  rendered run ends elsewhere; in SO101's it does not), so the graded run draws nothing.
- The spatula scene's tool-only rule is checked. EmbodiedSWE states it in the scene's description and leaves it to
  the harness, whose grading does not check it.
- A malformed submissions directory (an entry that is not a checkpoint, more than 64 checkpoints) leaves those entries
  ungraded with score 0 instead of failing the whole evaluation.

## Time budget

The limit is EmbodiedSWE's rollout limit, 1800 s per candidate, start-up and import included. Measured on an L40S host
(one env, GPU PhysX, `env.step` only): a fresh process starts and builds a scene in 13 to 27 s once the Kit cache is
warm; the stack scene takes 26 ms per control step, fruit delivery 43 ms, spatula 138 ms, and SO101, the slowest (two
arms, 16 physics substeps at 240 Hz, twelve screws with continuous collision detection), 377 to 394 ms, so 30 minutes
hold about 4,500 SO101 control steps, 5 minutes of simulated time.

## Device

PhysX and the G1's frozen locomotion policy run on the GPU (`device="cuda:0"`), as in EmbodiedSWE, in the verifier and
in the agent's development env alike (`make_env()` builds exactly as grading does). On one GPU model a fresh process
repeats a run bit for bit (measured on L40S: the stack scene's closed-loop rollout twice and its recorded actions
replayed twice, SO101 three times, the same final-state hash). The frozen instance records its start state's hash, its
scene state vector and the GPU model that froze it (NVIDIA L40S). The rollout compares its start state: on that model
the hash must match (a mismatch is the verifier's error, `instance_mismatch`); on another model the difference is
recorded (`instance_ok`, `instance_same_gpu`, `instance_max_abs_diff`, `instance_within_tol` at 1e-4 in
`reward.json`), never an error. A run on another GPU model may differ in its bits; a closed-loop solve that reads the
state is robust to that, an open-loop action sequence may not be.

## Image

`ghcr.io/mll-lab-nu/efe-private:embodiedswe-<VERSION>` (`images/embodiedswe/`, BUILD_ONLY): RoboLab's recipe plus
robobench at the pinned commit, scrubbed of graders, smoke probes and solve-calibrated comments, and its asset bundle
by sha256. Isaac Lab's default ground plane and its 3 textures are mirrored into the image by sha256
(`isaac_assets.json`), and the Isaac Lab kit files point their asset root there, so no scene reads the network. Some
upstream binary USD assets carry "Generated from Composed Stage" doc strings with their authors' build paths; these
hold no task content and are left as shipped.

## Provenance

EmbodiedSWE: github.com/EmbodiedSWE/EmbodiedSWE (Apache-2.0), assets on Hugging Face EmbodiedSWE/robobench-assets.
