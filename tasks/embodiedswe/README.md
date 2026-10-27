# EmbodiedSWE

EmbodiedSWE's robot benchmark (`robobench`, commit `67f44e8a2ca5`) on Isaac Lab 2.3.2 / Isaac Sim 5.1, PhysX on the
CPU: the agent writes a program, `solution/solve.py` exposing `solve(env)`, that completes a manipulation or
loco-manipulation task by stepping the env. `simulator = "embodiedswe"`. 22 tasks (assembly, packing, puzzle, cutting
and loco-manipulation scenes, one frozen instance each), privileged mode, `task_type = "policy"`, in the full set only.

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

## Grading

The verifier builds the env from the frozen instance (EmbodiedSWE's own build and seeded reset), attaches the task's
grader (EmbodiedSWE's, unchanged) and runs the delivered `solve(env)` in a separate process as an unprivileged user,
without network or GPU, behind a proxy env (`embodied_first_exam/benchmarks/embodiedswe/esw_proxy.py`): reads return copies,
`env.step(action)` steps the simulator, everything else is refused. The verdict is EmbodiedSWE's, read once on the
state when the rollout ends (`solve` returns or raises, its process ends, or the 30-minute limit passes): success = the
grader's end predicate (the cutting scenes have no `success()`: their graders' predicate, enough pieces and all of them
on the board, is also the tooling's `cut_success` for the development `check`); `progress` = its rubric score, the
peak recorded progress, so progress made before an exception or the time limit counts. An exception from
`env.step` is raised in `solve`, as EmbodiedSWE's `GradedEnv` lets it propagate. Actions are applied as given: as in
EmbodiedSWE, a Franka's operational-space controller scales the 6 pose deltas by `pos_scale` / `rot_scale` without
clipping them and clamps the joint torques (100 N·m); only an action's shape and finite values are checked. A run in
which no action reached the simulator scores 0. A fresh process then re-executes the recorded actions: a diagnostic
(`deterministic`, `replay_success`, `replay_progress` in `reward.json`) and the source of the video, which is marked
`video_mismatch` in `video.json` when it does not reach the run's final state; neither changes the verdict.

Differences from EmbodiedSWE's own grading, on purpose:
- PhysX runs on the CPU (see Device), so a fresh process repeats a run bit for bit and the video rendered after the
  verdict shows the graded run.
- Only the final `solution/` is graded, once: no submitted checkpoints, no best-of-N.
- `solve` runs in a sandbox (a separate process as an unprivileged user, no network, no GPU) behind a proxy env, with
  the 30-minute limit on its wall clock.
- One frozen instance per task, seed 0.

## Device

PhysX and the G1's frozen locomotion policy run on the CPU, in the verifier and in the agent's development env alike
(`make_env()` builds exactly as grading does; the tooling refuses an instance or a build on another device). A fresh
CPU process repeats a run bit for bit, which the replayed video needs. On a GPU the locomotion policy is deterministic
too but its bits differ from the CPU's, start states differ from the frozen instances, and a trajectory recorded on
the CPU fails there.

## Image

`ghcr.io/mll-lab-nu/efe-private:embodiedswe-<VERSION>` (`images/embodiedswe/`, BUILD_ONLY): RoboLab's recipe plus
robobench at the pinned commit, scrubbed of graders, smoke probes and solve-calibrated comments, and its asset bundle
by sha256. Isaac Lab's default ground plane and its 3 textures are mirrored into the image by sha256
(`isaac_assets.json`), and the Isaac Lab kit files point their asset root there, so no scene reads the network. Some
upstream binary USD assets carry "Generated from Composed Stage" doc strings with their authors' build paths; these
hold no task content and are left as shipped.

## Provenance

EmbodiedSWE: github.com/EmbodiedSWE/EmbodiedSWE (Apache-2.0), assets on Hugging Face EmbodiedSWE/robobench-assets.
