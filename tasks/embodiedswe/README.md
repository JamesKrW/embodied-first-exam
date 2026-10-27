# EmbodiedSWE

EmbodiedSWE's robot benchmark (`robobench`, commit `67f44e8a2ca5`) on Isaac Lab 2.3.2 / Isaac Sim 5.1, PhysX on the
CPU: the agent writes a program, `solution/solve.py` exposing `solve(env)`, that completes a manipulation or
loco-manipulation task by stepping the env. `simulator = "embodiedswe"`. Pilot (board 0265): three tasks, privileged
mode, `task_type = "policy"`, in the full set only.

| Task | Preset | Robot |
|---|---|---|
| `embodiedswe-assembly-bulb-franka-osc-i00-privileged` | `assembly.bulb.franka.osc` | Franka Panda, operational-space control |
| `embodiedswe-puzzle-stack-blocks-g1-pink-ik-i00-privileged` | `puzzle.stack_blocks.g1.pink_ik` | Unitree G1 (fixed pelvis), Pink IK |
| `embodiedswe-locomanip-fruit-delivery-g1-loco-pink-ik-i00-privileged` | `locomanip.fruit_delivery.g1.loco_pink_ik` | Unitree G1 (walking), Pink IK |

## Grading

The verifier builds the env from the frozen instance (EmbodiedSWE's own build and seeded reset), attaches the task's
grader (EmbodiedSWE's, unchanged) and runs the delivered `solve(env)` in a separate process as an unprivileged user,
without network or GPU, behind a proxy env (`images/embodiedswe/tools/esw_proxy.py`): reads return copies,
`env.step(action)` steps the simulator, everything else is refused. Success = the grader's end predicate on the final
state; `progress` = its rubric score. A fresh process re-executes the recorded actions and must reach the same final
state. No submissions, checkpoints or best-of-N: the final `solution/` is graded once.

## Image

`ghcr.io/mll-lab-nu/efe-private:embodiedswe-<VERSION>` (`images/embodiedswe/`, BUILD_ONLY): RoboLab's recipe plus
robobench at the pinned commit, scrubbed of graders, smoke probes and solve-calibrated comments, and its asset bundle
by sha256.

## Provenance

EmbodiedSWE: github.com/EmbodiedSWE/EmbodiedSWE (Apache-2.0), assets on Hugging Face EmbodiedSWE/robobench-assets.
