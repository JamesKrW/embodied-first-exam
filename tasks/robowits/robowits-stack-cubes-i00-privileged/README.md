# robowits-stack-cubes-i00-privileged

| | |
|---|---|
| Benchmark | RoboWits |
| Mode | privileged |
| Twin | `robowits-stack-cubes-i00-standard` |
| Simulator / robot | `robowits` / `marvin-bimanual` |
| Family / instance | `stack_cubes` / seed 0 |
| Oracle | `partial` — Reference solution solution/oracle_stack_cubes.py with its skills in solution/ref_primitives.py (written for this instance; RoboWits ships no expert). It scored 1 under Harbor -a oracle on an earlier version of this task: job robowits-stack-cubes-i00-oracle-thin-0925-000706, trial robowits-stack-cubes-i00__2rVdsVe, 2026-09-25 (laptop, Intel; success=1, deterministic=1, instance_exact=1; an uncommitted tree just before dev branch f15dbdb7). Same since then: the frozen instance (state hash 41c4d7e7d7193b0d) and the oracle script (byte-identical to that run's /app/output/gen.py; ref_primitives.py unchanged since f15dbdb7). Changed since then, so the label is partial: the grader tests/grade.py (dev branch 82d76c61 added the GPU-backend exception; this PR writes the verdict before the video) and the tooling rcb_robowits.py (dev branch 5aa848bd, bddfc520, adce3fef, 8f0ebd55, 82d76c61, 6c0b439e, fe5fb9e6, 1515ea4e). Not re-run on this version (static validation only, 2026-10-01). 2026-10-14 (an AMD workstation): Harbor -a oracle scored 1 on a pre-final tree of this change (under a temporary -i01 task name) (trial robowits-stack-cubes-i01-privileged__3ppSJuL; the tooling changed after it only in the BlenderKit service code, which this family does not use), nop 0; the oracle trajectory passes the 2026-10-14 grader in a batch re-grade. |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:robowits-0.1.1` |

## Task
Build a stable two-layer stack so the red cube's center is above the green colored dot. One replayable trajectory from
the frozen instance (evaluation scene 0 of RoboWits task 14), with the privileged tooling (`rcb_robowits.py`: the
scene's state, resets, thin primitives, replay); the success criterion as graded is in `instruction.md`.

## Difficulty
Tabletop bimanual manipulation with a 16-DoF joint-target action space: three 5 cm cubes must be rearranged into a
two-layer stack whose apex sits within 2.5 cm of a marker, and the two base cubes must end up touching. The tooling
gives only thin primitives (IK moves on RoboWits' own pink/pinocchio IK, joint moves, gripper, hold): grasping and
placing are the agent's to build. The wrist joints have ±60° limits (a vertical approach is only reachable for
finger-closing directions within roughly ±60° of world y), the placement tolerances are tight, and the assembled stack
must survive the release and retreat of the gripper. The physics runs at ~700 control steps/s, so iteration is cheap.

## Reference solution
`solution/oracle_stack_cubes.py`: with the left arm, grasp base cube 1 and place it axis-aligned on the marker, grasp
base cube 2 and place it flush against it, grasp the apex cube and place it centred over the marker, then retreat. Its
grasp / place skills live in `solution/ref_primitives.py` (never in the agent image). `solution/solve.sh` records it
into a trajectory.

## Verification
Open-loop replay of `/app/output/trajectory.npz` from `tests/instance.json` in a fresh process, twice; pass iff
RoboWits' `_check_success` holds after every replay, the final-state hashes agree and the rebuilt frozen scene matches
the recorded one (`tests/grade.py`; a scene that does not match is the host's failure, `infra_error = 1`, never the
agent's 0), plus `/logs/verifier/replay.mp4`. What the predicate checks, in words:
`instruction.md`, "Success criterion (as graded)". The trajectory is the per-control-step JOINT_ABS record (T, 16);
Genesis' CPU backend restores the frozen snapshot exactly, so in-process repeats and fresh processes agree bit for bit
on one CPU vendor (tasks/robowits/README.md).

Oracle partial: the reference solution scored 1 on an earlier version of this task and was not re-run on this one
(`oracle_evidence` in `task.toml`); human review in PR #3 (PROTOCOL §2.7).

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 20 GiB | 8192 | 6 GiB | 10 GiB |
| verifier | no cap | no cap | no cap | 6 GiB | — |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: any 8 GB+ NVIDIA GPU (RTX 4090 / L4).
- Measured: GPU memory 0.5-1.4 GB (physics on the CPU, the GPU renders the cameras; per-trial peaks of past trials); memory: sim p90 4.5 GB, privileged container p90 4.7 GB, verifier 2.4 GB (a per-container memory sampler); CPU: an estimate (single-threaded Genesis CPU backend)
- Measured (caps): GPU memory: a per-trial peak of 1.4 GiB -> max(1.5 x peak, peak + 4), rounded up: 6 GiB

## Details
| | |
|---|---|
| RoboWits task | `14-stack-cubes-v0` (RoboWits @ 9cc30ae, Genesis 0.4.7: CPU physics backend, EGL rendering on the GPU) |
| Instance | entry 0 of RoboWits' `eval_dataset_50/14.json`; object poses and the expected state hash in `environment/instance.json` (the verifier's copy in `tests/`) |
| Robot | Marvin bimanual: 2 × 7-DoF arms with Pika parallel grippers; JOINT_ABS actions (T, 16) at 30 Hz |
| GPU | EGL camera rendering only (`environment/docker-compose.yaml`, `tests/docker-compose.yaml`) |

### Change Log
- 2026-10-10 (review; findings RW-1, RW-5, RW-6, RW-7): the verifier reports an infrastructure error (`infra_error = 1`) instead of the agent's 0 when it cannot build the frozen scene or the scene it rebuilt does not match the frozen instance on its host (`instance_ok = 0`: another CPU vendor, tasks/robowits/README.md, Hosts); a `trajectory.npz` that cannot be read or lacks `task` / `actions` is `bad_trajectory`, no longer `grader_error`; `reward.json` always carries `infra_error`; the verifier runs without network (`network_mode = "no-network"`); the instruction states the tooling's cap of 200000 control steps per trajectory instead of leaving it unsaid. The pass rule, the frozen instance, the tooling and the predicate are unchanged; validated with synthetic tests of the grader, not re-run in the simulator.
- 2026-10-10 (review round 2; minor 2): `tests/grade.py` time-boxes the replay video, which it renders after the verdict is on disk: it is skipped or stopped (`video_timed_out = 1`) 180 s before the verifier's `timeout_sec` (`tests/test.sh` passes it as `--budget`). Before, the video replay of a very long trajectory could run into Harbor's verifier timeout, which discards the verdict already written. The verdict, the pass rule, the frozen instance and the tooling are unchanged; validated with a synthetic test of the grader, not re-run in the simulator.
- 2026-10-10 (owner decision): the agent's network egress is restricted to the model APIs (`[agent] network_mode = "allowlist"`, the hosts of `scripts/set_agent_egress.py`; since 2026-10-11 these include the package registries, PyPI and the Debian / Ubuntu archives, so that `pip install` and `apt-get install` work); it was open before (Harbor's default), and agents could download upstream sources, reference solutions and robot models from their own shell. The scene, the frozen instance, the tooling and the pass rule are unchanged, so the task keeps its name; results from before this date were obtained with open egress.
- 2026-10-14 (in place: a new DIGEST, PROTOCOL v2.0 §1 and H.8; the verifier changed). From the failure analysis of the 10-14 results (25 of 84 RoboWits successes did not do the task: an object held in place at the end, a trajectory cut while things still moved, a gripper propping the pages from just outside the 3 cm zone, a cube knocked off the table). (1) A settle window after the last action (`rcb_robowits.Session.settle_check`, the verifier's `replay`): the last command is held for 1 s, then for 2 s the goal must hold at every 6th control step and at the last, and every graded object stay within 2 mm (at rest); before, the predicate was checked once, on the state right after the last action. (2) The goal is RoboWits' predicate (unchanged) with the graded objects released: no robot contact with them (Genesis' contact list); move_cube's cube must also be on the table. (3) The BlenderKit models reach the agent's container through a `blenderkit` service that writes this family's models alone into a volume; the host's folder, its key and the rest of the bundle are no longer mounted in the agent's container. The frozen instance, RoboWits' predicate and the time limit are unchanged; results recorded before this date (the earlier DIGEST) are not comparable with this version.
