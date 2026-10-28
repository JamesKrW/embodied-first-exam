# robolab-take-spatula-off-shelf-i00-privileged

| | |
|---|---|
| Benchmark | RoboLab |
| Mode | privileged |
| Twin | `robolab-take-spatula-off-shelf-i00-standard` |
| Simulator / robot | `robolab` / `franka-robotiq-2f85` |
| Family / instance | `take_spatula_off_shelf` / seed 0 |
| Oracle | `none` — No reference solution: RoboLab ships no expert or planner. Positive examples graded 1 by the separate verifier: robolab-take-spatula-off-shelf-i__vipvPdN (robolab-take-spatula-off-shelf-i00-privileged before 2026-10-14, run codex-gpt6_luna-xhigh, robolab-lunacx-0925-2038); robolab-take-spatula-off-shelf-i__TqQfCCX (robolab-take-spatula-off-shelf-i00-standard before 2026-10-14, run codex-gpt6_luna-xhigh, robolab-lunaxh-limited-0928). The positives were graded before 2026-10-14 (one-finger gripper contact, no settle hold); not re-graded under the 2026-10-14 term unless stated here. Human review per family (PROTOCOL §2.7). |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:robolab-0.2.1` |

## Task
Take the spatula off the shelf and put it on the table.

## Difficulty
RoboLab labels this task "moderate" (1 subtasks; attributes: affordance, spatial); its success term (`object_outside_of_and_on_surface`) asks for the spatula taken off the wire shelf and laid on the table. The tooling gives only thin primitives (move_ee on Isaac Lab's differential IK, joint moves, gripper, hold), so every grasp, transport and placement is the agent's to build. RoboLab ships no expert solution.

## Reference solution
None: RoboLab ships no expert or planner, and no reference solution has been written for this family (`oracle = "none"`). Positive examples, model trials the separate verifier scored 1:
- `robolab-take-spatula-off-shelf-i__vipvPdN` (privileged mode, on this task before 2026-10-14; run `codex-gpt6_luna-xhigh`, batch `robolab-lunacx-0925-2038`)
- `robolab-take-spatula-off-shelf-i__TqQfCCX` (standard mode, on this task before 2026-10-14; run `codex-gpt6_luna-xhigh`, batch `robolab-lunaxh-limited-0928`)

## Verification
Open-loop replay of `/app/output/trajectory.npz` from `tests/instance.json` in two fresh processes; pass iff RoboLab's
`success` term holds in every replay, the final-state hashes agree and the rebuilt frozen scene matches the recorded
one (`tests/grade.py`), plus `/logs/verifier/replay.mp4`. The trajectory is the per-control-step (T, 8) action record;
with PhysX on the CPU a fresh process reproduces the live run bit for bit (the GPU pipeline does not, which is why
physics is pinned to the CPU); later replays inside one process can differ in the last bits, hence two fresh processes.
RoboLab's success term is instantaneous: it is checked after every control step and the first step where it holds ends
the episode (RoboLab's own termination; the replay stops there), so a goal state that holds at one control step counts.

No oracle; human review in PR #3 (PROTOCOL §2.7: per family, this task and its twin, a positive example and the `nop` result).

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 29 GiB | 16384 | 24 GiB | 20 GiB |
| verifier | no cap | no cap | no cap | 24 GiB | no cap |

- GPU: an RTX-class NVIDIA GPU (ray-tracing cores), one of the models in `gpu_types`: L40S, L40, L4, RTX 4090, RTX 6000 Ada, RTX PRO 6000 Blackwell, held by `main`.
- Recommended GPU: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: GPU memory 4.0-5.5 GB per trial (one simulator), anon memory 7.6-10.3 GB, CPU p90 2.4-8 cores over 7 trials (a per-container sampler every 30 s); per-trial peaks of past trials 14.5-15.7 GB of GPU memory, which include agents' extra simulator processes and Isaac's card-sized pools; one Isaac simulator per 32 GB of host memory
- Measured (caps): GPU memory: per-trial peaks of 15.7 GiB on a 48 GB card and 15.3 GiB on a 24 GB card, agents' extra simulators included -> max(1.5 x peak, peak + 4), rounded up: 24 GiB; processes and disk: defaults, not measured

## Details
| | |
|---|---|
| Simulator | NVIDIA RoboLab v0.3.1 (Isaac Lab 2.3.2 / Isaac Sim 5.1, PhysX on the CPU, RTX rendering), Franka Panda + Robotiq 2F-85 (DROID setup) |
| RoboLab task | `TakeSpatulaOffShelfTask`; the frozen instance (the full scene state after RoboLab's canonical seed-0 reset, with its hash) is `environment/instance.json` |
| Image | built locally on every host (`images/robolab/BUILD_ONLY`: it contains Isaac Sim); needs an RTX-class NVIDIA GPU (`environment/docker-compose.yaml`) |
| Budget | the agent's cutoff is the `Agent time budget` row above (this task's `[agent] timeout_sec`); out of it, every Python process pays ~2 min of Isaac Sim launch + scene build (tasks/robolab/README.md) |
| Benchmark notes | tasks/robolab/README.md |

### Change Log
- 2026-10-10: grading robustness and time limits; the scene, the frozen instance and the pass rule are
  unchanged, so the task keeps its name. `tests/grade.py`: each fresh-process replay has
  max(1800 s, 900 s + rows / 7.5) instead of a flat 1800 s and `[verifier] timeout_sec` is 29100 s (was 2400 s, less than two
  replays at their limit), so a long trajectory is graded instead of the verifier being killed; a replay that still runs
  out of time is `replay_timed_out` with `grader_error` = `infra_error` = 1; a file that is not a readable trajectory is
  `bad_trajectory` (was `grader_error`); `instance_ok` = 0 (the rebuilt scene differs on this host) sets `grader_error` and
  `infra_error`; every reward has `deterministic` and `infra_error`. Tooling: a failed replay video never voids the
  replay, and the video of a trajectory longer than 9,000 rows is a time lapse (at most 9,000 frames).
- 2026-10-10 (owner decision): the agent's network egress is restricted to the model APIs (`[agent] network_mode = "allowlist"`, the hosts of `scripts/set_agent_egress.py`; since 2026-10-11 these include the package registries, PyPI and the Debian / Ubuntu archives, so that `pip install` and `apt-get install` work); it was open before (Harbor's default), and agents could download upstream sources, reference solutions and robot models from their own shell. The simulator image is `efe-private:robolab-0.1.2` (was 0.1.1): it holds the MDL materials and textures that RoboLab's asset library fetched from NVIDIA's Omniverse content bucket whenever a scene was built, so that the cameras show under the allowlist what they showed with open egress (on 0.1.1 without that download the table renders red). The scene, the frozen instance, the physics, the tooling and the pass rule are unchanged, so the task keeps its name; results from before this date were obtained with open egress.
- 2026-10-10: the agent's image no longer holds RoboLab's recorded example episode (`/opt/robolab/examples/recorded_data`: the actions of a successful `RubiksCubeAndBananaTask` run, a solution of `robolab-rubiks-cube-and-banana-i00-privileged` within the agent's reach, PROTOCOL §2.7); `environment/Dockerfile` removes it. Nothing at run time reads it (only RoboLab's `examples/run_recorded.py` does), so the scene, the frozen instance, the tooling and the pass rule are unchanged; a privileged trial from before this date could read that file.
- 2026-10-14 (in place: a new DIGEST, PROTOCOL v2.0 §1 and H.8; the verifier changed). From the failure analysis of the 10-14 results (19 of 194 RoboLab successes fired before the task was done). (1) "The gripper" of every contact check is both Robotiq fingers (`rcb_robolab.GRIPPER_CONTACT`, RoboLab's own alias group: an object touching either finger is held); RoboLab's DROID setup watched the left inner finger alone, so an object pinched by the other finger read as released and the success fired mid-carry. (2) `take_measuring_spoon_out` and `cooking_clear_plate`, whose words say to put the objects on the table, use RoboLab's own `object_outside_of_and_on_surface` (surface `table`) with the same parameters instead of `object_outside_of` (an object lifted out of the container or knocked off the table counted). (3) After the last row the replay holds it for 30 more control steps (2 s), the term still checked, so a release in the last rows completes. (4) The instruction says that only the first episode of a process replays exactly in a fresh process (an episode after `s.reset()` can end differently). The frozen instance and the time limit are unchanged; results recorded before this date (the earlier DIGEST) are not comparable with this version.
- 2026-10-16 (in place: a new DIGEST, PROTOCOL v2.0 §1 and H.8; texts only). From the instruction audit of 2026-10-16, the instruction now states the rules the grader applies: (1) the Tooling comment on `s.reset()` no longer calls it an exact state restore (the Conventions say that not every PhysX internal is restored). The scene, the frozen instance, the tooling, the success term and the grader are unchanged, so the task keeps its name and results under the 2026-10-14 DIGEST are graded the same way.
