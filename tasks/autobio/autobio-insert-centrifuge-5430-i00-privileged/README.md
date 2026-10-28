# autobio-insert-centrifuge-5430-i00-privileged

| | |
|---|---|
| Benchmark | AutoBio |
| Mode | privileged |
| Twin | `autobio-insert-centrifuge-5430-i00-standard` |
| Simulator / robot | `autobio` / `ur5e-robotiq-2f85` |
| Family / instance | `insert_centrifuge_5430` / seed 0 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:autobio-0.1.0` |

## Task
AutoBio's `insert_centrifuge_5430` (Load centrifuge rotor), with a UR5e arm (6 joints) with a Robotiq 2F-85 gripper, fixed to the bench: Load a second 1.5 mL tube into the centrifuge's rotor, into the slot diametrically opposite the tube that is already in it (slot 8 of 30, `s.task.slot_id`; the tube in the rotor is in slot 23). The tube to load stands in the plate on the bench. Success at the first control step where the tube's body is within 5 mm of its seated position in that slot (`s.task.final_tar_tubepose`) and its height is between 0.955 and 0.961 m. That reference position is computed when the episode starts: turning the rotor moves the slot away from it. In this instance a tube that has been let go comes to rest 5.0 to 5.1 mm from it, just outside the 5 mm (AutoBio's own expert's tube included), so the check holds while the tube is still being lowered into the slot; the episode ends at that moment.
On the bench: a centrifuge (an Eppendorf 5430 model) with its lid open and a 30-slot rotor holding one 1.5 mL tube, and a well plate with a second 1.5 mL tube standing in it. The deliverable is one replayable trajectory of joint position targets (`/app/output/trajectory.npz`, `(T, 7)`, one row per 2 ms control step).

## Difficulty
AutoBio's level: hard. A 1.5 mL tube is taken from a plate and seated within 5 mm in one particular slot of a 30-slot rotor, inside the centrifuge's bowl. The agent has full access to the simulator and to AutoBio's task code, but not to its scripted expert: it has to build the motion itself from joint position targets at the physics rate.

## Reference solution
AutoBio's scripted expert (load_centrifuge_5430.py @ 0850421), seed 0: the position targets it sends, recorded at every physics step, clipped to the actuators' ranges, rounded to float32 (so the standard twin's client executes the same rows), cut at the first success: `solution/oracle.npz`, 4582 rows, success after 4582 control steps. Replayed and saved by `solution/solve.sh`; re-checked by the privileged verifier (`harbor run -a oracle`: two fresh-process replays, same final state).

## Verification
`tests/grade.py` in a separate sandbox without network: two replays of `/app/output/trajectory.npz`, each in a fresh process, from the verifier's own copy of the instance (`tests/instance.json`; the compiled model and the rebuilt start state are checked against its recorded hashes); pass iff both episodes succeed (the first control step at which AutoBio's check holds ends the episode in success) in the same state (`deterministic`). `nop`: no trajectory, reward 0.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 12 GiB | 8192 | no GPU | 10 GiB |
| verifier | 4 | 8 GiB | no cap | no GPU | — |

- GPU: not needed.
- Measured: CPU only; 4 CPUs and 8 GB per trial, the verifier the same (the benchmark's estimate)

## Details
| | |
|---|---|
| Upstream | AutoBio (github.com/autobio-bench/AutoBio @ 0850421, no licence stated: the image is built locally, images/autobio/BUILD_ONLY) on MuJoCo 3.3.0 |
| Instance | `InsertCentrifuge5430` (`load_centrifuge_5430.py`) after AutoBio's `reset(seed=0)`; model hash `a45ad43e4c568577`, start-state hash `843c5cc8cbffee91`, checked on every load |
| Actions | 7 position targets per control step (2 ms, one physics step); success rule `first` (tasks/autobio/README.md) |
| Resources | 4 CPUs, no GPU (CPU physics, OSMesa rendering) |

### Change Log
- 2026-10-10: the grader's time limits and flags; the scene, the frozen instance and the pass rule are unchanged, so the task keeps its name. `tests/grade.py`: each verdict replay may take 2400 s and the video replay gets what is left of the verifier's 5400 s (every replay had 1700 s), so a reward is written before Harbor's timeout whatever the trajectory's length; a replay without a result also sets `infra_error` (with `grader_error`, as before), every `reward.json` carries `infra_error`, and an error inside the grader still writes a reward (`grader_error = 1`). A trajectory that was graded before gets the same verdict.
- 2026-10-10 (review round 3): `tests/grade.py` only. A byte that is not valid UTF-8 in the session log the agent's tools write (`/app/output/rcb_session_log.jsonl`, read only for the self-reported `agent_*` cost fields) was graded `grader_error = 1` and `infra_error = 1` with reward 0, whatever the trajectory; the log is now read tolerantly and can no longer decide anything. A trial whose log is valid is graded as before; the pass rule is unchanged.
- 2026-10-14: the instruction now states the whole of AutoBio's test: the reference position is computed when the episode starts (turning the rotor moves the slot away from it), and in this instance a released tube rests 5.0 to 5.1 mm from it, so the test holds while the tube is still being lowered (before, this was only in `tasks/autobio/README.md`, Known limits). Text only: the scene, the frozen instance, the reference solution, AutoBio's check and the verifier are unchanged, so the task keeps its name.
