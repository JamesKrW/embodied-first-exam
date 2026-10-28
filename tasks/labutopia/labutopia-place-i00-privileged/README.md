# labutopia-place-i00-privileged

| | |
|---|---|
| Benchmark | LabUtopia |
| Mode | privileged |
| Twin | `labutopia-place-i00-standard` |
| Simulator / robot | `labutopia` / `franka-panda` |
| Family / instance | `place` / seed 0 |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:labutopia-0.1.0` |

## Task
LabUtopia's `place` (Place (LabUtopia's level-2 TransportBeaker is the same task)), with a Franka Panda arm (7 joints) with its two-finger gripper, fixed to a laboratory bench: Pick up the beaker and set it down on the square target plate. Success, at the first control step at which the second condition has held for 60 consecutive control steps (1 s): (1) the beaker's centre has been more than 0.10 m above where it started; (2) after that, its centre is within 0.05 m (horizontally) of the plate's centre and within 0.05 m of its initial height, and the hand (the point between the fingertips) is more than 0.05 m from the beaker's centre.
On the bench in front of the arm: a glass beaker, about 11 cm across and 9 cm tall, and a white square plate, 10 cm x 10 cm, lying flat on the bench. The deliverable is one replayable trajectory of joint targets (`/app/output/trajectory.npz`, `(T, 18)`, one row per 1/60 s control step: 9 position targets, then 9 velocity targets).

## Difficulty
LabUtopia's level 1. The beaker has to be lifted at least 10 cm, carried about half a metre and released upright within 5 cm of the centre of a 10 cm plate. The agent has full access to the simulator and to LabUtopia's scene and task code, but not to its scripted controller or its motion primitives: it has to build the motion itself from joint targets at 60 Hz.

## Reference solution
LabUtopia's scripted controller (place, config level1_place @ 8df7278), seed 0: the joint targets it sends, one row per control step (float32, as PhysX holds them), cut at the first step at which the rule is met: `solution/oracle.npz`, 836 rows, success after 836 control steps. Replayed and saved by `solution/solve.sh`; re-checked by the privileged verifier (`harbor run -a oracle`: two fresh-process replays, same final state).

## Verification
`tests/grade.py` in a separate sandbox without network: two replays of `/app/output/trajectory.npz`, each in a fresh process, from the verifier's own copy of the instance (`tests/instance.json`; the rebuilt placements are checked against its recorded hash); pass iff both episodes succeed (the first control step at which the rule is met ends the episode) in the same state (`deterministic`). `nop`: no trajectory, reward 0.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 8 | 39 GiB | 16384 | 16 GiB | 20 GiB |
| verifier | 8 | 32 GiB | no cap | 16 GiB | no cap |

- GPU: an RTX-class NVIDIA GPU (ray-tracing cores), one of the models in `gpu_types`: RTX 4090, L40S, held by `main`.
- Recommended GPU: RTX 4090 24GB / L40S 48GB.
- Measured: oracle checks on an RTX 4090, lab_001 pick: the rendering container peaks at 17 GB of memory, about 7 GB of GPU memory, 8 CPUs (start-up bursts to 16 cores for about 70 s); lab_003 families 21 GB at the peak (on an 8 vCPU host)
- Measured (caps): GPU memory: about 7 GiB in the oracle on an RTX 4090 -> max(1.5 x peak, peak + 4), at least 16 GiB for Isaac Sim: 16 GiB; processes and disk: defaults, not measured

## Details
| | |
|---|---|
| Upstream | LabUtopia (github.com/Rui-li023/LabUtopia @ 8df7278; code MIT, assets CC BY-NC 4.0) on NVIDIA Isaac Sim 6.1.0 (LabUtopia pins 5.1.0; the image is built locally, images/labutopia/BUILD_ONLY) |
| Instance | config `level1_place` (task `place`, stage `assets/chemistry_lab/lab_001/lab_001.usd`) after LabUtopia's reset with seed 0 and 5 physics steps; placement hash `80b18aa75143294a` (checked on every load), start-state hash `e11556dd2ab6ecd8` |
| Rule | `PlaceRule` in `rcb_labutopia.py`: the object's centre has been more than 0.10 m above where it started; after that it is within 0.05 m (horizontally) of the target's centre and within 0.05 m of its initial height, with the hand more than 0.05 m from the object's centre (released), for 60 consecutive control steps (1 s) |
| Actions | 18 joint targets per control step (1/60 s, one physics step); the first step at which the rule is met ends the episode (tasks/labutopia/README.md) |
| Resources | an RTX-class NVIDIA GPU (Isaac Sim), 8 CPUs, 32 GB of RAM |

### Change Log
- 2026-10-10: the grader's time limits and flags; the scene, the frozen instance and the pass rule are unchanged, so the task keeps its name. `tests/grade.py`: each verdict replay may take 2400 s and the video replay gets what is left of the verifier's 5400 s (each of the three replays had 2400 s, together more than the verifier's 3600 s), so a reward is written before Harbor's timeout whatever the trajectory's length; a replay without a result also sets `infra_error` (with `grader_error`, as before), every `reward.json` carries `infra_error`, and an error inside the grader still writes a reward (`grader_error = 1`). A trajectory that was graded before gets the same verdict.
- 2026-10-10 (review round 2): texts only. The renderer's first start is stated as measured (5 to 27 minutes by stage and host: 26-27 minutes for the `lab_003` stage on an 8-vCPU L40S host, where the texts said about 15), in the instruction and in the tooling's progress message. `pour` only: the instruction says that the last condition counts 84 control steps in all, not in a row (what the rule always did). Nothing else changes.
- 2026-10-10 (review round 3): `tests/grade.py` only. A byte that is not valid UTF-8 in the session log the agent's tools write (`/app/output/rcb_session_log.jsonl`, read only for the self-reported `agent_*` cost fields) was graded `grader_error = 1` and `infra_error = 1` with reward 0, whatever the trajectory; the log is now read tolerantly and can no longer decide anything. A trial whose log is valid is graded as before; the pass rule is unchanged.
