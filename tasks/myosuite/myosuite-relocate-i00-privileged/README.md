# myosuite-relocate-i00-privileged

| | |
|---|---|
| Benchmark | MyoSuite |
| Mode | privileged |
| Twin | `myosuite-relocate-i00-standard` |
| Simulator / robot | `myosuite` / `myoarm` |
| Family / instance | `relocate` / seed 0 |
| Oracle | `none` — a successful model trial on this frozen instance: Codex + GPT-6 Luna (run `codex-gpt6_luna-high`, via an OpenRouter adapter before protocol v1.0, 2026-09-29), trial `myosuite-relocate-i00__PjqTvZ2` (231 rows); today's verifier scores its trajectory 1 in two fresh trials (tasks/myosuite/README.md); human review in PR #65 |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:myosuite-0.1.1` |

## Task
MyoSuite `myoChallengeRelocateP1-v0` (MyoArm, a musculoskeletal model of a human shoulder, arm and hand, 63 muscles): Pick up the object from the table and put it in the target receptacle: the object's position within 0.1 m of the target (rotation is ignored) for 100 consecutive control steps (1.0 s). Failure (the episode ends): the palm more than 0.5 m from the object.
An object stands on the table in front of the arm; a receptacle marks the target position. The deliverable is one replayable trajectory of actuator commands (`/app/output/trajectory.npz`, `(T, 63)`, one row per 10 ms control step).

## Difficulty
First-pass rating: hard. The action is muscle-level (63 muscles, each only pulling, several crossing more than one joint, with activation delays): the agent has to build its own joint control from muscle excitations. MyoSuite's goal has to hold for 1.0 s, so passing through the goal does not count.

## Reference solution
None. Solvability evidence: a successful model trial on this frozen instance: Codex + GPT-6 Luna (run `codex-gpt6_luna-high`, via an OpenRouter adapter before protocol v1.0, 2026-09-29), trial `myosuite-relocate-i00__PjqTvZ2` (231 rows); today's verifier scores its trajectory 1 in two fresh trials (tasks/myosuite/README.md); human review in PR #65.

## Verification
`tests/grade.py` in a separate sandbox without network: two replays of `/app/output/trajectory.npz`, each in a fresh process, from the verifier's own copy of the instance (`tests/instance.npz`; the rebuilt start state is checked against its recorded hash); pass iff both episodes end in success (the goal held 100 control steps) in the same state (`deterministic`). No oracle; human review in PR #65. `nop`: no trajectory, reward 0.

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 12 GiB | 8192 | no GPU | 10 GiB |
| verifier | 4 | 8 GiB | — | no GPU | — |

- GPU: not needed.
- Measured: memory: privileged container median 6.2 GB, sim p90 1.4 GB, verifier 1.4 GB, mem_guard (docker memory per container, an RTX 4090 workstation, 10-02..10); CPU-only; CPU author estimate
- Measured (caps): memory: the [environment] cap 8 GiB, which every past trial ran under (docker peak 8.0 GiB over 18 samples, mem_guard (docker memory incl. page cache, top-4 containers every 10 min, an RTX 4090 workstation, 10-02..10)); processes 512 and disk 10 GiB: design defaults, not yet measured

## Details
| | |
|---|---|
| Upstream | MyoSuite 2.12.2 (Apache-2.0) on MuJoCo 3.6.0, the models that ship in its wheel |
| Instance | `myoChallengeRelocateP1-v0` after `seed(0)` and `reset(seed=0)` (`rcb_myosuite.py make-instance`); state hash `2edad198591899f9`, checked on every load |
| Actions | 63 actuator commands per control step (10 ms, 5 physics substeps); success rule `hold` (tasks/myosuite/README.md) |
| Resources | 4 CPUs, no GPU (CPU physics, OSMesa rendering) |

### Provenance
The frozen instance (`environment/instance.npz`) and its scene facts were made at generation time; see `tasks/myosuite/README.md`.

### Change Log
- 2026-10-10 (review round 3): `tests/grade.py` only. A byte that is not valid UTF-8 in the session log
  the agent's tools write (`/app/output/rcb_session_log.jsonl`, read only for the self-reported `agent_*` cost fields)
  stopped the grader before it wrote a reward, so the trial had no verdict; the log is now read tolerantly and can no
  longer decide anything. A trial whose log is valid is graded as before; the pass rule is unchanged.
- 2026-10-10 (review round 3): `tests/grade.py`, whose failure a 0 is. The width of the rows is checked
  against the instance's actuators before any replay (a file that is no trajectory for this instance is
  `bad_trajectory`, as before). After that check a replay process that gives no result (out of time, killed for
  memory, not started) is `replay_failed` with `grader_error = 1` and `infra_error = 1`; it was booked as the agent's
  `bad_trajectory`. Every `reward.json` now carries `infra_error` (1 next to every `grader_error`, so also when the
  verifier's host does not rebuild the frozen instance). The pass rule is unchanged; a trial graded `bad_trajectory`
  before this date whose rows had the right shape was an infrastructure failure.
