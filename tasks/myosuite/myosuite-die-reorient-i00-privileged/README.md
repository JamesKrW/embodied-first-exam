# myosuite-die-reorient-i00-privileged

| | |
|---|---|
| Benchmark | MyoSuite |
| Mode | privileged |
| Twin | `myosuite-die-reorient-i00-standard` |
| Simulator / robot | `myosuite` / `myohand` |
| Family / instance | `die_reorient` / seed 0 |
| Oracle | `none` — a successful model trial on this frozen instance: Claude Code + Claude Opus 5.5 (run `claude_code-claude_opus_5_5-default`, 2026-09-28), trial `myosuite-die-reorient-i00__EtyJ3X7` (225 rows); today's verifier scores its trajectory 1 in two fresh trials (tasks/myosuite/README.md); human review in PR #65 |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:myosuite-0.1.1` |

## Task
MyoSuite `myoChallengeDieReorientP1-v0` (MyoHand, a musculoskeletal model of a human forearm and hand, 39 muscles): Rotate the die in the hand to the target pose: its position within 0.025 m of the target and the norm of its Euler-angle error below 0.262 rad, for 100 consecutive control steps (1.0 s). Failure (the episode ends): the die more than 0.2 m from the target (dropped).
A die rests in the palm of the upturned hand; a translucent target die shows the goal orientation. The deliverable is one replayable trajectory of actuator commands (`/app/output/trajectory.npz`, `(T, 39)`, one row per 10 ms control step).

## Difficulty
First-pass rating: hard. The action is muscle-level (39 muscles, each only pulling, several crossing more than one joint, with activation delays): the agent has to build its own joint control from muscle excitations. MyoSuite's goal has to hold for 1.0 s, so passing through the goal does not count.

## Reference solution
None. Solvability evidence: a successful model trial on this frozen instance: Claude Code + Claude Opus 5.5 (run `claude_code-claude_opus_5_5-default`, 2026-09-28), trial `myosuite-die-reorient-i00__EtyJ3X7` (225 rows); today's verifier scores its trajectory 1 in two fresh trials (tasks/myosuite/README.md); human review in PR #65.

## Verification
`tests/grade.py` in a separate sandbox without network: two replays of `/app/output/trajectory.npz`, each in a fresh process, from the verifier's own copy of the instance (`tests/instance.npz`; the rebuilt start state is checked against its recorded hash); pass iff both episodes end in success (the goal held 100 control steps) in the same state (`deterministic`). No oracle; human review in PR #65. `nop`: no trajectory, reward 0.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 12 GiB | 8192 | no GPU | 10 GiB |
| verifier | 4 | 8 GiB | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: memory: privileged container median 6.2 GB, verifier 1.4 GB (a per-container memory sampler); CPU only; CPU: an estimate
- Measured (caps): processes and disk: defaults, not measured

## Details
| | |
|---|---|
| Upstream | MyoSuite 2.12.2 (Apache-2.0) on MuJoCo 3.6.0, the models that ship in its wheel |
| Instance | `myoChallengeDieReorientP1-v0` after `seed(0)` and `reset(seed=0)` (`rcb_myosuite.py make-instance`); state hash `7e5922c982779368`, checked on every load |
| Actions | 39 actuator commands per control step (10 ms, 5 physics substeps); success rule `hold` (tasks/myosuite/README.md) |
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
