# metaworldplus-peg-unplug-side-i00-privileged

| | |
|---|---|
| Benchmark | MetaWorld+ |
| Mode | privileged |
| Twin | `metaworldplus-peg-unplug-side-i00-standard` |
| Simulator / robot | `mujoco` / `sawyer` |
| Family / instance | `metaworld/peg-unplug-side` / seed 0 |
| Oracle | `full` |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:metaworld-0.1.0` |

## Task
A Sawyer arm at a table, Meta-World's `peg-unplug-side-v3` (reward v2) on one frozen instance: Pull the peg out of the side-facing socket. The agent writes a
trajectory of Meta-World's native actions with full privileged state and unlimited resets; it is graded by replay on
the state after the last action. Benchmark notes in
`tasks/metaworldplus/README.md`.

## Difficulty
Full privileged state, unlimited resets and self-replay: this mode is the control for the standard twin
(`metaworldplus-peg-unplug-side-i00-standard`). It is reported, but not in the main (standard-mode) table, because it is
saturated under final-state grading (decision on the benchmark proposal, #10). What remains hard here is planning the
contacts (grasp, push, turn) with a mocap-driven hand whose wrist does not rotate, within 500 actions.

## Reference solution
Meta-World's scripted policy for `peg-unplug-side-v3` (`metaworld/policies/`, removed from the image), run on this instance up to
its first success: 106 actions in `solution/oracle.npz`.

## Verification
`tests/grade.py` in a separate sandbox without network: two replays in fresh processes from the verifier's own copy of
the instance; `success = deterministic ∧ success on the final state`. Oracle: two fresh-process replays end in the
live episode's state (hash `dcb265b2b1d7…`) and succeed. `nop`: no trajectory, reward 0; the instance is
not solved at the start.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 10 GiB | 8192 | no GPU | 10 GiB |
| verifier | no cap | no cap | no cap | no GPU | — |

- GPU: not needed.
- Measured: memory: privileged container p90 2.7 GB, sim 0.7 GB (a per-container memory sampler); CPU only; CPU: an estimate

## Details
| | |
|---|---|
| source | [Meta-World](https://github.com/Farama-Foundation/Metaworld) @ 59fc34d (MIT) on MuJoCo 3.3.0; image `efe:metaworld-0.1.0` |
| instance | Meta-World's reset randomization with seed 0; MuJoCo state, model arrays, env caches and RNG frozen in `instance.npz` (SHA-256 `eb490ec582cb…`, checked on load) |
| deliverable | `(T, 4)` native actions `[dx, dy, dz, grip]`, 1 ≤ T ≤ 500 (Meta-World's `max_path_length`) |
| success | Meta-World's own check (`evaluate_state`) on the state after the last action. The peg's end is within 7 cm of the target point. |
| actions | no `joint_position`: Meta-World's Sawyer has no arm actuators (the hand is a mocap body, the arm follows it) |

### Change Log
- 2026-10-10 (review 0067): the instruction states this environment's success condition in words (it already pointed to `s.check()` and Meta-World's source, which this mode can read) and the limits on a hand-written `trajectory.npz`. Nothing in the verifier changed.
