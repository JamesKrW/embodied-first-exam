# metaworldplus-bin-picking-i00-privileged

| | |
|---|---|
| Benchmark | MetaWorld+ |
| Mode | privileged |
| Twin | `metaworldplus-bin-picking-i00-standard` |
| Simulator / robot | `mujoco` / `sawyer` |
| Family / instance | `metaworld/bin-picking` / seed 0 |
| Oracle | `full` |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:metaworld-0.1.0` |

## Task
A Sawyer arm at a table, Meta-World's `bin-picking-v3` (reward v2) on one frozen instance: Pick up the object from its bin and move it to the target bin. The agent writes a
trajectory of Meta-World's native actions with full privileged state and unlimited resets; it is graded by replay on
the state after the last action. Benchmark notes in
`tasks/metaworldplus/README.md`.

## Difficulty
Full privileged state, unlimited resets and self-replay: this mode is the control for the standard twin
(`metaworldplus-bin-picking-i00-standard`). It is reported, but not in the main (standard-mode) table, because it is
saturated under final-state grading (decision on the benchmark proposal, #10). What remains hard here is planning the
contacts (grasp, push, turn) with a mocap-driven hand whose wrist does not rotate, within 500 actions.

## Reference solution
Meta-World's scripted policy for `bin-picking-v3` (`metaworld/policies/`, removed from the image), run on this instance up to
its first success: 109 actions in `solution/oracle.npz`.

## Verification
`tests/grade.py` in a separate sandbox without network: two replays in fresh processes from the verifier's own copy of
the instance; `success = deterministic ∧ success on the final state`. Oracle: two fresh-process replays end in the
live episode's state (hash `a94e22d5f927…`) and succeed. `nop`: no trajectory, reward 0; the instance is
not solved at the start.

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 10 GiB | 8192 | no GPU | 10 GiB |
| verifier (as `main`) | 4 | 10 GiB | — | no GPU | — |

- GPU: not needed.
- Measured: memory: privileged container p90 2.7 GB, sim 0.7 GB, mem_guard (docker memory per container, an RTX 4090 workstation, 10-02..10); CPU-only; CPU author estimate
- Measured (caps): memory: the [environment] cap 8 GiB, which every past trial ran under (docker peak 3.2 GiB over 20 samples, mem_guard (docker memory incl. page cache, top-4 containers every 10 min, an RTX 4090 workstation, 10-02..10)); processes 512 and disk 10 GiB: design defaults, not yet measured

## Details
| | |
|---|---|
| source | [Meta-World](https://github.com/Farama-Foundation/Metaworld) @ 59fc34d (MIT) on MuJoCo 3.3.0; image `efe:metaworld-0.1.0` |
| instance | Meta-World's reset randomization with seed 0; MuJoCo state, model arrays, env caches and RNG frozen in `instance.npz` (SHA-256 `c2684e1a7c44…`, checked on load) |
| deliverable | `(T, 4)` native actions `[dx, dy, dz, grip]`, 1 ≤ T ≤ 500 (Meta-World's `max_path_length`) |
| success | Meta-World's own check (`evaluate_state`) on the state after the last action. The object's centre is within 5 cm of the target point. |
| actions | no `joint_position`: Meta-World's Sawyer has no arm actuators (the hand is a mocap body, the arm follows it) |

### Change Log
- 2026-10-10 (review 0067): the instruction states this environment's success condition in words (it already pointed to `s.check()` and Meta-World's source, which this mode can read) and the limits on a hand-written `trajectory.npz`. Nothing in the verifier changed.
