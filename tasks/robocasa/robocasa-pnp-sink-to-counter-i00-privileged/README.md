# robocasa-pnp-sink-to-counter-i00-privileged

| | |
|---|---|
| Benchmark | RoboCasa |
| Mode | privileged |
| Twin | `robocasa-pnp-sink-to-counter-i00-standard` |
| Simulator / robot | `robosuite` / `franka-panda-omron` |
| Family / instance | `robocasa/PnPSinkToCounter` |
| Oracle | `full` |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:robocasa-0.1.0` |

## Task
RoboCasa's `PnPSinkToCounter` kitchen task with a Franka Panda arm on an Omron mobile base (with a torso lift): Pick the tomato from the sink and place it on the plate located on the counter. Produce one replayable trajectory of the robot's native controller commands on one frozen instance; graded on the state after the last action. Benchmark notes in `tasks/robocasa/README.md`.

## Difficulty
Full privileged state, unlimited resets and self-replay; a mobile manipulator in a cluttered kitchen, long-horizon contact-rich manipulation with a 20 Hz controller. The standard twin removes the privileged state.

## Reference solution
The demonstration's own actions (original: 273 actions), executed from the frozen scene by the robot's own controller: 273 actions in `solution/oracle.npz`. The demonstration is a reference solution: it is not in the image and the agent cannot reach it.

## Verification
`tests/grade.py` in a separate sandbox without network: two replays in fresh processes from the verifier's own copy of the instance; `success = deterministic ∧ success on the final state ∧ not solved at the start`. Reference: two fresh-process replays end in the live run's state (hash `67abe2c02fcc…`) and succeed. `nop`: no trajectory, reward 0; the instance is not solved at the start.

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 20 GiB | 8192 | no GPU | 10 GiB |
| verifier | 4 | 16 GiB | — | no GPU | — |

- GPU: not needed.
- Measured: memory: robosuite sim p90 9.7 GB, verifier p90 4.0 GB (RoboCasa and RoboCasa-GR1 together), mem_guard (docker memory per container, an RTX 4090 workstation, 10-02..10); CPU-only (OSMesa cameras); CPU author estimate
- Measured (caps): memory: the [environment] cap 16 GiB, which every past trial ran under (docker peak 16.0 GiB over 3 samples, mem_guard (docker memory incl. page cache, top-4 containers every 10 min, an RTX 4090 workstation, 10-02..10)); processes 512 and disk 10 GiB: design defaults, not yet measured

## Details
| | |
|---|---|
| Source | [RoboCasa](https://github.com/robocasa/robocasa) @ 756598a (v0.2; MIT), robosuite 1.5.1, MuJoCo 3.2.6; [environment source](https://github.com/robocasa/robocasa/blob/756598a5be52e052339bb2d957426e39015c2afb/robocasa/environments/kitchen/single_stage/kitchen_pnp.py#L369) |
| Instance | the initial state of official demonstration episode 0 (`demo_1`, v0.1/single_stage/kitchen_pnp/PnPSinkToCounter/2024-04-26_2); MuJoCo state, model arrays, task state and RNG frozen in `instance.npz` (SHA-256 `337584e93136…`, checked on load) |
| Deliverable | `(T, 12)` native controller commands, 1 ≤ T ≤ 3000, 20 Hz |
| Success | the environment's own check (`_check_success`) on the state after the last action |
