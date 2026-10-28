# roboengineer-go2-weave-i01-standard

| | |
|---|---|
| Benchmark | roboEngineer |
| Mode | standard (`eai-policy/1.1`) |
| Twin | — (none: roboEngineer has no privileged mode, PROTOCOL §2.15) |
| Simulator / robot | `mujoco` / `unitree-go2` |
| Family / instance | `roboengineer/go2-weave` / seed 1 |
| Oracle | `none` — Harbor 0.23.0 oracle agent on aws:rcb-cpu3, 2026-10-18, image efe-private:control-transfer-0.2.0: the reference controller (solution/, three feedback rounds) succeeded on 0/10 held-out cases, mean engineering score 0.00; nop 0. No full oracle: human review (PROTOCOL §4). No positive example yet. |
| Agent time budget | 14400 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:control-transfer-0.2.0` |

## Task
Execute a predeclared body velocity command schedule to pass four alternating poles without contact; no global-position feedback.

## Difficulty
Infer transfer failures from video and noisy sensors across bounded adaptation rounds.

## Reference solution
Harbor 0.23.0 oracle agent on aws:rcb-cpu3, 2026-10-18, image efe-private:control-transfer-0.2.0: the reference controller (solution/, three feedback rounds) succeeded on 0/10 held-out cases, mean engineering score 0.00; nop 0. No full oracle: human review (PROTOCOL §4). No positive example yet.

## Verification
Frozen development and held-out banks, data-only deployment, fresh-process state hashes, separate no-network verifier. All numeric scores and physical outcomes are operator-only until submission closes.

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 2 | 6 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 4 | 6 GiB | 8192 | no GPU | — |
| verifier (as `main`) | 2 | 6 GiB | — | no GPU | — |

- GPU: not needed.
- Measured: Harbor 0.23 oracle on aws:rcb-cpu3 (2026-10-18, anycar-tracking, anycar-compact-lap-monaco-6, anycar-tight-drift-park, neuralfly-general-skydio, go2-weave; docker stats + cgroup memory.peak every 3-6 s): sidecar 1.7-1.9 cores during deployments, its 4-CPU cap during finalization (held-out x2, 4 workers, + video: 79-199 s), peak 0.6-2.13 GiB (go2-weave); the agent's container <= 0.23 GiB. Minimum = the agent's 6 GB cap (its 4 GiB limit is recorded, not enforced, by default) + the sidecar's peak x 1.2 + the monitor's 0.25 GB; CPU = the agent's 2-core cap + the sidecar's ~2 during deployments, the sidecar's 4 after the agent is stopped; verifier phase = the sidecar's collect hook (4 cores, <= 2.13 GiB x 1.2 + the monitor); the verifier itself only reads JSON (~20 s)
- Measured (caps): the reference runs (2026-10-18) peaked at 0.23 GiB in the agent's container; 4 GiB (the author's budget) leaves room for an agent's own nominal training; processes / disk: design defaults

## Details

### Notes
A simulated port of published quadruped performance tasks, not a hardware calibration; MuJoCo contacts differ from other simulators.
