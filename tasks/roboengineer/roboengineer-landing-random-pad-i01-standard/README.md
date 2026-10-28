# roboengineer-landing-random-pad-i01-standard

| | |
|---|---|
| Benchmark | roboEngineer |
| Mode | standard (`eai-policy/1.0`) |
| Twin | — (none: roboEngineer has no privileged mode, PROTOCOL §2.15) |
| Simulator / robot | `mujoco` / `intel-aero-partial` |
| Family / instance | `roboengineer/landing-random-pad` / seed 1 |
| Oracle | `full` — Harbor 0.23.0 oracle agent on aws:rcb-cpu3, 2026-10-18, image efe-private:quadrotor-landing-0.1.0: six safe held-out landings of six, mean score 82.89; nop 0. |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:quadrotor-landing-0.1.0` |

## Task
**roboEngineer → Drone → Neural Landing.** See the [collection roadmap](tasks/roboengineer/README.md).

Randomized aerial landing with bounded sensor-only policy adaptation and a private held-out bank. Reference runs: the Oracle row; build instructions: tasks/roboengineer/README.md.

## Difficulty
Identify unmodeled aerodynamic forces using noisy sensors and camera samples, while trading landing time against contact safety.

## Reference solution
The scripted learner in `solution/` fits force corrections from permitted IMU/RPM feedback over three rounds. It is not installed in the agent image. It lands all six held-out cases on both instances (the Oracle row).

## Verification
The private MuJoCo service evaluates the sealed policy in six held-out cases, each replayed twice in fresh processes. The no-network verifier reads only collected simulator evidence and emits binary success plus a continuous score. Reference times come from finite full-model optimization, without a claim of global optimality. Ground-truth states, model residuals, seeds, reference searches and grades are never feedback to the agent.

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 2 | 6 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 4 | 6 GiB | 8192 | no GPU | — |
| verifier (as `main`) | 2 | 6 GiB | — | no GPU | — |

- GPU: not needed.
- Measured: Harbor 0.23 oracle on aws:rcb-cpu3 (2026-10-18; docker stats + cgroup memory.peak every 3-6 s): the sidecar holds its 4-CPU cap through the reference search before the agent (~480 s, the agent's container idle) and the finalization (136 s), peak 1.18 GiB; the sidecar 1-2 cores during deployments; the agent's container 0.06 GiB. Minimum = the agent's 6 GB cap (its 4 GiB limit is recorded, not enforced, by default) + the sidecar's peak x 1.2 + the monitor's 0.25 GB; CPU = the sidecar's 4 (never at the same time as the agent's work: the agent waits for its health check, and is stopped before finalization), the agent's 2-core cap + ~2 during deployments; verifier phase = the sidecar's collect hook (4 cores, <= 2.13 GiB x 1.2 + the monitor); the verifier itself only reads JSON (~20 s)
- Measured (caps): the reference runs (2026-10-18) peaked at 0.23 GiB in the agent's container; 4 GiB (the author's budget) leaves room for an agent's own nominal training; processes / disk: design defaults
