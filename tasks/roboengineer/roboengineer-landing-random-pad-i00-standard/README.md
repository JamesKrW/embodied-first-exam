# roboengineer-landing-random-pad-i00-standard

| | |
|---|---|
| Benchmark | roboEngineer |
| Mode | standard (`eai-policy/1.0`) |
| Twin | — (none: roboEngineer has no privileged mode, PROTOCOL §2) |
| Simulator / robot | `mujoco` / `intel-aero-partial` |
| Family / instance | `roboengineer/landing-random-pad` / seed 0 |
| Oracle | `full` — Harbor 0.23.0 oracle agent on a cloud CPU host, 2026-10-18, image efe-private:quadrotor-landing-0.1.0: six safe held-out landings of six, mean score 84.11; nop 0. |
| Agent time budget | 3600 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
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
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 2 | 4 GiB | 1024 | no GPU | 10 GiB |
| `sim` | 4 | 5 GiB | 1024 | no GPU | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: Harbor oracle on a cloud CPU host (docker stats and cgroup memory.peak every 3-6 s): the sidecar holds its 4-CPU cap through the reference search before the agent starts (~480 s, the agent's container idle) and the finalization (136 s), peak 1.18 GiB; the sidecar 1-2 cores during deployments; the agent's container 0.06 GiB; the agent waits for the sidecar's health check and is stopped before finalization, so the sidecar's 4 cores never coincide with the agent's work; the verifier itself only reads JSON (~20 s)
