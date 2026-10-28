# roboengineer-neuralfly-vector-wind-i00-standard

| | |
|---|---|
| Benchmark | roboEngineer |
| Mode | standard (`eai-policy/1.1`) |
| Twin | — (none: roboEngineer has no privileged mode, PROTOCOL §2) |
| Simulator / robot | `mujoco` / `neural-fly-partial` |
| Family / instance | `roboengineer/neuralfly-vector-wind` / seed 0 |
| Oracle | `full` — Harbor 0.23.0 oracle agent on a cloud CPU host, 2026-10-18, image efe-private:control-transfer-0.2.0: the reference controller (solution/, three feedback rounds) succeeded on 10/10 held-out cases, mean engineering score 90.01; nop 0. |
| Agent time budget | 14400 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe-private:control-transfer-0.2.0` |

## Task
Track six timed laps of a randomized figure eight: vertical, horizontal or inclined plane; yaw in [-pi,pi]; both traversal directions; half extents 0.9-1.5 m and 0.45-0.85 m; angular rate 0.8-1.1 rad/s; randomized phase, center and initial offset. Follow the provided world-frame position, velocity and acceleration commands.

## Difficulty
Infer transfer failures from video and noisy sensors across bounded adaptation rounds.

## Reference solution
Harbor 0.23.0 oracle agent on a cloud CPU host, 2026-10-18, image efe-private:control-transfer-0.2.0: the reference controller (solution/, three feedback rounds) succeeded on 10/10 held-out cases, mean engineering score 90.01; nop 0.

## Verification
Frozen development and held-out banks, data-only deployment, fresh-process state hashes, separate no-network verifier. All numeric scores and physical outcomes are operator-only until submission closes.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 2 | 6 GiB | 8192 | no GPU | 20 GiB |
| `sim` | 4 | 6 GiB | 8192 | no GPU | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: Harbor oracle on a cloud CPU host (anycar-tracking, anycar-compact-lap-monaco-6, anycar-tight-drift-park, neuralfly-general-skydio, go2-weave; docker stats and cgroup memory.peak every 3-6 s): the sidecar 1.7-1.9 cores during deployments, its 4-CPU cap during finalization (the held-out cases twice on 4 workers, then the video: 79-199 s), peak 0.6-2.13 GiB (go2-weave); the agent's container at most 0.23 GiB; the verifier phase is the sidecar's collect hook (4 cores, at most 2.13 GiB); the verifier itself only reads JSON (~20 s)
- Measured (caps): processes and disk: defaults, not measured

## Details

### Notes
Flight-data quadratic force surrogate and approximate rigid-body/actuator dynamics; not a learned aerodynamics model or flight-stack simulation. Fixed 50-degree command tilt envelope; 2.2 thrust-to-weight limit. Synthetic world-frame vector wind with isotropic quadratic drag increment (0.045 kg/m); not calibrated directional aerodynamics. Per-axis residual force cap remains 12 N.
