# roboengineer-go2-yaw-jump-i00-standard

| | |
|---|---|
| Benchmark | roboEngineer |
| Mode | standard (`eai-policy/1.1`) |
| Twin | — (none: roboEngineer has no privileged mode, PROTOCOL §2) |
| Simulator / robot | `mujoco` / `unitree-go2` |
| Family / instance | `roboengineer/go2-yaw-jump` / seed 0 |
| Oracle | `none` — Harbor 0.23.0 oracle agent on a cloud CPU host, 2026-10-18, image efe-private:control-transfer-0.2.0: the reference controller (solution/, three feedback rounds) succeeded on 0/10 held-out cases, mean engineering score 9.80; nop 0. No full oracle: human review (PROTOCOL §4). No positive example yet. |
| Agent time budget | 14400 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe-private:control-transfer-0.2.0` |

## Task
Jump through a commanded 110–150 degree heading change with verified airborne rotation and stable touchdown; ground spinning is rejected.

## Difficulty
Infer transfer failures from video and noisy sensors across bounded adaptation rounds.

## Reference solution
Harbor 0.23.0 oracle agent on a cloud CPU host, 2026-10-18, image efe-private:control-transfer-0.2.0: the reference controller (solution/, three feedback rounds) succeeded on 0/10 held-out cases, mean engineering score 9.80; nop 0. No full oracle: human review (PROTOCOL §4). No positive example yet.

## Verification
Frozen development and held-out banks, data-only deployment, fresh-process state hashes, separate no-network verifier. All numeric scores and physical outcomes are operator-only until submission closes.

## Resources
Caps per container (PROTOCOL §6).

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
A simulated port of published quadruped performance tasks, not a hardware calibration; MuJoCo contacts differ from other simulators.
