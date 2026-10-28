# roboengineer-anycar-tracking-i01-standard

| | |
|---|---|
| Benchmark | roboEngineer |
| Mode | standard (`eai-policy/1.1`) |
| Twin | — (none: roboEngineer has no privileged mode, PROTOCOL §2) |
| Simulator / robot | `mujoco` / `anycar` |
| Family / instance | `roboengineer/anycar-tracking` / seed 1 |
| Oracle | `full` — Harbor 0.23.0 oracle agent on a cloud CPU host, 2026-10-18, image efe-private:control-transfer-0.2.0: the reference controller (solution/, three feedback rounds) succeeded on 10/10 held-out cases, mean engineering score 72.87; nop 0. |
| Agent time budget | 14400 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe-private:control-transfer-0.2.0` |

## Task
Track one prescribed-time lap of a randomized ellipse across rigid traction patches; avoid barriers, rollover and offtrack excursions.

## Difficulty
Infer transfer failures from video and noisy sensors across bounded adaptation rounds.

## Reference solution
Harbor 0.23.0 oracle agent on a cloud CPU host, 2026-10-18, image efe-private:control-transfer-0.2.0: the reference controller (solution/, three feedback rounds) succeeded on 10/10 held-out cases, mean engineering score 72.87; nop 0.

## Verification
Frozen development and held-out banks, data-only deployment, fresh-process state hashes, separate no-network verifier. All numeric scores and physical outcomes are operator-only until submission closes.

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 2 | 6 GiB | 1024 | no GPU | 10 GiB |
| `sim` | 4 | 6 GiB | 1024 | no GPU | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: not needed.
- Measured: Harbor oracle on a cloud CPU host (anycar-tracking, anycar-compact-lap-monaco-6, anycar-tight-drift-park, neuralfly-general-skydio, go2-weave; docker stats and cgroup memory.peak every 3-6 s): the sidecar 1.7-1.9 cores during deployments, its 4-CPU cap during finalization (the held-out cases twice on 4 workers, then the video: 79-199 s), peak 0.6-2.13 GiB (go2-weave); the agent's container at most 0.23 GiB; the verifier phase is the sidecar's collect hook (4 cores, at most 2.13 GiB); the verifier itself only reads JSON (~20 s)
- Measured (caps): per-container peaks in 110 past trials: the agent's container memory 4.5 GiB, 162 threads, CPU at its cap (2), disk 1.3 GiB; the sim service memory 1.0 GiB (2.1 GiB in an earlier measurement), 30 threads, CPU 1.9 cores

## Details

### Notes
Procedural chassis with retained contact-wheel mechanism. Rigid Coulomb friction is not deformable sand. No learned vehicle model is evaluated.
