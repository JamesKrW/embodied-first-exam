# roboengineer-g1-terrain-walk-i00-standard

| | |
|---|---|
| Benchmark | roboEngineer |
| Mode | standard (`eai-policy/1.1`) |
| Twin | — (none: roboEngineer has no privileged mode, PROTOCOL §2) |
| Simulator / robot | `mujoco` / `unitree-g1` |
| Family / instance | `roboengineer/g1-terrain-walk` / seed 0 |
| Oracle | `partial` — Harbor 0.23.0 oracle agent on a cloud L40S host, 2026-10-18, images efe-private:control-transfer-0.2.0-train / 0.2.0, an earlier planner of the reference with a fixed first segment (2500 + 1733 + 1155 PPO iterations x 4096 environments from scratch; 12638 s of the 14400 s): the reference (trains the public curriculum from scratch on the agent's GPU, three deployments, the last sealed) succeeded on 1/12 held-out cases, mean engineering score 43.14; nop 0. No full oracle: human review (PROTOCOL §4). |
| Agent time budget | 14400 s; tokens: protocol default (docs/protocol/running.md "Official runs") |
| Base image | `ghcr.io/mll-lab-nu/efe-private:control-transfer-0.2.0` |

## Task
Train a blind 29-DoF Unitree G1 policy in mjlab/MuJoCo Warp and transfer to CPU MuJoCo over flat, sloped, rough and stepped terrain. Stand 2 s, then follow a public forward velocity command of 0.5–0.65 m/s for 18 s.

## Difficulty
Infer transfer failures from video and noisy sensors across bounded adaptation rounds.

## Reference solution
Harbor 0.23.0 oracle agent on a cloud L40S host, 2026-10-18, images efe-private:control-transfer-0.2.0-train / 0.2.0, an earlier planner of the reference with a fixed first segment (2500 + 1733 + 1155 PPO iterations x 4096 environments from scratch; 12638 s of the 14400 s): the reference (trains the public curriculum from scratch on the agent's GPU, three deployments, the last sealed) succeeded on 1/12 held-out cases, mean engineering score 43.14; nop 0. No full oracle: human review (PROTOCOL §4).

## Verification
Frozen development and held-out banks, data-only deployment, fresh-process state hashes, separate no-network verifier. All numeric scores and physical outcomes are operator-only until submission closes.

## Resources
Caps per container (PROTOCOL §6).

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 4 | 12 GiB | 8192 | 20 GiB | 20 GiB |
| `sim` | 4 | 6 GiB | 8192 | no GPU | — |
| verifier | no cap | no cap | no cap | no GPU | no cap |

- GPU: any NVIDIA GPU, held by `main`.
- Recommended GPU: L40S 48GB (all measurements; a 24 GB card fits the memory but trains slower, so fewer iterations fit the cutoff).
- Measured: Harbor oracle (the reference: 4096 environments, 12638 s of training and deployments) and smoke trials on two L40S hosts (docker stats, cgroup memory.peak and nvidia-smi per container every 3-6 s): the agent's container 16.1 GiB (16492 MiB) of GPU memory, 5.3 GiB of memory (5.6 GiB cgroup peak, growing over the 3.5 h run), 1.0-1.6 cores (one Python thread drives the GPU); the public trainer's default 256 environments 1.8 GB, its cap of 1024 4.7 GB; the sidecar 2.2-2.6 cores during deployments (OSMesa), its 4-CPU cap during finalization (12 held-out cases twice and the video: 419 s), peak 2.06 GiB; at most 0.86 GiB of new files and 6 processes in the agent's container
- Measured (caps): GPU memory: the reference (4096 environments) peaked at 16.1 GiB -> x 1.15, rounded up with headroom: 20 GiB (enough for about 4900 environments); disk: at most 0.86 GiB of new files in the reference

## Details

### Notes
Simulation transfer only, not hardware. Fixed 98→512→256→128→29 ELU actor and public PD wrapper. Coding-agent feedback is noisy joint angles, public commands and images.
