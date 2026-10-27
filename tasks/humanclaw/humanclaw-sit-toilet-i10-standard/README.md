# humanclaw-sit-toilet-i10-standard

| | |
|---|---|
| Benchmark | HumanCLAW |
| Mode | standard (`eai-standard/2.1`) |
| Twin | `humanclaw-sit-toilet-i10-privileged` |
| Simulator / robot | `humanclaw` / `smplx-human` |
| Family / instance | `sit-toilet` / seed 0 |
| Oracle | `full` — the privileged oracle's 33 actions streamed as skill rows through the client (open loop: they succeed on L40S) |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:humanclaw-0.1.0` |

## Task
Standard mode (robot as a service, `eai-standard/2.1`) of `humanclaw-sit-toilet-i10-privileged`: the same frozen episode and the same
success rule. HumanCLAW runs in the `sim` sidecar (`environment/docker-compose.yaml`, GPU on the sidecar), which
judges the episode as it runs with HumanCLAW's own metric code, records it and, after the verdict, renders its video
from the poses the live episode recorded; the verifier grades the sidecar's record in a container of its own
(`tests/Dockerfile`, no network).

## Difficulty
See `humanclaw-sit-toilet-i10-privileged`'s README; here the agent has only the head camera, no map, no reset and no score.

## Reference solution
See `solution/`.

## Verification
`success` = live success: the episode the sim service ran ended with a stop and met HumanCLAW's InteractSR
(HumanCLAW's `PaperMetricRecorder.finalize`, judged on the live simulator). Nothing is replayed to grade it. The
scene's start is checked against the frozen instance's recorded state hash when the episode begins. `nop` scores 0.

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 4 | 1 |
| Memory | 8 GB | 1 GB |
| GPU memory | 4 GB | 0 GB |

- GPU: required, any NVIDIA GPU; recommended: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: author estimate, not yet measured: MotionDiT 1.6 GB GPU (RTX 4090, 20 skills), Habitat EGL rendering on top
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), the agent's container: no GPU, 8 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 16 GiB GPU memory, 24 GiB memory. Limits measured: design defaults, not yet measured.
