# humanclaw-nav-tv-i13-privileged

| | |
|---|---|
| Benchmark | HumanCLAW |
| Mode | privileged |
| Twin | `humanclaw-nav-tv-i13-standard` |
| Simulator / robot | `humanclaw` / `smplx-human` |
| Family / instance | `nav-tv` / seed 0 |
| Oracle | `full` — navmesh oracle (privileged tools, closed loop, on L40S): 20 actions; two fresh-process replays on L40S succeed with equal final hashes (f1431277ba8dfe66) from the start hash cb2e24c26a002ada; one RTX 4090 replay also succeeds; image humanclaw-0.1.0, 2026-10-27 |
| Agent time budget | 3600 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe-private:humanclaw-0.1.0` |

## Task
HumanCLAW (HumanClawBench) Find/Nav, one frozen episode of its full
validation set (TV), privileged mode: the agent has the simulator, the episode's privileged state and
free resets, and delivers one trajectory of at most 100 motion skills. Twin: `humanclaw-nav-tv-i13-standard`.

## Difficulty
Selected by Codex + GPT-6 Astra (max effort) outcomes in HumanCLAW's own full-validation run (tasks/humanclaw/README.md).

## Reference solution
See `solution/`.

## Verification
`tests/grade.py` replays `/app/output/trajectory.npz` twice, each in a fresh process, from `tests/instance.json`;
success = both replays succeed under HumanCLAW's NavSR@20cm and end in the same state hash, and the rebuilt start
hashes as recorded. `nop` (no file) scores 0.

## Compute resources
Minimum allocation for one trial (docs/protocol/README.md §2.9; `[metadata.resources]` in `task.toml`). The agent phase counts every container of the trial at once (the agent's, and in standard modes the simulator sidecar); the verifier phase runs after it.

| | Agent phase | Verifier |
|---|---|---|
| CPU cores | 4 | 4 |
| Memory | 8 GB | 8 GB |
| GPU memory | 4 GB | 4 GB |

- GPU: required, any NVIDIA GPU; recommended: L40S 48GB / L4 24GB / RTX 4090 24GB.
- Measured: author estimate, not yet measured: MotionDiT 1.6 GB GPU (RTX 4090, 20 skills), Habitat EGL rendering on top
- Limits in the agent phase (docs/protocol/README.md §2.10; `[metadata.resources.limits]`, shown to the agent in `instruction.md`; always measured and reported, and exceeding one fails the trial only when the run sets `RCB_LIMITS_ENFORCE=1`), every container of the trial: 16 GiB GPU memory, 24 GiB memory, 512 processes, 20 GiB of new files under `/app`, `/tmp`, `/root`; 60 s within 120 s. Expected peak of the whole trial: 16 GiB GPU memory, 24 GiB memory. Limits measured: design defaults, not yet measured.
