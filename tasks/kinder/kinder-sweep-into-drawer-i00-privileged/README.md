# kinder-sweep-into-drawer-i00-privileged

| | |
|---|---|
| Benchmark | KinDER |
| Mode | privileged |
| Twin | `kinder-sweep-into-drawer-i00-standard` |
| Simulator / robot | `kinder` / `tidybot-plus-plus` |
| Family / instance | `sweep-into-drawer` / seed 0 |
| Oracle | `none` — GPT-6 Luna's privileged-mode trajectory of 2026-10-04, run `codex-gpt6_luna-medium` on the results site, which today's privileged verifier scores 1 (it moves the base diagonally by up to 0.13 m a step, beyond standard mode's 0.1 m disc, so it is no standard-mode example); in standard mode, the graded episode of run `codex-gpt6_1_sol-high` of 2026-10-03 (960 control steps, no reset), which the standard service scored 1 (live, fresh replay, deterministic); human review in PR #8 |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:kinder-0.1.6` |

## Task
KinDER `kinder/SweepIntoDrawer3D-o5-v0` (Dynamic3D, TidyBot++), seed 0, privileged mode: the agent has the simulator and its state and
hands in one trajectory, replayed open-loop in a fresh process. Its standard-mode twin is `kinder-sweep-into-drawer-i00-standard`.

Task: get the five small dark-red cubes on top of the kitchen island into the middle drawer of the island's upper row of drawers, which starts closed; a wiper lies next to the cubes. The task succeeds the moment all five cubes' centres are inside that drawer, each at least 2.5 cm from the drawer's two side walls.

## Difficulty
Nonprehensile, tool use, dynamics: open a closed drawer, then sweep five small cubes off the counter into it with a wiper. KinDER's paper reports at most 0.14 success for its best method on this variant and 0.00 for sampling MPC on the true simulator.

## Reference solution
None. Solvability rests on the positive example in `oracle_evidence` (task.toml), checked by a human reviewer (docs/protocol/README.md §2.7).

## Verification
Two open-loop replays of `/app/output/trajectory.npz` from `tests/instance.json` in a fresh process, in a container of
their own (`tests/Dockerfile`, no network): pass iff both reach the goal within 1000 control steps and end in
the same state. No oracle; human review in PR #8. `nop` scores 0.

Benchmark notes: `tasks/kinder/README.md`.

## Resources
Caps per container, each an upper bound for that one container (docs/protocol/README.md §6): `main`'s CPUs, memory and disk from `[environment]` in `task.toml` (`cpus`, `memory_mb`, `storage_mb`), everything else from `environment/docker-compose.yaml` (`cpus`, `mem_limit`, `pids_limit`, and `x-gpu-memory-gb` on the service that holds the GPU). A machine running one trial needs, in each phase, the sum over the containers running then.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 16 | 20 GiB | 8192 | no GPU | 10 GiB |
| verifier | 4 | 8 GiB | — | no GPU | — |

- GPU: not needed.
- Measured: GPU 1.2-1.3 GB (oracle 1.2 GB on RTX 4090), evaluation-pool GPU profile (per-trial GPU peaks, 10-03..10); memory: privileged container p90 2.8 GB, sim 1.1 GB, mem_guard (docker memory per container, an RTX 4090 workstation, 10-02..10); CPU author estimate
- Measured (caps): memory: the [environment] cap 16 GiB, which every past trial ran under (docker peak 3.7 GiB over 6 samples, mem_guard (docker memory incl. page cache, top-4 containers every 10 min, an RTX 4090 workstation, 10-02..10)); processes 512 and disk 10 GiB: design defaults, not yet measured

## Details

### Change Log
- 2026-10-16: agent time limit 3600 s -> 1800 s (`[agent] timeout_sec`, both twins; author decision, tasks/kinder/README.md). The scene, instance, pass rule and verifier logic are unchanged (`tests/rcb_limits.json` carries the new cutoff), so the task keeps its name; runs recorded before this date are reported separately.
