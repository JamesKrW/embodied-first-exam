# kinder-constrained-cupboard-i00-privileged

| | |
|---|---|
| Benchmark | KinDER |
| Mode | privileged |
| Twin | `kinder-constrained-cupboard-i00-standard` |
| Simulator / robot | `kinder` / `tidybot-plus-plus` |
| Family / instance | `constrained-cupboard` / seed 0 |
| Oracle | `none` — GPT-6 Luna's privileged-mode trajectory, run `codex-gpt6_luna-medium` on the results site, which today's privileged verifier scores 1; it moves the base diagonally by up to 0.14 m a step, beyond standard mode's 0.1 m disc, so it is no standard-mode example, and none is known yet; human review in PR #8 |
| Agent time budget | 1800 s; tokens: protocol default (PROTOCOL §5) |
| Base image | `ghcr.io/mll-lab-nu/efe:kinder-0.1.6` |

## Task
KinDER `kinder/ConstrainedCupboard3D-o6-v0` (Dynamic3D, TidyBot++), seed 0, privileged mode: the agent has the simulator and its state and
hands in one trajectory, replayed open-loop in a fresh process. Its standard-mode twin is `kinder-constrained-cupboard-i00-standard`.

Task: put the six brown rods lying on the floor (30 cm long) into the six 40 cm-tall compartments of the row of eleven narrow cupboards standing side by side, one rod each, as the state names pair them: rod `cuboid_i` into cupboard `cupboard_i` for i = 0 to 5, whose tall compartment is the upper one in `cupboard_0` to `cupboard_2` and the lower one in `cupboard_3` to `cupboard_5`. The task succeeds the moment every rod's centre is inside its compartment: above that compartment's shelf, below the shelf over it, and within the cupboard's 10 cm width and 30 cm depth.

## Difficulty
Geometric: six identical rods, each into the one narrow compartment it is owed (standard mode names them by distance and by place in the row).

## Reference solution
None. Solvability rests on the positive example in `oracle_evidence` (task.toml), checked by a human reviewer (docs/protocol/README.md §2.7).

## Verification
Two open-loop replays of `/app/output/trajectory.npz` from `tests/instance.json` in a fresh process, in a container of
their own (`tests/Dockerfile`, no network): pass iff both reach the goal within 1000 control steps and end in
the same state. No oracle; human review in PR #8. `nop` scores 0.

Benchmark notes: `tasks/kinder/README.md`.

## Resources
Each number is an upper bound for one container; docs/protocol/README.md §6 says where each is set and what enforces it.

| Container | CPUs | Memory | Processes | GPU memory | Disk |
|---|---|---|---|---|---|
| `main` (the agent's) | 16 | 20 GiB | 8192 | no GPU | 10 GiB |
| verifier | 4 | 8 GiB | no cap | no GPU | — |

- GPU: not needed.
- Measured: GPU memory 1.2-1.3 GB (the oracle 1.2 GB on an RTX 4090; per-trial peaks of past trials); memory: privileged container p90 2.8 GB, sim 1.1 GB (a per-container memory sampler); CPU: an estimate

## Details

### Change Log
- 2026-10-16: agent time limit 3600 s -> 1800 s (`[agent] timeout_sec`, both twins; author decision, tasks/kinder/README.md). The scene, instance, pass rule and verifier logic are unchanged (`tests/rcb_limits.json` carries the new cutoff), so the task keeps its name; runs recorded before this date are reported separately.
