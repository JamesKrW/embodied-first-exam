## Kind
<!-- Pick one. A pull request that adds tasks and changes shared files will be split. -->
- [ ] New benchmark (proposal: #___)
- [ ] Tasks for an existing benchmark
- [ ] Task fix
- [ ] Shared files / protocol (protocol-change issue: #___)

## What and why
<!-- Written by you. For a fix: which agent behaviour it closes, with before/after runs. -->

## Contract
- Mode(s): privileged / standard / standard-realtime — protocol (standard: `eai-standard/2.1`/`2.2`/`2.4`; standard-realtime: `eai-standard/2.3`): `eai-standard/2.__`
- Oracle: full / partial / none — evidence: ___
- Base image(s): `efe:<sim>-<ver>` (published) / `efe-private:<sim>-<ver>` (BUILD_ONLY)
- Agent time cutoff: ___ s per mode, `[agent] timeout_sec` in each task's task.toml; default 3600, reason if other; at most 14400; the same for a task's privileged and standard versions (PROTOCOL §2.12). A change of an existing task's cutoff: old -> new, decided by ___
- Resources (caps per container, PROTOCOL §6, GiB): main ___ CPU / ___ memory / ___ processes / ___ disk / ___ GPU memory; each other service ___ CPU / ___ memory / ___ processes / ___ GPU memory; GPU types (RTX-class only) ___; measured from ___

## Checklist
- [ ] `bash ci_checks/run_all.sh <tasks>` passes
- [ ] Tasks only: no task-generation code (generators, oracle searches, instance freezers); shared code in the image or `embodied_first_exam/benchmarks/<benchmark>/` (PROTOCOL §1)
- [ ] Oracle and `nop` on a sample (≥ 2 tasks per mode), table below
- [ ] New simulator or family: two fresh replays give the same state hash
- [ ] Oracle `partial`/`none`: per-family table of positive examples (PROTOCOL §2.7)
- [ ] Runs with Harbor's native agents (no custom runner, gateway or adapter)
- [ ] Names `<base>-privileged` / `<base>-standard` / `<base>-standard-realtime`; README in the standard layout; no `results/`
- [ ] A new benchmark: its label is in `scripts/task_conventions.py` `BENCHMARKS` (shared-files pull request), with a prefix of its own
- [ ] The cutoff is each task's `[agent] timeout_sec`; `instruction.md` ends with the `## Time limit` section `normalize` renders from it and states the limit nowhere else (`ci_checks/check-cutoffs.py`; PROTOCOL §2.12)
- [ ] The agent's egress is the allowlist (model APIs + package registries) and the oracle passes under it (the task itself downloads nothing at run time); no file the agent reads, in the task or its image, names or locates a solution, a demonstration or its dataset (PROTOCOL §2.7, §2.11)
- [ ] The caps of every container (a `[[cap]]` rule in `scripts/task_resources.toml`) set from measured peaks with a margin; the oracle stays inside them; `instruction.md` and the README have the rendered `## Resources` (PROTOCOL §6)
- [ ] No credentials, personal paths or files > 2 MB; each changed task's `DIGEST` current (`normalize` writes it; `tasks/dataset.toml` is no longer committed)

## Evidence
<!-- Oracle / nop table; optionally one strong-model run and why it failed. -->
