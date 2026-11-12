# RoboDojo Original

RoboDojo's 54 simulation tasks in standard mode, each with RoboDojo's own instruction as its task text. Everything else
is RoboDojo's standard tasks (`tasks/robodojo/`, README there): the same frozen scenes, sim service, step limits, success
checks, verifier, images, caps and seeds.

## Quick start
```bash
harbor run -d embodied-first-exam@all -p tasks/robodojo-original -a codex -m openai/gpt-6-astra
```

## Before you run
As for RoboDojo (tasks/robodojo/README.md, "Before you run"): the same simulator image, built from the same recipe, with
the same licences, GPU, driver and disk needs.

## Tasks
`robodojo-original-<task>-i00-standard`, one per `robodojo-<task>-i00-standard` (54 dirs; `eai-standard/2.2`,
agent-ended episodes, RoboDojo's own `step_lim` as the step limit). Standard mode only: there are no privileged or
realtime twins. Each dir has the same `environment/` and `tests/` as its RoboDojo counterpart (byte for byte, apart from
the task name in the two resource-limits files), so the scene, service and grading are
the same. Only `instruction.md`, `README.md` and `task.toml`'s name, description and oracle evidence differ.

## Instructions
The task text is RoboDojo's own instruction for the frozen scene, verbatim: the sentence that
`gen_instruction()` in `task/RoboDojo/tasks/<task>.py` of
[RoboDojo](https://github.com/RoboDojo-Benchmark/RoboDojo) @ `cb2a37d58dbbeb2777b0c2b30e97ecb8ab91095c` produces,
with the placeholders filled by RoboDojo's description manager (`env/description_manager/desc_manager.py`) for
evaluation layout `arx_x5/0/<task>_0.json`. It is recorded as `instruction` in each `environment/sim/instance.json`, and
the service's `/spec` returns it as `task_text`. The rest of the instruction is only what a standard task needs to run:
the robot, the rules of the episode, the client and its action types, the world frame, and the sections every task
renders (`## Episode end`, `## External material`, `## Resources`, `## Time limit`).

This departs from the instruction convention (PROTOCOL §5) on purpose: an instruction here does not describe the
objects or state the check, its thresholds or its stages, and it keeps RoboDojo's wording as it is (including
identifiers such as `wooden_toy` and lower-case starts). The agent has what a RoboDojo policy has: the sentence, the
cameras and the proprioception. `tasks/robodojo/` keeps the instructions that describe the objects and the check.

## Success
RoboDojo's own check, unchanged (tasks/robodojo/README.md "Success" and "Grading"): judged by the sim service on the
live episode, which the agent ends with `POST /finish`, or which ends at the step limit or the time limit.

## Oracle
None (`oracle = "none"`): RoboDojo releases no expert. The scenes and checks have the positive examples recorded for
`tasks/robodojo/`. These texts have no trial yet.

## Core set
Not in `docs/core_set.toml`: the suite is a separate run of its own.
