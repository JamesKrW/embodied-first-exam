# RoboDojo Original Astra Prompt

RoboDojo Original Auto-end (`tasks/robodojo-original-autoend/`) with the text RoboDojo's own GPT-6 Astra evaluation gave
the model. RoboDojo evaluated GPT-6 Astra itself (the leaderboard's Agent entries; adapter `GPT_6_Astra_Direct_EEF` in
XPolicyLab). Its prompt had three task-facing texts, and each task here has the same three:
- RoboDojo's one-sentence instruction, unchanged;
- the task recipe: the adapter's `inspect/recipes/<task>.md`, verbatim. That is RoboDojo's wiki capability dimension,
  description and scoring ladder, plus a short `## Notes` for four tasks. It comes after the instruction under
  `TASK RECIPE:`, as in the adapter's Goal turn (`inspect/policy.py` `_goal_content`). A random-layout variant has its
  base task's recipe, as in the adapter (`inspect/recipes/__init__.py` `task_recipe`);
- the note on the arms: the adapter's `docs.py` lines 71-75, verbatim, in the paragraph that describes the robot.

Everything else is Auto-end's: the frozen scenes, sim service, step limits, success checks, episode end (a success ends
the episode; no `POST /finish`), verifier, images, caps, seeds, client and time limit.

Source: XPolicyLab @ `6baccd57ff958570d0db1588370289ee36c8bb70`, `policy/GPT_6_Astra_Direct_EEF/`, under the Apache License
2.0. A copy of that licence is in `embodied_first_exam/benchmarks/robodojo-original-astra/LICENSE.xpolicylab`. The
recipes and the note are XPolicyLab's text, unmodified.

## Quick start
```bash
harbor run -d embodied-first-exam@all -p tasks/robodojo-original-astra -a codex -m openai/gpt-6-astra
```

## Before you run
As for RoboDojo (tasks/robodojo/README.md, "Before you run"): the same simulator image, built from the same recipe,
with the same licences, GPU, driver and disk needs.

## Tasks
`robodojo-original-astra-<task>-i00-standard`, one per `robodojo-original-autoend-<task>-i00-standard` (54 dirs;
`eai-standard/2.2`, RoboDojo's own `step_lim` as the step limit). Standard mode only. Each dir is its Auto-end
counterpart with these differences:
- `instruction.md`: the task recipe after the task sentence, the note on the arms, and the sentence that introduces
  them;
- the task name (`task.toml`, the two resource-limits files), `task.toml`'s description, `README.md`.

## Success
RoboDojo's own check, unchanged (tasks/robodojo/README.md "Success" and "Grading"). The sim service judges it on the
live episode, which ends at the check's first pass, at the step limit, at a failure or at the time limit. The recipe's
100-point row describes that check. Its lower rows are RoboDojo's partial scores, which are not graded here.

## Oracle
None (`oracle = "none"`): RoboDojo releases no expert.

## Core set
Not in `docs/core_set.toml`: the suite is a separate run of its own.
