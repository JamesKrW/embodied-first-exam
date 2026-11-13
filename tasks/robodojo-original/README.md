# RoboDojo Original

RoboDojo's 54 standard tasks with the text RoboDojo itself gave GPT-6 Astra in its own evaluation, and with RoboDojo's
own episode end. That evaluation is the leaderboard's Agent entry (report arXiv 2609.24170; adapter
`GPT_6_Astra_Direct_EEF` in XPolicyLab). Each task's text has three parts, as that prompt had:
- RoboDojo's one-sentence instruction for the frozen scene (`gen_instruction()` of `task/RoboDojo/tasks/<task>.py`,
  RoboDojo @ `cb2a37d`), unchanged;
- the task recipe: the adapter's `inspect/recipes/<task>.md`, verbatim. That is RoboDojo's wiki capability dimension,
  description and scoring ladder, plus a short `## Notes` for four tasks. It comes after the instruction under
  `TASK RECIPE:`, as in the adapter's Goal turn (`inspect/policy.py` `_goal_content`). A random-layout variant has its
  base task's recipe, as in the adapter (`inspect/recipes/__init__.py` `task_recipe`);
- the note on the arms: the adapter's `docs.py` lines 71-75, verbatim, in the paragraph that describes the robot.

Source of the recipes and the note: XPolicyLab @ `6baccd57ff958570d0db1588370289ee36c8bb70`,
`policy/GPT_6_Astra_Direct_EEF/`, under the Apache License 2.0. A copy of that licence is in
`embodied_first_exam/benchmarks/robodojo-original/LICENSE.xpolicylab`. The recipes and the note are XPolicyLab's text,
unmodified. Each task's README names its recipe file.

The scenes, sim service, step limits and success checks are RoboDojo's, as in `tasks/robodojo/` (the same images).

## Quick start
```bash
harbor run -d embodied-first-exam@all -p tasks/robodojo-original -a codex -m openai/gpt-6-astra
```

## Before you run
As for RoboDojo (tasks/robodojo/README.md, "Before you run"): the same simulator image, built from the same recipe,
with the same licences, GPU, driver and disk needs.

## Tasks
`robodojo-original-<task>-i00-standard`, one per RoboDojo task in `tasks/robodojo/` (54 dirs; `eai-standard/2.2`,
RoboDojo's own `step_lim` as the step limit). Standard mode only. Each dir has RoboDojo's frozen evaluation layout
(`arx_x5/0/<task>_0.json`) and `tasks/robodojo/`'s sim service, check and verifier.

## The episode end
The sim service ends the episode as RoboDojo's evaluator ends a policy's episode (`EvalEnv.is_episode_end`,
`src/eval_client/eval_env.py` @ `cb2a37d`): at the first action whose reward reaches 1, at a failure, or after `step_lim`
actions. The agent never ends it.

The compose file's `sim` service sets `RCB_END_ON_SUCCESS: "1"` and `RCB_SERVER: /srv/rcb_end_on_success.py`. The
entry point `environment/sim/rcb_end_on_success.py` is the same file in every dir. It runs RoboDojo's sim service
(`server.py` of `images/robodojo-files`, unchanged) in its own process, after changing four things of the
agent-ended-episode layer it uses (`rcb_agent_end.py`, PROTOCOL §3):
- every control step, after the service records whether RoboDojo's check passes, a pass ends the episode at that step
  (reason `"success"`; the call stops there and answers `meta["done"]`, `meta["reason"]`; later `/submit` calls are 409);
- the verdict is RoboDojo's reward reaching 1 in the episode (the service's latch), so an end at the success is a
  success and every other end (the step limit, a check that fails for good, the end of the agent's session, a simulator
  error) is not;
- `POST /finish` is a 404: the agent never ends the episode, as a RoboDojo policy cannot;
- `/spec` (`success_signal`, `episode`) and `final.json` (`success_signal = "ends_episode"`, `ended_by = "success"`)
  say so.

## Instructions
RoboDojo's sentence, the recipe and the note, plus only what a standard task needs to run. `## Episode end` is rendered by
`scripts/task_conventions.py` for a service with `RCB_END_ON_SUCCESS`.

## Success
RoboDojo's own check, unchanged (tasks/robodojo/README.md "Success" and "Grading"). The sim service judges it on the
live episode, which ends at the check's first pass, at the step limit, at a failure or at the time limit. The recipe's
100-point row describes that check. Its lower rows are RoboDojo's partial scores, which are not graded here.

## Oracle
None (`oracle = "none"`): RoboDojo releases no expert.

## Core set
Not in `docs/core_set.toml`.
