# RoboDojo Original Auto-end

RoboDojo Original (`tasks/robodojo-original/`) with RoboDojo's own episode end: the episode ends as a success at the
first step where RoboDojo's check for the task passes, as RoboDojo's evaluator ends a policy's episode, and the agent
never ends it (no `POST /finish`). Everything else is RoboDojo Original's: RoboDojo's one-sentence instruction, the same
frozen scenes, sim service, step limits, success checks, verifier, images, caps and seeds.

## Quick start
```bash
harbor run -d embodied-first-exam@all -p tasks/robodojo-original-autoend -a codex -m openai/gpt-6-astra
```

## Before you run
As for RoboDojo (tasks/robodojo/README.md, "Before you run"): the same simulator image, built from the same recipe, with
the same licences, GPU, driver and disk needs.

## Tasks
`robodojo-original-autoend-<task>-i00-standard`, one per `robodojo-original-<task>-i00-standard` (54 dirs;
`eai-standard/2.2`, RoboDojo's own `step_lim` as the step limit). Standard mode only. Each dir is its RoboDojo Original
counterpart with these differences:
- `environment/docker-compose.yaml`: the `sim` service sets `RCB_END_ON_SUCCESS: "1"` and
  `RCB_SERVER: /srv/rcb_end_on_success.py`;
- `environment/sim/rcb_end_on_success.py`: the service's entry point (below), the same file in every dir;
- `instruction.md`: the `## Episode end` section, and the two sentences that point to it;
- the task name (`task.toml`, the two resource-limits files), `task.toml`'s description and oracle evidence, `README.md`.

## The episode end
`rcb_end_on_success.py` runs RoboDojo's sim service (`server.py` of `images/robodojo-files`, unchanged) in its own
process after changing four things of the agent-ended-episode layer it uses (`rcb_agent_end.py`, PROTOCOL §3):
- every control step, after the service records whether RoboDojo's check passes, a pass ends the episode at that step
  (reason `"success"`; the call stops there and answers `meta["done"]`, `meta["reason"]`; later `/submit` calls are 409);
- the verdict is unchanged: RoboDojo's reward reached 1 in the episode (the service's latch), so an end at the success is
  a success and every other end (the step limit, a check that fails for good, the end of the agent's session, a
  simulator error) is not;
- `POST /finish` is a 404: the agent never ends the episode, as a RoboDojo policy cannot;
- `/spec` (`success_signal`, `episode`) and `final.json` (`success_signal = "ends_episode"`, `ended_by = "success"`)
  say so.

So an episode runs as in RoboDojo's own evaluation (`EvalEnv.is_episode_end`, `src/eval_client/eval_env.py` @
`cb2a37d`): it ends at the first action whose reward reaches 1, at a failure, or after `step_lim` actions. The end of the
episode is the agent's only sign of a success: RoboDojo's check is not stated (as in RoboDojo Original), and in most
tasks it includes both arms back at their start pose.

## Instructions
RoboDojo Original's (tasks/robodojo-original/README.md "Instructions"): RoboDojo's own sentence for the frozen scene,
verbatim, plus only what a standard task needs to run. `## Episode end` is rendered by `scripts/task_conventions.py`
for a service with `RCB_END_ON_SUCCESS`.

## Success
RoboDojo's own check, unchanged (tasks/robodojo/README.md "Success" and "Grading"): judged by the sim service on the
live episode, which ends at the check's first pass, at the step limit, at a failure or at the time limit.

## Oracle
None (`oracle = "none"`): RoboDojo releases no expert.

## Core set
Not in `docs/core_set.toml`: the suite is a separate run of its own.
