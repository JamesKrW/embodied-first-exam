"""roboEngineer's instruction sections for the ablations (scripts/ablation_tasks.py writes them for a task dir under
tasks/roboengineer/): the same headings as every other benchmark's, the text for its policy-submission interface
(tasks/roboengineer/README.md "Ablations"). The landing tasks' client takes the route as `call(operation, ...)`."""
from __future__ import annotations

import pathlib

BENCHMARK = "roboengineer"
NOSIGNAL_HEAD = "## This run: no success signal"
QUERY_HEAD = "## This run: privileged state queries"
RESET_HEAD = "## This run: unlimited resets, save and load"
LANDING_CALL = "url=\"http://127.0.0.1:8000\""


def applies(d: pathlib.Path | None) -> bool:
    return d is not None and pathlib.Path(d).resolve().parent.name == BENCHMARK


def _landing(d: pathlib.Path) -> bool:
    return "-landing-" in pathlib.Path(d).name


def _base(d: pathlib.Path) -> str:
    """The robot's entity name (trusted/ablation.py `entities`): the car, the drone, else the robot."""
    name = pathlib.Path(d).name
    return "car" if "-anycar-" in name else "drone" if "-neuralfly-" in name or "-landing-" in name else "robot"


def nosignal(d: pathlib.Path) -> str:
    call = ("`from scripts.quadrotor_landing.public.client import call; call(\"finish\", {\"round_id\": \"round-2\", "
            f"\"summary\": \"...\"}}, {LANDING_CALL})`") if _landing(d) else "`Client().finish(round_id, summary)`"
    return f"""{NOSIGNAL_HEAD}

This run adds an explicit end to adaptation. This section overrides anything above that says only sealing ends it.

- No response tells you whether a policy succeeds or how it scores: a deployment returns its noisy telemetry and frames
  only, as described above, and `/spec` says `"success_signal": "hidden"`.
- You decide when you are done: send `POST /finish` with `{{"round_id": <a completed round>, "summary": "..."}}` (both
  optional) on the same base URL as your other calls (in Python: {call}). That round, or without one the latest
  completed round, is the graded policy, evaluated on the held-out cases as described above. After `/finish` every call
  is refused (409). Sealing a round ends adaptation the same way.
- If you do neither, the latest round completed before your time is up is graded, as described above.

"""


def query(d: pathlib.Path) -> str:
    if _landing(d):
        ent = "`json.load(urllib.request.urlopen(\"http://127.0.0.1:8000/state/entities\"))`"
        q = (f"`call(\"state/query\", {{\"names\": [\"drone\"], \"case\": 0}}, {LANDING_CALL})` (with `call` from "
             "`scripts.quadrotor_landing.public.client`)")
    else:
        ent, q = "`Client().state_entities()`", f"`Client().state_query([\"{_base(d)}\"], case=0)`"
    return f"""{QUERY_HEAD}

This run lets you read the simulator's true state during your development episodes. This section overrides anything
above, and anything in `/describe`, that says the true state is never returned during adaptation.

- `GET /state/entities` (in Python: {ent}) lists what you can query: `{{"frame", "quaternion_order", "entities": {{name:
  {{"kind", "fields": {{field: unit}}, "static"}}}}, "rounds"}}`. The kinds are `robot` (the robot's base or root),
  `robot_link` (a foot, a hand, the torso), `object` and `scene_feature` (poles, a parking slot, a pad, the terrain).
  `rounds` lists the completed rounds you can query.
- `POST /state/query` with `{{"names": [...], "fields": [...], "round_id": ..., "case": ..., "t": ...}}` (in Python:
  {q}) answers `{{"round_id", "cases": [{{"case", "t": [...], "state": {{name: {{field: [one value per time]}}}}}}],
  "state_queries"}}`: the true state (no noise, no bias) of that round's development episodes at their telemetry
  times. Only `names` is required: `fields` defaults to every field, `round_id` to the latest completed round, `case`
  to every development case of that round, and `t` (seconds, or a window `[t0, t1]`) to every sample. A `static`
  entity gives one value instead of a list.
  - The usual fields: `pos` [x, y, z] m, `quat` (in `quaternion_order`), `lin_vel` m/s, `ang_vel` rad/s (both in the
    world frame), `height_above_ground` m, `joint_pos` / `joint_vel` {{joint: value}}, `size` m. Each field's unit is
    in the list.
  - An unknown name, field or round is a 400 that lists the valid ones.
- A query runs no simulation, changes nothing and costs no round. Queries are counted and recorded; they do not change
  the grading. After adaptation has ended a query is refused (409).
- Nothing else changes: deployments return the same noisy feedback, and no answer holds a score, a success flag, the
  hidden dynamics or anything about the held-out cases.

"""


def reset(d: pathlib.Path) -> str:
    if _landing(d):
        rid = "\"round-1\""
        calls = (f"`call(\"save_state\", {{...}}, {LANDING_CALL})`, `call(\"load_state\", {{...}}, {LANDING_CALL})`, "
                 "then the client's `deploy`")
    else:
        rid, calls = "0", "`c.save_state(round_id, case, t)`, `c.load_state(handle)`, then `c.deploy(policy, diagnosis)`"
    return f"""{RESET_HEAD}

This run raises the deployment budget and adds two calls. This section overrides anything above, and anything in
`/describe`, that says there are three rounds, or that no reset exists.

- You have 1003 deployment rounds (`max_rounds` in `/describe`): in practice as many as your time allows. Every
  deployment starts each development case afresh from its starting state (that is the reset); the cases and the hidden
  dynamics stay the same.
- `POST /save_state` with `{{"round_id": {rid}, "case": 0, "t": 4.0}}` names the state of that completed development
  episode at its telemetry time at or before `t` seconds and answers `{{"handle": "s1", "t": ...}}`. The handle is all
  you get, never the state's contents. At most 32 handles are kept (the least recently used one is dropped).
- `POST /load_state` with `{{"handle": "s1"}}` makes your next deployment run only that case, starting from that saved
  state, with the policy you deploy taking over there. The controller's run-time state (its phase, timers,
  integrators, last action) carries over; its parameters are the new policy's. The loaded deployment's feedback covers
  the whole episode, the saved path included, and its answer has `start` and `load_verified`. `{{"handle": null}}`
  cancels a pending load. Saves and loads cost no round. In Python: {calls}.
- Grading is unchanged: the selected round's policy (sealed, or else the latest completed round) runs every held-out
  case from its start, as described above; a policy deployed after a load is evaluated like any other.
- Your time keeps running across deployments, saves and loads.

"""
