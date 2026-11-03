#!/usr/bin/env python3
"""The task dirs of the reset + save/load and state-query ablations (docs/protocol/standard_mode.md, Part 1).

    python3 scripts/ablation_tasks.py apply reset|query|both <task dir> ...
    python3 scripts/ablation_tasks.py check reset|query|both <task dir> ...

reset: compose `sim` environment: RCB_RESET_BUDGET default -> 1000 (added when absent), RCB_STATE_OPS "1";
       instruction: every stated "<N> resets" -> 1000, and the section RESET_HEAD before `## External material`.
query: compose `sim` environment: RCB_STATE_QUERY "1"; instruction: the section QUERY_HEAD.
both:  both. Idempotent; a section or switch of an earlier form (OLD_HEADS,
       RCB_EXPOSE_OBJECT_POSES) is replaced.
"""
from __future__ import annotations

import pathlib
import re
import sys

BUDGET = 1000
RESET_HEAD = "## This run: unlimited resets, save and load"
RESET_TEXT = f"""{RESET_HEAD}

This run changes the reset budget and adds two calls. This section overrides anything above, and anything in `/spec`,
that says there is no reset, one episode only, or fewer resets.

- You have {BUDGET} resets (`spec()["budget"]["reset"]` is {BUDGET}): in practice as many as your time allows.
- `POST /reset` on the same base URL as your other calls (`http://sim:8000` or `http://localhost:8000`), with a JSON body
  such as `{{"cameras": []}}` (in Python, for example `requests.post(base + "/reset", json={{"cameras": []}})`; the
  client's `_post("/reset", {{"cameras": []}}, [], 60)` where it has no `reset()`). It abandons the running episode, or
  follows one that has ended (a failure or a fall included), and starts a new episode from the same starting state.
  The answer carries the new episode's first frames; `meta` and `GET /status` add `episode`, `resets_left`, `saves`,
  `loads` and `control_steps_total`.
- Where `/spec` has `state_ops`, two more calls exist (its text is the full rule):
  - `POST /save_state` with `{{}}` stores the current state in the service and answers JSON
    `{{"handle": "s1", "step": ...}}`. The handle is all you get, never the state's contents.
  - `POST /load_state` with `{{"handle": "s1", "cameras": []}}` restores a state saved earlier in this run and starts a
    new episode there; it answers as `/reset` does. At most 32 handles are kept (the least recently used one is
    dropped). Saves and loads do not use up resets.
  Where `/spec` has no `state_ops`, this task offers resets only.
- Your time keeps running across resets and loads (in standard-realtime the robot's clock pauses after a reset or a
  load until your next command).
- Only the last episode is graded: from the starting state, or for a loaded episode the path that led to the saved
  state followed by what you did after the load. The success rule is checked again from your next command on. A
  success still ends the run: no reset, save or load after it.

"""
QUERY_HEAD = "## This run: privileged state queries"
OLD_HEADS = ("## This run: object poses",)            # sections an earlier form of this tool wrote: replaced
QUERY_TEXT = f"""{QUERY_HEAD}

This run lets you query the simulator's state. This section overrides anything above, and anything in `/spec`, that
says object poses or the scene's state are never given.

- `GET /state/entities` (on the same base URL as your other calls) lists what you can query:
  `{{"frame": ..., "quaternion_order": ..., "entities": {{name: {{"kind": ..., "fields": {{field: unit}}}}}}}}`. The
  kinds are `robot` (the robot's base or root), `robot_link` (a hand, a foot, an end effector), `object`,
  `articulation` (a door, a drawer, a faucet: its joints) and `scene_feature` (a zone, a line, a hazard, a canvas
  region). The list is fixed for the run.
- `POST /state/query` with `{{"names": ["robot", ...], "fields": ["pos", "quat"]}}` answers JSON
  `{{"state": {{name: {{field: value}}}}, "control_steps": ..., "sim_time_s": ..., "state_queries": ...}}` with the
  current values. `fields` is optional: without it you get every field of each name. In Python:
  `requests.post(base + "/state/query", json={{"names": ["robot"]}}).json()["state"]`.
  - The usual fields: `pos` [x, y, z] m, `quat` (in `quaternion_order`), `lin_vel` m/s, `ang_vel` rad/s,
    `height_above_ground` m, `joint_pos` / `joint_vel` {{joint: value}}, `size` [3] m (full extents), and `bbox`
    [[min x, y, z], [max x, y, z]] m. A task may add its own; each field's unit is in the list.
  - All values are in the world frame that `frame` describes.
  - A field that an entity does not have is left out of its entry.
  - An unknown name or field is a 400 that lists the valid ones.
- A query only reads. It takes no control step, and it does not advance or pause the clock. In standard-realtime it
  answers at the current simulation time while the robot keeps moving. It is served while the episode runs (409 after
  the episode ended). Queries are counted and recorded; they do not change the grading.
- Nothing else changes: the goal, the success rule and its thresholds are not given, and no answer says whether the
  task is done.

"""


def _sim_env(lines: list[str]) -> tuple[int, str]:
    sim = next(i for i, ln in enumerate(lines) if re.match(r"^  sim:\s*(#.*)?$", ln))
    env = next(i for i in range(sim + 1, len(lines)) if re.match(r"^    environment:\s*(#.*)?$", lines[i]))
    if any(re.match(r"^  \S", ln) for ln in lines[sim + 1:env]):
        raise ValueError("the sim service has no environment mapping")
    indent = re.match(r"^(\s*)", lines[env + 1]).group(1)
    if len(indent) <= 4 or lines[env + 1].lstrip().startswith("-"):
        raise ValueError("sim environment is not a mapping")
    return env, indent


def compose(text: str, kind: str) -> str:
    lines = text.split("\n")
    env, indent = _sim_env(lines)
    add = []
    if kind in ("reset", "both"):
        decl = [i for i, ln in enumerate(lines) if re.match(r"^\s*RCB_RESET_BUDGET:", ln)]
        new = f'{indent}RCB_RESET_BUDGET: "${{RCB_RESET_BUDGET:-{BUDGET}}}"   # the unlimited-reset ablation'
        if decl:
            for i in decl:
                lines[i] = new
        else:
            add.append(new)
        if "RCB_STATE_OPS" not in text:
            add.append(f'{indent}RCB_STATE_OPS: "1"   # the reset ablation: /reset, /save_state, /load_state and their record')
    if kind in ("query", "both"):
        lines = [ln for ln in lines if not re.match(r"^\s*RCB_EXPOSE_OBJECT_POSES:", ln)]
        env, indent = _sim_env(lines)
        if "RCB_STATE_QUERY" not in text:
            add.append(f'{indent}RCB_STATE_QUERY: "1"   # the state-query ablation (/state/entities, /state/query)')
    lines[env + 1:env + 1] = add
    return "\n".join(lines)


STATED = re.compile(r"\b(\d+)(\*{0,2}\s+resets?)\b", re.I)


def instruction(text: str, kind: str) -> str:
    secs = []
    if kind in ("reset", "both"):
        text = STATED.sub(lambda m: f"{BUDGET}{m.group(2)}" if m.group(1) != str(BUDGET) else m.group(0), text)
        if RESET_HEAD not in text:
            secs.append(RESET_TEXT)
    if kind in ("query", "both"):
        for head in OLD_HEADS:                       # an earlier section: up to the next "## " heading
            if head in text:
                i = text.index(head)
                j = text.index("\n## ", i + len(head)) + 1
                text = text[:i] + text[j:]
        if QUERY_HEAD not in text:
            secs.append(QUERY_TEXT)
    if secs:
        k = text.index("\n## External material")
        text = text[:k + 1] + "".join(secs) + text[k + 1:]
    return text


def apply(d: pathlib.Path, kind: str) -> bool:
    changed = False
    for name, fn in (("environment/docker-compose.yaml", compose), ("instruction.md", instruction)):
        p = d / name
        old = p.read_text()
        new = fn(old, kind)
        if new != old:
            p.write_text(new)
            changed = True
    return changed


def main() -> int:
    if len(sys.argv) < 3 or sys.argv[1] not in ("apply", "check") or sys.argv[2] not in ("reset", "query", "both"):
        print(__doc__)
        return 2
    kind, dirs = sys.argv[2], [pathlib.Path(a) for a in sys.argv[3:]]
    if sys.argv[1] == "apply":
        print(f"changed {sum(apply(d, kind) for d in dirs)} of {len(dirs)} task dirs")
        return 0
    bad = [d for d in dirs if any(fn((d / n).read_text(), kind) != (d / n).read_text()
                                  for n, fn in (("environment/docker-compose.yaml", compose), ("instruction.md", instruction)))]
    for d in bad:
        print(f"not in the ablation's form: {d}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
