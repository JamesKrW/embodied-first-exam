#!/usr/bin/env python3
"""The Embodied Olympics match tasks' ablation task dirs (docs/ablations.md, "Embodied Olympics match"): a standard or
standard-realtime match task dir as an ablation runs it. The match service runs unchanged behind a thin layer
(scripts/ablation_eomatch/rcb_ablation/, copied into the task's sim image at /srv/rcb_ablation/) that adds the
ablation's calls when its switches are set:

  nosignal  RCB_HIDE_SUCCESS "1"                     no score, point results or verdict in any reply; POST /finish
  query     RCB_STATE_QUERY "1"                      GET /state/entities, POST /state/query
  reset     RCB_STATE_OPS "1", RCB_RESET_BUDGET 1000 POST /reset, /save_state, /load_state
  both      query + reset

Per task dir (idempotent):
  - environment/docker-compose.yaml: the `sim` service's command `python3 /srv/rcb_ablation/launch.py` (core/launch.py
    unchanged, its match service started as `python -m rcb_ablation.serve`) and the switches first in its environment;
  - environment/sim/rcb_ablation/: the layer (the sim image's `COPY . /srv/` takes it);
  - tests/rcb_ablation_reward.py, run by tests/test.sh after fexam_reward.py: the ablation's counts from final.json
    into reward.json (numbers only; the success is the grader's);
  - instruction.md: the ablation's section before `## External material`.

    python3 scripts/ablation_eomatch.py apply nosignal|query|reset|both <task dir> ...   (then normalize: DIGEST)
    python3 scripts/ablation_eomatch.py check nosignal|query|reset|both <task dir> ...
    python3 scripts/ablation_eomatch.py core nosignal|query|reset|both                   the core set's dirs
"""
from __future__ import annotations

import filecmp
import pathlib
import re
import shutil
import sys
import tomllib

ROOT = pathlib.Path(__file__).resolve().parents[1]
LAYER = ROOT / "scripts" / "ablation_eomatch" / "rcb_ablation"
BENCH = "embodied-olympics-match"
BUDGET = 1000
LAUNCH_OLD = "command: python3 /srv/core/launch.py"
LAUNCH_NEW = "command: python3 /srv/rcb_ablation/launch.py   # the ablation's layer around core/launch.py"
SWITCHES = {"nosignal": {"RCB_HIDE_SUCCESS": "1"}, "query": {"RCB_STATE_QUERY": "1"},
            "reset": {"RCB_STATE_OPS": "1", "RCB_RESET_BUDGET": str(BUDGET)},
            "both": {"RCB_STATE_QUERY": "1", "RCB_STATE_OPS": "1", "RCB_RESET_BUDGET": str(BUDGET)}}

NOSIGNAL_HEAD = "## This run: no success signal"
NOSIGNAL_TEXT = """{head}

This run hides the match's result. This section overrides anything above, and anything in `/spec`, that says the
replies show the score.

- No reply shows the score, a point's, rally's or phrase's result, who scored a goal or the match's winner: they are
  left out of `/status`, `/observe` and every other answer (`/spec` keeps the rules). The rest is unchanged: the
  phase (`over` once the match has ended by its rules), the reviews, the clocks, your robot's state and the images.
- You decide when you are done: `POST http://localhost:8000/$SEAT/finish` with `{{}}` (`game` has no command for it; in
  Python `requests.post(base + "/finish", json={{}})`). It ends your session at once and the match is decided as when
  your session ends: before `game start` a no-show, during the match abandoned (a valid result, success 0), after the
  match has ended its result stands. After `/finish` every call but `/spec` and `/status` is refused (409).
- The match is graded with the task's rules: you succeed if you win it. Play it to its end before you finish.

"""
QUERY_HEAD = "## This run: privileged state queries"
QUERY_TEXT = """{head}

This run lets you query the simulator's state. This section overrides anything above that says to use only the
camera images, the service's replies and this document, or that the simulator's state is never given.

- `GET http://localhost:8000/$SEAT/state/entities` lists what you can query:
  `{{"frame": ..., "quaternion_order": ..., "entities": {{name: {{"kind": ..., "fields": {{field: unit}}, "note"?}}}}}}`.
  The kinds are `robot` (your robot, and the opponent's when it is one), `robot_link` (your paddle, racket, rods or
  blade), `object` (the ball or the shuttle, the opponent's paddle, racket or blade, a scripted opponent that is only
  drawn), `articulation` (the opponent's rods) and `scene_feature` (the table, net, court, goals, playfield, piste and
  its lines); `note` says what an entity is. The list is fixed for the run.
- `POST http://localhost:8000/$SEAT/state/query` with `{{"names": ["ball", ...], "fields": ["pos"]}}` answers JSON
  `{{"state": {{name: {{field: value}}}}, "control_steps": ..., "sim_time_s": ..., "state_queries": ...}}` with the
  current values from the simulator. `fields` is optional. The fields and their units are in the list: `pos` m,
  `quat`, `lin_vel` m/s, `ang_vel` rad/s, `joint_pos` / `joint_vel` / `joint_range` {{joint: value}}, `size` (the
  full extents along the body's own axes, m) and `bbox` (its axis-aligned box, `[[x, y, z] min, [x, y, z] max]`, m),
  all in the frame `frame` names (your own frame, as in the observation). An unknown name or field is a 400 that
  lists the valid ones.{between}
- A query only reads: no control step, the clock neither advances nor pauses ({clock}).
  It is served until the match is over (409 after). Queries are counted and recorded; they do not change the grading.
- Nothing else changes: no answer holds the score, a point's result or the opponent's plan or strategy.

"""
RESET_HEAD = "## This run: unlimited resets, save and load"
RESET_TEXT = """{head}

This run lets you restart the match and go back to a saved moment. This section overrides anything above that says
there is one match only, or that a lost match ends your run.

- `POST http://localhost:8000/$SEAT/reset` with `{{}}`: the match from its start again (the same instance: the world,
  the opponent, the score and the clocks), as a new episode. You have {budget} resets (`budget.reset` in `/spec`).
- `POST http://localhost:8000/$SEAT/save_state` with `{{}}` answers `{{"handle": "s1", "step": ...}}`: the whole match
  now (the world, the opponent with its own internal state, the random generators, the score, the game clock) kept in
  the service; the handle is all you get. `POST http://localhost:8000/$SEAT/load_state` with `{{"handle": "s1"}}` goes
  back to it exactly, as a new episode. At most 32 handles are kept (the least recently used is dropped). Saves and
  loads do not use up resets.
- {after} The task's time limit keeps running.
- A match you lose by its rules (the opponent wins it, a missed review, the clock) is held at its end: `/status` shows
  phase `over`, and you may still reset or load. A match you win ends the run: no reset, save or load after it.
- Only the last episode is graded, with the task's rules: from the start, or for a loaded episode the play that led
  to the saved moment followed by what came after the load. If your session ends during a match, that match is
  abandoned (success 0); a held loss stays a loss. `final.json` records `resets`, `saves`, `loads` and `episodes`.

"""
SECTIONS = {"nosignal": [(NOSIGNAL_HEAD, NOSIGNAL_TEXT)], "query": [(QUERY_HEAD, QUERY_TEXT)],
            "reset": [(RESET_HEAD, RESET_TEXT)], "both": [(RESET_HEAD, RESET_TEXT), (QUERY_HEAD, QUERY_TEXT)]}

REWARD = '''#!/usr/bin/env python3
"""The ablation's counts (tests/test.sh, after fexam_reward.py): numbers from the sim's final.json added to reward.json
when present. first_success: a success with no reset and no load."""
import json
from pathlib import Path

LOG, DATA = Path("/logs/verifier"), Path("/data")
try:
    r = json.loads((LOG / "reward.json").read_text())
    f = json.loads((DATA / "final.json").read_text())
except (OSError, ValueError):
    raise SystemExit(0)
for k in ("resets", "saves", "loads", "episodes", "state_queries", "resets_at_success", "loads_at_success"):
    if isinstance(f.get(k), (int, float)) and not isinstance(f.get(k), bool):
        r[k] = f[k]
if "finished" in f:
    r["finished"] = int(bool(f["finished"]))
if "resets" in f:
    r["first_success"] = int(bool(r.get("success")) and (f.get("resets") or 0) + (f.get("loads") or 0) == 0)
(LOG / "reward.json").write_text(json.dumps({k: v for k, v in r.items() if isinstance(v, (int, float))}))
'''
TEST_LINE = "python /tests/rcb_ablation_reward.py 2>> /logs/verifier/grade.log   # the ablation's counts"
DOCKER_LINE = "COPY rcb_ablation_reward.py /tests/rcb_ablation_reward.py"


def _compose(text: str, mode: str) -> str:
    lines = text.split("\n")
    sim = next(i for i, ln in enumerate(lines) if re.match(r"^  sim:\s*(#.*)?$", ln))
    end = next((i for i in range(sim + 1, len(lines)) if re.match(r"^  \S", lines[i]) or re.match(r"^\S", lines[i])),
               len(lines))
    for i in range(sim + 1, end):
        if lines[i].strip() == LAUNCH_OLD:
            lines[i] = lines[i].replace(LAUNCH_OLD, LAUNCH_NEW)
    if not any(ln.strip() == LAUNCH_NEW for ln in lines[sim + 1:end]):
        raise RuntimeError("the sim service has no `command: python3 /srv/core/launch.py`")
    env = next(i for i in range(sim + 1, end) if re.match(r"^    environment:\s*(#.*)?$", lines[i]))
    have = {m.group(1) for ln in lines[env + 1:end] if (m := re.match(r"^      (\w+):", ln))}
    add = [f'      {k}: "{v}"   # the ablation (docs/ablations.md)' for k, v in SWITCHES[mode].items() if k not in have]
    lines[env + 1:env + 1] = add
    return "\n".join(lines)


AFTER_REALTIME = ("After a reset or a load the world is frozen until you send `game start` (or `game ready`), at most\n"
                  "  `timing.review_timeout_s` of real time; then it runs on.")
AFTER_TURN = "After a reset or a load the bout resumes at once, on the turn it had, with your clock as it was."
CLOCK = {False: "the world keeps moving in real time", True: "your turn's clock keeps running"}
BETWEEN = {False: "", True: ("\n- A phrase is fenced at once, inside the act that completes it; between phrases the fencers "
                              "stand on\n  guard, and a query gives that layout.")}


def _instruction(text: str, mode: str, turn: bool) -> str:
    after = AFTER_TURN if turn else AFTER_REALTIME
    for head, body in SECTIONS[mode]:
        if head in text:
            continue
        i = text.index("## External material")
        text = text[:i] + body.format(head=head, budget=BUDGET, after=after, clock=CLOCK[turn],
                                      between=BETWEEN[turn]) + text[i:]
    return text


def _turn(compose: str) -> bool:
    """A turn match (the bout): its sport's protocol is eai-standard-turn."""
    return re.search(r'^      RO_SPORT: "fencing"', compose, re.M) is not None


def _layer_files():
    return sorted(p.relative_to(LAYER) for p in LAYER.rglob("*.py"))


def apply(d: pathlib.Path, mode: str) -> bool:
    before = snapshot(d)
    comp = d / "environment" / "docker-compose.yaml"
    comp.write_text(_compose(comp.read_text(), mode))
    dst = d / "environment" / "sim" / "rcb_ablation"
    for rel in _layer_files():
        (dst / rel).parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(LAYER / rel, dst / rel)
    tests = d / "tests"
    (tests / "rcb_ablation_reward.py").write_text(REWARD)
    sh = (tests / "test.sh").read_text()
    if TEST_LINE not in sh:
        sh = sh.rstrip("\n") + "\n" + TEST_LINE + "\n"
        (tests / "test.sh").write_text(sh)
    df = (tests / "Dockerfile").read_text()
    if DOCKER_LINE not in df:
        df = df.replace("RUN chmod +x /tests/test.sh", DOCKER_LINE + "\nRUN chmod +x /tests/test.sh", 1)
        if DOCKER_LINE not in df:
            raise RuntimeError(f"{d.name}: tests/Dockerfile has no `RUN chmod +x /tests/test.sh`")
        (tests / "Dockerfile").write_text(df)
    ins = d / "instruction.md"
    ins.write_text(_instruction(ins.read_text(), mode, _turn(comp.read_text())))
    return snapshot(d) != before


def snapshot(d: pathlib.Path) -> dict:
    return {str(p.relative_to(d)): p.read_bytes() for p in sorted(d.rglob("*")) if p.is_file()}


def check(d: pathlib.Path, mode: str) -> list[str]:
    bad = []
    comp = (d / "environment" / "docker-compose.yaml").read_text()
    if LAUNCH_NEW not in comp:
        bad.append("compose: the sim command")
    bad += [f"compose: {k}" for k in SWITCHES[mode] if not re.search(rf'^      {k}: "', comp, re.M)]
    others = {k for m, s in SWITCHES.items() for k in s} - set(SWITCHES[mode])
    bad += [f"compose: {k} (not this ablation's)" for k in others if re.search(rf'^      {k}: "', comp, re.M)]
    dst = d / "environment" / "sim" / "rcb_ablation"
    bad += [f"layer: {rel}" for rel in _layer_files()
            if not (dst / rel).exists() or not filecmp.cmp(LAYER / rel, dst / rel, shallow=False)]
    if not (d / "tests" / "rcb_ablation_reward.py").exists() or \
            (d / "tests" / "rcb_ablation_reward.py").read_text() != REWARD:
        bad.append("tests/rcb_ablation_reward.py")
    if TEST_LINE not in (d / "tests" / "test.sh").read_text():
        bad.append("tests/test.sh")
    if DOCKER_LINE not in (d / "tests" / "Dockerfile").read_text():
        bad.append("tests/Dockerfile")
    ins = (d / "instruction.md").read_text()
    bad += [f"instruction: {h}" for h, _ in SECTIONS[mode] if h not in ins]
    return bad


def core(mode: str) -> list[pathlib.Path]:
    cs = tomllib.loads((ROOT / "docs" / "core_set.toml").read_text())
    out = []
    for t in cs.get("task", []):
        if t["task"].startswith(f"{BENCH}-") and t.get("default_mode") in ("standard", "standard-realtime"):
            d = ROOT / "tasks" / BENCH / f"{t['task']}-{t['default_mode']}"
            if d.is_dir():
                out.append(d)
    return out


def main(argv):
    if len(argv) < 2 or argv[0] not in ("apply", "check", "core") or argv[1] not in SWITCHES:
        print(__doc__)
        return 2
    cmd, mode, dirs = argv[0], argv[1], [pathlib.Path(a).resolve() for a in argv[2:]]
    if cmd == "core":
        for d in core(mode):
            print(d.relative_to(ROOT))
        return 0
    rc = 0
    for d in dirs:
        if cmd == "apply":
            print(f"{d.name}: {'changed' if apply(d, mode) else 'unchanged'}")
        else:
            bad = check(d, mode)
            if bad:
                print(f"{d.name}: {', '.join(bad)}")
                rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
