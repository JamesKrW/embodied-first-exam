#!/usr/bin/env python3
"""The no-success-signal ablation's task dirs (docs/protocol/standard_mode.md "No success signal"): a standard or
standard-realtime task dir as the ablation runs it. Two changes, both idempotent:

  - environment/docker-compose.yaml: `RCB_HIDE_SUCCESS: "1"` first in the `sim` service's environment;
  - instruction.md: the section ADDENDUM before `## External material` (the closing sections stay last).

    python3 scripts/ablation_tasks.py apply <task dir> ...     change them (then normalize them: their DIGEST)
    python3 scripts/ablation_tasks.py check <task dir> ...     exit 1 if any is not in the ablation's form
    python3 scripts/ablation_tasks.py core [--out FILE]        the core set's ablation dirs: default mode standard or
                                                               standard-realtime, a benchmark served by the shared
                                                               eai-standard services (not EXCLUDED)
"""
from __future__ import annotations

import pathlib
import re
import sys
import tomllib

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
import task_conventions as tc  # noqa: E402

# a benchmark's own runtime (not an eai-standard service): out of the ablation's scope
EXCLUDED = ("embodied-olympics-match", "roboengineer")
ENV_LINE = 'RCB_HIDE_SUCCESS: "1"                            # the no-success-signal ablation (standard_mode.md)'
HEADING = "## This run: no success signal"
ADDENDUM = f"""{HEADING}

This run changes how the episode ends. This section overrides anything above, and anything in `/spec`, that says a
success ends the episode, that `meta["reason"]` can be `"success"`, or that a success refuses later calls or resets.

- The robot never tells you whether you have succeeded: reaching the goal changes no response, status, error or
  counter, and the episode does not end there.
- You decide when you are done: send `POST /finish` with an empty JSON body (`{{}}`) to the service, on the same base
  URL as your other calls (`http://sim:8000` or `http://localhost:8000`; in Python, for example
  `requests.post(base + "/finish", json={{}})`). The state at that moment is graded with the task's success rule (with
  the task's settle or hold time described above, run from that state). After `/finish` every call is refused (409).
- If you do not call it, the state when your session ends (you stop, or your time runs out) is graded.
- A success reached earlier and then undone does not count: only the state at the end is graded. An event the task
  records once it happens (for example a scored goal or a completed stage sequence) stays part of that state, also
  when one of the task's failure rules ends the episode after it. Resets work as described above, within the budget,
  and only the episode that ends with `/finish` or at the end of your session is graded.
- The episode still ends on its own on a simulator error, at the task's technical step limit (graded as it stands),
  and where the task's own rules end it as a failure (for example a fall), as described above.

"""


def apply(d: pathlib.Path) -> bool:
    """Bring one task dir into the ablation's form; True if anything changed."""
    changed = False
    comp = d / "environment" / "docker-compose.yaml"
    text = comp.read_text()
    if "RCB_HIDE_SUCCESS" not in text:
        lines = text.split("\n")
        sim = next(i for i, ln in enumerate(lines) if re.match(r"^  sim:\s*(#.*)?$", ln))
        env = next(i for i in range(sim + 1, len(lines)) if re.match(r"^    environment:\s*(#.*)?$", lines[i]))
        if any(re.match(r"^  \S", ln) for ln in lines[sim + 1:env]):
            raise ValueError(f"{comp}: the sim service has no environment mapping")
        nxt = lines[env + 1]
        indent = re.match(r"^(\s*)", nxt).group(1)
        if len(indent) <= 4 or nxt.lstrip().startswith("-"):
            raise ValueError(f"{comp}: sim environment is not a mapping")
        lines.insert(env + 1, indent + ENV_LINE)
        comp.write_text("\n".join(lines))
        changed = True
    ins = d / "instruction.md"
    text = ins.read_text()
    if HEADING not in text:
        k = text.index("\n## External material")
        ins.write_text(text[:k + 1] + ADDENDUM + text[k + 1:])
        changed = True
    return changed


def in_form(d: pathlib.Path) -> bool:
    comp = (d / "environment" / "docker-compose.yaml").read_text()
    ins = (d / "instruction.md").read_text()
    return 'RCB_HIDE_SUCCESS: "1"' in comp and ADDENDUM.strip() in ins


def core() -> list[pathlib.Path]:
    out = []
    for t in tomllib.loads((ROOT / "docs" / "core_set.toml").read_text())["task"]:
        if t["default_mode"] not in ("standard", "standard-realtime"):
            continue
        d = tc.task_dir(f"{t['task']}-{t['default_mode']}", ROOT / "tasks")
        if d.parent.name not in EXCLUDED:
            out.append(d)
    return out


def main() -> int:
    if len(sys.argv) < 2 or sys.argv[1] not in ("apply", "check", "core"):
        print(__doc__)
        return 2
    if sys.argv[1] == "core":
        dirs = core()
        text = "".join(f"{d.relative_to(ROOT)}\n" for d in dirs)
        if "--out" in sys.argv:
            pathlib.Path(sys.argv[sys.argv.index("--out") + 1]).write_text(text)
        else:
            sys.stdout.write(text)
        print(f"{len(dirs)} task dirs", file=sys.stderr)
        return 0
    dirs = [pathlib.Path(a).resolve() for a in sys.argv[2:]]
    if sys.argv[1] == "apply":
        n = sum(apply(d) for d in dirs)
        print(f"changed {n} of {len(dirs)} task dirs")
        return 0
    bad = [d for d in dirs if not in_form(d)]
    for d in bad:
        print(f"not in the ablation's form: {d}")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
