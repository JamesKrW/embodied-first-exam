"""Scrub an EmbodiedSWE checkout down to what the agent's image may hold (docs/protocol/leakage.md, PROTOCOL H.2).

    python -I scrub_embodiedswe.py /opt/embodiedswe [--check]

Run in the image build, on a fresh checkout of EmbodiedSWE at the pinned commit, after its `.git` is gone (its history
holds solve-calibrated configurations). Idempotent: a second run changes nothing, and `--check` fails when any rule
would still change something or a banned pointer is left.

What it removes, and why:
- Everything but `robobench/`, `LICENSE` and `pyproject.toml`: `eval/` (prompts, hints, skills, example solutions,
  their Harbor adapter), `data_engine/`, `vla/` (names the public demonstration datasets, e.g. the bulb dataset),
  `rl/`, `sim_gen/`, `real_to_sim/`, `experiments/`, `scripts/`, `tests/`, `docs/`.
- Every suite's `grader/` (the hidden rubric; the verifier gets its task's grader back from the task's `tests/`) and
  `smokes/` (scripted teleport probes and their calibration printouts), and the `deformable` suite (Newton, VLM judge;
  not shipped).
- Every comment and every docstring but the module's in each suite's `configs/envs.py` (the bindings): they are the
  developers' calibration notes (reach envelopes measured with an IK probe, calibrated grasp and release stances,
  grasp-approach azimuth bands, "load-bearing calibrated constants"). A stripped docstring becomes a neutral one.
- In the shipped tasks' scene modules, the comments that hand the agent solve-calibrated numbers (the calibrated grasp
  and placing stances, the measured grasp windows): replaced by a neutral line.
Code is never changed: the scrub asserts that every module's AST without docstrings is unchanged.
"""
from __future__ import annotations

import argparse
import ast
import shutil
import sys
from pathlib import Path

KEEP_TOP = {"robobench", "LICENSE", "pyproject.toml"}
DROP_SUITE_DIRS = ("grader", "smokes")
DROP_SUITES = ("deformable",)

# (file, old text, new text): each old text must occur exactly once on a fresh checkout and never after the scrub.
EDITS: list[tuple[str, str, str]] = [
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "distance is now the hard part: pick-side geometry reproduces the fixed-base tier's measured\n"
     "grasp cell, so the NEW skill this task grades on top of it is the carry.\n",
     "distance is now the hard part: the NEW skill this task grades on top of it is the carry.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "  * PICK ZONE (robot's side, y < 0): the fruit at the calibrated stance spot, the produce\n",
     "  * PICK ZONE (robot's side, y < 0): the fruit, the produce\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "  * FAR SIDE (y > 0): the clay plate, centred at `plate_pos`, inset from the far edge so a\n"
     "    placing stance on the far side reproduces the fixed-base tier's plate reach.\n",
     "  * FAR SIDE (y > 0): the clay plate, centred at `plate_pos`, inset from the far edge.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    The G1-tier manifest keeps the fruits this hand measurably grasps (the fixed-base tier's\n"
     "    findings carry over: the lemon is dependable; near-spheres roll out of the three-finger\n"
     "    pinch) and ALL SEVEN non-fruit distractors, because the crowd is what makes the scene read\n"
     "    like the original — and what makes the identification non-trivial.\n",
     "    The G1-tier manifest keeps one fruit and ALL SEVEN non-fruit distractors, because the crowd\n"
     "    is what makes the scene read like the original — and what makes the identification non-trivial.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: separation from the closest pick-side shoulder is 0.93 m (asserted below). The inset\n"
     "    #: is chosen so a FAR-side stance with the pelvis 0.13 m off the far edge (torso clear)\n"
     "    #: sees the dish centre ~0.38 m from its shoulder, comfortably inside the arm's\n"
     "    #: measured 0.20-0.46 m release annulus; at 0.24 the same stance was 0.44 m out.\n",
     "    #: separation from the closest pick-side shoulder is 0.93 m (asserted below).\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: pick zone on the NEAR side: the fruit line sits 0.28 m ahead of a shoulder whose\n"
     "    #: pelvis stands clear of the near edge — the fixed-base tier's calibrated grasp\n"
     "    #: distance, reproduced at the walking stance.\n",
     "    #: pick zone on the NEAR side.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: G1 tier: ONE fruit — the dependable lemon — and the full distractor set. The task\n"
     "    #: is one complete delivery: pick, carry around the table, place from the far side.\n"
     "    #: Near-spherical fruit stays excluded — measured to roll out of the pinch — and the\n"
     "    #: small lemon (40 mm, under the hand's ~50 mm minimum) with it.\n",
     "    #: G1 tier: ONE fruit (a lemon) and the full distractor set. The task is one complete\n"
     "    #: delivery: pick, carry around the table, place from the far side.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: for the one-lemon tier; kept as the hook for re-admitting the small lemon (x1.3 puts\n"
     "    #: its 40 mm short axis at ~52 mm, inside the hand's measured 50-64 mm window).\n",
     "    #: for the one-lemon tier.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: Reset regime of the fixed-base G1 tier (`_fruits_on_plate_g1_cfg`), whose measured grasp\n"
     "    #: this task inherits: the fruit's long axis along +y (narrow axis at the hand's azimuth\n"
     "    #: 180, dead centre of its reachable band) with +/-30 deg of yaw and +/-15 mm of xy\n"
     "    #: jitter, spawned essentially at rest so the commanded orientation survives settling.\n",
     "    #: Reset regime: the fruit's long axis along +y with +/-30 deg of yaw and +/-15 mm of xy\n"
     "    #: jitter, spawned essentially at rest so the commanded orientation survives settling.\n"),
]
# Phrases that must not survive in the files the scrub edits (a pointer to a solve-calibrated number).
STRIP_GLOB = "robobench/suites/*/configs/envs.py"
NEUTRAL_DOC = '"""(binding notes removed from this image)"""'
BANNED = ("calibrated grasp", "calibrated stance", "release annulus", "pink_probe", "reach envelope", "reach map",
          "azimuth\n    #: 180", "pick radius", "grasp cell", "REORIENT_STUCK", "robot_binding_smoke before")


def code_fingerprint(src: str) -> str:
    """The module's AST without docstrings: a comment or docstring edit leaves it unchanged."""
    tree = ast.parse(src)
    for node in ast.walk(tree):
        body = getattr(node, "body", None)
        if isinstance(body, list) and body and isinstance(body[0], ast.Expr) \
                and isinstance(getattr(body[0], "value", None), ast.Constant) and isinstance(body[0].value.value, str):
            body[0] = ast.Pass()
    return ast.dump(tree)


def strip_comments(src: str) -> str:
    """Remove every comment and replace every docstring but the module's with NEUTRAL_DOC; code unchanged."""
    import io
    import tokenize

    lines = src.splitlines(keepends=True)
    cut: dict[int, int] = {}                       # 1-based row -> column where its comment starts
    for tok in tokenize.generate_tokens(io.StringIO(src).readline):
        if tok.type == tokenize.COMMENT:
            cut[tok.start[0]] = tok.start[1]
    docs = []                                      # (first row, last row, indent) of each non-module docstring
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and node.body:
            b = node.body[0]
            if isinstance(b, ast.Expr) and isinstance(b.value, ast.Constant) and isinstance(b.value.value, str) \
                    and b.value.value != NEUTRAL_DOC.strip('"'):
                docs.append((b.lineno, b.end_lineno, b.col_offset))
    out, skip = [], set()
    for first, last, col in docs:
        skip.update(range(first + 1, last + 1))
    doc_first = {first: col for first, _last, col in docs}
    for i, line in enumerate(lines, start=1):
        if i in skip:
            continue
        if i in doc_first:
            out.append(" " * doc_first[i] + NEUTRAL_DOC + "\n")
            continue
        if i in cut:
            head = line[:cut[i]].rstrip()
            if not head:
                continue                           # a comment-only line goes entirely
            line = head + "\n"
        out.append(line)
    return "".join(out)


def plan(root: Path) -> list[str]:
    """What a run would still change (empty after a scrub)."""
    todo = [f"remove {p.name}" for p in sorted(root.iterdir()) if p.name not in KEEP_TOP]
    suites = root / "robobench" / "suites"
    for s in sorted(suites.iterdir()) if suites.is_dir() else []:
        if s.is_dir() and s.name in DROP_SUITES:
            todo.append(f"remove suites/{s.name}")
        for d in DROP_SUITE_DIRS:
            if (s / d).exists():
                todo.append(f"remove suites/{s.name}/{d}")
    for rel, old, _new in EDITS:
        if old in (root / rel).read_text():
            todo.append(f"edit {rel}")
    for path in sorted(root.glob(STRIP_GLOB)):
        src = path.read_text()
        if strip_comments(src) != src:
            todo.append(f"strip {path.relative_to(root)}")
    todo += [f"pycache {p}" for p in root.rglob("__pycache__")]
    return todo


def scrub(root: Path) -> None:
    for p in root.rglob("__pycache__"):           # bytecode built before the removal keeps the old code
        shutil.rmtree(p, ignore_errors=True)
    for p in sorted(root.iterdir()):
        if p.name not in KEEP_TOP:
            shutil.rmtree(p) if p.is_dir() and not p.is_symlink() else p.unlink()
    suites = root / "robobench" / "suites"
    for s in sorted(suites.iterdir()):
        if not s.is_dir():
            continue
        if s.name in DROP_SUITES:
            shutil.rmtree(s)
            continue
        for d in DROP_SUITE_DIRS:
            if (s / d).exists():
                shutil.rmtree(s / d)
    for rel, old, new in EDITS:
        path = root / rel
        src = path.read_text()
        n = src.count(old)
        if n == 0:
            continue                               # already scrubbed (idempotent)
        if n != 1:
            raise SystemExit(f"scrub: {rel}: expected the block once, found {n}")
        out = src.replace(old, new)
        if code_fingerprint(out) != code_fingerprint(src):
            raise SystemExit(f"scrub: {rel}: an edit changed code, not only comments")
        path.write_text(out)
    for path in sorted(root.glob(STRIP_GLOB)):
        src = path.read_text()
        out = strip_comments(src)
        if code_fingerprint(out) != code_fingerprint(src):
            raise SystemExit(f"scrub: {path}: stripping changed code")
        path.write_text(out)


def check(root: Path) -> list[str]:
    bad = plan(root)
    for rel in sorted({e[0] for e in EDITS} | {str(p.relative_to(root)) for p in root.glob(STRIP_GLOB)}):
        txt = (root / rel).read_text()
        bad += [f"banned phrase {b!r} in {rel}" for b in BANNED if b in txt]
    # the suites still import (registration only; no simulator): every scene module the tasks need is present
    for rel in ("robobench/suites/assembly/scenes/bulb_assembly.py", "robobench/suites/puzzle/scenes/stack_blocks.py",
                "robobench/suites/locomanip/scenes/fruit_delivery.py", "robobench/suites/packing/scenes/fruits_on_plate.py"):
        if not (root / rel).is_file():
            bad.append(f"missing {rel}")
    return bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--check", action="store_true", help="change nothing; fail if a rule would still apply")
    a = ap.parse_args()
    root = a.root.resolve()
    if not a.check:
        # a fresh checkout must still carry every block (else upstream moved and the rules need a look)
        for rel, old, _new in EDITS:
            if old not in (root / rel).read_text() and _new not in (root / rel).read_text():
                raise SystemExit(f"scrub: {rel}: neither the original nor the scrubbed block is present")
        scrub(root)
    bad = check(root)
    for b in bad:
        print(f"scrub check: {b}", file=sys.stderr)
    print(f"scrub: {'FAILED' if bad else 'ok'} ({len(EDITS)} scene-comment edits, "
          f"{len(list(root.glob(STRIP_GLOB)))} binding files stripped, graders/smokes removed)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
