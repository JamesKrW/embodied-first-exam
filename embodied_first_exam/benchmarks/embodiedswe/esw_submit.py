#!/usr/bin/env python3
"""`submit "note"`: freeze a checkpoint of /app/output/solution/ (EmbodiedSWE's harbor `submit`, adapted to this
container's paths). The copy goes to /app/output/submissions/NNNNNN/solution/ with a receipt, submission.json
(schema_version, created_at, note, files = {path: sha256} of every regular file, sha256 of that inventory). Concurrent
submits are serialized with flock. At most 64 checkpoints; the final /app/output/solution/ is graded as well. Every
distinct checkpoint is graded in a fresh simulator; the best one is the result (tests/grade.py).

In the image at /opt/efe/esw_submit.py, linked as /usr/local/bin/submit. Standard library only.
"""
import argparse
import fcntl
import hashlib
import json
import os
import shutil
import stat
import tempfile
from datetime import datetime, timezone
from pathlib import Path

SOLUTION = Path("/app/output/solution")
SUBMISSIONS = Path("/app/output/submissions")
LIMIT = 64
MAX_FILES, MAX_BYTES = 20_000, 2 << 30           # the grader's limits on a solution folder


def inventory(root: Path) -> dict[str, str]:
    """{relative path: sha256} of every regular file under `root`; a link or a special file is an error."""
    if root.is_symlink() or not root.is_dir():
        raise ValueError(f"expected a real directory: {root}")
    out, size = {}, 0
    for path in sorted(root.rglob("*")):
        mode = path.lstat().st_mode
        if stat.S_ISDIR(mode):
            continue
        if not stat.S_ISREG(mode):
            raise ValueError(f"symbolic link or special file not supported: {path}")
        size += path.lstat().st_size
        if len(out) >= MAX_FILES or size > MAX_BYTES:
            raise ValueError("solution larger than 20,000 files or 2 GiB")
        with path.open("rb") as f:
            out[path.relative_to(root).as_posix()] = hashlib.file_digest(f, "sha256").hexdigest()
    return out


def content_hash(files: dict[str, str]) -> str:
    return hashlib.sha256(json.dumps(files, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def snapshot(solution: Path, submissions: Path, note: str, limit: int = LIMIT) -> Path:
    if not (solution / "solve.py").is_file():
        raise ValueError(f"no solve.py in {solution}")
    submissions.mkdir(parents=True, exist_ok=True)
    with (submissions / ".submit.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        existing = [p for p in submissions.iterdir() if p.name.isdecimal()]
        if len(existing) >= limit:
            raise ValueError(f"submission limit ({limit}) reached; the final solution/ is still graded")
        number = max((int(p.name) for p in existing), default=0) + 1
        target = submissions / f"{number:06d}"
        before = inventory(solution)
        temp = Path(tempfile.mkdtemp(prefix=".pending-", dir=submissions))
        try:
            shutil.copytree(solution, temp / "solution", symlinks=True)
            copied = inventory(temp / "solution")
            if copied != before or inventory(solution) != before:
                raise ValueError("solution/ changed during submit; stop writing to it and submit again")
            (temp / "submission.json").write_text(json.dumps({
                "schema_version": 1, "created_at": datetime.now(timezone.utc).isoformat(), "note": note[:2000],
                "files": copied, "sha256": content_hash(copied)}, indent=2, sort_keys=True) + "\n")
            os.chmod(temp, 0o755)
            temp.rename(target)
        finally:
            if temp.exists():
                shutil.rmtree(temp)
    return target


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("note", nargs="*", help="a short description of this checkpoint")
    args = parser.parse_args()
    try:
        path = snapshot(SOLUTION, SUBMISSIONS, " ".join(args.note))
    except (OSError, ValueError) as exc:
        parser.exit(1, f"submit: {exc}\n")
    print(f"Submitted {path.name} ({path}). Continue working; the final solution/ is also graded.")


if __name__ == "__main__":
    main()
