#!/usr/bin/env python3
"""Stage one RoboTangle task's frozen instance from the image into a task directory, and check it against the task's
runtime binding. Standard library only.

usage: python /opt/robotangle/stage_instance.py <base> <dest> [--binding <runtime_binding.json>] [--seed-file <seed>] [--purge]

The image (images/robotangle/Dockerfile) carries every task's frozen instance, instance.json + instance_assets/, under
/opt/robotangle/instances/<base>/. The files come from the private Hugging Face dataset eai-v2-internal/robotangle-assets
at the revision pinned in scripts/robotangle/assets.json, and the image build checks each file's sha256 there. A task
directory holds only runtime_binding.json, which names the instance's canonical sha256 and the sha256 of every asset.
This script:
  - copies the instance into <dest>, next to <dest>/runtime_binding.json (--binding copies that file there first);
  - checks both against the binding, by the rule of topogym.simulator.frozen_binding.check_inputs, which every session
    applies again at start;
  - with --purge, removes /opt/robotangle/instances afterwards, so a task image keeps only its own instance.
Seeds (docs/RUNNING.md "Seeds"): a task that allows several seeds has a file `seed` (the integer the runner chose; the
committed one says the default, 0) and, for every other seed S, `runtime_binding.s<S>.json` next to its
runtime_binding.json. With --seed-file, seed 0 is the base's own instance as above; a seed S > 0 is staged from
/opt/robotangle/instances/<base>/seeds/<S>/ against runtime_binding.s<S>.json (from the directory of --binding, else
<dest>), which becomes <dest>/runtime_binding.json. A seed without its instance or binding, or a seed file that is not
a non-negative integer, fails: nothing falls back to another seed. Every runtime_binding.s*.json is removed from <dest>
afterwards.
Privileged tasks run it in environment/Dockerfile and tests/Dockerfile. Standard tasks run it in the sim service's
entrypoint, before `sh /srv/run.sh`, and RCB_INSTANCE points into <dest>.
"""
import hashlib
import json
import os
import pathlib
import shutil
import sys

SRC = pathlib.Path("/opt/robotangle/instances")


def file_sha(p: pathlib.Path) -> str:
    h = hashlib.sha256()
    with p.open("rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def option(argv: list[str], name: str) -> pathlib.Path | None:
    if name not in argv:
        return None
    i = argv.index(name)
    v = pathlib.Path(argv[i + 1])
    del argv[i:i + 2]
    return v


def read_seed(f: pathlib.Path) -> int:
    t = f.read_text().strip()
    if not t.isdigit():
        raise ValueError(f"seed file {f} says {t!r}, not a non-negative integer")
    return int(t)


def main(argv: list[str]) -> int:
    purge = "--purge" in argv
    argv = [a for a in argv if a != "--purge"]
    binding = option(argv, "--binding")
    seed_file = option(argv, "--seed-file")
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    base, dest = argv[0], pathlib.Path(argv[1])
    try:
        seed = 0 if seed_file is None else read_seed(seed_file)
    except (OSError, ValueError) as e:
        print(f"FAIL stage_instance {base}: {e}", file=sys.stderr)
        return 1
    src = SRC / base if seed == 0 else SRC / base / "seeds" / str(seed)
    if not (src / "instance.json").is_file():
        print(f"FAIL stage_instance: no frozen instance for {base}" + (f" seed {seed}" if seed else "")
              + f" in this image ({src})", file=sys.stderr)
        return 1
    dest.mkdir(parents=True, exist_ok=True)
    if seed:                                   # the seed's own binding, never another seed's
        bdir = binding.parent if binding is not None else dest
        sb = bdir / f"runtime_binding.s{seed}.json"
        if not sb.is_file():
            print(f"FAIL stage_instance {base}: seed {seed} has no binding {sb}", file=sys.stderr)
            return 1
        shutil.copyfile(sb, dest / "runtime_binding.json")
    elif binding is not None:
        shutil.copyfile(binding, dest / "runtime_binding.json")
    for p in dest.glob("runtime_binding.s*.json"):
        p.unlink()
    lock = json.loads((dest / "runtime_binding.json").read_text())
    shutil.copyfile(src / "instance.json", dest / "instance.json")
    if (dest / "instance_assets").exists():
        shutil.rmtree(dest / "instance_assets")
    shutil.copytree(src / "instance_assets", dest / "instance_assets", symlinks=True)
    bad = []
    inst = json.loads((dest / "instance.json").read_text())
    canon = hashlib.sha256(json.dumps(inst, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    if canon != lock["instance_sha256"]:
        bad.append(f"instance.json {canon[:12]} is not the bound instance {lock['instance_sha256'][:12]}")
    assets = {}
    for p in sorted((dest / "instance_assets").rglob("*")):
        if "__pycache__" in p.parts:
            continue
        if p.is_symlink():
            bad.append(f"symlink in the instance assets: {p}")
        elif p.is_file():
            assets["instance_assets/" + p.relative_to(dest / "instance_assets").as_posix()] = file_sha(p)
    if assets != lock["asset_files"]:
        diff = sorted(set(assets.items()) ^ set(lock["asset_files"].items()))
        bad.append(f"instance_assets differ from the binding ({len(diff)} entries, e.g. {diff[:2]})")
    for p in [dest / "instance.json", *(dest / "instance_assets").rglob("*")]:
        os.chmod(p, 0o755 if p.is_dir() else 0o644)
    for b in bad:
        print(f"FAIL stage_instance {base}: {b}", file=sys.stderr)
    if bad:
        return 1
    if purge:
        shutil.rmtree(SRC)
    print(f"ok stage_instance {base}" + (f" seed {seed}" if seed_file is not None else "")
          + f": instance {canon[:12]}, {len(assets)} asset files -> {dest}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
