#!/usr/bin/env python3
"""fetch_isaac_assets.py: mirror the Isaac Sim 5.1 cloud assets EmbodiedSWE's scenes read, with their full USD dependency
closure, for an offline image.

Some EmbodiedSWE scenes (puzzle.spatula, puzzle.syringe, cutting.slice, cutting.dice) spawn Isaac Lab's default
`sim_utils.GroundPlaneCfg()`, whose `usd_path` is `f"{ISAAC_NUCLEUS_DIR}/Environments/Grid/default_environment.usd"`, with
ISAAC_NUCLEUS_DIR = carb setting `/persistent/isaac/asset_root/cloud` + "/Isaac" (isaaclab.utils.assets, read once at
import). The image keeps a mirror of exactly those files under MIRROR (= the bucket's key layout) and sets the asset
root to MIRROR; the scene code stays unchanged.

Usage (needs network and `pxr`, e.g. `pip install usd-core`):
  fetch_isaac_assets.py --out DIR [--manifest ASSETS.json] [--dockerfile-lines FILE]
      download the closure of SEEDS into DIR/<key>, write the manifest (path, url, sha256, bytes) and, if asked, the
      Dockerfile `ADD --checksum` lines that put each file at MIRROR/<key>.
  fetch_isaac_assets.py --check DIR --manifest ASSETS.json
      verify DIR holds exactly the manifest's files with their sha256 (no network, no pxr).

Closure rule: every layer is opened with pxr; its sublayers, references, payloads and asset-valued attributes
(UsdUtils.ExtractExternalReferences) are resolved against the layer's URL and fetched; fetched USD layers are walked
the same way. A bare MDL module name with no directory (e.g. `OmniPBR.mdl`) is a Kit core MDL resolved from Kit's own
MDL search path (shipped in the image's Isaac Sim wheels), so it is listed as `builtin` and not fetched.
"""
import argparse
import hashlib
import json
import os
import posixpath
import sys
import urllib.request

BUCKET = "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
ROOT_KEY = "Assets/Isaac/5.1"                         # the asset_root's key in the bucket (Isaac Lab 2.3.2 kit files)
MIRROR = "/opt/isaac_assets/" + ROOT_KEY             # in the image: /persistent/isaac/asset_root/{default,cloud,nvidia}
SEEDS = ["Isaac/Environments/Grid/default_environment.usd"]   # relative to the asset root
USD_EXT = (".usd", ".usda", ".usdc", ".usdz")


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for b in iter(lambda: f.read(1 << 20), b""):
            h.update(b)
    return h.hexdigest()


def download(url, dst):
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    tmp = dst + ".part"
    with urllib.request.urlopen(url, timeout=60) as r, open(tmp, "wb") as f:
        if r.status != 200:
            raise RuntimeError(f"{url}: HTTP {r.status}")
        while True:
            b = r.read(1 << 20)
            if not b:
                break
            f.write(b)
    os.replace(tmp, dst)


def deps(local_path):
    from pxr import UsdUtils
    sub, refs, payloads = UsdUtils.ExtractExternalReferences(local_path)
    return [("sublayer", p) for p in sub] + [("reference", p) for p in refs] + [("payload", p) for p in payloads]


def fetch(out, manifest_path, lines_path):
    files, builtin, seen = {}, {}, set()
    queue = [(k, "seed", None) for k in SEEDS]
    while queue:
        rel, kind, parent = queue.pop(0)
        rel = posixpath.normpath(rel)
        if rel in seen:
            continue
        seen.add(rel)
        url = f"{BUCKET}/{ROOT_KEY}/{rel}"
        dst = os.path.join(out, rel)
        download(url, dst)
        files[rel] = {"path": f"{MIRROR}/{rel}", "url": url, "sha256": sha256(dst), "bytes": os.path.getsize(dst),
                      "kind": kind, "referenced_by": parent}
        print(f"fetched {rel} ({files[rel]['bytes']} B, {kind} of {parent})", file=sys.stderr)
        if rel.lower().endswith(USD_EXT):
            for k, p in deps(dst):
                if "://" in p or p.startswith("omniverse:"):
                    raise SystemExit(f"{rel}: absolute URL dependency {p!r}: extend the mirror rule")
                if p.lower().endswith(".mdl") and "/" not in p:
                    builtin.setdefault(p, []).append(rel)
                    continue
                if p.startswith("/"):
                    raise SystemExit(f"{rel}: absolute path dependency {p!r}")
                queue.append((posixpath.join(posixpath.dirname(rel), p), k, rel))
    man = {"bucket": BUCKET, "root_key": ROOT_KEY, "mirror": MIRROR, "seeds": SEEDS,
           "files": [dict(rel=r, **files[r]) for r in sorted(files)],
           "builtin_mdl": {k: sorted(set(v)) for k, v in sorted(builtin.items())},
           "total_bytes": sum(f["bytes"] for f in files.values())}
    if manifest_path:
        with open(manifest_path, "w") as f:
            json.dump(man, f, indent=1)
            f.write("\n")
    if lines_path:
        with open(lines_path, "w") as f:
            for e in man["files"]:
                f.write(f"ADD --chmod=644 --checksum=sha256:{e['sha256']} {e['url']} {e['path']}\n")
    print(json.dumps({"files": len(files), "bytes": man["total_bytes"], "builtin_mdl": sorted(builtin)}))


def check(root, manifest_path):
    man = json.load(open(manifest_path))
    want = {e["rel"]: e for e in man["files"]}
    have = set()
    for d, _, fs in os.walk(root):
        for n in fs:
            have.add(os.path.relpath(os.path.join(d, n), root).replace(os.sep, "/"))
    bad = sorted(have ^ set(want))
    for r in sorted(set(want) & have):
        p = os.path.join(root, r)
        if sha256(p) != want[r]["sha256"] or os.path.getsize(p) != want[r]["bytes"]:
            bad.append(r)
    if bad:
        raise SystemExit(f"mirror mismatch: {bad}")
    print(f"mirror ok: {len(want)} files under {root}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--out")
    ap.add_argument("--check")
    ap.add_argument("--manifest")
    ap.add_argument("--dockerfile-lines")
    a = ap.parse_args()
    if a.check:
        check(a.check, a.manifest)
    elif a.out:
        fetch(a.out, a.manifest, a.dockerfile_lines)
    else:
        ap.error("--out or --check")
