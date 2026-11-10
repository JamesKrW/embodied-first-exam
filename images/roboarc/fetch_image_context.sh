#!/usr/bin/env bash
# Prepare the RoboARC image build context images/roboarc/context/ (images/roboarc/BUILD_CONTEXT):
#   - the RoboARC repository's source/ at the release commit (images/roboarc/release.json release_commit): the simulator
#     source, the robot assets and the image helpers images/roboarc/Dockerfile copies;
#   - instances/: every task's frozen MuJoCo instance and its appearance seeds, every file checked against
#     images/roboarc/assets.json: from the private HF dataset at the pinned revision, or from a local copy of the
#     release's instance tree (RCB_ROBOARC_INSTANCES=<dir>, holding instances/<base>/...).
# Needs a clone of the RoboARC repository (RCB_ROBOARC_REPO) and read access to the private RoboARC asset dataset
# (organization members only; to run the tasks, pull the published image instead): `hf auth login`, or HF_TOKEN in the
# environment (never printed).
# usage: RCB_ROBOARC_REPO=<clone> [RCB_ROBOARC_INSTANCES=<dir>] images/roboarc/fetch_image_context.sh [<download dir>]
#        (download dir default: $RCB_ROBOTANGLE_ASSETS or ~/.cache/robotangle-assets)
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; HERE="$ROOT/images/roboarc"; OUT="$ROOT/images/roboarc/context"
DL="${RCB_ROBOARC_INSTANCES:-${1:-${RCB_ROBOTANGLE_ASSETS:-$HOME/.cache/robotangle-assets}}}"
REPO="${RCB_ROBOARC_REPO:?set RCB_ROBOARC_REPO to a clone of the RoboARC repository}"
COMMIT="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["release_commit"])' "$HERE/release.json")"
python3 "$HERE/assets.py" "$([ -n "${RCB_ROBOARC_INSTANCES:-}" ] && echo check || echo fetch)" "$DL"
rm -rf "$OUT"; mkdir -p "$OUT"
git -C "$REPO" archive "$COMMIT" source | tar -x -C "$OUT" --strip-components=1
mkdir -p "$OUT/instances"
python3 - "$HERE/assets.json" "$DL" "$OUT/instances" <<'PY'
import json, pathlib, shutil, sys
pins, dl, out = json.load(open(sys.argv[1])), pathlib.Path(sys.argv[2]), pathlib.Path(sys.argv[3])
for base in pins["instances"]:
    shutil.copytree(dl / "instances" / base, out / base)
print(f"context: {len(pins['instances'])} instances")
PY
