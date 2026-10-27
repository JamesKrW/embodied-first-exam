#!/bin/bash
# Reference solution: the navmesh oracle's actions (shortest path to the target, turn-and-walk, then sit for an
# interaction episode, then stop), found closed-loop with the privileged tools and replayed here.
set -e
mkdir -p /app/output
python - <<'PY'
import json, numpy as np
a = np.array(json.load(open("/solution/actions.json")), dtype=np.float64).reshape(-1, 2)
np.savez("/app/output/trajectory.npz", actions=a)
PY
