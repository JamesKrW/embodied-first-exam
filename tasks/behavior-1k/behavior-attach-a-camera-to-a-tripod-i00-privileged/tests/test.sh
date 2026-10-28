#!/bin/bash
# Verifier: replay /app/output/trajectory.npz from the frozen instance (the tests/ copy) in a fresh process.
# Writes /logs/verifier/reward.txt (1/0) and reward.json (numeric fields, see docs/protocol/README.md).
rm -rf /logs/verifier/*   # verdicts from an earlier run never survive (docs/protocol/README.md Appendix H.1)
trap 'p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py verify' EXIT   # resource limits, after the grader (docs/protocol/README.md §6)
p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py sanitize /app/output || true   # the agent's files are untrusted input: no link out of /app/output, no FIFO or device (docs/protocol/README.md Appendix H.1)
mkdir -p /logs/verifier
python /tests/grade.py --family attach_a_camera_to_a_tripod --instance /tests/instance.json --traj /app/output/trajectory.npz --repeat 2 \
  2> /logs/verifier/grade.log
