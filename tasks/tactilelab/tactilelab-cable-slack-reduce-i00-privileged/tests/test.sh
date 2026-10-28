#!/bin/bash
# Verifier: replay /app/output/trajectory.npz from the frozen instance (the tests/ copy) in 2 fresh processes,
# then render the video in one more (after the verdict).
# Writes /logs/verifier/reward.txt (1/0) and reward.json (numeric fields, see docs/protocol/README.md).
# Harbor also mounts /logs/verifier into the agent's container: start from an empty directory, so nothing the agent
# left there (a planted reward, a symlink) can count. rm never follows symlinks.
rm -rf /logs/verifier/*   # verdicts from an earlier run never survive (docs/protocol/README.md §2.6)
trap 'p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py verify' EXIT   # resource limits, after the grader (docs/protocol/README.md §6)
p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py sanitize /app/output || true   # the agent's files are untrusted input: no link out of /app/output, no FIFO or device (docs/protocol/README.md Appendix H.1)
mkdir -p /logs/verifier
python /tests/grade.py --family cable_slack_reduce --instance /tests/instance.json --traj /app/output/trajectory.npz --repeat 2 \
  --require-identical 1 --replay-limit-s 3000 --video-limit-s 3000 --max-rows 4800 2> /logs/verifier/grade.log
