#!/bin/bash
# Verifier: grade every submitted checkpoint (/app/output/submissions/) and the final /app/output/solution/ against the
# frozen instance (the tests/ copy), each in a fresh simulator; the best one is the result. Writes
# /logs/verifier/reward.txt (1/0), reward.json (numbers only), results.json (every candidate), trajectory.npz and the video.
rm -rf /logs/verifier/* /logs/verifier/.[!.]*   # verdicts from an earlier run never survive (docs/protocol/README.md §2.6)
trap 'p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py verify' EXIT   # resource limits, after the grader (docs/protocol/README.md §2.10)
p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py sanitize /app/output || true   # the agent's files are untrusted input: no link out of /app/output, no FIFO or device (docs/protocol/README.md §2.6)
mkdir -p /logs/verifier
chmod 700 /logs/verifier                        # the solution process (nobody) can neither read nor write the logs
python /tests/grade.py --instance /tests/instance.json --grader /tests/grader/push_shapes.py 2> /logs/verifier/grade.log
