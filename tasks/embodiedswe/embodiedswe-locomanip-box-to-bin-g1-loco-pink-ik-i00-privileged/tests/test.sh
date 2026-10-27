#!/bin/bash
# Verifier: run the delivered program /app/output/solution/ against the frozen instance (the tests/ copy), then re-execute
# its recorded actions in a fresh process. Writes /logs/verifier/reward.txt (1/0) and reward.json (numbers only).
rm -rf /logs/verifier/* /logs/verifier/.[!.]*   # verdicts from an earlier run never survive (docs/protocol/README.md §2.6)
trap 'p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py verify' EXIT   # resource limits, after the grader (docs/protocol/README.md §2.10)
p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py sanitize /app/output || true   # the agent's files are untrusted input: no link out of /app/output, no FIFO or device (docs/protocol/README.md §2.6)
mkdir -p /logs/verifier
chmod 700 /logs/verifier                        # the solution process (nobody) can neither read nor write the logs
python /tests/grade.py --instance /tests/instance.json --grader /tests/grader/box_to_bin.py 2> /logs/verifier/grade.log
