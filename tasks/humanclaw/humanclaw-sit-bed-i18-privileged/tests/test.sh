#!/bin/bash
# Verifier: replay /app/output/trajectory.npz twice, each in a fresh process, from the frozen instance (the tests/ copy).
# Writes /logs/verifier/reward.txt (1/0), reward.json (numbers only) and replay.mp4 (docs/protocol/README.md §2.6).
rm -rf /logs/verifier/*
trap 'p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py verify' EXIT   # resource limits, after the grader (docs/protocol/README.md §2.10)
p=$(command -v python3 || command -v python); "$p" /tests/rcb_limits.py sanitize /app/output || true   # the agent's files are untrusted input: no link out of /app/output, no FIFO or device (docs/protocol/README.md §2.6)
mkdir -p /logs/verifier
python /tests/grade.py --instance /tests/instance.json --traj /app/output/trajectory.npz --repeat 2 2> /logs/verifier/grade.log
