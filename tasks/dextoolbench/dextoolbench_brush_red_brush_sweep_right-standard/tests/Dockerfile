# Separate verifier sandbox: grades the sim service's own artifacts; needs nothing but python.
FROM docker.io/library/python:3.12-slim
COPY test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dextoolbench-files-0.1.2@sha256:7350a45dab90f7d8132e0ed981562e44c809e9539c53fc4d0355e8bd0186084c /standard/tests/grade.py /tests/grade.py
RUN chmod +x /tests/test.sh
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
