# Separate verifier sandbox: grades the sim service's own artifacts; needs nothing but Python.
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:libero-max-files-0.3.1@sha256:cf5dc5cd5ee3f6b84375f7ed906ac02da2b81487bef3bd7b78272104c69cb2aa /standard/tests/grade.py /tests/grade.py
RUN chmod +x /tests/test.sh
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
