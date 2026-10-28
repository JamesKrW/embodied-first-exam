# Separate verifier sandbox: grades the sim service's own artifacts (/data/final.json, the live episode's verdict);
# needs nothing but python (no simulator: nothing is replayed).
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:ikea-mujoco-files-0.3.0@sha256:8396e542e144afe82fb30773316193b23d5c942e05363888f857bccdf163d7fa /standard/tests/grade.py /tests/grade.py
RUN chmod +x /tests/test.sh && mkdir -p /logs/verifier
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:9d09fa2fe44ee6af5ccdff49827f005934c053f5dce34044e036c9facc3b343a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
