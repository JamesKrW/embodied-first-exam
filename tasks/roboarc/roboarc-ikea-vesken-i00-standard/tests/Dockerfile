# Separate verifier sandbox: grades the sim service's own artifacts (/data/final.json, the live episode's verdict);
# needs nothing but python (no simulator: nothing is replayed).
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:ikea-mujoco-files-0.3.0@sha256:8396e542e144afe82fb30773316193b23d5c942e05363888f857bccdf163d7fa /standard/tests/grade.py /tests/grade.py
RUN chmod +x /tests/test.sh && mkdir -p /logs/verifier
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:69dcd5dd5dff08cfc35fc891e2563a374b6c4cc156be84ee88b9db456662f726 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
