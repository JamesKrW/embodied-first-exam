# Separate verifier sandbox: grades the sim service's own artifacts; needs nothing but python (the shared client base).
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY test.sh /tests/test.sh
COPY --from=ghcr.io/mll-lab-nu/efe:dexjoco-files-0.2.0@sha256:6ba4f565aa356ec3a2a3ccae648a8e6047aa69086353be86de95041041e0e73f /standard/tests/grade.py /tests/grade.py
RUN chmod +x /tests/test.sh
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:69dcd5dd5dff08cfc35fc891e2563a374b6c4cc156be84ee88b9db456662f726 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
