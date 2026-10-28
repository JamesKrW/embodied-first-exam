# Separate verifier sandbox: grades the sim service's own artifacts; needs nothing but Python (the sim service judged the live episode).
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:ese-files-0.6.1@sha256:9e4d43abde2f33076467f09c035fec44d15ea4c38a0d6fe8ebf8dbc76a2db219 /realtime/grade.py /tests/grade.py
COPY --from=ghcr.io/mll-lab-nu/efe:ese-files-0.6.1@sha256:9e4d43abde2f33076467f09c035fec44d15ea4c38a0d6fe8ebf8dbc76a2db219 /standard/grade.py /tests/grade_ese.py
RUN chmod +x /tests/test.sh
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:76b33f25e81b0e58f7c1f0b0e1e7e031e067b11da79dc5bc832a82ac53b4f7a2 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
