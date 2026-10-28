# Separate verifier sandbox: grades the sim service's own artifacts; needs nothing but python (the shared eai-standard
# client base, embodied_first_exam/standard/client.env; it installs nothing). The grader comes from the files-only image
# eai-behavior-files (images/behavior-files/, pinned by digest).
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY test.sh /tests/test.sh
COPY --from=ghcr.io/mll-lab-nu/efe:behavior-files-0.3.0@sha256:7f5258c606013691940401161b8be39cd183bd36219479aaa10d4e4bfe42d175 /standard/grade.py /tests/grade.py
RUN chmod +x /tests/test.sh
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:3209fb078999b179d0c2b249822172ecd8296cb4ee6f36389198b667ca879a0d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
