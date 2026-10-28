# Separate verifier sandbox (docs/protocol/README.md §2.6): the same base image, plus the verifier's own copies of the
# tooling and the frozen instance. Nothing the agent did in its container reaches this one: Harbor uploads only the
# declared artifacts (/app/output/).
FROM ghcr.io/mll-lab-nu/efe:kinder-0.1.6
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:kinder-files-0.3.1@sha256:28ae8c1aebd73de12ba242d818f665b2e3f8eb9df28fe8c9f52bea45744ec116 /privileged/environment/rcb_kinder.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:76b33f25e81b0e58f7c1f0b0e1e7e031e067b11da79dc5bc832a82ac53b4f7a2 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
