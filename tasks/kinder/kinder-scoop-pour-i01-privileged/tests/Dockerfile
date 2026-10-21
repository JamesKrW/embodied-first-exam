# Separate verifier sandbox (docs/protocol/README.md §2.6): the same base image, plus the verifier's own copies of the
# tooling and the frozen instance. Nothing the agent did in its container reaches this one: Harbor uploads only the
# declared artifacts (/app/output/).
FROM ghcr.io/mll-lab-nu/efe:kinder-0.1.6
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:kinder-files-0.3.0@sha256:267ea96a2c43a8e8b9a00ab7c74c3b919be238724aad0b4f85f5f37454f6c2ef /privileged/environment/rcb_kinder.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
