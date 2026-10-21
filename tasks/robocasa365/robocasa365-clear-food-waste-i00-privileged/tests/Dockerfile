# Separate verifier sandbox: the same image with the verifier's own copies of the tooling and the frozen instance;
# only the declared artifacts (/app/output/) arrive from the agent.
FROM ghcr.io/mll-lab-nu/efe:robocasa365-0.1.0
COPY test.sh instance.npz /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa365-files-0.3.0@sha256:7239869e98f61c352b412536a6e2a465a793aa7fffadc7b31858091347cfcebb /privileged/environment/casa_tool.py /tests/casa_tool.py
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa-files-0.3.0@sha256:cc2593ceffa46f83daf8c59051bbada963d9830673a607522c1dd288082683bf /privileged/tests/grade.py /tests/grade.py
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
