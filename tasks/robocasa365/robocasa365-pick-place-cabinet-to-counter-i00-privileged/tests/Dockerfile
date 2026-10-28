# Separate verifier sandbox: the same image with the verifier's own copies of the tooling and the frozen instance;
# only the declared artifacts (/app/output/) arrive from the agent.
FROM ghcr.io/mll-lab-nu/efe:robocasa365-0.1.0
COPY test.sh instance.npz /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa365-files-0.4.1@sha256:144bd5d53a1ac662a902702fbcefced0cb40d19183965ce49515a9b441deadfe /privileged/environment/casa_tool.py /tests/casa_tool.py
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa-files-0.3.0@sha256:cc2593ceffa46f83daf8c59051bbada963d9830673a607522c1dd288082683bf /privileged/tests/grade.py /tests/grade.py
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:935a38662d5158d6091473c718a3c4c83fa43656e3548d4f84008dfc02678987 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
