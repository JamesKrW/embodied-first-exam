# Separate verifier sandbox: the same image with the verifier's own copies of the tooling and the frozen instance;
# only the declared artifacts (/app/output/) arrive from the agent.
FROM ghcr.io/mll-lab-nu/efe:libero-max-0.1.1
COPY test.sh instance.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:libero-max-files-0.3.1@sha256:cf5dc5cd5ee3f6b84375f7ed906ac02da2b81487bef3bd7b78272104c69cb2aa /privileged/environment/lmx_tool.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests:/opt/libero-max/overlay:/opt/libero-max/src
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:3209fb078999b179d0c2b249822172ecd8296cb4ee6f36389198b667ca879a0d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
