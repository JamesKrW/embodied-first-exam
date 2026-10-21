# Separate verifier sandbox (PROTOCOL §4): the same image with the verifier's own copies of the tooling and the frozen
# instance. Nothing the agent did in its container reaches this one: Harbor uploads only the declared artifacts.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.3
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.3.0@sha256:e46bd0d06598e0f0ccbe6d9f96f2a999cd8117afc13b6808d2a02d5ed1e8cb20 /privileged/environment/rcb_quadruped.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.0@sha256:53154417782408df8309194971b08b18c3682d2b337e2e3b2eb4c88fe025ab85 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
