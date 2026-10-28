# Separate verifier sandbox (PROTOCOL §4): the same image with the verifier's own copies of the tooling and the frozen
# instance. Nothing the agent did in its container reaches this one: Harbor uploads only the declared artifacts.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.3
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.4.1@sha256:edc4ee147fcda819b1ed3df6faa7277aca673f75f52bac2cf34f3dd9357c4e9b /privileged/environment/rcb_quadruped.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:9d09fa2fe44ee6af5ccdff49827f005934c053f5dce34044e036c9facc3b343a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
