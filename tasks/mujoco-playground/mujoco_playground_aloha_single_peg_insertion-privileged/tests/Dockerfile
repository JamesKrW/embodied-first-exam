# Separate verifier sandbox (PROTOCOL §4): the MuJoCo image with the verifier's own copies of the tooling and the
# frozen instance; only the declared artifacts (/app/output/) arrive from the agent.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.4.1@sha256:edc4ee147fcda819b1ed3df6faa7277aca673f75f52bac2cf34f3dd9357c4e9b /privileged/environment/rcb_aloha.py /privileged/tests/variants/72df271117/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:3209fb078999b179d0c2b249822172ecd8296cb4ee6f36389198b667ca879a0d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
