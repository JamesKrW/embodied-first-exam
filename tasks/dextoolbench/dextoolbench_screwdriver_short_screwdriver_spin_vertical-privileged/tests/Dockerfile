# Separate verifier sandbox (PROTOCOL §4): the MuJoCo image with the verifier's own copies of the tooling and the
# frozen instance; only the declared artifacts (/app/output/) arrive from the agent.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dextoolbench-files-0.1.2@sha256:7350a45dab90f7d8132e0ed981562e44c809e9539c53fc4d0355e8bd0186084c /privileged/environment/rcb_dtb.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
