# Separate verifier sandbox (PROTOCOL §2.6): the same image with the verifier's own copies of the tooling and the frozen
# instance. Nothing the agent did in its container reaches this one: Harbor uploads only the declared artifacts.
FROM ghcr.io/mll-lab-nu/efe:myosuite-0.1.1
COPY instance.npz test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:myosuite-files-0.2.0@sha256:8f4861ea442bf2d6891985fd96928e9ba02f7e1439b1c4e4c7aeef0082c54149 /privileged/environment/rcb_myosuite.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.0@sha256:53154417782408df8309194971b08b18c3682d2b337e2e3b2eb4c88fe025ab85 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
