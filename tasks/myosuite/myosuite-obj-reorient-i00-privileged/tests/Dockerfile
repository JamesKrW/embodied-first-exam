# Separate verifier sandbox (PROTOCOL §2.6): the same image with the verifier's own copies of the tooling and the frozen
# instance. Nothing the agent did in its container reaches this one: Harbor uploads only the declared artifacts.
FROM ghcr.io/mll-lab-nu/efe:myosuite-0.1.1
COPY instance.npz test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:myosuite-files-0.3.1@sha256:0824ca4608a892943c657a4d5ad514636c56c6336fc4dc6b962cdd08a2b8d3d1 /privileged/environment/rcb_myosuite.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:76b33f25e81b0e58f7c1f0b0e1e7e031e067b11da79dc5bc832a82ac53b4f7a2 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
