# Separate verifier sandbox (PROTOCOL §2.6): the same image with the verifier's own copies of the tooling and the frozen
# instance. Nothing the agent did in its container reaches this one: Harbor uploads only the declared artifacts.
FROM ghcr.io/mll-lab-nu/efe:myosuite-0.1.1
COPY instance.npz test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:myosuite-files-0.1.2@sha256:a33a6854224ca140c368c245bd6b1e632bf31423e850db43d0bc0c7fd16e989a /privileged/environment/rcb_myosuite.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
