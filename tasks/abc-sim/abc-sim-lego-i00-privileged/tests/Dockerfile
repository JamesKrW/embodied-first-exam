# Separate verifier sandbox: the same image with the verifier's own copy of the frozen instance (the grader is the
# image's /opt/abc-tools/grade.py). Nothing the agent did in its container reaches this one: Harbor uploads only the
# declared artifacts.
FROM ghcr.io/mll-lab-nu/efe:abc-sim-0.1.0
COPY instance.npz test.sh /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:935a38662d5158d6091473c718a3c4c83fa43656e3548d4f84008dfc02678987 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
