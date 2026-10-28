# Separate verifier sandbox: the same image with the verifier's own copy of the frozen instance (the grader is the
# image's /opt/abc-tools/grade.py). Nothing the agent did in its container reaches this one: Harbor uploads only the
# declared artifacts.
FROM ghcr.io/mll-lab-nu/efe:abc-sim-0.1.0
COPY instance.npz test.sh /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:3209fb078999b179d0c2b249822172ecd8296cb4ee6f36389198b667ca879a0d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
