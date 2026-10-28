# Separate verifier sandbox (PROTOCOL H.1): the same simulator image with the verifier's own copies of the tooling and
# the frozen instance. Nothing the agent did in its container reaches this one: Harbor uploads only /app/output/.
FROM ghcr.io/mll-lab-nu/efe:bigym2-0.1.1
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:bigym2-files-0.3.1@sha256:17b2f27405fe018fbddf4bf8f71003e4d73c3216f24c865f30e894c2f5643501 /privileged/environment/rcb_bigym2.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:3209fb078999b179d0c2b249822172ecd8296cb4ee6f36389198b667ca879a0d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
