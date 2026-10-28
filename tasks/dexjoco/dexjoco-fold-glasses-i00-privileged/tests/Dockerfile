# Separate verifier sandbox (PROTOCOL H.1): the same simulator image with the verifier's own copies of the tooling and
# the frozen instance baked in. Nothing the agent did in its container reaches this one: Harbor uploads only the declared
# artifacts (/app/output/), so a patched success check in the agent container cannot count.
FROM ghcr.io/mll-lab-nu/efe:dexjoco-0.1.2
COPY instance.npz test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexjoco-files-0.2.0@sha256:6ba4f565aa356ec3a2a3ccae648a8e6047aa69086353be86de95041041e0e73f /privileged/environment/rcb_dexjoco.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
# the thread count the instances and references were made with; it needs 4 CPUs visible to this container, which
# grade.py checks before it replays (tasks/dexjoco/README.md, Threads)
ENV OPENBLAS_NUM_THREADS=4
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:3209fb078999b179d0c2b249822172ecd8296cb4ee6f36389198b667ca879a0d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
