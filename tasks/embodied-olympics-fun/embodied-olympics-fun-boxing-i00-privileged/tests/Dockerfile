# Separate verifier sandbox (PROTOCOL §2.6): the same simulator image with the verifier's own copies of the tooling, the
# world package, the family's rules and the frozen instance (the grader, the tooling and the world package's core from
# the files-only image images/embodied-olympics-fun-files/). Nothing the agent did in its container reaches this one:
# Harbor uploads only /app/output/.
FROM ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1
COPY instance.json rfo_task.py test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /grade.py /tests/grade.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /rcb_rfo.py /tests/rcb_rfo.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /rfo/__init__.py /tests/rfo/__init__.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /rfo/scene.py /tests/rfo/scene.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /rfo/looks.py /tests/rfo/looks.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /rfo/world.py /tests/rfo/world.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /rfo/referee.py /tests/rfo/referee.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /rfo/task.py /tests/rfo/task.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:b01dfa6fc44ade0e46bdb16b0bfdc0fbc7e0e87200a0c49f676d80bc3c9eb95a /rfo/realhand.py /tests/rfo/realhand.py
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
