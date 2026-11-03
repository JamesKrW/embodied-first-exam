# Separate verifier sandbox (docs/protocol/README.md Appendix H.1): the same base image, plus the verifier's own copies of the
# tooling, this task's reward and the frozen instance. Nothing the agent did in its container reaches this one:
# Harbor uploads only the declared artifacts (/app/output/).
# Generated from scripts/humanoidbench/templates/tests.Dockerfile -- edit the template, not this copy.
FROM ghcr.io/mll-lab-nu/efe:humanoidbench-0.2.0
COPY instance.npz rewards.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:humanoidbench-files-0.4.0-stateq-leakfix@sha256:cb7ca74f44da387d94cfd0f780b37d4926e3c17ee66028cb7052646f5b7a67a2 /privileged/environment/rcb_hb.py /privileged/tests/grade.py /privileged/tests/rcb_rewards.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:ecc5f24f1516ca11711d7c2d3ac07c89f79cd9220812c1244498ec71e352402a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
