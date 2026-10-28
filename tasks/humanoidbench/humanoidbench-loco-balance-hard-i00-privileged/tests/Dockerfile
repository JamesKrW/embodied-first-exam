# Separate verifier sandbox (docs/protocol/README.md Appendix H.1): the same base image, plus the verifier's own copies of the
# tooling, this task's reward and the frozen instance. Nothing the agent did in its container reaches this one:
# Harbor uploads only the declared artifacts (/app/output/).
# Generated from scripts/humanoidbench/templates/tests.Dockerfile -- edit the template, not this copy.
FROM ghcr.io/mll-lab-nu/efe:humanoidbench-0.2.0
COPY instance.npz rewards.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:humanoidbench-files-0.3.1@sha256:df3e8d9b499e19de3627cd895b65de21e8f73e90bb9cce51badaabe0de9447a0 /privileged/environment/rcb_hb.py /privileged/tests/grade.py /privileged/tests/rcb_rewards.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:935a38662d5158d6091473c718a3c4c83fa43656e3548d4f84008dfc02678987 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
