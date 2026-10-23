# Base image = HumanoidBench environment (see images/humanoidbench/). Everything task-specific is copied here.
# Generated from scripts/humanoidbench/templates/Dockerfile -- edit the template, not this copy.
FROM ghcr.io/mll-lab-nu/efe:humanoidbench-0.2.0
COPY instance.npz /app/instance.npz
COPY --from=ghcr.io/mll-lab-nu/efe:humanoidbench-files-0.3.1@sha256:df3e8d9b499e19de3627cd895b65de21e8f73e90bb9cce51badaabe0de9447a0 /privileged/environment/rcb_hb.py /opt/tools/rcb_hb.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
