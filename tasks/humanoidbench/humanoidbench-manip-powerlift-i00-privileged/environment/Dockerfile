# Base image = HumanoidBench environment (see images/humanoidbench/). Everything task-specific is copied here.
# Generated from scripts/humanoidbench/templates/Dockerfile -- edit the template, not this copy.
FROM ghcr.io/mll-lab-nu/efe:humanoidbench-0.2.0
COPY instance.npz /app/instance.npz
COPY --from=ghcr.io/mll-lab-nu/efe:humanoidbench-files-0.4.0-stateq-leakfix@sha256:cb7ca74f44da387d94cfd0f780b37d4926e3c17ee66028cb7052646f5b7a67a2 /privileged/environment/rcb_hb.py /opt/tools/rcb_hb.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
