# Base image = plain MuJoCo 3.14.0 and the IKEA scenes (images/ikea-mujoco/). Everything task-specific is copied here.
FROM ghcr.io/mll-lab-nu/efe:ikea-mujoco-0.3.0
COPY --from=ghcr.io/mll-lab-nu/efe:ikea-mujoco-files-0.4.3@sha256:c15ce62523e6a0cffb1bfdb52a1241b54a5abd957ab05a341d6db210d52c2a38 /privileged/environment/rcb_ikea.py /opt/tools/rcb_ikea.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
