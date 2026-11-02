# Base image = plain MuJoCo 3.14.0 and the IKEA scenes (images/ikea-mujoco/). Everything task-specific is copied here.
FROM ghcr.io/mll-lab-nu/efe:ikea-mujoco-0.3.0
COPY --from=ghcr.io/mll-lab-nu/efe:ikea-mujoco-files-0.4.0-resetpose@sha256:3ab58e65b7eafdf1d5e15c56a8e711041eb7938661e13ba9a84c75b92012b909 /privileged/environment/rcb_ikea.py /opt/tools/rcb_ikea.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
