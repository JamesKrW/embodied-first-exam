# Base image = plain MuJoCo 3.14.0 and the IKEA scenes (images/ikea-mujoco/). Everything task-specific is copied here.
FROM ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b
COPY --from=ghcr.io/mll-lab-nu/efe:ikea-mujoco-files-0.5.0@sha256:fdebfb286aacaf4e9b5b5391c213f7800f655e1f8f3e5665e681020ad7a0824a /privileged/environment/rcb_ikea.py /opt/tools/rcb_ikea.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
