# MuJoCo Playground quadruped task (privileged): plain MuJoCo with Playground's and Menagerie's assets
# (images/mujoco/prepare_assets.sh), the task's tooling and its frozen instance.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.3
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.5.0@sha256:3f0b9610b741f2807ff2c5b545c5287500de98ab1f7dcadc9f4780cd0a6bc42a /privileged/environment/rcb_quadruped.py /opt/tools/rcb_quadruped.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
