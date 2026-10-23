# MuJoCo Playground quadruped task (privileged): plain MuJoCo with Playground's and Menagerie's assets
# (images/mujoco/prepare_assets.sh), the task's tooling and its frozen instance.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.3
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.4.1@sha256:edc4ee147fcda819b1ed3df6faa7277aca673f75f52bac2cf34f3dd9357c4e9b /privileged/environment/rcb_quadruped.py /opt/tools/rcb_quadruped.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
