# MuJoCo Playground quadruped task (privileged): plain MuJoCo with Playground's and Menagerie's assets
# (images/mujoco/prepare_assets.sh), the task's tooling and its frozen instance.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.3
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.5.0-stateq-pose@sha256:5c488bba17f485ba756b059a24787885fb3453e82104abe63dd248b806f0715d /privileged/environment/rcb_quadruped.py /opt/tools/rcb_quadruped.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
