FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.4.1@sha256:edc4ee147fcda819b1ed3df6faa7277aca673f75f52bac2cf34f3dd9357c4e9b /privileged/environment/rcb_panda.py /opt/tools/rcb_panda.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
