FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.5.0@sha256:cda363caebdc631e68f07852ae5e78b7fb884f39310d380bad1aef611b267af3 /privileged/environment/rcb_aloha.py /opt/tools/rcb_aloha.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
