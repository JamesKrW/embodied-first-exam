FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/variants/522102e076/catalog.json /privileged/environment/hands.py /privileged/environment/variants/1ccb553961/replay.py /privileged/environment/variants/6efba74799/runtime.py /privileged/environment/variants/af13909348/scene.py /privileged/environment/variants/35cd429f41/task.json /opt/tools/
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
