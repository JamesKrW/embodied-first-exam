FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/hands.py /privileged/environment/arm_rig.py /privileged/environment/variants/85d44635c5/catalog.json /privileged/environment/variants/cf1c141a55/checker.py /privileged/environment/variants/4c79bfe5f4/replay.py /privileged/environment/variants/969b07e0b2/runtime.py /privileged/environment/variants/95e2695c2d/scene.py /privileged/environment/variants/cf26dc5048/task.json /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/assets/water-lilies.png /opt/tools/assets/water-lilies.png
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
