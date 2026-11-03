FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.4.0-stateq@sha256:21852e2ab6751b6f200608173cc10f64296b1b49be905d3eb7158f26fa2a4149 /privileged/environment/hands.py /privileged/environment/catalog.json /privileged/environment/variants/bd227f1806/checker.py /privileged/environment/variants/b8d126ab19/replay.py /privileged/environment/variants/edb2fa483d/runtime.py /privileged/environment/variants/833515eb2e/scene.py /privileged/environment/variants/e23dbd3027/task.json /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.4.0-stateq@sha256:21852e2ab6751b6f200608173cc10f64296b1b49be905d3eb7158f26fa2a4149 /privileged/environment/assets/sharpa_bimanual_hands.xml /privileged/environment/assets/t48_tray_glasses.xml /opt/tools/assets/
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
