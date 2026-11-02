FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.4.0-resetpose@sha256:9d1f54a40c8ea34df8a02efd57fd333ff7b7e96fee2bf691c83fa44551804a1f /privileged/environment/hands.py /privileged/environment/arm_rig.py /privileged/environment/variants/c1ce7ef43a/catalog.json /privileged/environment/variants/112500031b/checker.py /privileged/environment/variants/6e5f22b48c/replay.py /privileged/environment/variants/4b30501d99/runtime.py /privileged/environment/variants/e893d23529/scene.py /privileged/environment/variants/ccb1b8661e/task.json /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.4.0-resetpose@sha256:9d1f54a40c8ea34df8a02efd57fd333ff7b7e96fee2bf691c83fa44551804a1f /privileged/environment/assets/usb_scene.json /opt/tools/assets/usb_scene.json
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
