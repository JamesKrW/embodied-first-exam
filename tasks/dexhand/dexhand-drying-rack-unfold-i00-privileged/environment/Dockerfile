FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/hands.py /privileged/environment/variants/3840cb4b04/catalog.json /privileged/environment/variants/2f476ee029/checker.py /privileged/environment/variants/d2bc9037f8/replay.py /privileged/environment/variants/eb836781a3/runtime.py /privileged/environment/variants/862070f80d/scene.py /privileged/environment/variants/b8a169a7d9/task.json /opt/tools/
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
