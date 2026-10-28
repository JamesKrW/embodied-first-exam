FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/hands.py /privileged/tests/grade.py /privileged/environment/arm_rig.py /privileged/environment/variants/af4ca24aff/catalog.json /privileged/environment/variants/0093c53b9d/checker.py /privileged/environment/variants/40f0888ab7/replay.py /privileged/environment/variants/48e11c53de/runtime.py /privileged/environment/variants/78e1d563c1/scene.py /privileged/environment/variants/0e8a7ea9b2/task.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/assets/vacuum_scene.xml /tests/assets/vacuum_scene.xml
ENV PYTHONPATH=/tests
RUN chmod +x /tests/test.sh
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:69dcd5dd5dff08cfc35fc891e2563a374b6c4cc156be84ee88b9db456662f726 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
