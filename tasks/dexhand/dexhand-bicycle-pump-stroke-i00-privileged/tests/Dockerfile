FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/hands.py /privileged/tests/grade.py /privileged/environment/arm_rig.py /privileged/environment/variants/c10daa5175/catalog.json /privileged/environment/variants/989e170b27/checker.py /privileged/environment/variants/52e9f44eef/replay.py /privileged/environment/variants/328103d2bd/runtime.py /privileged/environment/variants/4053e4abc2/scene.py /privileged/environment/task.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/assets/meshes/continuous_collar.obj /privileged/environment/assets/meshes/continuous_hollow_head.obj /privileged/environment/assets/meshes/rim.obj /privileged/environment/assets/meshes/smooth_full_tire.obj /tests/assets/meshes/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/assets/t50_scene.json /tests/assets/t50_scene.json
ENV PYTHONPATH=/tests
RUN chmod +x /tests/test.sh
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:69dcd5dd5dff08cfc35fc891e2563a374b6c4cc156be84ee88b9db456662f726 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
