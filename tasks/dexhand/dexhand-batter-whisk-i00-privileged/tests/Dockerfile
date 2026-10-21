FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/hands.py /privileged/tests/grade.py /privileged/environment/arm_rig.py /privileged/environment/variants/bc48a3e915/catalog.json /privileged/environment/variants/5b6c0efb04/checker.py /privileged/environment/variants/b1a971bef9/replay.py /privileged/environment/variants/0bd2e5aa3c/runtime.py /privileged/environment/variants/c1c76156ec/scene.py /privileged/environment/variants/0b89750a2c/task.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/assets/rosle-bowl-original.obj /privileged/environment/assets/rosle-whisk-wires-original.obj /tests/assets/
ENV PYTHONPATH=/tests
RUN chmod +x /tests/test.sh
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
