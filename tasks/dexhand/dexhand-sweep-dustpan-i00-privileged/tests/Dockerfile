FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/hands.py /privileged/tests/grade.py /privileged/environment/variants/99a6471895/catalog.json /privileged/environment/variants/8ae9cbedaa/checker.py /privileged/environment/variants/e8cd4d53ee/replay.py /privileged/environment/variants/2ef7f8ba38/runtime.py /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/assets/external_assets.json /privileged/environment/assets/reset.npz /privileged/environment/assets/reset_wuji.npz /privileged/environment/assets/sweep_sharpa.xml /tests/assets/
ENV PYTHONPATH=/tests
RUN chmod +x /tests/test.sh
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:9d09fa2fe44ee6af5ccdff49827f005934c053f5dce34044e036c9facc3b343a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
