# Base image = DexJoCo on MuJoCo 3.4.0 (images/dexjoco/). Everything task-specific is copied here. CPU only.
FROM ghcr.io/mll-lab-nu/efe:dexjoco-0.1.2
COPY instance.npz /app/instance.npz
COPY --from=ghcr.io/mll-lab-nu/efe:dexjoco-files-0.2.0@sha256:6ba4f565aa356ec3a2a3ccae648a8e6047aa69086353be86de95041041e0e73f /privileged/environment/rcb_dexjoco.py /opt/tools/rcb_dexjoco.py
ENV PYTHONPATH=/opt/tools
# the simulation's result depends on OpenBLAS's thread count below 4; 4 is what the verifier runs with (tasks/dexjoco/README.md)
ENV OPENBLAS_NUM_THREADS=4
RUN mkdir -p /app/output
WORKDIR /app
