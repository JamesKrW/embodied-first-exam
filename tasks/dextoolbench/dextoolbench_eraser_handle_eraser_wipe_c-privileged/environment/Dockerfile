FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:dextoolbench-files-0.4.0-stateq@sha256:0c97c6e4b22c469d5b3ec8c85bc8788cbdc79892d341d3cfa2d5d1b6d8002be4 /privileged/environment/rcb_dtb.py /opt/tools/rcb_dtb.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
