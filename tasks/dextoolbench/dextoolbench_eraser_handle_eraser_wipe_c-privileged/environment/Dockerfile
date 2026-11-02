FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:dextoolbench-files-0.4.0-resetpose@sha256:c998059bb7df1ab37bb6640a026a86f15d735bc1e26bf2f2b6f24b30d996529c /privileged/environment/rcb_dtb.py /opt/tools/rcb_dtb.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
