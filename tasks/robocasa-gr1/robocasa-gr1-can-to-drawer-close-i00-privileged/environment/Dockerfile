# RoboCasa-GR1 privileged task: the benchmark image, the task's tooling and its frozen instance.
FROM ghcr.io/mll-lab-nu/efe:robocasa-gr1-0.1.0
COPY instance.npz /app/instance.npz
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa-files-0.4.0-stateq@sha256:ec81fd1e6376a82856ab44faeb904821ec114b1c31a6ad6552c5f689cf3cb7bf /privileged/environment/casa_tool.py /opt/tools/casa_tool.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
