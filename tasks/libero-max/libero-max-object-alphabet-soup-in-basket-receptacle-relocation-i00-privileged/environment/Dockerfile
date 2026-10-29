# LIBERO-MAX privileged task: the LIBERO-MAX image, the task's tooling and its frozen instance.
FROM ghcr.io/mll-lab-nu/efe:libero-max-0.1.1
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:libero-max-files-0.4.0@sha256:3fc8688820782240c5c75283da820c46a3831ad0b706e6793d91110132a49a3e /privileged/environment/lmx_tool.py /opt/tools/lmx_tool.py
ENV PYTHONPATH=/opt/tools:/opt/libero-max/overlay:/opt/libero-max/src
RUN mkdir -p /app/output
WORKDIR /app
