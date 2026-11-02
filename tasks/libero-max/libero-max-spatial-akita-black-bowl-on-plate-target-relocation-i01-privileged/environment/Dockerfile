# LIBERO-MAX privileged task: the LIBERO-MAX image, the task's tooling and its frozen instance.
FROM ghcr.io/mll-lab-nu/efe:libero-max-0.1.1
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:libero-max-files-0.4.0-resetpose@sha256:bb46a93b4c58e8a6410fc89121b470dc674915c8c07e3c90ba2422d25de71a68 /privileged/environment/lmx_tool.py /opt/tools/lmx_tool.py
ENV PYTHONPATH=/opt/tools:/opt/libero-max/overlay:/opt/libero-max/src
RUN mkdir -p /app/output
WORKDIR /app
