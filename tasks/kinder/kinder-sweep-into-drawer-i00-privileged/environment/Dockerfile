# Base image = KinDER (images/kinder/). Everything task-specific is copied here.
FROM ghcr.io/mll-lab-nu/efe:kinder-0.1.6
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:kinder-files-0.4.0-resetpose@sha256:7dbf9243a9da97d0ed198fd8e186ea4cde07834d04f76547195aa16b2f582ba8 /privileged/environment/rcb_kinder.py /opt/tools/rcb_kinder.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
