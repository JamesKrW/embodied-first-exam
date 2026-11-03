# Base image = KinDER (images/kinder/). Everything task-specific is copied here.
FROM ghcr.io/mll-lab-nu/efe:kinder-0.1.6
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:kinder-files-0.4.0-stateq@sha256:45e8a2fcd544150607869fd9b0c9d881a5e80d86c1dc636f674b3c7c99fef021 /privileged/environment/rcb_kinder.py /opt/tools/rcb_kinder.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
