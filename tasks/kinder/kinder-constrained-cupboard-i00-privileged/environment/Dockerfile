# Base image = KinDER (images/kinder/). Everything task-specific is copied here.
FROM ghcr.io/mll-lab-nu/efe:kinder-0.1.6
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:kinder-files-0.3.0@sha256:267ea96a2c43a8e8b9a00ab7c74c3b919be238724aad0b4f85f5f37454f6c2ef /privileged/environment/rcb_kinder.py /opt/tools/rcb_kinder.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
