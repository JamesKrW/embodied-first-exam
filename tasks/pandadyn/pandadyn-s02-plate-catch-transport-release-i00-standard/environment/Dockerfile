# Standard mode: the agent container has NO simulator and no Panda Dynamic Scenes code: only Python and the client
# (eai-standard/2.1, with reset).
# The client base image and the pip pins are the shared ones of every standard task (embodied_first_exam/standard/client.env).
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY client-requirements.txt /tmp/client-requirements.txt
RUN pip install --no-cache-dir -r /tmp/client-requirements.txt
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.2.1@sha256:192c971a2bb1fa15cf1b3057a7be354a12495f53c1ec9ccc6318177cd880d603 /rcb1.py /v2_1/rcb_client.py /opt/tools/
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
