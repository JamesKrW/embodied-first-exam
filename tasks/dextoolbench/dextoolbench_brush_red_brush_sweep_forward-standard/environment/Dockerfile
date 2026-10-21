# Standard mode: the agent container has NO simulator and no task code — only Python and the client.
FROM docker.io/library/python:3.12-slim
RUN pip install --no-cache-dir requests numpy pillow scipy
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.2.1@sha256:192c971a2bb1fa15cf1b3057a7be354a12495f53c1ec9ccc6318177cd880d603 /rcb1.py /v2_1/variants/62def5e7ce/rcb_client.py /opt/tools/
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
