# Standard mode: the agent container has NO simulator and no task code — only Python and the client.
FROM docker.io/library/python:3.12-slim
RUN pip install --no-cache-dir requests numpy pillow scipy
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.2.2@sha256:287237e431c08fdbc820a96282536b52c68c27e60cc244ba70b07cb1ae526af2 /rcb1.py /v2_1/variants/62def5e7ce/rcb_client.py /opt/tools/
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
