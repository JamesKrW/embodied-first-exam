# Standard mode: the agent container has NO simulator and no task code — only Python and the client.
FROM docker.io/library/python:3.12-slim
RUN pip install --no-cache-dir requests numpy pillow scipy
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:331e2a74d699de500cbc63027d16e144db89c59b955cf6efbb3198ab1f69bcd8 /rcb1.py /v2_1/variants/62def5e7ce/rcb_client.py /opt/tools/
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
