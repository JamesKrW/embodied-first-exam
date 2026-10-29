# Standard mode: the agent container has NO simulator and no MolmoSpaces code, only Python and the client.
# The client base image and the pip pins are the shared ones of every standard task (embodied_first_exam/standard/client.env).
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY client-requirements.txt /tmp/client-requirements.txt
RUN pip install --no-cache-dir -r /tmp/client-requirements.txt
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:331e2a74d699de500cbc63027d16e144db89c59b955cf6efbb3198ab1f69bcd8 /rcb_client.py /rcb1.py /opt/tools/
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
