FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
LABEL roboengineer.instance_sha256="e7032816b31bc45e519af1a93a64c51926eb14711e5e932329d1f806fdfe7fea"
COPY test.sh instance.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:roboengineer-files-0.3.0@sha256:6d1e352b3c04c10b77cfe5d8afaa349a34678bca4f49259d82e6e093b6066a84 /standard/tests/grade.py /tests/grade.py
RUN echo 'e7032816b31bc45e519af1a93a64c51926eb14711e5e932329d1f806fdfe7fea  /tests/instance.json' | sha256sum -c - && chmod +x /tests/test.sh
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:3209fb078999b179d0c2b249822172ecd8296cb4ee6f36389198b667ca879a0d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
