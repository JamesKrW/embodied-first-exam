FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
LABEL roboengineer.instance_sha256="40ecdfeb531b3f8112ca0caaf241147080777200a699d469f0096b72ba95e0d7"
COPY test.sh instance.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:roboengineer-files-0.3.0@sha256:6d1e352b3c04c10b77cfe5d8afaa349a34678bca4f49259d82e6e093b6066a84 /standard/tests/grade.py /tests/grade.py
RUN echo '40ecdfeb531b3f8112ca0caaf241147080777200a699d469f0096b72ba95e0d7  /tests/instance.json' | sha256sum -c - && chmod +x /tests/test.sh
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:935a38662d5158d6091473c718a3c4c83fa43656e3548d4f84008dfc02678987 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
