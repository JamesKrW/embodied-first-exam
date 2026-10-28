FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
LABEL roboengineer.instance_sha256="80590ad4cdf02479a092974d3c40b94fb25190aee49f47f66de2f71f72163c2a"
COPY test.sh instance.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:roboengineer-files-0.3.0@sha256:6d1e352b3c04c10b77cfe5d8afaa349a34678bca4f49259d82e6e093b6066a84 /standard/tests/grade.py /tests/grade.py
RUN echo '80590ad4cdf02479a092974d3c40b94fb25190aee49f47f66de2f71f72163c2a  /tests/instance.json' | sha256sum -c - && chmod +x /tests/test.sh
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:76b33f25e81b0e58f7c1f0b0e1e7e031e067b11da79dc5bc832a82ac53b4f7a2 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
