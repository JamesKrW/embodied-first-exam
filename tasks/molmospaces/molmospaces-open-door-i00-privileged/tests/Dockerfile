# Separate verifier sandbox (PROTOCOL §4): the same simulator image with the verifier's own copies of the tooling and the
# frozen instance. Nothing the agent did in its container reaches this one: Harbor uploads only /app/output/.
FROM ghcr.io/mll-lab-nu/efe:molmospaces-0.2.5
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:molmospaces-files-0.2.0@sha256:d6664262a50477c3a55c6e50347817ea4645e3e60ef586d9c2e9a3cb2748f7fc /privileged/environment/rcb_molmospaces.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:ecc5f24f1516ca11711d7c2d3ac07c89f79cd9220812c1244498ec71e352402a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
