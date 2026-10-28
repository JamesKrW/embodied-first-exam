# Separate verifier sandbox (PROTOCOL H.1): the same image with the verifier's own copies of the tooling and the frozen
# instance. Nothing the agent did in its container reaches this one: Harbor uploads only the declared artifacts.
FROM ghcr.io/mll-lab-nu/efe:myosuite-0.1.1
COPY instance.npz test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:myosuite-files-0.4.0@sha256:3bb4826f99a51edabdd6ff29b6948bfbeb4332222f6ae7a6d85601268a309143 /privileged/environment/rcb_myosuite.py /privileged/tests/grade.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:ecc5f24f1516ca11711d7c2d3ac07c89f79cd9220812c1244498ec71e352402a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
