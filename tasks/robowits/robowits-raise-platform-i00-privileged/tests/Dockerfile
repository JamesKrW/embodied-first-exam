# Separate verifier sandbox (PROTOCOL §4): the same simulator image with the verifier's own copies of the
# tooling and the frozen instance baked in. Nothing the agent did in its container reaches this one — Harbor
# uploads only the declared artifacts (/app/output/), so a patched success check or a swapped interpreter in
# the agent container cannot influence the verdict.
FROM ghcr.io/mll-lab-nu/efe:robowits-0.1.1
COPY instance.json test.sh /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:robowits-files-0.4.0-resetpose@sha256:cf0bf5dd5db2f6b60dc3ed60ff3c3b7bc6d2a0145658a7d91e211e61f9a9aef9 /privileged/tests/grade.py /privileged/tests/rcb_robowits.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests QD_OFFLINE_CACHE=0
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:ecc5f24f1516ca11711d7c2d3ac07c89f79cd9220812c1244498ec71e352402a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
