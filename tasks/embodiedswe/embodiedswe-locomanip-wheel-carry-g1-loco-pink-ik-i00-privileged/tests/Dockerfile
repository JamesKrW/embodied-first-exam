# Separate verifier sandbox (PROTOCOL H.1): the same base image, the verifier's own copy of the frozen instance and the
# task's grader (EmbodiedSWE's, unchanged, at the pinned commit). /tests is root-only: the solution process (nobody)
# cannot read it.
FROM ghcr.io/mll-lab-nu/efe-private:embodiedswe-0.3.0
COPY --from=ghcr.io/mll-lab-nu/efe:embodiedswe-files-0.3.0@sha256:8cd9ce255fcc7034d98426df007760035af5c4777f1bf208c93146a2eee78425 /rcb_embodiedswe.py /esw_proxy.py /opt/efe/
COPY grade.py instance.json test.sh /tests/
COPY grader/ /tests/grader/
RUN chmod 700 /tests && chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
