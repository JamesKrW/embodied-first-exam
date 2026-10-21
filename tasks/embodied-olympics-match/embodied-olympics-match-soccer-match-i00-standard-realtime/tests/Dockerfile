# Separate verifier sandbox: grades the sim service's artifacts; needs nothing but Python.
FROM docker.io/library/python:3.12.14-slim@sha256:f77ac9e44ae96ef2c90b8053ea08c31f8be030f824196b0ae4db6d462c84e51f
COPY test.sh /tests/test.sh
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-match-files-0.1.4@sha256:2bbbccb896382d7c43aa45ffc893a7b11c962014b8bedc9c947f04dfd49a2700 /shared/tests/grade.py /tests/grade.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-match-files-0.1.4@sha256:2bbbccb896382d7c43aa45ffc893a7b11c962014b8bedc9c947f04dfd49a2700 /shared/tests/variants/44aa8d7a20/sport_grade.py /tests/sport_grade.py
COPY contract.json /tests/contract.json
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-match-files-0.1.4@sha256:2bbbccb896382d7c43aa45ffc893a7b11c962014b8bedc9c947f04dfd49a2700 /shared/tests/nocode_audit.py /tests/nocode_audit.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-match-files-0.1.4@sha256:2bbbccb896382d7c43aa45ffc893a7b11c962014b8bedc9c947f04dfd49a2700 /shared/tests/seeds.py /tests/seeds.py
COPY fexam_reward.py /tests/fexam_reward.py
RUN chmod +x /tests/test.sh
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.3.1@sha256:dd6eacbf8886a62365d53bd0b3832d09749ea0e10b098b3442bb0cd26cbfd35d /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
