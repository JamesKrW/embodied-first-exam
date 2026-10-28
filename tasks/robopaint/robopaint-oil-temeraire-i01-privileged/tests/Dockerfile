# Separate verifier sandbox (PROTOCOL §4): the same simulator image with the verifier's own copies of the tooling and the graders'
# frozen instance (+ the target picture and its licence) baked in. Nothing the agent did in its container reaches this one — Harbor
# uploads only the declared artifacts (/app/output/), so a patched scorer or a swapped interpreter cannot influence the verdict.
FROM ghcr.io/mll-lab-nu/efe:robopaint-0.2.0
COPY instance.json truth_alpha.npz test.sh NOTICE-exemplar.md /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:robopaint-pictures-files-0.1.0@sha256:3ce92526aa79918994ccb934390221e1dd6c3f4364109a40e5746140def6062e /oil-temeraire-i01/target.png /tests/target.png
COPY --from=ghcr.io/mll-lab-nu/efe:robopaint-files-0.3.0@sha256:ffcfa94d02192f1e999c8016e814fb4c4dbbd6c9aa65f22bfc75bc9931e3238a /privileged/grade.py /tests/grade.py
COPY --from=ghcr.io/mll-lab-nu/efe:robopaint-files-0.3.0@sha256:ffcfa94d02192f1e999c8016e814fb4c4dbbd6c9aa65f22bfc75bc9931e3238a /rcb_brush.py /tests/rcb_brush.py
COPY --from=ghcr.io/mll-lab-nu/efe:robopaint-files-0.3.0@sha256:ffcfa94d02192f1e999c8016e814fb4c4dbbd6c9aa65f22bfc75bc9931e3238a /rcb_brush_score.py /tests/rcb_brush_score.py
COPY --from=ghcr.io/mll-lab-nu/efe:robopaint-files-0.3.0@sha256:ffcfa94d02192f1e999c8016e814fb4c4dbbd6c9aa65f22bfc75bc9931e3238a /rcb_brush_strict.py /tests/rcb_brush_strict.py
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §2.10) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:76b33f25e81b0e58f7c1f0b0e1e7e031e067b11da79dc5bc832a82ac53b4f7a2 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
