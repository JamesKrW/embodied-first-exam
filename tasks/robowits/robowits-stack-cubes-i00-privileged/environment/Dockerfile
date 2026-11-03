# Base image = RoboWits + Genesis 0.4.7 (see images/robowits/). Everything task-specific is copied here.
FROM ghcr.io/mll-lab-nu/efe:robowits-0.1.1
COPY instance.json /app/instance.json
COPY --from=ghcr.io/mll-lab-nu/efe:robowits-files-0.4.0-stateq-pose@sha256:ccd560ab17df600a6295470375534ddd90371d9a0e8654de96b73add1b541b95 /privileged/tests/rcb_robowits.py /opt/tools/rcb_robowits.py
ENV PYTHONPATH=/opt/tools QD_OFFLINE_CACHE=0
RUN mkdir -p /app/output
WORKDIR /app
