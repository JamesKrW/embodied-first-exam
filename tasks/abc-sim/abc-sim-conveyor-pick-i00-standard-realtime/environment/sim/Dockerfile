# The `sim` sidecar's image (environment/docker-compose.yaml builds it): the base image (the simulator and the
# benchmark's realtime adapter at /opt/abc-tools), this directory at /srv (the frozen instance and the task's text), and
# the shared realtime layer from the pinned files-only image. .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:abc-sim-0.2.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.0@sha256:e2d97664330216cfc9359f88199033f94142e7e53505625de0ab0c3842379bbc \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
