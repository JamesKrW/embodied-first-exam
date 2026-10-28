# The `sim` sidecar's image (environment/docker-compose.yaml builds it): the base image (the simulator and the
# benchmark's 2.2 adapter at /opt/abc-tools), this directory at /srv (the frozen instance and the task's text), and
# the shared 2.2 layer from the pinned files-only image. .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:abc-sim-0.2.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:f8652e8f64c8da11b6c60b1acbdb76db5cb48813bd6ae17d11ecbabebe3fa184 \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
