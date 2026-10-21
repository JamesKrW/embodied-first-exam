# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:humanoidbench-0.2.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.2@sha256:57c474cd8a011e77b8351e2d7aed463994fc4aa02ff1f465b80efb214ae4521a \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:humanoidbench-sim-files-0.1.1@sha256:72c7bb82b590dfcc70bea5840f72110687b4fed51dbfe93129d50758b1e2d785 \
     /privileged/environment/rcb_hb.py \
     /privileged/tests/rcb_rewards.py \
     /sim/rcb_service.py \
     /sim/server.py \
     /srv/
