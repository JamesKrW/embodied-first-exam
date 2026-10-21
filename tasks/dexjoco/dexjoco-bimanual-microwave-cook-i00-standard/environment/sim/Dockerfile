# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:dexjoco-0.1.2
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:dexjoco-sim-files-0.1.1@sha256:1ddd0c270ea7162462b584677e6ec3cc76ec912d7d2531fd90446aa8c8e4ff6a \
     /sim/finalize.sh \
     /privileged/environment/rcb_dexjoco.py \
     /sim/server.py \
     /sim/video.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.2@sha256:57c474cd8a011e77b8351e2d7aed463994fc4aa02ff1f465b80efb214ae4521a \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
