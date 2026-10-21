# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:dexjoco-0.1.2
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:dexjoco-files-0.2.0@sha256:6ba4f565aa356ec3a2a3ccae648a8e6047aa69086353be86de95041041e0e73f \
     /sim/finalize.sh \
     /privileged/environment/rcb_dexjoco.py \
     /sim/server.py \
     /sim/video.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.1@sha256:d9a4e130a67056d0fb47594562ac303b2f000c5f48fbe5ec235593bef0f3543f \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
