# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:pandadyn-0.1.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.1@sha256:d9a4e130a67056d0fb47594562ac303b2f000c5f48fbe5ec235593bef0f3543f /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:pandadyn-files-0.3.0@sha256:f41e31ef2583946d8b3e99144b2659cbcf443f888c3bbfd1c4868c1e13452033 \
     /privileged/environment/rcb_pandadyn.py \
     /sim/rcb_service.py \
     /sim/server.py \
     /srv/
