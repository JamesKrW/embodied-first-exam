# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:pandadyn-0.1.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:pandadyn-files-0.3.0@sha256:f41e31ef2583946d8b3e99144b2659cbcf443f888c3bbfd1c4868c1e13452033 \
     /privileged/environment/rcb_pandadyn.py \
     /sim/rcb_service.py \
     /sim/variants/727bef3b2d/server.py \
     /srv/
