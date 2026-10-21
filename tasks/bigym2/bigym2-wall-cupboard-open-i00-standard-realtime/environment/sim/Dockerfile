# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:bigym2-0.1.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:bigym2-files-0.3.0@sha256:f02ae38e6422be2ea202aa3ec03c32ff1124a0a9e685735196cf0ce32a0f0284 \
     /privileged/environment/rcb_bigym2.py \
     /sim/rcb_service.py \
     /sim/server.py \
     /srv/
