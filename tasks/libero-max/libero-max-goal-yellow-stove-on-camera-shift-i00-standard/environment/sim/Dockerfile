# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:libero-max-0.1.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:libero-max-files-0.3.0@sha256:cb8e07bea10b998bd6c0b2234424b2c35b6aed48278067311187004f9c7b3363 \
     /sim/finalize.sh \
     /privileged/environment/lmx_tool.py \
     /sim/rcb_service.py \
     /sim/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 /rcb1.py /srv/rcb1.py
