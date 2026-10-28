# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:pandadyn-0.2.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:1dde7816f44b428dd7045c4883935decaec4f8a76360a3379dbe90a7fc90d7dc /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:pandadyn-files-0.4.0@sha256:1fa728bd3b4ce80213fc0f2613f97bbbb51f31a5c5b5aa9ed9aa8582a61fa1e1 \
     /privileged/environment/rcb_pandadyn.py \
     /sim/rcb_service.py \
     /sim/server.py \
     /srv/
