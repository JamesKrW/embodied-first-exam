# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:kinder-0.1.6
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.0@sha256:e2d97664330216cfc9359f88199033f94142e7e53505625de0ab0c3842379bbc /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:kinder-files-0.3.1@sha256:28ae8c1aebd73de12ba242d818f665b2e3f8eb9df28fe8c9f52bea45744ec116 \
     /privileged/environment/rcb_kinder.py \
     /sim/rcb_service.py \
     /sim/server.py \
     /srv/
