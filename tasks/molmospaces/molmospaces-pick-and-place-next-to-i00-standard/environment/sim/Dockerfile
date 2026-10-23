# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:molmospaces-0.2.5
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:molmospaces-files-0.3.0@sha256:6f32e96e232b86d82a663573c68d1f06f4cff5312355410d831c2b2b06f730be \
     /sim/finalize.sh \
     /privileged/environment/rcb_molmospaces.py \
     /sim/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.0@sha256:e2d97664330216cfc9359f88199033f94142e7e53505625de0ab0c3842379bbc \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
