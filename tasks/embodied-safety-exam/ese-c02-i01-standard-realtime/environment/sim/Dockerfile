# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:ese-0.5.6
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:ese-files-0.6.0@sha256:cf1cf3fb360e6607e3bb33ad733b207b56bffc621c2bb987f7d2fcbb69c89599 \
     /standard/candidate_identity.py \
     /standard/picture_delivery.py \
     /standard/rcb_ese.py \
     /realtime/rcb_ese_rt.py \
     /standard/rcb_service.py \
     /standard/replay.py \
     /standard/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
