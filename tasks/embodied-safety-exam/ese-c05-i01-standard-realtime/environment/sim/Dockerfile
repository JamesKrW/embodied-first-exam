# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:ese-0.5.6
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:ese-files-0.6.1@sha256:9e4d43abde2f33076467f09c035fec44d15ea4c38a0d6fe8ebf8dbc76a2db219 \
     /standard/candidate_identity.py \
     /standard/picture_delivery.py \
     /standard/rcb_ese.py \
     /realtime/rcb_ese_rt.py \
     /standard/rcb_service.py \
     /standard/replay.py \
     /standard/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.0@sha256:e2d97664330216cfc9359f88199033f94142e7e53505625de0ab0c3842379bbc \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
