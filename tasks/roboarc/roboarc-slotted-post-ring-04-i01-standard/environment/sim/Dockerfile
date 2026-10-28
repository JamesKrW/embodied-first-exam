# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:roboarc-0.2.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:roboarc-files-0.4.0@sha256:beb13ec1db0d85df2c5527529cda03101b15dd6cf524d34b9b288273131f44ca \
     /standard/sim/finalize.sh \
     /nyx_direct.py \
     /rcb_topogym.py \
     /render_bridge.py \
     /standard/sim/replay.py \
     /sim/run.sh \
     /standard/sim/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:f8652e8f64c8da11b6c60b1acbdb76db5cb48813bd6ae17d11ecbabebe3fa184 \
     /rcb1.py \
     /v2_4/rcb_limited.py \
     /srv/
