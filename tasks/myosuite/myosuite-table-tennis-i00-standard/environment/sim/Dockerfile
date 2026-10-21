# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:myosuite-0.1.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:myosuite-files-0.3.0@sha256:59164d61e41178e371d175e17e33d42beb29fbee529e7fabd37bc244f4e32625 \
     /sim/finalize.sh \
     /privileged/environment/rcb_myosuite.py \
     /sim/server.py \
     /sim/task.json \
     /sim/video.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
