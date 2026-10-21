# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:myosuite-0.1.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:myosuite-sim-files-0.1.1@sha256:ad012d5c44ddd6231c2f4b23c9f9c2003729623cee70802630d7d5a515d815a1 \
     /sim/finalize.sh \
     /privileged/environment/rcb_myosuite.py \
     /sim/server.py \
     /sim/video.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.2@sha256:57c474cd8a011e77b8351e2d7aed463994fc4aa02ff1f465b80efb214ae4521a \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
