# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:ikea-mujoco-sim-0.4.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:ikea-mujoco-sim-files-0.1.1@sha256:803b86ace64a8a7587e9a9bed6d0cc1879dc17b8c55e2bdb00201ae55f94977d \
     /sim/finalize.sh \
     /sim/ikea_nyx.py \
     /sim/ikea_rig.py \
     /privileged/environment/rcb_ikea.py \
     /sim/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:roboarc-files-0.1.0@sha256:3ebbe4b6947849941de2dd5b9af0592ead637da1bd9125df1a1b0d584ea4a49a /nyx_direct.py /srv/nyx_direct.py
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.2@sha256:57c474cd8a011e77b8351e2d7aed463994fc4aa02ff1f465b80efb214ae4521a \
     /rcb1.py \
     /v2_4/rcb_limited.py \
     /run.sh \
     /srv/
