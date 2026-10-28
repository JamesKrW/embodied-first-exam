# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.4.0@sha256:f9bfacf8829c29d55552a1646f4af4878a958a06025452a1fa6ada74d530c556 \
     /privileged/environment/arm_rig.py \
     /sim/finalize.py \
     /sim/finalize.sh \
     /privileged/environment/hands.py \
     /sim/rcb_dexhand.py \
     /sim/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:1dde7816f44b428dd7045c4883935decaec4f8a76360a3379dbe90a7fc90d7dc \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
