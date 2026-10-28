# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:myosuite-0.1.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:myosuite-files-0.4.0@sha256:3bb4826f99a51edabdd6ff29b6948bfbeb4332222f6ae7a6d85601268a309143 \
     /sim/finalize.sh \
     /privileged/environment/rcb_myosuite.py \
     /sim/server.py \
     /sim/task.json \
     /sim/video.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:1dde7816f44b428dd7045c4883935decaec4f8a76360a3379dbe90a7fc90d7dc \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
