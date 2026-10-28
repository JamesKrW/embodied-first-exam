# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:robocasa365-0.1.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa365-files-0.5.0@sha256:ecf24cf62b905572241b9746d50055bc7f6526194ef6b31266dc02027681ad0d \
     /privileged/environment/casa_tool.py \
     /sim/rcb_service.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:f8652e8f64c8da11b6c60b1acbdb76db5cb48813bd6ae17d11ecbabebe3fa184 \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
