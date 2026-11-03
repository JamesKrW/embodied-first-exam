# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:humanoidbench-0.2.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0-stateq@sha256:07fbf9e22ea11efd3398ef74d888ee7544410c1e07ff49e3fc5b8606b9acfe38 \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:humanoidbench-files-0.4.0-stateq@sha256:891cdc5edcf01f945332d55ef988f5c419e45a1c7c91b526dd9cc28cd12d7aae \
     /privileged/environment/rcb_hb.py \
     /privileged/tests/rcb_rewards.py \
     /sim/rcb_service.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:humanoidbench-leakfix-files-0.1.0@sha256:9a18496a7bf7195440fd1b41215d5bf65c421bd4d074cdf36deaf02e9f177f18 \
     /sim/server.py \
     /srv/
