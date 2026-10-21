# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:metaworld-0.1.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:metaworldplus-files-0.2.0@sha256:121530d521058255bdb8aa0ea3f61c5733c1381f5e1254532c8c355218d22cf2 \
     /privileged/environment/mw_tool.py \
     /sim/server_mw.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.1@sha256:d9a4e130a67056d0fb47594562ac303b2f000c5f48fbe5ec235593bef0f3543f /rcb1.py /srv/rcb1.py
