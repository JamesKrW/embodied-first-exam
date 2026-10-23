# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:metaworld-0.1.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:metaworldplus-files-0.4.0@sha256:8bea99665e84ea44a36a3d24848ed4313d9d12cd68743167707ae04aad2b99b5 \
     /privileged/environment/mw_tool.py \
     /sim/server_mw.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.0@sha256:e2d97664330216cfc9359f88199033f94142e7e53505625de0ab0c3842379bbc /rcb1.py /srv/rcb1.py
