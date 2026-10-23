# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.0@sha256:e2d97664330216cfc9359f88199033f94142e7e53505625de0ab0c3842379bbc /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:dextoolbench-files-0.4.0@sha256:0de208719e2f05270dca92b371243f5d31103a28039569f00183c5c27b26db19 \
     /privileged/environment/rcb_dtb.py \
     /sim/server_mj.py \
     /srv/
