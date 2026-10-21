# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-sim-files-0.1.1@sha256:209f484647ce3285b3b9618dc3ef2294ec4b45ca218b35a5cf910d32b78b6604 \
     /privileged/environment/rcb_panda.py \
     /sim/server_mj.py \
     /srv/
