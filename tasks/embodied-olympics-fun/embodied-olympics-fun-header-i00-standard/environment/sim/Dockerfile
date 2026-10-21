# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.0@sha256:06b914d9ae91b6306d5001eb6b1f5d696bbd2a08b4db883ec77359da3d367324 /rfo/tasks/header.py /srv/rfo_task.py
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.0@sha256:06b914d9ae91b6306d5001eb6b1f5d696bbd2a08b4db883ec77359da3d367324 \
     /rcb_rfo.py \
     /standard/rcb_service.py \
     /standard/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.0@sha256:06b914d9ae91b6306d5001eb6b1f5d696bbd2a08b4db883ec77359da3d367324 \
     /rfo/__init__.py \
     /rfo/looks.py \
     /rfo/realhand.py \
     /rfo/referee.py \
     /rfo/scene.py \
     /rfo/task.py \
     /rfo/world.py \
     /srv/rfo/
