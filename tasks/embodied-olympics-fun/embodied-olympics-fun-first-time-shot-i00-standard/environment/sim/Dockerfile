# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rfo/tasks/first_time_shot.py /srv/rfo_task.py
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.1@sha256:77cfe73759fc3961ef393bfa6f119d635a29ea905175a12e4001298018ec2e56 /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 \
     /rcb_rfo.py \
     /standard/rcb_service.py \
     /standard/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 \
     /rfo/__init__.py \
     /rfo/looks.py \
     /rfo/realhand.py \
     /rfo/referee.py \
     /rfo/scene.py \
     /rfo/task.py \
     /rfo/world.py \
     /srv/rfo/
