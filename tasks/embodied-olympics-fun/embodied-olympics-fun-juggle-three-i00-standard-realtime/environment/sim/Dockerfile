# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.2@sha256:fa9954dbe06da7337d236c30640e7b2d8fe17a207404dbe9b42d450f9ca9a8a2 /rfo/tasks/juggle_three.py /srv/rfo_task.py
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.0@sha256:e2d97664330216cfc9359f88199033f94142e7e53505625de0ab0c3842379bbc \
     /rcb1.py \
     /realtime/rcb_realtime.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.4.0@sha256:45cab9387a91b8895f623ef91feac3f159afa29aa4dca9bb4294490927fe0160 \
     /rcb_rfo.py \
     /standard/rcb_service.py \
     /standard/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.2@sha256:fa9954dbe06da7337d236c30640e7b2d8fe17a207404dbe9b42d450f9ca9a8a2 \
     /rfo/__init__.py \
     /rfo/looks.py \
     /rfo/realhand.py \
     /rfo/referee.py \
     /rfo/scene.py \
     /rfo/task.py \
     /rfo/world.py \
     /srv/rfo/
