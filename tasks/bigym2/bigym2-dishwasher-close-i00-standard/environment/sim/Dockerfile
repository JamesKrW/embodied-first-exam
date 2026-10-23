# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:bigym2-0.1.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.1@sha256:77cfe73759fc3961ef393bfa6f119d635a29ea905175a12e4001298018ec2e56 /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:bigym2-files-0.3.1@sha256:17b2f27405fe018fbddf4bf8f71003e4d73c3216f24c865f30e894c2f5643501 \
     /privileged/environment/rcb_bigym2.py \
     /sim/rcb_service.py \
     /sim/server.py \
     /srv/
