# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.3.1@sha256:77cfe73759fc3961ef393bfa6f119d635a29ea905175a12e4001298018ec2e56 /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:dextoolbench-files-0.3.0@sha256:e15e01e5c682e09c96b9c6e2d48def61a483a1ff8d2df74b813c4d3d845847a9 \
     /privileged/environment/rcb_dtb.py \
     /sim/server_mj.py \
     /srv/
