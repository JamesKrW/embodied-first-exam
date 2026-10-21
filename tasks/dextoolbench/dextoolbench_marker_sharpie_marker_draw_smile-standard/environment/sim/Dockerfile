# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.2@sha256:57c474cd8a011e77b8351e2d7aed463994fc4aa02ff1f465b80efb214ae4521a /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:dextoolbench-sim-files-0.1.1@sha256:ebc574bacb4303980f67ab72f04a7df170066292e0f0b5bc21e54860054c3a9d \
     /privileged/environment/rcb_dtb.py \
     /sim/server_mj.py \
     /srv/
