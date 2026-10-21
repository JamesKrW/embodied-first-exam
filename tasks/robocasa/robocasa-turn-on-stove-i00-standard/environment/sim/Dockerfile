# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:robocasa-0.1.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa-sim-files-0.1.1@sha256:0afacac14656daa02142f80f61f7fb93e819f462fda73f68f0f9876919a5df75 \
     /privileged/environment/casa_tool.py \
     /sim/finalize.sh \
     /sim/server_casa.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.2@sha256:57c474cd8a011e77b8351e2d7aed463994fc4aa02ff1f465b80efb214ae4521a /rcb1.py /srv/rcb1.py
