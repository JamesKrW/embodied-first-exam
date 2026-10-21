# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:robocasa-gr1-0.1.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa-files-0.1.2@sha256:5d0e5aa691ff90b770ef62a8ee04829e838bf8c4b4744f06253a2f9f10654de9 /privileged/environment/casa_tool.py /srv/casa_tool.py
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa-gr1-sim-files-0.1.1@sha256:78683e4934958f003961a37d9f3da2d37b2f8fa0d091fc0aa90be496466768fb \
     /sim/finalize.sh \
     /sim/server_casa.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 /rcb1.py /srv/rcb1.py
