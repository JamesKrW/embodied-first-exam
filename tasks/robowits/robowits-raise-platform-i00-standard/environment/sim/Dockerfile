# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:robowits-0.1.1
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:331e2a74d699de500cbc63027d16e144db89c59b955cf6efbb3198ab1f69bcd8 \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:robowits-files-0.4.0@sha256:efc96cd1f591e69d0005326dec61c3644065943a9b2a04905fc2a68a4c400dca \
     /privileged/tests/rcb_robowits.py \
     /sim/server.py \
     /srv/
