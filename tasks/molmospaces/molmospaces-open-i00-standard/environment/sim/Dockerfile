# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:molmospaces-0.2.5
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:molmospaces-files-0.3.0@sha256:9fafad0888023b251e737ddd4745cc99e527106168442e3dfdb1f4268dfe902a \
     /sim/finalize.sh \
     /privileged/environment/rcb_molmospaces.py \
     /sim/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:331e2a74d699de500cbc63027d16e144db89c59b955cf6efbb3198ab1f69bcd8 \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
