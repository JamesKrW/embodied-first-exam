# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:331e2a74d699de500cbc63027d16e144db89c59b955cf6efbb3198ab1f69bcd8 /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.5.0@sha256:3f0b9610b741f2807ff2c5b545c5287500de98ab1f7dcadc9f4780cd0a6bc42a \
     /privileged/environment/rcb_aloha.py \
     /sim/server_mj.py \
     /srv/
