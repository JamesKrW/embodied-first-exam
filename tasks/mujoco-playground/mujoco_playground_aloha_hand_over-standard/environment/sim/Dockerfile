# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:mujoco-0.1.2
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:f8652e8f64c8da11b6c60b1acbdb76db5cb48813bd6ae17d11ecbabebe3fa184 /rcb1.py /srv/rcb1.py
COPY --from=ghcr.io/mll-lab-nu/efe:mujoco-playground-files-0.5.0@sha256:cda363caebdc631e68f07852ae5e78b7fb884f39310d380bad1aef611b267af3 \
     /privileged/environment/rcb_aloha.py \
     /sim/server_mj.py \
     /srv/
