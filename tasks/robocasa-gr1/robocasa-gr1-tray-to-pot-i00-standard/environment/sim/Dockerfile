# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:robocasa-gr1-0.1.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa-files-0.4.0@sha256:27cef7b3d1e57adb7d835fe8650a539f0354e5c8ff11be2a1115e540998c080c /privileged/environment/casa_tool.py /srv/casa_tool.py
COPY --from=ghcr.io/mll-lab-nu/efe:robocasa-files-0.4.0@sha256:27cef7b3d1e57adb7d835fe8650a539f0354e5c8ff11be2a1115e540998c080c \
     /sim/finalize.sh \
     /sim/server_casa.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-files-0.4.0@sha256:1dde7816f44b428dd7045c4883935decaec4f8a76360a3379dbe90a7fc90d7dc /rcb1.py /srv/rcb1.py
