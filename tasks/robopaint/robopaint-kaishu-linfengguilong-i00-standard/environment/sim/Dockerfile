# The `sim` sidecar's image (environment/docker-compose.yaml builds it): its base image, this directory at /srv, and at
# their paths under /srv the files it shares with other tasks, from pinned files-only images (images/<name>/
# FILES.json, scripts/files_image.py). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:robopaint-0.2.0
COPY . /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:robopaint-files-0.3.0@sha256:ffcfa94d02192f1e999c8016e814fb4c4dbbd6c9aa65f22bfc75bc9931e3238a \
     /sim/ARPHICPL.TXT \
     /sim/variants/49c8421692/NOTICE-exemplar.md \
     /sim/finalize.sh \
     /rcb_brush.py \
     /rcb_brush_score.py \
     /rcb_brush_strict.py \
     /sim/server.py \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:standard-sim-files-0.1.3@sha256:48ed026a31b72a54269db520e3172bace45552d08887a5776d72f531834144f5 \
     /rcb1.py \
     /rcb_limited.py \
     /run.sh \
     /srv/
COPY --from=ghcr.io/mll-lab-nu/efe:robopaint-pictures-files-0.1.0@sha256:3ce92526aa79918994ccb934390221e1dd6c3f4364109a40e5746140def6062e /kaishu-linfengguilong-i00/target.png /srv/target.png
