# The `sim` sidecar's image (environment/docker-compose.yaml builds it; the practice service and the practice player
# run from it too): the sport's image (Embodied Olympics' generated sim, practice world and practice player of
# tabletennis_g1, g1tt-10+core-0.5), then this directory at /srv (Embodied First Exam's patches of core/, listed in
# ci_checks/benchmarks/embodied-olympics-match/exported.toml). .dockerignore keeps the build's own two files out of /srv.
FROM ghcr.io/mll-lab-nu/efe:embodied-olympics-match-tabletennisg1-0.1.0
COPY . /srv/
