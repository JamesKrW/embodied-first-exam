# RoboARC's IKEA image recipe (roboarc-ikea), copied unchanged below from the RoboARC repository's
# ikea/images/ikea-mujoco/Dockerfile at 1.1.0. Context: images/ikea-mujoco/context/, staged by images/ikea-mujoco/stage_context.py
# (fetch_image_context.sh; BUILD_CONTEXT). The tasks pin the published image, ghcr.io/mll-lab-nu/efe:roboarc-ikea-<VERSION>.
# roboarc-ikea: MuJoCo 3.14.0 (CPU, float64; the wheel with the EPA horizon fix byte-patched in) and the IKEA furniture
# scenes: the privileged tasks' agent and verifier image, and the physics of the standard tasks' sidecar (sim/).
# Built by images/ikea-mujoco/build.sh (README.md there): the context is images/ikea-mujoco/context/, which
# scripts/ikea_mujoco/stage_context.py writes (scenes/<scene>/ checked against scripts/ikea_mujoco/assets.json, assets.json,
# the EPA patch script and requirements.lock). No task code here: the tooling and the graders are copied by each task.
#
# Every input is pinned (since 1.1.0), and the image holds only what the tasks run:
#   - the Python base image by digest (Python 3.12.15, Debian 13);
#   - Debian packages at exact versions from the Debian snapshot archive of 2026-10-19 (the day the 1.0.0 images
#     installed them, so the same versions): OSMesa (libosmesa6) for the verifier's replay video, MuJoCo's renderer on
#     the CPU. Until 1.0.x each task's verifier image installed it at trial time from the live archive;
#   - wheels by sha256 (requirements.lock): mujoco 3.14.0, numpy 2.5.4 and what the graded runtime imports of mujoco's
#     dependencies (glfw, PyOpenGL, typing-extensions), and the replay video's imageio, imageio-ffmpeg and Pillow.
#     mujoco's other declared dependencies (absl-py, etils, fsspec, websockets, zipp: its viewer, studio and test code)
#     are left out; no bytecode caches (Python compiles in memory: PYTHONDONTWRITEBYTECODE below).
ARG PYTHON_BASE=python:3.12-slim@sha256:a6e34c598f2467ed0e9a8d349809fcd8b5c603269512df273a0bb1784edc11b1

# The scenes with their final modes, copied into the image as one layer
FROM ${PYTHON_BASE} AS scenes
COPY scenes/ /opt/ikea/scenes/
COPY assets.json /opt/ikea/assets.json
RUN chmod -R a+rX,go-w /opt/ikea

FROM ${PYTHON_BASE}
# No documentation pages for added packages (dpkg path-exclude, copyright files kept), no apt/dpkg logs or caches (they
# hold build times). The image keeps Debian's own source list.
ARG DEBIAN_SNAPSHOT=20261019T080000Z
RUN set -eux; printf '%s\n' 'path-exclude=/usr/share/doc/*' 'path-include=/usr/share/doc/*/copyright' \
      'path-exclude=/usr/share/man/*' 'path-exclude=/usr/share/info/*' 'path-exclude=/usr/share/lintian/*' \
      > /etc/dpkg/dpkg.cfg.d/roboarc-nodoc; \
    mv /etc/apt/sources.list.d/debian.sources /tmp/debian.sources; \
    for s in "debian trixie" "debian trixie-updates" "debian-security trixie-security"; do set -- $s; \
      echo "deb [check-valid-until=no] http://snapshot.debian.org/archive/$1/${DEBIAN_SNAPSHOT} $2 main"; done \
      > /etc/apt/sources.list.d/snapshot.list; \
    apt-get update; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libosmesa6=25.0.7-2+deb13u1; \
    rm /etc/apt/sources.list.d/snapshot.list; mv /tmp/debian.sources /etc/apt/sources.list.d/debian.sources; \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/*.bin /var/cache/ldconfig/aux-cache /var/cache/debconf/*-old \
      /var/lib/dpkg/*-old /var/log/apt/* /var/log/dpkg.log /var/log/alternatives.log
# Python packages: requirements.lock (exact versions and wheel sha256, no dependency resolution: the list is complete)
RUN --mount=type=bind,source=requirements.lock,target=/tmp/requirements.lock \
    PIP_DISABLE_PIP_VERSION_CHECK=1 PYTHONDONTWRITEBYTECODE=1 pip install --no-cache-dir --no-compile --no-deps \
      --require-hashes -r /tmp/requirements.lock
# MuJoCo 3.14.0 + the EPA horizon-overflow fix as a 14-byte patch of the wheel's own libmujoco.so
# (source/images/roboarc/mujoco-3.14.0-epa/: upstream 6d662aed; sha256 5e7623e3... -> 52e2a24e927ccac047b13576ff9fcbe455bc9a12b0d2e0bf1868bb7ca5be2e4d,
# asserted by the script; bit-identical to the stock wheel on every IKEA reference, no SIGSEGV on eket's crash reproducers)
RUN --mount=type=bind,source=bytepatch_libmujoco.py,target=/tmp/bytepatch_libmujoco.py \
    PYTHONDONTWRITEBYTECODE=1 python /tmp/bytepatch_libmujoco.py /usr/local/lib/python3.12/site-packages/mujoco/libmujoco.so.3.14.0 \
 && echo "52e2a24e927ccac047b13576ff9fcbe455bc9a12b0d2e0bf1868bb7ca5be2e4d  /usr/local/lib/python3.12/site-packages/mujoco/libmujoco.so.3.14.0" | sha256sum -c - \
 && PYTHONDONTWRITEBYTECODE=1 python -c "import numpy, mujoco; assert (numpy.__version__, mujoco.__version__) == ('2.5.4', '3.14.0')"
COPY --from=scenes /opt/ikea /opt/ikea
# numpy / OpenBLAS SIMD masking (as the RoboARC image): the scenes' mechanisms compute with numpy, whose AVX-512 and
# AVX2 reductions differ in the last bit; unmasked, eket's reference diverged on Zen 4 hosts before row 20000 and
# crashed MuJoCo (SIGSEGV at row 29396), while Intel AVX2 hosts replayed it. Masked, both classes give the same hashes.
ENV IKEA_SCENES=/opt/ikea/scenes OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 \
    NPY_DISABLE_CPU_FEATURES=AVX512F,AVX512CD,AVX512_SKX,AVX512_CLX,AVX512_CNL,AVX512_ICL,AVX512_SPR \
    OPENBLAS_CORETYPE=Haswell
WORKDIR /app
# Labels only (no filesystem change): build.sh passes the version.
ARG VERSION=dev
LABEL org.opencontainers.image.title="roboarc-ikea" \
      org.opencontainers.image.version="${VERSION}" \
      org.opencontainers.image.source="https://github.com/Embodied-First-Exam/RoboARC"
