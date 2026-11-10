# RoboARC's release image recipe, copied unchanged below from the RoboARC repository's source/images/roboarc/Dockerfile at
# the release commit (images/roboarc/release.json release_commit). Context: images/roboarc/context/, prepared by
# images/roboarc/fetch_image_context.sh (BUILD_CONTEXT). The published image is ghcr.io/mll-lab-nu/efe:roboarc-<VERSION>.
# The RoboARC release image, ghcr.io/mll-lab-nu/roboarc:<images/roboarc/VERSION> (images/roboarc/README.md).
# Built by images/roboarc/build.sh from source/ (source/.dockerignore keeps the context to the files this recipe copies),
# with the frozen MuJoCo instances as the named build context `instances` (empty: the image without instances).
#
# Since 1.1.0 the image is built from the Python base image directly. Up to 1.0.x it was a delta on the RoboTangle
# 0.2.22 image (Genesis 1.2.1, libuipc), whose Genesis and libuipc no RoboARC task used; 1.1.0 installs the same
# operating system and the same wheels without them (docs/history/GENESIS_REMOVAL.md):
#   - the same Python base image, pinned by digest (Debian 12, glibc 2.36, Python 3.11.15);
#   - the same Debian packages at the same versions, from the Debian snapshot of the day the 0.2.22 base installed them;
#   - the same wheels: runtime.lock is the 0.2.22 base's uv lock (versions and sha256) without Genesis, libuipc and the
#     packages only they needed; numpy 2.3.5 / scipy 1.17.1 (their OpenBLAS kernels decide frozen asset bytes: the task
#     containers set the SIMD masking, scripts/roboarc/mk_tasks.py MUJOCO_ENV), mujoco 3.14.0 (with the EPA byte patch
#     below), torch 2.11.0+cu128 (nyx_direct.py computes the camera poses with it), gs-nyx 0.1.3 (the Nyx renderer),
#     trimesh, coacd, scipy, ...; newton 1.6.1 and warp-lang 1.18.0 for the VBD cloth (vbd-requirements.txt).
# Build: images/roboarc/build.sh (the release build, with the runtime check).
ARG PYTHON_BASE=python:3.11.15-slim-bookworm@sha256:d29f48a31a8b408ed19272ca1e7b10ebae13b240a27e862d3d4217c528e2e0c3
# uv installs the wheels (as it installed them in the 0.2.22 base); it is a build tool only and stays in this stage
FROM ${PYTHON_BASE} AS uv
RUN echo "uv==0.11.30 --hash=sha256:cc28cb55c2b3c80a26ea374a172fec70b0561ada211e6fb23936ccea3ecb80b2" > /tmp/uv.txt \
 && pip install --no-cache-dir --require-hashes -r /tmp/uv.txt

FROM ${PYTHON_BASE} AS unsquashed
# Debian packages, each at the version the 0.2.22 base installed (2026-09-30), from the snapshot archive of that day, so a
# later point release cannot change a library; their dependencies resolve to the same versions. What the runtime uses:
# ffmpeg / ffprobe (the replay videos), the GL / EGL / OpenGL / Vulkan loaders (Nyx, MuJoCo's depth raster), glib, X11
# client libraries, libgomp, liblzma5 (the base image's, upgraded as 0.2.22 had it); curl, ca-certificates and ripgrep
# for the agent harness inside the privileged container.
# No documentation pages (dpkg path-exclude, copyright files kept), no apt/dpkg logs or caches (they hold build times). The
# image keeps Debian's own source list.
ARG DEBIAN_SNAPSHOT=20260930T230000Z
RUN set -eux; printf '%s\n' 'path-exclude=/usr/share/doc/*' 'path-include=/usr/share/doc/*/copyright' \
      'path-exclude=/usr/share/man/*' 'path-exclude=/usr/share/info/*' > /etc/dpkg/dpkg.cfg.d/roboarc-nodoc; \
    mv /etc/apt/sources.list.d/debian.sources /tmp/debian.sources; \
    for s in "debian bookworm" "debian bookworm-updates" "debian-security bookworm-security"; do set -- $s; \
      echo "deb [check-valid-until=no] http://snapshot.debian.org/archive/$1/${DEBIAN_SNAPSHOT} $2 main"; done \
      > /etc/apt/sources.list.d/snapshot.list; \
    apt-get update; \
    apt-get install -y --no-install-recommends ffmpeg=7:5.1.9-0+deb12u1 libegl1=1.6.0-1 libgl1=1.6.0-1 \
      libopengl0=1.6.0-1 libglib2.0-0=2.74.6-2+deb12u9 libgomp1=12.2.0-14+deb12u1 libvulkan1=1.3.239.0-1 \
      libx11-6=2:1.8.4-2+deb12u2 libxext6=2:1.3.4-1+b1 libxrender1=1:0.9.10-1.1 \
      liblzma5=5.4.1-1+deb12u2 curl=7.88.1-10+deb12u15 ca-certificates=20250419~deb12u1 ripgrep=13.0.0-4+b2; \
    rm /etc/apt/sources.list.d/snapshot.list; mv /tmp/debian.sources /etc/apt/sources.list.d/debian.sources; \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/*.bin /var/cache/fontconfig/* /var/cache/ldconfig/aux-cache \
      /var/log/apt/* /var/log/dpkg.log /var/log/alternatives.log
ENV TOPOGYM_CUDA_FLAVOR=cu128 PYTHONUNBUFFERED=1 PIP_DISABLE_PIP_VERSION_CHECK=1 \
    TOPOGYM_VISUAL_ASSET_ROOT=/opt/topogym-visuals TOPOGYM_ASSET_ROOT=/opt/topogym-assets \
    NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all
# Python packages: runtime.lock (exact versions and wheel sha256, no dependency resolution: the list is complete)
COPY images/roboarc/runtime.lock /opt/requirements/runtime.lock
RUN --mount=type=bind,from=uv,source=/usr/local/bin/uv,target=/usr/local/bin/uv \
    uv pip install --system --no-cache --no-deps --require-hashes --torch-backend "${TOPOGYM_CUDA_FLAVOR}" \
      -r /opt/requirements/runtime.lock \
 && rm -f /tmp/uv-*.lock \
 && PYTHONDONTWRITEBYTECODE=1 python -c "import mujoco, coacd, gs_nyx, torch; assert mujoco.__version__ == '3.14.0', mujoco.__version__"
# The pip-provided CUDA libraries on the loader's path (as in 0.2.22), independent of torch's import order.
RUN python -c 'import site; from pathlib import Path; paths=sorted({str(p) for s in site.getsitepackages() for p in (Path(s)/"nvidia").rglob("lib") if p.is_dir()}); assert paths; Path("/etc/ld.so.conf.d/topogym-cuda.conf").write_text("\n".join(paths)+"\n")' \
 && ldconfig && rm -f /var/cache/ldconfig/aux-cache
# The VBD cloth's packages (exact versions and wheel hashes; newton needs only warp-lang, warp-lang only numpy)
COPY images/roboarc/vbd-requirements.txt /tmp/vbd-requirements.txt
RUN python -m pip install --no-cache-dir --no-deps --require-hashes -r /tmp/vbd-requirements.txt && rm /tmp/vbd-requirements.txt \
 && python -c "from importlib.metadata import version as v; assert (v('newton'), v('warp-lang')) == ('1.6.1', '1.18.0')"
# Nyx's studio HDRI, the table's wood maps and the appearance seeds' texture maps (topogym/world/look.py; CC0, Poly Haven;
# sha256-checked downloads, their CREDITS.txt), and the ground plane's checker
# texture nyx_direct.py draws (genesis-world 1.2.1's textures/checker.png, Apache-2.0, source/assets/textures/README.md).
COPY images/roboarc/prepare_nyx_studio.py /tmp/prepare_nyx_studio.py
RUN python /tmp/prepare_nyx_studio.py && rm /tmp/prepare_nyx_studio.py
COPY assets/textures/genesis/ /opt/topogym-visuals/genesis/
RUN chmod -R a+rX /opt/topogym-visuals
# Graphics: the NVIDIA Vulkan ICD and EGL vendor files. The tasks pass one GPU with a compose device reservation and no
# `runtime: nvidia`; NVIDIA_DRIVER_CAPABILITIES=all makes the container toolkit mount the driver's graphics libraries,
# and these files point the Vulkan and EGL loaders at them (Nyx renders through Vulkan). VK_ICD_FILENAMES names the
# image's file: a toolkit in CDI mode also mounts the host's ICD json, which would list the GPU twice.
RUN mkdir -p /usr/share/vulkan/icd.d /usr/share/glvnd/egl_vendor.d && \
    printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0","api_version":"1.3.0"}}\n' > /usr/share/vulkan/icd.d/nvidia_icd.json && \
    printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}\n' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
ENV VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json
# One cache directory for every user (agents and verifiers run as root, the standard sim service may not): the asset cache
# and the NVIDIA shader cache follow XDG_CACHE_HOME, the CUDA JIT cache CUDA_CACHE_PATH; /data for the sim service.
ENV XDG_CACHE_HOME=/opt/rcb-cache CUDA_CACHE_PATH=/opt/rcb-cache/nv/ComputeCache
RUN mkdir -p /opt/rcb-cache /data && chmod 1777 /opt/rcb-cache /data
# the simulator source (pruned: no solutions; SOURCE.json is its hash, which every task's runtime binding names)
ENV PYTHONPATH=/opt/topogym TOPOGYM_SOURCE=/opt/topogym/SOURCE.json
COPY topogym/ /opt/topogym/topogym/
COPY images/topogym/prune.py /opt/prune.py
RUN python /opt/prune.py /opt/topogym && rm /opt/prune.py && chmod -R a+rX /opt/topogym
COPY assets/robots/ /opt/topogym-assets/robots/
RUN chmod -R a+rX /opt/topogym-assets
# The VBD cloth's Warp CPU kernels, compiled into the read-only cache with the pinned target of
# topogym/world/mujoco/warp_cpu.py (docs/simulator/VBD_CLOTH.md)
ENV WARP_CACHE_PATH=/opt/warp-cache
COPY images/roboarc/warp_precompile.py /tmp/warp_precompile.py
RUN python /tmp/warp_precompile.py && rm /tmp/warp_precompile.py && chmod -R a+rX,a-w /opt/warp-cache
# Every task's frozen MuJoCo instance (instance.json + instance_assets/), from the named build context `instances`, with
# its appearance seeds under <base>/seeds/<S>/ (docs/RUNNING.md "Seeds"); a task stages its own (its seed's, with
# --seed-file) with /opt/robotangle/stage_instance.py and checks it against its runtime binding.
COPY images/roboarc/stage_instance.py /opt/robotangle/stage_instance.py
RUN --mount=type=bind,from=instances,target=/tmp/mj-instances \
    set -e; mkdir -p /opt/robotangle/instances; for d in /tmp/mj-instances/*/; do [ -d "$d" ] || continue; b=$(basename "$d"); \
      for i in "$d" "$d"seeds/*/; do [ -d "$i" ] || continue; test -f "$i/instance.json"; \
        python -c "import json, sys; assert json.load(open(sys.argv[1]))['simulator']['physics_engine'] == 'mujoco'" "$i/instance.json"; done; \
      cp -a "$d" "/opt/robotangle/instances/$b"; echo "MuJoCo instance $b $(ls "$d"seeds 2>/dev/null | tr '\n' ' ')"; done; \
    chmod -R a+rX /opt/robotangle
# MuJoCo 3.14.0 + the EPA horizon-overflow fix (RoboARC 0.2.1): a 14-byte patch of the wheel's own libmujoco.so
# (mujoco-3.14.0-epa/, docs/simulator/MUJOCO_EPA_PATCH.md, upstream 6d662aed; sha256 5e7623e3... -> 52e2a24e..., the
# script asserts both and every original byte).
COPY images/roboarc/mujoco-3.14.0-epa/bytepatch_libmujoco.py /tmp/bytepatch_libmujoco.py
RUN PYTHONDONTWRITEBYTECODE=1 python /tmp/bytepatch_libmujoco.py /usr/local/lib/python3.11/site-packages/mujoco/libmujoco.so.3.14.0 \
 && rm /tmp/bytepatch_libmujoco.py \
 && echo "52e2a24e927ccac047b13576ff9fcbe455bc9a12b0d2e0bf1868bb7ca5be2e4d  /usr/local/lib/python3.11/site-packages/mujoco/libmujoco.so.3.14.0" | sha256sum -c -
WORKDIR /app

# The image that ships: one filesystem layer. FROM scratch keeps no configuration: every field of the stage above is
# declared again, value for value (the Python base image's environment, then this file's).
FROM scratch
COPY --from=unsquashed / /
ENV PATH=/usr/local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
    LANG=C.UTF-8 \
    GPG_KEY=A035C8C19219BA821ECEA86B64E628F8D684696D \
    PYTHON_VERSION=3.11.15 \
    PYTHON_SHA256=272179ddd9a2e41a0fc8e42e33dfbdca0b3711aa5abf372d3f2d51543d09b625 \
    TOPOGYM_CUDA_FLAVOR=cu128 \
    PYTHONUNBUFFERED=1 \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    TOPOGYM_VISUAL_ASSET_ROOT=/opt/topogym-visuals \
    TOPOGYM_ASSET_ROOT=/opt/topogym-assets \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    XDG_CACHE_HOME=/opt/rcb-cache \
    CUDA_CACHE_PATH=/opt/rcb-cache/nv/ComputeCache \
    PYTHONPATH=/opt/topogym \
    TOPOGYM_SOURCE=/opt/topogym/SOURCE.json \
    WARP_CACHE_PATH=/opt/warp-cache
WORKDIR /app
CMD ["python3"]
# Labels only (no filesystem change): images/roboarc/build.sh passes the release version.
ARG RELEASE_VERSION=dev
LABEL org.opencontainers.image.title="RoboARC" \
      org.opencontainers.image.version="${RELEASE_VERSION}" \
      org.opencontainers.image.source="https://github.com/mll-lab-nu/RoboARC"
