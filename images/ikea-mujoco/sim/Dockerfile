# RoboARC's IKEA sidecar recipe (roboarc-ikea-sim), copied from the RoboARC repository's ikea/images/ikea-mujoco/sim/Dockerfile
# at 1.1.0, unchanged except its FROM, which names the same base image (same digest) under its tag here. The tasks pin the
# published image, ghcr.io/mll-lab-nu/efe:roboarc-ikea-sim-<sim/VERSION>.
# roboarc-ikea-sim: the `sim` sidecar image of the IKEA MuJoCo STANDARD-mode tasks (tasks/roboarc-ikea-<task>-i00-standard).
# = roboarc-ikea (plain MuJoCo 3.14.0 CPU float64 with the EPA-patched libmujoco.so + numpy 2.5.4 + the frozen scenes +
# imageio/ffmpeg, Pillow and PyOpenGL, unchanged: the physics process) + what the robot-as-a-service layer needs to render:
#   - Nyx, RoboARC's renderer, in its OWN interpreter environment, the venv /opt/nyx (python 3.12 of the base, separate
#     site-packages): gs-nyx 0.1.3 needs numpy < 2.4, the physics keeps numpy 2.5.4, so the sim service renders every RGB
#     image in a separate render process (scripts/ikea_mujoco/standard/ikea_nyx.py) fed the scene once and per-frame poses
#     over a pipe. The venv's packages are RoboARC's exact versions, one wheel each by sha256 (nyx-requirements.txt):
#     gs-nyx 0.1.3, torch 2.11.0+cu128 with the CUDA 12.8 libraries it loads (nyx_direct.py computes the camera poses
#     with it), numpy 2.3.5, scipy 1.17.1, trimesh 5.1.0 (+ networkx), mujoco 3.14.0 (stock wheel, only to read the compiled
#     scene's visual geometry; it never steps physics) with PyOpenGL;
#   - RoboARC's studio HDRI (Poly Haven studio_small_09, CC0; the URL and sha256 RoboARC's image uses);
#   - the GL / EGL / OpenGL / Vulkan loaders (Debian, exact versions from the snapshot archive of 2026-10-19, as the 1.0.0
#     image) and RoboARC's NVIDIA Vulkan ICD / EGL vendor files: Nyx renders through Vulkan; MuJoCo's EGL raster draws the
#     depth images.
# Built by images/ikea-mujoco/build.sh FROM the base image by digest: the build passes the base it has just built
# (reproducibly, so with this digest) as the named context of this reference; a build that gives the base another digest
# stops there.
FROM ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b
ARG DEBIAN_SNAPSHOT=20261019T080000Z
RUN set -eux; mv /etc/apt/sources.list.d/debian.sources /tmp/debian.sources; \
    for s in "debian trixie" "debian trixie-updates" "debian-security trixie-security"; do set -- $s; \
      echo "deb [check-valid-until=no] http://snapshot.debian.org/archive/$1/${DEBIAN_SNAPSHOT} $2 main"; done \
      > /etc/apt/sources.list.d/snapshot.list; \
    apt-get update; \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends libegl1=1.7.0-1+b2 libgl1=1.7.0-1+b2 \
      libopengl0=1.7.0-1+b2 libglvnd0=1.7.0-1+b2 libvulkan1=1.4.309.0-1; \
    rm /etc/apt/sources.list.d/snapshot.list; mv /tmp/debian.sources /etc/apt/sources.list.d/debian.sources; \
    rm -rf /var/lib/apt/lists/* /var/cache/apt/*.bin /var/cache/ldconfig/aux-cache /var/cache/debconf/*-old \
      /var/lib/dpkg/*-old /var/log/apt/* /var/log/dpkg.log /var/log/alternatives.log
# The Nyx venv: its packages, no bytecode caches; its pip is removed once they are installed
COPY nyx-requirements.txt /opt/nyx-requirements.txt
RUN python -m venv /opt/nyx \
 && PIP_DISABLE_PIP_VERSION_CHECK=1 /opt/nyx/bin/pip install --no-cache-dir --no-compile --no-deps --require-hashes \
      -r /opt/nyx-requirements.txt \
 && /opt/nyx/bin/python -m pip uninstall -y -q pip \
 && /opt/nyx/bin/python -c "import numpy, mujoco, trimesh, torch, gs_nyx.nyx_py_sdk; assert (numpy.__version__, mujoco.__version__, trimesh.__version__, torch.__version__) == ('2.3.5', '3.14.0', '5.1.0', '2.11.0+cu128')"
RUN mkdir -p /opt/nyx-assets \
 && PYTHONDONTWRITEBYTECODE=1 python -c "import urllib.request; urllib.request.urlretrieve('https://dl.polyhaven.org/file/ph-assets/HDRIs/hdr/2k/studio_small_09_2k.hdr', '/opt/nyx-assets/studio_small_09_2k.hdr')" \
 && echo "36724313c0fc66dab126ff90f081b85a0b1ef47be65eda1417bd20b033f0573f  /opt/nyx-assets/studio_small_09_2k.hdr" | sha256sum -c - \
 && printf 'Studio Small 09 by Sergej Majboroda / Poly Haven\nhttps://polyhaven.com/a/studio_small_09\nLicense: CC0\n' > /opt/nyx-assets/CREDITS.txt \
 && chmod -R a+rX /opt/nyx-assets
# NVIDIA_DRIVER_CAPABILITIES=all (compose) makes the container toolkit mount the driver's graphics libraries; these files
# point the Vulkan and EGL loaders at them (as RoboARC's simulator image does). The toolkit's CDI mode also mounts the host's
# ICD json, which would list the GPU twice: VK_ICD_FILENAMES names ours only.
RUN mkdir -p /usr/share/vulkan/icd.d /usr/share/glvnd/egl_vendor.d && \
    printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0","api_version":"1.3.0"}}\n' > /usr/share/vulkan/icd.d/nvidia_icd.json && \
    printf '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}\n' > /usr/share/glvnd/egl_vendor.d/10_nvidia.json
ENV MUJOCO_GL=egl PYOPENGL_PLATFORM=egl NVIDIA_DRIVER_CAPABILITIES=all VK_ICD_FILENAMES=/usr/share/vulkan/icd.d/nvidia_icd.json \
    RCB_NYX_PYTHON=/opt/nyx/bin/python RCB_NYX_ENV_MAP=/opt/nyx-assets/studio_small_09_2k.hdr
ARG VERSION=dev
LABEL org.opencontainers.image.title="roboarc-ikea-sim" \
      org.opencontainers.image.version="${VERSION}" \
      org.opencontainers.image.source="https://github.com/Embodied-First-Exam/RoboARC"
