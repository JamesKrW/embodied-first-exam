# roboarc

`ghcr.io/mll-lab-nu/efe:roboarc-<images/roboarc/VERSION>` is RoboARC's release image of that version, byte for byte (the
same manifest digest as RoboARC's own release; the tasks pin it by digest). The privileged agent and verifier images are
built `FROM` it, and the standard `sim` service runs it. `Dockerfile` is RoboARC's recipe at the release commit
(`images/roboarc/release.json` `release_commit`), copied unchanged below a three-line header. The image holds:

- Python 3.11.15 on Debian 12 (the official Python base image, pinned by digest);
- the simulator source (`/opt/topogym/topogym`, the pruned `topogym` package with its MuJoCo core) and its hash
  `/opt/topogym/SOURCE.json`, which every task's runtime binding names;
- the Python packages the simulator, the task tooling, the graders and the renderer use: MuJoCo 3.14.0 with a 14-byte
  patch of its `libmujoco.so` (the EPA fix), numpy 2.3.5, scipy 1.17.1, trimesh, coacd, the Nyx renderer `gs-nyx` 0.1.3,
  torch 2.11.0+cu128 (`nyx_direct.py` computes the camera poses with it) with the CUDA libraries it loads, the cloth
  packages newton 1.6.1 and warp-lang 1.18.0 and their Warp CPU kernels, compiled at build into `/opt/warp-cache`;
- the Debian packages they need (ffmpeg, the GL / EGL / Vulkan loaders, X11 client libraries) and, for the agent
  harness, curl, ca-certificates and ripgrep;
- the robot assets (`/opt/topogym-assets/robots`), Nyx's studio HDRI and wood maps, the appearance seeds' texture maps
  (nine CC0 Poly Haven texture sets, with a CREDITS.txt) and the ground plane's checker texture
  (`/opt/topogym-visuals`), the NVIDIA Vulkan ICD and EGL vendor files;
- every task's frozen MuJoCo instance and its appearance seeds (`/opt/robotangle/instances/<base>/`, seeds
  `.../seeds/<S>/`), pinned file by file in `assets.json`. `/opt/robotangle/stage_instance.py` (`stage_instance.py`
  here) copies a task's instance, or the seed's, next to its runtime binding and checks it (privileged `environment/`
  and `tests/` images; the standard `sim` entrypoint).

The task tooling (`rcb_topogym.py`, `nyx_direct.py`, `render_bridge.py`) is not in the image: the tasks take it from the
files-only image `images/roboarc-files/`.

- **Nyx.** The image contains Genesis AI's Nyx renderer (the `gs-nyx` wheels on PyPI). The benchmark owner has
  permission to redistribute it in this image.
- **Build source.** The build context is the RoboARC repository's `source/` at the release commit plus the frozen
  instances, which come from the private RoboARC asset dataset on Hugging Face (organization members only; `assets.json`
  pins the revision and every file's sha256), or from a local copy of the release's instance tree. Only the built image
  is public, and running the tasks needs only that image.

## Host

- Linux x86_64. Not WSL: Nyx needs the NVIDIA Vulkan driver inside the container.
- An NVIDIA GPU with RT cores and driver >= 580 with its graphics (GL/Vulkan) user-space libraries; GPUs without RT
  cores (e.g. A100, H100) cannot run Nyx (`nyx_direct.py` raises UnsupportedDevice), so they run no RoboARC task. Docker
  with the NVIDIA Container Toolkit, the default runtime left at `runc`.
- Exactly one GPU per container: Nyx crashes when two GPUs are visible. The task compose files reserve one GPU with a
  device reservation (`count: 1`, `capabilities: [gpu]`), without `runtime: nvidia`. NVIDIA_DRIVER_CAPABILITIES=all makes
  the toolkit mount the driver's graphics libraries, and the image's Vulkan ICD and EGL vendor files point the loaders at
  them. `VK_ICD_FILENAMES` names the image's ICD file: a toolkit in CDI mode also mounts the host's
  `/etc/vulkan/icd.d/nvidia_icd.json`, which would list the GPU twice.
- Physics runs on the CPU (MuJoCo, float64), except the blanket families' cloth step, which runs on the GPU on a
  deterministic solver.
- Running needs no network. The image is not cache-warmed: every container compiles its kernels on first use.

## Build (from the repository root)

```bash
RCB_ROBOARC_REPO=<RoboARC clone> images/roboarc/fetch_image_context.sh [<download dir>]   # images/roboarc/context/, checked
RCB_ROBOARC_REPO=<RoboARC clone> RCB_ROBOARC_INSTANCES=<dir> images/roboarc/fetch_image_context.sh   # or a local instance tree
docker build -f images/roboarc/Dockerfile -t ghcr.io/mll-lab-nu/efe:roboarc-$(cat images/roboarc/VERSION) images/roboarc/context
```

`assets.py check <dir>` requires exactly the pinned files, each with its sha256, before the context is assembled. The
appearance seeds are pinned in `assets.json` but not yet in the dataset revision it names: until that revision is
uploaded, build from a local copy of the release's instance tree (`RCB_ROBOARC_INSTANCES`). The build needs Docker with
BuildKit, about 25 GB of free disk and network access to Docker Hub, snapshot.debian.org, pypi.org,
download.pytorch.org and dl.polyhaven.org. `scripts/build_images.sh` builds it from `images/roboarc/BUILD_CONTEXT` once
the context exists, and skips it otherwise.

## Check a build

The image must install exactly the runtime every task is bound to:

```bash
docker run --rm ghcr.io/mll-lab-nu/efe:roboarc-$(cat images/roboarc/VERSION) python -c 'import json; from topogym.simulator.frozen_binding import installed_runtime; print(json.dumps(installed_runtime(), sort_keys=True))' > /tmp/roboarc_runtime.json
python3 -c 'import json, sys; sys.exit(0 if json.load(open("/tmp/roboarc_runtime.json")) == json.load(open("images/roboarc/release.json"))["runtime"] else "image differs from the tasks runtime binding: do not run")'
```

The runtime is the simulator source's hash (`SOURCE.json`, re-hashed from the installed files), the Python version and
the versions of torch, numpy, scipy, mujoco, newton and warp-lang. Every session checks it again at start
(`runtime_binding_ok` in reward.json) and refuses a different runtime.
