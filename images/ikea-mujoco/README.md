# ikea-mujoco

`ghcr.io/mll-lab-nu/efe:roboarc-ikea-<images/ikea-mujoco/VERSION>` is RoboARC's IKEA release image of that version,
byte for byte (the same manifest digest as RoboARC's own release; the tasks pin it by digest): MuJoCo 3.14.0 (the PyPI
wheel with the EPA horizon-overflow fix byte-patched in, `mujoco-3.14.0-epa/` here; CPU, float64; numpy 2.5.4, Python
3.12) and the twelve IKEA furniture scenes of RoboARC's IKEA suite (`tasks/roboarc/roboarc-ikea-<scene>-i00-{privileged,standard}`;
tasks/roboarc/README.md, "IKEA suite"). Every scene is a task in both modes. The privileged tasks' agent and verifier
images are built `FROM` it; it is also the physics process of the standard tasks' sidecar.

Each scene is self-contained in `/opt/ikea/scenes/<scene>/`: `scene.py` (`load()` returns the scene: `S.model`,
`S.data`, `S.step(n)` = n physics steps plus the scene's mechanism, `S.success()`), `scene.xml` with the two arms, the
scene's mechanism and success modules, `task.json`, `layout.json`, `assets/` (meshes). The image holds no task code: the
tooling (`rcb_ikea.py`) and the graders come from the files-only image `images/ikea-mujoco-files/`, and no robot demo or
direct-force reference is in it. The image also carries OSMesa, imageio with ffmpeg and Pillow (the verifier's replay
video), so the task images install nothing.

`sim/` is the standard tasks' sidecar image, `ghcr.io/mll-lab-nu/efe:roboarc-ikea-sim-<images/ikea-mujoco/sim/VERSION>`,
also RoboARC's release image byte for byte: this image unchanged as the physics process, plus Nyx (RoboARC's renderer) in
its own venv `/opt/nyx` (`sim/nyx-requirements.txt`: one wheel each by sha256), the GL / EGL / OpenGL / Vulkan loaders
with the NVIDIA ICD / EGL vendor files, and the studio HDRI.

| image | id | manifest digest |
|---|---|---|
| `ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0` | `sha256:9f9cbab8aa74…` | `sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b` |
| `ghcr.io/mll-lab-nu/efe:roboarc-ikea-sim-1.1.0` | `sha256:cbd929360257…` | `sha256:f22a1e9042c7877d33703960313cf548f2af7f15d6fcfa61db83315bfdacba1e` |

`Dockerfile` and `sim/Dockerfile` are RoboARC's recipes for these images (`sim/Dockerfile`'s `FROM` names the base under
its tag here, with the same digest); `requirements.lock` pins the base image's wheels (exact versions and sha256), and
the Debian packages come at exact versions from the Debian snapshot archive of 2026-10-19.

## Build source

The scene code is in git, `scenes/<scene>/` here. The meshes (`scenes/<scene>/assets/`, 2,078 files, about 40 MB per
scene) are not; `images/ikea-mujoco/assets.json` pins the sha256 of every file (per scene `files`) and their sources:
- `assets/panda/`, `assets/aloha/` (828 files): MuJoCo Menagerie (`franka_emika_panda/`, Apache-2.0; `aloha/`,
  BSD-3-Clause) at the commit in `upstream`, downloaded anonymously from GitHub;
- `assets/parts/` (1,250 files, the furniture parts: parameterized reconstructions drawn from the IKEA assembly manuals,
  not IKEA CAD): the Hugging Face dataset `repo` at `revision` (layout `<path_prefix><scene>/assets/parts/<file>`),
  the public `MLL-Lab/embodied-first-exam-public` (`ikea-mujoco/`, CC BY 4.0), downloaded anonymously.

Every privileged task's `tests/runtime_binding.json` and every standard task's `environment/sim/runtime_binding.json`
repeat their scene's pins, which the grader checks (`instance_exact`) together with the initial-state hash
(`instance_ok`); `images/ikea-mujoco/bindings.json` holds what each scene builds to in the image (the base image's id,
the initial-state hash, the speed limits).

What is staged per scene: `scene.py`, `scene.xml`, `task.json`, `layout.json`, `sw_cutover.json` (where present), the
scene's own Python modules (not `_*.py` scripts) and `assets/`; then `assets.json` (its `note` and `tasks`, the pins the
image carries at `/opt/ikea/assets.json`), `bytepatch_libmujoco.py` and `requirements.lock`. `stage_context.py` refuses
any file whose sha256 differs from the pins and any scene text that points at upstream material (PROTOCOL Appendix H.2);
the scenes' upstream provenance is kept outside this repository.

## Build (from the repository root)

```bash
images/ikea-mujoco/fetch_image_context.sh          # robot meshes from Menagerie, part meshes from the pinned dataset revision
                                                    # (public, no login), checked, staged with the scene
                                                    # code into images/ikea-mujoco/context/
DOCKER_BUILDKIT=1 docker build -f images/ikea-mujoco/Dockerfile -t ghcr.io/mll-lab-nu/efe:roboarc-ikea-$(cat images/ikea-mujoco/VERSION) images/ikea-mujoco/context
```

A local build installs the same packages and the same scene files, but its layers, and so its id and digest, differ
from the published image's, and `sim/Dockerfile` names the published base by digest: run the tasks on the published
images (`docker pull`), which need no build. The build needs network access to docker.io, snapshot.debian.org,
pypi.org (and download.pytorch.org, dl.polyhaven.org for `sim/`); the fetch to raw.githubusercontent.com and
huggingface.co; running needs none. Host: Linux x86_64, Docker; the base image needs no GPU, the sidecar an NVIDIA GPU
with Vulkan and EGL.

After a scene changes: `stage_context.py <tree> --pin`, rebuild, bump `VERSION`, refresh
`images/ikea-mujoco/bindings.json` (`python /opt/tools/rcb_ikea.py info --task <scene>` in the image, per scene), upload
the meshes as a new dataset revision and pin it, and re-render the tasks. A changed scene is a new task (PROTOCOL §2).
