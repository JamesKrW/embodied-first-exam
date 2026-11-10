# Base image = TopoGym's simulator (see images/topogym/). Everything task-specific is copied here.
FROM ghcr.io/mll-lab-nu/efe:roboarc-2.2.0@sha256:96a0a895c0e4934adca39f7a370e67bf82bfb4db7ef049131a3ac0dc9d0f9057
# MuJoCo task: numpy / OpenBLAS SIMD masking, so every CPU class generates the frozen bytes
ENV NPY_DISABLE_CPU_FEATURES=AVX512F,AVX512CD,AVX512_SKX,AVX512_CLX,AVX512_CNL,AVX512_ICL,AVX512_SPR OPENBLAS_CORETYPE=Haswell
COPY runtime_binding.json runtime_binding.s1.json runtime_binding.s2.json seed /tmp/rcb-seed/
RUN python /opt/robotangle/stage_instance.py hedgehog-in-cage-06-i01 /app --binding /tmp/rcb-seed/runtime_binding.json --seed-file /tmp/rcb-seed/seed --purge && rm -rf /tmp/rcb-seed   # the frozen instance, from the image (images/roboarc/assets.json), checked against runtime_binding.json
COPY --from=ghcr.io/mll-lab-nu/efe:roboarc-files-0.5.0@sha256:8c65ad7d9650114e298c79a9c88641e849925182b4afa47a4099ded49b73dc5f /rcb_topogym.py /opt/tools/rcb_topogym.py
COPY --from=ghcr.io/mll-lab-nu/efe:roboarc-files-0.5.0@sha256:8c65ad7d9650114e298c79a9c88641e849925182b4afa47a4099ded49b73dc5f /render_bridge.py /opt/tools/render_bridge.py
COPY --from=ghcr.io/mll-lab-nu/efe:roboarc-files-0.5.0@sha256:8c65ad7d9650114e298c79a9c88641e849925182b4afa47a4099ded49b73dc5f /nyx_direct.py /opt/tools/nyx_direct.py
ENV PYTHONPATH=/opt/tools:/opt/topogym
# the longest trajectory the verifier replays within its timeout
ENV RCB_MAX_STEPS=200000
RUN mkdir -p /app/output
WORKDIR /app
