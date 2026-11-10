# Separate verifier sandbox (PROTOCOL §4): the same image with the verifier's own copies of the tooling and the scene's
# binding. Harbor uploads only the declared artifacts (/app/output/), so nothing the agent changed in its container
# reaches the verdict.
FROM ghcr.io/mll-lab-nu/efe:roboarc-ikea-1.1.0@sha256:2c56d9226aaf97a325b610d464a1cd81e87db1a6285e923539397f493e21136b
# Replay video (PROTOCOL §4; evidence only): MuJoCo's renderer on CPU software OpenGL (OSMesa) and ffmpeg, both in the
# base image (pinned there; images/ikea-mujoco/Dockerfile), so building this image installs nothing.
ENV MUJOCO_GL=osmesa PYOPENGL_PLATFORM=osmesa
COPY video_view.json test.sh runtime_binding.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:ikea-mujoco-files-0.5.0@sha256:fdebfb286aacaf4e9b5b5391c213f7800f655e1f8f3e5665e681020ad7a0824a /privileged/environment/rcb_ikea.py /privileged/tests/grade.py /privileged/tests/render_video.py /tests/
RUN chmod +x /tests/test.sh && mkdir -p /app/output /logs/verifier
ENV PYTHONPATH=/tests
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:ecc5f24f1516ca11711d7c2d3ac07c89f79cd9220812c1244498ec71e352402a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
