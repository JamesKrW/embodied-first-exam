# Base image = the Embodied Olympics simulator (MuJoCo 3.3.7 on the CPU, the robot library with its Unitree G1 and H1-2, the walking
# and running policies; images/embodied-olympics-fun/). Everything task-specific is copied here: the frozen instance and this
# family's rules; the tooling and the world package's core come from the files-only image images/embodied-olympics-fun-files/.
FROM ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-0.2.1
COPY instance.json /app/instance.json
COPY rfo_task.py /opt/tools/
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rcb_rfo.py /opt/tools/rcb_rfo.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rfo/__init__.py /opt/tools/rfo/__init__.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rfo/scene.py /opt/tools/rfo/scene.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rfo/looks.py /opt/tools/rfo/looks.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rfo/world.py /opt/tools/rfo/world.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rfo/referee.py /opt/tools/rfo/referee.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rfo/task.py /opt/tools/rfo/task.py
COPY --from=ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-files-0.3.4@sha256:03440edb1a5bf4ed470ddf7e9bd9d47d08135fd3f34e62a10d1df494b0b1d998 /rfo/realhand.py /opt/tools/rfo/realhand.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
