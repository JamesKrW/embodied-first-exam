# EmbodiedSWE base image (images/embodiedswe/: RoboLab's Isaac Sim 5.1 / Isaac Lab 2.3.2 recipe + robobench at a pinned
# commit, scrubbed, + its assets by sha256); built locally, not published (images/embodiedswe/BUILD_ONLY).
FROM ghcr.io/mll-lab-nu/efe-private:embodiedswe-0.1.0
COPY instance.json /app/instance.json
RUN mkdir -p /app/output
WORKDIR /app
