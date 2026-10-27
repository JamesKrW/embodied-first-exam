# EmbodiedSWE base image (images/embodiedswe/: RoboLab's Isaac Sim 5.1 / Isaac Lab 2.3.2 recipe + robobench at a pinned
# commit, scrubbed, + its assets by sha256); built locally, not published (images/embodiedswe/BUILD_ONLY). The tooling
# comes from the files image (images/embodiedswe-files/, source embodied_first_exam/benchmarks/embodiedswe/).
FROM ghcr.io/mll-lab-nu/efe-private:embodiedswe-0.2.0
COPY --from=ghcr.io/mll-lab-nu/efe:embodiedswe-files-0.1.0@sha256:07b70f9162c93e22541b236ef0903603567d947e3784e664c03d1f768315c388 /rcb_embodiedswe.py /esw_proxy.py /opt/efe/
COPY instance.json /app/instance.json
RUN mkdir -p /app/output
WORKDIR /app
