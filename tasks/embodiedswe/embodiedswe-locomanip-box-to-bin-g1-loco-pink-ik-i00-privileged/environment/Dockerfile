# EmbodiedSWE base image (images/embodiedswe/: RoboLab's Isaac Sim 5.1 / Isaac Lab 2.3.2 recipe + robobench at a pinned
# commit, scrubbed, + its assets by sha256); built locally, not published (images/embodiedswe/BUILD_ONLY). The tooling
# comes from the files image (images/embodiedswe-files/, source embodied_first_exam/benchmarks/embodiedswe/).
FROM ghcr.io/mll-lab-nu/efe-private:embodiedswe-0.3.0
COPY --from=ghcr.io/mll-lab-nu/efe:embodiedswe-files-0.2.0@sha256:adbb103b7b91043b83339530224d912ca0eb66993064779e0e532b51175a162e /rcb_embodiedswe.py /esw_proxy.py /opt/efe/
COPY instance.json /app/instance.json
RUN mkdir -p /app/output
WORKDIR /app
