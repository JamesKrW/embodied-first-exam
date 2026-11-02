# MetaWorld+ privileged task: the Meta-World image, the task's tooling and its frozen instance.
FROM ghcr.io/mll-lab-nu/efe:metaworld-0.1.0
COPY instance.npz /app/instance.npz
COPY --from=ghcr.io/mll-lab-nu/efe:metaworldplus-files-0.4.0-resetpose@sha256:d3831bd4fdee48fc1909abb3e9110e64f7055ac89919965f05e8492ed742df05 /privileged/environment/mw_tool.py /opt/tools/mw_tool.py
ENV PYTHONPATH=/opt/tools
RUN mkdir -p /app/output
WORKDIR /app
