FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/hands.py /privileged/tests/grade.py /privileged/environment/variants/2f2a9b1205/catalog.json /privileged/environment/checker.py /privileged/environment/essential.py /privileged/environment/essential_articulated.py /privileged/environment/essential_held_joystick_scene.py /privileged/environment/essential_inhand.py /privileged/environment/essential_pen_regrasp_scene.py /privileged/environment/essential_rotary_gait_scene.py /privileged/environment/essential_rotary_multi.py /privileged/environment/essential_two_object_scene.py /privileged/environment/essential_visual.py /privileged/environment/replay.py /privileged/environment/runtime.py /privileged/environment/scene.py /tests/
ENV PYTHONPATH=/tests
RUN chmod +x /tests/test.sh
WORKDIR /tests
# resource limits: the verifier step (from the monitor image) and its copy of the limits (docs/protocol/README.md §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.4.0@sha256:9d09fa2fe44ee6af5ccdff49827f005934c053f5dce34044e036c9facc3b343a /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
