FROM ghcr.io/mll-lab-nu/efe:dexhand-0.2.0@sha256:0d42994b83d95ab5301c592b35586550c9ee426490115e4b43f3b8cf8cc43350
COPY . /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/hands.py /privileged/tests/grade.py /privileged/environment/checker.py /privileged/environment/essential.py /privileged/environment/essential_articulated.py /privileged/environment/essential_held_joystick_scene.py /privileged/environment/essential_inhand.py /privileged/environment/essential_pen_regrasp_scene.py /privileged/environment/essential_rotary_gait_scene.py /privileged/environment/essential_rotary_multi.py /privileged/environment/essential_two_object_scene.py /privileged/environment/essential_visual.py /privileged/environment/replay.py /privileged/environment/runtime.py /privileged/environment/scene.py /privileged/environment/variants/9e58171df6/catalog.json /privileged/environment/variants/14a2b04dc9/wuji_reset.json /tests/
COPY --from=ghcr.io/mll-lab-nu/efe:dexhand-files-0.3.0@sha256:1c9e0841e0e2915ab6a6bd6962fd564f9eedf575176e7581dca394adf5e37b93 /privileged/environment/object_assets/essential_pen_regrasp/pen_base_color.png /privileged/environment/object_assets/essential_pen_regrasp/pen_body.obj /privileged/environment/object_assets/essential_pen_regrasp/pen_normal.png /privileged/environment/object_assets/essential_pen_regrasp/pen_orm.png /privileged/environment/object_assets/essential_pen_regrasp/pen_tail_collision.obj /privileged/environment/object_assets/essential_pen_regrasp/pen_tip_collision.obj /tests/object_assets/essential_pen_regrasp/
ENV PYTHONPATH=/tests
RUN chmod +x /tests/test.sh
WORKDIR /tests
# resources (PROTOCOL §6) # rcb-monitor
COPY --from=ghcr.io/mll-lab-nu/efe:monitor-0.5.0@sha256:69dcd5dd5dff08cfc35fc891e2563a374b6c4cc156be84ee88b9db456662f726 /opt/rcb_monitor/rcb_limits.py /tests/rcb_limits.py
COPY rcb_limits.json /tests/rcb_limits.json
