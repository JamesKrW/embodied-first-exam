"""sweeper-jump: three tuck jumps over the boom, closed loop on privileged state (the boom's angle and speed, the
pelvis), through `walk`, `joints` and `lowcmd` rows (after the long-rope reference controller). Between jumps the
walking policy keeps the balance, brings the pelvis back over the podium's centre and turns the robot to face the
oncoming boom (so that it passes under both feet at once); about 2 s before the boom arrives, at a moment of double
support with the trunk nearly upright, the legs blend into a crouch whose centre of mass is over the soles (`joints`,
ankles and hip rolls corrected by the pelvis' pitch, roll, velocity and drift); push_lead s before the boom reaches
the robot both legs extend at once with raised gains (`lowcmd`), tuck in the air (hips and knees bent: the feet go
up), reach for a landing crouch, hold it at the touchdown with landing gains, and hand back to the walking policy.
After the third jump the walking policy holds the stand. The numbers come from a random local search over perturbed
starts and +-3 % of its own gains (tests/sw_rrs.py). Measured: soles 4-8 cm over the boom, success at 21.1 s; 9 of 11
perturbed starts and 15 of 20 runs with its own gains varied by 3 % succeed (the landing on the 0.9 m podium is the
weak point)."""
import math

import numpy as np

P = dict(face=math.pi / 2, roll_ok=0.0345, rate_ok=0.7575, kwalk=0.9148, crouch_lead=2.0906, kc_land=0.9017,
         kc=0.9909, push_lead=0.2773, blend=0.5247, ext_h=0.0229, ext_k=0.2458, ext_a=0.2967, kp_push=2.6127,
         tuck_h=-1.406, tuck_k=1.912, tuck_a=-0.2493, tuck_s=0.2782, kp_land=0.788, kd_land=2.3114, land_hold=0.4467,
         kpitch=-0.1585, kvx=0.9444, kxw=0.0048, kpitch_push=0.5506, kvx_push=0.0351, kxp=0.3163, ky=-0.1967,
         kroll=-0.0531, kvy=-0.004, kdpitch=0.0473, air_min=0.02, bias=0.0, walk_after=1.0, min_walk=0.9602)


def _com_ahead(w, legs, upper):
    import mujoco
    m, r = w.m, w.robot
    d2 = mujoco.MjData(m)
    d2.qpos[:] = w.d.qpos
    h, k, a = legs
    for s in (0, 6):
        d2.qpos[r.qadr[s + 0]], d2.qpos[r.qadr[s + 3]], d2.qpos[r.qadr[s + 4]] = h, k, a
    d2.qpos[r.qadr[12:]] = upper
    th = -(h + k + a)
    d2.qpos[r.fq + 3:r.fq + 7] = [np.cos(th / 2), 0, np.sin(th / 2), 0]     # (the forward axis taken as +x)
    mujoco.mj_kinematics(m, d2)
    mujoco.mj_comPos(m, d2)
    feet = [g for g in range(m.ngeom) if m.geom(g).name.startswith(("left_ankle_roll_link_sc", "right_ankle_roll_link_sc"))]
    return float(d2.subtree_com[r.pelvis][0]) - float(np.mean(d2.geom_xpos[feet][:, 0]))


def balanced_crouch(w, knee, upper, bias=0.0):
    best = None
    for a in np.linspace(-0.85, 0.2, 43):
        lo, hi = -2.4, 0.3
        for _ in range(40):
            h = (lo + hi) / 2
            lo, hi = (h, hi) if _com_ahead(w, (h, knee, a), upper) - bias > 0 else (lo, h)
        if abs(_com_ahead(w, (h, knee, a), upper) - bias) < 0.004 and (best is None or abs(h + knee + a) < best[0]):
            best = (abs(h + knee + a), (float(h), knee, float(a)))
    return best[1]


def solve(s, p=None):
    p = {**P, **(p or {})}
    w = s.world
    r, NJ = w.robot, w.L.NJ
    task = w.task
    q0 = r.arb.target().copy()
    up = r.rest[12:].copy()
    # the crouch is computed for the robot facing +x (the kinematics only): a copy of the state with the yaw removed
    qsave = w.d.qpos.copy()
    w.d.qpos[r.fq + 3:r.fq + 7] = [1, 0, 0, 0]
    C = balanced_crouch(w, p["kc"], up, p["bias"])
    C2 = balanced_crouch(w, p["kc_land"], up, p["bias"])
    w.d.qpos[:] = qsave
    z = np.zeros(NJ)
    kp_push = np.minimum(r.kp0 * p["kp_push"], r.kp_max)
    kp_land, kd_land = np.minimum(r.kp0 * p["kp_land"], r.kp_max), np.minimum(r.kd0 * p["kd_land"], r.kd_max)
    x0, y0, yaw0 = task.start
    yaw0 = yaw0 + p["face"]                                  # the heading the robot turns to and jumps in
    cy, sy = math.cos(yaw0), math.sin(yaw0)
    n_req = int(task.p["jumps_required"])

    def pose(h, k, a, dh=0.0, da=0.0, dr=0.0):
        q = q0.copy()
        for i in (0, 6):
            q[i], q[i + 3], q[i + 4] = h + dh, k, a + da
        q[1] += dr
        q[7] += dr
        q[12:] = up
        return q

    phase, t_phase, air_t, armed = "walk", 0.0, None, True
    walk_up = up.copy()
    while not s.done:
        pv = task.privileged(w)
        ttp = pv["time_to_pass_s"]
        if pv["boom_angle_to_robot_rad"] < -math.pi / 2:
            armed = True
        air = w.ref.airborne()
        qw, qx, qy, qz = w.d.qpos[r.fq + 3:r.fq + 7]
        wv = w.d.qvel[r.fv + 3:r.fv + 6]
        Rm = w.d.xmat[r.pelvis].reshape(3, 3)
        wb = Rm.T @ wv
        pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
        roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
        dx, dy = float(w.d.qpos[r.fq]) - x0, float(w.d.qpos[r.fq + 1]) - y0
        vx, vy = float(w.d.qvel[r.fv]), float(w.d.qvel[r.fv + 1])
        px, py = cy * dx + sy * dy, -sy * dx + cy * dy            # in the frame of the jumping heading
        vxc, vyc = cy * vx + sy * vy, -sy * vx + cy * vy
        da = p["kpitch"] * pitch + p["kdpitch"] * wb[1] + p["kvx"] * vxc + p["kxw"] * px
        dr = p["kroll"] * roll + p["kvy"] * vyc + p["ky"] * py
        t = w.t
        done_jumping = task.trackers["jumps"] >= n_req
        if phase == "walk":                           # the walking policy keeps the balance, back to the middle
            cyw, syw = math.cos(yaw), math.sin(yaw)
            ex, ey = -dx, -dy                         # to the podium's centre, in the pelvis' heading frame
            bx, by = cyw * ex + syw * ey, -syw * ex + cyw * ey
            eyaw = (yaw0 - yaw + math.pi) % (2 * math.pi) - math.pi
            cmd = [float(np.clip(p["kwalk"] * bx, -0.3, 0.3)), float(np.clip(p["kwalk"] * by, -0.3, 0.3)),
                   float(np.clip(2.0 * eyaw, -1.5, 1.5))]
            row = ("walk", np.r_[cmd, walk_up, 0, 0])
            feet = w.ref.parts_touching(task.podium)
            wr = float(np.linalg.norm(wb[:2]))
            if (armed and not done_jumping and ttp is not None and ttp <= p["crouch_lead"]
                    and t - t_phase > p["min_walk"] and {"left_foot", "right_foot"} <= feet
                    and abs(roll) < p["roll_ok"] and wr < p["rate_ok"] and abs(eyaw) < 0.12):
                phase, t_phase, q_from = "blend", t, r.arb.target().copy()
        if phase == "blend":                          # from the stance into the crouch
            f = min(1.0, (t - t_phase) / p["blend"])
            f = 0.5 - 0.5 * math.cos(math.pi * f)
            row = ("joints", np.r_[q_from + f * (pose(*C, da=da, dr=dr) - q_from), 0, 0])
            if f >= 1.0:
                phase = "wait"
        if phase == "wait":
            row = ("joints", np.r_[pose(*C, da=da, dr=dr), 0, 0])
            if armed and not done_jumping and ttp is not None and ttp <= p["push_lead"]:
                phase, t_phase, armed = "push", t, False
        if phase == "push":
            q = pose(p["ext_h"], p["ext_k"], p["ext_a"], dh=p["kvx_push"] * vxc + p["kpitch_push"] * pitch
                     + p["kxp"] * px, dr=dr)
            row = ("lowcmd", np.r_[q, z, z, kp_push, r.kd0, 0, 0])
            if air:
                air_t = t if air_t is None else air_t
                if t - air_t >= p["air_min"]:
                    phase, t_phase = "air", t
            else:
                air_t = None
                if t - t_phase > 0.5:
                    phase, t_phase = "land", t
        if phase == "air":
            if t - t_phase < p["tuck_s"]:
                row = ("lowcmd", np.r_[pose(p["tuck_h"], p["tuck_k"], p["tuck_a"]), z, z, r.kp0, r.kd0, 0, 0])
            else:
                row = ("lowcmd", np.r_[pose(*C2), z, z, r.kp0, r.kd0, 0, 0])
            if not air:
                phase, t_phase = "land", t
        if phase == "land":
            row = ("lowcmd", np.r_[pose(*C2, da=da, dr=dr), z, z, kp_land, kd_land, 0, 0])
            if t - t_phase > p["land_hold"]:
                phase, t_phase = ("walk" if not done_jumping or p["walk_after"] else "stay"), t
        if phase == "stay":
            row = ("joints", np.r_[pose(*C2, da=da, dr=dr), 0, 0])
        if s.step(*row):
            return
