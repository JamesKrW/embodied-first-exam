"""Reference solution for platform-10m (privileged mode): a scripted forward dive straight (101A) for the Unitree G1.

The arms go overhead slowly (`stand` rows), a crouch balanced by kinematics (`joints`), a short lean with soft
ankles over the end, a two-foot push with the hips opened (`lowcmd`), then the straight entry shape held through the
flight: legs straight and drawn together, toes pointed, arms stretched overhead with the hands together. The dive is
open loop; the solver resets the privileged session between tries and bisects the lean time on the referee's outcome
(over-rotated or fallen off without rising: lean less; anything else: lean longer) for up to three hip openings,
keeping the first dive the referee accepts (the same search an agent can run in privileged mode)."""
import mujoco
import numpy as np

BASE = dict(arm_s=3.0, bias=0.06, knee=1.4, kp_scale=2.5, hold_s=0.25)


def _com_ahead(w, legs, upper) -> float:
    """How far the centre of mass is ahead of the soles' centre in this pose with the soles flat (kinematics)."""
    m, r = w.m, w.robot
    d2 = mujoco.MjData(m)
    d2.qpos[:] = w.d.qpos
    h, k, a = legs
    for s in (0, 6):
        d2.qpos[r.qadr[s + 0]], d2.qpos[r.qadr[s + 3]], d2.qpos[r.qadr[s + 4]] = h, k, a
    d2.qpos[r.qadr[12:]] = upper
    th = -(h + k + a)                                # the pelvis pitch that keeps the soles flat
    d2.qpos[r.fq + 3:r.fq + 7] = [np.cos(th / 2), 0, np.sin(th / 2), 0]
    mujoco.mj_kinematics(m, d2)
    mujoco.mj_comPos(m, d2)
    feet = [g for g in range(m.ngeom) if "ankle_roll_link_sc" in m.geom(g).name]
    return float(d2.subtree_com[r.pelvis][0]) - float(np.mean(d2.geom_xpos[feet][:, 0]))


def balanced_crouch(w, knee: float, upper, bias: float):
    """(hip pitch, knee, ankle pitch) with the centre of mass `bias` m ahead of the soles' centre, the trunk as
    upright as that allows."""
    best = None
    for a in np.linspace(-0.85, 0.2, 43):
        lo, hi = -2.4, 0.3
        for _ in range(40):
            h = (lo + hi) / 2
            lo, hi = (h, hi) if _com_ahead(w, (h, knee, a), upper) - bias > 0 else (lo, h)
        if abs(_com_ahead(w, (h, knee, a), upper) - bias) < 0.004 and (best is None or abs(h + knee + a) < best[0]):
            best = (abs(h + knee + a), (float(h), knee, float(a)))
    return best[1]



ARM_UP = dict(shoulder_pitch=-2.57, shoulder_roll=0.19, shoulder_yaw=0.15, elbow=0.50)


def jidx(w):
    return {n[:-6]: i for i, n in enumerate(w.L.JOINTS)}


def upper_up(w):
    J = jidx(w)
    q = w.robot.rest.copy()
    for k, v in ARM_UP.items():
        q[J["left_" + k]] = v
        q[J["right_" + k]] = -v if k in ("shoulder_roll", "shoulder_yaw") else v
    for s in ("left", "right"):
        for k in ("wrist_roll", "wrist_pitch", "wrist_yaw"):
            q[J[f"{s}_{k}"]] = 0.0
    return q[12:]


def straight(w, hip=0.0, knee=0.0, ankle=0.45, roll=0.14, waist=0.0):
    J = jidx(w)
    q = np.zeros(w.L.NJ)
    q[12:] = upper_up(w)
    for s, sg in (("left", 1), ("right", -1)):
        q[J[f"{s}_hip_pitch"]] = hip
        q[J[f"{s}_hip_roll"]] = -sg * roll
        q[J[f"{s}_knee"]] = knee
        q[J[f"{s}_ankle_pitch"]] = ankle
    q[J["waist_pitch"]] = waist
    return q


def dive(w, A, act, knee=1.2, bias=0.04, lean=-0.35, ext_ankle=0.45, kp_scale=1.4, arm_s=1.0, crouch_s=0.6,
         flight_hip=0.0, hold_s=0.05, crouch_arms_up=True, com_kp=0.0, com_kd=0.0, com_off=0.0, ext_s=0.0, flight_arms_up=True, flight_roll=0.14, flight_ankle=0.45,
         push=True, tip_s=0.0, tip_kp=5.0, settle_s=0.4, after=None, rows=None):
    """Drive the world to the end of the episode. Returns nothing; the referee has the verdict."""
    r, NJ = w.robot, w.L.NJ
    up0, up1 = r.rest[12:].copy(), upper_up(w)
    n = int(arm_s / w.CONTROL_DT)
    if n > 0:
        x_ref = float(w.com()[0]) + com_off
        legs = r.arb.target()[:12].copy()
        for k in range(n + int(settle_s / w.CONTROL_DT)):
            f = 0.5 - 0.5 * np.cos(np.pi * min(1.0, k / n))
            up = up0 + f * (up1 - up0)
            if com_kp:                             # an ankle strategy keeps the centre of mass over the feet
                vx = float(w.d.qvel[r.fv])
                e = float(w.com()[0]) - x_ref
                lg = legs.copy()
                lg[[4, 10]] += np.clip(com_kp * e + com_kd * vx, -0.4, 0.4)
                act(w, A, "joints", np.r_[lg, up, 0, 0])
            else:
                act(w, A, "stand", np.r_[up, 0, 0])
    q0 = r.arb.target().copy()
    upc = up1 if crouch_arms_up else q0[12:]
    h, kn, a = balanced_crouch(w, knee, upc, bias)
    qc = q0.copy()
    for s in (0, 6):
        qc[s], qc[s + 3], qc[s + 4] = h, kn, a
    qc[12:] = upc
    n = int(crouch_s / w.CONTROL_DT)
    for k in range(n + 10):
        f = 0.5 - 0.5 * np.cos(np.pi * min(1.0, k / n))
        act(w, A, "joints", np.r_[q0 + f * (qc - q0), 0, 0])
    kpt = r.kp0.copy()
    kpt[[4, 10]] = tip_kp                          # soft ankles: the body tips forwards over the toes
    for k in range(int(round(tip_s / w.CONTROL_DT))):
        act(w, A, "lowcmd", np.r_[qc, np.zeros(NJ), np.zeros(NJ), kpt, r.kd0, 0, 0])
    qx = straight(w, hip=lean, ankle=ext_ankle)
    kp = np.minimum(r.kp0 * kp_scale, r.kp_max)
    qf = straight(w, hip=flight_hip, roll=flight_roll, ankle=flight_ankle)
    if not flight_arms_up:
        qf[12:] = r.rest[12:]
    if not push:                                   # no extension: the soft ankles let the body fall forwards
        qx = qc.copy()
        kp = kpt
    air = None
    t_ext = w.t
    while not w.done:
        tgt = qx if air is None or w.t - air < hold_s else qf
        if ext_s > 0 and air is None:              # a ramped push: the legs open over ext_s seconds
            f = min(1.0, (w.t - t_ext) / ext_s)
            tgt = qc + f * (qx - qc)
        act(w, A, "lowcmd", np.r_[tgt, np.zeros(NJ), np.zeros(NJ), kp, r.kd0, 0, 0])
        if air is None and w.ref.airborne():
            air = w.t
        if after:
            after(w)


def solve(s, variant=None):
    """Drive the session to a successful dive. For each hip opening, a bisection on the lean time: a dive that turns
    past the half somersault before the hips reach the water leans less next time, any other failure (short, scraping
    the end, landing back on it) leans longer. variant: overrides of BASE (the robustness check's perturbations)."""
    w = s.world
    act = lambda w_, A_, typ, row: None if s.done else s.step(typ, row)  # noqa: E731
    last = None
    for lean in (0.6, 0.5, 0.7):
        lo, hi = 0.3, 0.45
        for _ in range(7):
            tip = round((lo + hi) / 2, 4)
            s.reset()
            dive(w, None, act, **{**BASE, **(variant or {}), "lean": lean, "tip_s": tip})
            last = s.status()
            if last["success"]:
                return last
            if w.task.trackers["turn"] > np.pi or "no take-off" in (last["failed"] or ""):
                hi = tip                             # over-rotated, or it fell off the end without rising
            else:
                lo = tip
    return last
