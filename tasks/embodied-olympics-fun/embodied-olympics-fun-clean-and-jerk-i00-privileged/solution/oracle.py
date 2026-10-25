"""clean-and-jerk (the H1-2's Inspire hands): the reference solution, through `lowcmd` rows (privileged planning).

Quasi-static: every pose of the plan has both feet flat and the centre of mass (the bar's included) over the soles;
the rows carry the static torques as feed-forward (gravity, the bar's weight at the hands, the feet's wrenches).
1. Bend down to the bar on its blocks with the hands open, palms down, the fingers pointing forwards and a little
   down; close both hands round the bar; stand up with straight arms, the bar's path LIFT_DX ahead of where it lay
   (clear of the shins); the attempt begins.
2. Straighten the knees; bring the bar up in front of the hips and the trunk to the shoulders: the clean (the arms'
   inverse kinematics keep the bar along the hands' grasp axes).
3. Press it overhead: the arms' null space drawn smoothly to straight arms; hold until the down signal.
The arms' inverse kinematics put each hand's grasp centre (rfo.realhand: where a bar's axis sits in the closed hand)
on the bar. `stop_after`: "plan", "lift" or "rack" end early (for the tests).
2026-10-22, the core with real self-collision (920f215c72): the squat to a bar on 0.20 m blocks needs the hips flexed
past where the hip links meet the pelvis (the planner's bottom poses put them 15 to 46 mm into it, or the thumbs into
the knees; 52 squats tried all fall onto a thigh or knock the bar). The family now lifts from 0.35 m blocks (a block
clean & jerk), and the squat is shallower: knees 1.2 rad (knee_g), the pelvis pitched 0.8 rad (pitch_g). grip_y and
com_x are the search's extra knobs (defaults unchanged). Measured: success at 20.29 s, no arm into the
robot's own body; live, replay and the state hash agree. Of 7 neighbours 4 succeed (all servo gains x 0.97 / 1.03,
the squat at knees 1.4 / pitch 0.9); a squat at 1.1 / 0.75 never reaches the bar, the bar 1 cm farther (a hand
touches a block), 1 cm nearer or the plan 10 % slower (the bar drops after the press) fail."""
import math

import mujoco
import numpy as np

# -------------------------------------------------------------------------------------------------------------- whole-body tools
MIRROR = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
G = 9.81


def smooth(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3 - 2 * x)


class Planner:
    def __init__(self, w, grip_p=0.13, grip_y=0.25):
        self.w, self.m, self.r = w, w.m, w.robot
        m, r = self.m, self.r
        self.d = mujoco.MjData(m)
        self.d.qpos[:] = w.d.qpos
        mujoco.mj_kinematics(m, self.d)
        self.foot0 = {s: self.d.site_xpos[r.foot_site[s]].copy() for s in ("left", "right")}
        self.feet_mid = 0.5 * (self.foot0["left"] + self.foot0["right"])
        self.foot_z_drop = 0.020                       # the soles settle that much into the platform (measured)
        for s_ in self.foot0:
            self.foot0[s_][2] -= self.foot_z_drop
        J = {j[:-6]: i for i, j in enumerate(w.L.JOINTS)}
        self.J = J
        self.leg_idx = [(J[f"{s}_hip_pitch"], J[f"{s}_knee"], J[f"{s}_ankle_pitch"]) for s in ("left", "right")]
        a = w.L.ARMS.start
        self.LA, self.RA = np.arange(a, a + 7), np.arange(a + 7, a + 14)
        self.site = {s_: m.site(f"{s_}_grasp").id for s_ in ("left", "right")}
        self.grip_y = grip_y
        self.bar_id = m.body("bar").id
        self.bar_mass = float(m.body_subtreemass[self.bar_id])
        self.robot_mass = float(sum(m.body_mass[b] for b in r.bodies))
        self.qa_bar = w.free["bar"][0]
        self.va_bar = w.free["bar"][1]

    # ------------------------------------------------------------------ poses
    def set(self, q, pitch):
        """The robot at joint angles q with its pelvis pitched by `pitch`, placed so that the left foot's site is
        where it stood."""
        m, d, r = self.m, self.d, self.r
        d.qpos[r.qadr] = q
        d.qpos[r.fq:r.fq + 3] = 0.0
        d.qpos[r.fq + 3:r.fq + 7] = [math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0]
        d.qpos[self.qa_bar:self.qa_bar + 3] = (50.0, 0.0, 1.0)
        mujoco.mj_kinematics(m, d)
        d.qpos[r.fq:r.fq + 3] += self.foot0["left"] - d.site_xpos[r.foot_site["left"]]
        d.qpos[r.fq + 1] = 0.0
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)

    def grip_point(self, side="left"):
        """The hand's grasp centre (where the bar's axis sits in the closed hand) and its frame (x out of the palm, y
        along the bar, z along the fingers)."""
        k = self.site[side]
        return self.d.site_xpos[k].copy(), self.d.site_xmat[k].reshape(3, 3).copy()

    def com(self, held):
        c = self.d.subtree_com[self.r.pelvis] * self.robot_mass
        M = self.robot_mass
        if held:
            g = 0.5 * (self.grip_point("left")[0] + self.grip_point("right")[0])
            c = c + self.bar_mass * g
            M += self.bar_mass
        return c / M

    COM_X = 0.046                                      # the soles' middle ahead of the foot sites (the platform's
    #                                                    contact points: -0.108 .. 0.200 m from the pelvis at t = 0)

    def legs(self, q, knee, pitch, held=False, com_x=None):
        """Leg angles for this knee angle and pelvis pitch: feet flat, the centre of mass over the feet's middle
        (+ com_x)."""
        q = q.copy()
        com_x = self.COM_X if com_x is None else com_x

        def put(h):
            for ih, ik, ia in self.leg_idx:
                q[ih], q[ik], q[ia] = h, knee, -(pitch + h + knee)
            self.set(q, pitch)
            return float(self.com(held)[0] - self.feet_mid[0]) - com_x

        lo, hi = -3.0, 1.0
        flo = put(lo)
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            f = put(mid)
            if (f > 0) == (flo > 0):
                lo, flo = mid, f
            else:
                hi = mid
        put(0.5 * (lo + hi))
        return q

    def arms(self, q, pitch, target, iters=60, lam=0.02, rest=None, rest_w=0.05, fingers=None, bend=None):
        """bend: instead of `rest`, draw the elbow towards 0.9 rad and the wrists towards 0 (gain `bend`), the elbow
        kept at or below pi/2 (bending forwards only), the bend measured in the injected lift that checks the referee."""
        """Arm angles (the right mirrors the left) that bring the left grip point to `target` with the jaws' across
        axis along y: damped least squares, drawn in the null space towards `rest` (default: q's arms). Returns
        (q, position error)."""
        m, d, r = self.m, self.d, self.r
        q = q.copy()
        lo, hi = r.lo[self.LA] + 1e-3, r.hi[self.LA] - 1e-3
        if bend is not None:
            hi[3] = min(hi[3], math.pi / 2)
        rest = q[self.LA].copy() if rest is None else np.asarray(rest, float)
        va = r.vadr[self.LA]
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for _ in range(iters):
            self.set(q, pitch)
            p, R = self.grip_point()
            mujoco.mj_jacSite(m, d, jp, jr, self.site["left"])
            ys = np.array([0.0, np.sign(R[1, 1]) or 1.0, 0.0])
            if fingers is None:                    # the jaws' across axis along y
                e = np.r_[np.asarray(target) - p, 0.3 * np.cross(R[:, 1], ys)[[0, 2]]]
                Jm = np.vstack([jp[:, va], 0.3 * jr[:, va][[0, 2]]])
            else:                                  # ... and the fingers along `fingers`, the palm facing down
                f = np.asarray(fingers, float) / np.linalg.norm(fingers)
                if np.cross(ys, f)[2] > 0:
                    ys = -ys
                Rd = np.column_stack([np.cross(ys, f), ys, f])
                er = 0.5 * sum(np.cross(R[:, i], Rd[:, i]) for i in range(3))
                e = np.r_[np.asarray(target) - p, 0.3 * er]
                Jm = np.vstack([jp[:, va], 0.3 * jr[:, va]])
            inv = Jm.T @ np.linalg.inv(Jm @ Jm.T + lam * np.eye(len(e)))
            if bend is None:
                pull = rest_w * (rest - q[self.LA])
            else:
                pull = np.zeros(7)
                pull[3] = bend * (0.9 - q[self.LA[3]])
                pull[5:] = -bend * q[self.LA[5:]]
            dq = inv @ e + (np.eye(7) - inv @ Jm) @ pull
            q[self.LA] = np.clip(q[self.LA] + np.clip(dq, -0.2, 0.2), lo, hi)
            q[self.RA] = q[self.LA] * MIRROR
        self.set(q, pitch)
        p, _ = self.grip_point()
        return q, float(np.linalg.norm(np.asarray(target) - p))

    def base(self):
        return self.d.qpos[self.r.fq:self.r.fq + 7].copy()

    # ------------------------------------------------------------------ statics
    def static_tau(self, q, pitch, held):
        """Joint torques (robot order) that hold this pose still with both feet on the ground."""
        m, d, r = self.m, self.d, self.r
        self.set(q, pitch)
        d.qvel[:] = 0.0
        mujoco.mj_forward(m, d)
        need = d.qfrc_bias.copy()                      # what the actuators + the ground must supply
        if held:
            jp = np.zeros((3, m.nv))
            for s in ("left", "right"):
                mujoco.mj_jacSite(m, d, jp, None, self.site[s])
                need += jp.T @ np.array([0.0, 0.0, self.bar_mass * G / 2])
        Jc = []
        for s in ("left", "right"):
            jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
            mujoco.mj_jacSite(m, d, jp, jr, r.foot_site[s])
            Jc.append(np.vstack([jp, jr]))
        Jc = np.vstack(Jc)                              # 12 x nv
        bv = np.arange(r.fv, r.fv + 6)
        A = Jc[:, bv].T                                # 6 x 12
        f = np.linalg.lstsq(A, need[bv], rcond=None)[0]
        tau = need - Jc.T @ f
        return tau[r.vadr], f


class Runner:
    """Plays a plan: a list of (q (robot order), pitch, held, grips (2,)) per control step, as lowcmd rows."""

    def __init__(self, s, planner, gains=1.0, ankle_kp=800.0, ankle_kd=20.0):
        self.s, self.P = s, planner
        r = planner.r
        self.kp, self.kd = np.array(r.kp0, float) * gains, np.array(r.kd0, float)
        for s_ in ("left", "right"):
            for j in ("ankle_pitch", "ankle_roll"):
                i = planner.J[f"{s_}_{j}"]
                self.kp[i], self.kd[i] = ankle_kp, ankle_kd
        self.n = len(self.kp)
        self.prev = None

    def play(self, plan, on_step=None):
        dt = self.s.world.CONTROL_DT
        for k, (q, pitch, held, grips) in enumerate(plan):
            if self.s.done:
                return False
            nxt = plan[k + 1][0] if k + 1 < len(plan) else q
            dq = (nxt - q) / dt
            tau, _ = self.P.static_tau(q, pitch, held)
            row = np.r_[q, dq, tau, self.kp, self.kd, grips]
            self.s.step("lowcmd", row)
            if on_step is not None:
                on_step(k)
        return not self.s.done


# -------------------------------------------------------------------------------------------------------------- the plan


def task_plan(P, keys, q0, fingers_down_below=0.45, bend=None, rest=None, rest_w=0.05, first_iters=300):
    """Dense plan from task-space keyframes: (seconds, knee, pitch, target (x, z) of the left grip point or None
    (arms rest), held, grips). Each control step: legs for the knee and pitch (feet flat, centre of mass over the
    feet), arms by IK to the interpolated target."""
    dt = P.w.CONTROL_DT
    q = q0.copy()
    out = []
    prev = keys[0]
    cur_t = None
    total = sum(max(1, int(round(k[0] / dt))) for k in keys[1:])
    for key in keys[1:]:
        sec, knee, pitch, tgt, held, grips = key
        n = max(1, int(round(sec / dt)))
        for i in range(1, n + 1):
            u = smooth(i / n)
            kn = prev[1] + (knee - prev[1]) * u
            pi = prev[2] + (pitch - prev[2]) * u
            if tgt is not None:
                a = np.asarray(prev[3] if prev[3] is not None else cur_t, float)
                b = np.asarray(tgt, float)
                t = a + (b - a) * u
                q = P.legs(q, kn, pi, held)
                fg = FINGERS if t[1] < fingers_down_below else None   # near the floor: the palm down on the bar
                rs = None
                if rest is not None:                  # the null space's target moves with the plan (no jump)
                    r0, r1 = rest
                    frac = (len(out) + 1) / total
                    rs = np.asarray(r0) + (np.asarray(r1) - np.asarray(r0)) * smooth(frac)
                q, err = P.arms(q, pi, [t[0], P.grip_y, t[1]], iters=40 if out else first_iters, fingers=fg, bend=bend,
                                rest=rs, rest_w=rest_w)
                q = P.legs(q, kn, pi, held)
            else:
                q = P.legs(q, kn, pi, held)
            out.append((q.copy(), pi, held, np.asarray(grips, float)))
        if tgt is not None:
            cur_t = tgt
        prev = key if key[3] is not None else (key[0], key[1], key[2], cur_t, key[4], key[5])
    return out




def joint_plan(P, q_from, segs, held=True):
    """Dense plan from joint-space segments: (seconds, knee, pitch, ankle_extra, arm (7) or None, grips). Legs from the
    planner (feet flat, centre of mass over the feet) plus an extra ankle angle (a rise onto the toes); the left arm
    interpolated in joint space (the right mirrors)."""
    dt = P.w.CONTROL_DT
    out = []
    q = q_from.copy()
    knee0 = float(q[P.leg_idx[0][1]])
    pitch0, ank0, arm0 = segs[0][2], 0.0, q[P.LA].copy()
    prev = (0.0, knee0, pitch0, ank0, arm0, segs[0][5])
    for sec, knee, pitch, ank, arm, grips in segs:
        n = max(1, int(round(sec / dt)))
        a_to = arm0 if arm is None else np.asarray(arm, float)
        for i in range(1, n + 1):
            u = smooth(i / n)
            kn = prev[1] + (knee - prev[1]) * u
            pi = prev[2] + (pitch - prev[2]) * u
            ak = prev[3] + (ank - prev[3]) * u
            q[P.LA] = prev[4] + (a_to - prev[4]) * u
            q[P.RA] = q[P.LA] * MIRROR
            q = P.legs(q, kn, pi, held)
            for ih, ik, ia in P.leg_idx:
                q[ia] += ak
            out.append((q.copy(), pi, held, np.asarray(grips, float)))
        arm0 = a_to
        prev = (sec, knee, pitch, ank, a_to, grips)
    return out



LIFT_DX = 0.04                    # the bar's path ahead of where it lay while standing up (clear of the shins)
FINGERS = (1.0, 0.0, -0.35)       # near the floor the fingers point forwards and a little down, the palm on the bar
TOP_ARM = (-2.983, 0.601, -0.028, 1.551, -1.642, 0.078, 0.0)   # the left arm straight up (test_clean_and_jerk's seed)
TIMING = dict(pull=1.5, turn=2.4, rack=1.4, press=1.6, straighten=1.0, top=3.0, pull_dx=0.02, via_dx=0.20,
              rack_dx=0.135, rack_dz=0.02, top_w=0.05, hold_max=6.0)


def solve(s, grip_p=0.125, grip_y=0.25, com_x=None, knee_g=1.2, pitch_g=0.8, down_s=3.0, lift_s=4.0, verbose=False, stop_after=None,
          timing=None):
    """The reference solution. timing: overrides of TIMING; stop_after: 'plan', 'lift' or 'rack' (for the tests)."""
    timing = dict(TIMING, **(timing or {}))
    w = s.world
    P = Planner(w, grip_p=grip_p, grip_y=grip_y)
    if com_x is not None:
        P.COM_X = com_x
    T = w.task
    bar = T.rest
    q0 = np.r_[w.robot.stance, w.robot.rest[12:]].copy()
    P.set(q0, 0.0)
    g0, _ = P.grip_point()
    zb, xb = float(bar[2]), float(bar[0])
    keys = [(0.0, 0.36, 0.0, (g0[0], g0[2]), False, (0, 0)),
            (down_s * 0.6, 1.2, 0.8, (xb, zb + 0.12), False, (0, 0)),
            (down_s * 0.4, knee_g, pitch_g, (xb, zb), False, (0, 0)),
            (0.5, knee_g, pitch_g, (xb, zb), False, (1, 1)),
            (lift_s, 0.36, 0.0, (xb + LIFT_DX, zb + 0.55), True, (1, 1)),
            (0.5, 0.36, 0.0, (xb + LIFT_DX, zb + 0.55), True, (1, 1))]
    plan = task_plan(P, keys, q0)
    solve.lift_end = plan[-1][0].copy()
    if stop_after == "plan":
        return
    R = Runner(s, P)
    if not R.play(plan) or stop_after == "lift":
        return
    hang = plan[-1][0]
    t = timing
    ks = 0.05                                          # straight knees (within 10 degrees) for the holds
    qs = P.legs(hang.copy(), ks, 0.0, True)
    P.set(qs, 0.0)
    sh = P.d.xpos[P.m.body("left_shoulder_pitch_link").id].copy()
    pull_end = (xb + t["pull_dx"], zb + 0.78)          # ahead of the hips and the belly: the trunk's front is about
    via = (sh[0] + t["via_dx"], sh[2] - 0.25)            # 0.10 m ahead of the shoulder joints (the bar must not meet it)
    rack = (sh[0] + t["rack_dx"], sh[2] + t["rack_dz"])
    jerk1, jerk2 = (sh[0] + 0.22, sh[2] + 0.25), (sh[0] + 0.20, sh[2] + 0.52)
    g_hang = (xb + LIFT_DX, zb + 0.55)
    keys2 = [(0.0, 0.36, 0.0, g_hang, True, (1, 1)),
             (t["pull"], ks, 0.0, pull_end, True, (1, 1)),
             (t["turn"] / 2, ks, 0.0, via, True, (1, 1)),
             (t["turn"] / 2, ks, 0.0, rack, True, (1, 1)),
             (t["rack"], ks, 0.0, rack, True, (1, 1)),
             (t["press"] / 2, ks, 0.0, jerk1, True, (1, 1)),
             (t["press"] / 2, ks, 0.0, jerk2, True, (1, 1))]
    plan2 = task_plan(P, keys2, hang, fingers_down_below=-1.0, bend=0.05)
    if not R.play(plan2) or stop_after == "rack":
        return
    top = np.asarray(TOP_ARM, float)                   # the overhead target: where the straight arms put the bar
    qt = plan2[-1][0].copy()
    qt[P.LA], qt[P.RA] = top, top * MIRROR
    qt = P.legs(qt, ks, 0.0, True)
    g_top, _ = P.grip_point()
    keys3 = [(0.0, ks, 0.0, jerk2, True, (1, 1)),
             (t["straighten"], ks, 0.0, (g_top[0], g_top[2] - 0.01), True, (1, 1)),
             (t["top"], ks, 0.0, (g_top[0], g_top[2] - 0.01), True, (1, 1))]
    plan3 = task_plan(P, keys3, plan2[-1][0], fingers_down_below=-1.0, rest=(plan2[-1][0][P.LA], top),
                      rest_w=t["top_w"], first_iters=10)
    if not R.play(plan3):
        return
    R.play([plan3[-1]] * int(round(t["hold_max"] / w.CONTROL_DT)))     # held until the down signal
