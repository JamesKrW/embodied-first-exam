"""first-time-shot reference solution (privileged): a scripted standing side-foot kick of the G1, timed on the ball.

The robot stays where it starts. When the rolling ball is about 1.45 s from its right foot it moves its weight onto
the left foot (joints mode: both hip rolls and ankle rolls, 0.5 s), then draws the right leg back with its hip turned
out (hip yaw -0.3 rad) while the left ankle balances the centre of mass over the left foot (feedback on the
privileged centre of mass). When the ball is `off` metres plus 0.1 s of travel from the foot it swings the leg through
at the joints' speed limits; one control step after the foot touches the ball the walking policy takes over at zero
command and keeps the robot on its feet. The ball leaves the foot at about 5 m/s (it came at 3.1), towards the goal.

The swing's start inside a control step is set by blending the first swing target (a sub-step start time), so the
contact point on the ball varies smoothly with `off`. The privileged session has unlimited resets: if an attempt
does not score, the next `off` of a short list around the nominal one is tried (each attempt is a full episode from
the frozen start; the rows of the last attempt are the trajectory)."""
import numpy as np

OFFS = (0.125, 0.115, 0.135, 0.105, 0.145, 0.095, 0.155)


class Kicker:
    def __init__(self, w, eta, hit, t_shift=1.1, t_back=0.35, t_swing=0.10, a=-0.13, ky=-4.0, kx=1.5, kv=0.15,
                 kyaw=1.0, hy=-0.3):
        self.w, self.eta, self.hit = w, eta, hit
        r = w.robot
        names = [j[:-6] for j in w.L.JOINTS]
        self.I = {n: i for i, n in enumerate(names)}
        self.base = np.r_[r.stance, r.rest[12:]].copy()
        self.rest = r.rest[12:].copy()
        self.t_shift, self.t_back, self.t_swing = t_shift, t_back, t_swing
        self.a, self.ky, self.kx, self.kv, self.kyaw = a, ky, kx, kv, kyaw
        self.back = dict(right_hip_pitch=0.15, right_knee=0.9, right_ankle_pitch=-0.3, right_hip_yaw=hy)
        self.thru = dict(right_hip_pitch=-0.4, right_knee=0.3, right_ankle_pitch=-0.2, right_hip_yaw=hy)
        self.phase, self.k, self.k_hit, self.prev = "wait", 0, None, None

    def pose(self, **kw):
        q = self.base.copy()
        a = self.a
        for j, v in (("left_hip_roll", a), ("right_hip_roll", a), ("left_ankle_roll", -a), ("right_ankle_roll", -a)):
            q[self.I[j]] = v
        for k, v in kw.items():
            q[self.I[k]] = v
        return q

    def balance(self, q):
        w, I = self.w, self.I
        c = w.com()
        f = w.d.site_xpos[w.robot.foot_site["left"]]
        v = np.zeros(3) if self.prev is None else (c - self.prev) / w.CONTROL_DT
        self.prev = c.copy()
        e = c[:2] - (f[:2] + np.array([0.03, 0.0]))
        q = q.copy()
        q[I["left_ankle_roll"]] += self.ky * (e[1] + self.kv * v[1])
        q[I["left_ankle_pitch"]] += self.kx * (e[0] + self.kv * v[0])
        q[I["left_hip_yaw"]] += self.kyaw * (0.0 - float(w.pelvis()[3]))
        return q

    def _go(self, phase):
        self.phase, self.k = phase, 0

    def step(self):
        w, dt = self.w, self.w.CONTROL_DT
        eta = self.eta(w)
        self.k += 1
        if self.phase == "wait":
            if eta is not None and eta <= self.t_shift + self.t_back:
                self._go("shift")
            return "stand", np.r_[self.rest, 0, 0]
        if self.phase == "shift":
            q = self.base + (self.pose() - self.base) * min(1.0, self.k / 50)
            if eta is not None and eta <= self.t_back:
                self._go("back")
            return "joints", np.r_[q, 0, 0]
        if self.phase == "back":
            q0, q1 = self.pose(), self.pose(**self.back)
            q = q0 + (q1 - q0) * min(1.0, self.k / 20)
            if eta is not None and eta <= self.t_swing + dt:
                frac = min(1.0, max(0.0, (self.t_swing + dt - eta) / dt))
                self._go("thru")
                q = q1 + (self.pose(**self.thru) - q1) * frac
            return "joints", np.r_[self.balance(q), 0, 0]
        if self.phase == "thru":
            if self.k_hit is None and self.hit(w):
                self.k_hit = self.k
            if self.k >= 20 or (self.k_hit is not None and self.k - self.k_hit >= 1):
                self._go("walk")
            return "joints", np.r_[self.balance(self.pose(**self.thru)), 0, 0]
        return "walk", np.r_[0.0, 0.0, 0.0, self.rest, 0, 0]


def make_eta(off):
    """Seconds until the rolling ball is `off` metres (along the pass) from the right foot's strike face."""
    def eta(w):
        b, v = w.pos("ball"), w.vel("ball")
        if b[2] > 0.115 or v[1] > -0.5:              # still on the ramp, or not rolling across
            return None
        fy = w.d.geom_xpos[w.m.geom("right_foot_strike").id][1]
        return (b[1] - fy - off) / -v[1]
    return eta


def attempt(s, off, **gains):
    w = s.world
    K = Kicker(w, make_eta(off), hit=lambda w: w.task.trackers["strike"].phase >= 1, **gains)
    while not s.done:
        typ, row = K.step()
        s.step(typ, row)
    return s.status()["success"]


def solve(s, offs=OFFS, **gains):
    for i, off in enumerate(offs):
        if i:
            s.reset()
        if attempt(s, off, **gains):
            return True
    return False
