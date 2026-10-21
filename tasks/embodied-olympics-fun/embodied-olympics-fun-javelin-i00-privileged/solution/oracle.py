"""javelin (the women's 600 g javelin, held at its cord only by the friction of the right Dex1-1 gripper; its flight
from published wind-tunnel coefficients): a standing throw by the trunk's turn with the arm raised at the side,
Unitree LowCmd rows along a Catmull-Rom curve through five whole-body key poses (RoboOlympics' kit/throwing method);
the closure command goes to 0 at release_s, but the javelin slips out of the jaws a little before, as the hand turns.
As soon as the referee has the release, the swing's follow-through is dropped and the walking policy at zero command
takes over (closed loop on the referee's release: the follow-through keys would topple the robot once the javelin's
weight is gone), then walks backwards until the robot is 4 m behind the arc's ends, and stands.
Found offline by CMA-ES (diagonal, numpy; the group's search_javelin.py) from the former 800 g reference technique,
over the upper body's joints of the three swing keys, their times and release_s, every trial the whole episode in the
task's physics, scored on the worst of three servo-gain scalings (x 0.97, 1.0, 1.03).
"""
import numpy as np

TECHNIQUE = {"times": [0.7052, 0.9241621719546359, 1.0518539104876143, 1.4158535604473736, 1.9607], "release_s": 1.1782481855989455, "hand": "right", "keys": [[0.43, -0.3488, -0.0167, 0.7477, -0.3989, 0.0169, 0.43, -0.444, -0.0442, 0.7294, -0.285, 0.041, -0.9426, -0.3231, 0.3198, 0, 1.6495, 0, 0, 0, 0.6404, -0.6798, -0.4461, 0.5432, -0.664, 0.1913, 0.6534], [-0.0072, -0.4714, -0.0591, 0.9271, -0.4557, 0.0591, -0.0073, -0.4697, -0.0591, 0.9271, -0.4575, 0.0591, -1.1635382634974505, 0.23724514340493627, 0.8374161510961429, -0.23934795978568948, 0.3506058836421093, 0.27054155787984985, 0.05167302143345309, 0.028743838875561034, -1.0008285326276634, -1.4681305198965824, -0.09448370330063052, 1.3173335273289657, -0.552046645765251, -0.01833905740418397, 1.1951485356395704], [-0.3906, -0.2253, 0.0411, 0.4343, -0.209, -0.0409, -0.3846, -0.1284, 0.0246, 0.4116, -0.2833, -0.0252, 0.7827995832679151, -0.6405044373356068, -0.38309718048966795, -0.08898280744404091, 0.5306385474850368, -0.19370860178257157, 0.09874643814156023, 0.18012516071862436, 0.2774379171003911, -2.451864216804686, -0.3410979823535518, 1.043071184333173, -0.8021459998127781, -0.512057246398679, 0.22778949553605546], [-0.3412, -0.3202, 0.064, 0.4536, -0.133, -0.0624, -0.3346, -0.2683, 0.0513, 0.4935, -0.2251, -0.0512, 2.2520874476760864, 0.3341500722868048, 0.7726443787243787, -0.11164898973852562, -0.8075549581832155, -0.22357364348299355, 0.2607609898139279, -0.0952410891597321, -0.19078568025972803, -1.401219779318202, -0.4638232580458351, 2.2114929232015728, -0.8250147800276586, -0.41726871114807906, 0.16298415560699725], [-0.0625, -0.013, 0.0093, -0.1206, 0.1336, -0.0093, -0.0622, 0.0142, 0.0089, -0.1477, 0.1335, -0.0089, 0.5851, -0.5734, -0.2281, 0, -0.95, 0, 0, 0, -0.4841, -0.5778, 0.3384, 0.5476, -0.4606, 0.3996, 0.3907]]}


def curve(q0, keys, times, dt):
    """Positions and speeds every dt along the cubic Hermite curve through q0 (t = 0) and the keys, Catmull-Rom
    tangents, zero speed at both ends (RoboOlympics' kit/throwing/technique.py)."""
    p = np.vstack([q0, keys])
    t = np.concatenate([[0.0], times])
    v = np.zeros_like(p)
    v[1:-1] = (p[2:] - p[:-2]) / (t[2:] - t[:-2])[:, None]
    s = np.arange(1, int(np.ceil(t[-1] / dt - 1e-9)) + 1) * dt
    s = np.minimum(s, t[-1])
    i = np.clip(np.searchsorted(t, s, side="right") - 1, 0, len(t) - 2)
    h = (t[i + 1] - t[i])[:, None]
    u = (s - t[i])[:, None] / h
    q = ((2 * u ** 3 - 3 * u ** 2 + 1) * p[i] + (u ** 3 - 2 * u ** 2 + u) * h * v[i]
         + (-2 * u ** 3 + 3 * u ** 2) * p[i + 1] + (u ** 3 - u ** 2) * h * v[i + 1])
    qd = ((6 * u ** 2 - 6 * u) * p[i] / h + (3 * u ** 2 - 4 * u + 1) * v[i]
          + (-6 * u ** 2 + 6 * u) * p[i + 1] / h + (3 * u ** 2 - 2 * u) * v[i + 1])
    return q, qd


def play(s, tech, watch=None):
    """The technique from the current state (after its walk, if any): one LowCmd row per control step to its last
    key; the grip of tech["hand"] (if any) on until release_s. watch(s) is called after every step (the search's probe). Returns True if the episode ended."""
    w = s.world
    r = w.robot
    n = r.L.NJ
    for vx, vy, wz, secs in tech.get("prelude", []):     # a walk first (the walking policy), the arms as they are
        up = w.d.qpos[r.qadr][12:].copy()
        for _ in range(int(round(secs / w.CONTROL_DT))):
            if s.step("walk", np.r_[vx, vy, wz, up, 0.0, 0.0]):
                return True
            if watch:
                watch(s)
    q0 = w.d.qpos[r.qadr].copy()
    Q, QD = curve(q0, np.asarray(tech["keys"], float), np.asarray(tech["times"], float), w.CONTROL_DT)
    kp, kd = r.kp0 * tech.get("kp_scale", 1.0), r.kd0
    g = {"right": 1, "left": 0}.get(tech.get("hand", "right"))
    for k in range(len(Q)):
        grip = np.zeros(2)
        if g is not None:
            grip[g] = 1.0 if (k + 1) * w.CONTROL_DT < tech["release_s"] else 0.0
        if tech.get("legs") == "walk":           # the walking policy keeps the balance; the keys give the upper body
            row = np.r_[0.0, 0.0, 0.0, Q[k][12:], grip]
            if s.step("walk", row):
                return True
        elif s.step("lowcmd", np.r_[Q[k], QD[k], np.zeros(n), kp, kd, grip]):
            return True
        if watch:
            watch(s)
    return False


def upper(s):
    w = s.world
    return w.d.qpos[w.robot.qadr][12:].copy()


def recover(s, back_s=None, vx=-0.3, settle_s=1.5, watch=None):
    """The walking policy at zero command (the arms where they are, brought down), then backwards for back_s s (or
    until the referee's exit stage), then at rest to the end."""
    w = s.world
    r = w.robot
    up0 = upper(s)
    rest = r.rest[12:]
    k = 0
    while not s.done:
        t = k * w.CONTROL_DT
        f = min(1.0, t / 1.0)
        up = up0 + (rest - up0) * f
        moving = settle_s <= t and not w.ref.has("exit") and (back_s is None or t < settle_s + back_s)
        if s.step("walk", np.r_[vx if moving else 0.0, 0.0, 0.0, up, 0.0, 0.0]):
            return
        if watch:
            watch(s)
        k += 1


class _Released(Exception):
    pass


def solve(s, tech=None):
    tech = tech or TECHNIQUE

    def watch(s_):
        if s_.world.ref.has("release") or s_.world.ref.failed:
            raise _Released

    try:
        if play(s, tech, watch=watch):
            return
    except _Released:
        pass
    if not s.done:
        recover(s)
