"""volleyball-set (v2): an overhead set from a technique found offline by scripts/roboolympics/search_technique.py,
RoboOlympics' method (pingyue's kit/throwing/optimize.py): the walking policy holds the robot at zero command until
just before the toss comes down, then four key poses of the trunk and the arms play on it (the fists of the referee
test above the forehead, the elbows bent, then driven up and a little forward through the ball; their times
searched). Searched by CMA-ES in the task's own frozen instance, each trial scored on how near a hand comes to the
ball, then on the set's height and where it comes down (the referee's distance from the target).
Measured (10-20, machines-feet: the toss now played by the physical two-wheel ball machine, ball_machine.py v4;
re-searched: 60 + 25 generations of CMA-ES (24-32), then 50 single-process Gaussian refinements): the toss comes down
at 6.6 m/s and leaves the hands upward at 7.1 m/s (1.07 times; the rule asks 1.0), rises to 3.92 m (3.43 m asked)
and lands 0.80 m from the target's centre (radius 1 m); success after 5.8 s. Open loop on the one frozen start: not
robust (1 of about 50 neighbouring techniques in the refinement succeeded)."""
import numpy as np

TECHNIQUE = {"times": [0.7395, 1.1957, 1.3245, 2.4073], "release_s": 0.05, "hand": None, "legs": 'walk', "prelude": [[0.0, 0.0, 0.0, 3.0]], "keys": [
        [0, -0.16, 0, 0.36, -0.2, -0, 0, -0.16, 0, 0.36, -0.2, -0, 0.3529, -2.6067, 0.0303, -0.1988, 0.3911, 0, 0, 0, -2.641, -0.0862, -0.1515, 0.4977, 0, 0, 0],
        [0, -0.16, 0, 0.36, -0.2, -0, 0, -0.16, 0, 0.36, -0.2, -0, -0.1395, -2.7461, -0.0167, 0.0839, 0.7714, 0, 0, 0, -2.7483, -0.7764, -0.23, 0.8168, 0, 0, 0],
        [0, -0.16, 0, 0.36, -0.2, -0, 0, -0.16, 0, 0.36, -0.2, -0, 0.1623, -2.8325, 0.2371, -0.2617, 1.2398, 0, 0, 0, -2.5413, -0.2543, 0.0041, 1.3925, 0, 0, 0],
        [0, -0.16, 0, 0.36, -0.2, -0, 0, -0.16, 0, 0.36, -0.2, -0, -0.012, 0.2662, 0.1313, -0.2601, -0.0031, 0, 0, 0, -0.1103, 0.0422, 0.4565, -0.0252, 0, 0, 0]]}


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


def solve(s, tech=None):
    tech = tech or TECHNIQUE
    if not play(s, tech):
        recover(s)
