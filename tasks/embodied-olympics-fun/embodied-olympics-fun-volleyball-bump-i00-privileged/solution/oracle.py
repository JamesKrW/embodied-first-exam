"""volleyball-bump: a forearm-platform pass (bump-shells, 2026-10-20, with the core's prop-only body shells).

Technique. The walking policy walks the robot forwards and to the left to where the serve comes down and then stands
(zero command; it keeps the legs). Meanwhile the arms come up forward, apart, and then turn in into a forearm platform:
both upper arms forward, the elbows bent and the shoulders turned in so that the two forearms lie side by side and
parallel, 0.15 m apart between their axes (at least 5 cm of air between the two arms and hands: the H1-2 has no
collision between its arms, so the technique keeps them apart itself), tilted so that the platform faces the net and
up; the wrists are bent down a little, the hands out of the way. The platform is held still while the serve comes
down (a slight lift of the shoulders at the contact): the ball rebounds off the right forearm (the H1-2's elbow-pitch
and elbow-roll links, between the elbow and the wrist), not off the wrist housing, the wrist, the hands or the upper
arm, rises above the net and comes down in the setter's target. Then the arms come back forward, apart and down.

The pose was found by forward kinematics (the platform's normal, the forearms' separation, their clearance from each
other and from the trunk), and the walk, the timing and small offsets of the pose by CMA-ES in the frozen instance,
scored on the referee's outcome with penalties for any ball contact other than the forearms and for the arms coming
closer than 5 mm, then refined on the mean over small perturbations of its own timing and of the arms' servo gains
(the search is tooling outside this file; the oracle imports numpy only). Open loop: the serve is the same in every
attempt."""
import numpy as np

# walk: the walking policy's command (vx, vy, wz) until walk[3] s, then zero. times (s) / keys: the upper body (torso;
# left arm, right arm: shoulder pitch, roll, yaw, elbow pitch, elbow roll, wrist pitch, wrist yaw): at rest; the arms
# forward and apart; the platform; held; the platform at the contact; forward and apart; at rest.
TECHNIQUE = {"walk": [0.5287, 0.5241, 0.0143, 2.0], "times": [2.4504, 3.0004, 3.4504, 4.2418, 4.3618, 4.8118, 5.5618], "keys": [
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        [-0.0677, -1.4113, 0, 0, 2.0623, 0, 0.4308, 0, -1.3717, 0, 0, 2.0623, 0, 0.4308, 0],
        [-0.0677, -1.4113, -0.2053, -0.7625, 2.0623, -0.741, 0.4308, 0, -1.3717, 0.2053, 0.7625, 2.0623, 0.741, 0.4308, 0],
        [-0.0677, -1.4113, -0.2053, -0.7625, 2.0623, -0.741, 0.4308, 0, -1.3717, 0.2053, 0.7625, 2.0623, 0.741, 0.4308, 0],
        [-0.0677, -1.3098, -0.2053, -0.7625, 2.0623, -0.741, 0.4308, 0, -1.2702, 0.2053, 0.7625, 2.0623, 0.741, 0.4308, 0],
        [-0.0677, -1.4113, 0, 0, 2.0623, 0, 0.4308, 0, -1.3717, 0, 0, 2.0623, 0, 0.4308, 0],
        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]]}


def curve(keys, times, t):
    """The upper body's target at time t: the cubic Hermite curve through the keys (Catmull-Rom tangents, zero speed
    at the first and the last key); the first key before it, the last after it."""
    p = np.asarray(keys, float)
    tt = np.asarray(times, float)
    if t <= tt[0]:
        return p[0]
    if t >= tt[-1]:
        return p[-1]
    v = np.zeros_like(p)
    v[1:-1] = (p[2:] - p[:-2]) / (tt[2:] - tt[:-2])[:, None]
    i = int(np.searchsorted(tt, t, side="right")) - 1
    h = tt[i + 1] - tt[i]
    u = (t - tt[i]) / h
    return ((2 * u ** 3 - 3 * u ** 2 + 1) * p[i] + (u ** 3 - 2 * u ** 2 + u) * h * v[i]
            + (-2 * u ** 3 + 3 * u ** 2) * p[i + 1] + (u ** 3 - u ** 2) * h * v[i + 1])


def solve(s, tech=None):
    """One walk command row per control step (the walking policy has the legs): the walk's velocity command until
    walk[3] s, then zero; the upper body along the keys; to the end of the episode."""
    tech = tech or TECHNIQUE
    w = s.world
    dt = w.CONTROL_DT
    vx, vy, wz, t_stop = tech["walk"]
    k = 0
    while not s.done:
        t = k * dt
        cmd = (vx, vy, wz) if t < t_stop else (0.0, 0.0, 0.0)
        up = curve(tech["keys"], tech["times"], t + dt)
        if s.step("walk", np.r_[cmd, up, 0.0, 0.0]):
            return
        k += 1
