"""sumo: oshi-zumo on a walking policy, closed loop on privileged state. The robot walks into the opponent at a steady
command on FALCON's walking policy (trained with forces on the hands), forearms forward and low (elbows bent, the
fists at the opponent's chest), steering not at the opponent but at a point beyond it on the line from the ring's
centre through it, so that the push carries the opponent towards the nearest edge of the ring; the opponent's own
walking policy, turning to face the robot, is pushed back and sideways until a foot lands outside the bales or it
goes down. The options lat / tilt_stop / back / edge_rate (off by default) were tried and did not help. Measured
(tests/sumo_robust.py): the default instance wins at 11.6 s (the opponent goes down); 8 of 10 perturbed starts (the
opponent's start time, speed, turn rate, the start distance) and 29 of 40 runs with its own numbers varied by 3 %
win: two balance policies shoving each other make the bout chaotic."""
import math

import numpy as np

P = dict(lat=0.0, lat_gap=0.0, tilt_stop=0.0, back=0.0, edge_rate=0.0, walker="walk_falcon", vx=0.48, vy=0.0, kz=2.4, edge=1.0, sp=-0.6, el=1.2, roll=0.3, wz_max=1.5)


def solve(s, p=None):
    p = {**P, **(p or {})}
    w = s.world
    r, L = w.robot, w.L
    task = w.task
    up = r.rest[12:].copy()
    na = L.NW
    for i, sgn in ((0, 1.0), (1, -1.0)):                 # left arm, right arm: shoulder pitch, roll, yaw, elbow
        up[na + 7 * i + 0] = p["sp"]
        up[na + 7 * i + 1] = sgn * p["roll"]
        up[na + 7 * i + 2] = 0.0
        up[na + 7 * i + 3] = p["el"]
    t_contact = None
    while not s.done:
        pv = task.privileged(w)
        if t_contact is None and task.trackers["first_contact_t"] is not None:
            t_contact = w.t
        ox, oy = pv["opponent_pelvis_xyz_yaw"][:2]
        x, y, _, yaw = pv["pelvis"]
        ex, ey = ox, oy
        rr = math.hypot(ox, oy)
        edge = p["edge"] + (p["edge_rate"] * (w.t - t_contact) if t_contact is not None else 0.0)
        if edge > 0 and rr > 1e-3:                       # aim beyond the opponent, outwards from the centre
            ex, ey = ox + edge * ox / rr, oy + edge * oy / rr
        gap = math.hypot(ox - x, oy - y)
        if p["lat"] and gap > p["lat_gap"] and t_contact is None:   # come in at the opponent's side first
            ux, uy = (ox - x) / gap, (oy - y) / gap
            ex, ey = ox - p["lat"] * uy, oy + p["lat"] * ux
        err = (math.atan2(ey - y, ex - x) - yaw + math.pi) % (2 * math.pi) - math.pi
        vx = p["vx"]
        if p["tilt_stop"] > 0 and math.acos(max(-1.0, min(1.0, w.upright()))) > p["tilt_stop"]:
            vx = -p["back"]                              # tilted: stop pushing (or step back) until upright again
        cmd = [vx, p["vy"], float(np.clip(p["kz"] * err, -p["wz_max"], p["wz_max"]))]
        if s.step(p["walker"], np.r_[cmd, up, 0, 0]):
            return
