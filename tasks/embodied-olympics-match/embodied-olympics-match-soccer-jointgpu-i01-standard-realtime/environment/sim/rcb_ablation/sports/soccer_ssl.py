"""RoboCup SSL soccer (format match against the scripted team): the state table, and the no-success-signal edits.

Your team's frame (the observation's: you attack +x; team (x, y) = attack_sign * world (x, y), team yaw = world yaw
(+ pi when attack_sign = -1); the ends change at half time, so the frame follows attack_sign now). Every robot: its
pose on the field (x, y, yaw), its velocity (vx, vy, yaw rate) and whether it is on the field; the ball: position and
velocity. Straight from the simulator (sim/world.py World.robot_pose, World.ball)."""
from __future__ import annotations

from .soccer import hide  # noqa: F401  (a goal's stoppage keeps its kind, not the scoring team)

HIDE_KEYS = ("shootout", "winner_team", "end_how")
ROBOT = {"pose": "[x m, y m, yaw rad]", "vel": "[vx m/s, vy m/s, yaw rate rad/s]", "on_field": "bool"}
BALL = {"pos": "m", "vel": "m/s"}


def _teams(m, seat):
    from sim.frames import other                        # sport/sim/frames.py
    me = m.team_of_seat(seat)
    return me, other(me)


def entities(m, seat):
    me, them = _teams(m, seat)
    ents = {"ball": {"kind": "ball", "fields": dict(BALL)}}
    for rid in m.w.ids:
        if rid[0] == me:
            ents[f"robot_{rid[1]}"] = {"kind": "robot", "fields": dict(ROBOT), "what": f"your robot {rid[1]}"}
    for rid in m.w.ids:
        if rid[0] == them:
            ents[f"opponent_{rid[1]}"] = {"kind": "opponent", "fields": dict(ROBOT),
                                          "what": f"the opponent's robot {rid[1]}"}
    return {"frame": "your team's frame now (you attack +x; y to the left facing +x; z up; yaw counter-clockwise "
                     "from +x)", "quaternion_order": None, "entities": ents}


def _r(x, n=5):
    return [round(float(v), n) for v in x]


def read(m, seat, names):
    from sim.frames import to_team, vec_to_team
    w = m.w
    me, them = _teams(m, seat)
    s = w.sign(me)
    by = {f"robot_{rid[1]}": rid for rid in w.ids if rid[0] == me}
    by |= {f"opponent_{rid[1]}": rid for rid in w.ids if rid[0] == them}
    out = {}
    for n in names:
        if n == "ball":
            p, v = w.ball.pos, w.ball.vel
            x, y = to_team(s, float(p[0]), float(p[1]))
            vx, vy = vec_to_team(s, float(v[0]), float(v[1]))
            out[n] = {"pos": _r([x, y, p[2]]), "vel": _r([vx, vy, v[2]])}
        elif n in by:
            rid = by[n]
            p, v = w.robot_pose(rid)
            x, y, yaw = to_team(s, p[0], p[1], p[2])
            vx, vy = vec_to_team(s, v[0], v[1])
            out[n] = {"pose": _r([x, y, yaw]), "vel": _r([vx, vy, v[2]]), "on_field": bool(w.on_field[rid])}
    return out
