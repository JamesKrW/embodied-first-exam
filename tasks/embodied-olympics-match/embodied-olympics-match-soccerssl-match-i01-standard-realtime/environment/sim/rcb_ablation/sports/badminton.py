"""Badminton (format match-wallclock, its low level, and the duel against the built-in learned seat): the state table.

Your frame (spec frame: x to your right, y towards the opponent, z up; your half y < 0). The shuttle is the sport's
point mass with drag (not a MuJoCo body): its position and velocity. Format match: the scripted opponent is a player
standing at a point of its half, moving on the court (sim/bm.py Match.opp_xy; it is drawn, it has no arm): its
position and velocity on the floor. Duel: the other robot runs in its own copy of the model in its own frame (sim/
bduel.py); its base, joints and racket are turned into your frame here."""
from __future__ import annotations

from . import _mj

ARM = ["base_x", "base_y"] + [f"joint{i}" for i in range(1, 8)]
ROBOT = {**_mj.POSE, **_mj.JOINTS}
SHUTTLE = {"pos": "m", "lin_vel": "m/s", "in_flight": "bool (false: at rest, tossed but not yet free, or landed)"}


def _duel(m):
    return hasattr(m.g, "sides")


def entities(m, seat):
    ents = {"robot": {"kind": "robot", "fields": dict(ROBOT),
                      "what": "your robot: the sliding base (body base) and its 9 joints (base_x, base_y, joint1-7)"},
            "racket": {"kind": "robot_link", "fields": dict(_mj.POSE), "what": "your racket head's centre"},
            "shuttle": {"kind": "ball", "fields": dict(SHUTTLE)}}
    if _duel(m):
        ents["opponent"] = {"kind": "opponent", "fields": dict(ROBOT),
                            "what": "the other robot (the same robot as yours): its base and its 9 joints"}
        ents["opponent_racket"] = {"kind": "opponent_link", "fields": dict(_mj.POSE)}
    else:
        ents["opponent"] = {"kind": "opponent", "fields": {"pos": "m", "lin_vel": "m/s"},
                            "what": "the scripted opponent: the point of the floor it stands on (z = 0)"}
    return {"frame": "your frame (x to your right, y towards the opponent, z up; your half y < 0)",
            "quaternion_order": "wxyz", "entities": ents}


def _robot(phys, turned):
    model, data = phys.m, phys.d
    rob = {**_mj.body(model, data, "base"), **_mj.joints(model, data, ARM)}
    rk = _mj.site(model, data, "racket_center")
    return (_mj.turn(rob), _mj.turn(rk)) if turned else (rob, rk)


def read(m, seat, names):
    g, out = m.g, {}
    duel = _duel(m)
    if duel:
        other = "B" if seat == "A" else "A"
        mine, theirs = g.sides[seat].s, g.sides[other].s
        # each robot's own model is in its own frame; the shuttle is in A's frame (the world)
        rob, rk = _robot(mine, False)
        orob, ork = _robot(theirs, True)
        sp, sv, live = g.sh_p, g.sh_v, g.sh_live
        if seat == "B":
            sp, sv = _mj.turn({"pos": list(sp)})["pos"], _mj.turn({"pos": list(sv)})["pos"]
    else:
        rob, rk = _robot(g.s, False)
        sp, sv, live = g.s.sh_p, g.s.sh_v, g.s.sh_live
    for n in names:
        if n == "robot":
            out[n] = rob
        elif n == "racket":
            out[n] = rk
        elif n == "shuttle":
            out[n] = {"pos": _mj.r(sp), "lin_vel": _mj.r(sv), "in_flight": bool(live)}
        elif n == "opponent":
            if duel:
                out[n] = orob
            else:
                from bm import CONTROL_DT                 # sport/sim/bm.py
                p0, p1 = g.opp_xy(), g.opp_xy(g.k + 1)
                out[n] = {"pos": _mj.r([p0[0], p0[1], 0.0]),
                          "lin_vel": _mj.r([(p1[0] - p0[0]) / CONTROL_DT, (p1[1] - p0[1]) / CONTROL_DT, 0.0])}
        elif n == "opponent_racket" and duel:
            out[n] = ork
    return out
