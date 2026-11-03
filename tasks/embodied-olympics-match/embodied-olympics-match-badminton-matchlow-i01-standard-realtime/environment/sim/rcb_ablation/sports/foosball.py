"""Foosball (format match against the scripted opponent): the state table.

Your local frame (spec geometry frame: u towards the opponent goal, v to your left, z up; playfield z = 0, centre
(0, 0)). The ball, your four rods and the opponent's four rods: each rod's body (its figures' bar) and its two joints
(slide along the bar, spin about it) straight from the simulator."""
from __future__ import annotations

from . import _mj

BALL = {"pos": "m", "quat": "wxyz", "lin_vel": "m/s", "ang_vel": "rad/s"}
ROD = {"pos": "m (the rod body's origin)", "slide": "m", "angle": "rad", "slide_velocity": "m/s",
       "angular_velocity": "rad/s"}


def _foos():
    import foos                                         # sport/foos.py
    return foos


def _sides(seat):
    return seat[0], ("B" if seat[0] == "A" else "A")


def entities(m, seat):
    F = _foos()
    ents = {"ball": {"kind": "ball", "fields": dict(BALL)}}
    for r in F.RODS:
        ents[f"rod_{r}"] = {"kind": "robot_link", "fields": dict(ROD), "what": f"your {r} rod"}
    for r in F.RODS:
        ents[f"opponent_rod_{r}"] = {"kind": "opponent_link", "fields": dict(ROD), "what": f"the opponent's {r} rod"}
    return {"frame": "your local frame (u towards the opponent goal, v to your left, z up; playfield z = 0, centre "
                     "(0, 0)); fields pos / lin_vel / ang_vel are (u, v, z)",
            "quaternion_order": "wxyz", "entities": ents}


def _rod(model, data, side, rod, turned):
    import mujoco as mj
    b = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, f"{side}_{rod}")
    js = _mj.joints(model, data, [f"{side}_{rod}_slide", f"{side}_{rod}_spin"])
    out = {"pos": _mj.r(data.xpos[b]), "slide": js["joint_pos"].get(f"{side}_{rod}_slide"),
           "angle": js["joint_pos"].get(f"{side}_{rod}_spin"),
           "slide_velocity": js["joint_vel"].get(f"{side}_{rod}_slide"),
           "angular_velocity": js["joint_vel"].get(f"{side}_{rod}_spin")}
    return _mj.turn(out) if turned else out


def read(m, seat, names):
    F = _foos()
    data = m.r.data
    model = data.model
    me, them = _sides(seat)
    turned = F.OWN[me] < 0                              # world (u, v) = OWN * local (u, v)
    out = {}
    for n in names:
        if n == "ball":
            b = _mj.free_joint(model, data, "ball")
            out[n] = _mj.turn(b) if turned else b
        elif n.startswith("opponent_rod_"):
            out[n] = _rod(model, data, them, n[len("opponent_rod_"):], turned)
        elif n.startswith("rod_"):
            out[n] = _rod(model, data, me, n[len("rod_"):], turned)
    return out
