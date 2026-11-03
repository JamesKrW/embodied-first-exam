"""Table tennis (format match, its low level, and the duel against the built-in learned seat): the state table.

The world frame of the match is your frame (spec world: x to your right, y towards the opponent, z up, table top
z = 0). Format match: the scripted opponent is a paddle on the far end line, moved along x within the rail's limits
(sim/match.py `_opp_jl`; it is drawn, it has no arm): its paddle's centre and velocity. Duel: the other robot of the
duel world (bodies and joints *_b) and its paddle."""
from __future__ import annotations

from . import _mj

ARM = ["base_x", "base_y"] + [f"joint{i}" for i in range(1, 8)]
BALL = {"pos": "m", "quat": "wxyz", "lin_vel": "m/s", "ang_vel": "rad/s (the ball's spin)"}
ROBOT = {**_mj.POSE, **_mj.JOINTS}
FEATURE = {"pos": "m", "size": "m (full extents)"}


def _world(m):
    if hasattr(m, "R"):                                   # format match (TTMatch): the match's session
        return m.R.s.m, m.R.s.d, False
    return m.g.m, m.g.d, True                             # duel (TTDuel): the duel world


def entities(m, seat):
    _model, _data, duel = _world(m)
    ents = {"robot": {"kind": "robot", "fields": dict(ROBOT), "what": "your robot: the rail base (body base) and its "
                      "9 joints (base_x, base_y, joint1-7)"},
            "paddle": {"kind": "robot_link", "fields": dict(_mj.POSE), "what": "your paddle's blade centre"},
            "ball": {"kind": "ball", "fields": dict(BALL)},
            "table": {"kind": "scene_feature", "fields": dict(FEATURE)},
            "net": {"kind": "scene_feature", "fields": dict(FEATURE)}}
    if duel:
        ents["opponent"] = {"kind": "opponent", "fields": dict(ROBOT),
                            "what": "the other robot: its rail base and joints (base_x_b, base_y_b, joint1_b-7_b)"}
        ents["opponent_paddle"] = {"kind": "opponent_link", "fields": dict(_mj.POSE)}
    else:
        ents["opponent_paddle"] = {"kind": "opponent", "fields": {"pos": "m", "lin_vel": "m/s"},
                                   "what": "the scripted opponent's paddle (it moves along its end line)"}
    return {"frame": "the match world = your frame (x to your right, y towards the opponent, z up; table top z = 0)",
            "quaternion_order": "wxyz", "entities": ents}


def _feature(model, data, geom):
    import mujoco as mj
    g = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, geom)
    if g < 0:
        return {}
    return {"pos": _mj.r(data.geom_xpos[g]), "size": _mj.r(2 * model.geom_size[g])}


def read(m, seat, names):
    model, data, duel = _world(m)
    out = {}
    for n in names:
        if n == "robot":
            out[n] = {**_mj.body(model, data, "base"), **_mj.joints(model, data, ARM)}
        elif n == "paddle":
            out[n] = _mj.site(model, data, "paddle_center")
        elif n == "ball":
            out[n] = _mj.free_joint(model, data, "ball")
        elif n == "table":
            out[n] = _feature(model, data, "geom_table")
        elif n == "net":
            out[n] = _feature(model, data, "geom_net")
        elif n == "opponent" and duel:
            out[n] = {**_mj.body(model, data, "base_b"), **_mj.joints(model, data, [a + "_b" for a in ARM])}
        elif n == "opponent_paddle":
            if duel:
                out[n] = _mj.site(model, data, "paddle_center_b")
            else:
                import match as M                         # sport/sim/match.py: where the paddle is drawn
                jl = m.R._opp_jl
                out[n] = {"pos": _mj.r([float(jl.q[0]), M.OPP_Y, M.OPP_Z]),
                          "lin_vel": _mj.r([float(jl.qd[0]), 0.0, 0.0])}
    return out
