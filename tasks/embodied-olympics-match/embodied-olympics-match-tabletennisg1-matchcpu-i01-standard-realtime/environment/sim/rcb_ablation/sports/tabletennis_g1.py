"""Table tennis with Unitree G1 humanoids (format match against the scripted G1): the state table.

Your own frame (the observation's: the world for side A; side B's frame is the world turned 180 degrees about z).
Both humanoids live in one MuJoCo world: each one's pelvis (its floating base) and every joint, its paddle's centre
(site <side>_hand_center); the ball (a free body), the table and the net. Straight from the simulator's data."""
from __future__ import annotations

from . import _mj

ROBOT = {**_mj.POSE, **_mj.JOINTS}
BALL = {"pos": "m", "quat": "wxyz", "lin_vel": "m/s", "ang_vel": "rad/s (the ball's spin)"}
FEATURE = {"pos": "m", "size": "m (full extents)"}


def _side(seat):
    return "B" if seat == "B" else "A"


def entities(m, seat):
    ents = {"robot": {"kind": "robot", "fields": dict(ROBOT),
                      "what": "your G1: its pelvis (the floating base) and its joints (names without the side prefix)"},
            "paddle": {"kind": "robot_link", "fields": dict(_mj.POSE), "what": "your paddle's blade centre"},
            "ball": {"kind": "ball", "fields": dict(BALL)},
            "table": {"kind": "scene_feature", "fields": dict(FEATURE)},
            "net": {"kind": "scene_feature", "fields": dict(FEATURE)},
            "opponent": {"kind": "opponent", "fields": dict(ROBOT), "what": "the opponent's G1: pelvis and joints"},
            "opponent_paddle": {"kind": "opponent_link", "fields": dict(_mj.POSE)}}
    return {"frame": "your own frame (the observation's: x to your right, y towards the opponent, z up)",
            "quaternion_order": "wxyz", "entities": ents}


def _robot(model, data, side):
    js = _mj.joint_names(model, rf"^{side}_")
    out = {**_mj.body(model, data, f"{side}_pelvis"), **_mj.joints(model, data, js)}
    out["joint_pos"] = {k[2:]: v for k, v in out["joint_pos"].items()}
    out["joint_vel"] = {k[2:]: v for k, v in out["joint_vel"].items()}
    return out


def _feature(model, data, geom):
    import mujoco as mj
    g = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, geom)
    return {} if g < 0 else {"pos": _mj.r(data.geom_xpos[g]), "size": _mj.r(2 * model.geom_size[g])}


def read(m, seat, names):
    data = m.g.d
    model = data.model
    me = _side(seat)
    them = "B" if me == "A" else "A"
    out = {}
    for n in names:
        if n == "robot":
            v = _robot(model, data, me)
        elif n == "paddle":
            v = _mj.site(model, data, f"{me}_hand_center")
        elif n == "ball":
            v = _mj.free_joint(model, data, "ball")
        elif n == "table":
            v = _feature(model, data, "geom_table")
        elif n == "net":
            v = _feature(model, data, "geom_net")
        elif n == "opponent":
            v = _robot(model, data, them)
        elif n == "opponent_paddle":
            v = _mj.site(model, data, f"{them}_hand_center")
        else:
            continue
        out[n] = _mj.turn(v) if me == "B" else v
    return out
