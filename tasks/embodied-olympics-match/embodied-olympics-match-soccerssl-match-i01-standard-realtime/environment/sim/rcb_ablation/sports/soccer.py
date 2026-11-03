"""Soccer (T1 humanoids 2 v 2, format match against the scripted team; the high level and the joint level): the
state table, and the no-success-signal edits.

Your team's frame (spec frame: you attack the goal at x = +5, defend x = -5, y to the left facing +x, z up). Every
robot: its trunk's pose and velocity and its leg joints; the ball (a free body). Straight from the simulator's MuJoCo
data, turned from the world into your team's frame (sport soccer.py to_team: a 180 degree turn for team B)."""
from __future__ import annotations

from . import _mj

HIDE_KEYS = ("shootout",)
ROBOT = {**_mj.POSE, **_mj.JOINTS}
BALL = {"pos": "m", "quat": "wxyz", "lin_vel": "m/s", "ang_vel": "rad/s"}


def hide(obj):
    """A stoppage for a goal keeps its kind (the restart follows from it) but not which team scored."""
    def walk(x):
        if isinstance(x, dict):
            st = x.get("stoppage")
            if isinstance(st, dict) and "goal" in str(st.get("kind", "")).lower():
                x = {**x, "stoppage": {k: v for k, v in st.items() if k != "team"}}
            return {k: walk(v) for k, v in x.items()}
        if isinstance(x, list):
            return [walk(v) for v in x]
        return x
    return walk(obj)


def _robots(seat):
    import soccer as S                                  # sport/soccer.py
    team = seat[0]
    return team, S.TEAM_ROBOTS[team], S.TEAM_ROBOTS[S.OTHER[team]]


def entities(m, seat):
    team, mine, theirs = _robots(seat)
    ents = {"ball": {"kind": "ball", "fields": dict(BALL)}}
    for r in mine:
        ents[f"robot_{r[1:]}"] = {"kind": "robot", "fields": dict(ROBOT),
                                  "what": f"your robot {r[1:]}: its trunk and its leg joints"}
    for r in theirs:
        ents[f"opponent_{r[1:]}"] = {"kind": "opponent", "fields": dict(ROBOT),
                                     "what": f"the opponent's robot {r[1:]}: its trunk and its leg joints"}
    return {"frame": "your team's frame (you attack the goal at x = +5 and defend x = -5; y to the left facing +x; z "
                     "up)", "quaternion_order": "wxyz", "entities": ents}


def _robot(model, data, r, turned):
    js = [n for n in _mj.joint_names(model, rf"^{r}:")]
    out = {**_mj.body(model, data, f"{r}:Trunk"), **_mj.joints(model, data, js)}
    out["joint_pos"] = {k.split(":", 1)[1]: v for k, v in out["joint_pos"].items()}
    out["joint_vel"] = {k.split(":", 1)[1]: v for k, v in out["joint_vel"].items()}
    return _mj.turn(out) if turned else out


def read(m, seat, names):
    data = m.w.d
    model = data.model
    team, mine, theirs = _robots(seat)
    turned = team != "A"
    by = {f"robot_{r[1:]}": r for r in mine} | {f"opponent_{r[1:]}": r for r in theirs}
    out = {}
    for n in names:
        if n == "ball":
            b = _mj.free_joint(model, data, "ball")
            out[n] = _mj.turn(b) if turned else b
        elif n in by:
            out[n] = _robot(model, data, by[n], turned)
    return out
