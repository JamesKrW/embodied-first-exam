"""Standard-mode sim service for the MetaWorld+ tasks (wire protocol eai-standard/2.1): robot as a service.

Derived from the MuJoCo service (images/mujoco/limited/server_mj.py) and identical to it in protocol and lifecycle:
GET /spec, GET /status, POST /observe, POST /submit, binary RCB1 responses, no reset, crash recovery (journal +
per-call checkpoints, at most RCB_MAX_RESUMES resumes), startup watchdog, SIGUSR1 from the collect hook ends the
episode, /data evidence. Differences, all from Meta-World:
- the action type is the tool module's (mw_tool.ACTION_TYPE: `ee_delta`, Meta-World's native [dx, dy, dz, grip]);
  Meta-World's Sawyer has no arm actuators (the hand is driven by a mocap weld), so there is no `joint_position`;
- watching (/observe with seconds > 0) holds the hand still (S.hold_targets()), since actions are deltas;
- cameras and proprioception come from the tool module (mw_tool.CAMERA_VIEWS, mw_tool.PROPRIO_LAYOUT);
- like the MuJoCo service since the migration (docs/protocol/README.md §7): a simulator error (500) ends the episode with
  reason "error", an oversized body is answered with 413. Requests are validated in full before any physics runs: a
  malformed one (`dt` / `seconds` / `video_fps` not a finite number, `cameras` not a list of names, ...) is a 400 that
  changes nothing.
- final.json is `finalized` as soon as the verdict is in it; the video is rendered after that and never changes or
  delays the verdict (`video_pending` while it renders; `video_done` tells the collect hook that nothing more comes).
- `time_s` / `time_left_s` count from the agent's first request to the service, not from the service's start.
- camera calibration: the pose is that of the viewpoint the image is rendered from (a free camera's is the mean of the
  scene's two eye cameras; scene.camera[0] alone is the left eye, 3.4 cm to the side).

What the agent gets: camera RGB (+ depth) with each camera's calibration, the robot's own state (hand position, gripper
opening, mocap target, arm and finger joints, last command) and the task's target point. Never an object's pose.

Ablation (off unless the task sets RCB_STATE_QUERY=1; nothing changes when off): GET /state/entities and POST
/state/query serve state_table(), the privileged state in the world frame of /spec "frames" (quaternions w x y z): the
Sawyer (base pose, velocity, height above the floor, every joint), its hand, finger pads and tcp, the task objects (the
bodies Meta-World's object observation reads, and every free body: pose, velocity, size, bbox), every articulated part
with its joints and ranges, and the fixed scene bodies. A query reads a scratch copy of the live state (MjRead), under
the lock between calls; it takes no control step and leaves the clock as it is.
The episode runs until the agent's time is up; the state after the last executed control step is graded.

Grading (owner 2026-10-17; docs/benchmarks/metaworldplus.md "Grading"): the verdict is the live
episode's. When the episode ends, Meta-World's own success check (`evaluate_state`, the one the privileged verifier
applies; mw_tool.Session.check()) is read on the live simulator's state after the last executed control step, and
that is `success`: nothing is replayed to grade it. The check has no hold or settle window (it is a predicate on one
state), so the live rule is the replay's rule. The frozen start is checked when the episode begins: the instance's
SHA-256 (RCB_INSTANCE_SHA) and the starting state's hash (RCB_START_SHA = SHA-256 of the instance's MuJoCo
integration state, which the scene is restored to), both recorded by scripts/metaworldplus/generate.py from the frozen
instance; a mismatch gives up before the agent is served (instance_mismatch, an infrastructure failure).

Evidence under /data: episode.jsonl (every call), calls/NNNNNN/ (meta + actions), and at the end trajectory.npz
((T, 4) per control step, the deliverable: it replays the episode from the frozen instance), final.json with the
live verdict (graded_by "live", replays 0, the live end state's hash), then replay.mp4 and replay.last.png, rendered
from trajectory.npz in one pass after the verdict is on disk (video_source "render"; for people only).
"""

from __future__ import annotations

import copy
import errno
import faulthandler
import importlib
import inspect
import json
import math
import os
import re
import signal
import sys
import tempfile
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rcb1  # noqa: E402

PROTOCOL = "eai-standard/2.1"
DATA = os.environ.get("RCB_DATA", "/data")  # RCB_DATA / RCB_PORT: test hooks (never set in tasks)
os.makedirs(os.path.join(DATA, "calls"), exist_ok=True)
TOOL = importlib.import_module(os.environ.get("RCB_TOOL", "mw_tool"))
INSTANCE = os.environ.get("RCB_INSTANCE", os.path.join(os.path.dirname(__file__), "instance.npz"))
MAX_CHUNK_S = float(os.environ.get("RCB_MAX_CHUNK_S", "10"))
MAX_WATCH_S = float(os.environ.get("RCB_MAX_WATCH_S", "30"))
MAX_CONTROL_STEPS = int(os.environ.get("RCB_MAX_CONTROL_STEPS", "0"))  # technical episode cap (0 = none)
MAX_VIDEO_FPS = 30
MAX_FRAMES_PER_CALL = 3000  # frames x cameras per response: bounds the service's memory
WALL_CLOCK_S = float(os.environ.get("RCB_WALL_CLOCK_S", "3600"))
# The unlimited-reset ablation (off unless the task sets RCB_STATE_OPS=1; nothing changes when off): POST /reset
# (RCB_RESET_BUDGET times; back to the frozen start in place: the instance's model fields, env caches, random states and
# MuJoCo integration state, as mw_tool.Session.reset() restores them) and POST /save_state and /load_state (snapshot:
# mw_tool.snapshot() of the env plus the session's observation, reward and last command, and the graded rows, in
# memory); every reset and load begins a new episode, only the last one is graded; meta and /status add episode,
# resets_left, control_steps_total, saves and loads; final.json adds resets, reset_budget, episodes, saves, loads,
# episodes_log, first_success and, for a success, resets_at_success and loads_at_success; a crash resume replays every
# reset, save and load at its journal row.
STATE_OPS = os.environ.get("RCB_STATE_OPS") == "1"
RESET_BUDGET = max(0, int(os.environ.get("RCB_RESET_BUDGET", "0") or 0)) if STATE_OPS else 0
MAX_HANDLES = int(os.environ.get("RCB_MAX_STATE_HANDLES", "32"))
CAN_SAVE = STATE_OPS and bool(RESET_BUDGET)
# The state-query ablation (off unless the task sets RCB_STATE_QUERY=1; nothing changes when off): GET /state/entities
# lists what the agent may ask about (the adapter's StateTable: every entity's kind and fields, with units), and POST
# /state/query {"names": [...], "fields": [...]} answers their current values in the world frame. A query only reads:
# no control step, the clock neither advances nor pauses (on a realtime clock it answers at the current simulation
# time), served only while the episode runs (409 after its end). episode.jsonl logs each answered query, final.json
# adds state_queries (their count). A service without a state table refuses to start with the switch on.
STATE_QUERY = os.environ.get("RCB_STATE_QUERY") == "1"
STATE_KINDS = ("robot", "robot_link", "object", "articulation", "scene_feature")
MAX_QUERY_NAMES = 512
STATE_QUERY_NOTE = (
    "privileged state (an ablation of this run): GET /state/entities lists every entity you may query, {\"frame\", "
    "\"quaternion_order\", \"entities\": {name: {\"kind\", \"fields\": {field: unit}}}}; kinds are robot, robot_link, "
    "object, articulation and scene_feature. POST /state/query {\"names\": [names], \"fields\": [fields] (optional; "
    "default: every field of each name)} answers JSON {\"state\": {name: {field: value}}, \"control_steps\", "
    "\"sim_time_s\", \"state_queries\"}: the current values in the world frame `frame`, in the units listed; a "
    "requested field that an entity does not have is left out of its entry. An unknown name or field is a 400 that "
    "lists the valid ones. A query only reads the simulator: it takes no control step and does not advance or pause "
    "the clock (on a realtime clock it answers at the current simulation time). It is served while the episode runs "
    "(409 after it ended). Every query is counted (state_queries)")


def state_value(v, digits: int = 6):
    """A queried value as JSON holds it: numbers rounded (NaN / inf as null), arrays as (nested) lists, dicts, lists and
    tuples taken apart, strings, bools and None as they are."""
    if isinstance(v, dict):
        return {str(k): state_value(x, digits) for k, x in v.items()}
    if v is None or isinstance(v, str):
        return v
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return round(f, digits) if np.isfinite(f) else None
    if isinstance(v, np.ndarray):
        return state_value(v.tolist(), digits)
    if isinstance(v, (list, tuple)):
        return [state_value(x, digits) for x in v]
    raise TypeError(f"a state value must be numbers, arrays, strings or dicts of them, not {type(v).__name__}")


class StateTable:
    """The state-query ablation's entities (RCB_STATE_QUERY=1), built by an adapter once its scene is built:
        add(name, kind, {field: (unit, getter)}, note=None)
    for each entity, kind one of STATE_KINDS; getter() reads the field's current value from the live simulator, in the
    world frame (an array, a number, a bool, a string or a dict of them). The usual fields: pos [x, y, z] m, quat [4]
    (quaternion_order), lin_vel [3] m/s, ang_vel [3] rad/s, height_above_ground m, joint_pos / joint_vel {joint: rad or
    m (/s)}, size [3] m (full extents along the body axes), bbox [[x, y, z] min, [x, y, z] max] m; a benchmark adds its
    own (a coverage fraction, a line's endpoints). The entities and their fields are fixed for the episode. Never a
    task predicate, a success flag, a progress or a grader value."""

    def __init__(self, frame: str, quaternion_order: str = "wxyz", note: str | None = None):
        if quaternion_order not in ("wxyz", "xyzw"):
            raise ValueError("quaternion_order is wxyz or xyzw")
        self.frame, self.quaternion_order, self.note = str(frame), quaternion_order, note
        self._ents: dict = {}

    def add(self, name: str, kind: str, fields: dict, note: str | None = None) -> "StateTable":
        if kind not in STATE_KINDS:
            raise ValueError(f"kind {kind!r} is not one of {STATE_KINDS}")
        if not isinstance(name, str) or not name or name in self._ents:
            raise ValueError(f"entity name {name!r} empty or added twice")
        if not fields:
            raise ValueError(f"entity {name!r} has no fields")
        for f, spec in fields.items():
            if not (isinstance(f, str) and isinstance(spec, tuple) and len(spec) == 2 and isinstance(spec[0], str)
                    and callable(spec[1])):
                raise ValueError(f"{name}.{f}: a field is (unit text, getter)")
        self._ents[name] = {"kind": kind, "fields": dict(fields), "note": note}
        return self

    def entities(self) -> dict:
        ents = {n: {"kind": e["kind"], "fields": {f: u for f, (u, _) in e["fields"].items()},
                    **({"note": e["note"]} if e["note"] else {})} for n, e in self._ents.items()}
        return {"frame": self.frame, "quaternion_order": self.quaternion_order, **({"note": self.note} if self.note else {}),
                "entities": ents}

    def query(self, plan: dict) -> dict:
        """plan = {name: [fields]}, already checked (plan_state_query) -> {name: {field: value}}."""
        return {n: {f: state_value(self._ents[n]["fields"][f][1]()) for f in fs} for n, fs in plan.items()}


def check_state_table(table) -> dict:
    """The table's entities() as GET /state/entities serves it, checked (a malformed table refuses to start)."""
    if not (callable(getattr(table, "entities", None)) and callable(getattr(table, "query", None))):
        raise TypeError("RCB_STATE_QUERY=1: the state table must have entities() and query(plan)")
    ents = table.entities()
    if not isinstance(ents, dict) or not isinstance(ents.get("entities"), dict) or not ents["entities"] \
            or not isinstance(ents.get("frame"), str) or ents.get("quaternion_order") not in ("wxyz", "xyzw"):
        raise TypeError("RCB_STATE_QUERY=1: entities() must give frame, quaternion_order and a non-empty entities dict")
    for n, e in ents["entities"].items():
        if e.get("kind") not in STATE_KINDS or not isinstance(e.get("fields"), dict) or not e["fields"] \
                or not all(isinstance(u, str) for u in e["fields"].values()):
            raise TypeError(f"RCB_STATE_QUERY=1: entity {n!r} needs a kind in {STATE_KINDS} and fields {{field: unit}}")
    json.dumps(ents, allow_nan=False)
    return ents


def plan_state_query(ents: dict, body: dict) -> tuple[dict | None, dict | None]:
    """A /state/query body -> ({name: [fields]}, None), or (None, the 400's JSON body)."""
    table = ents["entities"]
    extra = sorted(set(body) - {"names", "fields"})
    if extra:
        return None, {"error": f"unknown keys {extra}: the body is {{\"names\": [...], \"fields\": [...]}}"}
    names = body.get("names")
    if not isinstance(names, list) or not names or not all(isinstance(n, str) for n in names):
        return None, {"error": "names must be a non-empty list of entity names (GET /state/entities)",
                      "names": list(table)}
    names = list(dict.fromkeys(names))
    if len(names) > MAX_QUERY_NAMES:
        return None, {"error": f"at most {MAX_QUERY_NAMES} names per query"}
    unknown = [n for n in names if n not in table]
    if unknown:
        return None, {"error": "unknown names", "unknown": unknown, "names": list(table)}
    fields = body.get("fields")
    if fields is None:
        return {n: list(table[n]["fields"]) for n in names}, None
    if not isinstance(fields, list) or not fields or not all(isinstance(f, str) for f in fields):
        return None, {"error": "fields must be a non-empty list of field names, or left out for all of them"}
    fields = list(dict.fromkeys(fields))
    have = {f for n in names for f in table[n]["fields"]}
    bad = [f for f in fields if f not in have]
    if bad:
        return None, {"error": "unknown fields for these names", "unknown": bad,
                      "fields": {n: list(table[n]["fields"]) for n in names}}
    return {n: [f for f in fields if f in table[n]["fields"]] for n in names}, None
MAX_BODY = 64 * 1024 * 1024
MAX_RESP = 256 * 1024 * 1024
LOCK = threading.Lock()
E = {"ready": False, "calls": 0, "rows": 0, "done": False, "reason": None, "t0": time.time(), "finalized": False}
# t_first: the agent's first request (its clock starts there); video_pending: the video renders (after the verdict)
E.update({"t_first": None, "video_done": False, "video_pending": False})
if STATE_OPS:  # resets / saves / loads that took effect; journal_rows: control steps of the whole trial (record.f64's
    # length); ep_start: where the current episode begins in it; stopping: the agent's time is up (no new episode)
    E.update({"resets": 0, "saves": 0, "loads": 0, "ep_start": 0, "journal_rows": 0, "stopping": False})
HANDLES: dict = {}  # RCB_STATE_OPS: handle -> (simulator snapshot, the graded rows), LRU order
if STATE_QUERY:
    E["state_queries"] = 0  # /state/query calls answered

# SIGUSR1 (the collect hook's "episode over") may arrive before the service is ready, e.g. during a resume replay; its
# default action would kill the process. Until the real handler is installed, only remember it.
_USR1 = {"pending": False}
signal.signal(signal.SIGUSR1, lambda *_: _USR1.__setitem__("pending", True))

# Startup watchdog (as in server_mj.py): a scene build that hangs must end the process so compose's restart retries.
# faulthandler's C watchdog thread dumps every thread's traceback and _exit(1)s; cancelled once the scene exists.
STARTUP_TIMEOUT_S = float(os.environ.get("RCB_STARTUP_TIMEOUT_S", "300"))
faulthandler.dump_traceback_later(STARTUP_TIMEOUT_S, exit=True)

MAX_RESUMES = int(os.environ.get("RCB_MAX_RESUMES", "2"))
REC_PATH = os.path.join(DATA, "record.f64")  # journal: every executed control step's action, float64 (N,) rows


def _events() -> list:
    try:
        return [json.loads(line) for line in open(os.path.join(DATA, "episode.jsonl")) if line.strip()]
    except OSError:
        return []


def _write_json(path: str, obj: dict) -> None:
    """Atomic: a reader (the collect hook, the verifier) never sees a half-written file."""
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _verdict_on_disk(fin: dict) -> bool:
    """final.json holds a verdict: the live one (graded_by, since 2026-10-17) or, from a service of before, a replay's
    (deterministic), or the service's failure to write one (finalize_error: infrastructure, final as well). It is never
    rewritten."""
    return bool(fin.get("finalized")) and ("graded_by" in fin or "deterministic" in fin or "finalize_error" in fin)


def _give_up(reason: str, infra: bool) -> None:
    """End the trial's episode without a new one: mark it failed, keep the PID file for the collect hook, and idle."""
    log({"ev": "restart_refused", "reason": reason})
    try:
        prev = json.load(open(os.path.join(DATA, "final.json")))
    except Exception:  # noqa: BLE001
        prev = {}
    if not isinstance(prev, dict):
        prev = {}
    if not _verdict_on_disk(prev):  # a verdict already on disk is kept as it is
        prev.update({"success": False, "done": True, "reason": reason, "finalized": True, "protocol": PROTOCOL})
        if infra:
            prev["infra_error"] = 1
    prev["video_done"] = True  # nothing more will be written: the collect hook stops waiting
    prev["video_pending"] = False  # a video being rendered when the process died is not resumed
    _write_json(os.path.join(DATA, "final.json"), prev)
    open(os.path.join(DATA, "server.pid"), "w").write(str(os.getpid()))
    signal.signal(signal.SIGUSR1, lambda *_: None)
    print(f"restart refused: {reason}", flush=True)
    while True:
        time.sleep(3600)


with open(os.path.join(DATA, "startups.jsonl"), "a") as _f:
    _f.write(json.dumps({"ts": time.time(), "pid": os.getpid()}) + "\n")

# Back off before rebuilding after a failed start: compose restarts immediately.
_n_starts = sum(1 for _ in open(os.path.join(DATA, "startups.jsonl")))
if _n_starts > 1 and not os.path.exists(os.path.join(DATA, "episode.jsonl")):
    time.sleep(min(120, 20 * (_n_starts - 1)))

S = TOOL.Session(INSTANCE, enable_render=True)
E["built"] = True
faulthandler.cancel_dump_traceback_later()  # the scene exists: the startup watchdog stands down
CTRL_DT = float(TOOL.CONTROL_DT)
N = int(TOOL.N)
NAMES = list(TOOL.ACTION_NAMES)
ACTION = dict(TOOL.ACTION_TYPE)
ACTION_NAME = ACTION.pop("name")
PROPRIO_LAYOUT = list(TOOL.PROPRIO_LAYOUT)
RECORD: list = []  # the episode: one action row per control step


def _cams() -> dict:
    """name -> (mujoco camera id or None, free-camera settings or None), from the tool's CAMERA_VIEWS."""
    out = {}
    for name, view in TOOL.CAMERA_VIEWS.items():
        out[name] = (S.model.camera(view["camera"]).id, None) if "camera" in view else (None, view)
    return out


CAMS = _cams()
W, H = 320, 240
_R = {"rgb": None, "depth": None}


def _renderer(depth: bool):
    k = "depth" if depth else "rgb"
    if _R[k] is None:
        _R[k] = S.mj.Renderer(S.model, H, W)
        if depth:
            _R[k].enable_depth_rendering()
    return _R[k]


def _update(r, cam):
    cid, free = CAMS[cam]
    if cid is not None:
        r.update_scene(S.data, camera=cid)
    else:
        c = S.mj.MjvCamera()
        c.lookat[:] = free["lookat"]
        c.distance = free["distance"]
        c.azimuth = free["azimuth"]
        c.elevation = free["elevation"]
        r.update_scene(S.data, camera=c)


def grab(cams, depth: bool):
    imgs, dep = {}, {}
    for c in cams:
        r = _renderer(False)
        _update(r, c)
        imgs[c] = r.render().copy()
        if depth:
            r = _renderer(True)
            _update(r, c)
            dep[c] = r.render().astype(np.float16)
    return imgs, dep


def _moving(cam: str) -> bool:
    """True for a camera whose pose can change during the episode: a tracking / targeting camera, or one on a body that a
    joint or a mocap pose moves. Free cameras and cameras fixed to the world never move (all four MetaWorld+ views)."""
    cid, _ = CAMS[cam]
    if cid is None:
        return False
    m = S.model
    if int(m.cam_mode[cid]) != 0:
        return True
    b = int(m.cam_bodyid[cid])
    while b > 0:
        if int(m.body_jntnum[b]) > 0 or int(m.body_mocapid[b]) >= 0:
            return True
        b = int(m.body_parentid[b])
    return False


MOVING = {c: _moving(c) for c in CAMS}


def cam2world(cam: str) -> np.ndarray:
    """Camera-to-world pose (4x4, OpenGL convention: x right, y up, looks along -z) of the viewpoint the mono RGB / depth
    image is rendered from, in the current state. A model camera: data.cam_xpos / data.cam_xmat, which is what the
    renderer reads. A free camera: the scene's two eye cameras share their axes and sit ipd/2 to either side of the
    viewpoint, so the viewpoint is their mean (scene.camera[0] alone is the LEFT eye, 3.4 cm off)."""
    cid, _ = CAMS[cam]
    if cid is not None:
        R = np.array(S.data.cam_xmat[cid], float).reshape(3, 3)
        pos = np.array(S.data.cam_xpos[cid], float)
    else:
        r = _renderer(False)
        _update(r, cam)
        a, b = r.scene.camera[0], r.scene.camera[1]
        fwd = np.array(a.forward, float)
        up = np.array(a.up, float)
        pos = 0.5 * (np.array(a.pos, float) + np.array(b.pos, float))
        R = np.column_stack([np.cross(fwd, up), up, -fwd])
    return np.vstack([np.column_stack([R, pos]), [0.0, 0.0, 0.0, 1.0]])


def calib() -> dict:
    """Per camera: pinhole intrinsics and the camera-to-world pose in the current state (see cam2world)."""
    out = {}
    for c in CAMS:
        fovy = float(S.model.cam_fovy[CAMS[c][0]]) if CAMS[c][0] is not None else float(S.model.vis.global_.fovy)
        f = 0.5 * H / np.tan(np.deg2rad(fovy) / 2)
        out[c] = {
            "intrinsic_cv": [[f, 0, W / 2], [0, f, H / 2], [0, 0, 1]],
            "cam2world_gl": cam2world(c).tolist(),
            "width": W,
            "height": H,
            "fovy_deg": fovy,
            "moving": MOVING[c],
        }
    return out


def proprio() -> np.ndarray:
    return np.asarray(S.proprio(), np.float32)


def task_info() -> dict:
    return {"target_pos": np.asarray(S.target_pos).tolist()}


SPEC = {
    "protocol": PROTOCOL,
    "simulator": "mujoco",
    "task_text": S.inst.get("text", ""),
    "control_dt": CTRL_DT,
    "max_chunk_s": MAX_CHUNK_S,
    "max_watch_s": MAX_WATCH_S,
    "max_control_steps": MAX_CONTROL_STEPS or None,
    "max_video_fps": MAX_VIDEO_FPS,
    "max_frames_per_call": MAX_FRAMES_PER_CALL,
    "on_service_error": (
        "if a call fails with a connection error or 5xx, the service may have crashed and resumed the same "
        "episode (never a reset): the state is the one after the last call that completed on the service, "
        "which you may not have received; check GET /status (control_steps) before continuing"
    ),
    "cameras": {c: {"rgb": True, "depth": True, "size": [W, H], "moving": MOVING[c]} for c in CAMS},
    "camera_calibration": (
        'meta["cameras"][cam] of every response: intrinsic_cv (pinhole, pixels) and cam2world_gl (4x4, OpenGL axes: x '
        "right, y up, the camera looks along -z; the viewpoint the image is rendered from). The cameras are fixed "
        "(moving = false): the calibration is the same in every response."
    ),
    "action_types": {
        ACTION_NAME: {
            "dim": N,
            "layout": NAMES,
            "lower": S.lower.tolist(),
            "upper": S.upper.tolist(),
            **ACTION,
            "rows": (
                "rows `dt` apart (a multiple of control_dt, >= control_dt); each row is applied at every control step "
                "it covers (interpolate=hold) or ramped linearly from the previous row (linear); clipped to "
                "[lower, upper]. With dt = control_dt both are the same: one row = one native action."
            ),
        },
        "not_yet": ["joint_position", "ee_pose"],
    },
    "proprio_layout": PROPRIO_LAYOUT,
    "frames": "world frame: x to the robot's right, y forward (away from the robot base), z up; metres",
    "budget": {
        "reset": 0,
        "wall_clock_s": WALL_CLOCK_S,
        "clock": (
            "meta time_s / time_left_s count from your first request to the service (any endpoint), not from the "
            "service's start, and keep running through a restart of the service; time_left_s = budget.wall_clock_s - "
            "time_s. The harness counts the same limit from the start of your session, so time_left_s never runs out "
            "before the harness stops you: your real time left is time_left_s minus the time you spent before that "
            "first request. Plan by time_left_s."
        ),
    },
    "episode": "runs until your time is up; the state after the last executed control step is graded",
}


class MjRead:
    """The state-query ablation's read of a live MuJoCo simulation, read-only: refresh() copies the live qpos, qvel and
    mocap poses into a scratch MjData of the same model and runs the position and velocity stages there (mj_kinematics,
    mj_comPos, mj_comVel), so each pose and velocity is the current state's and the live MjData is never written. The
    model and the live data may be given as callables (a simulator that rebuilds them on reset)."""

    def __init__(self, mj, model, data):
        self.mj, self._model, self._live = mj, model, data
        self.m = self.d = None

    def refresh(self) -> "MjRead":
        m = self._model() if callable(self._model) else self._model
        live = self._live() if callable(self._live) else self._live
        if m is not self.m:
            self.m, self.d = m, self.mj.MjData(m)
        d = self.d
        d.qpos[:] = live.qpos
        d.qvel[:] = live.qvel
        if m.nmocap:
            d.mocap_pos[:] = live.mocap_pos
            d.mocap_quat[:] = live.mocap_quat
        d.time = live.time
        self.mj.mj_kinematics(m, d)
        self.mj.mj_comPos(m, d)
        self.mj.mj_comVel(m, d)
        return self

    def vel(self, objtype: str, i: int):
        """(linear m/s, angular rad/s) of a body or site, world frame, at its frame origin."""
        r = np.zeros(6)
        self.mj.mj_objectVelocity(self.m, self.d, getattr(self.mj.mjtObj, f"mjOBJ_{objtype.upper()}"), int(i), r, 0)
        return r[3:].copy(), r[:3].copy()

    def site_quat(self, i: int):
        q = np.zeros(4)
        self.mj.mju_mat2Quat(q, self.d.site_xmat[i])
        return q

    def body_geoms(self, b: int) -> list:
        """The geoms of body b and of every body fixed to it (reached without crossing a joint)."""
        m, out, todo = self.m, [], [int(b)]
        while todo:
            c = todo.pop()
            out += range(int(m.body_geomadr[c]), int(m.body_geomadr[c]) + int(m.body_geomnum[c])) if m.body_geomnum[c] else []
            todo += [k for k in range(c + 1, m.nbody) if int(m.body_parentid[k]) == c and not m.body_jntnum[k]]
        return out

    def _corners(self, g: int):
        """The 8 corners of geom g's own bounding box (model.geom_aabb), world frame, in the current state."""
        c, h = self.m.geom_aabb[g][:3], self.m.geom_aabb[g][3:]
        s = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
        return (c + s * h) @ self.d.geom_xmat[g].reshape(3, 3).T + self.d.geom_xpos[g]

    def size(self, b: int):
        """Full extents [3] (m) of body_geoms(b) along the body's own axes, or None (no geom)."""
        gs = self.body_geoms(b)
        if not gs:
            return None
        p = (np.concatenate([self._corners(g) for g in gs]) - self.d.xpos[b]) @ self.d.xmat[b].reshape(3, 3)
        return p.max(0) - p.min(0)

    def bbox(self, b: int):
        """[[x, y, z] min, [x, y, z] max] (m) of body_geoms(b), world axes, in the current state, or None."""
        gs = self.body_geoms(b)
        if not gs:
            return None
        p = np.concatenate([self._corners(g) for g in gs])
        return [p.min(0), p.max(0)]

    def height_above_ground(self, b: int, exclude: set):
        """The body's height above the first colliding geom straight below its frame origin that belongs to none of the
        bodies in `exclude` (the robot itself), by ray casts on the scratch data; None when nothing is below."""
        m, d = self.m, self.d
        p0 = d.xpos[b].copy()
        p = p0.copy()
        gid = np.zeros(1, np.int32)
        for _ in range(16):
            dist = self.mj.mj_ray(m, d, p, np.array([0.0, 0.0, -1.0]), None, 1, -1, gid)
            if dist < 0 or gid[0] < 0:
                return None
            g = int(gid[0])
            if int(m.geom_bodyid[g]) in exclude or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                p = p - np.array([0.0, 0.0, dist + 1e-4])
                continue
            return float(p0[2] - (p[2] - dist))
        return None

    def subtree(self, root: int) -> set:
        """Body ids of root and every body below it."""
        out, m = {int(root)}, self.m
        for b in range(int(root) + 1, m.nbody):
            if int(m.body_parentid[b]) in out:
                out.add(b)
        return out

    def joints(self, ids) -> dict:
        """{name: qpos address, dof address, range or None} of hinge and slide joints."""
        m, out = self.m, {}
        for j in ids:
            j = int(j)
            if int(m.jnt_type[j]) in (2, 3):              # mjJNT_SLIDE, mjJNT_HINGE
                rng = [float(m.jnt_range[j][0]), float(m.jnt_range[j][1])] if m.jnt_limited[j] else None
                out[m.joint(j).name or f"joint{j}"] = (int(m.jnt_qposadr[j]), int(m.jnt_dofadr[j]), rng)
        return out


def mj_body_fields(R: MjRead, b: int, *, vel: bool = True, size: bool = False, bbox: bool = False,
                   ground_exclude: set | None = None) -> dict:
    """StateTable fields of one MuJoCo body: pos, quat (w x y z), lin_vel / ang_vel (world frame), size, bbox,
    height_above_ground."""
    f = {"pos": ("m", lambda: R.d.xpos[b].copy()), "quat": ("1 (w x y z)", lambda: R.d.xquat[b].copy())}
    if vel:
        f["lin_vel"] = ("m/s", lambda: R.vel("body", b)[0])
        f["ang_vel"] = ("rad/s", lambda: R.vel("body", b)[1])
    if size and R.body_geoms(b):
        ext = R.size(b)
        f["size"] = ("m (full extents of its geoms and those fixed to it, along the body's axes)", lambda: ext)
    if bbox and R.body_geoms(b):
        f["bbox"] = ("m ([min xyz, max xyz] of its geoms and those fixed to it, world axes)", lambda: R.bbox(b))
    if ground_exclude is not None:
        f["height_above_ground"] = ("m", lambda: R.height_above_ground(b, ground_exclude))
    return f


def mj_site_fields(R: MjRead, s: int, *, vel: bool = True) -> dict:
    f = {"pos": ("m", lambda: R.d.site_xpos[s].copy()), "quat": ("1 (w x y z)", lambda: R.site_quat(s))}
    if vel:
        f["lin_vel"] = ("m/s", lambda: R.vel("site", s)[0])
        f["ang_vel"] = ("rad/s", lambda: R.vel("site", s)[1])
    return f


def mj_joint_fields(R: MjRead, joint_ids, *, ranges: bool = False) -> dict:
    """joint_pos / joint_vel {joint: rad or m (/s)} (and joint_range {joint: [lo, hi]}) of hinge and slide joints."""
    J = R.joints(joint_ids)
    if not J:
        return {}
    f = {"joint_pos": ("rad or m", lambda: {n: float(R.d.qpos[a]) for n, (a, _, _) in J.items()}),
         "joint_vel": ("rad/s or m/s", lambda: {n: float(R.d.qvel[v]) for n, (_, v, _) in J.items()})}
    rng = {n: r for n, (_, _, r) in J.items() if r is not None}
    if ranges and rng:
        f["joint_range"] = ("rad or m ([lo, hi])", lambda: rng)
    return f


class MjStateTable:
    """A StateTable whose query() refreshes the MjRead once, then reads every planned field from it."""

    def __init__(self, table, reader: MjRead):
        self.table, self.reader = table, reader

    def add(self, *a, **k):
        self.table.add(*a, **k)
        return self

    def entities(self) -> dict:
        return self.table.entities()

    def query(self, plan: dict) -> dict:
        self.reader.refresh()
        return self.table.query(plan)


def _pose_bodies() -> list:
    """The task objects: the bodies Meta-World's own object observation reads (the env's _get_pos_objects and
    _get_quat_objects name them: body("...") / get_body_com("...")), by body name."""
    names = []
    for fn in ("_get_quat_objects", "_get_pos_objects"):
        try:
            src = inspect.getsource(getattr(type(S.env), fn))
        except (OSError, TypeError, AttributeError):
            continue
        names += re.findall(r'(?:body|get_body_com)\(\s*"([^"]+)"', src)
    names = list(dict.fromkeys(names))
    for n in names:
        S.model.body(n)  # a name the model does not have is a KeyError: the service refuses to start
    if not names:
        raise TypeError(f"RCB_STATE_QUERY=1 but no task object body found for {S.inst.get('env_id')}")
    return names


def _top(m, b: int) -> int:
    """The body's ancestor that hangs from the world body."""
    while int(m.body_parentid[b]) != 0:
        b = int(m.body_parentid[b])
    return b


def state_table():
    """The state-query ablation's table (RCB_STATE_QUERY=1), built once after the scene, in the world frame of /spec
    "frames": the Sawyer (`robot`: its fixed base's pose, velocity and height above the floor, every arm and finger
    joint), its links (`hand`, the two finger pads, `tcp` = the pads' midpoint, as Meta-World's tcp_center), the task
    objects (the bodies Meta-World's own object observation reads, and every other free body) with size and bbox, every
    articulated part (a body of the scene with a hinge or slide joint: button, drawer, door, ...) with its joints, and the
    fixed scene bodies (the table, a hoop, a box) as scene features. Read by MjRead (a scratch copy of the live state);
    the model and data are looked up at every query (Session.reset() rebuilds the env)."""
    m = S.model
    R = MjRead(S.mj, lambda: S.model, lambda: S.data).refresh()
    name = lambda b: m.body(b).name  # noqa: E731
    act_j = [int(m.actuator_trnid[a, 0]) for a in range(m.nu) if int(m.actuator_trntype[a]) == 0]
    roots = sorted({_top(m, int(m.jnt_bodyid[j])) for j in act_j})
    robot = set().union(*(R.subtree(r) for r in roots))
    t = MjStateTable(StateTable(frame=SPEC["frames"] + "; the frame of task.target_pos and of camera_calibration's "
                                "cam2world_gl", quaternion_order="wxyz",
                                note="every pose is a body's frame origin; velocities are world-frame, at that origin"), R)
    for r in roots:
        joints = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in R.subtree(r)]
        t.add("robot" if len(roots) == 1 else name(r), "robot",
              {**mj_body_fields(R, r, ground_exclude=robot), **mj_joint_fields(R, joints, ranges=True)},
              note=f"the Sawyer, base body {name(r)!r} (fixed base; the hand follows a mocap target, proprio mocap_*)")
    bid = {name(b): b for b in range(m.nbody)}
    for link, body in (("hand", "hand"), ("right_pad", "rightpad"), ("left_pad", "leftpad")):
        if body in bid:
            t.add(link, "robot_link", mj_body_fields(R, bid[body]), note=f"body {body!r}")
    sid = {m.site(i).name: i for i in range(m.nsite)}
    if "rightEndEffector" in sid and "leftEndEffector" in sid:
        a_, b_ = sid["rightEndEffector"], sid["leftEndEffector"]
        t.add("tcp", "robot_link", {"pos": ("m", lambda: (R.d.site_xpos[a_] + R.d.site_xpos[b_]) / 2)},
              note="the midpoint of the two finger pads' end-effector sites (Meta-World's tcp_center, proprio tcp_*)")
    jointed = {int(m.jnt_bodyid[j]) for j in range(m.njnt)}
    free = {int(m.jnt_bodyid[j]) for j in range(m.njnt) if int(m.jnt_type[j]) == 0} - robot
    objects = [bid[n] for n in _pose_bodies()] + sorted(free)
    for b in dict.fromkeys(objects):
        if b in robot:
            continue
        hinge = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) == b and int(m.jnt_type[j]) in (2, 3)]
        t.add(name(b), "articulation" if hinge and b not in free else "object",
              {**mj_body_fields(R, b, size=True, bbox=True), **mj_joint_fields(R, hinge, ranges=True)})
    have = set(t.table.entities()["entities"])
    for j in range(m.njnt):
        b = int(m.jnt_bodyid[j])
        if b in robot or int(m.jnt_type[j]) not in (2, 3) or name(b) in have:
            continue
        hinge = [k for k in range(m.njnt) if int(m.jnt_bodyid[k]) == b and int(m.jnt_type[k]) in (2, 3)]
        t.add(name(b), "articulation", {**mj_body_fields(R, b, size=True, bbox=True), **mj_joint_fields(R, hinge, ranges=True)})
        have.add(name(b))
    for b in range(1, m.nbody):
        if (int(m.body_parentid[b]) == 0 and b not in robot and b not in jointed and int(m.body_mocapid[b]) < 0
                and R.body_geoms(b) and name(b) and name(b) not in have):
            t.add(name(b), "scene_feature", mj_body_fields(R, b, vel=False, size=True, bbox=True),
                  note="a fixed body of the scene, with every body fixed below it")
    return t


if STATE_OPS:
    SPEC["budget"]["reset"] = RESET_BUDGET
if RESET_BUDGET:
    SPEC["reset"] = (
        "POST /reset abandons the running episode and starts a new one from the same starting state, "
        "budget.reset times in all; only the last episode is graded (its state when your time is up). The clock "
        "(time_left_s) keeps running. meta and /status add episode, resets_left and control_steps_total"
    )
    SPEC["episode"] += "; with resets, only the last episode is scored"
STATE_TABLE = state_table() if STATE_QUERY else None
STATE_ENTS = check_state_table(STATE_TABLE) if STATE_QUERY else None
if STATE_QUERY:
    SPEC["state_query"] = STATE_QUERY_NOTE
if CAN_SAVE:
    SPEC["state_ops"] = (
        "POST /save_state ({}) stores the current state of the running episode in this service "
        'and answers JSON {"handle": "s<k>", "step": control steps of the episode, ...}: the '
        "handle is all you get, never the state's contents. POST /load_state {\"handle\": h, "
        "cameras, video} restores a state saved earlier in this trial and starts a new episode "
        "there (answered as /reset is); nothing else can be loaded. The task's success rule is "
        f"checked again from your next command on. At most {MAX_HANDLES} handles are kept: saving "
        "more drops the least recently used one (loading it is a 409). Saves and loads do not "
        "count against budget.reset; a success ends the trial (409 after it)"
    )


def _reset_counters() -> dict:
    """RCB_STATE_OPS: the episode counters of meta and /status ({} when off)."""
    if not RESET_BUDGET:
        return {}
    return {
        "episode": E["resets"] + E["loads"] + 1,
        "control_steps_total": E["journal_rows"],
        "resets_left": 0 if E["stopping"] else RESET_BUDGET - E["resets"],
        **({"saves": E["saves"], "loads": E["loads"]} if CAN_SAVE else {}),
    }


def _journal_pos() -> dict:
    """RCB_STATE_OPS: where a checkpoint sits in the journal ({} when off)."""
    if not STATE_OPS:
        return {}
    return {
        "journal_rows": E["journal_rows"],
        "ep_start": E["ep_start"],
        "resets": E["resets"],
        "saves": E["saves"],
        "loads": E["loads"],
    }


def _agent_time() -> float:
    """Seconds since the agent's first request to the service (0 before it): the agent's clock. The simulator is up
    before the agent starts (the harness installs the agent first), so the service's own start is not that moment."""
    return 0.0 if E["t_first"] is None else time.time() - E["t_first"]


def _touch() -> None:
    """Every request passes here: the first one starts the agent's clock (logged, so a resume keeps it)."""
    if E["t_first"] is None:
        E["t_first"] = time.time()
        log({"ev": "first_request", "ts": E["t_first"]})


def meta_common() -> dict:
    out = {
        "calls": E["calls"],
        "rows": E["rows"],
        "control_steps": len(RECORD),
        "sim_steps": len(RECORD),
        "sim_time_s": round(len(RECORD) * CTRL_DT, 4),
        "time_s": round(_agent_time(), 2),
        "time_left_s": max(0.0, round(WALL_CLOCK_S - _agent_time(), 1)),
        "proprio": {"layout": PROPRIO_LAYOUT, "now": proprio().tolist()},
        "task": task_info(),
        "cameras": calib(),
        "done": E["done"],
        "reason": E["reason"],
    }
    if STATE_OPS:
        out.update(_reset_counters())
    return out


def log(ev: dict) -> None:
    with open(os.path.join(DATA, "episode.jsonl"), "a") as f:
        f.write(json.dumps({"ts": time.time(), **ev}) + "\n")


def write_final(extra: dict | None = None) -> None:
    d = {
        "protocol": PROTOCOL,
        "tool": TOOL.__name__,
        "calls": E["calls"],
        "rows": E["rows"],
        "control_steps": len(RECORD),
        "elapsed_s": round((E.get("t_end") or time.time()) - E["t0"], 1),
        "done": E["done"],
        "reason": E["reason"],
        "finalized": E["finalized"],
        "video_done": E["video_done"],
        "video_pending": E["video_pending"],
    }
    d.update(E.get("verdict") or {})
    d.update(extra or {})
    if STATE_OPS:
        d.update(
            reset_budget=RESET_BUDGET,
            resets=E["resets"],
            saves=E["saves"],
            loads=E["loads"],
            episodes=E["resets"] + E["loads"] + 1,
            control_steps_total=E["journal_rows"],
        )
        if "success" in d:  # the first attempt: graded only when no reset or load came after it
            d["first_success"] = bool(d["success"]) and E["resets"] + E["loads"] == 0
        if d.get("success"):  # the graded episode is the last one
            d.update(resets_at_success=E["resets"], loads_at_success=E["loads"])
    if STATE_QUERY:
        d["state_queries"] = E["state_queries"]
    _write_json(os.path.join(DATA, "final.json"), d)


def _every(fps: int) -> int:
    """The Recorder's capture stride: a frame every this many control steps for the requested video_fps."""
    return max(1, int(round(1.0 / (CTRL_DT * max(1, fps)))))


class Recorder:
    def __init__(self, cams, fps, depth):
        self.cams = cams
        self.every = _every(fps)
        self.depth = depth
        self.frames = {c: [] for c in cams}
        self.dep = {c: [] for c in cams}
        self.prop = []
        self.n = 0

    def tick(self, force=False):
        if force or self.n % self.every == 0:
            imgs, dep = grab(self.cams, self.depth)
            for c in self.cams:
                self.frames[c].append(imgs[c])
                if c in dep:
                    self.dep[c].append(dep[c])
            self.prop.append(proprio())
        self.n += 1

    def parts(self, video: str, fps: int):
        """The response's parts, with one copy of the frames: no np.stack; frames/ and depth/ are written into the body
        from the lists (rcb1.frames_part, which hands them over), the mp4 is encoded from the list."""
        out = []
        for c in self.cams:
            fp = rcb1.frames_part(f"frames/{c}", self.frames[c], empty=np.zeros((0, H, W, 3), np.uint8))
            self.frames[c] = []
            fr = fp[2].arrays if isinstance(fp[2], rcb1.Frames) else []
            dt = fp[2].dtype if fr else np.uint8
            if video in ("mp4", "both") and len(fr):
                import imageio.v2 as imageio

                with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as tf:
                    path = tf.name
                try:
                    with imageio.get_writer(
                        path,
                        format="FFMPEG",
                        mode="I",
                        fps=fps,
                        codec="libx264",
                        macro_block_size=1,
                        output_params=["-movflags", "+faststart"],
                    ) as w:
                        for f in fr:
                            w.append_data(np.ascontiguousarray(f, dtype=dt))
                    data = open(path, "rb").read()
                finally:
                    try:
                        os.unlink(path)
                    except OSError:
                        pass
                out.append((f"video/{c}", "mp4", data, {"fps": fps, "n": int(len(fr))}))
            last = rcb1.array_part(f"last/{c}", np.asarray(fr[-1], dtype=dt)) if len(fr) else None
            if video in ("frames", "both"):
                out.append(fp)
            else:
                fr.clear()
            if last:
                out.append(last)
            if self.dep[c]:
                out.append(rcb1.frames_part(f"depth/{c}", self.dep[c]))  # float16 at capture: 2 bytes a pixel
            self.dep[c] = []
        prop = np.stack(self.prop) if self.prop else np.zeros((0, len(PROPRIO_LAYOUT)), np.float32)
        out.append(rcb1.array_part("proprio", prop))
        return out


def _ctrl_step(tg, rec) -> bool:
    """One control step with action `tg`; returns True if the episode ended (technical cap only)."""
    S.step(tg)
    row = np.asarray(S.targets, float).copy()
    RECORD.append(row)
    RECF.write(row.tobytes())
    if STATE_OPS:
        E["journal_rows"] += 1
    rec.tick()
    if MAX_CONTROL_STEPS and len(RECORD) >= MAX_CONTROL_STEPS:
        finish("step limit")
        return True
    return False


# ---- request parsing: everything here runs BEFORE any physics. A malformed request is the caller's mistake: it raises
# ValueError (answered 400, nothing executed), never a simulator error (500, which ends the episode; PROTOCOL §4.6).
VIDEO_MODES = ("mp4", "frames", "both", "none")


def _fps(meta: dict) -> int:
    try:
        fps = int(meta.get("video_fps", 10))
    except (TypeError, ValueError, OverflowError):
        raise ValueError("video_fps must be an integer") from None
    if not 1 <= fps <= MAX_VIDEO_FPS:
        raise ValueError(f"video_fps must be in [1, {MAX_VIDEO_FPS}]")
    return fps


def _num(meta: dict, key: str, default: float) -> float:
    """A finite numeric request field; anything else (null, a string, NaN, Infinity) is a 400."""
    try:
        v = meta.get(key, default)
        if isinstance(v, bool):  # JSON true is no number (float(True) is 1.0)
            raise TypeError
        v = float(v)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"{key} must be a number") from None
    if not math.isfinite(v):
        raise ValueError(f"{key} must be a finite number")
    return v


def _cameras(meta: dict, default: list) -> list:
    """The requested cameras, each once (unknown names are ignored; none left = the default). A repeated name is dropped
    BEFORE the frame budget and any rendering: the recorder keeps frames per name, so repeats would multiply its work."""
    cams = meta.get("cameras")
    if cams is None:
        return list(default)
    if not isinstance(cams, (list, tuple)) or not all(isinstance(c, str) for c in cams):
        raise ValueError(f"cameras must be a list of camera names: {list(CAMS)}")
    return list(dict.fromkeys(c for c in cams if c in CAMS)) or list(default)


def _video(meta: dict, default: str) -> str:
    v = meta.get("video", default)
    if v is None:
        return default
    if not isinstance(v, str) or v not in VIDEO_MODES:
        raise ValueError("video must be mp4|frames|both|none")
    return v


def _frame_budget(cams, fps: int, steps: int, depth: bool) -> None:
    """Refuse (400, before anything runs) a call of `steps` control steps whose Recorder would take more than
    MAX_FRAMES_PER_CALL images: per camera the forced first and last frame and one every _every(fps) steps (the stride
    it really films at, not the requested fps), depth counted again."""
    n = (int(steps) // _every(fps) + 2) * len(cams) * (2 if depth else 1)
    if n > MAX_FRAMES_PER_CALL:
        raise ValueError(
            f"this call would return {n} images (cameras x frames at the capture stride, depth counted again); the "
            f"limit is max_frames_per_call = {MAX_FRAMES_PER_CALL}: use fewer cameras, a lower video_fps or a shorter chunk"
        )


def parse_submit(meta: dict, parts: dict) -> dict:
    """Validate a /submit request completely; returns what do_submit executes. No physics, no state change."""
    if meta.get("type", ACTION_NAME) != ACTION_NAME:
        raise ValueError(f"type must be {ACTION_NAME} (see /spec action_types)")
    raw = parts["actions"].get("data") if isinstance(parts.get("actions"), dict) else meta.get("actions", [])
    try:
        A = np.asarray(raw, dtype=np.float64)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"actions must be a numeric array (T, {N})") from None
    if A.ndim != 2 or A.shape[1] != N:
        raise ValueError(f"{ACTION_NAME} actions must be (T, {N}), got {list(A.shape)}")
    if not np.all(np.isfinite(A)):
        raise ValueError("actions contain NaN/inf")
    dt = _num(meta, "dt", CTRL_DT)
    if not 0 < dt <= 1e6:
        raise ValueError(f"dt must be a positive multiple of control_dt={CTRL_DT}")
    k = int(round(dt / CTRL_DT))
    if k < 1 or abs(k * CTRL_DT - dt) > 1e-6:
        raise ValueError(f"dt must be a positive multiple of control_dt={CTRL_DT}")
    if A.shape[0] * dt > MAX_CHUNK_S + 1e-9:
        raise ValueError(f"chunk longer than max_chunk_s={MAX_CHUNK_S}")
    interp = meta.get("interpolate", "linear")
    if interp not in ("hold", "linear"):
        raise ValueError("interpolate must be hold|linear")
    cams = _cameras(meta, list(CAMS)[:1])
    fps = _fps(meta)
    video = _video(meta, "mp4")
    depth = bool(meta.get("include_depth", False))
    _frame_budget(cams, fps, A.shape[0] * k, depth)
    note = str(meta.get("note") or "")[:500]
    return {"A": A, "dt": dt, "k": k, "interp": interp, "cams": cams, "fps": fps, "video": video, "depth": depth, "note": note}


def parse_observe(meta: dict) -> dict:
    """Validate an /observe request completely; returns what do_observe executes."""
    cams = _cameras(meta, list(CAMS))
    seconds = _num(meta, "seconds", 0)
    fps = _fps(meta)
    video = _video(meta, "frames")
    depth = bool(meta.get("include_depth", False))
    if seconds < 0 or seconds > MAX_WATCH_S:
        raise ValueError(f"seconds must be in [0, {MAX_WATCH_S}]")
    _frame_budget(cams, fps, int(round(seconds / CTRL_DT)), depth)
    return {"cams": cams, "seconds": seconds, "fps": fps, "video": video, "depth": depth}


def do_submit(q: dict):
    A, dt, k, interp, cams, fps, video, depth = (q[x] for x in ("A", "dt", "k", "interp", "cams", "fps", "video", "depth"))
    rec = Recorder(cams, fps, depth)
    rec.tick(force=True)
    prev = np.asarray(S.targets, float).copy()
    n_clip = 0
    done = False
    executed = 0  # rows begun: fewer than the chunk's when the step limit ends the episode inside it
    for row in A.astype(np.float64):
        executed += 1
        tgt = np.clip(row, S.lower, S.upper)
        n_clip += int(np.any(tgt != row))
        for j in range(1, k + 1):
            tg = tgt if interp == "hold" else prev + (tgt - prev) * (j / k)
            if _ctrl_step(tg, rec):
                done = True
                break
        prev = tgt
        if done:
            break
    rec.tick(force=True)
    E["calls"] += 1
    E["rows"] += executed
    RECF.flush()
    out = {
        **meta_common(),
        "chunk_id": E["calls"],
        "type": ACTION_NAME,
        "executed_rows": executed,
        "clipped": n_clip,
        "n_clipped_rows": n_clip,
        "ok": True,
    }
    log(
        {
            "ev": "submit",
            "call": E["calls"],
            "type": ACTION_NAME,
            "rows": int(A.shape[0]),
            "dt": dt,
            "interpolate": interp,
            "cameras": cams,
            "note": q["note"],
            "control_steps": len(RECORD),
            "n_clipped_rows": n_clip,
            "task": task_info(),
            "rows_total": E["rows"],
            **_journal_pos(),
            "state_sha": S.state_sha(),
        }
    )
    d = os.path.join(DATA, "calls", f"{E['calls']:06d}")
    os.makedirs(d, exist_ok=True)
    np.save(os.path.join(d, "actions.npy"), A.astype(np.float64))
    return out, rec.parts(video, fps)


def do_observe(q: dict):
    cams, seconds, fps, video, depth = (q[x] for x in ("cams", "seconds", "fps", "video", "depth"))
    rec = Recorder(cams, fps, depth)
    rec.tick(force=True)
    if seconds > 0 and not E["done"]:
        hold = S.hold_targets()  # actions are deltas: watching keeps the hand still and the fingers as they are
        for _ in range(int(round(seconds / CTRL_DT))):
            if _ctrl_step(hold, rec):
                break
        rec.tick(force=True)
    E["calls"] += 1
    RECF.flush()
    log(
        {
            "ev": "observe",
            "call": E["calls"],
            "seconds": seconds,
            "cameras": cams,
            "control_steps": len(RECORD),
            "rows_total": E["rows"],
            **_journal_pos(),
            "state_sha": S.state_sha(),
        }
    )
    out = {**meta_common(), "obs_id": E["calls"], "seconds": seconds, "ok": True}
    return out, rec.parts(video if seconds > 0 else ("frames" if video == "mp4" else video), fps)


# ---- RCB_STATE_OPS: /reset, /save_state, /load_state. The reset restores the frozen start in place (no new env, so the
# renderers stay valid): mw_tool.Session.reset()'s restore of the instance, applied to the running env.
class Refused(Exception):
    """A request refused before anything changed (409, or `code`)."""

    def __init__(self, body: dict, code: int = 409):
        super().__init__(body.get("error"))
        self.body = body
        self.code = code


def _restore(f: dict) -> None:
    """The env to the state `f` records (the frozen instance, or a mw_tool.snapshot()): model fields, env caches, the
    env's and numpy's random states, the MuJoCo integration state (as mw_tool.Session.reset())."""
    env = S.env
    for k in TOOL.MODEL_FIELDS:
        getattr(env.model, k)[:] = f["model_" + k]
    for k, v in TOOL.decode(json.loads(str(f["caches"]))).items():
        setattr(env, k, v)
    env.np_random.bit_generator.state = TOOL.decode(json.loads(str(f["rng"])))
    np.random.set_state(TOOL.decode(json.loads(str(f["global_rng"]))))
    S.mj.mj_setState(env.model, env.data, f["integration"], TOOL.STATE_SPEC)
    S.mj.mj_forward(env.model, env.data)
    S.mj.mj_setState(env.model, env.data, f["integration"], TOOL.STATE_SPEC)
    env.max_path_length = float("inf")


def sim_reset() -> None:
    """Back to the frozen start, in place: the session's own episode state as mw_tool.Session.reset() sets it."""
    _restore(S.frozen)
    S.obs = S.frozen["observation"].copy()
    S.actions = []
    S.targets = np.array([0.0, 0.0, 0.0, float(S.env.data.ctrl[0])])
    S.reward, S.info = S.env.evaluate_state(S.obs, np.zeros(N))
    S.reward = float(S.reward)


def sim_save() -> dict:
    return {
        "env": TOOL.snapshot(S.env),
        "obs": S.obs.copy(),
        "actions": [a.copy() for a in S.actions],
        "targets": np.asarray(S.targets).copy(),
        "reward": S.reward,
        "info": copy.deepcopy(S.info),
    }


def sim_load(snap: dict) -> None:
    _restore(snap["env"])
    S.obs = snap["obs"].copy()
    S.actions = [a.copy() for a in snap["actions"]]
    S.targets = snap["targets"].copy()
    S.reward, S.info = snap["reward"], copy.deepcopy(snap["info"])


def _keep(h: str, snap) -> None:
    HANDLES.pop(h, None)
    HANDLES[h] = snap
    while len(HANDLES) > MAX_HANDLES:
        HANDLES.pop(next(iter(HANDLES)))


def check_reset() -> None:
    """409 when no new episode may start: the agent's time is up, or the budget is spent."""
    if E["stopping"] or E.get("finalizing"):
        raise Refused({"error": "trial finished: your time is up", "reason": E["reason"], "resets_left": 0})
    if E["resets"] >= RESET_BUDGET:
        raise Refused({"error": "reset budget exhausted", "reason": "reset budget exhausted", "budget": RESET_BUDGET,
                       "resets_left": 0})


def check_ops(meta, path: str) -> dict:
    """A save's or load's checks, all of them, before anything changes (as check_reset's)."""
    if E["stopping"] or E.get("finalizing"):
        raise Refused({"error": "trial finished: your time is up", "reason": E["reason"]})
    if path == "/save_state":
        if E["done"]:
            raise Refused({"error": "the episode has ended: reset or load a saved state first", "reason": E["reason"]})
        return {}
    h = meta.get("handle") if isinstance(meta, dict) else None
    if not isinstance(h, str):
        raise Refused({"error": 'load_state needs {"handle": "s<k>"} from an earlier /save_state'}, 400)
    if h not in HANDLES:
        raise Refused({"error": f"unknown handle {h!r}: not saved in this trial, or dropped (at most {MAX_HANDLES} "
                                "are kept)", "handle": h})
    return {"handle": h}


def parse_reset(meta: dict) -> dict:
    """Validate a /reset or /load_state request's frames (cameras, include_depth; no physics)."""
    cams = _cameras(meta, list(CAMS))
    fps = _fps(meta)
    video = _video(meta, "frames")
    depth = bool(meta.get("include_depth", False))
    _frame_budget(cams, fps, 0, depth)
    return {"cams": cams, "fps": fps, "video": "frames" if video == "mp4" else video, "depth": depth}


def _new_episode(q: dict, ev: str, extra: dict):
    """After a reset or a load: the new episode's first answer (its frames, no time) and its checkpoint."""
    E["ep_start"] = E["journal_rows"]
    E["done"] = False
    E["reason"] = None
    E["t_end"] = None
    rec = Recorder(q["cams"], q["fps"], q["depth"])
    rec.tick(force=True)
    E["calls"] += 1
    RECF.flush()
    log({"ev": ev, "call": E["calls"], **extra, "cameras": q["cams"], "control_steps": len(RECORD),
         "rows_total": E["rows"], **_journal_pos(), "state_sha": S.state_sha()})
    write_final()
    return {**meta_common(), **({"reset_id": E["calls"]} if ev == "reset" else {}), "ok": True, **extra}, \
        rec.parts(q["video"], q["fps"])


def do_reset(q: dict):
    """Abandon the running episode (reason "reset" unless it had ended already) and start a new one from the frozen
    start. The journal keeps every episode's rows; RECORD (what is graded) restarts empty."""
    if not E["done"]:
        finish("reset")
    try:
        sim_reset()
    except Exception:  # the simulator failed to start the new episode: the trial ends on a simulator error
        E["done"] = True
        E["reason"] = "error"
        log({"ev": "end", "reason": "error", "reset_failed": True, "control_steps": len(RECORD), **_journal_pos()})
        write_final()
        raise
    want = os.environ.get("RCB_START_SHA")
    if want and S.state_sha() != want:  # no trial of this task can go on from here: infrastructure, finalized
        E["done"] = True
        E["reason"] = f"instance_mismatch: a reset brought the scene to {S.state_sha()[:16]}, the frozen instance at {want[:16]}"
        E["finalizing"] = E["finalized"] = E["video_done"] = True
        E["verdict"] = {"success": False, "live_success": False, "infra_error": 1, "instance_mismatch": 1, "gave_up": True}
        log({"ev": "end", "reason": E["reason"], "control_steps": len(RECORD), **_journal_pos()})
        write_final()
        raise Refused({"error": "the sim service gave up on this trial", "reason": E["reason"], "gave_up": True})
    E["resets"] += 1
    RECORD.clear()
    return _new_episode(q, "reset", {})


def do_save(_q: dict):
    snap = (sim_save(), [r.copy() for r in RECORD])
    E["saves"] += 1
    h = f"s{E['saves']}"
    _keep(h, snap)
    E["calls"] += 1
    RECF.flush()
    log({"ev": "save", "call": E["calls"], "handle": h, "control_steps": len(RECORD), "rows_total": E["rows"],
         **_journal_pos(), "state_sha": S.state_sha()})
    return {"handle": h, "step": len(RECORD), "calls": E["calls"], "rows": E["rows"], "control_steps": len(RECORD),
            "done": E["done"], "reason": E["reason"], **_reset_counters()}, None


def do_load(q: dict):
    """Abandon the running episode (reason "load" unless it had ended already) and start a new one at the saved state;
    RECORD (what is graded) becomes the saved episode's rows."""
    if not E["done"]:
        finish("load")
    h = q["handle"]
    snap = HANDLES[h]
    _keep(h, snap)  # a load counts as a use
    try:
        sim_load(snap[0])
    except Exception:
        E["done"] = True
        E["reason"] = "error"
        log({"ev": "end", "reason": "error", "load_failed": True, "control_steps": len(RECORD), **_journal_pos()})
        write_final()
        raise
    E["loads"] += 1
    RECORD[:] = [r.copy() for r in snap[1]]
    return _new_episode(q, "load", {"handle": h})


def _replay_ops(R: np.ndarray, ops: list) -> None:
    """A resume with saves or loads: the whole journal from the frozen start, every reset, save and load at the journal
    row its checkpoint logged (ops: the completed calls' checkpoints, in order)."""
    rows, j = [], 0
    HANDLES.clear()
    E["saves"] = E["loads"] = 0

    def apply(e):
        nonlocal rows
        if e["ev"] == "reset":
            sim_reset()
            rows = []
        elif e["ev"] == "save":
            E["saves"] += 1
            _keep(e["handle"], (sim_save(), [r.copy() for r in rows]))
        else:
            snap = HANDLES[e["handle"]]
            _keep(e["handle"], snap)
            sim_load(snap[0])
            E["loads"] += 1
            rows = [r.copy() for r in snap[1]]

    for i, row in enumerate(R):
        while j < len(ops) and int(ops[j]["journal_rows"]) <= i:
            apply(ops[j])
            j += 1
        S.step(row)
        rows.append(row.copy())
    while j < len(ops):
        apply(ops[j])
        j += 1
    RECORD[:] = rows


def episodes_log(ev: list) -> list:
    """One entry per episode of the trial, from the service's own log: the first begins at the agent's first request,
    each later one at the checkpoint of the reset or load that began it; an episode ends at its last "end" event (a
    reset lost with its crashed call did not end it: a later end replaces it); times on the agent's clock."""
    base = E["t_first"] if E["t_first"] is not None else next((float(e["ts"]) for e in ev if e.get("ev") == "start"), 0.0)
    t = lambda e: round(max(0.0, float(e["ts"]) - base), 2)  # noqa: E731
    out, cur = [], {"episode": 1, "began": "start", "t_start_s": 0.0, "row0": 0, "t_end_s": None, "control_steps": 0,
                    "ended": None}
    for e in ev:
        if "journal_rows" not in e:
            continue
        if e.get("ev") in ("reset", "load"):
            out.append(cur)
            cur = {"episode": cur["episode"] + 1, "began": e["ev"], "t_start_s": t(e), "row0": int(e["journal_rows"]),
                   "t_end_s": None, "control_steps": 0, "ended": None}
        elif e.get("ev") == "end":
            cur.update(t_end_s=t(e), ended=e.get("reason"), control_steps=int(e["journal_rows"]) - cur["row0"])
    out.append(cur)
    return [{k: v for k, v in x.items() if k != "row0"} for x in out]


def finalize() -> None:
    """The episode is over (the agent's container is stopped first: task.toml's collect hooks). The verdict is the live
    episode's: Meta-World's success check on the live state after the last executed control step, written with
    "finalized": true in one atomic step, nothing replayed. Then, and only then, the video (render_video())."""
    A = None
    with LOCK:
        try:
            A = np.asarray(RECORD, float).reshape(-1, N)
            live_sha = S.state_sha()
            live = S.check()
            np.savez(
                os.path.join(DATA, "trajectory.npz"),
                actions=A,
                tool=TOOL.__name__,
                meta=json.dumps({"source": "standard-mode episode", "reason": E["reason"]}),
            )
            # An episode ended by a simulator error is no success: Session.step raised before Meta-World's check was
            # updated, so check() still reports the step before while the state is the failed step's.
            ok = bool(live["success"]) and E["reason"] != "error"
            E["verdict"] = {
                "graded_by": "live",
                "replays": 0,
                "live_success": ok,
                "success": ok,
                "state_sha": live_sha,
                "live_state_sha": live_sha,
                "final_reward": float(live.get("reward") or 0.0),
                "n_actions": int(len(A)),
                "video": False,
                "video_source": "render",
            }
            if STATE_OPS:
                E["verdict"]["episodes_log"] = episodes_log(_events())
            E["video_pending"] = True  # announced with the verdict: whoever sees it knows a video is on its way
        except Exception as exc:  # noqa: BLE001 -- the service failed to record the verdict (not the agent's doing)
            traceback.print_exc()
            log({"ev": "finalize_error", "error": repr(exc)})
            E["verdict"] = {"finalize_error": repr(exc)[:300]}
            A = None
        E["finalized"] = True
        if A is None:
            E["video_done"] = True
        write_final()  # the verdict and "finalized": true, in one atomic write; nothing below changes them
        log({"ev": "verdict", **{k: v for k, v in E["verdict"].items() if isinstance(v, (bool, int, float, str))}})
        if A is not None:
            render_video(A)  # under the lock, as before: no request renders on the live scene meanwhile


def render_video(A: np.ndarray) -> None:
    """The episode's video, for people: one pass over the recorded trajectory in a fresh simulator, after the verdict is
    on disk. It never changes the verdict: success adds "video": true, and any failure (an exception; a crash, after
    which the restarted service finds the verdict and idles; the collect hook giving up on it) leaves only the video
    missing."""
    t = time.time()
    try:
        ver = TOOL.Session(INSTANCE, enable_render=True)
        tmp = os.path.join(DATA, "replay.tmp.mp4")  # rendered aside: a half-written replay.mp4 is never collected
        ver.render_video(A, tmp, every=max(2, len(A) // 600), fps=25)
        os.replace(tmp, os.path.join(DATA, "replay.mp4"))
        if os.path.exists(os.path.join(DATA, "replay.tmp.last.png")):
            os.replace(os.path.join(DATA, "replay.tmp.last.png"), os.path.join(DATA, "replay.last.png"))
        E["verdict"]["video"] = True
        log({"ev": "video", "video_s": round(time.time() - t, 1)})
    except Exception as exc:  # noqa: BLE001
        traceback.print_exc()
        try:
            log({"ev": "video_error", "error": repr(exc)[:300]})
        except Exception:  # noqa: BLE001
            pass
    E["video_pending"] = False
    E["video_done"] = True
    try:
        write_final()
    except Exception:  # noqa: BLE001
        traceback.print_exc()


def finish(reason: str) -> None:
    if E["done"]:
        return
    E["done"] = True
    E["reason"] = reason
    E["t_end"] = time.time()
    log({"ev": "end", "reason": reason, "control_steps": len(RECORD), **_journal_pos()})
    write_final()


def start_finalize() -> None:
    if E.get("finalizing"):
        return
    E["finalizing"] = True
    threading.Thread(target=finalize, daemon=True).start()


class Hd(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, code, obj):
        b = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _rcb1(self, meta, parts):
        if rcb1.size(meta, parts) > MAX_RESP:  # known before anything is built: the body is built once
            for p in parts:
                if p[0].startswith(("frames/", "depth/")) and isinstance(p[2], rcb1.Frames):
                    p[2].arrays.clear()
            parts = [p for p in parts if not (p[0].startswith("frames/") or p[0].startswith("depth/"))]
            meta = {**meta, "degraded": "frames/depth dropped (response too large)"}
        body = rcb1.encode(meta, parts)
        self.send_response(200)
        self.send_header("Content-Type", "application/x-rcb1")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _body(self):
        raw = self.rfile.read(self._n) if self._n else b""
        if raw[:4] == rcb1.MAGIC:
            return rcb1.decode(raw)
        b = json.loads(raw or b"{}")
        if not isinstance(b, dict):
            raise ValueError("body must be a JSON object or an RCB1 frame")
        return b, {}

    def do_GET(self):
        _touch()
        if self.path == "/spec":
            return self._json(200, SPEC)
        if self.path == "/status":
            return self._json(
                200,
                {
                    "calls": E["calls"],
                    "control_steps": len(RECORD),
                    "sim_steps": len(RECORD),
                    "rows": E["rows"],
                    "done": E["done"],
                    "reason": E["reason"],
                    "elapsed_s": round(time.time() - E["t0"], 1),
                    **_reset_counters(),
                    **({"state_queries": E["state_queries"]} if STATE_QUERY else {}),
                },
            )
        if STATE_QUERY and self.path == "/state/entities":
            return self._json(200, STATE_ENTS)
        return self._json(404, {"error": "no such endpoint"})

    def do_POST(self):
        _touch()
        if STATE_QUERY and self.path == "/state/query":
            return self._state_query()
        if self.path not in ("/submit", "/observe") + (("/reset",) if RESET_BUDGET else ()) + (
            ("/save_state", "/load_state") if CAN_SAVE else ()
        ):
            return self._json(404, {"error": "no such endpoint"})
        try:
            self._n = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError:
            self._n = -1
        if self._n < 0:  # read(-1) would read until the client hangs up, without any bound
            return self._json(400, {"error": "bad Content-Length"})
        if self._n > MAX_BODY:
            return self._json(413, {"error": f"request body over {MAX_BODY} bytes"})
        try:
            self._req = self._body()
        except Exception as exc:  # noqa: BLE001
            return self._json(400, {"error": str(exc)[:200]})
        with LOCK:
            log({"ev": "call_begin", "path": self.path, "call": E["calls"] + 1})
            try:
                return self._serve()
            finally:
                log({"ev": "call_end", "path": self.path})

    def _state_query(self):
        """RCB_STATE_QUERY: POST /state/query, under the lock between calls: reads the simulator (MjRead), takes no
        control step and leaves the clock and every counter but state_queries as they are; served only while the
        episode runs."""
        try:
            self._n = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError:
            self._n = -1
        if self._n < 0:
            return self._json(400, {"error": "bad Content-Length"})
        if self._n > MAX_BODY:
            return self._json(413, {"error": f"request body over {MAX_BODY} bytes"})
        try:
            meta, _ = self._body()
        except Exception as exc:  # noqa: BLE001
            return self._json(400, {"error": str(exc)[:200]})
        with LOCK:
            if E["done"] or E.get("stopping") or E["finalized"]:
                return self._json(409, {"error": "the episode has ended: state queries are served only while it runs",
                                        "reason": E["reason"]})
            plan, err = plan_state_query(STATE_ENTS, meta)
            if err:
                return self._json(400, err)
            try:
                state = STATE_TABLE.query(plan)
                json.dumps(state, allow_nan=False)
            except Exception as exc:  # noqa: BLE001 -- the query only reads: an adapter error ends nothing
                traceback.print_exc()
                log({"ev": "state_query_error", "error": repr(exc)[:300]})
                return self._json(500, {"error": "state query failed", "detail": repr(exc)[:200]})
            E["state_queries"] += 1
            log({"ev": "state_query", "n": E["state_queries"], "names": list(plan),
                 "fields": sorted({f for fs in plan.values() for f in fs}), "control_steps": len(RECORD)})
            return self._json(200, {"state": state, "control_steps": len(RECORD),
                                    "sim_time_s": round(len(RECORD) * CTRL_DT, 4), "state_queries": E["state_queries"]})

    def _serve(self):
        meta, parts = self._req
        if self.path == "/submit" and E["done"]:
            return self._json(409, {"error": "episode finished", "reason": E["reason"],
                                    **({"resets_left": _reset_counters()["resets_left"]} if RESET_BUDGET else {})})
        op = {}
        try:
            if self.path == "/reset":
                check_reset()
            elif self.path in ("/save_state", "/load_state"):
                op = check_ops(meta, self.path)
        except Refused as exc:
            return self._json(exc.code, exc.body)
        try:  # the whole request is checked before any physics: whatever is wrong with it is a 400 and changes nothing
            if not isinstance(meta, dict) or not isinstance(parts, dict):
                raise ValueError("the request's meta must be a JSON object")
            if self.path in ("/submit", "/observe"):
                q = parse_submit(meta, parts) if self.path == "/submit" else parse_observe(meta)
            else:
                q = op if self.path == "/save_state" else {**parse_reset(meta), **op}
        except (ValueError, TypeError, OverflowError, AttributeError, KeyError, IndexError) as exc:
            return self._json(400, {"error": (str(exc) or type(exc).__name__)[:300]})
        try:
            if self.path in ("/submit", "/observe"):
                out, blobs = do_submit(q) if self.path == "/submit" else do_observe(q)
            else:
                out, blobs = {"/reset": do_reset, "/save_state": do_save, "/load_state": do_load}[self.path](q)
        except Refused as exc:  # a reset that found the scene off the frozen start (the trial is given up)
            return self._json(exc.code, exc.body)
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            log({"ev": "error", "error": repr(exc)})
            finish("error")  # protocol v1.0 §4.6: a simulator error ends the episode
            return self._json(500, {"error": "simulator error", "detail": repr(exc)[:200]})
        if blobs is None:  # a JSON answer (/save_state)
            return self._json(200, out)
        return self._rcb1(out, blobs)


def _end_from_signal():
    with LOCK:
        if STATE_OPS:
            E["stopping"] = True  # the agent's time is up: no new episode after this one
        if not E["done"]:
            finish("timeout")
    start_finalize()


def _check_start() -> None:
    """The frozen start, checked when the episode begins (the verdict is the live episode's, so no later replay checks
    it): the instance is the frozen one (its SHA-256, RCB_INSTANCE_SHA) and the scene was restored to its recorded state
    (RCB_START_SHA, the state hash of the instance's MuJoCo integration state). Both come from the generator; a
    mismatch is no trial of this task: an infrastructure failure, before the agent is served."""
    want = os.environ.get("RCB_INSTANCE_SHA")
    got = str(getattr(S, "instance_sha", ""))
    if want and got != want:
        _give_up(f"instance_mismatch: the instance's SHA-256 is {got[:16]}, the frozen one's {want[:16]}", infra=True)
    want = os.environ.get("RCB_START_SHA")
    if want and S.state_sha() != want:
        _give_up(f"instance_mismatch: the scene started at {S.state_sha()[:16]}, the frozen instance at {want[:16]}",
                 infra=True)


def _resume_or_start() -> None:
    """Fresh start, or (after a crash: compose restarts the container) resume the SAME episode. The journal holds every
    executed control step; the log holds, per completed call, the step count and the state hash. Replaying the journal up
    to the last completed call rebuilds that exact state (MuJoCo on the CPU is bit-exact) and the hash is checked. A call
    in flight when the service died is lost as a whole; its caller got an error, not a result, so nothing is gained. No
    reset: the episode never goes back to its start. More than MAX_RESUMES crashes end it; if every crash happened while
    serving a call, it counts as the agent's (sim_crashed_on_call), else as infrastructure (sim_crashed)."""
    ev = _events()
    starts = [e for e in ev if e.get("ev") == "start"]
    if not starts:
        try:
            os.remove(REC_PATH)
        except OSError:
            pass
        _check_start()
        log({"ev": "start", "tool": TOOL.__name__, "state_sha": S.state_sha(), "protocol": PROTOCOL})
        write_final()
        E["t0"] = time.time()
        return
    try:
        fin = json.load(open(os.path.join(DATA, "final.json")))
    except Exception:  # noqa: BLE001
        fin = {}
    if _verdict_on_disk(fin):  # the verdict is on disk: nothing is replayed, redone or rewritten (at most the video
        _give_up("restarted after finalize", infra=False)  # is lost)
    resumes = [e for e in ev if e.get("ev") == "resumed"]
    in_call = bool(ev) and ev[-1].get("ev") == "call_begin"
    crashes_in_calls = sum(1 for e in resumes if e.get("crashed_in_call")) + int(in_call)
    if len(resumes) >= MAX_RESUMES:
        all_in_calls = crashes_in_calls == len(resumes) + 1
        _give_up("sim_crashed_on_call" if all_in_calls else "sim_crashed", infra=not all_in_calls)
    done = [e for e in ev if e.get("ev") in ("submit", "observe") + (("reset", "save", "load") if STATE_OPS else ())
            and "state_sha" in e]
    last = done[-1] if done else starts[0]
    n = int(last.get("journal_rows", last.get("control_steps", 0))) if STATE_OPS else int(last.get("control_steps", 0))
    try:
        R = np.fromfile(REC_PATH, dtype=np.float64)
    except OSError:
        R = np.zeros(0)
    if R.size < n * N:
        _give_up("resume_failed: journal shorter than the log", infra=True)
    R = R[: n * N].reshape(n, N)
    ep0 = int(last.get("ep_start", 0)) if STATE_OPS else 0
    if STATE_OPS and any(e.get("ev") in ("save", "load") for e in done):  # the whole journal, every op at its row
        _replay_ops(R, [e for e in done if e.get("ev") in ("reset", "save", "load")])
    else:
        for row in R[ep0:]:  # the current episode, from the frozen start
            S.step(row)
            RECORD.append(row.copy())
    if S.state_sha() != last["state_sha"]:
        _give_up("resume_failed: replayed state differs from the logged one", infra=True)
    R.tofile(REC_PATH)  # drop the steps of the call that was in flight
    E["calls"] = int(last.get("call", 0))
    E["rows"] = int(last.get("rows_total", 0))
    E["t0"] = float(starts[0]["ts"])
    if STATE_QUERY:  # each answered query is logged before it is answered
        E["state_queries"] = sum(1 for e in ev if e.get("ev") == "state_query")
    first = [e for e in ev if e.get("ev") == "first_request"]
    if first:
        E["t_first"] = float(first[0]["ts"])  # the agent's clock keeps running through a crash
    # the episode's end, if it is within the last completed call: an end past it (the step limit hit by the call that was
    # in flight) was lost with that call, as the call itself
    ended = [e for e in ev if e.get("ev") == "end" and int(e.get("control_steps") or 0) <= n]
    if STATE_OPS:  # the current episode's end: after the reset or load that began it, never a lost reset's or load's
        k0 = max((i for i, e in enumerate(ev) if e is last), default=0)
        b = max((i for i, e in enumerate(ev[: k0 + 1]) if e.get("ev") in ("reset", "load") and "state_sha" in e),
                default=-1)
        ended = [e for e in ev[b + 1:] if e.get("ev") == "end" and e.get("reason") not in ("reset", "load")
                 and int(e.get("journal_rows", e.get("control_steps")) or 0) <= n]
        E.update(resets=int(last.get("resets", 0)), saves=int(last.get("saves", E["saves"])),
                 loads=int(last.get("loads", E["loads"])), ep_start=ep0, journal_rows=n)
        E["stopping"] = any(e.get("ev") == "end" and e.get("reason") == "timeout" for e in ev)
    if ended:
        E["done"] = True
        E["reason"] = ended[-1].get("reason")
        E["t_end"] = float(ended[-1]["ts"])
    log(
        {
            "ev": "resumed",
            "control_steps": n,
            "calls": E["calls"],
            "state_sha": S.state_sha(),
            "crashed_in_call": in_call,
            "downtime_s": round(time.time() - float(ev[-1]["ts"]), 1),
        }
    )
    write_final()


def _bind(port: int, handler) -> HTTPServer:
    """The listening server. Under Harbor's egress control the agent's container shares this network namespace, so a
    process of the agent may hold the port when the service (re)starts. Wait for the port instead of failing: an
    exception here would count as a crash of the service (the collect hook's signal is still served meanwhile)."""
    said = False
    while True:
        try:
            return HTTPServer(("0.0.0.0", port), handler)
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE:
                raise
        if not said:
            said = True
            print(f"[sim] port {port} is in use (a process in the agent's container?): waiting for it to be free", flush=True)
        time.sleep(1.0)


_resume_or_start()
signal.signal(signal.SIGUSR1, lambda *_: threading.Thread(target=_end_from_signal, daemon=True).start())
if _USR1["pending"]:
    threading.Thread(target=_end_from_signal, daemon=True).start()
RECF = open(REC_PATH, "ab")
E["ready"] = True
open(os.path.join(DATA, "server.pid"), "w").write(str(os.getpid()))
print(f"{PROTOCOL} MetaWorld+ sim ready ({TOOL.__name__}: {S.inst.get('env_id')})", flush=True)
_bind(int(os.environ.get("RCB_PORT", "8000")), Hd).serve_forever()
