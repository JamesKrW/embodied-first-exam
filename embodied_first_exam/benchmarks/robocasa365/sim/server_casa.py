"""Standard-mode sim service for the RoboCasa-series tasks (eai-standard/2.1): robot as a service.

Derived from the MuJoCo service (images/mujoco/limited/server_mj.py, via images/metaworld/standard/server_mw.py) and
identical to it in protocol and lifecycle: GET /spec, GET /status, POST /observe, POST /submit, binary RCB1 responses,
no reset, crash recovery (journal + per-call checkpoints, at most RCB_MAX_RESUMES resumes), startup watchdog, SIGUSR1
from the collect hook ends the episode, /data evidence. Differences, all from robosuite / RoboCasa:
- the action type is the robot's native composite-controller command, laid out per instance by the tool module
  (casa_tool.action_spec): named after its arm channels -- `ee_delta` (mobile Panda, OSC deltas), `joint_position`
  (mobile Panda, absolute joint targets) or `ee_pose` (GR1, absolute grip-site poses solved by whole-body IK);
- watching (/observe with seconds > 0) holds the robot still (S.hold_targets(): delta channels zero, the rest kept);
- cameras are the robot's own (they move with its base), so every response carries their current calibration;
- protocol v1.0 items the MuJoCo service gets in the migration (docs/protocol/README.md §7): the eai-standard/2.1 name, a
  simulator error (500) ends the episode with reason "error", an oversized body is answered with 413.

What the agent gets: camera RGB (+ depth) with each camera's calibration, the robot's own joints, end-effector
positions and last command, and the task's instruction. Never an object's pose, the scene layout or the success check.
The episode runs until the agent's time is up; the state after the last executed control step is graded.

The verdict is the live episode's (docs/protocol/README.md H.4; owner, 2026-10-17): the environment's own success
check on the state after the last control step this service executed, not already solved at the start, and the
episode not ended by a simulator error. Nothing is replayed to grade it. The frozen initial state is checked when the
episode begins (the instance file's hash, RCB_INSTANCE_SHA, and the start state's hash, RCB_START_SHA, both from the
generator): a scene that did not build to it is refused (instance_mismatch, an infrastructure failure) before the agent
is served.

Evidence under /data: episode.jsonl (every call), calls/NNNNNN/ (meta + actions), and at the end trajectory.npz
((T, dim) per control step) and final.json with the verdict and "finalized": true, written together in one atomic step.
Only then replay.mp4 and replay.last.png are rendered, from trajectory.npz (one render pass, video_source "render"):
optional, they can add "video": true to final.json and nothing else, and no failure there (an exception, a crash, the
collect hook running out of time) changes the verdict. /data/video.pending exists (and final.json says
"video_pending": true) while that rendering runs.

A request is validated as a whole before any physics: whatever is wrong with it is a 400 and nothing ran (PROTOCOL
§4.6). Only an error while the simulator executes is a 500, which ends the episode.

Ablations (off unless the task sets them; nothing changes without them): RCB_STATE_OPS (reset / save / load, below) and
RCB_STATE_QUERY: GET /state/entities and POST /state/query (STATE_QUERY_NOTE) answer from the state table: the robot
(its base pose, velocity and height above the floor, its joints), its end-effector sites, base / torso / head links and
fingertips, every task object (env.objects: pose, velocity, extents), every fixture with hinge / slide joints (doors,
drawers, a microwave, a sink's faucet, stove knobs: an articulation with its joints and their ranges) and the fixtures
the task refers to (ep_meta fixture_refs) as scene features, all in the world frame (that of the camera calibration and
proprio's eef positions), quaternions w x y z. A query copies the live joint state into a private MjData and runs
MuJoCo's kinematics there; it takes no control step. final.json and /status add state_queries.
"""

from __future__ import annotations

import copy
import errno
import faulthandler
import importlib
import json
import math
import os
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
DATA = os.environ.get("RCB_DATA", "/data")  # the evidence directory (the variable is for tests; tasks use /data)
os.makedirs(os.path.join(DATA, "calls"), exist_ok=True)
TOOL = importlib.import_module(os.environ.get("RCB_TOOL", "casa_tool"))
INSTANCE = os.environ.get("RCB_INSTANCE", os.path.join(os.path.dirname(__file__), "instance.npz"))
MAX_CHUNK_S = float(os.environ.get("RCB_MAX_CHUNK_S", "10"))
MAX_WATCH_S = float(os.environ.get("RCB_MAX_WATCH_S", "30"))
MAX_CONTROL_STEPS = int(os.environ.get("RCB_MAX_CONTROL_STEPS", "0"))  # technical episode cap (0 = none)
MAX_VIDEO_FPS = 30
MAX_FRAMES_PER_CALL = 3000  # frames x cameras per response: bounds the service's memory
WALL_CLOCK_S = float(os.environ.get("RCB_WALL_CLOCK_S", "3600"))
MAX_BODY = 64 * 1024 * 1024
MAX_RESP = 256 * 1024 * 1024
# The unlimited-reset ablation (off unless the task sets RCB_STATE_OPS=1; nothing changes when off): POST /reset
# (RCB_RESET_BUDGET, read only then) and, beside it, POST /save_state and /load_state (snapshot: casa_tool.snapshot of
# the env, the session's own per-episode state and the graded rows, in memory); every reset and load begins a new
# episode, only the last one is graded; meta and /status add episode, resets_left, control_steps_total, saves and loads;
# final.json adds resets, saves, loads, episodes, episodes_log, first_success and, for a success, resets_at_success and
# loads_at_success; a crash resume replays every reset, save and load at its journal row.
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
LOCK = threading.Lock()
E = {"ready": False, "calls": 0, "rows": 0, "done": False, "reason": None, "t0": time.time(), "finalized": False}
E["t_first"] = None  # the agent's first request (its clock starts there: _touch, _agent_time)
if STATE_QUERY:
    E["state_queries"] = 0  # /state/query calls answered (each is logged before it is answered; a resume counts them)
if STATE_OPS:
    # resets: /reset calls that started a new episode; journal_rows: control steps executed over the whole trial
    # (record.f64's length); ep_start: where the current episode begins in it; stopping: the agent's time is up (no
    # reset any more); saves / loads: /save_state and /load_state calls that took effect; start_sha: the frozen start's
    # state hash, which every reset must bring the scene back to
    E.update(resets=0, ep_start=0, journal_rows=0, stopping=False, saves=0, loads=0, start_sha=None)
HANDLES: dict = {}  # RCB_STATE_OPS: handle -> (simulator snapshot, the graded rows), LRU order

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
FINAL = os.path.join(DATA, "final.json")
VIDEO_PENDING = os.path.join(DATA, "video.pending")  # exists while the optional video is being rendered
VIDEO_TMP = os.path.join(DATA, "replay.tmp.mp4")  # the video under construction; renamed to replay.mp4 when complete


def _events() -> list:
    try:
        return [json.loads(line) for line in open(os.path.join(DATA, "episode.jsonl")) if line.strip()]
    except OSError:
        return []


def _dump_final(d: dict) -> None:
    """Replace final.json atomically: a reader (the collect hook, the verifier) never sees a half-written file."""
    tmp = FINAL + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, FINAL)


def _load_final() -> dict:
    try:
        d = json.load(open(FINAL))
    except Exception:  # noqa: BLE001
        return {}
    return d if isinstance(d, dict) else {}


def _graded(d: dict) -> bool:
    """final.json holds a verdict: "graded_by" since 2026-10-17 (the live episode's), "deterministic" before (a replay's),
    or "finalized" (a verdict, or a give-up, that is final)."""
    return bool(d.get("finalized")) or "graded_by" in d or "deterministic" in d


def _remove(*paths: str) -> None:
    for p in paths:
        try:
            os.remove(p)
        except OSError:
            pass


def _give_up(reason: str, infra: bool) -> None:
    """End the trial's episode without a new one: mark it failed, keep the PID file for the collect hook, and idle.
    A verdict already on disk is never overwritten (PROTOCOL §4.7), whatever else the file says: at most its missing
    "finalized" flag is set, or a "video_pending" left by a process that died rendering the video is cleared (that
    video is lost), which changes nothing the verdict says."""
    log({"ev": "restart_refused", "reason": reason})
    prev = _load_final()
    if _graded(prev):
        if not prev.get("finalized") or prev.get("video_pending"):
            prev["finalized"] = True
            prev["video_pending"] = False
            _dump_final(prev)
    else:
        prev.update({"success": False, "done": True, "reason": reason, "finalized": True, "protocol": PROTOCOL})
        if infra:
            prev["infra_error"] = 1
        _dump_final(prev)
    open(os.path.join(DATA, "server.pid"), "w").write(str(os.getpid()))
    signal.signal(signal.SIGUSR1, lambda *_: None)
    print(f"restart refused: {reason}", flush=True)
    while True:
        time.sleep(3600)


with open(os.path.join(DATA, "startups.jsonl"), "a") as _f:
    _f.write(json.dumps({"ts": time.time(), "pid": os.getpid()}) + "\n")
# A video that was being rendered when the process died is not resumed: clear its marker and its partial file at once
# (before the scene is rebuilt), so the collect hook stops waiting for it. The verdict on disk is not touched.
_remove(VIDEO_PENDING, VIDEO_TMP, VIDEO_TMP[:-4] + ".last.png")
# Back off before rebuilding after a failed start: compose restarts immediately.
_n_starts = sum(1 for _ in open(os.path.join(DATA, "startups.jsonl")))
if _n_starts > 1 and not os.path.exists(os.path.join(DATA, "episode.jsonl")):
    time.sleep(min(120, 20 * (_n_starts - 1)))

S = TOOL.Session(INSTANCE, enable_render=True)
E["built"] = True
faulthandler.cancel_dump_traceback_later()  # the scene exists: the startup watchdog stands down
CTRL_DT = float(S.control_dt)
N = int(S.action_dim)
NAMES = list(S.action_names)
ACTION_NAME = {12: "ee_delta", 13: "joint_position", 24: "ee_pose"}[N]
ACTION = {
    "dtype": "float64",
    "units": [c["unit"] for c in S.action_spec["channels"]],
    "kinds": [c["kind"] for c in S.action_spec["channels"]],
    "parts": S.action_spec["parts"],
    "frame": S.action_spec["frame"],
    "semantics": (
        "the robot's native composite-controller command (robosuite), applied once per control step; "
        "kinds: delta = motion per control step (0 holds still), absolute = target, grip / mode = held setting"
    ),
    **{k: S.action_spec[k] for k in ("grip", "hybrid_mode") if k in S.action_spec},
}
PROPRIO_LAYOUT = list(S.proprio_layout)
RECORD: list = []  # the episode: one action row per control step


def _cams() -> dict:
    """name -> (mujoco camera id or None, free-camera settings or None), from the session's camera_views."""
    out = {}
    for name, view in S.camera_views.items():
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
    """Visual geoms only, as robosuite's own cameras (casa_tool.scene_option): no collision geoms in RGB or depth."""
    cid, free = CAMS[cam]
    if cid is not None:
        r.update_scene(S.data, camera=cid, scene_option=TOOL.scene_option())
    else:
        c = S.mj.MjvCamera()
        c.lookat[:] = free["lookat"]
        c.distance = free["distance"]
        c.azimuth = free["azimuth"]
        c.elevation = free["elevation"]
        r.update_scene(S.data, camera=c, scene_option=TOOL.scene_option())


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


def cam2world(cam: str) -> np.ndarray:
    """Camera-to-world pose (4x4, OpenGL convention: x right, y up, looks along -z) of the viewpoint the mono RGB / depth
    image is rendered from, in the current state (as server_mj.py). A model camera (every RoboCasa camera): data.cam_xpos
    / data.cam_xmat, which is what the renderer reads. A free camera: the scene's two eye cameras share their axes and sit
    ipd/2 to either side of the viewpoint, so the viewpoint is their mean. scene.camera[0] alone is the LEFT eye, ipd/2 =
    3.4 cm to the left of the viewpoint: until 2026-10-13 that was reported for every camera."""
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
    """Per camera: pinhole intrinsics and the camera-to-world pose in the current state (see cam2world). The RoboCasa
    cameras are given by their vertical field of view (no sensor size), so pixels are square: fx = fy = (H / 2) /
    tan(fovy / 2), principal point at the image centre."""
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
        }
    return out


def proprio() -> np.ndarray:
    return np.asarray(S.proprio(), np.float32)


def task_info() -> dict:
    return {"instruction": S.inst.get("text", "")}


SPEC = {
    "protocol": PROTOCOL,
    "simulator": "robosuite",
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
    "cameras": {c: {"rgb": True, "depth": True, "size": [W, H]} for c in CAMS},
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
        "not_yet": [k for k in ("ee_delta", "joint_position", "ee_pose") if k != ACTION_NAME],
    },
    "proprio_layout": PROPRIO_LAYOUT,
    "frames": "world frame: metres, z up; robot-mounted cameras move with the robot (calibration in every response)",
    "budget": {
        "reset": RESET_BUDGET,
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
if RESET_BUDGET:
    SPEC["reset"] = ("POST /reset abandons the running episode and starts a new one from the same starting state, "
                     "budget.reset times in all; only the last episode is graded (its state after the last executed "
                     "control step). The clock (time_left_s) keeps running. meta and /status add episode, resets_left "
                     "and control_steps_total")
    SPEC["episode"] += "; with resets, only the last episode is scored"


# RCB_STATE_QUERY: the state table. Robot = every body named with ROBOT_PREFIXES (casa_tool); objects = env.objects (by
# their root bodies, as the instance's object_cfgs and the privileged state() name them); fixtures = env.fixtures: one
# with hinge / slide joints (a door, drawer, microwave, a sink's faucet handle and spout, a stove knob) is an
# articulation, one the task refers to (ep_meta fixture_refs) without joints a scene feature. A query copies the live
# qpos / qvel / mocap poses into a private MjData and runs MuJoCo's kinematics there; the live data is never touched.

class CasaStateTable:
    def __init__(self, frame: str, note: str | None = None):
        self.table = StateTable(frame=frame, quaternion_order="wxyz", note=note)
        self.add, self.model, self.data = self.table.add, None, None

    def entities(self) -> dict:
        return self.table.entities()

    def query(self, plan: dict) -> dict:
        m, live = S.model, S.data
        if self.model is not m:                      # a reset rebuilds the env from the same XML: the same ids
            self.model, self.data = m, S.mj.MjData(m)
        d = self.data
        d.qpos[:], d.qvel[:] = live.qpos, live.qvel
        d.mocap_pos[:], d.mocap_quat[:] = live.mocap_pos, live.mocap_quat
        S.mj.mj_kinematics(m, d)
        S.mj.mj_comPos(m, d)
        S.mj.mj_comVel(m, d)
        return self.table.query(plan)

    def vel(self, b: int) -> np.ndarray:
        out = np.zeros(6)
        S.mj.mj_objectVelocity(self.model, self.data, S.mj.mjtObj.mjOBJ_BODY, b, out, 0)
        return out                                   # [angular, linear], world axes, at the body's origin

    def body(self, b: int, velocity: bool = True) -> dict:
        f = {"pos": ("m", lambda: self.data.xpos[b].copy()), "quat": ("wxyz", lambda: self.data.xquat[b].copy())}
        if velocity:
            f["lin_vel"] = ("m/s", lambda: self.vel(b)[3:])
            f["ang_vel"] = ("rad/s", lambda: self.vel(b)[:3])
        return f

    def site(self, i: int) -> dict:
        def quat():
            q = np.zeros(4)
            S.mj.mju_mat2Quat(q, self.data.site_xmat[i])
            return q
        return {"pos": ("m", lambda: self.data.site_xpos[i].copy()), "quat": ("wxyz", quat)}

    def joints(self, ids: list) -> dict:
        m = S.model
        names = [m.joint(j).name for j in ids]
        qa = [int(m.jnt_qposadr[j]) for j in ids]
        va = [int(m.jnt_dofadr[j]) for j in ids]
        kinds = {int(m.jnt_type[j]) for j in ids}
        unit = "rad or m (hinge / slide)" if len(kinds) > 1 else ("m" if kinds == {int(S.mj.mjtJoint.mjJNT_SLIDE)} else "rad")
        rng = {n: m.jnt_range[j].tolist() for n, j in zip(names, ids) if m.jnt_limited[j]}
        out = {"joint_pos": (unit, lambda: dict(zip(names, self.data.qpos[qa].tolist()))),
               "joint_vel": (("rad/s or m/s (hinge / slide)" if "or" in unit else unit + "/s"), lambda: dict(zip(names, self.data.qvel[va].tolist())))}
        if rng:
            out["joint_range"] = (unit, lambda: rng)
        return out


def _subtree_extent(root: int) -> list | None:
    """Full extents (m) of the collision geoms of a body and the bodies welded to it, along the body's axes, at the state
    the scene was built in (MuJoCo's geom bounding boxes)."""
    m, d = S.model, S.data
    welded = {b for b in range(m.nbody) if int(m.body_weldid[b]) == int(m.body_weldid[root]) and
              (b == root or _ancestor(root, b))}
    Rr = d.xmat[root].reshape(3, 3)
    pts = []
    for g in range(m.ngeom):
        if int(m.geom_bodyid[g]) not in welded or not (m.geom_contype[g] or m.geom_conaffinity[g]) \
                or int(m.geom_type[g]) == int(S.mj.mjtGeom.mjGEOM_PLANE):
            continue
        Rg = d.geom_xmat[g].reshape(3, 3)
        c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
        for sg in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3):
            pts.append(Rr.T @ (d.geom_xpos[g] + Rg @ (c + sg * h) - d.xpos[root]))
    if not pts:
        return None
    pts = np.asarray(pts)
    return (pts.max(axis=0) - pts.min(axis=0)).tolist()


def _ancestor(a: int, b: int) -> bool:
    while b > 0:
        b = int(S.model.body_parentid[b])
        if b == a:
            return True
    return False


def state_table() -> CasaStateTable:
    m, env = S.model, S.env
    hinge_slide = (int(S.mj.mjtJoint.mjJNT_HINGE), int(S.mj.mjtJoint.mjJNT_SLIDE))
    # the floor: the top of the room's floor (the widest colliding box named "floor*", no robot's), else a plane of the
    # world body
    box = int(S.mj.mjtGeom.mjGEOM_BOX)
    boxes = [g for g in range(m.ngeom) if int(m.geom_type[g]) == box and m.geom(g).name.lower().startswith("floor")
             and (m.geom_contype[g] or m.geom_conaffinity[g])]
    planes = [float(m.geom_pos[g][2]) for g in range(m.ngeom)
              if int(m.geom_type[g]) == int(S.mj.mjtGeom.mjGEOM_PLANE) and int(m.geom_bodyid[g]) == 0]
    if boxes:
        g = max(boxes, key=lambda g: float(m.geom_size[g][0] * m.geom_size[g][1]))
        floor = float(S.data.geom_xpos[g][2]) + float(m.geom_size[g][2])
    else:
        floor = max(planes) if planes else 0.0
    t = CasaStateTable("world (metres, z up; the frame of camera_calibration's cam2world_gl and of proprio's eef "
                       f"positions): the kitchen's frame, the floor at z = {floor:g}; quaternions w x y z, velocities in "
                       "world axes")
    t.model, t.data = m, S.mj.MjData(m)
    names = [m.body(b).name for b in range(m.nbody)]
    robot = [b for b in range(1, m.nbody) if names[b].startswith(TOOL.ROBOT_PREFIXES)]
    rset = set(robot)
    roots = [b for b in robot if int(m.body_parentid[b]) not in rset]
    # the robot's pose: the mobile base's body (its base joints move it) when it has one, else its root body
    base = next((b for b in robot if names[b] == "mobilebase0_base"), roots[0])
    rj = [m.joint(n).id for n in S._robot_joints()]
    t.add("robot", "robot", {**t.body(base), "height_above_ground": ("m", lambda: float(t.data.xpos[base][2]) - floor),
                             **t.joints(rj)},
          note=f"{type(env.robots[0].robot_model).__name__}: pos / quat = body {names[base]!r}; joint_pos / joint_vel = "
               "its joints (proprio *_pos / *_vel)")
    for arm, sid in sorted(env.robots[0].eef_site_id.items()):
        t.add(f"eef_{arm}", "robot_link", t.site(int(sid)), note=f"the {arm} end-effector site (proprio eef_{arm}_*)")
    links = [b for b in robot if names[b].endswith(("_base", "torso", "torso_link", "head", "head_link", "pelvis"))
             and b != base]
    jointed = {b for b in robot if int(m.body_jntnum[b]) > 0}
    inner = set()                      # jointed bodies with a jointed descendant
    for b in jointed:
        p = int(m.body_parentid[b])
        while p in rset:
            inner.add(p)
            p = int(m.body_parentid[p])
    tips = [b for b in sorted(jointed - inner) if not names[b].startswith("mobilebase0_")]
    for b in dict.fromkeys(links + tips):
        t.add(names[b], "robot_link", t.body(b), note="a finger (the last jointed link of its chain)" if b in tips else None)
    for n, obj in env.objects.items():
        b = m.body(obj.root_body).id
        size = _subtree_extent(b)
        t.add(n, "object", {**t.body(b), **({"size": ("m", lambda size=size: size)} if size else {})},
              note="its root body" + ("; size along its own axes" if size else ""))
    refs = {}
    for ref, fx in ((S.meta.get("fixture_refs") or {}).items()):
        refs.setdefault(str(fx), []).append(str(ref))
    for n, fx in env.fixtures.items():
        prefix = getattr(fx, "naming_prefix", n + "_")
        try:
            b = m.body(fx.root_body).id
        except Exception:  # noqa: BLE001 -- a fixture without its own body (merged into the room)
            continue
        jids = [j for j in range(m.njnt) if m.joint(j).name.startswith(prefix) and int(m.jnt_type[j]) in hinge_slide]
        note = f"the task refers to it as {', '.join(refs[n])}" if n in refs else None
        if jids:
            t.add(n, "articulation", {**t.body(b), **t.joints(jids)}, note=note)
        elif n in refs:
            size = _subtree_extent(b)
            t.add(n, "scene_feature", {**t.body(b, velocity=False), **({"size": ("m", lambda size=size: size)} if size else {})},
                  note=note)
    return t


STATE_TABLE = state_table() if STATE_QUERY else None
STATE_ENTS = check_state_table(STATE_TABLE) if STATE_QUERY else None
if STATE_QUERY:
    SPEC["state_query"] = STATE_QUERY_NOTE
if CAN_SAVE:
    SPEC["state_ops"] = (
        "POST /save_state ({}) stores the current state of the running episode in this service "
        "and answers JSON {\"handle\": \"s<k>\", \"step\": control steps of the episode, ...}: the "
        "handle is all you get, never the state's contents. POST /load_state {\"handle\": h, "
        "cameras, video} restores a state saved earlier in this trial and starts a new episode "
        "there (answered as /reset is); nothing else can be loaded. The task's success rule is "
        f"checked again from your next command on. At most {MAX_HANDLES} handles are kept: saving "
        "more drops the least recently used one (loading it is a 409). Saves and loads do not "
        "count against budget.reset; a success ends the trial (409 after it)"
    )


def _reset_counters() -> dict:
    if not RESET_BUDGET:
        return {}
    return {
        "episode": E["resets"] + E["loads"] + 1,
        "control_steps_total": E["journal_rows"],
        "resets_left": 0 if E["stopping"] else RESET_BUDGET - E["resets"],
        **({"saves": E["saves"], "loads": E["loads"]} if CAN_SAVE else {}),
    }


def _journal_pos() -> dict:
    """RCB_STATE_OPS: where a checkpoint (a completed call's log line) sits in the journal: what a resume replays and how
    many resets, saves and loads came before it."""
    return {"journal_rows": E["journal_rows"], "ep_start": E["ep_start"], "resets": E["resets"], "saves": E["saves"],
            "loads": E["loads"]}


def _agent_time() -> float:
    """Seconds since the agent's first request to the service (0 before it): the agent's clock. Harbor counts the
    agent's cutoff around the agent's run only, and the sidecar starts with the environment, before the harness is
    installed: counted from this process's first start (until 2026-10-13) the clock ran ahead of the agent's by that
    set-up time (a median of 79 s over the recorded trials) and time_left_s reached 0 that much too early."""
    return 0.0 if E["t_first"] is None else time.time() - E["t_first"]


def _touch() -> None:
    """Every request passes here: the first one starts the agent's clock (logged, so a resume keeps it). A probe that
    must not start it connects to the port without sending a request."""
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
        "cameras": calib(),  # robot-mounted cameras move: their calibration is current
        "done": E["done"],
        "reason": E["reason"],
        **_reset_counters(),
    }
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
    }
    d.update(E.get("verdict") or {})
    d.update(extra or {})
    if STATE_QUERY:
        d["state_queries"] = E["state_queries"]
    if STATE_OPS:
        d.update(reset_budget=RESET_BUDGET, resets=E["resets"], saves=E["saves"], loads=E["loads"],
                 episodes=E["resets"] + E["loads"] + 1, control_steps_total=E["journal_rows"])
        if "success" in d:  # the first attempt: graded only when no reset or load came after it
            d["first_success"] = bool(d["success"]) and E["resets"] + E["loads"] == 0
        if d.get("success"):  # the graded episode is the last one
            d.update(resets_at_success=E["resets"], loads_at_success=E["loads"])
    _dump_final(d)


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


def _number(meta: dict, key: str, default: float) -> float:
    """A finite number from the request; missing or null means the default."""
    v = meta.get(key)
    if v is None:
        return float(default)
    try:
        if isinstance(v, bool):  # JSON true is no number (float(True) is 1.0: `dt: true` ran as one second per row)
            raise TypeError
        x = float(v)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"{key} must be a number") from None
    if not math.isfinite(x):
        raise ValueError(f"{key} must be finite")
    return x


def _fps(meta: dict) -> int:
    fps = int(_number(meta, "video_fps", 10))
    if not 1 <= fps <= MAX_VIDEO_FPS:
        raise ValueError(f"video_fps must be in [1, {MAX_VIDEO_FPS}]")
    return fps


def _cameras(meta: dict, default: list) -> list:
    """The requested cameras: a list of names (or one name); missing, null or empty means the default. Names the robot
    does not have are left out; if none remains, the default is used. A repeated name is dropped BEFORE the frame
    budget and any rendering: the recorder keeps frames per name, so repeats would multiply its work."""
    v = meta.get("cameras")
    if v is None:
        return list(default)
    if isinstance(v, str):
        v = [v]
    if not isinstance(v, (list, tuple)) or not all(isinstance(c, str) for c in v):
        raise ValueError("cameras must be a list of camera names (see /spec cameras)")
    return list(dict.fromkeys(c for c in v if c in CAMS)) or list(default)


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
    """Validate a /submit as a whole, before any physics. Raises ValueError for what is wrong with it (-> 400)."""
    typ = meta.get("type")
    if typ is not None and typ != ACTION_NAME:
        raise ValueError(f"type must be {ACTION_NAME} (see /spec action_types)")
    try:
        A = np.asarray(parts["actions"]["data"] if "actions" in parts else meta.get("actions", []), np.float64)
    except (TypeError, ValueError, OverflowError, KeyError):
        raise ValueError(f"{ACTION_NAME} actions must be a (T, {N}) array of numbers") from None
    if A.ndim != 2 or A.shape[1] != N:
        raise ValueError(f"{ACTION_NAME} actions must be (T, {N}), got {list(A.shape)}")
    if not np.all(np.isfinite(A)):
        raise ValueError("actions contain NaN/inf")
    dt = _number(meta, "dt", CTRL_DT)
    if not CTRL_DT - 1e-6 <= dt <= MAX_CHUNK_S + 1e-9:  # bounded before it is divided: no overflow below
        raise ValueError(f"dt must be a positive multiple of control_dt={CTRL_DT}, at most max_chunk_s={MAX_CHUNK_S}")
    k = int(round(dt / CTRL_DT))
    if k < 1 or abs(k * CTRL_DT - dt) > 1e-6:
        raise ValueError(f"dt must be a positive multiple of control_dt={CTRL_DT}")
    if A.shape[0] * dt > MAX_CHUNK_S + 1e-9:
        raise ValueError(f"chunk longer than max_chunk_s={MAX_CHUNK_S}")
    interp = meta.get("interpolate")
    interp = "linear" if interp is None else interp
    if not isinstance(interp, str) or interp not in ("hold", "linear"):
        raise ValueError("interpolate must be hold|linear")
    cams = _cameras(meta, list(CAMS)[:1])
    fps = _fps(meta)
    video = meta.get("video")
    video = "mp4" if video is None else video
    depth = bool(meta.get("include_depth", False))
    note = meta.get("note")
    note = "" if note is None else note
    if not isinstance(note, str):
        raise ValueError("note must be a string")
    _frame_budget(cams, fps, A.shape[0] * k, depth)
    return {
        "A": A.astype(np.float64),
        "dt": dt,
        "k": k,
        "interp": interp,
        "cams": cams,
        "fps": fps,
        "video": video,
        "depth": depth,
        "note": note[:500],
    }


def parse_observe(meta: dict) -> dict:
    """Validate an /observe as a whole, before any physics. Raises ValueError for what is wrong with it (-> 400)."""
    cams = _cameras(meta, list(CAMS))
    seconds = _number(meta, "seconds", 0.0)
    if not 0 <= seconds <= MAX_WATCH_S:
        raise ValueError(f"seconds must be in [0, {MAX_WATCH_S}]")
    fps = _fps(meta)
    video = meta.get("video")
    video = "frames" if video is None else video
    depth = bool(meta.get("include_depth", False))
    _frame_budget(cams, fps, int(round(seconds / CTRL_DT)), depth)
    return {"cams": cams, "seconds": seconds, "fps": fps, "video": video, "depth": depth}


def run_submit(q: dict):
    """Execute a validated chunk (parse_submit's result)."""
    A, dt, k, interp, cams, fps = q["A"], q["dt"], q["k"], q["interp"], q["cams"], q["fps"]
    rec = Recorder(cams, fps, q["depth"])
    rec.tick(force=True)
    prev = np.asarray(S.targets, float).copy()
    n_clip = 0
    done = False
    executed = 0  # rows begun: fewer than the chunk's when the step limit ends the episode inside it
    for row in A:
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
            "state_sha": S.state_sha(),
            **(_journal_pos() if STATE_OPS else {}),
        }
    )
    d = os.path.join(DATA, "calls", f"{E['calls']:06d}")
    os.makedirs(d, exist_ok=True)
    np.save(os.path.join(d, "actions.npy"), A)
    return out, rec.parts(q["video"], fps)


def run_observe(q: dict):
    """Execute a validated observation (parse_observe's result)."""
    cams, seconds, fps, video = q["cams"], q["seconds"], q["fps"], q["video"]
    rec = Recorder(cams, fps, q["depth"])
    rec.tick(force=True)
    if seconds > 0 and not E["done"]:
        hold = S.hold_targets()  # delta channels zero, targets / grip / mode kept: the robot holds still
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
            "state_sha": S.state_sha(),
            **(_journal_pos() if STATE_OPS else {}),
        }
    )
    out = {**meta_common(), "obs_id": E["calls"], "seconds": seconds, "ok": True}
    return out, rec.parts(video if seconds > 0 else ("frames" if video == "mp4" else video), fps)


# ---- RCB_STATE_OPS: /reset, /save_state and /load_state (as server_mj.py's)
def parse_reset(meta: dict) -> dict:
    """Validate a /reset or /load_state request's frames (cameras, video, include_depth; no physics)."""
    cams = _cameras(meta, list(CAMS))
    fps = _fps(meta)
    video = meta.get("video")
    video = "frames" if video is None else video
    if not isinstance(video, str) or video not in ("mp4", "frames", "both", "none"):
        raise ValueError("video must be mp4|frames|both|none")
    depth = bool(meta.get("include_depth", False))
    _frame_budget(cams, fps, 0, depth)
    return {"cams": cams, "fps": fps, "video": "frames" if video == "mp4" else video, "depth": depth}


class Refused(Exception):
    """A request refused before anything changed (409, or `code`)."""

    def __init__(self, body: dict, code: int = 409):
        super().__init__(body.get("error"))
        self.body = body
        self.code = code


def check_reset() -> None:
    """409 when no new episode may start: the agent's time is up, or the budget is spent."""
    if E["stopping"] or E.get("finalizing"):
        raise Refused({"error": "trial finished: your time is up", "reason": E["reason"], "resets_left": 0})
    if E["resets"] >= RESET_BUDGET:
        raise Refused({"error": "reset budget exhausted", "reason": "reset budget exhausted", "budget": RESET_BUDGET,
                       "resets_left": 0})


def check_ops(meta: dict, path: str) -> dict:
    """A save's or load's checks, all of them, before anything changes (as check_reset's)."""
    if E["stopping"] or E.get("finalizing"):
        raise Refused({"error": "trial finished: your time is up", "reason": E["reason"]})
    if path == "/save_state":
        if E["done"]:
            raise Refused({"error": "the episode has ended: reset or load a saved state first", "reason": E["reason"]})
        return {}
    h = meta.get("handle")
    if not isinstance(h, str):
        raise Refused({"error": "load_state needs {\"handle\": \"s<k>\"} from an earlier /save_state"}, 400)
    if h not in HANDLES:
        raise Refused({"error": f"unknown handle {h!r}: not saved in this trial, or dropped (at most {MAX_HANDLES} "
                                "are kept)", "handle": h})
    return {"handle": h}


_OWN = ("actions", "success", "initial_success", "reward", "targets")  # the session's per-episode state


def _new_renderers() -> None:
    """The scene was rebuilt (new model and data): the cached renderers belong to the old one."""
    for k, r in _R.items():
        if r is not None:
            try:
                r.close()
            except Exception:  # noqa: BLE001
                pass
        _R[k] = None


def reset_sim() -> None:
    """Back to the frozen start as Session.reset() does, but in place: casa_tool.restore(env, frozen) on the env that
    exists (model fields, the task's Python state, the RNGs, MuJoCo's integration state), then the reset's one success
    check. A scene the in-place restore does not bring back to the frozen start's state hash is rebuilt
    (Session.reset(): the env made afresh from the instance, seconds)."""
    TOOL.restore(S.env, S.frozen)
    S.actions = []
    S.success = bool(S.env._check_success())
    S.initial_success = S.success
    S.reward = 0.0
    S.targets = S.initial_command()
    if E["start_sha"] and S.state_sha() != E["start_sha"]:
        log({"ev": "reset_rebuild", "in_place_sha": S.state_sha()})
        S.reset()
        _new_renderers()


def sim_save() -> dict:
    """The env's whole state as casa_tool.snapshot takes it (MuJoCo's integration state, the model fields a kitchen's
    reset rewrites, every JSON-encodable attribute of the env, its fixtures, robots, controllers, grippers, buffers and
    observables, the attribute names, the RNGs) and the session's own per-episode state."""
    return {"env": TOOL.snapshot(S.env), "own": {k: copy.deepcopy(getattr(S, k)) for k in _OWN}, "sha": S.state_sha()}


def sim_load(snap: dict) -> None:
    TOOL.restore(S.env, snap["env"])
    for k, v in snap["own"].items():
        setattr(S, k, copy.deepcopy(v))


def _keep(h: str, snap) -> None:
    HANDLES.pop(h, None)
    HANDLES[h] = snap
    while len(HANDLES) > MAX_HANDLES:
        HANDLES.pop(next(iter(HANDLES)))


def _give_up_live(reason: str) -> None:
    """A reset that did not bring the scene back to the frozen start: no trial of this task can go on from here. The
    service gives up at once (instance_mismatch, infrastructure), finalized, and refuses every call after this one."""
    E.update(done=True, reason=reason, gave_up=True, finalized=True, t_end=time.time())
    E["verdict"] = {"success": False, "live_success": False, "infra_error": 1, "instance_mismatch": 1, "gave_up": True,
                    "video_pending": False}
    log({"ev": "restart_refused", "reason": reason, **_journal_pos()})
    write_final()
    raise Refused({"error": "the sim service gave up on this trial", "reason": reason, "gave_up": True})


def do_reset(q: dict):
    """Abandon the running episode (it ends with reason "reset" unless it had ended already) and start a new one from
    the frozen start state. The journal keeps every episode's rows; RECORD (what is graded) restarts empty."""
    if not E["done"]:
        finish("reset")
    try:
        reset_sim()
    except Exception:  # the simulator failed to start the new episode: the trial ends on a simulator error
        E["done"] = True
        E["reason"] = "error"
        log({"ev": "end", "reason": "error", "reset_failed": True, "control_steps": len(RECORD), **_journal_pos()})
        write_final()
        raise
    E["resets"] += 1
    E["ep_start"] = E["journal_rows"]
    RECORD.clear()
    E["done"] = False
    E["reason"] = None
    E["t_end"] = None
    sha = S.state_sha()
    if E["start_sha"] and sha != E["start_sha"]:
        _give_up_live(f"instance_mismatch: a reset brought the scene to {sha[:16]}, the frozen start is {E['start_sha'][:16]}")
    rec = Recorder(q["cams"], q["fps"], q["depth"])
    rec.tick(force=True)
    E["calls"] += 1
    RECF.flush()
    log({"ev": "reset", "call": E["calls"], "cameras": q["cams"], "control_steps": 0, "rows_total": E["rows"],
         **_journal_pos(), "state_sha": sha})
    write_final()
    return {**meta_common(), "reset_id": E["calls"], "ok": True}, rec.parts(q["video"], q["fps"])


def do_save(_q: dict):
    snap = (sim_save(), [r.copy() for r in RECORD])
    E["saves"] += 1
    h = f"s{E['saves']}"
    _keep(h, snap)
    E["calls"] += 1
    RECF.flush()
    log({"ev": "save", "call": E["calls"], "handle": h, "control_steps": len(RECORD), "rows_total": E["rows"],
         **_journal_pos(), "state_sha": snap[0]["sha"]})
    return {"handle": h, "step": len(RECORD), "calls": E["calls"], "rows": E["rows"], "control_steps": len(RECORD),
            "done": E["done"], "reason": E["reason"], **_reset_counters()}, None


def do_load(q: dict):
    """Abandon the running episode (reason "load" unless it had ended already) and start a new one at the saved state.
    The journal keeps every episode's rows; RECORD (what is graded) becomes the saved episode's rows."""
    if not E["done"]:
        finish("load")
    h = q["handle"]
    snap = HANDLES[h]
    _keep(h, snap)  # a load counts as a use
    try:
        sim_load(snap[0])
    except Exception:  # the simulator failed to start the new episode: the trial ends on a simulator error
        E["done"] = True
        E["reason"] = "error"
        log({"ev": "end", "reason": "error", "load_failed": True, "control_steps": len(RECORD), **_journal_pos()})
        write_final()
        raise
    E["loads"] += 1
    E["ep_start"] = E["journal_rows"]
    RECORD[:] = [r.copy() for r in snap[1]]
    E["done"] = False
    E["reason"] = None
    E["t_end"] = None
    rec = Recorder(q["cams"], q["fps"], q["depth"])
    rec.tick(force=True)
    E["calls"] += 1
    RECF.flush()
    sha = S.state_sha()
    log({"ev": "load", "call": E["calls"], "handle": h, "cameras": q["cams"], "control_steps": len(RECORD),
         "rows_total": E["rows"], **_journal_pos(), "state_sha": sha, "restored_exact": sha == snap[0]["sha"]})
    write_final()
    return {**meta_common(), "ok": True, "handle": h}, rec.parts(q["video"], q["fps"])


def _replay_ops(R: np.ndarray, ops: list) -> None:
    """A resume after resets, saves or loads (RCB_STATE_OPS): the whole journal from the frozen start, every reset, save
    and load at the journal row its checkpoint logged (ops: the completed calls' checkpoints, in order), as live."""
    rows, j = [], 0
    HANDLES.clear()
    E["saves"] = E["loads"] = 0

    def apply(e):
        nonlocal rows
        if e["ev"] == "reset":
            reset_sim()
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


def _render_video(A: np.ndarray) -> None:
    """The video: strictly optional, and only after the verdict is final on disk. One render pass: the live session is
    rebuilt from the frozen instance and runs the recorded trajectory again with its camera on (nothing is compared or
    judged; the agent's container is gone and LOCK keeps requests out meanwhile). It is rendered under a temporary name
    and renamed when complete; success adds "video": true to final.json and nothing else. Any failure here (an
    exception; a crash, after which the restarted service finds the verdict and idles; the collect hook giving up on
    it) leaves the verdict exactly as finalize() wrote it."""
    t = time.time()
    try:
        S.render_video(A, VIDEO_TMP, every=max(2, len(A) // 300), fps=10)
        os.replace(VIDEO_TMP[:-4] + ".last.png", os.path.join(DATA, "replay.last.png"))
        os.replace(VIDEO_TMP, os.path.join(DATA, "replay.mp4"))
        E["verdict"]["video"] = True
        log({"ev": "video", "video_s": round(time.time() - t, 1)})
    except Exception as exc:  # noqa: BLE001
        traceback.print_exc()
        try:
            log({"ev": "video_error", "error": repr(exc)[:300], "video_s": round(time.time() - t, 1)})
        except Exception:  # noqa: BLE001
            pass
    finally:
        E["verdict"]["video_pending"] = False
        try:
            if _load_final().get("finalized"):
                write_final()  # "video" and "video_pending" only: the verdict's fields are E["verdict"]'s, unchanged
        except Exception:  # noqa: BLE001
            traceback.print_exc()
        _remove(VIDEO_PENDING, VIDEO_TMP, VIDEO_TMP[:-4] + ".last.png")


def finalize() -> None:
    """The episode is over: the verdict is the live episode's (docs/protocol/README.md H.4; owner, 2026-10-17).
    success = the environment's success check (`_check_success`, as the live episode last ran it) holds on the state
    after the last executed control step, the scene was not solved at the start, and the episode did not end on a
    simulator error (whose state is the partial step's, not the last executed one). Nothing is replayed to grade it.
    The trajectory and the verdict with "finalized": true are written (one atomic write of final.json); then, and only
    then, the optional video is rendered from the trajectory."""
    if STATE_OPS and E.get("gave_up"):  # the verdict (a give-up) is on disk already
        return
    with LOCK:
        A = None
        try:
            A = np.asarray(RECORD, float).reshape(-1, N)
            live_sha = S.state_sha()
            live = S.check()
            initial = bool(getattr(S, "initial_success", False))
            np.savez(
                os.path.join(DATA, "trajectory.npz"),
                actions=A,
                tool=TOOL.__name__,
                meta=json.dumps({"source": "standard-mode episode", "reason": E["reason"]}),
            )
            live_ok = bool(live["success"])
            E["verdict"] = {
                "graded_by": "live",
                "replays": 0,
                "live_success": live_ok,
                "initial_success": initial,
                "success": bool(live_ok and not initial and E["reason"] != "error"),
                "live_state_sha": live_sha,
                "state_sha": live_sha,
                "final_reward": float(live.get("reward") or 0.0),
                "n_actions": int(len(A)),
                "video": False,
                "video_pending": True,
                "video_source": "render",
            }
            if STATE_OPS:
                E["verdict"]["episodes_log"] = episodes_log(_events())
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            log({"ev": "finalize_error", "error": repr(exc)})
            E["verdict"] = {"finalize_error": repr(exc)[:300], "graded_by": "live", "replays": 0}
            A = None
        if A is not None:
            # Announced before the verdict is written: whoever sees the verdict also sees that a video is on its way.
            open(VIDEO_PENDING, "w").close()
        E["finalized"] = True
        write_final()  # the verdict and "finalized": true, in one atomic write; nothing below changes them
        if A is not None:
            log({"ev": "verdict", "control_steps": int(len(A)), "success": E["verdict"]["success"],
                 "live_success": E["verdict"]["live_success"]})
            _render_video(A)


def finish(reason: str) -> None:
    if E["done"]:
        return
    E["done"] = True
    E["reason"] = reason
    E["t_end"] = time.time()
    log({"ev": "end", "reason": reason, "control_steps": len(RECORD), **(_journal_pos() if STATE_OPS else {})})
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
            meta, parts = rcb1.decode(raw)
            if not isinstance(meta, dict):
                raise ValueError("the RCB1 header's meta must be a JSON object")
            return meta, parts
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
        if STATE_QUERY and self.path == "/state/entities":  # fixed for the episode
            return self._json(200, STATE_ENTS)
        return self._json(404, {"error": "no such endpoint"})

    def do_POST(self):
        _touch()
        ops = (("/reset",) if RESET_BUDGET else ()) + (("/save_state", "/load_state") if CAN_SAVE else ())
        if self.path not in ("/submit", "/observe") + ops + (("/state/query",) if STATE_QUERY else ()):
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
        if self.path == "/state/query":
            return self._state_query()
        with LOCK:
            log({"ev": "call_begin", "path": self.path, "call": E["calls"] + 1})
            try:
                return self._serve()
            finally:
                log({"ev": "call_end", "path": self.path})

    def _serve(self):
        meta, parts = self._req
        submit = self.path == "/submit"
        if STATE_OPS and E.get("gave_up"):
            return self._json(409, {"error": "the sim service gave up on this trial", "reason": E["reason"], "gave_up": True})
        if submit and E["done"]:
            return self._json(409, {"error": "episode finished", "reason": E["reason"],
                                    **({"resets_left": _reset_counters()["resets_left"]} if RESET_BUDGET else {})})
        if self.path in ("/reset", "/save_state", "/load_state"):
            return self._serve_op(meta)
        # 1. Validation: nothing has run yet, so whatever is wrong here is the request's fault and a 400; it never ends
        #    the episode (PROTOCOL §4.6). ValueError carries the explanation; any other exception is a field of a type
        #    the checks did not expect.
        try:
            q = parse_submit(meta, parts) if submit else parse_observe(meta)
        except ValueError as exc:
            return self._json(400, {"error": str(exc)[:500]})
        except Exception as exc:  # noqa: BLE001
            return self._json(400, {"error": f"malformed request ({type(exc).__name__}: {str(exc)[:200]})"})
        # 2. Execution: an error from here on is the simulator's (500) and ends the episode.
        try:
            out, blobs = run_submit(q) if submit else run_observe(q)
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            log({"ev": "error", "error": repr(exc)})
            finish("error")  # protocol v1.0 §4.6: a simulator error ends the episode
            return self._json(500, {"error": "simulator error", "detail": repr(exc)[:200]})
        return self._rcb1(out, blobs)

    def _state_query(self):
        """RCB_STATE_QUERY: POST /state/query reads the simulator between calls (no control step, no clock); served only
        while the episode runs (409 after it ended)."""
        meta, _ = self._req
        with LOCK:
            if E["done"] or E["finalized"] or E.get("stopping") or E.get("gave_up"):
                return self._json(409, {"error": "the episode has ended: state queries are served only while it runs",
                                        "reason": E["reason"]})
            plan, err = plan_state_query(STATE_ENTS, meta)
            if err:
                return self._json(400, err)
            try:
                state = STATE_TABLE.query(plan)
                json.dumps(state, allow_nan=False)
            except Exception as exc:  # noqa: BLE001 -- the query only reads: nothing ends
                traceback.print_exc()
                log({"ev": "state_query_error", "error": repr(exc)[:300]})
                return self._json(500, {"error": "state query failed", "detail": repr(exc)[:200]})
            E["state_queries"] += 1
            log({"ev": "state_query", "n": E["state_queries"], "names": list(plan),
                 "fields": sorted({f for fs in plan.values() for f in fs}), "control_steps": len(RECORD)})
            return self._json(200, {"state": state, "control_steps": len(RECORD),
                                    "sim_time_s": round(len(RECORD) * CTRL_DT, 4), "state_queries": E["state_queries"]})

    def _serve_op(self, meta):
        """RCB_STATE_OPS: /reset, /save_state, /load_state. Checks (409 / 400) and validation first, nothing changed."""
        try:
            op = (check_reset() or {}) if self.path == "/reset" else check_ops(meta, self.path)
        except Refused as exc:
            return self._json(exc.code, exc.body)
        try:
            q = parse_reset(meta) if self.path != "/save_state" else {}
        except ValueError as exc:
            return self._json(400, {"error": str(exc)[:500]})
        except Exception as exc:  # noqa: BLE001
            return self._json(400, {"error": f"malformed request ({type(exc).__name__}: {str(exc)[:200]})"})
        try:
            out, blobs = {"/reset": do_reset, "/save_state": do_save, "/load_state": do_load}[self.path]({**q, **op})
        except Refused as exc:  # a reset that missed the frozen start: the service gave up
            return self._json(exc.code, exc.body)
        except Exception as exc:  # noqa: BLE001
            traceback.print_exc()
            log({"ev": "error", "error": repr(exc)})
            finish("error")  # a simulator error ends the episode
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


_renderer(False)  # opens the renderer before the first request
calib()


def _start_mismatch(sha: str) -> str | None:
    """Why the scene just built is not the frozen instance's start, or None. The verdict is the live episode's, so no
    later replay from the frozen instance checks the start: it is checked here, before the agent is served (board
    0151). RCB_INSTANCE_SHA: the instance's own hash (casa_tool checks the file against it on load; this checks that the
    file is the task's frozen instance); RCB_START_SHA: the state hash a fresh episode of this instance starts in
    (scripts/<benchmark>/frozen/start_state.json). A frozen instance is never solved at its start (the generator only
    publishes verified ones), so a scene that is is not that instance either."""
    want = os.environ.get("RCB_INSTANCE_SHA", "")
    have = str(getattr(S, "instance_sha", ""))
    if want and have != want:
        return f"the instance file has hash {have[:16]}, the task's frozen instance {want[:16]}"
    want = os.environ.get("RCB_START_SHA", "")
    if want and sha != want:
        return f"the scene starts in state {sha[:16]}, the frozen instance in {want[:16]}"
    if getattr(S, "initial_success", False):
        return "the scene is solved at the start; the frozen instance is not"
    return None


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
        sha = S.state_sha()
        if STATE_OPS:
            E["start_sha"] = sha
        log({"ev": "start", "tool": TOOL.__name__, "state_sha": sha, "protocol": PROTOCOL})
        bad = _start_mismatch(sha)
        if bad:  # before the port is bound: the agent is never served a scene that is not the task's
            _give_up(f"instance_mismatch: {bad}", infra=True)
        write_final()
        E["t0"] = time.time()
        return
    fin = _load_final()
    if _graded(fin):  # the verdict is on disk: nothing is replayed, redone or rewritten
        _give_up("restarted after finalize", infra=False)
    resumes = [e for e in ev if e.get("ev") == "resumed"]
    in_call = bool(ev) and ev[-1].get("ev") == "call_begin"
    crashes_in_calls = sum(1 for e in resumes if e.get("crashed_in_call")) + int(in_call)
    if len(resumes) >= MAX_RESUMES:
        all_in_calls = crashes_in_calls == len(resumes) + 1
        _give_up("sim_crashed_on_call" if all_in_calls else "sim_crashed", infra=not all_in_calls)
    kinds = ("submit", "observe") + (("reset", "save", "load") if STATE_OPS else ())
    done = [e for e in ev if e.get("ev") in kinds and "state_sha" in e]
    last = done[-1] if done else starts[0]
    n = int(last.get("control_steps", 0))
    if STATE_OPS:  # journal rows (every episode) at that checkpoint
        n = int(last.get("journal_rows", n))
        E["start_sha"] = starts[0].get("state_sha")
    try:
        R = np.fromfile(REC_PATH, dtype=np.float64)
    except OSError:
        R = np.zeros(0)
    if R.size < n * N:
        _give_up("resume_failed: journal shorter than the log", infra=True)
    R = R[: n * N].reshape(n, N)
    with_ops = STATE_OPS and any(e.get("ev") in ("reset", "save", "load") for e in done)
    if with_ops:  # resets, saves or loads: the whole journal, every op at its row
        _replay_ops(R, [e for e in done if e.get("ev") in ("reset", "save", "load")])
    else:
        for row in R:
            S.step(row)
            RECORD.append(row.copy())
    if S.state_sha() != last["state_sha"]:
        _give_up("resume_failed: replayed state differs from the logged one", infra=True)
    R.tofile(REC_PATH)  # drop the steps of the call that was in flight
    E["calls"] = int(last.get("call", 0))
    E["rows"] = int(last.get("rows_total", 0))
    if STATE_QUERY:
        E["state_queries"] = sum(1 for e in ev if e.get("ev") == "state_query")
    E["t0"] = float(starts[0]["ts"])
    first = [e for e in ev if e.get("ev") == "first_request"]
    if first:
        E["t_first"] = float(first[0]["ts"])  # the agent's clock keeps running through a crash, never back to full
    # the episode's end, if it is within the last completed call: an end past it (the step limit hit by the call that was
    # in flight) was lost with that call, as the call itself
    ended = [e for e in ev if e.get("ev") == "end" and int(e.get("control_steps") or 0) <= n]
    if STATE_OPS:  # the current episode's end: after the reset or load that began it, never a lost reset's or load's
        k0 = max((i for i, e in enumerate(ev) if e is last), default=0)
        b = max((i for i, e in enumerate(ev[: k0 + 1]) if e.get("ev") in ("reset", "load") and "state_sha" in e), default=-1)
        ended = [e for e in ev[b + 1 :] if e.get("ev") == "end" and e.get("reason") not in ("reset", "load")
                 and int(e.get("journal_rows", e.get("control_steps")) or 0) <= n]
        E["resets"], E["ep_start"], E["journal_rows"] = int(last.get("resets", 0)), int(last.get("ep_start", 0)), n
        E["saves"], E["loads"] = int(last.get("saves", E["saves"])), int(last.get("loads", E["loads"]))
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
print(f"{PROTOCOL} RoboCasa sim ready ({TOOL.__name__}: {S.inst.get('env_name')})", flush=True)
_bind(int(os.environ.get("RCB_PORT", "8000")), Hd).serve_forever()  # RCB_PORT: for tests, as RCB_DATA
