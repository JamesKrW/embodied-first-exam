"""rcb-limited/2.1 sim service for the MuJoCo tasks (DexToolBench, MuJoCo Playground): robot as a service.

The sidecar owns the simulator (the task's tooling module, RCB_TOOL = rcb_dtb | rcb_aloha | rcb_panda, with the task's
frozen instance) and serves the same wire protocol as the RoboTwin limited tasks (docs/protocol/standard_mode.md), so
the agent uses the same client (rcb_client.py): GET /spec, GET /status, POST /observe, POST /submit, binary RCB1
responses. What the agent gets: camera RGB (+ depth, with each camera's intrinsics and pose: the viewpoint the image is
rendered from, recomputed for every response, and per frame for a camera that moves), the robot's own joint
positions, velocities and current targets, and the task's definition (DexToolBench: the current goal pose and its index,
as SimToolReal's deployment gives its policy; Playground: the target). Never the object's pose.

Actions: `joint_position` rows (the tooling's N actuator targets), each held for `dt` (a multiple of the control step,
hold or linear interpolation per control step). DexToolBench episodes end when all goals are reached, when a goal times
out (only if RCB_DTB_GOAL_TIMEOUT_S > 0; the limited twins set 0), at RCB_MAX_CONTROL_STEPS, or when the agent's time is
up; Playground episodes run until the agent phase ends and the final state is graded.

Evidence under /data: episode.jsonl (every call), calls/NNNNNN/ (meta + actions), and at the end trajectory.npz
((T, N) per control step), final.json and replay.mp4. The verdict is the live episode's (docs/protocol/README.md H.4; owner,
2026-10-17; graded_by = "live", replays = 0): the task's own rule on the episode this service ran
(DexToolBench: every goal reached, the goal tracker running every live control step; Playground: the tooling's check() of
the live final state); nothing is replayed to grade it. The collect hook ends the episode with SIGUSR1. final.json is
`finalized` as soon as the verdict is in it; the video is then rendered from the trajectory in one pass and never
changes or delays the verdict (`video_pending` while it renders; `video_done` tells the collect hook that nothing more
comes). The frozen start is checked when the episode begins: the built scene's state hash against the instance's
recorded one (instance.json `state_sha`, else RCB_START_SHA); a mismatch gets no episode (instance_mismatch, an
infrastructure failure).

Requests are validated in full before any physics runs: a malformed one (dt / seconds / video_fps not a finite number,
cameras not a list of names, ...) is answered 400 and changes nothing; only a failure of the simulator itself is a 500
that ends the episode. `time_s` / `time_left_s` count from the agent's first request to the service.

Optional reset capability (PROTOCOL §4.5; RCB_RESET_BUDGET = N > 0, default 0; DexToolBench's standard twins set 20):
POST /reset abandons the running episode and starts a new one from the frozen start state (Session.reset()), N times
in all. Only the last episode is graded (trajectory.npz holds its rows, replayable from the frozen start); a
success ends the trial (no reset after it), as does the agent's time (SIGUSR1). Without a budget /reset is a 404.

Ablation (off unless the task sets RCB_STATE_QUERY=1; nothing changes when off): GET /state/entities and POST
/state/query serve state_table(), the privileged state in MuJoCo's world frame (quaternions w x y z): the robot (its fixed
base's pose, velocity and height above the floor, every joint of it), its palm and fingertips (DexToolBench) or gripper
sites (ALOHA), every free object (the tool; the peg and the socket) with pose, velocity, size and bbox, and the table with
its props. A query reads a scratch copy of the live state (MjRead: positions and velocities recomputed there, the live
data untouched), under the lock between calls; it takes no control step and leaves the clock as it is."""
from __future__ import annotations
import copy, errno, importlib, json, math, os, signal, sys, threading, time, traceback
from http.server import BaseHTTPRequestHandler, HTTPServer
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
os.environ["RCB_SESSION_LOG"] = ""; os.environ["RCB_AGENT_TRACE_DIR"] = ""
import rcb1  # noqa: E402

PROTOCOL = "eai-standard/2.1"
DATA = os.environ.get("RCB_DATA", "/data"); os.makedirs(os.path.join(DATA, "calls"), exist_ok=True)   # RCB_DATA / RCB_PORT: test hooks (never set in tasks)
TOOL = importlib.import_module(os.environ.get("RCB_TOOL", "rcb_dtb"))
INSTANCE = os.environ.get("RCB_INSTANCE", os.path.join(os.path.dirname(__file__), "instance.json"))
MAX_CHUNK_S = float(os.environ.get("RCB_MAX_CHUNK_S", "10")); MAX_WATCH_S = float(os.environ.get("RCB_MAX_WATCH_S", "30"))
# Episode cap in control steps (0 = none). Set where nothing else ends the episode, e.g. DexToolBench without its per-goal
# time limit (the limited twins: RCB_DTB_GOAL_TIMEOUT_S=0, RCB_MAX_CONTROL_STEPS=360000 = 100 min of robot time).
MAX_CONTROL_STEPS = int(os.environ.get("RCB_MAX_CONTROL_STEPS", "0"))
MAX_VIDEO_FPS = 30; MAX_FRAMES_PER_CALL = 3000   # frames x cameras per response: bounds the service's memory
WALL_CLOCK_S = float(os.environ.get("RCB_WALL_CLOCK_S", "3600"))
RESET_BUDGET = max(0, int(os.environ.get("RCB_RESET_BUDGET", "0") or 0))   # PROTOCOL §4.5: 0 = one episode, no /reset
# The unlimited-reset ablation (off unless the task sets RCB_STATE_OPS=1; nothing changes when off): beside the reset
# (RCB_RESET_BUDGET), POST /save_state and /load_state (snapshot: MuJoCo's integration state, the tooling's own
# per-episode state and the graded rows, in memory); every reset and load begins a new episode; meta and /status add
# saves and loads; final.json adds saves, loads, episodes_log, first_success and, for a success, resets_at_success and
# loads_at_success; a crash resume replays every reset, save and load at its journal row.
STATE_OPS = os.environ.get("RCB_STATE_OPS") == "1"
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
MAX_BODY = 64 * 1024 * 1024; MAX_RESP = 256 * 1024 * 1024
LOCK = threading.Lock()
E = {"ready": False, "calls": 0, "rows": 0, "done": False, "reason": None, "t0": time.time(), "finalized": False,
     "t_first": None, "video_done": False, "resets": 0, "ep_start": 0, "journal_rows": 0, "stopping": False}
if STATE_OPS: E.update(saves=0, loads=0)   # /save_state and /load_state calls that took effect
if STATE_QUERY: E["state_queries"] = 0     # /state/query calls answered
HANDLES: dict = {}                         # RCB_STATE_OPS: handle -> (simulator snapshot, the graded rows), LRU order
# t_first: the agent's first request (its clock starts there). resets: POST /reset calls that started a new episode;
# journal_rows: control steps executed over the whole trial (record.f64's length); ep_start: where the current episode
# begins in it. stopping: the agent's time is up (SIGUSR1): no reset any more.

# SIGUSR1 (the collect hook's "episode over") may arrive before the service is ready, e.g. during a resume replay; its
# default action would kill the process. Until the real handler is installed, only remember it.
_USR1 = {"pending": False}
signal.signal(signal.SIGUSR1, lambda *_: _USR1.__setitem__("pending", True))

# Startup watchdog: a scene build that hangs (seen under GPU memory pressure: no crash, so compose never restarted it)
# must end the process so compose's restart (on-failure) retries. Neither a Python thread (it needs the GIL, which a hang
# inside native SAPIEN/Vulkan init may hold) nor SIGALRM's default action (the service is PID 1 in its container, and
# PID 1 ignores default-action signals) can do it; faulthandler's C watchdog thread can: it dumps every thread's
# traceback to stderr (the container log: where it hung) and _exit(1)s. Cancelled as soon as the scene exists (a resume
# replay may take minutes). Before the scene exists no episode exists, so the retry is not a reset.
import faulthandler  # noqa: E402
STARTUP_TIMEOUT_S = float(os.environ.get("RCB_STARTUP_TIMEOUT_S", "300"))
faulthandler.dump_traceback_later(STARTUP_TIMEOUT_S, exit=True)


MAX_RESUMES = int(os.environ.get("RCB_MAX_RESUMES", "2"))
REC_PATH = os.path.join(DATA, "record.f64")      # journal: every executed control step's targets, float64 (N,) rows


def _events() -> list:
    try: return [json.loads(l) for l in open(os.path.join(DATA, "episode.jsonl")) if l.strip()]
    except OSError: return []


def _write_json(path: str, obj: dict) -> None:
    """Atomic: a reader (the collect hook, the verifier) never sees a half-written file."""
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w") as f: json.dump(obj, f); f.flush(); os.fsync(f.fileno())
    os.replace(tmp, path)


def log(ev: dict) -> None:
    with open(os.path.join(DATA, "episode.jsonl"), "a") as f: f.write(json.dumps({"ts": time.time(), **ev}) + "\n")


def _graded(fin: dict) -> bool:
    """final.json holds a verdict: the live one (graded_by), or one of a service before 2026-10-17 (deterministic)."""
    return bool(fin.get("finalized")) and ("graded_by" in fin or "deterministic" in fin)


def _give_up(reason: str, infra: bool, **extra) -> None:
    """End the trial's episode without a new one: mark it failed, keep the PID file for the collect hook, and answer every
    request with the refusal (409) until the container is removed."""
    log({"ev": "restart_refused", "reason": reason})
    try: prev = json.load(open(os.path.join(DATA, "final.json")))
    except Exception: prev = {}  # noqa
    if not isinstance(prev, dict): prev = {}
    if not _graded(prev):                                           # a verdict already on disk is kept as it is
        prev.update({"success": False, "live_success": False, "done": True, "reason": reason, "finalized": True, "gave_up": True,
                     "protocol": PROTOCOL, **extra, **({"infra_error": 1} if infra else {})})
    elif prev.get("video_pending"):                                 # the service died rendering the video: only the video is lost
        prev.update(video_pending=False, video=False)
    prev["video_done"] = True                                       # nothing more will be written: the collect hook stops waiting
    _write_json(os.path.join(DATA, "final.json"), prev)
    open(os.path.join(DATA, "server.pid"), "w").write(str(os.getpid()))
    signal.signal(signal.SIGUSR1, lambda *_: None); print(f"restart refused: {reason}", flush=True)
    _refuse_forever(reason, infra)


def _refuse_forever(reason: str, infra: bool) -> None:
    """After a give-up: answer every request 409 {"error", "reason", "gave_up": true, "infra_error"} until the container is
    removed (as the 2.2 layer's rcb_limited.refuse_forever), so the agent's client stops at once (EpisodeOver) instead of
    waiting out its timeout for a service that will never serve. The verdict is on disk already; this never changes it."""
    body = json.dumps({"error": "the sim service gave up on this episode and will not serve it", "reason": reason,
                       "gave_up": True, "infra_error": int(bool(infra))}).encode()

    class Refusal(BaseHTTPRequestHandler):
        timeout = 30                                                # a client that stops sending must not hold the thread
        def _refuse(self):
            try:
                n = int(self.headers.get("Content-Length") or 0)
                if 0 < n <= 64 << 20: self.rfile.read(n)            # take the request first: the client then sees the answer
                self.send_response(409); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
            except (OSError, ValueError): pass
            self.close_connection = True
        do_GET = do_POST = _refuse
        def log_message(self, *a): pass

    def answer():
        port = int(os.environ.get("RCB_PORT", "8000"))
        while True:                                                 # the port may still be held: wait for it
            try: HTTPServer(("0.0.0.0", port), Refusal).serve_forever(); return
            except OSError as exc:
                if exc.errno != errno.EADDRINUSE: traceback.print_exc(); return
            time.sleep(1.0)

    threading.Thread(target=answer, daemon=True).start()
    while True: time.sleep(3600)


with open(os.path.join(DATA, "startups.jsonl"), "a") as _f: _f.write(json.dumps({"ts": time.time(), "pid": os.getpid()}) + "\n")

# Back off before rebuilding after a failed start (e.g. a GPU out-of-memory crash while other trials hold the GPU): compose
# restarts immediately, so without a pause every retry meets the same full GPU (seen: 4 starts in 75 s, all failed).
_n_starts = sum(1 for _ in open(os.path.join(DATA, "startups.jsonl")))
if _n_starts > 1 and not os.path.exists(os.path.join(DATA, "episode.jsonl")): time.sleep(min(120, 20 * (_n_starts - 1)))

# The frozen start's recorded state hash (qpos + qvel after the reset): instance.json's own `state_sha` (Playground ALOHA /
# Panda; their tooling refuses another start itself), else RCB_START_SHA (DexToolBench, whose instance records none: the
# generator scripts/mj_mk_limited.py writes the hash every archived episode of the task started from).
try: START_SHA = str(json.load(open(INSTANCE)).get("state_sha") or os.environ.get("RCB_START_SHA") or "")
except Exception: START_SHA = os.environ.get("RCB_START_SHA") or ""  # noqa
try:
    S = TOOL.Session(INSTANCE, enable_render=True)
except RuntimeError as exc:   # the tooling's own start check (ALOHA, Panda): the scene did not build to the frozen start
    if "initial state hash" not in str(exc): raise
    faulthandler.cancel_dump_traceback_later()
    _give_up("instance_mismatch", infra=True, instance_mismatch=1, error=str(exc)[:300], video_done=True)
E["built"] = True; faulthandler.cancel_dump_traceback_later()    # the scene exists: the startup watchdog stands down (a resume replay may take minutes)
KIND = "dtb" if TOOL.__name__ == "rcb_dtb" else "pg"
CTRL_DT = float(TOOL.CONTROL_DT); N = int(TOOL.N)
NAMES = list(getattr(TOOL, "JOINT_NAMES", None) or getattr(TOOL, "ACTUATOR_NAMES"))
RECORD: list = []                         # the current episode: one row of targets per control step (what is graded)


def _cams() -> dict:
    """name -> (mujoco camera id or None, free-camera settings or None)."""
    m = S.model; named = [m.camera(i).name for i in range(m.ncam)]
    want = list(getattr(TOOL, "CAMERAS", ())) or []
    out = {c: (m.camera(c).id, None) for c in want if c in named}
    if not out:   # scenes without named cameras (Panda): four fixed free-camera views around the workspace
        for name, (az, el) in {"front": (180, -20), "left": (90, -25), "right": (270, -25), "top": (180, -70)}.items():
            out[name] = (None, dict(lookat=(0.45, 0.0, 0.25), distance=1.5, azimuth=az, elevation=el))
    return out


CAMS = _cams()
W, H = 320, 240
_R = {"rgb": None, "depth": None}


def _renderer(depth: bool):
    k = "depth" if depth else "rgb"
    if _R[k] is None:
        _R[k] = S.mj.Renderer(S.model, H, W)
        if depth: _R[k].enable_depth_rendering()
    return _R[k]


def _update(r, cam):
    cid, free = CAMS[cam]
    if cid is not None: r.update_scene(S.data, camera=cid)
    else:
        c = S.mj.MjvCamera(); c.lookat[:] = free["lookat"]; c.distance = free["distance"]; c.azimuth = free["azimuth"]; c.elevation = free["elevation"]
        r.update_scene(S.data, camera=c)


def grab(cams, depth: bool):
    imgs, dep = {}, {}
    for c in cams:
        r = _renderer(False); _update(r, c); imgs[c] = r.render().copy()
        if depth:
            r = _renderer(True); _update(r, c); dep[c] = r.render().astype(np.float16)
    return imgs, dep


def _moving(cam: str) -> bool:
    """True for a camera whose pose can change during the episode: a tracking / targeting camera, or one on a body that a
    joint or a mocap pose moves (ALOHA's wrist cameras). Free cameras and cameras fixed to the world never move."""
    cid, _ = CAMS[cam]
    if cid is None: return False
    m = S.model
    if int(m.cam_mode[cid]) != 0: return True
    b = int(m.cam_bodyid[cid])
    while b > 0:
        if int(m.body_jntnum[b]) > 0 or int(m.body_mocapid[b]) >= 0: return True
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
        R = np.array(S.data.cam_xmat[cid], float).reshape(3, 3); pos = np.array(S.data.cam_xpos[cid], float)
    else:
        r = _renderer(False); _update(r, cam); a, b = r.scene.camera[0], r.scene.camera[1]
        fwd = np.array(a.forward, float); up = np.array(a.up, float); pos = 0.5 * (np.array(a.pos, float) + np.array(b.pos, float))
        R = np.column_stack([np.cross(fwd, up), up, -fwd])
    return np.vstack([np.column_stack([R, pos]), [0.0, 0.0, 0.0, 1.0]])


def intrinsics(cam: str) -> tuple[float, float, float, float, float]:
    """(fx, fy, cx, cy) in pixels of the W x H image MuJoCo renders for this camera, and its vertical field of view in
    degrees. MuJoCo's rule (3.3.7, checked against CPU renders):
    - a camera given by its vertical field of view (`fovy`; every free camera: the model's global fovy) has square
      pixels whatever the image's aspect: fx = fy = (H / 2) / tan(fovy / 2), principal point at the image centre;
    - a camera given by its sensor (`sensorsize`, with `focal` / `focalpixel` and optionally `principal` /
      `principalpixel`: model.cam_sensorsize > 0, model.cam_intrinsic = focal x, focal y, principal x, principal y in
      length units) is rendered with the sensor's own extent in each direction, squeezed or stretched into the W x H
      image: fx = W * focal_x / sensor_x, fy = H * focal_y / sensor_y, cx = W / 2 - W * principal_x / sensor_x,
      cy = H / 2 + H * principal_y / sensor_y. Its pixels are square only when W : H is the sensor's aspect.
    Until 2026-10-13 every camera was reported by the first rule. ALOHA's overhead and wrist cameras are given by a
    3.896 x 2.140 mm sensor with a 1.93 mm lens and rendered at 320 x 240: fx is 158.52 px, not the 216.45 reported."""
    cid = CAMS[cam][0]; m = S.model
    fovy = float(m.cam_fovy[cid]) if cid is not None else float(m.vis.global_.fovy)
    sx, sy = (float(v) for v in m.cam_sensorsize[cid]) if cid is not None else (0.0, 0.0)
    if sx > 0 and sy > 0:
        fxl, fyl, pxl, pyl = (float(v) for v in m.cam_intrinsic[cid])
        return W * fxl / sx, H * fyl / sy, W / 2 - W * pxl / sx, H / 2 + H * pyl / sy, fovy
    f = 0.5 * H / np.tan(np.deg2rad(fovy) / 2)
    return f, f, W / 2, H / 2, fovy


def calib() -> dict:
    """Per camera: pinhole intrinsics (see intrinsics) and the camera-to-world pose in the current state (see cam2world)."""
    out = {}
    for c in CAMS:
        fx, fy, cx, cy, fovy = intrinsics(c)
        out[c] = {"intrinsic_cv": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], "cam2world_gl": cam2world(c).tolist(),
                  "width": W, "height": H, "fovy_deg": fovy, "moving": MOVING[c]}
    return out


def proprio() -> np.ndarray:
    st = S.state(); q = np.asarray(st.get("q", st.get("qpos"))[:N] if "q" in st else _robot_q(), float)
    qd = np.asarray(st["qd"] if "qd" in st else _robot_qd(), float)
    return np.concatenate([q, qd, np.asarray(st["targets"], float)]).astype(np.float32)


def _robot_q():
    return np.array([S.data.qpos[S.model.jnt_qposadr[S.model.actuator_trnid[a, 0]]] for a in S._act])


def _robot_qd():
    return np.array([S.data.qvel[S.model.jnt_dofadr[S.model.actuator_trnid[a, 0]]] for a in S._act])


def task_info() -> dict:
    if KIND == "dtb":
        p = S.progress()
        return {"goal_index": p["reached"], "n_goals": p["n_goals"], "goal": None if p["done"] else list(S.goals[p["reached"]]),
                "goals_reached": p["reached"], "failed_on_timeout": p["failed"], "steps_left_on_goal": p["steps_left_on_goal"],
                "goal_time_limit_s": round(TOOL.GOAL_TIMEOUT_STEPS * CTRL_DT, 3) or None,
                "pose_format": "x y z qx qy qz qw (world frame, metres)"}
    info = {"target_pos": np.asarray(S.target_pos).tolist()} if hasattr(S, "target_pos") else {}
    if hasattr(S, "target_quat"): info["target_quat_wxyz"] = np.asarray(S.target_quat).tolist()
    return info


PROPRIO_LAYOUT = [f"{n}_pos" for n in NAMES] + [f"{n}_vel" for n in NAMES] + [f"{n}_target" for n in NAMES]
SPEC = {"protocol": PROTOCOL, "simulator": "mujoco", "task_text": S.inst.get("text", ""), "control_dt": CTRL_DT,
        "max_chunk_s": MAX_CHUNK_S, "max_watch_s": MAX_WATCH_S, "max_control_steps": MAX_CONTROL_STEPS or None,
        "max_video_fps": MAX_VIDEO_FPS, "max_frames_per_call": MAX_FRAMES_PER_CALL,
        "on_service_error": ("if a call fails with a connection error or 5xx, the service may have crashed and resumed the same "
                             "episode (never a reset): the state is the one after the last call that completed on the service, "
                             "which you may not have received; check GET /status (control_steps) before continuing"),
        "cameras": {c: {"rgb": True, "depth": True, "size": [W, H], "moving": MOVING[c]} for c in CAMS},
        "camera_calibration": ('meta["cameras"][cam] of every response: intrinsic_cv (pinhole, pixels of the returned image: [[fx, 0, cx], [0, fy, cy], '
                               "[0, 0, 1]]; fx and fy differ for a camera whose sensor has another aspect than the image) and cam2world_gl (4x4, OpenGL axes: x right, "
                               "y up, the camera looks along -z; the viewpoint the image is rendered from), computed for the state at the END "
                               "of that call (the `last/<cam>` image and the last frame of every stack). A camera with moving = false never "
                               "moves. For a camera with moving = true (mounted on the robot) the response also carries the part "
                               "`cam2world_gl/<cam>`: float64 (n_frames, 4, 4), one pose per returned frame of that camera "
                               '(client: res.raw["cam2world_gl/<cam>"]["data"])'),
        "action_types": {"joint_position": {"dim": N, "layout": NAMES, "lower": S.lower.tolist(), "upper": S.upper.tolist(),
                                            "semantics": "rows `dt` apart (a multiple of control_dt, >= control_dt); each row's targets held "
                                                         "(interpolate=hold) or ramped linearly from the previous row (linear) over its control steps; "
                                                         "clipped to [lower, upper]"},
                         "not_yet": ["ee_pose", "joint_delta"]},
        "proprio_layout": PROPRIO_LAYOUT,
        "budget": {"reset": RESET_BUDGET, "wall_clock_s": WALL_CLOCK_S,
                   "clock": ("meta time_s / time_left_s count from your first request to the service (any endpoint), not from the "
                             "service's start, and keep running through a restart of the service; time_left_s = budget.wall_clock_s - "
                             "time_s. The harness counts the same limit from the start of your session, so time_left_s never runs out "
                             "before the harness stops you: your real time left is time_left_s minus the time you spent before that "
                             "first request. Plan by time_left_s.")},
        "episode": (("ends when every goal is reached" + (f", when a goal is not reached within {TOOL.GOAL_TIMEOUT_STEPS * CTRL_DT:g} s" if TOOL.GOAL_TIMEOUT_STEPS else "")
                     + (f", after {MAX_CONTROL_STEPS} control steps" if MAX_CONTROL_STEPS else "") + ", or when your time is up; the recorded trajectory is scored")
                    if KIND == "dtb" else "runs until the agent phase ends; the final state is graded")}
if RESET_BUDGET:
    SPEC["reset"] = ("POST /reset abandons the running episode and starts a new one from the same starting state, "
                     "budget.reset times in all; only the last episode is graded, and a success ends the trial. The clock "
                     "(time_left_s) keeps running. meta and /status add episode, resets_left and control_steps_total")
    SPEC["episode"] += "; with resets, only the last episode is scored"


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


def _top(m, b: int) -> int:
    """The body's ancestor that hangs from the world body."""
    while int(m.body_parentid[b]) != 0:
        b = int(m.body_parentid[b])
    return b


def state_table():
    """The state-query ablation's table (RCB_STATE_QUERY=1), built once after the scene: MuJoCo's world frame. Robots:
    each body hanging from the world whose subtree holds an actuated joint (DexToolBench: the iiwa with the Sharpa hand,
    `robot`; ALOHA: `left_arm`, `right_arm`), its base pose, velocity and height above the floor, and every hinge or slide
    joint of its subtree; links: the hand's palm and fingertips (DexToolBench), the grippers' sites (ALOHA); objects:
    every free body (the privileged state()'s names: DexToolBench `object`, the tool; ALOHA `peg` and `socket`, or
    `box`), with size and bbox; scene features: the table and its props. The goal marker is not in it (task.goal gives
    the goal). Read by MjRead (a scratch copy of the live state)."""
    m = S.model
    R = MjRead(S.mj, lambda: S.model, lambda: S.data).refresh()
    name = lambda b: m.body(b).name  # noqa: E731
    act_j = [int(m.actuator_trnid[a, 0]) for a in S._act if int(m.actuator_trntype[a]) == 0]
    roots = sorted({_top(m, int(m.jnt_bodyid[j])) for j in act_j})
    robot_bodies = set().union(*(R.subtree(r) for r in roots))
    if KIND == "dtb":
        frame = ("MuJoCo's world frame (metres; z up; origin on the floor under the table's centre, the table top at "
                 "z = 0.53 m): the frame of camera_calibration's cam2world_gl and of task.goal; quat is w x y z, while "
                 "task.goal's pose_format is x y z qx qy qz qw")
    else:
        frame = ("MuJoCo's world frame (metres; z up; origin and axes as the scene file sets them): the frame of "
                 "camera_calibration's cam2world_gl" + (" and of task.target_pos" if hasattr(S, "target_pos") else "")
                 + "; quat is w x y z")
    t = MjStateTable(StateTable(frame=frame, quaternion_order="wxyz",
                                note="every pose is a body's (or site's) frame origin; velocities are world-frame, at "
                                     "that origin"), R)
    for r in roots:
        rname = "robot" if len(roots) == 1 else (name(r).split("/")[0] + "_arm" if "/" in name(r) else name(r))
        joints = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in R.subtree(r)]
        t.add(rname, "robot", {**mj_body_fields(R, r, ground_exclude=robot_bodies), **mj_joint_fields(R, joints, ranges=True)},
              note=f"base body {name(r)!r} (fixed base)")
    if KIND == "dtb":
        for b in sorted(robot_bodies):
            n = name(b)
            if n.endswith("hand_C_MC"):
                t.add("palm", "robot_link", mj_body_fields(R, b), note=f"the Sharpa hand's palm, body {n!r}")
            elif n.endswith("_DP"):
                t.add(n.split("_")[-2] + "_tip", "robot_link", mj_body_fields(R, b), note=f"a fingertip: the distal phalanx body {n!r}")
    else:
        for sname in ("left/gripper", "right/gripper"):
            try:
                sid = m.site(sname).id
            except KeyError:
                continue
            t.add(sname.split("/")[0] + "_gripper", "robot_link", mj_site_fields(R, sid), note=f"site {sname!r}")
    free = [int(m.jnt_bodyid[j]) for j in range(m.njnt) if int(m.jnt_type[j]) == 0 and int(m.jnt_bodyid[j]) not in robot_bodies]
    for b in free:
        t.add(name(b) or f"body{b}", "object", mj_body_fields(R, b, size=True, bbox=True))
    try:
        tb = m.body("table").id
    except KeyError:
        tb = None
    if tb is not None:
        t.add("table", "scene_feature", {k: v for k, v in mj_body_fields(R, tb, vel=False, size=True, bbox=True).items()},
              note="the table body with every prop fixed to it")
    groups: dict = {}
    for g in range(m.ngeom):
        n = m.geom(g).name
        if int(m.geom_bodyid[g]) in (tb, 0) and n and n not in ("table_geom", "floor", "table"):
            groups.setdefault(n.split("_")[0], []).append(g)
    for gname, gids in groups.items():
        if gname in t.table.entities()["entities"]:
            continue

        def gbox(gids=gids):
            p = np.concatenate([R._corners(g) for g in gids])
            return [p.min(0), p.max(0)]
        t.add(gname, "scene_feature", {"pos": ("m (its bbox's centre)", lambda gbox=gbox: (gbox()[0] + gbox()[1]) / 2),
                                       "bbox": ("m ([min xyz, max xyz], world axes)", gbox)},
              note=f"fixed geometry: {', '.join(m.geom(g).name for g in gids[:4])}" + (" ..." if len(gids) > 4 else ""))
    return t


STATE_TABLE = state_table() if STATE_QUERY else None
STATE_ENTS = check_state_table(STATE_TABLE) if STATE_QUERY else None
if STATE_QUERY:
    SPEC["state_query"] = STATE_QUERY_NOTE
if CAN_SAVE:
    SPEC["state_ops"] = ("POST /save_state ({}) stores the current state of the running episode in this service "
                         "and answers JSON {\"handle\": \"s<k>\", \"step\": control steps of the episode, ...}: the "
                         "handle is all you get, never the state's contents. POST /load_state {\"handle\": h, "
                         "cameras, video} restores a state saved earlier in this trial and starts a new episode "
                         "there (answered as /reset is); nothing else can be loaded. The task's success rule is "
                         f"checked again from your next command on. At most {MAX_HANDLES} handles are kept: saving "
                         "more drops the least recently used one (loading it is a 409). Saves and loads do not "
                         "count against budget.reset; a success ends the trial (409 after it)")


def _success() -> bool:
    """The running episode has succeeded (DexToolBench: every goal reached): the trial is over, no reset after it."""
    return E["done"] and E["reason"] == "all goals reached"


def _reset_counters() -> dict:
    if not RESET_BUDGET: return {}
    return {"episode": E["resets"] + E.get("loads", 0) + 1, "control_steps_total": E["journal_rows"],
            "resets_left": 0 if _success() or E["stopping"] else RESET_BUDGET - E["resets"],
            **({"saves": E["saves"], "loads": E["loads"]} if CAN_SAVE else {})}


def _agent_time() -> float:
    """Seconds since the agent's first request to the service (0 before it): the agent's clock. The simulator is up
    before the agent starts (the harness installs the agent first), so the service's own start is not that moment."""
    return 0.0 if E["t_first"] is None else time.time() - E["t_first"]


def _touch() -> None:
    """Every request passes here: the first one starts the agent's clock (logged, so a resume keeps it)."""
    if E["t_first"] is None:
        E["t_first"] = time.time(); log({"ev": "first_request", "ts": E["t_first"]})


def meta_common() -> dict:
    out = {"calls": E["calls"], "rows": E["rows"], "control_steps": len(RECORD), "sim_time_s": round(len(RECORD) * CTRL_DT, 3),
           "time_s": round(_agent_time(), 2), "time_left_s": max(0.0, round(WALL_CLOCK_S - _agent_time(), 1)),
           "proprio": {"layout": PROPRIO_LAYOUT, "now": proprio().tolist()}, "task": task_info(), "cameras": calib(),
           "done": E["done"], "reason": E["reason"], **_reset_counters()}
    return out


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


def write_final(extra: dict | None = None) -> None:
    d = {"protocol": PROTOCOL, "tool": TOOL.__name__, "calls": E["calls"], "rows": E["rows"], "control_steps": len(RECORD),
         "elapsed_s": round((E.get("t_end") or time.time()) - E["t0"], 1), "done": E["done"], "reason": E["reason"], "finalized": E["finalized"],
         "video_done": E["video_done"], "reset_budget": RESET_BUDGET, "resets": E["resets"], "episodes": E["resets"] + 1,
         "control_steps_total": E["journal_rows"]}
    d.update(E.get("verdict") or {}); d.update(extra or {})
    if STATE_OPS:
        d.update(saves=E["saves"], loads=E["loads"], episodes=E["resets"] + E["loads"] + 1)
        if "success" in d:                         # the first attempt: graded only when no reset or load came after it
            d["first_success"] = bool(d["success"]) and E["resets"] + E["loads"] == 0
        if d.get("success"):                       # a success ends the trial: the graded episode is the last one
            d.update(resets_at_success=E["resets"], loads_at_success=E["loads"])
    if STATE_QUERY: d["state_queries"] = E["state_queries"]
    _write_json(os.path.join(DATA, "final.json"), d)


def _every(fps: int) -> int:
    """The Recorder's capture stride: a frame every this many control steps for the requested video_fps."""
    return max(1, int(round(1.0 / (CTRL_DT * max(1, fps)))))


class Recorder:
    def __init__(self, cams, fps, depth):
        self.cams = cams; self.every = _every(fps); self.depth = depth
        self.frames = {c: [] for c in cams}; self.dep = {c: [] for c in cams}; self.prop = []; self.n = 0
        self.pose = {c: [] for c in cams if MOVING[c]}     # a moving camera: its pose with every frame
    def tick(self, force=False):
        if force or self.n % self.every == 0:
            imgs, dep = grab(self.cams, self.depth)
            for c in self.cams:
                self.frames[c].append(imgs[c])
                if c in dep: self.dep[c].append(dep[c])
                if c in self.pose: self.pose[c].append(cam2world(c))
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
                import imageio.v2 as imageio, tempfile
                with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as tf: path = tf.name
                try:
                    with imageio.get_writer(path, format="FFMPEG", mode="I", fps=fps, codec="libx264", macro_block_size=1,
                                            output_params=["-movflags", "+faststart", "-pix_fmt", "yuv420p"]) as w:
                        for f in fr: w.append_data(np.ascontiguousarray(f, dtype=dt))
                    data = open(path, "rb").read()
                finally:
                    try: os.unlink(path)
                    except OSError: pass
                out.append((f"video/{c}", "mp4", data, {"fps": fps, "n": int(len(fr))}))
            last = rcb1.array_part(f"last/{c}", np.asarray(fr[-1], dtype=dt)) if len(fr) else None
            if video in ("frames", "both"): out.append(fp)
            else: fr.clear()
            if last: out.append(last)
            if self.dep[c]: out.append(rcb1.frames_part(f"depth/{c}", self.dep[c]))   # float16 at capture: 2 bytes a pixel
            self.dep[c] = []
            if self.pose.get(c): out.append(rcb1.array_part(f"cam2world_gl/{c}", np.stack(self.pose[c]).astype(np.float64)))
        out.append(rcb1.array_part("proprio", np.stack(self.prop) if self.prop else np.zeros((0, len(PROPRIO_LAYOUT)), np.float32)))
        return out


def _ctrl_step(tg, rec) -> bool:
    """One control step with targets `tg`; returns True if the episode ended (DexToolBench goal rule)."""
    S.step(tg); row = np.asarray(S.targets, float).copy(); RECORD.append(row); RECF.write(row.tobytes()); E["journal_rows"] += 1; rec.tick()
    if KIND == "dtb":
        p = S.progress()
        if p["done"]: finish("all goals reached"); return True
        if p["failed"]: finish("goal timeout"); return True
    if MAX_CONTROL_STEPS and len(RECORD) >= MAX_CONTROL_STEPS: finish("step limit"); return True
    return False




# ---- request parsing: everything here runs BEFORE any physics. A malformed request is the caller's mistake: it raises
# ValueError (answered 400, nothing executed), never a simulator error (500, which ends the episode; PROTOCOL §4.6).
VIDEO_MODES = ("mp4", "frames", "both", "none")


def _fps(meta: dict) -> int:
    try: fps = int(meta.get("video_fps", 10))
    except (TypeError, ValueError, OverflowError): raise ValueError("video_fps must be an integer") from None
    if not 1 <= fps <= MAX_VIDEO_FPS: raise ValueError(f"video_fps must be in [1, {MAX_VIDEO_FPS}]")
    return fps


def _num(meta: dict, key: str, default: float) -> float:
    """A finite numeric request field; anything else (null, a string, NaN, Infinity) is a 400."""
    try:
        v = meta.get(key, default)
        if isinstance(v, bool): raise TypeError     # JSON true is no number (float(True) is 1.0)
        v = float(v)
    except (TypeError, ValueError, OverflowError): raise ValueError(f"{key} must be a number") from None
    if not math.isfinite(v): raise ValueError(f"{key} must be a finite number")
    return v


def _cameras(meta: dict, default: list) -> list:
    """The requested cameras, each once (unknown names are ignored; none left = the default). A repeated name is dropped
    BEFORE the frame budget and any rendering: the recorder keeps frames per name, so repeats would multiply its work."""
    cams = meta.get("cameras")
    if cams is None: return list(default)
    if not isinstance(cams, (list, tuple)) or not all(isinstance(c, str) for c in cams): raise ValueError(f"cameras must be a list of camera names: {list(CAMS)}")
    return list(dict.fromkeys(c for c in cams if c in CAMS)) or list(default)


def _video(meta: dict, default: str) -> str:
    v = meta.get("video", default)
    if v is None: return default
    if not isinstance(v, str) or v not in VIDEO_MODES: raise ValueError("video must be mp4|frames|both|none")
    return v


def _frame_budget(cams, fps: int, steps: int, depth: bool) -> None:
    """Refuse (400, before anything runs) a call of `steps` control steps whose Recorder would take more than
    MAX_FRAMES_PER_CALL images: per camera the forced first and last frame and one every _every(fps) steps (the stride
    it really films at, not the requested fps), depth counted again."""
    n = (int(steps) // _every(fps) + 2) * len(cams) * (2 if depth else 1)
    if n > MAX_FRAMES_PER_CALL: raise ValueError(f"this call would return {n} images (cameras x frames at the capture stride, depth counted again); "
                                                 f"the limit is max_frames_per_call = {MAX_FRAMES_PER_CALL}: use fewer cameras, a lower video_fps or a shorter chunk")


def parse_submit(meta: dict, parts: dict) -> dict:
    """Validate a /submit request completely; returns what do_submit executes. No physics, no state change."""
    if meta.get("type", "joint_position") != "joint_position": raise ValueError(f"type must be joint_position (not yet: {SPEC['action_types']['not_yet']})")
    raw = parts["actions"].get("data") if isinstance(parts.get("actions"), dict) else meta.get("actions", [])
    try: A = np.asarray(raw, dtype=np.float64)
    except (TypeError, ValueError, OverflowError): raise ValueError(f"actions must be a numeric array (T, {N})") from None
    if A.ndim != 2 or A.shape[1] != N: raise ValueError(f"joint_position actions must be (T, {N}), got {list(A.shape)}")
    if not np.all(np.isfinite(A)): raise ValueError("actions contain NaN/inf")
    dt = _num(meta, "dt", CTRL_DT)
    if not 0 < dt <= 1e6: raise ValueError(f"dt must be a positive multiple of control_dt={CTRL_DT}")
    k = int(round(dt / CTRL_DT))
    if k < 1 or abs(k * CTRL_DT - dt) > 1e-6: raise ValueError(f"dt must be a positive multiple of control_dt={CTRL_DT}")
    if A.shape[0] * dt > MAX_CHUNK_S + 1e-9: raise ValueError(f"chunk longer than max_chunk_s={MAX_CHUNK_S}")
    interp = meta.get("interpolate", "linear")
    if interp not in ("hold", "linear"): raise ValueError("interpolate must be hold|linear")
    cams = _cameras(meta, list(CAMS)[:1]); fps = _fps(meta); video = _video(meta, "mp4"); depth = bool(meta.get("include_depth", False))
    _frame_budget(cams, fps, A.shape[0] * k, depth)
    return {"A": A, "dt": dt, "k": k, "interp": interp, "cams": cams, "fps": fps, "video": video, "depth": depth, "note": str(meta.get("note") or "")[:500]}


def parse_observe(meta: dict) -> dict:
    """Validate an /observe request completely; returns what do_observe executes."""
    cams = _cameras(meta, list(CAMS)); seconds = _num(meta, "seconds", 0); fps = _fps(meta); video = _video(meta, "frames"); depth = bool(meta.get("include_depth", False))
    if seconds < 0 or seconds > MAX_WATCH_S: raise ValueError(f"seconds must be in [0, {MAX_WATCH_S}]")
    _frame_budget(cams, fps, int(round(seconds / CTRL_DT)), depth)
    return {"cams": cams, "seconds": seconds, "fps": fps, "video": video, "depth": depth}


def do_submit(q: dict):
    A, dt, k, interp, cams, fps, video, depth = (q[x] for x in ("A", "dt", "k", "interp", "cams", "fps", "video", "depth"))
    rec = Recorder(cams, fps, depth); rec.tick(force=True); prev = np.asarray(S.targets, float).copy(); n_clip = 0; done = False
    for row in A.astype(np.float64):
        tgt = np.clip(row, S.lower, S.upper); n_clip += int(np.any(tgt != row))
        for j in range(1, k + 1):
            tg = tgt if interp == "hold" else prev + (tgt - prev) * (j / k)
            if _ctrl_step(tg, rec): done = True; break
        prev = tgt
        if done: break
    rec.tick(force=True); E["calls"] += 1; E["rows"] += int(A.shape[0]); RECF.flush()
    out = {**meta_common(), "chunk_id": E["calls"], "type": "joint_position", "executed_rows": int(A.shape[0]), "n_clipped_rows": n_clip, "ok": True}
    log({"ev": "submit", "call": E["calls"], "type": "joint_position", "rows": int(A.shape[0]), "dt": dt, "interpolate": interp, "cameras": cams,
         "note": q["note"], "control_steps": len(RECORD), "n_clipped_rows": n_clip, "task": task_info(),
         "rows_total": E["rows"], **_journal_pos(), "state_sha": S.state_sha()})
    d = os.path.join(DATA, "calls", f"{E['calls']:06d}"); os.makedirs(d, exist_ok=True); np.save(os.path.join(d, "actions.npy"), A.astype(np.float32))
    return out, rec.parts(video, fps)


def do_observe(q: dict):
    cams, seconds, fps, video, depth = (q[x] for x in ("cams", "seconds", "fps", "video", "depth"))
    rec = Recorder(cams, fps, depth); rec.tick(force=True)
    if seconds > 0 and not E["done"]:
        hold = np.asarray(S.targets, float).copy()
        for _ in range(int(round(seconds / CTRL_DT))):
            if _ctrl_step(hold, rec): break
        rec.tick(force=True)
    E["calls"] += 1; RECF.flush()
    log({"ev": "observe", "call": E["calls"], "seconds": seconds, "cameras": cams, "control_steps": len(RECORD),
         "rows_total": E["rows"], **_journal_pos(), "state_sha": S.state_sha()})
    return {**meta_common(), "obs_id": E["calls"], "seconds": seconds, "ok": True}, rec.parts(video if seconds > 0 else ("frames" if video == "mp4" else video), fps)


def _journal_pos() -> dict:
    """Where a checkpoint (a completed call's log line) sits in the journal: what a resume replays (the current episode =
    journal rows ep_start .. journal_rows) and how many resets came before it."""
    return {"journal_rows": E["journal_rows"], "ep_start": E["ep_start"], "resets": E["resets"],
            **({"saves": E["saves"], "loads": E["loads"]} if STATE_OPS else {})}


def parse_reset(meta: dict) -> dict:
    """Validate a /reset request completely (the start's frames: cameras, include_depth; no physics)."""
    cams = _cameras(meta, list(CAMS)); fps = _fps(meta); video = _video(meta, "frames"); depth = bool(meta.get("include_depth", False))
    _frame_budget(cams, fps, 0, depth)
    return {"cams": cams, "fps": fps, "video": "frames" if video == "mp4" else video, "depth": depth}


class Refused(Exception):
    """A request refused before anything changed (409, or `code`)."""
    def __init__(self, body: dict, code: int = 409): super().__init__(body.get("error")); self.body = body; self.code = code


def check_reset() -> None:
    """409 when no new episode may start: the trial is over (a success, or the agent's time is up), or the budget is spent."""
    if _success() or E["stopping"] or E.get("finalizing"):
        raise Refused({"error": "trial finished: " + ("an episode succeeded" if _success() else "your time is up"), "reason": E["reason"], "resets_left": 0})
    if E["resets"] >= RESET_BUDGET:
        raise Refused({"error": "reset budget exhausted", "reason": "reset budget exhausted", "budget": RESET_BUDGET, "resets_left": 0})


def do_reset(q: dict):
    """Abandon the running episode (it ends with reason "reset" unless it had ended already) and start a new one from the
    frozen start state. The journal keeps every episode's rows; RECORD (what is graded) restarts empty."""
    if not E["done"]: finish("reset")
    try:
        S.reset()
    except Exception:   # the simulator failed to start the new episode: the trial ends on a simulator error
        E["done"] = True; E["reason"] = "error"; log({"ev": "end", "reason": "error", "reset_failed": True, "control_steps": len(RECORD), **_journal_pos()}); write_final()
        raise
    E["resets"] += 1; E["ep_start"] = E["journal_rows"]; RECORD.clear()
    E["done"] = False; E["reason"] = None; E["t_end"] = None
    rec = Recorder(q["cams"], q["fps"], q["depth"]); rec.tick(force=True)
    E["calls"] += 1; RECF.flush()
    log({"ev": "reset", "call": E["calls"], "cameras": q["cams"], "control_steps": 0, "rows_total": E["rows"], **_journal_pos(),
         "state_sha": S.state_sha()})
    write_final()
    return {**meta_common(), "reset_id": E["calls"], "ok": True}, rec.parts(q["video"], q["fps"])


# ---- RCB_STATE_OPS: /save_state and /load_state. A snapshot is the simulator's whole integration state
# (mj_getState(mjSTATE_INTEGRATION): time, qpos, qvel, act, warm start, ctrl, applied forces, mocap, equality flags, user
# data, plugin state), the tooling's own per-episode state (current targets, step count, DexToolBench's goal tracker and
# goal marker) and a copy of the graded rows: loading it continues exactly as the saved episode would have.
_OWN = ("targets", "steps", "goal_index", "steps_on_goal", "failed")


def sim_save() -> dict:
    mj, m, d = S.mj, S.model, S.data
    spec = mj.mjtState.mjSTATE_INTEGRATION
    st = np.empty(mj.mj_stateSize(m, spec), np.float64); mj.mj_getState(m, d, st, spec)
    goal = (m.body_pos[S._goal].copy(), m.body_quat[S._goal].copy()) if hasattr(S, "_goal") else None
    return {"state": st, "own": {k: copy.deepcopy(getattr(S, k)) for k in _OWN if hasattr(S, k)}, "goal": goal}


def sim_load(snap: dict) -> None:
    mj, m, d = S.mj, S.model, S.data
    mj.mj_setState(m, d, snap["state"], mj.mjtState.mjSTATE_INTEGRATION)
    for k, v in snap["own"].items(): setattr(S, k, copy.deepcopy(v))
    if snap["goal"] is not None: m.body_pos[S._goal] = snap["goal"][0]; m.body_quat[S._goal] = snap["goal"][1]
    mj.mj_forward(m, d)            # positions, contacts and cameras of the loaded state (the next step recomputes them)


def _keep(h: str, snap) -> None:
    HANDLES.pop(h, None); HANDLES[h] = snap
    while len(HANDLES) > MAX_HANDLES: HANDLES.pop(next(iter(HANDLES)))


def check_ops(meta, path: str) -> dict:
    """A save's or load's checks, all of them, before anything changes (as check_reset's); a load's frames are parsed."""
    if _success() or E["stopping"] or E.get("finalizing"):
        raise Refused({"error": "trial finished: " + ("an episode succeeded" if _success() else "your time is up"), "reason": E["reason"]})
    if path == "/save_state":
        if E["done"]: raise Refused({"error": "the episode has ended: reset or load a saved state first", "reason": E["reason"]})
        return {}
    h = meta.get("handle") if isinstance(meta, dict) else None
    if not isinstance(h, str): raise Refused({"error": "load_state needs {\"handle\": \"s<k>\"} from an earlier /save_state"}, 400)
    if h not in HANDLES:
        raise Refused({"error": f"unknown handle {h!r}: not saved in this trial, or dropped (at most {MAX_HANDLES} are kept)", "handle": h})
    return {"handle": h}


def do_save(_q: dict):
    snap = (sim_save(), [r.copy() for r in RECORD])
    E["saves"] += 1; h = f"s{E['saves']}"; _keep(h, snap)
    E["calls"] += 1; RECF.flush()
    log({"ev": "save", "call": E["calls"], "handle": h, "control_steps": len(RECORD), "rows_total": E["rows"], **_journal_pos(),
         "state_sha": S.state_sha()})
    return {"handle": h, "step": len(RECORD), "calls": E["calls"], "rows": E["rows"], "control_steps": len(RECORD),
            "done": E["done"], "reason": E["reason"], **_reset_counters()}, None


def do_load(q: dict):
    """Abandon the running episode (reason "load" unless it had ended already) and start a new one at the saved state.
    The journal keeps every episode's rows; RECORD (what is graded) becomes the saved episode's rows."""
    if not E["done"]: finish("load")
    h = q["handle"]; snap = HANDLES[h]; _keep(h, snap)            # a load counts as a use
    try:
        sim_load(snap[0])
    except Exception:   # the simulator failed to start the new episode: the trial ends on a simulator error
        E["done"] = True; E["reason"] = "error"; log({"ev": "end", "reason": "error", "load_failed": True, "control_steps": len(RECORD), **_journal_pos()}); write_final()
        raise
    E["loads"] += 1; E["ep_start"] = E["journal_rows"]; RECORD[:] = [r.copy() for r in snap[1]]
    E["done"] = False; E["reason"] = None; E["t_end"] = None
    rec = Recorder(q["cams"], q["fps"], q["depth"]); rec.tick(force=True)
    E["calls"] += 1; RECF.flush()
    log({"ev": "load", "call": E["calls"], "handle": h, "cameras": q["cams"], "control_steps": len(RECORD), "rows_total": E["rows"],
         **_journal_pos(), "state_sha": S.state_sha()})
    write_final()
    return {**meta_common(), "ok": True, "handle": h}, rec.parts(q["video"], q["fps"])


def _replay_ops(R: np.ndarray, ops: list) -> None:
    """A resume with saves or loads (RCB_STATE_OPS): the whole journal from the frozen start, every reset, save and load
    at the journal row its checkpoint logged (ops: the completed calls' checkpoints, in order)."""
    rows, j = [], 0
    HANDLES.clear(); E["saves"] = E["loads"] = 0

    def apply(e):
        nonlocal rows
        if e["ev"] == "reset":
            S.reset(); rows = []
        elif e["ev"] == "save":
            E["saves"] += 1; _keep(e["handle"], (sim_save(), [r.copy() for r in rows]))
        else:
            snap = HANDLES[e["handle"]]; _keep(e["handle"], snap); sim_load(snap[0]); E["loads"] += 1; rows = [r.copy() for r in snap[1]]
    for i, row in enumerate(R):
        while j < len(ops) and int(ops[j]["journal_rows"]) <= i: apply(ops[j]); j += 1
        S.step(row); rows.append(row.copy())
    while j < len(ops): apply(ops[j]); j += 1
    RECORD[:] = rows


def finalize() -> None:
    """The verdict: the live episode's (docs/protocol/README.md H.4; owner, 2026-10-17). The graded episode (every
    control step since the last reset) is saved as the trajectory, and the task's rule is read off the live simulator:
    DexToolBench, every goal reached (the goal tracker ran on every executed control step, as the privileged replay runs
    it); Playground, the tooling's check() of the final state. Nothing is replayed to grade it. final.json is finalized
    before the video, which is one render pass over the trajectory from the frozen start and never changes the verdict."""
    with LOCK:
        A = None
        try:
            err = E["reason"] == "error"   # a simulator error ended the episode: the state after it is never judged
            A = np.asarray(RECORD, float).reshape(-1, N); live_sha = S.state_sha()
            live = {} if err else (S.progress() if KIND == "dtb" else S.check())
            np.savez(os.path.join(DATA, "trajectory.npz"), actions=A, tool=TOOL.__name__, meta=json.dumps({"source": "limited-mode episode", "reason": E["reason"],
                                                                                                          "episode": E["resets"] + 1, "reset_budget": RESET_BUDGET}))
            v = {"graded_by": "live", "replays": 0, "n_actions": int(len(A)), "live_state_sha": live_sha, "state_sha": live_sha,
                 "video": False, "video_pending": True, "video_source": "render"}
            if err:
                v.update(success=False, live_success=False)
            elif KIND == "dtb":
                ok = bool(live["done"])
                v.update(success=ok, live_success=ok, progress=round(float(live["progress"]), 4), goals_reached=int(live["reached"]),
                         n_goals=int(live["n_goals"]), failed_on_timeout=bool(live["failed"]), live_goals_reached=int(live["reached"]))
            else:
                v.update(success=bool(live["success"]), live_success=bool(live["success"]),
                         **{k: val for k, val in live.items() if k not in ("success", "state_sha", "steps") and isinstance(val, (int, float, bool))})
            if STATE_OPS:
                v["episodes_log"] = episodes_log(_events())
            # The verdict is final and on disk (finalized) before any video is rendered: the video never changes, hides or
            # delays it (a slow or failing render, or a collect hook that stops waiting, leaves the verdict as it is).
            E["verdict"] = v; E["finalized"] = True; write_final()
        except Exception as exc:  # noqa
            traceback.print_exc()
            if not E["finalized"]:        # no verdict was reached: the service itself failed (not the agent's 0)
                log({"ev": "finalize_error", "error": repr(exc)}); E["verdict"] = {"finalize_error": repr(exc)[:300]}
        if E["finalized"] and A is not None:
            try:   # the video: one pass over the trajectory from the frozen start (the live simulator, reset), rendered aside
                if hasattr(TOOL, "MAX_STEPS"): TOOL.MAX_STEPS = max(int(TOOL.MAX_STEPS), len(A))   # the privileged hand-in cap
                tmp = os.path.join(DATA, "replay.tmp.mp4")                 # a half-written replay.mp4 is never collected
                S.render_video(A, tmp, every=max(2, len(A) // 600), fps=25)
                os.replace(tmp, os.path.join(DATA, "replay.mp4"))
                if os.path.exists(os.path.join(DATA, "replay.tmp.last.png")): os.replace(os.path.join(DATA, "replay.tmp.last.png"), os.path.join(DATA, "replay.last.png"))
                E["verdict"]["video"] = True
            except Exception: traceback.print_exc()  # noqa -- the video is for people; it never decides the verdict
        E["finalized"] = True; E["video_done"] = True
        if isinstance(E.get("verdict"), dict) and "video_pending" in E["verdict"]: E["verdict"]["video_pending"] = False
        try: write_final()
        except Exception: traceback.print_exc()  # noqa


def finish(reason: str) -> None:
    if E["done"]: return
    E["done"] = True; E["reason"] = reason; E["t_end"] = time.time()
    log({"ev": "end", "reason": reason, "control_steps": len(RECORD), **_journal_pos()}); write_final()


def start_finalize() -> None:
    if E.get("finalizing"): return
    E["finalizing"] = True; threading.Thread(target=finalize, daemon=True).start()


class Hd(BaseHTTPRequestHandler):
    def log_message(self, *a): pass
    def _json(self, code, obj):
        b = json.dumps(obj).encode(); self.send_response(code); self.send_header("Content-Type", "application/json"); self.send_header("Content-Length", str(len(b))); self.end_headers(); self.wfile.write(b)
    def _rcb1(self, meta, parts):
        if rcb1.size(meta, parts) > MAX_RESP:   # known before anything is built: the body is built once
            for p in parts:
                if p[0].startswith(("frames/", "depth/")) and isinstance(p[2], rcb1.Frames): p[2].arrays.clear()
            parts = [p for p in parts if not (p[0].startswith("frames/") or p[0].startswith("depth/"))]; meta = {**meta, "degraded": "frames/depth dropped (response too large)"}
        body = rcb1.encode(meta, parts)
        self.send_response(200); self.send_header("Content-Type", "application/x-rcb1"); self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
    def _body(self):
        try: n = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError: raise ValueError("bad Content-Length") from None
        if n < 0: raise ValueError("bad Content-Length")   # read(-1) would read until the client hangs up, without any bound
        if n > MAX_BODY: raise ValueError("request too large")
        raw = self.rfile.read(n) if n else b""
        if raw[:4] == rcb1.MAGIC: return rcb1.decode(raw)
        b = json.loads(raw or b"{}")
        if not isinstance(b, dict): raise ValueError("body must be a JSON object or an RCB1 frame")
        return b, {}
    def do_GET(self):
        _touch()
        if self.path == "/spec": return self._json(200, SPEC)
        if self.path == "/status": return self._json(200, {"calls": E["calls"], "control_steps": len(RECORD), "done": E["done"], "reason": E["reason"], "elapsed_s": round(time.time() - E["t0"], 1), **_reset_counters(),
                                                           **({"state_queries": E["state_queries"]} if STATE_QUERY else {})})
        if STATE_QUERY and self.path == "/state/entities": return self._json(200, STATE_ENTS)
        return self._json(404, {"error": "no such endpoint"})
    def do_POST(self):
        _touch()
        if STATE_QUERY and self.path == "/state/query": return self._state_query()
        if self.path not in ("/submit", "/observe") + (("/reset",) if RESET_BUDGET else ()) + (("/save_state", "/load_state") if CAN_SAVE else ()): return self._json(404, {"error": "no such endpoint"})
        try: self._req = self._body()
        except Exception as exc: return self._json(413 if str(exc) == "request too large" else 400, {"error": str(exc)[:200]})
        with LOCK:
            log({"ev": "call_begin", "path": self.path, "call": E["calls"] + 1})
            try:
                return self._serve()
            finally:
                log({"ev": "call_end", "path": self.path})

    def _state_query(self):
        """RCB_STATE_QUERY: POST /state/query, under the lock between calls: reads the simulator (MjRead), takes no control
        step and leaves the clock and every counter but state_queries as they are; served only while the episode runs."""
        try: meta, _ = self._body()
        except Exception as exc: return self._json(413 if str(exc) == "request too large" else 400, {"error": str(exc)[:200]})
        if not isinstance(meta, dict): return self._json(400, {"error": "the body is {\"names\": [...], \"fields\": [...]}"})
        with LOCK:
            if E["done"] or E["stopping"] or E["finalized"]:
                return self._json(409, {"error": "the episode has ended: state queries are served only while it runs", "reason": E["reason"]})
            plan, err = plan_state_query(STATE_ENTS, meta)
            if err: return self._json(400, err)
            try:
                state = STATE_TABLE.query(plan); json.dumps(state, allow_nan=False)
            except Exception as exc:  # noqa -- the query only reads: an adapter error ends nothing
                traceback.print_exc(); log({"ev": "state_query_error", "error": repr(exc)[:300]})
                return self._json(500, {"error": "state query failed", "detail": repr(exc)[:200]})
            E["state_queries"] += 1
            log({"ev": "state_query", "n": E["state_queries"], "names": list(plan), "fields": sorted({f for fs in plan.values() for f in fs}),
                 "control_steps": len(RECORD)})
            return self._json(200, {"state": state, "control_steps": len(RECORD), "sim_time_s": round(len(RECORD) * CTRL_DT, 4),
                                    "state_queries": E["state_queries"]})

    def _serve(self):
        meta, parts = self._req
        if self.path == "/submit" and E["done"]:
            return self._json(409, {"error": "episode finished", "reason": E["reason"], **({"resets_left": _reset_counters()["resets_left"]} if RESET_BUDGET else {})})
        if self.path == "/reset":
            try: check_reset()
            except Refused as exc: return self._json(409, exc.body)
        if self.path in ("/save_state", "/load_state"):
            try: op = check_ops(meta, self.path)
            except Refused as exc: return self._json(exc.code, exc.body)
        try:      # the whole request is checked before any physics: whatever is wrong with it is a 400 and changes nothing
            if not isinstance(meta, dict) or not isinstance(parts, dict): raise ValueError("the request's meta must be a JSON object")
            q = {"/submit": parse_submit, "/observe": lambda m, _p: parse_observe(m), "/reset": lambda m, _p: parse_reset(m),
                 "/save_state": lambda m, _p: op, "/load_state": lambda m, _p: {**parse_reset(m), **op}}[self.path](meta, parts)
        except (ValueError, TypeError, OverflowError, AttributeError, KeyError, IndexError) as exc:
            return self._json(400, {"error": (str(exc) or type(exc).__name__)[:300]})
        try:
            out, blobs = {"/submit": do_submit, "/observe": do_observe, "/reset": do_reset, "/save_state": do_save, "/load_state": do_load}[self.path](q)
        except Exception as exc:  # noqa
            traceback.print_exc(); log({"ev": "error", "error": repr(exc)}); finish("error")   # PROTOCOL §4.6: a 500 ends the episode
            return self._json(500, {"error": "simulator error", "detail": repr(exc)[:200]})
        if blobs is None: return self._json(200, out)   # a JSON answer (/save_state)
        return self._rcb1(out, blobs)


def _end_from_signal():
    with LOCK:
        E["stopping"] = True                         # the agent's time is up: no new episode after this one
        if not E["done"]: finish("timeout")
    start_finalize()


def _resume_or_start() -> None:
    """Fresh start, or (after a crash: compose restarts the container) resume the SAME episode. The journal holds every
    executed control step; the log holds, per completed call, the step count and the state hash. Replaying the journal up
    to the last completed call rebuilds that exact state (MuJoCo on the CPU is bit-exact) and the hash is checked. A call
    in flight when the service died is lost as a whole; its caller got an error, not a result, so nothing is gained. No
    reset: the episode never goes back to its start. With a reset budget the journal holds every episode; a checkpoint
    carries where the current one begins (ep_start) and the resets before it, and only that episode is replayed (from the
    frozen start, as a reset begins it); a reset lost with its call never happened. More than MAX_RESUMES crashes end it; if every crash happened while
    serving a call, it counts as the agent's (sim_crashed_on_call), else as infrastructure (sim_crashed)."""
    ev = _events(); starts = [e for e in ev if e.get("ev") == "start"]
    if not starts:
        # The frozen start is checked when the episode begins (the verdict is the live episode's: no later replay checks
        # it): a scene that did not build to the instance's recorded hash is no trial of this task, before the agent is served.
        if START_SHA and S.state_sha() != START_SHA:
            _give_up("instance_mismatch", infra=True, instance_mismatch=1, start_sha=S.state_sha(), instance_sha=START_SHA,
                     error=f"the scene started at {S.state_sha()}, the frozen instance records {START_SHA}")
        try: os.remove(REC_PATH)
        except OSError: pass
        log({"ev": "start", "tool": TOOL.__name__, "state_sha": S.state_sha(), "instance_sha": START_SHA or None, "protocol": PROTOCOL}); write_final()
        E["t0"] = time.time(); return
    try: fin = json.load(open(os.path.join(DATA, "final.json")))
    except Exception: fin = {}  # noqa
    if _graded(fin): _give_up("restarted after finalize", infra=False)
    resumes = [e for e in ev if e.get("ev") == "resumed"]
    in_call = bool(ev) and ev[-1].get("ev") == "call_begin"
    crashes_in_calls = sum(1 for e in resumes if e.get("crashed_in_call")) + int(in_call)
    if len(resumes) >= MAX_RESUMES:
        _give_up("sim_crashed_on_call" if crashes_in_calls == len(resumes) + 1 else "sim_crashed", infra=crashes_in_calls != len(resumes) + 1)
    done = [e for e in ev if e.get("ev") in ("submit", "observe", "reset") + (("save", "load") if STATE_OPS else ()) and "state_sha" in e]
    last = done[-1] if done else starts[0]
    n = int(last.get("journal_rows", last.get("control_steps", 0)))           # journal rows (all episodes) at that checkpoint
    ep0 = int(last.get("ep_start", 0)); resets = int(last.get("resets", 0))
    try: R = np.fromfile(REC_PATH, dtype=np.float64)
    except OSError: R = np.zeros(0)
    if R.size < n * N: _give_up("resume_failed: journal shorter than the log", infra=True)
    R = R[: n * N].reshape(n, N)
    with_ops = STATE_OPS and any(e.get("ev") in ("save", "load") for e in done)
    if with_ops:                                         # saves or loads: the whole journal, every op at its row
        _replay_ops(R, [e for e in done if e.get("ev") in ("reset", "save", "load")])
    else:
        for row in R[ep0:]:                              # the current episode, from the frozen start
            S.step(row); RECORD.append(row.copy())
    if S.state_sha() != last["state_sha"]: _give_up("resume_failed: replayed state differs from the logged one", infra=True)
    R.tofile(REC_PATH)                                   # drop the steps of the call that was in flight
    E["calls"] = int(last.get("call", 0)); E["rows"] = int(last.get("rows_total", 0)); E["t0"] = float(starts[0]["ts"])
    if STATE_QUERY: E["state_queries"] = sum(1 for e in ev if e.get("ev") == "state_query")   # each is logged before it is answered
    E["resets"] = resets; E["ep_start"] = ep0; E["journal_rows"] = n
    first = [e for e in ev if e.get("ev") == "first_request"]
    if first: E["t_first"] = float(first[0]["ts"])       # the agent's clock keeps running through a crash
    # the current episode's end, not the lost call's (past the checkpoint), not an earlier episode's, not a lost reset's
    ended = [e for e in ev if e.get("ev") == "end" and e.get("reason") != "reset" and int(e.get("resets", 0)) == resets
             and int(e.get("journal_rows", e.get("control_steps")) or 0) <= n]
    if STATE_OPS:   # the current episode's end: after the reset or load that began it, never a lost reset's or load's
        k0 = max((i for i, e in enumerate(ev) if e is last), default=0)
        b = max((i for i, e in enumerate(ev[:k0 + 1]) if e.get("ev") in ("reset", "load") and "state_sha" in e), default=-1)
        ended = [e for e in ev[b + 1:] if e.get("ev") == "end" and e.get("reason") not in ("reset", "load")
                 and int(e.get("journal_rows", e.get("control_steps")) or 0) <= n]
        E["saves"], E["loads"] = int(last.get("saves", E["saves"])), int(last.get("loads", E["loads"]))
    if ended: E["done"] = True; E["reason"] = ended[-1].get("reason"); E["t_end"] = float(ended[-1]["ts"])
    E["stopping"] = any(e.get("ev") == "end" and e.get("reason") == "timeout" for e in ev)   # the agent's time was up already
    log({"ev": "resumed", "control_steps": len(RECORD), "journal_rows": n, "resets": resets, "calls": E["calls"], "state_sha": S.state_sha(), "crashed_in_call": in_call,
         "downtime_s": round(time.time() - float(ev[-1]["ts"]), 1)})
    write_final()


def _bind(port: int, handler) -> HTTPServer:
    """The listening server. Under Harbor's egress control the agent's container shares this network namespace, so a
    process of the agent may hold the port when the service (re)starts. Wait for the port instead of failing: an
    exception here would count as a crash of the service (the collect hook's signal is still served meanwhile)."""
    said = False
    while True:
        try: return HTTPServer(("0.0.0.0", port), handler)
        except OSError as exc:
            if exc.errno != errno.EADDRINUSE: raise
        if not said: said = True; print(f"[sim] port {port} is in use (a process in the agent's container?): waiting for it to be free", flush=True)
        time.sleep(1.0)


_resume_or_start()
signal.signal(signal.SIGUSR1, lambda *_: threading.Thread(target=_end_from_signal, daemon=True).start())
if _USR1["pending"]: threading.Thread(target=_end_from_signal, daemon=True).start()
RECF = open(REC_PATH, "ab")
E["ready"] = True
open(os.path.join(DATA, "server.pid"), "w").write(str(os.getpid()))
print(f"{PROTOCOL} MuJoCo sim ready ({TOOL.__name__})", flush=True)
_bind(int(os.environ.get("RCB_PORT", "8000")), Hd).serve_forever()
