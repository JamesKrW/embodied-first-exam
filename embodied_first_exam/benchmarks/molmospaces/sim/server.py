"""eai-standard/2.2 sim service for MolmoSpaces (docs/protocol/standard_mode.md, docs/benchmarks/molmospaces.md): the adapter
of the shared layer rcb_limited.py; the simulator is rcb_molmospaces.Episode (upstream MolmoSpaces, MuJoCo on the CPU).
HTTP surface: GET /spec, GET /status, POST /observe, POST /submit, POST /kinematics. Every call is logged to
/data/episode.jsonl; nothing about objects ever leaves this process.

The episode follows upstream's evaluation: one control step = one upstream Task.step; it ends when upstream's
is_done() says so (the benchmark's horizon), when the agent submits `done`, or when the agent's time is over (the
collect hook); success = upstream judge_success() on the final state, also when the task was met earlier and undone.

Crash recovery (rcb_limited.Episode, resume class `exact`: MuJoCo on the CPU repeats bit for bit in a fresh process):
every executed row is journaled; after a crash the service rebuilds the episode and replays the journal. When the
episode ends the service writes /data/trajectory.npz + /data/episode.json; the collect hook (finalize.sh) makes it exit
and writes final.json from episode.json.

The verdict is the live episode's (owner 2026-10-17): upstream's judge_success() on the state at the
episode's end, as upstream's evaluation and the privileged verifier's replay judge it; nothing replays it. The frozen
start is checked when the scene is built, before the agent is served: a start state whose hash is not the instance's
recorded t = 0 hash gives up as `instance_mismatch` (infra_error)."""

import math, os, sys, threading, time, traceback
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rcb_limited as L  # noqa: E402

DATA = os.environ.get("RCB_DATA", "/data")
INSTANCE = os.environ.get("RCB_INSTANCE", os.path.join(os.path.dirname(os.path.abspath(__file__)), "instance.json"))
WALL_CLOCK_S = float(os.environ.get("RCB_WALL_CLOCK_S", "3600"))
MAX_CHUNK_S = float(os.environ.get("RCB_MAX_CHUNK_S", "10"))
MAX_WATCH_S = float(os.environ.get("RCB_MAX_WATCH_S", "30"))
MAX_FRAMES = 1812  # /spec max_frames_per_call: images per call (cameras x frames, per-frame depth counted again)
UNITS = os.environ["RCB_ACTION_UNITS"]  # the row's units, as the instruction words them (compose, from families.json)
LOCK = threading.Lock()
PID_FILE = os.path.join(DATA, "server.pid")
E = {
    "ready": False,
    "step": 0,
    "calls": 0,
    "done": False,
    "reason": None,
    "success": False,
    "t0": time.time(),
    "error": None,
}

import rcb_molmospaces as M  # noqa: E402

INST = M.load_instance(INSTANCE)
EP = L.Episode(DATA, dim=INST["t0"]["action_dim"], resume="exact")
EP.startup()  # start count, back-off after failed starts, the startup watchdog until scene_built()
S = M.Episode(INST)
START_SHA = S.state_sha()  # before anything else touches the scene: the frozen start, checked here (not in a replay)
if START_SHA != INST["t0"].get("state_sha"):
    EP.give_up(
        f"instance_mismatch: the frozen start state did not rebuild here (start {START_SHA}, frozen {INST['t0'].get('state_sha')})",
        infra=True,
    )
ROWS: list = []  # every executed row (a resume replays them first): trajectory.npz
CTRL_HZ = 1.0 / S.dt
SUBSTEPS = int(round(S.dt / float(S.env.current_model.opt.timestep)))
CAMS = S.agent_cameras
CAM0 = {c: S.camera_info(c) for c in CAMS}
LIMITS = S.command_limits()
GROUPS = [
    ("joint_pos", S.joint_names, "measured joint positions of the controlled joints (rad; base x, y in m)"),
    ("joint_vel", [f"{j}_vel" for j in S.joint_names], "measured joint velocities (rad/s, m/s)"),
    (
        "joint_effort",
        [f"{j}_effort" for j in S.joint_names],
        "actuator forces of those joints (N m, N): what each drive demands; a joint with an effort_limit in robot_spec "
        "applies at most that",
    ),
    ("gripper_opening", [f"{g}_opening_m" for g in S.grippers], "distance between the fingertips (m)"),
    (
        "tcp_pose",
        [f"{g}_{k}" for g in S.grippers for k in ("x", "y", "z", "qw", "qx", "qy", "qz")],
        "tool centre point of each gripper: x y z (m) + quaternion wxyz, world frame",
    ),
]
PROPRIO_LAYOUT = [n for _, names, _ in GROUPS for n in names]
PROPRIO_GROUPS, _i = {}, 0
for _g, _names, _meaning in GROUPS:
    PROPRIO_GROUPS[_g] = [_i, len(_names), _meaning]
    _i += len(_names)

SPEC = {
    "protocol": L.PROTOCOL,
    "protocol_compat": list(L.COMPAT),
    "simulator": "molmospaces (molmo-spaces 0.2.9, MuJoCo 3.5.0)",
    "robot": INST["episode"]["robot"]["robot_name"],
    "task_text": INST["facts"]["goal"],
    "control_hz": CTRL_HZ,
    "physics_substeps": SUBSTEPS,
    "max_chunk_s": MAX_CHUNK_S,
    "max_watch_s": MAX_WATCH_S,
    "max_frames_per_call": MAX_FRAMES,
    "cameras": {
        c: {
            "rgb": True,
            "depth": True,
            "mounted": CAM0[c]["mount"]["type"],
            "resolution": CAM0[c]["image_size"],
            "intrinsic_cv": CAM0[c]["intrinsic_cv"],
            "mount": CAM0[c]["mount"],
        }
        for c in CAMS
    },
    "default_cameras": list(CAMS),
    "action_types": {
        "joint_position": {
            "dim": S.action_dim,
            "layout": S.layout,
            "limits": dict(zip(S.layout, LIMITS.tolist())),
            "units": UNITS,
            "semantics": "absolute targets of upstream's joint-position controllers, one per control step; rows `dt` "
            "apart (a multiple of the control period, at most 0.5 s), `hold` or `linear` between rows; "
            "values outside the limits are clipped and reported",
        },
        "done": {
            "dim": 0,
            "layout": [],
            "semantics": 'ends the episode now, in the current state (no physics); r.submit("done", [])',
        },
        "not_yet": ["ee_pose", "joint_delta", "joint_velocity", "ee_delta", "joint_torque"],
    },
    "proprio_layout": PROPRIO_LAYOUT,
    "proprio_groups": PROPRIO_GROUPS,
    "static": {},
    "world": {
        "frame": "the simulator's world frame of this house: z up; the robot's base pose is its own (proprio joint_pos "
        "for RB-Y1; robot_spec for the fixed Franka)",
        "up": "+z",
        "units": {"length": "m", "angle": "rad", "time": "s"},
        "quaternion_order": "wxyz",
        "camera_axes": "intrinsic_cv / extrinsic_cv: OpenCV camera axes (x right, y down, z forward); cam2world_gl: OpenGL axes",
    },
    "budget": {"reset": 0, "steps": None, "wall_clock_s": WALL_CLOCK_S},
    "max_control_steps": S.horizon,
    "episode": f"one episode, no reset (upstream's evaluation): it ends after {S.horizon} control steps (the benchmark's "
    f"{INST['facts']['horizon_s']} s horizon), when you submit `done`, or when your time is over; success is judged "
    "on the state at that moment, also when the task was met earlier and undone",
    "robot_spec": S.robot_spec(),
    "kinematics": S.kinematics_spec(),
    "depth": {
        "modes": list(L.DEPTH_MODES),
        "default": "none",
        "kind": "z-depth",
        "unit": "m",
        "invalid": 0,
        "max_m": L.DEPTH_MAX_M,
        "frames_encoding": "uint16 mm, zlib",
        "noise": L.DEPTH_NOISE,
    },
}
L.check_spec(SPEC)  # a spec that misses a 2.2 field or names a forbidden one never starts the episode
EP.scene_built()


def ctrl_steps() -> int:
    return S.steps


def proprio_row() -> np.ndarray:
    p = S.proprio()
    return np.concatenate([p[g] for g, _, _ in GROUPS]).astype(np.float32)


def grab(cams, want_depth: bool):
    return {c: S.render(c) for c in cams}, ({c: S.render_depth(c) for c in cams} if want_depth else {})


def extrinsics(cams) -> dict:
    return {c: np.asarray(S.camera_info(c)["extrinsic_cv"], dtype=np.float64) for c in cams}


def write_final() -> None:
    if L.final_verdict(DATA):  # finalize.sh's verdict, or a give-up after crashes: never overwritten
        return
    L._write_json(os.path.join(DATA, "final.json"), {**summary(), "finalized": False, "live_sha": E.get("live_sha")})


def summary() -> dict:
    n = int(E.get("episode_control_steps") or ctrl_steps())
    return {
        "success": bool(E["success"]),
        "steps": E["step"],
        "calls": E["calls"],
        "control_steps": n,
        "physics_steps": n * SUBSTEPS,
        "elapsed_s": round((E.get("t_end") or time.time()) - E["t0"], 1),
        "reason": E["reason"],
        "done": E["done"],
        "family": INST["family"],
        "protocol": L.PROTOCOL,
        "n_actions": len(ROWS),
        "start_sha": START_SHA,
        "graded_by": "live",
        "replays": 0,
        **EP.final_fields(),
    }


def finish(reason: str) -> None:
    """The episode is over: upstream's verdict on the final state, then the record for the collect hook."""
    if E["done"]:
        return
    E["done"] = True
    E["reason"] = reason
    E["t_end"] = time.time()
    E["episode_control_steps"] = ctrl_steps()
    try:
        # a simulator error raised by the agent's actions (reason "error") leaves a post-exception state: never judged
        E["success"] = False if reason == "error" else S.success()
    except Exception:  # noqa: BLE001
        traceback.print_exc()
    EP.log(
        {"ev": "end", "reason": reason, "step": E["step"], "control_steps": ctrl_steps(), "success": bool(E["success"])}
    )
    dump()


def dump() -> None:
    if E.get("dumped"):
        return
    E["dumped"] = True
    E["live_sha"] = S.state_sha()
    np.savez(
        os.path.join(DATA, "trajectory.npz"),
        actions=np.asarray(ROWS, dtype=np.float64).reshape(-1, S.action_dim),
        layout=np.array(S.layout),
        family=INST["family"],
    )
    L._write_json(os.path.join(DATA, "episode.json"), {**summary(), "live_sha": E["live_sha"],
                                                       "startup": EP.n_startups, "pid": os.getpid(),   # finalize.sh's late-exit check
                                                       **(OPS.fields(bool(E["success"]), E["reason"], EP.events(), E.get("t_first"),
                                                                     EP.rows) if OPS is not None else {})})
    write_final()


def ctrl_step(a, rec) -> bool:
    """One executed control step, journaled. True when the call must stop here."""
    S.step(a)
    ROWS.append(np.asarray(a, dtype=np.float64))
    stop = EP.journal(a)
    rec.tick()
    if S.done:
        finish("horizon")
    return E["done"] or stop


def frame_every(fps: int) -> int:
    """The Recorder's stride in control steps: round(control rate / video_fps)."""
    return max(1, int(round(CTRL_HZ / max(1, fps))))


def recorder(cams, fps: int, meta: dict) -> L.Recorder:
    return L.Recorder(
        cams,
        frame_every(fps),
        grab,
        proprio_row,
        extrinsics,
        ctrl_steps,
        depth=L.depth_mode(meta),
        clock_hz=CTRL_HZ,
        max_images=MAX_FRAMES,
    )


def frames_ok(steps: int, fps: int, cams, meta: dict) -> None:
    """The images the call's Recorder captures over `steps` control steps (its real stride, every camera, per-frame
    depth counted again), refused above max_frames_per_call before anything runs (a 400)."""
    L.check_frames(len(dict.fromkeys(cams)), L.frames_per_call(steps, frame_every(fps)), L.depth_mode(meta) in ("frames", "legacy"),
                   MAX_FRAMES)


def video_fps(meta: dict) -> int:
    fps = meta.get("video_fps")
    if fps is None:
        return 5
    if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not (1 <= fps <= 30):
        raise ValueError("video_fps must be in [1, 30]")
    return int(fps)


def cameras_of(meta: dict) -> list:
    cams = meta.get("cameras") or CAMS
    if not isinstance(cams, (list, tuple)) or not all(isinstance(c, str) for c in cams):
        raise ValueError(f"cameras must be a list of camera names; cameras: {CAMS}")
    bad = [c for c in cams if c not in CAMS]
    if bad:
        raise ValueError(f"unknown cameras {bad}; cameras: {CAMS}")
    return list(cams)


def number(meta: dict, key: str, default: float) -> float:
    """A numeric request field: `default` when the request has no such field; a finite number otherwise, else a 400 (a
    null, a string, a list, NaN/inf). Every field is checked before anything moves (PROTOCOL §4.6): a malformed request
    never becomes an exception past the checks, which would end the episode."""
    if key not in meta:
        return float(default)
    v = meta[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f"{key} must be a finite number, got {v!r:.80}")
    return float(v)


def note_of(meta: dict) -> str:
    note = meta.get("note")
    if note is not None and not isinstance(note, str):
        raise ValueError("note must be a string")
    return (note or "")[:500]


def meta_common(cams) -> dict:
    n = ctrl_steps()
    ci = {c: S.camera_info(c) for c in cams}
    return {
        "step": E["step"],
        "calls": E["calls"],
        "control_steps": n,
        "physics_steps": n * SUBSTEPS,
        "sim_time_s": round(n * S.dt, 3),
        "time_s": round(L.agent_time(E), 2),
        "time_left_s": max(0.0, round(WALL_CLOCK_S - L.agent_time(E), 1)),
        "proprio": {"layout": PROPRIO_LAYOUT, "now": proprio_row().tolist()},
        "done": E["done"],
        "reason": E["reason"],
        "cameras": {
            c: {k: ci[c][k] for k in ("intrinsic_cv", "extrinsic_cv", "cam2world_gl", "image_size")} for c in cams
        },
    }


def do_submit(meta: dict, A: np.ndarray):
    typ = meta.get("type")
    if not isinstance(typ, str):
        raise ValueError(f"type must be joint_position or done (not yet: {SPEC['action_types']['not_yet']})")
    if typ == "done":
        rec = recorder(cameras_of(meta), 5, meta)
        rec.tick(force=True, last=True)  # every response carries the last frames
        E["calls"] += 1
        finish("done")
        return {
            **meta_common(cameras_of(meta)),
            "chunk_id": E["calls"],
            "type": typ,
            "executed_rows": 0,
            "ok": True,
        }, rec.parts("none", 5)
    if typ != "joint_position":
        raise ValueError(f"type must be joint_position or done (not yet: {SPEC['action_types']['not_yet']})")
    dt = number(meta, "dt", S.dt)
    interp = meta.get("interpolate", "linear")
    video = meta.get("video", "mp4")
    cams, fps, note = cameras_of(meta), video_fps(meta), note_of(meta)
    k = dt / S.dt
    if A.ndim != 2 or A.shape[1] != S.action_dim:
        raise ValueError(f"joint_position actions must be (T, {S.action_dim}), got {list(A.shape)}")
    if not np.all(np.isfinite(A)):
        raise ValueError("actions contain NaN/inf")
    if not (dt <= 0.5 + 1e-9 and round(k) >= 1 and abs(k - round(k)) < 1e-6):
        raise ValueError(f"dt must be a whole multiple of the {S.dt:g} s control period and at most 0.5 s; got {dt}")
    if A.shape[0] * dt > MAX_CHUNK_S + 1e-9:
        raise ValueError(f"chunk longer than max_chunk_s={MAX_CHUNK_S}: {A.shape[0]} rows x {dt} s")
    if not isinstance(interp, str) or interp not in ("hold", "linear"):
        raise ValueError("interpolate must be hold|linear")
    if not isinstance(video, str) or video not in ("mp4", "frames", "both", "none"):
        raise ValueError("video must be mp4|frames|both|none")
    frames_ok(A.shape[0] * int(round(k)), fps, cams, meta)
    rec, clipped = recorder(cams, fps, meta), []
    E["calls"] += 1
    step0, before = E["step"], ctrl_steps()
    rec.tick(force=True)
    prev = S.hold_row()
    for ri, row in enumerate(A):
        E["step"] += 1
        target = np.clip(row, LIMITS[:, 0], LIMITS[:, 1])
        for j in np.flatnonzero(target != row)[:20]:
            clipped.append(
                {"row": ri, "index": int(j), "reason": f"{S.layout[j]} limit [{LIMITS[j, 0]:g}, {LIMITS[j, 1]:g}]"}
            )
        n = int(round(k))
        stop = False
        for s in range(1, n + 1):
            a = target if interp == "hold" else prev + (target - prev) * (s / n)
            if ctrl_step(a, rec):
                stop = True
                break
        prev = target
        if stop:
            break
    rec.tick(force=True, last=True)
    out = {
        **meta_common(cams),
        "chunk_id": E["calls"],
        "type": typ,
        "executed_rows": E["step"] - step0,
        "control_steps_chunk": ctrl_steps() - before,
        "physics_steps_chunk": (ctrl_steps() - before) * SUBSTEPS,
        "clipped": clipped[:200],
        "n_clipped": len(clipped),
        "plan": [],
        "ok": True,
    }
    EP.log(
        {
            "ev": "submit",
            "call": E["calls"],
            "rows": E["step"] - step0,
            "dt": dt,
            "interpolate": interp,
            "cameras": cams,
            "control_steps": ctrl_steps() - before,
            "n_clipped": len(clipped),
            "note": note,
            **({"stopped": EP.stop_reason} if EP.stopping else {}),
        }
    )
    return out, rec.parts(video, fps)


def do_observe(meta: dict):
    cams, fps = cameras_of(meta), video_fps(meta)
    seconds = number(meta, "seconds", 0.0)
    video = meta.get("video", "frames")
    if seconds < 0 or seconds > MAX_WATCH_S:
        raise ValueError(f"seconds must be in [0, {MAX_WATCH_S}]")
    if not isinstance(video, str) or video not in ("mp4", "frames", "both", "none"):
        raise ValueError("video must be mp4|frames|both|none")
    frames_ok(int(round(seconds / S.dt)), fps, cams, meta)
    rec, before = recorder(cams, fps, meta), ctrl_steps()
    if seconds > 0 and not E["done"]:
        rec.tick(force=True)
        hold = S.hold_row()
        for _ in range(int(round(seconds / S.dt))):
            if ctrl_step(hold, rec):
                break
    rec.tick(force=True, last=True)
    E["calls"] += 1
    # what was watched: whole control steps (the request rounded to the control period), fewer if the episode ended
    ran = round((ctrl_steps() - before) * S.dt, 6)
    out = {
        **meta_common(cams),
        "obs_id": E["calls"],
        "seconds": ran,
        "control_steps_watch": ctrl_steps() - before,
        "physics_steps_watch": (ctrl_steps() - before) * SUBSTEPS,
        "ok": True,
    }
    EP.log(
        {
            "ev": "observe",
            "call": E["calls"],
            "seconds": ran,
            "seconds_requested": seconds,
            "cameras": cams,
            "control_steps": ctrl_steps() - before,
            **({"stopped": EP.stop_reason} if EP.stopping else {}),
        }
    )
    return out, rec.parts(video if seconds > 0 else ("frames" if video == "mp4" else video), fps)


def status() -> dict:
    return {
        "step": E["step"],
        "calls": E["calls"],
        "control_steps": ctrl_steps(),
        "done": E["done"],
        "reason": E["reason"],
        "elapsed_s": round(time.time() - E["t0"], 1),
    }


def end_from_signal() -> None:
    """The collect hook's SIGUSR1: end the episode (the agent's time is over), dump it, exit (the hook then writes final.json)."""
    with LOCK:
        E["exiting"] = True
        if not E["done"]:
            finish("timeout")
        EP.log({"ev": "exit", "reason": "collect hook: episode dumped"})
        if not os.path.exists(PID_FILE):
            open(PID_FILE, "w").write(str(os.getpid()))
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


def replay_row(row) -> None:
    """A journaled row during a resume."""
    S.step(row)
    ROWS.append(np.asarray(row, dtype=np.float64))


# ---------------------------------------------------------------- ablations (rcb_limited "ablations"; off unless the task sets them)
# RCB_STATE_QUERY=1: GET /state/entities and POST /state/query (rcb_limited) on state_table(): the privileged state in
# the house's world frame (spec.world, quaternions w x y z): the robot's root body and every joint, the tcps, the
# house's free bodies, its articulated furniture and its static parts. A query reads a scratch copy of the live state
# (MjRead), under the service lock between calls; it takes no control step.
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


def _short_joints(f: dict) -> dict:
    """joint_* fields keyed by the joint's short name (the proprio and robot_spec name, no `robot_0/` prefix)."""
    out = dict(f)
    for k in ("joint_pos", "joint_vel", "joint_range"):
        if k in f:
            unit, get = f[k]
            out[k] = (unit, lambda get=get: {M._short(n): v for n, v in get().items()})
    return out


def state_table():
    """The state-query ablation's table (RCB_STATE_QUERY=1), built once after the scene, in the house's world frame
    (spec.world: the frame of proprio tcp_pose, /kinematics and the cameras). Robot `robot`: the robot's root body
    (RB-Y1's mobile base, the Franka's fixed base): pose, velocity and every joint of the robot
    with its range; links: each gripper's tcp (the robot_spec tcp frame, named as the gripper group), the RB-Y1's torso
    and head; objects: every free body of the house (pose, velocity, size, bbox); articulations: each piece of
    furniture with hinged or sliding parts (its pose, size, bbox and those joints); scene features: the other static
    top-level bodies (rooms, walls, counters, tables, ...): pose, size and bbox. Read by MjRead (a scratch copy of the
    live state)."""
    import mujoco

    m = S.env.current_model
    R = MjRead(mujoco, lambda: S.env.current_model, lambda: S.env.current_data).refresh()
    name = lambda b: m.body(b).name  # noqa: E731
    t = MjStateTable(L.StateTable(
        frame="the simulator's world frame of this house (spec.world; metres, z up): the frame of proprio tcp_pose, "
              "/kinematics and the cameras' extrinsics; RB-Y1's proprio base_x / base_y / base_theta are its base "
              "pose in this frame, the Franka's robot_spec base.pose is its fixed base here", quaternion_order="wxyz",
        note="a pose is a body's or site's frame origin (MuJoCo xpos / xquat); velocities are world-frame, at that "
             "origin; object names are the MuJoCo body names (the privileged twin's state() keys)"), R)
    rv = S.robot.robot_view
    root = rv.base.root_body_id
    root = int(getattr(root, "id", root))
    while int(m.body_parentid[root]) != 0:
        root = int(m.body_parentid[root])
    robot = R.subtree(root)
    rjoints = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in robot]
    t.add("robot", "robot", _short_joints({**mj_body_fields(R, root),
                                           **mj_joint_fields(R, rjoints, ranges=True)}),
          note=f"{SPEC['robot']}: root body {M._short(name(root))!r} "
               f"({'mobile base' if 'base' in S.groups else 'fixed base'}); joints by their proprio names")
    for g in S.grippers:
        site = int(rv.get_move_group(g).leaf_frame_id)
        t.add(g, "robot_link", mj_site_fields(R, site),
              note=f"the {g} tcp: site {M._short(m.site(site).name)!r} (proprio tcp_pose, /kinematics frame {g!r})")
    for b in sorted(robot):
        short = M._short(name(b))
        if short in ("link_torso_5", "link_head_2"):
            t.add("torso" if "torso" in short else "head", "robot_link", mj_body_fields(R, b), note=f"body {short!r}")
    free = {int(m.jnt_bodyid[j]) for j in range(m.njnt) if int(m.jnt_type[j]) == 0} - robot
    for b in sorted(free):
        t.add(name(b) or f"body{b}", "object", mj_body_fields(R, b, size=True, bbox=True), note="a free body of the house")
    inside = set()
    for b in free:
        inside |= R.subtree(b)
    for b in range(1, m.nbody):
        if int(m.body_parentid[b]) != 0 or b in robot or b in inside or int(m.body_jntnum[b]):
            continue
        sub = R.subtree(b)
        js = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in sub and int(m.jnt_type[j]) in (2, 3)]
        if js:
            t.add(name(b) or f"body{b}", "articulation",
                  {**mj_body_fields(R, b, vel=False, size=True, bbox=True), **mj_joint_fields(R, js, ranges=True)},
                  note="furniture with moving parts: pose, size and bbox of its static part, and its joints")
        elif any(m.body_geomnum[k] for k in sub):
            t.add(name(b) or f"body{b}", "scene_feature", mj_body_fields(R, b, vel=False, size=True, bbox=True),
                  note="a static part of the house")
    return t


STATE_TABLE = state_table() if L.STATE_QUERY else None


# RCB_STATE_OPS=1: save = MuJoCo's full integration state (mj_getState, mjSTATE_INTEGRATION: time, qpos, qvel, act,
# warm start, ctrl, applied forces, mocap, eq_active, user data) plus the plain-data attributes of upstream's task, env,
# robot and controllers (episode_step_count, controller targets, ...) and the random generators, and the graded rows;
# load = mj_setState + mj_forward (derived quantities for the answer's frames) and the rest restored; exact for the
# physics that follows (MuJoCo on the CPU). reset = load of the snapshot taken at the frozen start, before any row.
def _plain(v, depth: int = 0) -> bool:
    if v is None or isinstance(v, (bool, int, float, str, np.ndarray, np.generic, np.random.Generator, np.random.RandomState)):
        return True
    if depth >= 3:
        return False
    if isinstance(v, (list, tuple, set, frozenset)) or type(v).__name__ == "deque":
        return all(_plain(x, depth + 1) for x in v)
    if isinstance(v, dict):
        return all(isinstance(k, (str, int)) and _plain(x, depth + 1) for k, x in v.items())
    return False


def _py_parts() -> list:
    return [S.task, S.env, S.robot] + list(getattr(S.robot, "controllers", {}).values())


def ops_save():
    import copy, random
    import mujoco
    m, d = S.env.current_model, S.env.current_data
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    st = np.empty(mujoco.mj_stateSize(m, spec), dtype=np.float64)
    mujoco.mj_getState(m, d, st, spec)
    return {"mj": st, "py": [{k: copy.deepcopy(v) for k, v in vars(o).items() if _plain(v)} for o in _py_parts()],
            "np_random": np.random.get_state(), "random": random.getstate(), "rows": [r.copy() for r in ROWS]}


def ops_load(snap) -> None:
    import copy, random
    import mujoco
    m, d = S.env.current_model, S.env.current_data
    mujoco.mj_setState(m, d, snap["mj"].copy(), mujoco.mjtState.mjSTATE_INTEGRATION)
    for o, saved in zip(_py_parts(), snap["py"]):
        for k, v in saved.items():
            setattr(o, k, copy.deepcopy(v))
    np.random.set_state(snap["np_random"]); random.setstate(snap["random"])
    mujoco.mj_forward(m, d)
    ROWS[:] = [r.copy() for r in snap["rows"]]
    for k in ("episode_control_steps", "dumped", "live_sha"):
        E.pop(k, None)
    for f in ("episode.json", "trajectory.npz"):   # the ended episode's record is not the new episode's
        try:
            os.remove(os.path.join(DATA, f))
        except OSError:
            pass


S0 = ops_save() if L.STATE_OPS else None        # the frozen start (checked above), before a resume replays anything
OPS = L.StateOps(budget=max(0, int(os.environ.get("RCB_RESET_BUDGET") or 0)), reset=lambda: ops_load(S0), save=ops_save,
                 load=ops_load) if L.STATE_OPS else None


resumed = EP.begin(
    state=E,
    step=replay_row,
    state_sha=S.state_sha,
    proprio=proprio_row,
    spec=SPEC,
    on_end=end_from_signal,
    start_event={"family": INST["family"], "instance": INST["instance"], "cameras": CAMS},
    **({"ops": OPS} if OPS is not None else {}),
)
if E["done"]:
    dump()  # resumed after the episode had ended: the record is rewritten from the replay
if resumed and not E["done"]:
    # the episode goes on: an episode.json from a call the crash lost (it ended there, then was rolled back to the last
    # checkpoint) is not this episode's record; finalize.sh must never take it for one
    for f in ("episode.json", "trajectory.npz"):
        try:
            os.remove(os.path.join(DATA, f))
        except OSError:
            pass
E["ready"] = True
write_final()
open(PID_FILE, "w").write(str(os.getpid()))
print(f"{L.PROTOCOL} MolmoSpaces sim ready (startup {EP.n_startups}, resumes {EP.resumes})", flush=True)
L.serve(
    L.Service(
        spec=SPEC,
        lock=LOCK,
        state=E,
        submit=do_submit,
        observe=do_observe,
        kinematics=S.kinematics,
        status=status,
        on_submit_error=lambda exc: finish("error"),
        log=EP.log,
        episode=EP,
        state_sha=S.state_sha,
        proprio=proprio_row,
        **({"state_ops": OPS} if OPS is not None else {}),
        state_table=STATE_TABLE if L.STATE_QUERY else None,
    ),
    port=int(os.environ.get("RCB_PORT", "8000")),
)
