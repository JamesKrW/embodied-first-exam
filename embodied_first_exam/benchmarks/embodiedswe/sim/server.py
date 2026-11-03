"""eai-standard/2.2 sim service for EmbodiedSWE (robobench on Isaac Lab 2.3.2 / Isaac Sim 5.1, GPU PhysX; see
docs/protocol/standard_mode.md and tasks/embodiedswe/README.md "Standard mode"). Owns the scene. HTTP surface
(rcb_limited): GET /spec, GET /status, POST /observe, POST /submit, POST /kinematics (RCB1 bodies and responses).

The scene is the privileged task's: EmbodiedSWE's own build of the preset from the same frozen instance (instance.json,
seed, GPU PhysX; rcb_embodiedswe.build), checked against the instance's recorded start state before anything is served
(the cameras are created before the env's one sim.reset() and change nothing of it). The task's grader (grader/<scene>.py,
EmbodiedSWE's rubric) is built on the env and records after every control step, as in the privileged rollout.

Actions (esw_std): only `ee_pose`, streamed: rows `dt` apart (a multiple of the control period, at most 0.5 s), linear
(slerp for quaternions) or hold between rows; every control step hands the robot EmbodiedSWE's native action for the
target of that step (Franka: the OSC pose target of panda_hand, its action smoothing inverted; G1: the native wrist
poses + hand targets [+ base command]). Between calls the last row stays in force (a watch holds it).

The episode: no reset. It ends at the end of the first call after which the task's success predicate holds (the
grader's check_success, the privileged verdict's predicate), when a tool-only rule is broken (spatula), when the
agent's time is up (the collect hook), on a simulator error, or at the runaway cap (rcb_embodiedswe.MAX_STEPS). The
verdict is read on the live state at the end (rcb_embodiedswe._final: success, progress = the rubric's score, the
tool-only record), no grading replay. GPU PhysX repeats a run bit for bit only in the same process history (the cameras
in the scene and this start-up's reads of the articulation change the later bits; drawing them does not): a replay in
another process is not bit-exact, so the video is filmed live (live_video.py) and crash recovery is class `approx` (the
journal holds the native action rows).

Per call, images are bounded by spec.max_frames_per_call (PROTOCOL §3 "Images per call": the frames really captured,
at the recorder's stride, per-frame depth counted again), checked before anything runs; the shared Recorder holds one
copy of the frames and the shared layer builds the body once.

Ablations (off unless the task sets them): RCB_STATE_QUERY=1 serves the state table (state_table(): the robot, its key
links, the scene's other bodies with their poses, velocities, sizes and boxes (articulations also their joints; a body of
several rigid parts is one entity), world frame, read on the main thread between calls);
RCB_STATE_OPS=1 serves /reset, /save_state and /load_state (a snapshot of the full simulator state and of this service's
per-episode state; see "the reset ablation" below)."""
from __future__ import annotations
import json, math, os, sys, threading, time, traceback
from typing import Callable
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rcb_limited as L  # noqa: E402
import live_video  # noqa: E402

PROTOCOL = L.PROTOCOL
SRV = os.path.dirname(os.path.abspath(__file__))
DATA = os.environ.get("RCB_DATA", "/data"); os.makedirs(DATA, exist_ok=True); os.makedirs(os.path.join(DATA, "calls"), exist_ok=True)
TASK_FILE = os.environ.get("RCB_TASK_FILE", os.path.join(SRV, "task.json")); TASK = json.load(open(TASK_FILE))
PRESET = TASK["preset"]
INSTANCE = os.environ.get("RCB_INSTANCE", os.path.join(os.path.dirname(TASK_FILE), "instance.json"))
GRADER = os.path.join(os.path.dirname(TASK_FILE), TASK["grader"])
MAX_CHUNK_S = float(os.environ.get("RCB_MAX_CHUNK_S", "10")); MAX_WATCH_S = float(os.environ.get("RCB_MAX_WATCH_S", "30"))
# images per call (cameras x frames, per-frame depth counted again): 240 RGB frames of 640x480 are 221 MB, so the largest
# legal response stays under the shared layer's 256 MiB body limit (rcb_limited.MAX_RESP) with its mp4s, and nothing is dropped
MAX_FRAMES = int(os.environ.get("RCB_MAX_FRAMES_PER_CALL", "240"))
NATIVE_DIM = {"franka.osc": 8, "bimanual_franka.osc": 16, "g1.pink_ik": 28, "g1.loco_pink_ik": 32}[PRESET.split(".", 2)[2]]
LOCK = threading.Lock(); PID_FILE = os.path.join(DATA, "server.pid")
E = {"ready": False, "step": 0, "calls": 0, "done": False, "reason": None, "success": False, "t0": time.time(), "error": None, "tgt": None}
EP = L.Episode(DATA, dim=NATIVE_DIM, resume="approx", persist=("tgt", "verdict"))   # the verdict: success stays success through a resume
OPS = None          # RCB_STATE_OPS: the reset ablation's StateOps (below)
EP.startup()
log = EP.log

# ---------------------------------------------------------------- scene
import rcb_embodiedswe as R  # noqa: E402
import esw_std as X  # noqa: E402
R.launch(cameras=True)
INST = R.load_instance(INSTANCE)
CAMS_OBJ = X.Cameras([])


def _building_env():
    """robobench's BaseEnv.__init__ under construction (the camera hook runs inside its sim.reset()): its scene, robot,
    room."""
    f = sys._getframe()
    while f is not None:
        s = f.f_locals.get("self")
        if f.f_code.co_name == "__init__" and s is not None and hasattr(s, "scene") and hasattr(s, "robot") and "room" in f.f_locals:
            return s.scene, s.robot, f.f_locals["room"]
        f = f.f_back
    raise RuntimeError("camera hook: the env under construction was not found")


def make_cameras():
    scene, robot, room = _building_env()
    import omni.usd
    CAMS_OBJ.views = X.views_before_reset(scene, robot, room, omni.usd.get_context().get_stage())
    return CAMS_OBJ.make()


# RCB_STATE_OPS: every grasp weld uses up one joint of a pool authored before the simulation starts (a reset or a load
# with a part held one more), so the reset ablation's pools hold GRASP_POOL_OPS joints per part (8 without it)
GRASP_POOL_OPS = 256


def _grasp_pools_for_ops() -> None:
    """Before the build: every robobench scene's grasp-weld pool at GRASP_POOL_OPS joints per part."""
    if R.BENCH_ROOT not in sys.path:
        sys.path.insert(0, R.BENCH_ROOT)
    import robobench
    robobench.discover()
    from robobench.core.scene import BaseScene
    from robobench.core.grasp_weld import GraspWeldContract

    def subclasses(c):
        for s in c.__subclasses__():
            yield s
            yield from subclasses(s)
    for cls in {GraspWeldContract, *subclasses(BaseScene)}:
        if isinstance(getattr(cls, "GRASP_POOL", None), int):
            cls.GRASP_POOL = max(int(cls.GRASP_POOL), GRASP_POOL_OPS)


if L.STATE_OPS:
    _grasp_pools_for_ops()
env, _ = R._build_with_camera(INST, make_cameras)
grader = R.load_grader(GRADER, PRESET)(env)
START = R.check_start(env, INST)
INSTANCE_CHECK = {k: START[k] for k in ("instance_sha", "instance_ok", "instance_max_abs_diff", "instance_within_tol", "instance_mismatch",
                                        "gpu_model", "instance_gpu")}
INSTANCE_CHECK["instance_exact"] = bool(START["instance_ok"]); INSTANCE_CHECK["instance_expected_sha"] = INST["state_sha"]
if START["instance_mismatch"]:
    L._write_json(os.path.join(DATA, "final.json"), {"task": PRESET, "protocol": PROTOCOL, **INSTANCE_CHECK})
    EP.give_up(f"instance_mismatch: the rebuilt scene (state {START['instance_sha']}) is not the frozen instance "
               f"({INST['state_sha']}) on the GPU model that froze it", infra=True)   # never returns
RIG = X.make_rig(env)
assert RIG.native_dim == NATIVE_DIM, (RIG.native_dim, NATIVE_DIM)
CTRL_HZ = RIG.ctrl_hz; SUBSTEPS = RIG.substeps; DIM = RIG.dim
CAMS_OBJ.aim(env)
TRAJ = R.Recorder(env, PRESET, int(INST["seed"]))
TOOL = getattr(env, "_esw_tool_only", None)


def ctrl_steps() -> int:
    return len(TRAJ.actions)


# ---------------------------------------------------------------- cameras: render once per control step
_R = {"step": -1}


def render() -> None:
    n = ctrl_steps()
    if _R["step"] != n:
        env.sim.render()
        for c in CAMS_OBJ.cams.values():
            c.update(env.dt)
        _R["step"] = n


# warm-up renders before the first frame: a new view's first frames come out noisy without the renderer's temporal
# history (measured: wheel_carry's pick and place views); rendering changes nothing of the physics
for _ in range(int(os.environ.get("RCB_WARMUP_RENDERS", "12"))):
    env.sim.render()
render()
CALIB = X.calibrate(CAMS_OBJ, RIG)          # intrinsics; static extrinsics; link_T_cam_cv of the mounted ones (measured)
CAMS = [v["name"] for v in CAMS_OBJ.views]
# the cameras and frame rate a call gets when it names none: the robot's own cameras and one static view (the scene's
# "front" or "wide" view, else "overview"), at the highest rate (at most 5 fps) at which the longest default call, a
# 30 s watch with raw frames, fits max_frames_per_call: every call legal at the defaults is answered in full
MOUNTED_CAMS = [v["name"] for v in CAMS_OBJ.views if v.get("link_prim")]
DEFAULT_CAMS = [next(c for c in ("front", "wide", "overview") if c in CAMS)] + MOUNTED_CAMS
DEFAULT_FPS = next((f for f in range(5, 0, -1) if len(DEFAULT_CAMS) * (int(round(MAX_WATCH_S * CTRL_HZ))
                    // max(1, int(round(CTRL_HZ / f))) + 2) <= MAX_FRAMES), 0)
if not DEFAULT_FPS:
    raise RuntimeError(f"no default video_fps fits max_frames_per_call = {MAX_FRAMES} with the cameras {DEFAULT_CAMS}")


def cam_world_T_cv(c: str) -> np.ndarray:
    m = CALIB[c]
    return m["world_T_cv"] if m["mount"]["type"] == "static" else RIG.link_T(m["parent_body"]) @ m["link_T_cv"]


def camera_info(cams) -> dict:
    out = {}
    for c in cams:
        Tw = cam_world_T_cv(c)
        out[c] = {"intrinsic_cv": CALIB[c]["K"].tolist(), "extrinsic_cv": np.linalg.inv(Tw)[:3].tolist(),
                  "cam2world_gl": (Tw @ X.GL2CV).tolist(), "image_size": [X.W, X.H]}
    return out


def grab(cams, want_depth: bool):
    render()
    rgb, dep = {}, {}
    for c in cams:
        out = CAMS_OBJ.cams[c].data.output
        rgb[c] = out["rgb"][0, ..., :3].to("cpu").numpy()
        if want_depth:
            dep[c] = out["distance_to_image_plane"][0].to("cpu").numpy()
    return rgb, dep


def extrinsics(cams) -> dict:
    return {c: np.linalg.inv(cam_world_T_cv(c))[:3] for c in cams}


def video_frame():
    """The live video's frame: every camera, half size, three to a row."""
    rgb, _ = grab(CAMS, False)
    tiles = [np.ascontiguousarray(rgb[c][::2, ::2]) for c in CAMS]
    cols = min(3, len(tiles))
    while len(tiles) % cols:
        tiles.append(np.zeros_like(tiles[0]))
    return np.concatenate([np.concatenate(tiles[i:i + cols], axis=1) for i in range(0, len(tiles), cols)], axis=0)


VIDEO = live_video.LiveVideo(DATA, ctrl_hz=CTRL_HZ, frame=video_frame, log=EP.log)

# ---------------------------------------------------------------- the spec
PLAY, PGROUPS = RIG.proprio_spec()
KIN = RIG.kinematics_spec()
CAM0 = camera_info(CAMS)
KIN_DEV = RIG.kin_check()
if KIN_DEV > 0.002:
    raise RuntimeError(f"kinematic model: FK of the measured joints is {KIN_DEV * 1000:.2f} mm from the simulator's wrist/hand")
STEP_S = 1.0 / CTRL_HZ
SPEC = {
    "protocol": PROTOCOL, "protocol_compat": list(L.COMPAT), "simulator": "embodiedswe (robobench on Isaac Lab 2.3.2 / Isaac Sim 5.1, GPU PhysX)",
    "robot": RIG.robot_spec()["name"], "task_text": TASK["text"] + ("\n\n" + TASK["success"] if TASK.get("success") else ""), "control_hz": CTRL_HZ, "physics_substeps": SUBSTEPS, "physics_hz": 1.0 / env.dt,
    "max_chunk_s": MAX_CHUNK_S, "max_watch_s": MAX_WATCH_S, "max_frames_per_call": MAX_FRAMES,
    "cameras": {c: {"rgb": True, "depth": True, "mounted": v["what"], "resolution": [X.W, X.H], "calibration_in_obs": True,
                    "intrinsic_cv": CAM0[c]["intrinsic_cv"], "mount": CALIB[c]["mount"], "depth_range_m": [0.01, 100.0]}
                for c, v in ((v["name"], v) for v in CAMS_OBJ.views)},
    "default_cameras": list(DEFAULT_CAMS),
    "budget": {"reset": 0, "steps": None, "wall_clock_s": float(os.environ.get("RCB_WALL_CLOCK_S", "14400")),
               "default_video_fps": DEFAULT_FPS, "images_per_call": (f"max_frames_per_call = {MAX_FRAMES} images per call (PROTOCOL §3 \"Images per call\"): cameras x the "
                     "frames really captured (the first, one every round(control_hz / video_fps) control steps, the last), a "
                     "per-frame depth one more image each; video mp4, frames and both capture the same frames, video none only "
                     "the first and the last. A call over it is a 400 before anything runs. Defaults: "
                     f"cameras {DEFAULT_CAMS}, video_fps {DEFAULT_FPS} (the longest call at the defaults fits)")},
    "action_types": {
        "ee_pose": {"dim": DIM, "layout": list(RIG.layout), "frame": "world (the simulator's world frame, metres; z up)", "quaternion": "wxyz",
                    "description": RIG.describe_action(),
                    "semantics": (f"targets streamed at the {CTRL_HZ:g} Hz control rate: rows `dt` apart (a multiple of {STEP_S:.6f} s, at most "
                                  "0.5 s; anything else is a 400, never rounded); between rows linear (positions, joint targets; quaternions "
                                  "slerped) or hold (`interpolate`); the first row starts from the target in force. Each control step hands "
                                  "the robot's own controller the target of that step: " + RIG.controller_text() + ". The last row stays in "
                                  "force after the call (a watch holds it). Joint targets go to the position drives as given (not clipped, "
                                  "as in EmbodiedSWE): a target past a joint's limit presses the joint against that limit (a firmer grip)."),
                    "joint_limits": RIG.action_limits()},
        "not_yet": ["joint_position", "joint_delta", "joint_velocity", "ee_delta", "joint_torque"],
        "note": "joint_position is not offered: these robots are driven through EmbodiedSWE's own task-space controllers (operational-space "
                "torque control, Pink IK), and a joint-position mode would be another controller; use /kinematics to plan in joint space"},
    "proprio_layout": PLAY, "proprio_groups": PGROUPS,
    "static": {},
    "world": {"frame": "the simulator's world frame (one environment, its origin at the world origin); z up; every pose, camera "
                       "extrinsic and ee_pose target is in it", "up": "+z", "units": {"length": "m", "angle": "rad", "time": "s"},
              "quaternion_order": "wxyz",
              "camera_axes": "intrinsic_cv / extrinsic_cv: OpenCV camera axes (x right, y down, z forward); cam2world_gl: OpenGL axes (x right, y up, z back)"},
    "max_control_steps": int(R.MAX_STEPS),
    "episode": ("one episode, no reset: it ends after the first call at whose end the task is complete (checked after every "
                "call), " + ("when the hand or a finger touches the bread (the tool-only rule: that disqualifies the episode), " if TOOL else "")
                + f"when your time is up, on a simulator error, or after {int(R.MAX_STEPS):,} control steps "
                f"({R.MAX_STEPS / CTRL_HZ / 3600:.1f} h of simulated time: a technical cap, reason \"step_guard\"); graded as it stands then"),
    "robot_spec": RIG.robot_spec(),
    "kinematics": KIN,
    "depth": {"modes": list(L.DEPTH_MODES), "default": "none", "kind": "z-depth", "unit": "m", "invalid": 0, "max_m": L.DEPTH_MAX_M,
              "frames_encoding": "uint16 mm, zlib", "noise": L.DEPTH_NOISE},
}
if L.STATE_OPS:
    SPEC["episode"] = ("episodes from the same starting state, POST /reset and /load_state (spec state_ops) begin a new one; an "
                       "episode ends after the first call at whose end the task is complete (checked after every call; this ends the "
                       "trial), " + ("when the hand or a finger touches the bread (the tool-only rule: that disqualifies the episode), "
                       if TOOL else "") + f"when your time is up, on a simulator error, or after {int(R.MAX_STEPS):,} control steps of "
                       "one episode (a technical cap, reason \"step_guard\"); only the last episode is graded, as it stands then. "
                       f"Each part's grasp welds: at most {GRASP_POOL_OPS} engagements in the trial (a reset or a load with the part "
                       "held is one more)")
L.check_spec(SPEC)
EP.scene_built()


# ---------------------------------------------------------------- the episode record
def write_final() -> None:
    if L.final_verdict(DATA):
        return
    n = ctrl_steps()
    L._write_json(os.path.join(DATA, "final.json"),
                  {"success": bool(E["success"]), "steps": E["step"], "calls": E["calls"], "control_steps": n, "physics_steps": n * SUBSTEPS,
                   "elapsed_s": round((E.get("t_end") or time.time()) - E["t0"], 1), "reason": E["reason"], "done": E["done"], "task": PRESET,
                   "protocol": PROTOCOL, "finalized": False, **EP.final_fields()})


def verdict() -> dict:
    """The privileged verdict on the live state (rcb_embodiedswe._final: success, progress, the tool-only record)."""
    try:
        return R._final(env, grader, INST)
    except Exception as exc:  # noqa: BLE001 -- the env broken by a failed step: success 0, progress = the rubric's peak
        traceback.print_exc()
        return R._unreadable_final(env, grader, exc)


def last_frame():
    """The live video's last frame, or None when it would need a render outside the main thread (the collect hook's
    SIGUSR1 runs the end in a thread, and a Kit render from there hangs): after a call the cameras already show the end
    state; only a resumed episode (replayed without rendering) ends without its last frame."""
    if _R["step"] != ctrl_steps() and threading.current_thread() is not threading.main_thread():
        return None
    return video_frame()


def dump() -> None:
    if E.get("dumped"):
        return
    E["dumped"] = True
    V = dict(E.get("verdict") or verdict())
    # The live episode's ending decides: after a crash and an (approx) resume the verdict comes back from the checkpoint;
    # an episode that ended in success stays a success even when its replayed state reads otherwise, and a tool contact
    # (the disqualification) wins over everything
    if E.get("reason") == "success" and not V.get("success"):
        V.update({"success": True, "success_restored": True})
    if E.get("reason") == "tool_contact" or (TOOL is not None and TOOL.touched):
        V.update({"success": False})
    E["success"] = bool(V.get("success"))
    try:
        TRAJ.frame()
    except Exception as exc:  # noqa: BLE001
        log({"ev": "final_frame_error", "error": repr(exc)[:300]})
    try:
        TRAJ.save(os.path.join(DATA, "trajectory.npz"), label="standard", final_sha=V.get("final_sha") or "")
    except Exception as exc:  # noqa: BLE001
        traceback.print_exc(); log({"ev": "dump_error", "error": repr(exc)})
    n = ctrl_steps()
    L._write_json(os.path.join(DATA, "episode.json"),
                  {**{k: v for k, v in V.items() if k != "final_sha"}, "success": bool(V.get("success")), "live_sha": V.get("final_sha"),
                   "steps": E["step"], "calls": E["calls"], "control_steps": n, "physics_steps": n * SUBSTEPS,
                   "elapsed_s": round((E.get("t_end") or time.time()) - E["t0"], 1), "reason": E["reason"], "task": PRESET, "protocol": PROTOCOL,
                   "n_actions": n, "success_seen_step": E.get("success_step"), **INSTANCE_CHECK, **EP.final_fields(),
                   **({"infra_error": 1} if E["reason"] == "infra_error" else {}),
                   **(OPS.fields(bool(V.get("success")), E["reason"], EP.events(), E.get("t_first"), EP.rows) if OPS is not None else {}),
                   **({"restore_sha_mismatches": int(E.get("restore_sha_mismatches") or 0)} if OPS is not None else {})})
    write_final()
    VIDEO.last(vstep(), frame=last_frame)


def finish(reason: str) -> None:
    if E["done"]:
        return
    E["done"] = True; E["reason"] = reason; E["t_end"] = time.time()
    E["verdict"] = verdict(); E["success"] = bool(E["verdict"].get("success"))
    log({"ev": "end", "reason": reason, "step": E["step"], "control_steps": ctrl_steps(), "elapsed_s": round(E["t_end"] - E["t0"], 1),
         "success": E["success"], "progress": E["verdict"].get("progress")})
    dump()


def check_success() -> bool:
    if TOOL is not None and TOOL.touched:
        return False
    return bool(grader.check_success()[0])


# ---------------------------------------------------------------- request fields (a 400 before anything runs)
def number(meta: dict, key: str, default: float) -> float:
    v = meta.get(key)
    if v is None:
        return float(default)
    if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
        raise ValueError(f"{key} must be a finite number")
    return float(v)


def choice(meta: dict, key: str, default: str, allowed: tuple) -> str:
    v = meta.get(key)
    if v is None:
        return default
    if not isinstance(v, str) or v not in allowed:
        raise ValueError(f"{key} must be {'|'.join(allowed)}")
    return v


def cameras(meta: dict) -> list:
    v = meta.get("cameras")
    if v is None:
        return list(DEFAULT_CAMS)
    if isinstance(v, str):
        v = [v]
    if not isinstance(v, (list, tuple)) or not v or not all(isinstance(c, str) for c in v):
        raise ValueError(f"cameras must be a non-empty list of camera names ({', '.join(CAMS)})")
    bad = [c for c in v if c not in CAMS]
    if bad:
        raise ValueError(f"unknown camera(s) {bad}: the cameras are {', '.join(CAMS)}")
    return list(dict.fromkeys(v))


def video_fps(meta: dict) -> int:
    fps = meta.get("video_fps")
    if fps is None:
        return DEFAULT_FPS
    if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not (1 <= fps <= 30):
        raise ValueError("video_fps must be in [1, 30]")
    return int(fps)


def note_text(meta: dict) -> str:
    v = meta.get("note")
    if v is None:
        return ""
    if not isinstance(v, str):
        raise ValueError("note must be a string")
    return v[:500]


def as_400(parse: Callable, *args):
    try:
        return parse(*args)
    except ValueError:
        raise
    except Exception as exc:  # noqa: BLE001 -- nothing executed yet: a bad request, not a simulator error
        raise ValueError(f"malformed request: {exc!r}"[:300]) from None


def every_of(fps: int, video: str = "frames") -> int:
    """The recorder's stride in control steps; video "none" captures only the call's first and last frame."""
    if video == "none":
        return 1 << 62
    return max(1, int(round(CTRL_HZ / max(1, fps))))


def frames_ok(cams, steps: int, fps: int, depth: str, video: str) -> None:
    """A 400 before anything runs for a call of `steps` control steps over max_frames_per_call (PROTOCOL §3): the frames
    the recorder captures at its stride (rcb_limited.frames_per_call), per-frame depth counted again."""
    n = L.frames_per_call(steps, every_of(fps, video)) if steps > 0 else 1
    L.check_frames(len(cams), n, depth in ("frames", "legacy"), MAX_FRAMES)


def recorder(cams, fps: int, video: str, depth: str) -> L.Recorder:
    """The shared Recorder; max_images is a guard only (every call is checked before it runs): past the budget it stops
    filming, keeps the last frame and counts frames_skipped."""
    return L.Recorder(cams, every_of(fps, video), grab, proprio_row, extrinsics, ctrl_steps, depth=depth, clock_hz=CTRL_HZ,
                      max_images=MAX_FRAMES)


def proprio_row() -> np.ndarray:
    return RIG.proprio()


def meta_common(cams) -> dict:
    n = ctrl_steps()
    return {"step": E["step"], "calls": E["calls"], "control_steps": n, "physics_steps": n * SUBSTEPS, "sim_time_s": round(n / CTRL_HZ, 4),
            "time_s": round(L.agent_time(E), 2), "time_left_s": max(0.0, round(SPEC["budget"]["wall_clock_s"] - L.agent_time(E), 1)),
            "proprio": {"layout": PLAY, "now": proprio_row().tolist()}, "done": E["done"], "reason": E["reason"], "cameras": camera_info(cams)}


def save_call_evidence(idx: int, meta: dict, rec: L.Recorder) -> None:
    """The call's record on disk (meta + last frames); a disk failure here costs the record, never the episode."""
    try:
        d = os.path.join(DATA, "calls", f"{idx:06d}"); os.makedirs(d, exist_ok=True)
        L._write_json(os.path.join(d, "meta.json"), L._jsonable(meta))
        from PIL import Image
        for c in rec.cams:
            if rec.frames[c] and rec.frames[c][-1] is not None:
                Image.fromarray(rec.frames[c][-1]).save(os.path.join(d, f"last_{c}.jpg"), quality=85)
    except Exception as exc:  # noqa: BLE001
        log({"ev": "evidence_error", "call": idx, "error": repr(exc)[:300]})


class InfraError(RuntimeError):
    """A host failure (disk, ffmpeg) after the call ran: the episode ends infra_error, not as the agent's sim error."""


def respond(out: dict, rec: L.Recorder, video: str, fps: int) -> list:
    """The response parts (the shared layer sizes and builds the body); when the frames/video cannot be built (an
    encoder or disk failure) the call still answers, degraded to what is left: extrinsics, proprio, the per-frame arrays."""
    if rec.skipped:
        out["frames_skipped"] = rec.skipped
    try:
        return rec.parts(video, fps)
    except Exception as exc:  # noqa: BLE001
        traceback.print_exc(); log({"ev": "response_error", "error": repr(exc)[:300]})
        out["degraded"] = f"the frames/video could not be built ({type(exc).__name__}): dropped from this response"
    try:
        return rec.parts("none", fps)
    except Exception as exc:  # noqa: BLE001
        raise InfraError(f"the response could not be built: {exc!r}"[:300]) from exc


def executing(fn: Callable) -> Callable:
    """A ValueError once the call has started executing is a simulator error (500), not a bad request (400)."""
    def run(*args):
        try:
            return fn(*args)
        except ValueError as exc:
            raise RuntimeError(f"during execution: {exc}") from exc
    return run


# ---------------------------------------------------------------- executing
def native_step(native: np.ndarray) -> None:
    """One control step of EmbodiedSWE's native action, as the privileged rollout executes it (recorded, then stepped,
    then the grader records)."""
    import torch
    a = np.asarray(native, dtype=np.float32)
    TRAJ.actions.append(a.copy())
    env.step(torch.as_tensor(a, device=env.device).reshape(1, -1))
    grader.record()
    h = grader.history  # bounded: the peak-progress record and the latest one (the verdict reads only the peak)
    if len(h) > 2:
        h[:] = [h[1] if h[1][0]["progress"] > h[0][0]["progress"] else h[0], h[-1]]
    TRAJ.after_step()


def ctrl_step(target: np.ndarray, rec: L.Recorder) -> bool:
    """One control step towards `target` (a full ee_pose row); journaled. True when the call must stop here."""
    native = RIG.native(target)
    native_step(native); stop = EP.journal(native); rec.tick(); VIDEO.offer(vstep())
    if TOOL is not None and TOOL.touched:
        finish("tool_contact")
    elif ctrl_steps() >= R.MAX_STEPS:
        finish("step_guard")
    return E["done"] or stop


def parse_submit(meta: dict, A: np.ndarray) -> tuple:
    typ = meta.get("type")
    if typ == "joint_position":
        raise ValueError("joint_position is not offered by this task (spec.action_types.note): use ee_pose; /kinematics gives FK and IK")
    if not isinstance(typ, str) or typ != "ee_pose":
        raise ValueError(f"type must be ee_pose (not offered: {SPEC['action_types']['not_yet']})")
    dt = number(meta, "dt", STEP_S); interp = choice(meta, "interpolate", "linear", ("hold", "linear"))
    cams = cameras(meta); fps = video_fps(meta); video = choice(meta, "video", "mp4", ("mp4", "frames", "both", "none"))
    note = note_text(meta); depth = L.depth_mode(meta)
    if A.ndim != 2 or A.shape[1] != DIM or A.shape[0] < 1:
        raise ValueError(f"ee_pose actions must be (T, {DIM}) with T >= 1, got {list(A.shape)}")
    if not np.all(np.isfinite(A)):
        raise ValueError("actions contain NaN/inf")
    if not (STEP_S - 1e-9 <= dt <= 0.5 + 1e-9):
        raise ValueError(f"dt must be in [{STEP_S:.6f}, 0.5] s")
    k = dt * CTRL_HZ
    if abs(k - round(k)) > 1e-6:
        raise ValueError(f"dt must be a multiple of the {STEP_S:.6f} s control period (k / {CTRL_HZ:g} s); got {dt}")
    k = int(round(k))
    if A.shape[0] * k * STEP_S > MAX_CHUNK_S + 1e-9:
        raise ValueError(f"chunk longer than max_chunk_s = {MAX_CHUNK_S}: {A.shape[0]} rows x {dt} s")
    RIG.check_rows(A)
    frames_ok(cams, A.shape[0] * k, fps, depth, video)
    return dt, k, interp, cams, fps, video, note, depth


def do_submit(meta: dict, A: np.ndarray) -> tuple[dict, list]:
    return executing(run_submit)(A, *as_400(parse_submit, meta, A))


def run_submit(A, dt, k, interp, cams, fps, video, note, depth) -> tuple[dict, list]:
    rec = recorder(cams, fps, video, depth); clipped: list = []
    E["calls"] += 1; step0 = E["step"]; before = ctrl_steps(); rec.tick(force=True)
    prev = np.asarray(E["tgt"], dtype=np.float64)
    for ri, row in enumerate(A):
        E["step"] += 1
        row = np.asarray(row, dtype=np.float64)
        stop = False
        for j in range(1, k + 1):
            t = row if interp == "hold" else X.interp_row(RIG, prev, row, j / k)
            E["tgt"] = [float(v) for v in t]
            if ctrl_step(t, rec):
                stop = True; break
        E["tgt"] = [float(v) for v in row] if not stop else E["tgt"]
        prev = row
        if stop:
            break
    rec.tick(force=True, last=True)
    if not E["done"] and check_success():
        E["success_step"] = ctrl_steps(); finish("success")
    out = {**meta_common(cams), "chunk_id": E["calls"], "type": "ee_pose", "executed_rows": E["step"] - step0,
           "control_steps_chunk": ctrl_steps() - before, "physics_steps_chunk": (ctrl_steps() - before) * SUBSTEPS,
           "clipped": clipped[:200], "n_clipped": len(clipped), "plan": [], "rejected": None, "ok": True}
    log({"ev": "submit", "call": E["calls"], "type": "ee_pose", "rows": E["step"] - step0, "dt": dt, "interpolate": interp, "cameras": cams,
         "note": note, "depth": depth, "control_steps": ctrl_steps() - before, "n_clipped": len(clipped), "done": E["done"],
         **({"stopped": EP.stop_reason} if EP.stopping else {})})
    save_call_evidence(E["calls"], {k_: v for k_, v in out.items() if k_ != "cameras"}, rec)
    return out, respond(out, rec, video, fps)


def parse_observe(meta: dict) -> tuple:
    cams = cameras(meta); seconds = number(meta, "seconds", 0.0); fps = video_fps(meta)
    video = choice(meta, "video", "frames", ("mp4", "frames", "both", "none")); depth = L.depth_mode(meta)
    if seconds < 0 or seconds > MAX_WATCH_S:
        raise ValueError(f"seconds must be in [0, {MAX_WATCH_S}]")
    steps = int(round(seconds * CTRL_HZ))
    frames_ok(cams, steps, fps, depth, video)
    return cams, seconds, steps, fps, video, depth


def do_observe(meta: dict) -> tuple[dict, list]:
    return executing(run_observe)(*as_400(parse_observe, meta))


def run_observe(cams, seconds, steps, fps, video, depth) -> tuple[dict, list]:
    rec = recorder(cams, fps, video, depth); before = ctrl_steps()
    if steps > 0 and not E["done"]:
        rec.tick(force=True)
        hold = np.asarray(E["tgt"], dtype=np.float64)
        for _ in range(steps):
            if ctrl_step(hold, rec):
                break
        rec.tick(force=True, last=True)
        if not E["done"] and check_success():
            E["success_step"] = ctrl_steps(); finish("success")
    else:
        rec.tick(force=True, last=True)
    E["calls"] += 1
    out = {**meta_common(cams), "obs_id": E["calls"], "seconds": seconds, "control_steps_watch": ctrl_steps() - before,
           "physics_steps_watch": (ctrl_steps() - before) * SUBSTEPS, "ok": True}
    log({"ev": "observe", "call": E["calls"], "seconds": seconds, "cameras": cams, "depth": depth, "control_steps": ctrl_steps() - before,
         **({"stopped": EP.stop_reason} if EP.stopping else {})})
    if steps > 0:
        save_call_evidence(E["calls"], {k: v for k, v in out.items() if k != "cameras"}, rec)
    return out, respond(out, rec, video if steps > 0 else ("frames" if video == "mp4" else video), fps)


def do_kinematics(req: dict) -> dict:
    return RIG.kinematics(req)


def status() -> dict:
    return {"step": E["step"], "calls": E["calls"], "control_steps": ctrl_steps(), "done": E["done"], "reason": E["reason"],
            "elapsed_s": round(time.time() - E["t0"], 1)}


def end_from_signal() -> None:
    with LOCK:
        E["exiting"] = True
        if not E["done"]:
            finish("timeout")
        else:
            dump()
        log({"ev": "exit", "reason": "collect hook: episode dumped (graded by the live episode), releasing the simulator"})
        if not os.path.exists(PID_FILE):
            open(PID_FILE, "w").write(str(os.getpid()))
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(0)


def replay_row(row) -> None:
    """One journaled native row during a resume (nothing rendered)."""
    native_step(np.asarray(row, dtype=np.float64)); RIG.after_replay(row)


# ---------------------------------------------------------------- the ablations (rcb_limited "ablations"; off unless the task
# sets them). RCB_STATE_QUERY=1: state_table() below.
STATE_KINDS = ("rigid_object", "articulation", "deformable_object")
_SIGNS = np.array([[i, j, k] for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)], dtype=np.float64)


class _ReadOnce(L.StateTable):
    """L.StateTable whose getters share one read of the scene's state per query (memo)."""

    def query(self, plan: dict) -> dict:
        self._memo = {}
        return super().query(plan)

    def memo(self, key, fn):
        if key not in self._memo:
            self._memo[key] = fn()
        return self._memo[key]


def _ground_z() -> float | None:
    """The world z of the scene's ground plane (a prim named like one), None without one."""
    try:
        import omni.usd
        from pxr import Usd, UsdGeom
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        for prim in omni.usd.get_context().get_stage().Traverse():
            n = prim.GetName().lower()
            if ("groundplane" in n or n in ("ground", "ground_plane", "defaultgroundplane")) and prim.IsA(UsdGeom.Xformable):
                return float(cache.GetLocalToWorldTransform(prim).ExtractTranslation()[2])
    except Exception:  # noqa: BLE001
        pass
    return None


def _rotm(q) -> np.ndarray:
    """wxyz -> 3x3."""
    w, x, y, z = np.asarray(q, np.float64) / max(float(np.linalg.norm(q)), 1e-12)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _corners(lo, hi) -> np.ndarray:
    lo, hi = np.asarray(lo, np.float64), np.asarray(hi, np.float64)
    return (lo + hi) / 2 + _SIGNS * (hi - lo) / 2


def _body_boxes(path: str) -> dict:
    """{rigid body prim name: (lo, hi) [3] m} of the rigid bodies at or below the prim at `path` (the prim itself when none
    of them has the rigid-body API): the box of the geometry each body carries (its geometric prims down to the next
    rigid body, their authored extents), along the body's own axes, in metres (every scale applied: a body's pose carries
    none). From the USD stage as authored at the start; {} where nothing is found."""
    try:
        import omni.usd
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        stage = omni.usd.get_context().get_stage()
        root = stage.GetPrimAtPath(path)
        if not root or not root.IsValid():
            return {}
        xc = UsdGeom.XformCache(Usd.TimeCode.Default())
        prims = list(Usd.PrimRange(root, Usd.TraverseInstanceProxies()))
        bodies = [p for p in prims if p.HasAPI(UsdPhysics.RigidBodyAPI)] or [root]
        owner = {p.GetPath(): [] for p in bodies}
        for g in prims:
            if not g.IsA(UsdGeom.Gprim):
                continue
            b = g
            while b and b.IsValid() and b.GetPath() not in owner and b.GetPath() != root.GetPath():
                b = b.GetParent()
            if not b or b.GetPath() not in owner:
                continue
            try:
                bound = UsdGeom.Boundable(g)
                attr = bound.GetExtentAttr()   # a mesh's authored extent; a shape's from its size / radius / height
                ext = attr.Get() if g.IsA(UsdGeom.Mesh) and attr.HasAuthoredValue() else None
                if ext is None or len(ext) != 2:
                    ext = UsdGeom.Boundable.ComputeExtentFromPlugins(bound, Usd.TimeCode.Default())
                if ext is None or len(ext) != 2 or any(float(ext[1][i]) < float(ext[0][i]) for i in range(3)):
                    continue
                m = xc.GetLocalToWorldTransform(g)
                owner[b.GetPath()] += [np.array(m.Transform(Gf.Vec3d(*map(float, c)))) for c in _corners(ext[0], ext[1])]
            except Exception:  # noqa: BLE001
                continue
        out = {}
        for b in bodies:
            pts = owner[b.GetPath()]
            if not pts:
                continue
            m = xc.GetLocalToWorldTransform(b)
            axes = np.array([[m[i][j] for j in range(3)] for i in range(3)], np.float64)
            axes /= np.linalg.norm(axes, axis=1, keepdims=True)          # row i: the body's axis i in the world
            local = (np.asarray(pts) - np.array(m.ExtractTranslation(), np.float64)) @ axes.T
            out[b.GetName()] = (local.min(0), local.max(0))
        return out
    except Exception:  # noqa: BLE001
        return {}


def _world_box(parts) -> list:
    """parts: [(pos, quat, body-frame corners [8, 3])] -> [[x, y, z] min, [x, y, z] max] of all of them, world axes."""
    pts = np.concatenate([np.asarray(p, np.float64) + c @ _rotm(q).T for p, q, c in parts])
    return [pts.min(0), pts.max(0)]


def _frame_size(parts, pos, quat) -> np.ndarray:
    """The full extents of parts [(pos, quat, corners)] along the axes of the frame (pos, quat)."""
    pts = np.concatenate([np.asarray(p, np.float64) + c @ _rotm(q).T for p, q, c in parts])
    local = (pts - np.asarray(pos, np.float64)) @ _rotm(quat)
    return local.max(0) - local.min(0)


def state_table() -> _ReadOnce:
    """The state-query ablation's table (RCB_STATE_QUERY=1), built once the scene is: the robot (root pose and velocities,
    the root's height above the ground plane, every joint with its range), its key links (Franka: panda_link0..7,
    panda_hand, the fingertip centre, the fingers; G1: pelvis, torso, head, the wrists, every hand link), and the other
    bodies of Isaac Lab's scene, by their get_state names: every rigid object (pose, velocities, size, bbox), every
    articulation (also its joint state and ranges; one without joints is an object), every deformable object (its nodes'
    centroid and box). A body made of several rigid parts is one entity: the food of a cutting scene (all of its pieces:
    the centre of mass, the first piece's orientation, the box of all of it) and a coffee machine's fixed housing (its
    collision parts). Not listed: a body with collision disabled (a marker) and the bodies parked in a scene's off-stage
    depot (the variants absent from this episode); a scene's one present variant of a family is named by the family. No
    grader value, progress or tool-only record is in it. World frame (/spec world). Read from Isaac Lab's data buffers
    (scene.get_state, as the call checkpoints' state hash reads it, and the articulations' body / joint buffers, as
    proprio reads them) on the main thread, between calls: nothing steps or renders; the geometry (size, the bodies'
    boxes) is read from the USD stage once, here."""
    import re
    t = _ReadOnce(frame="the simulator's world frame (/spec world: one environment, its origin at the world origin), z up; the "
                        "frame of every pose, camera extrinsic and ee_pose target of this service",
                  quaternion_order="wxyz",
                  note="size = the full extents along the entity's own axes (pos / quat), from the USD geometry at the start, "
                       "scale applied; bbox = the world-axis box around all of the entity now")

    def arr(x):
        return x.detach().to("cpu").double().numpy() if hasattr(x, "detach") else np.asarray(x, np.float64)

    def scene_state():
        return t.memo("scene", lambda: env.iscene.get_state(is_relative=False))

    def entry(kind, name):
        return lambda: (scene_state().get(kind) or {}).get(name) or {}

    gz = _ground_z()
    arts = [(a.p.rstrip("_") or "robot", a.art) for a in RIG.arms] if RIG.kind == "franka" else [("robot", RIG.art)]
    for rname, art in arts:
        jn = list(art.joint_names)
        lim = arr(art.data.soft_joint_pos_limits[0])
        rng = {n: [float(lim[i, 0]), float(lim[i, 1])] for i, n in enumerate(jn)}

        def rdata(art=art, key=rname):
            def read():
                d = art.data
                return {"pos": arr(d.root_pos_w[0]), "quat": arr(d.root_quat_w[0]), "lin_vel": arr(d.root_lin_vel_w[0]),
                        "ang_vel": arr(d.root_ang_vel_w[0]), "q": arr(d.joint_pos[0]), "qd": arr(d.joint_vel[0])}
            return t.memo(("root", key), read)
        t.add(rname, "robot", {
            "pos": ("m", lambda r=rdata: r()["pos"]), "quat": ("1", lambda r=rdata: r()["quat"]),
            "lin_vel": ("m/s", lambda r=rdata: r()["lin_vel"]), "ang_vel": ("rad/s", lambda r=rdata: r()["ang_vel"]),
            "height_above_ground": ("m", lambda r=rdata: None if gz is None else float(r()["pos"][2]) - gz),
            "joint_pos": ("rad or m", lambda r=rdata, jn=jn: dict(zip(jn, r()["q"]))),
            "joint_vel": ("rad/s or m/s", lambda r=rdata, jn=jn: dict(zip(jn, r()["qd"]))),
            "joint_range": ("rad or m", lambda rng=rng: rng)},
            note=("the Franka arm: pos / quat = its root (panda_link0)" if RIG.kind == "franka" else
                  "the Unitree G1 with Dex3 hands: pos / quat = its root (the pelvis" + (")" if RIG.loco else ", welded to the world)"))
                 + "; height_above_ground = the root above the scene's ground plane (null when the scene has none)")
        bn = list(art.body_names)

        def bodies(art=art, key=rname):
            def read():
                d = art.data
                return arr(d.body_pos_w[0]), arr(d.body_quat_w[0]), arr(d.body_lin_vel_w[0]), arr(d.body_ang_vel_w[0])
            return t.memo(("bodies", key), read)
        if RIG.kind == "franka":
            keep = [b for b in bn if b.startswith("panda_")]
        else:
            keep = [b for b in bn if b in ("pelvis", "torso_link", "head_link", "waist_yaw_link", "left_wrist_yaw_link",
                                           "right_wrist_yaw_link") or "_hand_" in b or b.endswith("palm_link")
                    or b.endswith(("ankle_roll_link",))]
        pfx = "" if rname == "robot" else rname + "_"
        for b in keep:
            i = bn.index(b)
            t.add(pfx + b, "robot_link", {
                "pos": ("m", lambda i=i, bd=bodies: bd()[0][i]), "quat": ("1", lambda i=i, bd=bodies: bd()[1][i]),
                "lin_vel": ("m/s", lambda i=i, bd=bodies: bd()[2][i]), "ang_vel": ("rad/s", lambda i=i, bd=bodies: bd()[3][i])},
                note=f"link {b}")
        if RIG.kind == "franka" and "panda_hand" in bn:
            h = bn.index("panda_hand")

            def tcp(h=h, bd=bodies):
                T = X.T_of(bd()[0][h], bd()[1][h]); return T[:3, 3] + T[:3, 2] * X.FRANKA_TCP_Z
            t.add(pfx + "tcp", "robot_link", {"pos": ("m", tcp), "quat": ("1", lambda h=h, bd=bodies: bd()[1][h])},
                  note=f"the fingertip centre (proprio tcp_pose): panda_hand moved {X.FRANKA_TCP_Z} m along its approach axis")
    robot_arts = {id(a) for _, a in arts}
    st0 = env.iscene.get_state(is_relative=False)

    def prim_path(obj) -> str:
        return re.sub(r"env_\.\*", "env_0", str(obj.cfg.prim_path).replace("{ENV_REGEX_NS}", "/World/envs/env_0"))

    def pose0(n):
        rp = arr(((st0.get("rigid_object") or {}).get(n) or {}).get("root_pose")[0])
        return rp[:3], rp[3:7]

    def mass(obj) -> float:
        for read in (lambda: obj.data.default_mass, lambda: obj.root_physx_view.get_masses()):
            try:
                m = float(arr(read()).reshape(-1)[0])
                if m > 0 and math.isfinite(m):
                    return m
            except Exception:  # noqa: BLE001
                continue
        return 1.0
    rigid = {n: o for n, o in (getattr(env.iscene, "rigid_objects", None) or {}).items() if n in (st0.get("rigid_object") or {})}
    corners = {}
    for n, o in rigid.items():
        bx = _body_boxes(prim_path(o))
        if len(bx) == 1:
            corners[n] = _corners(*next(iter(bx.values())))

    # what is not listed: markers (collision disabled) and the variants parked in the scene's off-stage depot
    scfg = getattr(getattr(env, "scene", None), "cfg", None)
    park = getattr(scfg, "parking_pos", None)
    hidden = set()
    for n, o in rigid.items():
        cp = getattr(getattr(getattr(o, "cfg", None), "spawn", None), "collision_props", None)
        if getattr(cp, "collision_enabled", None) is False:
            hidden.add(n)
        elif park is not None:
            p0, _ = pose0(n)
            if p0[0] >= float(park[0]) - 0.05 and p0[1] >= float(park[1]) - 0.05:
                hidden.add(n)
    rename = {}
    fam = [f[0] for f in (getattr(scfg, "families", None) or ()) if isinstance(f, (tuple, list)) and f and isinstance(f[0], str)
           and f[0] in rigid]
    present = [n for n in fam if n not in hidden]
    if len(fam) > 1 and len(present) == 1:
        rename[present[0]] = os.path.commonprefix(fam).rstrip("_") or "item"

    # bodies of several rigid parts, one entity each
    groups = []
    pieces = sorted((n for n in rigid if re.fullmatch(r"piece_\d+", n) and n not in hidden), key=lambda n: int(n[6:]))
    if len(pieces) > 1:
        food = getattr(scfg, "food", None)
        groups.append((food if isinstance(food, str) and re.fullmatch(r"[a-z][a-z0-9_]*", food) else "food", pieces, "mass",
                       "the food as one body (all of its parts together): pos = their centre of mass, quat = the first "
                       "part's orientation, lin_vel = their centre of mass's, ang_vel = the first part's; size = its extents "
                       "at the start along those axes"))
    housing = sorted(n for n in rigid if n.startswith("cm_") and n not in hidden)
    if len(housing) > 1:
        groups.append(("coffee_machine", ["cm_tank"] + [n for n in housing if n != "cm_tank"] if "cm_tank" in housing
                       else housing, "box", "the machine's fixed housing as one body (the cover and the START key are their "
                       "own entities): pos = the centre of its box, quat = its frame"))
    grouped = {n for _, ms, _, _ in groups for n in ms}

    def rigid_pose(e):
        return lambda: arr(e()["root_pose"][0])

    for kind in STATE_KINDS:
        for n in sorted(st0.get(kind) or {}):
            if n == "robot" or (kind == "articulation" and id((env.iscene.articulations or {}).get(n)) in robot_arts):
                continue
            if n in hidden or n in grouped:
                continue
            e = entry(kind, n)
            if kind == "deformable_object":
                def nodes(e=e):
                    v = e().get("nodal_position")
                    return None if v is None else arr(v[0]).reshape(-1, 3)
                t.add(n, "object", {"pos": ("m", lambda nodes=nodes: None if nodes() is None else nodes().mean(0)),
                                    "bbox": ("m", lambda nodes=nodes: None if nodes() is None else [nodes().min(0), nodes().max(0)])},
                      note="a deformable body: the centroid and the axis-aligned box of its simulation nodes")
                continue
            fields = {"pos": ("m", lambda e=e: arr(e()["root_pose"][0])[:3]), "quat": ("1", lambda e=e: arr(e()["root_pose"][0])[3:7]),
                      "lin_vel": ("m/s", lambda e=e: arr(e()["root_velocity"][0])[:3]),
                      "ang_vel": ("rad/s", lambda e=e: arr(e()["root_velocity"][0])[3:6])}
            out_kind = "object"
            if kind == "rigid_object":
                if n in corners:
                    c = corners[n]
                    fields["size"] = ("m", lambda c=c: c.max(0) - c.min(0))
                    fields["bbox"] = ("m", lambda e=e, c=c: _world_box([(arr(e()["root_pose"][0])[:3], arr(e()["root_pose"][0])[3:7], c)]))
            else:
                art = env.iscene.articulations[n]
                bx = _body_boxes(prim_path(art))
                abn = list(art.body_names)
                idx = [(i, _corners(*bx[b])) for i, b in enumerate(abn) if b in bx]
                if idx:
                    d = art.data
                    bp, bq = arr(d.body_pos_w[0]), arr(d.body_quat_w[0])
                    r0 = arr(d.root_pos_w[0]), arr(d.root_quat_w[0])
                    sz = _frame_size([(bp[i], bq[i], c) for i, c in idx], *r0)

                    def links(art=art, key=n):
                        return t.memo(("links", key), lambda: (arr(art.data.body_pos_w[0]), arr(art.data.body_quat_w[0])))
                    fields["size"] = ("m", lambda sz=sz: sz)
                    fields["bbox"] = ("m", lambda lk=links, idx=idx: _world_box([(lk()[0][i], lk()[1][i], c) for i, c in idx]))
                jn = list(art.joint_names)
                if jn:
                    out_kind = "articulation"
                    lim = arr(art.data.soft_joint_pos_limits[0])
                    fields.update({"joint_pos": ("rad or m", lambda e=e, jn=jn: dict(zip(jn, arr(e()["joint_position"][0])))),
                                   "joint_vel": ("rad/s or m/s", lambda e=e, jn=jn: dict(zip(jn, arr(e()["joint_velocity"][0])))),
                                   "joint_range": ("rad or m", lambda r={j: [float(lim[i, 0]), float(lim[i, 1])] for i, j in enumerate(jn)}: r)})
            t.add(rename.get(n, n), out_kind, fields)

    for name, members, centre, note in groups:
        if name in t.entities()["entities"]:
            name = name + "_body"
        ents = [entry("rigid_object", m) for m in members]
        cs = [corners.get(m) for m in members]
        w = np.array([mass(rigid[m]) for m in members], np.float64)
        w = w / w.sum()

        def poses(ents=ents, key=name):
            return t.memo(("group", key), lambda: [arr(e()["root_pose"][0]) for e in ents])

        def vels(ents=ents, key=name):
            return t.memo(("group_v", key), lambda: [arr(e()["root_velocity"][0]) for e in ents])

        def box(ps=poses, cs=cs):
            parts = [(p[:3], p[3:7], c) for p, c in zip(ps(), cs) if c is not None]
            return _world_box(parts) if parts else None
        if centre == "mass":
            pos = (lambda ps=poses, w=w: np.sum([wi * p[:3] for wi, p in zip(w, ps())], axis=0))
            lin = (lambda vs=vels, w=w: np.sum([wi * v[:3] for wi, v in zip(w, vs())], axis=0))
        else:
            pos = (lambda b=box, ps=poses: ps()[0][:3] if b() is None else (b()[0] + b()[1]) / 2)
            lin = (lambda vs=vels: vs()[0][:3])
        fields = {"pos": ("m", pos), "quat": ("1", lambda ps=poses: ps()[0][3:7]), "lin_vel": ("m/s", lin),
                  "ang_vel": ("rad/s", lambda vs=vels: vs()[0][3:6])}
        if all(c is not None for c in cs):
            p0 = [pose0(m) for m in members]
            sz = _frame_size([(p, q, c) for (p, q), c in zip(p0, cs)], *p0[0])
            fields["size"] = ("m", lambda sz=sz: sz)
            fields["bbox"] = ("m", box)
        t.add(name, "object", fields, note=note)
    return t


# ---------------------------------------------------------------- the reset ablation (RCB_STATE_OPS=1; rcb_limited
# "ablations", off unless the task sets it). reset = load of the snapshot taken at the frozen start, before anything ran.
# A snapshot (save) is the simulator's full state as robobench and Isaac Lab hold it, plus this service's per-episode state:
#   - Isaac Lab's scene state (InteractiveScene.get_state: every articulation's root pose / velocity and joint positions /
#     velocities, every rigid object's root pose / velocity, every deformable's nodal positions / velocities), every
#     articulation's actuator targets (position, velocity, effort) and every asset's permanent external wrench;
#   - robobench's own restorable state (BaseEnv.get_states: the scene's mechanics, latches, weld and grasp-weld flags, the
#     robot's actuator setpoints and its controller's smoothing buffers), restored by BaseEnv.set_states;
#   - the RNG states (torch, CUDA, numpy, random);
#   - the grader's recorded state (its history, milestones and latches), the tool-only monitor's record, the graded rows
#     (the trajectory recorder's actions and states) and the ee_pose target in force.
# A restore writes all of it, updates the articulations' link poses (no physics step, no render) and checks the state
# hash (rcb_embodiedswe.state_sha) against the one taken at the save: a mismatch is counted (restore_sha_mismatches).
# GPU PhysX's solver caches are not part of any snapshot, so the steps after a restore are not bit-identical to the
# steps after the save (the grasp-weld pools: GRASP_POOL_OPS above). A snapshot is a few MB (the recorded rows; the simulator's state is small). The live video counts the trial's control
# steps across episodes (vstep).
VOFF = [0]           # control steps of the trial before the running episode's own (a reset / load restarts ctrl_steps)


def vstep() -> int:
    return VOFF[0] + ctrl_steps()


def _is_data(v) -> bool:
    import torch
    if v is None or isinstance(v, (bool, int, float, str, torch.Tensor, np.ndarray, np.generic)):
        return True
    if isinstance(v, dict):
        return all(_is_data(k) and _is_data(x) for k, x in v.items())
    if isinstance(v, (list, tuple, set, frozenset)):
        return all(_is_data(x) for x in v)
    return False


def _copy(v):
    """A deep copy of plain state (tensors cloned on their device, arrays copied)."""
    import torch
    if isinstance(v, torch.Tensor):
        return v.detach().clone()
    if isinstance(v, np.ndarray):
        return v.copy()
    if isinstance(v, dict):
        return {k: _copy(x) for k, x in v.items()}
    if isinstance(v, (list, tuple, set, frozenset)):
        return type(v)(_copy(x) for x in v)
    return v


def _data_attrs(obj, names=None, skip=()) -> dict:
    """The plain-data attributes of `obj` (all of them, or `names`), copied; handles and callables are left out."""
    keys = names if names is not None else [k for k in vars(obj) if k not in skip]
    return {k: _copy(getattr(obj, k)) for k in keys if hasattr(obj, k) and _is_data(getattr(obj, k))}


def _set_attrs(obj, saved: dict) -> None:
    for k, v in saved.items():
        setattr(obj, k, _copy(v))


def _assets() -> list:
    sc = env.iscene
    return [*(sc.articulations or {}).values(), *(getattr(sc, "rigid_objects", None) or {}).values()]


def _wrenches() -> list:
    out = []
    for a in _assets():
        c = getattr(a, "_permanent_wrench_composer", None)
        if c is None or not hasattr(c, "_composed_force_b_torch"):
            out.append(None)
            continue
        out.append({"active": bool(getattr(c, "_active", False)), "force": c._composed_force_b_torch.clone(),
                    "torque": c._composed_torque_b_torch.clone()})
    return out


def _rng() -> dict:
    import random
    import torch
    return {"torch": torch.get_rng_state(), "cuda": torch.cuda.get_rng_state_all() if torch.cuda.is_available() else None,
            "numpy": np.random.get_state(), "random": random.getstate()}


def _set_rng(s: dict) -> None:
    import random
    import torch
    torch.set_rng_state(s["torch"])
    if s["cuda"] is not None:
        torch.cuda.set_rng_state_all(s["cuda"])
    np.random.set_state(s["numpy"]); random.setstate(s["random"])


TOOL_KEYS = ("substeps", "touch_substeps", "first_touch_step", "max_force", "touched_links")


def ops_snapshot() -> dict:
    arts = env.iscene.articulations or {}
    return {"iscene": _copy(env.iscene.get_state(is_relative=False)),
            "targets": {n: {"pos": a.data.joint_pos_target.clone(), "vel": a.data.joint_vel_target.clone(),
                            "eff": a.data.joint_effort_target.clone()} for n, a in arts.items()},
            "wrench": _wrenches(), "robobench": _copy(env.get_states()), "rng": _rng(),
            "grader": _data_attrs(grader, skip=("env", "scene")),
            "tool": _data_attrs(TOOL, TOOL_KEYS) if TOOL is not None else None,
            "traj": {"actions": list(TRAJ.actions), "steps": list(TRAJ.steps), "rows": list(TRAJ.rows), "every": TRAJ.every,
                     "layout": _copy(TRAJ.layout)},
            "rig": _data_attrs(RIG, ("last_cmd",)), "tgt": list(E["tgt"]), "sha": R.state_sha(env)}


def ops_restore(snap: dict, op: str) -> None:
    env.iscene.reset_to(_copy(snap["iscene"]), env_ids=None, is_relative=False)
    for n, a in (env.iscene.articulations or {}).items():
        t = snap["targets"][n]
        a.set_joint_position_target(t["pos"].clone()); a.set_joint_velocity_target(t["vel"].clone())
        a.set_joint_effort_target(t["eff"].clone())
    env.set_states(_copy(snap["robobench"]))              # robobench's mechanics and the robot's setpoints and controller
    for a, w in zip(_assets(), snap["wrench"]):
        c = getattr(a, "_permanent_wrench_composer", None)
        if w is not None and c is not None:
            c._composed_force_b_torch.copy_(w["force"]); c._composed_torque_b_torch.copy_(w["torque"]); c._active = w["active"]
    env.iscene.write_data_to_sim()
    pv = env.sim.physics_sim_view
    if pv is not None:
        pv.update_articulations_kinematic()               # the links' poses from the restored joints (no physics step)
    env.sim.forward()
    _set_rng(snap["rng"])
    _set_attrs(grader, snap["grader"])
    if TOOL is not None and snap["tool"] is not None:
        _set_attrs(TOOL, snap["tool"])
    tr = snap["traj"]
    TRAJ.actions, TRAJ.steps, TRAJ.rows, TRAJ.every, TRAJ.layout = (list(tr["actions"]), list(tr["steps"]), list(tr["rows"]),
                                                                    tr["every"], _copy(tr["layout"]))
    _set_attrs(RIG, snap["rig"])
    E["tgt"] = list(snap["tgt"])
    _R["step"] = -1                                       # the cameras show the restored state at the next grab
    ok = R.state_sha(env) == snap["sha"]
    if not ok:
        E["restore_sha_mismatches"] = int(E.get("restore_sha_mismatches") or 0) + 1
    if E.get("ready"):                                    # (not while a resume replays the journal)
        log({"ev": "state_restore", "op": op, "sha_match": ok})


def _new_episode() -> None:
    """The ended episode's record (episode.json, trajectory.npz, the video's end frame) is not the new episode's."""
    for k in ("success_step", "verdict", "dumped"):
        E.pop(k, None)
    for f in ("episode.json", "trajectory.npz"):
        try:
            os.remove(os.path.join(DATA, f))
        except OSError:
            pass
    VIDEO.truncate(vstep())              # removes only last.jpg / last.json: no frame lies past the trial's step


def ops_reset() -> None:
    VOFF[0] += ctrl_steps()
    ops_restore(S0, "reset")
    _new_episode()


def ops_save():
    return ops_snapshot()


def ops_observe(meta: dict):
    """The first frames after a reset or a load. The op has already run, so a view that /observe would refuse (an
    empty camera list, an unknown camera, a bad video option) gets the default view instead of failing the new episode."""
    meta = {**meta, "seconds": 0}
    try:
        parse_observe(meta)
    except Exception:  # noqa: BLE001
        meta = {"seconds": 0}
    return do_observe(meta)


def ops_load(snap) -> None:
    VOFF[0] += ctrl_steps() - len(snap["traj"]["actions"])
    ops_restore(snap, "load")
    _new_episode()


E["tgt"] = [float(v) for v in RIG.hold()]
TRAJ.frame()
VIDEO.offer(0)
S0 = ops_snapshot() if L.STATE_OPS else None      # the frozen start, before anything ran (a reset loads it)
if L.STATE_OPS:
    OPS = L.StateOps(budget=max(0, int(os.environ.get("RCB_RESET_BUDGET") or 0)), reset=ops_reset, save=ops_save, load=ops_load)
resumed = EP.begin(state=E, step=replay_row, state_sha=lambda: R.state_sha(env), proprio=proprio_row, spec=SPEC, on_end=end_from_signal,
                   start_event={"task": PRESET, "seed": int(INST["seed"]), "cameras": CAMS, "kinematics_dev_m": round(KIN_DEV, 6),
                                **{k: v for k, v in INSTANCE_CHECK.items()}},
                   **({"ops": OPS} if OPS is not None else {}))
if resumed:
    VIDEO.truncate(vstep())
E["ready"] = True; write_final()
open(PID_FILE, "w").write(str(os.getpid()))
print(f"{PROTOCOL} sim ready: {PRESET} (startup {EP.n_startups}, resumes {EP.resumes})", flush=True)
L.serve(L.Service(spec=SPEC, lock=LOCK, state=E, submit=do_submit, observe=do_observe, kinematics=do_kinematics, status=status,
                  on_submit_error=lambda exc: finish("infra_error" if isinstance(exc, InfraError) else "error"), log=log, episode=EP, state_sha=lambda: R.state_sha(env), proprio=proprio_row,
                  state_table=state_table() if L.STATE_QUERY else None,
                  **({"state_ops": OPS, "observe_now": ops_observe} if OPS is not None else {})))
