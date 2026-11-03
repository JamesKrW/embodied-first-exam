"""eai-standard/2.2 sim service for the MyoSuite tasks (docs/protocol/standard_mode.md): the adapter of the shared layer
rcb_limited.py; the simulator is rcb_myosuite.Session (MyoSuite 2.12.2 on MuJoCo, CPU). HTTP surface: GET /spec,
GET /status, POST /observe, POST /submit, POST /kinematics, and POST /reset when the run grants resets (RCB_RESET_BUDGET,
the optional reset capability of docs/protocol/standard_mode.md; MyoSuite's standard tasks default to 10 in their compose
file, PROTOCOL §4.5). Every call is logged to /data/episode.jsonl.

The episode is the frozen instance's (environment/sim/instance.npz, the privileged twin's). The action is MyoSuite's own:
one command per muscle (excitation u = 1 / (1 + exp(-5 (a - 0.5)))), one row per control step. The episode ends the
moment the task's goal has held for its hold time (success), at the task's failure (object dropped, fingers off the key,
...), or when the agent's time is over (the collect hook). With a reset budget, POST /reset abandons the running (or
ended) episode and starts a new one from the frozen instance; only the last episode is graded, and a success ends the
trial. Nothing about objects or goals leaves this process: the proprioception is the limb's own joints and muscles,
and a camera that rides on a body of the scene (ChaseTag's `opponent_view`, soccer's `goalkeeper_view`) gives images only:
its pose would be that body's position (SCENE_CAMS).

Crash recovery (rcb_limited.Episode, resume class `exact`: MuJoCo on the CPU repeats bit for bit in a fresh process):
every executed row is journaled; after a crash the service rebuilds the episode and replays the journal, resetting where
the committed resets did (their journal rows are checkpointed, `persist`). When the episode ends the service writes
/data/trajectory.npz + /data/episode.json; the collect hook (finalize.sh) makes it exit and writes the verdict.

The verdict is the live episode's (owner, 2026-10-17): `success` in episode.json is the task's rule as
Session.step applied it to the steps this service ran (MyoSuite's goal held for the family's hold time, the challenge's
own success event, or the MyoDex track over the whole reference: the windows are part of the rule, so the episode only
ends in success once its window is complete). Nothing is replayed to grade it. The frozen initial state is checked when
the episode begins: a fresh start whose built scene does not hash to the instance's recorded sha gives up as an
infrastructure failure (`instance_mismatch`) before the agent is served."""

import contextlib
import copy
import json
import math
import os
import re
import sys
import threading
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rcb_limited as L  # noqa: E402

DATA = os.environ.get("RCB_DATA", "/data")
INSTANCE = os.environ.get("RCB_INSTANCE", os.path.join(os.path.dirname(os.path.abspath(__file__)), "instance.npz"))
WALL_CLOCK_S = float(os.environ.get("RCB_WALL_CLOCK_S", "5400"))
MAX_CHUNK_S = float(os.environ.get("RCB_MAX_CHUNK_S", "10"))
MAX_WATCH_S = float(os.environ.get("RCB_MAX_WATCH_S", "30"))
RESET_BUDGET = max(0, int(os.environ.get("RCB_RESET_BUDGET") or 0))  # the compose file's default: 10 (PROTOCOL §4.5)
W, H = 640, 480
# Images one call may return (cameras x frames, per-frame depth counted again), checked before anything runs: every
# frame is held until the response is sent. Measured on the layer's Recorder with 640x480 frames: 2.7 GB per 1000 RGB
# frames and 5.1 GB per 1000 images of one camera with per-frame depth, on top of the simulator's 1.4 GB, under the
# sidecar's 8 GiB cap (docs/benchmarks/myosuite.md). The 2.1 services' 3000 would need more than the cap.
MAX_FRAMES = 1000
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
    "resets": 0,  # resets done (the episode is resets + 1)
    "reset_at": [],  # the journal row at which each reset happened: a resume resets there again
}

os.environ["RCB_SESSION_LOG"] = ""
import rcb_myosuite as M  # noqa: E402  (first: it picks the rendering backend before mujoco loads)
import mujoco  # noqa: E402

M.SESSION_LOG = ""  # the sidecar's Session is not the agent's interaction log
_INST = np.load(INSTANCE, allow_pickle=False)
TASK = str(_INST["task"])
NU = int(_INST["model__actuator_ctrlrange"].shape[0])  # the journal's row width, known before the scene is built
EP = L.Episode(DATA, dim=NU, resume="exact", persist=("resets", "reset_at"))
EP.startup()  # start count, back-off after failed starts, the startup watchdog until scene_built()
S = M.Session(TASK, instance=INSTANCE)
assert S.nu == NU, (S.nu, NU)
MD = S.model
DT = S.dt
CTRL_HZ = 1.0 / DT
SUBSTEPS = int(S.base.frame_skip)
ROWS: list = []  # every executed row (a resume replays them first): trajectory.npz
BODY = M.BODY[TASK]
ROBOT = {
    "hand": ("MyoHand", "a musculoskeletal model of a human forearm and hand (MyoSuite)"),
    "arm": ("MyoArm", "a musculoskeletal model of a human shoulder, arm and hand (MyoSuite)"),
    "body": ("MyoArm with torso", "a musculoskeletal model of a human torso, right arm and hand on a pelvis that slides on the floor (MyoSuite)"),
    "bimanual": ("MyoArm + MPL", "a musculoskeletal right arm and hand (MyoSuite's MyoArm) next to a motorized prosthetic left arm (the Modular Prosthetic Limb)"),
    "legs": ("MyoLegs", "a free-standing musculoskeletal model of a human torso and both legs (MyoSuite)"),
    "osl": ("MyoLeg + OSL", "a free-standing musculoskeletal model of a human with the right leg's muscles and an Open-Source Leg knee-ankle prosthesis on the left (MyoSuite)"),
}[BODY]
TASK_INFO = json.load(open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "task.json")))
CAMS = list(S.cameras())
DEFAULT_CAMS = list(TASK_INFO["default_cameras"])
ACTS = S.actuators()
ACT_NAMES = [a["name"] for a in ACTS]
MUS_IDX = [a["index"] for a in ACTS if a["kind"] == "muscle"]
MUSCLES = [ACTS[i]["name"] for i in MUS_IDX]
OTHER = [a for a in ACTS if a["kind"] != "muscle"]
ATYPE = "muscle" if not OTHER else "actuators"
ACT_ADR = np.array([int(MD.actuator_actadr[i]) for i in MUS_IDX], dtype=int)
# the body's own joints only: the key's hinge and the OBJ* joints of pen, die and objects are object state, never sent
# (free joints, of the balls, objects and paddles, are left out by S.joints(); a free-standing body's root is reported
# on its own, in root_pose / root_vel)
LIMB = [j for j in S.joints() if j["name"] != "keyjoint" and not j["name"].startswith("OBJ")]
J_NAMES = [j["name"] for j in LIMB]
J_Q = np.array([j["qpos_adr"] for j in LIMB])
J_V = np.array([j["dof_adr"] for j in LIMB])
J_ID = [MD.joint(n).id for n in J_NAMES]
J_SLIDE = [int(MD.jnt_type[j]) == 2 for j in J_ID]
ROOT = MD.joint("root").id if mujoco.mj_name2id(MD, mujoco.mjtObj.mjOBJ_JOINT, "root") >= 0 else None
if ROOT is not None and int(MD.jnt_type[ROOT]) != 0:
    ROOT = None
OWN_JOINTS = set(J_ID) | ({ROOT} if ROOT is not None else set())  # what moves the body itself: all of it is proprioception


def own_body(b: int) -> bool:
    """Whether the MuJoCo body `b` is fixed in the world or moved by the body's own joints alone. A mocap body (the
    opponent, the goalkeeper, goal markers) or a body behind any other joint (objects, balls, the key) is scene state:
    where it is must never leave this process (PROTOCOL §4.5)."""
    while b > 0:
        if int(MD.body_mocapid[b]) >= 0:
            return False
        first = int(MD.body_jntadr[b])
        if any(j not in OWN_JOINTS for j in range(first, first + int(MD.body_jntnum[b]))):
            return False
        b = int(MD.body_parentid[b])
    return True


def scene_camera(cam: str) -> bool:
    """A camera that is mounted on, tracks or is aimed at a body of the scene that is not the agent's own: its pose
    is that body's position plus a constant, so the service gives its images (and depth) and never its pose."""
    i = MD.camera(cam).id
    bodies = [int(MD.cam_bodyid[i])]
    if int(MD.cam_mode[i]) in (3, 4):  # targetbody, targetbodycom: the orientation follows the target
        bodies.append(int(MD.cam_targetbodyid[i]))
    return not all(own_body(b) for b in bodies if b >= 0)


SCENE_CAMS = [c for c in CAMS if scene_camera(c)]
NO_POSE = ("not published: this camera rides on a body of the scene that is not yours, so its pose would be that body's "
           "position; it gives images and depth (camera frame) only")
GROUPS = [
    ("joint_pos", [f"q_{n}" for n in J_NAMES], "joint positions of the body (rad; m for sliding joints)"),
    ("joint_vel", [f"qd_{n}" for n in J_NAMES], "joint velocities (rad/s; m/s)"),
    (
        "muscle_activation",
        [f"act_{n}" for n in MUSCLES],
        "muscle activations in [0, 1] (they follow the excitations with a first-order delay)",
    ),
    ("muscle_length", [f"len_{n}" for n in MUSCLES], "muscle-tendon lengths (m)"),
    ("joint_effort", [f"effort_{n}" for n in J_NAMES], "generalized force of all actuators on each joint (N m; N)"),
    ("gripper_opening", [], "none (a hand, not a gripper)"),
    ("tcp_pose", [], "none (a hand, not a gripper); fingertip poses: /kinematics fk"),
]
if ROOT is not None:
    GROUPS += [
        ("root_pose", ["root_x", "root_y", "root_z", "root_qw", "root_qx", "root_qy", "root_qz"],
         "the pelvis (the free-floating root) in the world frame: position (m) and wxyz quaternion"),
        ("root_vel", ["root_vx", "root_vy", "root_vz", "root_wx", "root_wy", "root_wz"],
         "the root's linear (m/s, world axes) and angular velocity (rad/s, its own axes), as MuJoCo's free joint reports them"),
    ]
PROPRIO_LAYOUT = [n for _, names, _ in GROUPS for n in names]
PROPRIO_GROUPS, _i = {}, 0
for _g, _names, _meaning in GROUPS:
    PROPRIO_GROUPS[_g] = [_i, len(_names), _meaning]
    _i += len(_names)


# ------------------------------------------------------------------------------------------------- cameras
def _intrinsic(cam: str) -> list:
    fovy = math.radians(float(MD.cam_fovy[MD.camera(cam).id]))
    f = 0.5 * H / math.tan(fovy / 2)
    return [[f, 0.0, W / 2], [0.0, f, H / 2], [0.0, 0.0, 1.0]]


def _cam_T_gl(data, cam: str) -> np.ndarray:
    i = MD.camera(cam).id
    T = np.eye(4)
    T[:3, :3] = data.cam_xmat[i].reshape(3, 3)  # MuJoCo cameras: OpenGL axes (x right, y up, z back)
    T[:3, 3] = data.cam_xpos[i]
    return T


GL2CV = np.diag([1.0, -1.0, -1.0, 1.0])


def camera_info(cam: str) -> dict:
    """World -> camera (OpenCV axes) and camera -> world (OpenGL axes) at the current state; none of a scene camera."""
    if cam in SCENE_CAMS:
        return {"intrinsic_cv": _intrinsic(cam), "extrinsic_cv": None, "cam2world_gl": None, "image_size": [W, H],
                "pose": NO_POSE}
    c2w = _cam_T_gl(S.data, cam)
    ext = np.linalg.inv(c2w @ GL2CV)[:3, :]
    return {
        "intrinsic_cv": _intrinsic(cam),
        "extrinsic_cv": ext.tolist(),
        "cam2world_gl": c2w.tolist(),
        "image_size": [W, H],
    }


CAM_MODES = {0: "fixed", 1: "track", 2: "trackcom", 3: "targetbody", 4: "targetbodycom"}


def _mount(cam: str) -> dict:
    i = MD.camera(cam).id
    body = MD.body(int(MD.cam_bodyid[i])).name
    mode = int(MD.cam_mode[i])
    if cam in SCENE_CAMS:   # neither the body it rides on nor an offset: either would place that body
        return {"type": "link", "parent_link": None, "pose": NO_POSE,
                "note": "it moves with a body of the scene; extrinsic_cv and cam2world_gl are null in meta[\"cameras\"] "
                "and the per-frame extrinsic_cv rows are NaN"}
    if mode != 0:       # MuJoCo tracking cameras follow a body (or the model's centre of mass) at a fixed offset
        return {"type": "link", "parent_link": body, "tracking": CAM_MODES.get(mode, str(mode)),
                "note": "a MuJoCo tracking camera: it follows its body's position (or the centre of mass of its "
                "subtree) without turning with it; its pose is in every response (extrinsic_cv per frame)"}
    if body == "world":
        info = camera_info(cam)
        return {"type": "static", "extrinsic_cv": info["extrinsic_cv"], "cam2world_gl": info["cam2world_gl"]}
    b = MD.body(body).id
    Tb = np.eye(4)
    Tb[:3, :3] = S.data.xmat[b].reshape(3, 3)
    Tb[:3, 3] = S.data.xpos[b]
    link_T_cam_cv = np.linalg.inv(Tb) @ _cam_T_gl(S.data, cam) @ GL2CV
    return {"type": "link", "parent_link": body, "link_T_cam_cv": link_T_cam_cv.tolist()}


def _mounted(cam: str) -> str:
    m = _mount(cam)
    if cam in SCENE_CAMS:
        return "on a body of the scene that is not yours (it moves with it): images and depth only, no pose"
    if m["type"] == "static":
        return "static"
    if "tracking" in m:
        return f"tracking ({m['tracking']}) the body {m['parent_link']}"
    return f"fixed to the body {m['parent_link']}: it moves with it"


MOUNTED = {c: _mounted(c) for c in CAMS}


# ------------------------------------------------------------------------------------------------- kinematics
KD = mujoco.MjData(MD)  # kinematics only: a copy of the start state, never stepped
KD.qpos[:] = S.data.qpos
if ROOT is not None:  # a free-standing body answers in its root's frame (base_pose puts the root somewhere)
    KD.qpos[MD.jnt_qposadr[ROOT]:MD.jnt_qposadr[ROOT] + 7] = [0, 0, 0, 1, 0, 0, 0]
KIN_FRAME = "base" if ROOT is not None else "world"
TIPS = [
    n for n in ("THtip", "IFtip", "MFtip", "RFtip", "LFtip") if mujoco.mj_name2id(MD, mujoco.mjtObj.mjOBJ_SITE, n) >= 0
]
MOUNT_LINKS = sorted({MD.body(int(MD.cam_bodyid[MD.camera(c).id])).name for c in CAMS if c not in SCENE_CAMS} - {"world"})
LINKS = TIPS + MOUNT_LINKS
_rb = int(MD.jnt_bodyid[J_ID[0]])
while int(MD.body_parentid[_rb]) != 0:
    _rb = int(MD.body_parentid[_rb])
ROOT_BODY = MD.body(_rb).name
ROOT_POSE = [*S.data.xpos[_rb].tolist(), *S.data.xquat[_rb].tolist()]
LO = np.array([MD.jnt_range[j, 0] if MD.jnt_limited[j] else -np.inf for j in J_ID])
HI = np.array([MD.jnt_range[j, 1] if MD.jnt_limited[j] else np.inf for j in J_ID])


def _fk_kd(q, base_pose=None) -> None:
    if ROOT is not None:
        a = MD.jnt_qposadr[ROOT]
        KD.qpos[a:a + 7] = base_pose if base_pose is not None else [0, 0, 0, 1, 0, 0, 0]
    KD.qpos[J_Q] = q
    mujoco.mj_kinematics(MD, KD)
    mujoco.mj_comPos(MD, KD)  # the Jacobians (mj_jacSite) need it


def _pose(name: str) -> list:
    quat = np.zeros(4)
    if name in TIPS:
        sid = MD.site(name).id
        mujoco.mju_mat2Quat(quat, KD.site_xmat[sid])
        return [*KD.site_xpos[sid].tolist(), *quat.tolist()]
    b = MD.body(name).id
    return [*KD.xpos[b].tolist(), *KD.xquat[b].tolist()]


def do_kinematics(req: dict) -> dict:
    """Fingertips (sites) and the camera mounts (bodies) in the world frame for joint angles q (the limb's joints, in
    `joint_pos` order). IK solves fingertip positions only (Levenberg-Marquardt over all limb joints within their
    ranges); a fingertip's orientation is ignored."""
    if req["op"] == "fk":
        _fk_kd(np.asarray(req["q"]), req.get("base_pose"))
        return L.fk_response({n: _pose(n) for n in req["links"]}, KIN_FRAME)
    q = np.array(req["seed"] if req["seed"] is not None else S.data.qpos[J_Q], dtype=np.float64)
    tg = {k: np.asarray(v[:3]) for k, v in req["targets"].items()}
    jac = np.zeros((3, MD.nv))

    def err(qq):
        _fk_kd(qq)
        return np.concatenate([t - KD.site_xpos[MD.site(k).id] for k, t in tg.items()])

    lam, e = 1e-2, err(q)
    for _ in range(req["max_iters"]):  # Levenberg-Marquardt on the stacked fingertip position errors
        if max(np.linalg.norm(e[3 * i : 3 * i + 3]) for i in range(len(tg))) < req["tol_pos_m"]:
            break
        _fk_kd(q)
        Js = []
        for k in tg:
            mujoco.mj_jacSite(MD, KD, jac, None, MD.site(k).id)
            Js.append(jac[:, J_V].copy())
        J = np.vstack(Js)
        dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(len(e)), e)
        n = np.linalg.norm(dq)
        if n > 0.2:
            dq *= 0.2 / n
        qn = np.clip(q + dq, LO, HI)
        en = err(qn)
        if np.linalg.norm(en) < np.linalg.norm(e):
            q, e, lam = qn, en, max(lam / 2, 1e-6)
        else:
            lam = min(lam * 4, 1e2)
    _fk_kd(q)
    errors = {k: (float(np.linalg.norm(t - KD.site_xpos[MD.site(k).id])), 0.0) for k, t in tg.items()}
    ok = all(v[0] <= req["tol_pos_m"] for v in errors.values())
    return L.ik_response(
        q, ok, errors, KIN_FRAME, note="fingertips: position only (orientation ignored); no collision check"
    )


# ------------------------------------------------------------------------------------------------- spec
TASK_TEXT = TASK_INFO["task_text"]
ONE = "one episode, no reset" if not RESET_BUDGET else (
    f"an episode (with up to {RESET_BUDGET} resets: POST /reset starts a new one; only the last is graded)")
EPISODE = {
    "hold": f"{ONE}: it ends the moment the task's goal has held for {S.hold_steps} control steps in a row "
    f"({round(S.hold_steps * DT, 3)} s; success), at the task's failure (see task_text), or when your time is over",
    "event": f"{ONE}: it ends at the challenge's success event (success), at its failure conditions or "
    "its own time limit (see task_text), or when your time is over",
    "track": f"{ONE}: it lasts {S.track_steps} control steps (the reference motion; then it is judged), "
    "ends early at MyoDex's termination (failure), or when your time is over",
}[S.rule]
MUSCLE_TEXT = (
    "muscles: one real number a per muscle; excitation u = 1 / (1 + exp(-5 (a - 0.5))), a not clipped (a = -1: u ~ "
    "0.0006, 0.5: 0.5, 1: 0.92, 2: 0.9994); activations follow excitations with a first-order delay (~10 ms up, ~40 ms "
    "down); muscles only pull, one can cross several joints, and its force depends on its length and velocity"
)
ACTION = {
    "dim": NU,
    "layout": ACT_NAMES,
    "limits": None,
    "units": MUSCLE_TEXT if not OTHER else MUSCLE_TEXT + "; the other actuators: see `actuators`",
    "semantics": "one row per control step, every actuator in `layout` order. Rows are `dt` apart (a whole multiple of "
    "the control period, at most 0.5 s), `linear` (default: from the previous command) or `hold` between rows; "
    "nothing is clipped by the service (MuJoCo clips the non-muscle controls to their ranges)",
}
if OTHER:
    ACTION["actuators"] = [{"index": a["index"], "name": a["name"], "kind": a["kind"], "command": a["command"]} for a in OTHER]
SPEC = {
    "protocol": L.PROTOCOL,
    "protocol_compat": list(L.COMPAT),
    "simulator": "MyoSuite 2.12.2 on MuJoCo 3.6.0 (CPU)",
    "robot": f"{ROBOT[0]}: {ROBOT[1]}, {len(MUSCLES)} muscles" + (f" and {len(OTHER)} other actuators" if OTHER else ""),
    "task_text": TASK_TEXT,
    "control_hz": CTRL_HZ,
    "physics_substeps": SUBSTEPS,
    "max_chunk_s": MAX_CHUNK_S,
    "max_watch_s": MAX_WATCH_S,
    "max_frames_per_call": MAX_FRAMES,
    "cameras": {
        c: {
            "rgb": True,
            "depth": True,
            "resolution": [W, H],
            "intrinsic_cv": _intrinsic(c),
            "mounted": MOUNTED[c],
            "mount": _mount(c),
        }
        for c in CAMS
    },
    "default_cameras": DEFAULT_CAMS,
    "action_types": {ATYPE: ACTION, "not_yet": ["joint_position", "joint_torque", "joint_delta"]},
    "proprio_layout": PROPRIO_LAYOUT,
    "proprio_groups": PROPRIO_GROUPS,
    "static": {"muscles": MUSCLES},
    "world": {
        "frame": "MyoSuite's world frame, z up"
        + ("; the body stands free in it (its root pose is proprioception)" if ROOT is not None
           else "; the body's root is fixed in it")
        + " (no positions of the scene are given)",
        "up": "+z",
        "units": {"length": "m", "angle": "rad", "time": "s"},
        "quaternion_order": "wxyz",
        "camera_axes": "intrinsic_cv / extrinsic_cv: OpenCV camera axes (x right, y down, z forward); cam2world_gl: OpenGL axes"
        + (f"; no pose of {', '.join(SCENE_CAMS)} ({NO_POSE})" if SCENE_CAMS else ""),
    },
    "budget": {"reset": RESET_BUDGET, "steps": None, "wall_clock_s": WALL_CLOCK_S},
    "episode": EPISODE,
    "robot_spec": {
        "name": ROBOT[0],
        "description": f"{ROBOT[1]}: {len(MUSCLES)} muscle actuators"
        + (f", {len(OTHER)} other actuators" if OTHER else "") + f", {len(J_NAMES)} joints",
        "base": (
            {"type": "mobile", "link": ROOT_BODY, "pose_proprio": "root_pose", "velocity_proprio": "root_vel",
             "note": "a free-standing body: nothing holds its root (the pelvis); it moves only through contacts"}
            if ROOT is not None else
            {"type": "fixed", "link": ROOT_BODY, "pose": ROOT_POSE,
             "note": "the body's root is fixed in the world (pose: x y z + wxyz quaternion)"}
        ),
        "joints": [
            {
                "name": n,
                "type": ("prismatic" if J_SLIDE[k] else "revolute") if np.isfinite(LO[k]) else "continuous",
                "unit": "m" if J_SLIDE[k] else "rad",
                "position_limits": [float(LO[k]), float(HI[k])] if np.isfinite(LO[k]) else None,
                "velocity_limit": None,
                "effort_limit": None,
                "group": BODY,
            }
            for k, n in enumerate(J_NAMES)
        ],
        "grippers": {},
        "controller": {
            "type": "muscle actuators (MuJoCo muscle model); no joint-level controller"
            + ("; plus the non-muscle actuators in action_types" if OTHER else ""),
            "control_hz": CTRL_HZ,
            "physics_hz": CTRL_HZ * SUBSTEPS,
            "gains": {},
            "note": "the command is each muscle's excitation (the muscles: static.muscles); the joints move only "
            "through actuator forces",
        },
        "tcp": {},
    },
    "kinematics": {
        "endpoint": "/kinematics",
        "joint_names": GROUPS[0][1],
        "frame": KIN_FRAME,
        "quaternion_order": "wxyz",
        "links": LINKS,
        "tcp_frames": [],
        "ee_frames": TIPS,
        "ik_targets": TIPS,
        "ik": {"tol_pos_m": 0.002, "tol_rot_rad": 3.2, "max_iters": 500},
        "note": "fingertip sites and the camera mounts for joint positions q (`joint_pos` order), in the "
        + ("root's frame (or the world's, given base_pose)" if ROOT is not None else "world frame")
        + "; ik: fingertip positions only, over all body joints",
    },
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
if TASK_INFO.get("reference"):  # MyoDex: the reference motion is the goal (relative to the object's start pose)
    SPEC["reference_motion"] = {
        "columns": ["t_s", "dx_m", "dy_m", "dz_m", "qw", "qx", "qy", "qz"],
        "rows": TASK_INFO["reference"],
        "meaning": "the reference pose of the manipulated item per control step: its centre of mass relative to where "
        "it starts (world axes) and its orientation (world frame, wxyz)",
    }
if RESET_BUDGET and not L.STATE_OPS:  # with RCB_STATE_OPS=1 the shared layer serves /reset and describes it
    SPEC["reset"] = (
        f"POST /reset (JSON or RCB1 {{cameras, video_fps, depth}}; no physics) abandons the running episode, or follows "
        f"one that has ended, and starts a new one from the same frozen starting state, {RESET_BUDGET} times in all; it "
        "returns the new episode's first frames and proprioception. Only the last episode is graded: an episode you "
        "reset away is never graded, a success ends the trial (409 after it), and when your time is up the episode then "
        "running is the one graded. The wall clock never restarts. /status and every meta add `episode`, `resets_left` "
        "and `control_steps_total` (all episodes); `control_steps` counts the current episode's."
    )
L.check_spec(SPEC)  # a spec that misses a 2.2 field or names a forbidden one never starts the episode
EP.scene_built()


def ctrl_steps() -> int:
    return int(S.steps)


def proprio_row() -> np.ndarray:
    d = S.data
    # joint_effort from the physics, not the mirror: MyoSuite steps its robot wrapper's own MjData and refreshes
    # S.data (base.mj_data) from qpos / qvel / act only, then mj_forward with ctrl = 0. The mirror's qfrc_actuator was
    # the force of a zero command: the wrist / pelvis / MPL position servos read kp * (0 - q) whatever was commanded.
    phys = S.base.robot.mj_data
    parts = [d.qpos[J_Q], d.qvel[J_V], d.act[ACT_ADR], d.actuator_length[MUS_IDX], phys.qfrc_actuator[J_V]]
    if ROOT is not None:
        parts += [d.qpos[MD.jnt_qposadr[ROOT]:MD.jnt_qposadr[ROOT] + 7], d.qvel[MD.jnt_dofadr[ROOT]:MD.jnt_dofadr[ROOT] + 6]]
    return np.concatenate(parts).astype(
        np.float32
    )


def grab(cams, want_depth: bool):
    rgb, dep = {}, {}
    for c in cams:
        rgb[c] = S.render_frame(c, (H, W))
        if want_depth:  # the same renderer (a second OSMesa context next to a live one reads garbage)
            r = S._renderer
            r.enable_depth_rendering()
            r.update_scene(S.data, camera=c)
            dep[c] = r.render().astype(np.float32).copy()
            r.disable_depth_rendering()
    return rgb, dep


def extrinsics(cams) -> dict:
    """Per frame; a scene camera has none (the recorder writes NaN rows for a camera that is not in the dict)."""
    return {c: np.asarray(camera_info(c)["extrinsic_cv"], dtype=np.float64) for c in cams if c not in SCENE_CAMS}


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
        "task": TASK,
        "protocol": L.PROTOCOL,
        "n_actions": len(ROWS),
        "episodes": E["resets"] + 1,
        "resets": E["resets"],
        "reset_budget": RESET_BUDGET,
        "control_steps_total": EP.rows,
        **EP.final_fields(),
    }


def finish(reason: str) -> None:
    """The episode is over: the task's verdict on the steps executed, then the record for the collect hook."""
    if E["done"]:
        return
    E["done"] = True
    E["reason"] = reason
    E["t_end"] = time.time()
    E["episode_control_steps"] = ctrl_steps()
    # a simulator error ends the episode: the state after it is judged by nothing (PROTOCOL §4.6), never a success
    E["success"] = bool(S.success) and reason != "error"
    EP.log(
        {"ev": "end", "reason": reason, "step": E["step"], "control_steps": ctrl_steps(), "success": bool(E["success"]),
         "episode": OPS.episode if OPS is not None else E["resets"] + 1, "journal_rows": EP.rows}
    )
    dump()


def dump() -> None:
    if E.get("dumped"):
        return
    E["dumped"] = True
    E["live_sha"] = S.state_sha()
    S.save_trajectory(
        np.asarray(ROWS, dtype=np.float64).reshape(-1, NU),
        os.path.join(DATA, "trajectory.npz"),
        meta={"source": "standard-mode episode", "reason": E["reason"]},
    )
    L._write_json(os.path.join(DATA, "episode.json"), {**summary(), "live_sha": E["live_sha"], **ops_fields()})
    write_final()


def ctrl_step(a, rec) -> bool:
    """One executed control step, journaled. True when the call must stop here."""
    running = S.step(a)
    ROWS.append(np.asarray(a, dtype=np.float64))
    stop = EP.journal(a)
    rec.tick()
    if not running:
        finish(S.reason or "done")
    return E["done"] or stop


@contextlib.contextmanager
def checking():
    """Checking a request, before anything runs: a field of the wrong type is the request's fault -- a ValueError, which
    the shared handler answers with 400 and nothing changed -- never a simulator error that ends the episode (PROTOCOL
    §4.6: malformed -> 400, no physics)."""
    try:
        yield
    except ValueError:
        raise
    except (TypeError, AttributeError, KeyError, IndexError, OverflowError) as exc:
        raise ValueError(f"malformed request: {exc!r}"[:300]) from None


def number(meta: dict, key: str, default: float) -> float:
    """A numeric field in seconds; absent or null is the default. Not a finite number: ValueError (400)."""
    v = meta.get(key)
    if v is None:
        return float(default)
    try:
        if isinstance(v, bool):
            raise TypeError
        x = float(v)
    except (TypeError, ValueError, OverflowError):
        raise ValueError(f"{key} must be a number of seconds, got {v!r}"[:200]) from None
    if not math.isfinite(x):
        raise ValueError(f"{key} must be finite")
    return x


def note_of(meta: dict) -> str:
    """The call's free-text note for the log, read before anything runs (a note of another type is logged as text)."""
    note = meta.get("note")
    return "" if note is None else str(note)[:500]


def frame_every(fps: int, video: str) -> int:
    return max(1, int(round(CTRL_HZ / max(1, fps)))) if video != "none" else 10**9  # no video: first and last frame


def recorder(cams, fps: int, meta: dict, video: str, steps: int = 0) -> L.Recorder:
    """The call's recorder, after its frame budget: the images a call of `steps` control steps would hold and return
    (the first and the last frame plus one every frame_every control steps, for each camera; per-frame depth counted
    again) are refused above MAX_FRAMES (ValueError: 400, before anything runs)."""
    every = frame_every(fps, video)
    L.check_frames(len(cams), 2 + steps // every, L.depth_mode(meta) in ("frames", "legacy"), MAX_FRAMES)
    return L.Recorder(
        cams, every, grab, proprio_row, extrinsics, ctrl_steps, depth=L.depth_mode(meta), clock_hz=CTRL_HZ
    )


def video_fps(meta: dict) -> int:
    fps = meta.get("video_fps")
    if fps is None:
        return 10
    if isinstance(fps, bool) or not isinstance(fps, (int, float)) or not (1 <= fps <= CTRL_HZ):
        raise ValueError(f"video_fps must be in [1, {CTRL_HZ:g}]")
    return int(fps)


def cameras_of(meta: dict) -> list:
    cams = meta.get("cameras") or DEFAULT_CAMS
    if not isinstance(cams, (list, tuple)) or not all(isinstance(c, str) for c in cams):
        raise ValueError(f"cameras must be a list of camera names; cameras: {CAMS}")
    bad = [c for c in cams if c not in CAMS]
    if bad:
        raise ValueError(f"unknown cameras {bad}; cameras: {CAMS}")
    return list(dict.fromkeys(cams))  # each camera once (POST /reset does not pass the layer's parse_request)


def meta_common(cams) -> dict:
    n = ctrl_steps()
    return {
        "step": E["step"],
        "calls": E["calls"],
        "control_steps": n,
        "physics_steps": n * SUBSTEPS,
        "sim_time_s": round(n * DT, 4),
        "time_s": round(L.agent_time(E), 2),
        "time_left_s": max(0.0, round(WALL_CLOCK_S - L.agent_time(E), 1)),
        "proprio": {"layout": PROPRIO_LAYOUT, "now": proprio_row().tolist()},
        "done": E["done"],
        "reason": E["reason"],
        "cameras": {c: camera_info(c) for c in cams},
        **reset_counters(),
    }


def last_command() -> np.ndarray:
    return ROWS[-1].copy() if ROWS else S.rest_action()


def do_submit(meta: dict, A: np.ndarray):
    with checking():  # every field, before a single control step runs
        typ = meta.get("type")
        if typ != ATYPE:
            raise ValueError(f"type must be {ATYPE} (not yet: {SPEC['action_types']['not_yet']})")
        dt = number(meta, "dt", DT)
        interp = meta.get("interpolate", "linear")
        video = meta.get("video", "mp4")
        cams, fps, note = cameras_of(meta), video_fps(meta), note_of(meta)
        k = dt / DT
        if A.ndim != 2 or A.shape[1] != NU:
            raise ValueError(f"{ATYPE} actions must be (T, {NU}), got {list(A.shape)}")
        if not np.all(np.isfinite(A)):
            raise ValueError("actions contain NaN/inf")
        if not (dt <= 0.5 + 1e-9 and round(k) >= 1 and abs(k - round(k)) < 1e-6):
            raise ValueError(f"dt must be a whole multiple of the {DT:g} s control period and at most 0.5 s; got {dt}")
        if A.shape[0] * dt > MAX_CHUNK_S + 1e-9:
            raise ValueError(f"chunk longer than max_chunk_s={MAX_CHUNK_S}: {A.shape[0]} rows x {dt} s")
        if not isinstance(interp, str) or interp not in ("hold", "linear"):
            raise ValueError("interpolate must be hold|linear")
        if not isinstance(video, str) or video not in ("mp4", "frames", "both", "none"):
            raise ValueError("video must be mp4|frames|both|none")
        rec = recorder(cams, fps, meta, video, steps=A.shape[0] * int(round(k)))
    E["calls"] += 1
    step0, before = E["step"], ctrl_steps()
    rec.tick(force=True)
    prev = last_command()
    n = int(round(k))
    for row in A:
        E["step"] += 1
        target = np.asarray(row, dtype=np.float64)
        stop = False
        for s in range(1, n + 1):
            a = target if (interp == "hold" or s == n) else prev + (target - prev) * (s / n)
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
        "clipped": [],
        "n_clipped": 0,
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
            "note": note,
            **({"stopped": EP.stop_reason} if EP.stopping else {}),
        }
    )
    return out, rec.parts(video, fps)


def do_observe(meta: dict):
    with checking():  # every field, before a single control step runs
        cams, fps = cameras_of(meta), video_fps(meta)
        seconds = number(meta, "seconds", 0.0)
        video = meta.get("video", "frames")
        if seconds < 0 or seconds > MAX_WATCH_S:
            raise ValueError(f"seconds must be in [0, {MAX_WATCH_S}]")
        if not isinstance(video, str) or video not in ("mp4", "frames", "both", "none"):
            raise ValueError("video must be mp4|frames|both|none")
        rec, before = recorder(cams, fps, meta, video, steps=int(round(seconds / DT))), ctrl_steps()
    if seconds > 0 and not E["done"]:
        rec.tick(force=True)
        hold = last_command()
        for _ in range(int(round(seconds / DT))):
            if ctrl_step(hold, rec):
                break
    rec.tick(force=True, last=True)
    E["calls"] += 1
    out = {
        **meta_common(cams),
        "obs_id": E["calls"],
        "seconds": seconds,
        "control_steps_watch": ctrl_steps() - before,
        "physics_steps_watch": (ctrl_steps() - before) * SUBSTEPS,
        "ok": True,
    }
    EP.log(
        {
            "ev": "observe",
            "call": E["calls"],
            "seconds": seconds,
            "cameras": cams,
            "control_steps": ctrl_steps() - before,
            **({"stopped": EP.stop_reason} if EP.stopping else {}),
        }
    )
    return out, rec.parts(video if seconds > 0 else ("frames" if video == "mp4" else video), fps)


def reset_counters() -> dict:
    if not RESET_BUDGET or OPS is not None:  # with RCB_STATE_OPS=1 the shared layer adds its own counters
        return {}
    left = 0 if (E["success"] or E.get("exiting")) else RESET_BUDGET - E["resets"]
    return {"episode": E["resets"] + 1, "resets_left": left, "control_steps_total": EP.rows}


def status() -> dict:
    return {
        "step": E["step"],
        "calls": E["calls"],
        "control_steps": ctrl_steps(),
        "done": E["done"],
        "reason": E["reason"],
        "elapsed_s": round(time.time() - E["t0"], 1),
        **reset_counters(),
    }


class Refused(Exception):
    """A /reset the service turns down (409): nothing changes."""

    def __init__(self, body: dict):
        super().__init__(body.get("error"))
        self.body = body


def do_reset(meta: dict):
    """POST /reset: a new episode from the frozen instance (the running one, if any, is abandoned, never graded)."""
    with checking():
        cams, fps = cameras_of(meta), video_fps(meta)
        # a bad depth value is a 400 before anything changes (as the frame budget would be: one frame per camera here)
        L.check_frames(len(cams), 2, L.depth_mode(meta) in ("frames", "legacy"), MAX_FRAMES)
    if E["success"]:
        raise Refused({"error": "trial finished: an episode succeeded", "reason": E["reason"], "resets_left": 0})
    if E["resets"] >= RESET_BUDGET:
        raise Refused({"error": "reset budget exhausted", "reason": "reset budget exhausted", "budget": RESET_BUDGET,
                       "resets_left": 0})
    abandoned = {"episode": E["resets"] + 1, "control_steps": ctrl_steps(), "done": E["done"], "reason": E["reason"]}
    S.reset()
    ROWS.clear()
    E["resets"] += 1
    E["reset_at"] = list(E["reset_at"]) + [EP.rows]
    E.update(done=False, reason=None, success=False, t_end=None, episode_control_steps=None, dumped=False)
    EP._ck = None  # the state changed without a journal row: the checkpoint after this call must hash it anew
    E["calls"] += 1
    EP.log({"ev": "reset", "call": E["calls"], "episode": E["resets"] + 1, "journal_rows": EP.rows,
            "abandoned": abandoned, "state_sha": S.state_sha()})
    rec = recorder(cams, fps, meta, "frames")
    rec.tick(force=True, last=True)
    out = {**meta_common(cams), "reset_id": E["resets"], "ok": True}
    return out, rec.parts("frames", fps)


def end_from_signal() -> None:
    """The collect hook's SIGUSR1: end the episode (the agent's time is over), dump it, exit so the hook can grade it."""
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


# The ablations' hooks (rcb_limited "ablations"; off unless the task sets the switches).
# RCB_STATE_QUERY=1: the state table, MyoSuite's world frame (z up; the frame of root_pose). The robot: its root body
# (the pelvis of a free-standing body, the fixed base of an arm or a hand), its height above the ground, its centre of
# mass, its own joints; its key links (pelvis, torso, head, feet; wrist, fingertips; the prosthesis's palm and
# fingertips). The task's objects: the bodies the task's observation dict reads (obs_dict obj_* / curr_obj_*, object1 /
# object2, ball), every body moved by a joint that is not the body's own; the opponent and the goalkeeper (mocap
# bodies: pose only). Scene features: the markers the instruction describes (the baoding targets, the translucent
# target object, the start and goal pillars), the world's named geoms (floor, terrain with its heights, goal, arena).
# Read off a private copy of the live state (forward kinematics there): the live simulator is never written.
OBJECT_LABELS = {"baoding": ("object1", "object2"), "soccer": ("ball",)}
MARKER_BODIES = ("target", "start", "goal")        # the markers the instructions describe (die / object, pillars)
MARKER_SITES = {"baoding": {"target1_site": "target1", "target2_site": "target2"}}   # where each ball should be now


def _object_bodies() -> dict:
    ids = [b for b in range(1, MD.nbody) if int(MD.body_mocapid[b]) < 0 and int(MD.body_jntnum[b]) > 0
           and any(j not in OWN_JOINTS for j in range(int(MD.body_jntadr[b]), int(MD.body_jntadr[b]) + int(MD.body_jntnum[b])))]
    labels = OBJECT_LABELS.get(TASK, ("object",) if len(ids) == 1 else tuple(MD.body(b).name for b in ids))
    return dict(zip(labels, ids))


# ------------------------------------------------------------------ the state table (L.STATE_QUERY)
# Read off a private MjData: each query copies the live state (qpos, qvel, act, the mocap poses) into it and runs the
# forward kinematics and the com velocities there, so a query never writes the live data and never changes the state
# hash or the next step.
_DOWN = np.array([0.0, 0.0, -1.0])
_GEOM_SHAPES = {0: "plane", 1: "hfield", 2: "sphere", 3: "capsule", 4: "ellipsoid", 5: "cylinder", 6: "box", 7: "mesh",
                8: "sdf"}
U_POS, U_QUAT, U_LIN, U_ANG = "m", "1 (unit quaternion, w x y z)", "m/s", "rad/s"
U_JPOS, U_JVEL = "rad (hinge), m (slide), unit quaternion w x y z (ball)", "rad/s, m/s; ball: rad/s about the child's axes"
U_BOX = "m: [[x, y, z] min, [x, y, z] max]"


def _pattern(name: str) -> str:
    return re.sub(r"[-+]?\d+", "#", name)


def _stem(pattern: str) -> str:
    return re.sub(r"_+", "_", pattern.replace("#", "")).strip("_-") or pattern


def _groups(names: list, at_least: int) -> list:
    """[(group name or None, [members])]: names that differ only in their numbers form a group when at least
    `at_least` share the pattern; the others stand alone."""
    by: dict = {}
    for n in names:
        by.setdefault(_pattern(n), []).append(n)
    out = []
    for pat, ns in by.items():
        if len(ns) >= at_least and "#" in pat:
            out.append((_stem(pat), ns))
        else:
            out += [(None, [n]) for n in ns]
    return out


class _Mirror:
    """The private MjData the state table reads (see above)."""

    def __init__(self, m, live):
        self.m, self.live, self.d = m, live, mujoco.MjData(m)
        if hasattr(m, "geom_aabb"):
            self.aabb = np.array(m.geom_aabb, dtype=np.float64).reshape(-1, 6)
        else:
            self.aabb = np.c_[np.zeros((m.ngeom, 3)), np.repeat(np.asarray(m.geom_rbound)[:, None], 3, 1)]
        for g in range(m.ngeom):                     # a plane: its drawn extent, flat (0: unbounded)
            if m.geom_type[g] == 0:
                self.aabb[g] = [0.0, 0.0, 0.0, m.geom_size[g][0], m.geom_size[g][1], 0.0]
        self._gid = np.zeros(1, np.int32)

    def refresh(self) -> None:
        m, src, d = self.m, self.live(), self.d
        d.qpos[:] = src.qpos
        d.qvel[:] = src.qvel
        if m.na:
            d.act[:] = src.act
        if m.nmocap:
            d.mocap_pos[:] = src.mocap_pos
            d.mocap_quat[:] = src.mocap_quat
        d.time = src.time
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        mujoco.mj_comVel(m, d)

    def vel(self, i: int, obj=mujoco.mjtObj.mjOBJ_BODY) -> np.ndarray:
        """[angular (rad/s), linear (m/s)] of a body or site, world axes, at its origin."""
        out = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, obj, int(i), out, 0)
        return out

    def site_quat(self, s: int) -> np.ndarray:
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, self.d.site_xmat[s])
        return q

    def geom_quat(self, g: int) -> np.ndarray:
        q = np.zeros(4)
        mujoco.mju_mat2Quat(q, self.d.geom_xmat[g])
        return q

    def box(self, geoms) -> list | None:
        """The world axis-aligned box around the geoms (an unbounded plane left out); None if nothing is left."""
        lo, hi, d = np.full(3, np.inf), np.full(3, -np.inf), self.d
        for g in geoms:
            a = self.aabb[g]
            if self.m.geom_type[g] == 0 and not a[3]:
                continue
            R = d.geom_xmat[g].reshape(3, 3)
            c, h = d.geom_xpos[g] + R @ a[:3], np.abs(R) @ a[3:]
            lo, hi = np.minimum(lo, c - h), np.maximum(hi, c + h)
        return [lo, hi] if np.isfinite(lo).all() else None

    def local_size(self, b: int) -> list | None:
        """Full extents [3] (m) of body b's own geoms along the body's axes (a model constant)."""
        m = self.m
        lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != b or (m.geom_type[g] == 0 and not self.aabb[g][3]):
                continue
            R = np.zeros(9)
            mujoco.mju_quat2Mat(R, m.geom_quat[g])
            R = R.reshape(3, 3)
            c, h = m.geom_pos[g] + R @ self.aabb[g][:3], np.abs(R) @ self.aabb[g][3:]
            lo, hi = np.minimum(lo, c - h), np.maximum(hi, c + h)
        return (hi - lo).tolist() if np.isfinite(lo).all() else None

    def part(self, what: str, i: int, geoms) -> dict:
        """One part of a scene-feature group: a geom's pose and extents, or a body's pose and box."""
        if what == "geom":
            return {"pos": self.d.geom_xpos[i], "quat": self.geom_quat(i), "size": 2 * self.aabb[i][3:]}
        return {"pos": self.d.xpos[i], "quat": self.d.xquat[i], "bbox": self.box(geoms)}

    def heights(self, g: int) -> dict:
        """A height field geom's surface: world x, y of the samples (the geom unrotated) and their world z."""
        m, h = self.m, int(self.m.geom_dataid[g])
        nr, nc, (sx, sy, sz, _) = int(m.hfield_nrow[h]), int(m.hfield_ncol[h]), m.hfield_size[h]
        a = int(m.hfield_adr[h])
        z = np.asarray(m.hfield_data[a:a + nr * nc]).reshape(nr, nc)
        kr, kc = max(1, -(-nr // 64)), max(1, -(-nc // 64))
        c = self.d.geom_xpos[g]
        return {"x": c[0] + np.linspace(-sx, sx, nc)[::kc], "y": c[1] + np.linspace(-sy, sy, nr)[::kr],
                "z": c[2] + sz * z[::kr, ::kc]}

    def tangible(self, g: int) -> bool:
        """A geom that collides, or that a camera shows (alpha above 0 and in a group drawn by default); an invisible
        helper that does neither is no part of the scene."""
        m = self.m
        if m.geom_contype[g] or m.geom_conaffinity[g]:
            return True
        mat = int(m.geom_matid[g])
        alpha = m.mat_rgba[mat][3] if mat >= 0 else m.geom_rgba[g][3]
        return bool(alpha > 0 and m.geom_group[g] < 3)

    def drop(self, p, skip: np.ndarray) -> float | None:
        """Distance straight down from p to the first collidable geom whose body is not in `skip` (a bool mask over
        bodies); None if there is none."""
        m, d, gid = self.m, self.d, self._gid
        p, gone = np.array(p, dtype=np.float64), 0.0
        for _ in range(32):
            dist = mujoco.mj_ray(m, d, p, _DOWN, None, 1, -1, gid)
            if dist < 0 or gid[0] < 0:
                return None
            g = int(gid[0])
            if not skip[m.geom_bodyid[g]] and (m.geom_contype[g] or m.geom_conaffinity[g]):
                return gone + dist
            step = dist + 1e-6
            gone += step
            p[2] -= step
        return None

    def joint(self, j: int, vel: bool = False):
        m, d = self.m, self.d
        if vel:
            a = int(m.jnt_dofadr[j])
            return d.qvel[a:a + 3].copy() if m.jnt_type[j] == 1 else float(d.qvel[a])
        a = int(m.jnt_qposadr[j])
        return d.qpos[a:a + 4].copy() if m.jnt_type[j] == 1 else float(d.qpos[a])

    def joint_range(self, j: int):
        m = self.m
        return [float(m.jnt_range[j][0]), float(m.jnt_range[j][1])] if m.jnt_limited[j] else None


class _MirrorTable(L.StateTable):
    """A state table whose getters read a _Mirror: one refresh per query."""

    def __init__(self, mirror: _Mirror, **kw):
        super().__init__(**kw)
        self.mirror, self.taken = mirror, set()

    def query(self, plan: dict) -> dict:
        self.mirror.refresh()
        return super().query(plan)

    def put(self, name: str, kind: str, fields: dict, note: str | None = None) -> str:
        """add() under a name not taken yet (a clash gets a numbered suffix)."""
        n, k = name, 2
        while n in self.taken:
            n, k = f"{name}_{k}", k + 1
        self.taken.add(n)
        self.add(n, kind, fields, note)
        return n


def _mj_robot(t: _MirrorTable, name: str, root: int, mask: np.ndarray, skip: np.ndarray, joints: dict,
              links: dict, extra: dict | None = None, note: str | None = None) -> None:
    """The robot (its root body `root`, the bodies in `mask`) and its key links {label: ("body" | "site", id)};
    joints {label: joint id}; height_above_ground looks straight down from the root past every body in `skip`."""
    M = t.mirror
    t.put(name, "robot", {
        "pos": (U_POS, lambda: M.d.xpos[root]), "quat": (U_QUAT, lambda: M.d.xquat[root]),
        "lin_vel": (U_LIN, lambda: M.vel(root)[3:]), "ang_vel": (U_ANG, lambda: M.vel(root)[:3]),
        "height_above_ground": ("m (null: nothing below)", lambda: M.drop(M.d.xpos[root], skip)),
        "com": (U_POS, lambda: M.d.subtree_com[root]),
        **({"joint_pos": (U_JPOS, lambda: {k: M.joint(j) for k, j in joints.items()}),
            "joint_vel": (U_JVEL, lambda: {k: M.joint(j, True) for k, j in joints.items()})} if joints else {}),
        **(extra or {})}, note)
    for label, (what, i) in links.items():
        if what == "site":
            f = {"pos": (U_POS, lambda i=i: M.d.site_xpos[i]), "quat": (U_QUAT, lambda i=i: M.site_quat(i)),
                 "lin_vel": (U_LIN, lambda i=i: M.vel(i, mujoco.mjtObj.mjOBJ_SITE)[3:]),
                 "ang_vel": (U_ANG, lambda i=i: M.vel(i, mujoco.mjtObj.mjOBJ_SITE)[:3])}
        else:
            f = {"pos": (U_POS, lambda i=i: M.d.xpos[i]), "quat": (U_QUAT, lambda i=i: M.d.xquat[i]),
                 "lin_vel": (U_LIN, lambda i=i: M.vel(i)[3:]), "ang_vel": (U_ANG, lambda i=i: M.vel(i)[:3])}
        t.put(label, "robot_link", f)


def _mj_scene(t: _MirrorTable, robot: np.ndarray, skip_name=lambda n: False, chain_min: int = 16,
              group_min: int = 8, expand: tuple = ()) -> np.ndarray:
    """Every top-level body that is not the robot's (`robot`: a bool mask over bodies; a body named in `expand` stands
    for its children instead) and the world body's named geoms:
      a body under a free joint or a mocap body: an object (bodies that differ only in their numbers, at least
      chain_min of them, are one chain: a rope);
      a body whose subtree has joints: an articulation (at least group_min alike: one entity with its parts);
      any other body with geoms, and the world body's named geoms: scene features (alike names: one entity with parts).
    Names for which skip_name() is true are left out (decoration), and so are geoms that neither collide nor show.
    Gives the bool mask over bodies of what moves freely (objects), for height_above_ground."""
    M, m = t.mirror, t.mirror.m
    parent = np.asarray(m.body_parentid)
    units = []
    for b in range(1, m.nbody):
        if parent[b] == 0 and not robot[b] and m.body(b).name and not skip_name(m.body(b).name):
            units += ([c for c in range(m.nbody) if parent[c] == b and m.body(c).name] if m.body(b).name in expand
                      else [b])
    unit_of, us = np.full(m.nbody, -1), set(units)
    for c in range(1, m.nbody):
        b = c
        while b > 0 and b not in us:
            b = int(parent[b])
        unit_of[c] = b if b > 0 else -1
    gunit, junit = unit_of[np.asarray(m.geom_bodyid)], unit_of[np.asarray(m.jnt_bodyid)]
    free_mask = np.zeros(m.nbody, bool)
    objs, arts, stat = {}, {}, {}
    for b in units:
        geoms = [int(g) for g in np.flatnonzero(gunit == b) if not skip_name(m.geom(g).name) and M.tangible(g)]
        joints = [int(j) for j in np.flatnonzero(junit == b)]
        if m.body_mocapid[b] >= 0 or (joints and m.jnt_type[joints[0]] == 0 and m.jnt_bodyid[joints[0]] == b):
            free_mask |= unit_of == b
            objs[m.body(b).name] = (b, geoms, [j for j in joints if m.jnt_type[j] != 0])
        elif joints:
            arts[m.body(b).name] = (b, geoms, joints)
        elif geoms:
            stat[m.body(b).name] = (b, geoms)
    for gname, members in _groups(list(objs), chain_min):
        if gname is None:
            b, geoms, joints = objs[members[0]]
            mocap = m.body_mocapid[b] >= 0
            f = {"pos": (U_POS, lambda b=b: M.d.xpos[b]), "quat": (U_QUAT, lambda b=b: M.d.xquat[b])}
            if not mocap:
                f |= {"lin_vel": (U_LIN, lambda b=b: M.vel(b)[3:]), "ang_vel": (U_ANG, lambda b=b: M.vel(b)[:3])}
            if (size := M.local_size(b)) is not None:
                f["size"] = ("m (full extents along the body's axes)", lambda s=size: s)
            f["bbox"] = (U_BOX, lambda g=geoms: M.box(g))
            if joints:
                f |= {"joint_pos": (U_JPOS, lambda js=joints: {m.joint(j).name: M.joint(j) for j in js}),
                      "joint_vel": (U_JVEL, lambda js=joints: {m.joint(j).name: M.joint(j, True) for j in js})}
            t.put(members[0], "object", f, "moved by the scene (a mocap body)" if mocap else None)
        else:
            bs = [objs[n][0] for n in members]
            geoms = [g for n in members for g in objs[n][1]]
            t.put(gname, "object", {
                "pos": ("m (the mean of its parts' positions)", lambda bs=bs: M.d.xpos[bs].mean(0)),
                "bbox": (U_BOX, lambda g=geoms: M.box(g)),
                "points": ("m: [[x, y, z]] each part's position, in the order of the note",
                           lambda bs=bs: M.d.xpos[bs])}, f"a chain of {len(bs)} bodies: {', '.join(members)}")
    for gname, members in _groups(list(arts), group_min):
        joints = [j for n in members for j in arts[n][2]]
        geoms = [g for n in members for g in arts[n][1]]
        f = {"joint_pos": (U_JPOS, lambda js=joints: {m.joint(j).name: M.joint(j) for j in js}),
             "joint_vel": (U_JVEL, lambda js=joints: {m.joint(j).name: M.joint(j, True) for j in js}),
             "joint_range": ("as joint_pos: [lower, upper] (null: unlimited)",
                             lambda js=joints: {m.joint(j).name: M.joint_range(j) for j in js}),
             "bbox": (U_BOX, lambda g=geoms: M.box(g))}
        if gname is None:
            b = arts[members[0]][0]
            t.put(members[0], "articulation", {"pos": (U_POS, lambda b=b: M.d.xpos[b]),
                                               "quat": (U_QUAT, lambda b=b: M.d.xquat[b]), **f})
        else:
            bs = {n: arts[n][0] for n in members}
            f["parts"] = ("{part: {pos m, quat w x y z}}",
                          lambda bs=bs: {n: {"pos": M.d.xpos[b], "quat": M.d.xquat[b]} for n, b in bs.items()})
            t.put(gname, "articulation", f, f"{len(bs)} alike bodies")
    feats = {n: ("body", b, geoms) for n, (b, geoms) in stat.items()}
    for g in range(m.ngeom):
        n = m.geom(g).name
        if m.geom_bodyid[g] == 0 and n and not skip_name(n) and n not in feats and M.tangible(g):
            feats[n] = ("geom", g, [g])
    for gname, members in _groups(list(feats), 2):
        if gname is None:
            what, i, geoms = feats[members[0]]
            if what == "geom":
                f = {"shape": ("geom type", lambda i=i: _GEOM_SHAPES.get(int(m.geom_type[i]), "other")),
                     "pos": (U_POS, lambda i=i: M.d.geom_xpos[i]), "quat": (U_QUAT, lambda i=i: M.geom_quat(i)),
                     "size": ("m (full extents along its own axes; 0: unbounded)", lambda i=i: 2 * M.aabb[i][3:])}
                if m.geom_type[i] == 1:
                    f["heights"] = ("m: {x [ncol], y [nrow], z [nrow][ncol]} world heights of the surface on a grid "
                                    "(every k-th sample, at most 64 a side)", lambda i=i: M.heights(i))
            else:
                f = {"pos": (U_POS, lambda i=i: M.d.xpos[i]), "quat": (U_QUAT, lambda i=i: M.d.xquat[i])}
            f["bbox"] = (U_BOX, lambda g=geoms: M.box(g))
            t.put(members[0], "scene_feature", f)
        else:
            parts = {n: feats[n] for n in members}
            geoms = [g for n in members for g in feats[n][2]]
            t.put(gname, "scene_feature", {
                "bbox": (U_BOX, lambda g=geoms: M.box(g)),
                "parts": ("{part: {pos m, quat w x y z, size m (full extents)} or {pos m, quat, bbox m}}",
                          lambda ps=parts: {n: M.part(*p) for n, p in ps.items()})}, f"{len(parts)} alike parts")
    return free_mask


def state_table() -> _MirrorTable:
    m = MD
    t = _MirrorTable(_Mirror(m, lambda: S.data), quaternion_order="wxyz", frame=(
        "MyoSuite's world frame: metres, z up (the frame of /spec's root_pose); a free-standing body's /kinematics "
        "base frame is this frame moved to robot.pos and turned by robot.quat"))
    rootid = np.asarray(m.body_rootid)
    robot = np.isin(rootid, sorted({int(rootid[m.jnt_bodyid[j]]) for j in J_ID}))
    skip = robot.copy()                             # height_above_ground looks past the robot and what moves freely
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)  # noqa: E731
    sid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)  # noqa: E731
    links = {}
    if BODY in ("legs", "osl"):
        links["pelvis"] = ("site", sid("pelvis")) if sid("pelvis") >= 0 else ("body", _rb)
        for n in ("torso", "head"):
            if sid(n) >= 0:
                links[n] = ("site", sid(n))
            elif bid(n) >= 0:
                links[n] = ("body", bid(n))
        for n in ("calcn_r", "toes_r", "calcn_l", "toes_l", "osl_knee_assembly", "osl_foot_assembly"):
            if bid(n) >= 0:
                links[n] = ("body", bid(n))
    else:
        for n in ("radius", "lunate"):
            if bid(n) >= 0:
                links[{"radius": "forearm", "lunate": "wrist"}[n]] = ("body", bid(n))
        if TIPS:
            links |= {n: ("site", sid(n)) for n in TIPS}
        else:
            links |= {n: ("body", bid(n)) for n in ("distal_thumb", "distph2", "distph3", "distph4", "distph5")
                      if bid(n) >= 0}
        links |= {n: ("body", bid(n)) for n in ("prosthesis/palm", "prosthesis/thumb3", "prosthesis/index3",
                                                 "prosthesis/middle3", "prosthesis/ring3", "prosthesis/pinky3")
                  if bid(n) >= 0}
    _mj_robot(t, "robot", _rb, robot, skip, {n: j for n, j in zip(J_NAMES, J_ID)}, links,
              note=f"{ROBOT[0]}: pos / quat / velocities are its root body's ({ROOT_BODY or 'unnamed'})")
    M = t.mirror
    for name, b in _object_bodies().items():
        skip |= rootid == rootid[b]
        geoms = [g for g in range(m.ngeom) if rootid[m.geom_bodyid[g]] == rootid[b] and M.tangible(g)]
        f = {"pos": (U_POS, lambda b=b: M.d.xpos[b]), "quat": (U_QUAT, lambda b=b: M.d.xquat[b]),
             "lin_vel": (U_LIN, lambda b=b: M.vel(b)[3:]), "ang_vel": (U_ANG, lambda b=b: M.vel(b)[:3]),
             "bbox": (U_BOX, lambda g=geoms: M.box(g))}
        if (size := M.local_size(b)) is not None:
            f["size"] = ("m (full extents along the body's axes)", lambda s=size: s)
        t.put(name, "object", f, None if name == m.body(b).name else f"MuJoCo body {m.body(b).name!r}")
    for b in range(1, m.nbody):
        if m.body_parentid[b] != 0 or robot[b] or skip[b]:
            continue
        geoms = [g for g in range(m.ngeom) if rootid[m.geom_bodyid[g]] == b and M.tangible(g)]
        f = {"pos": (U_POS, lambda b=b: M.d.xpos[b]), "quat": (U_QUAT, lambda b=b: M.d.xquat[b]),
             "bbox": (U_BOX, lambda g=geoms: M.box(g))}
        if m.body(b).name in MARKER_BODIES:
            t.put(m.body(b).name, "scene_feature", f, "a marker the instruction describes")
        elif m.body_mocapid[b] >= 0:
            skip |= rootid == b
            t.put(m.body(b).name, "object", f, "moved by the scene (a mocap body)")
        elif geoms and m.body(b).name:
            t.put(m.body(b).name, "scene_feature", f)
    for site, label in MARKER_SITES.get(TASK, {}).items():
        if (i := sid(site)) >= 0:
            t.put(label, "scene_feature", {"pos": (U_POS, lambda i=i: M.d.site_xpos[i])},
                  "a marker the instruction describes")
    _mj_scene(t, np.ones(m.nbody, bool))           # the world body's named geoms
    return t


# RCB_STATE_OPS=1: reset = Session.reset() (the frozen instance) and the episode's record cleared. A snapshot holds what
# the Session's own snapshot holds, taken so that a load continues exactly as the saved episode would have: the integration
# state, sensor readings and warnings of the robot's MjData and of each mirror (MyoSuite steps the robot's and refreshes
# the mirrors with its own controls), the model arrays a reset or a step edits, the env's plain attributes (obs_dict,
# rwd_dict, ...) as they were, its Python-side objects (as Session.reset restores them: controllers, opponent policies,
# the reference cursor), the robot's sensor cache, the random generator, the Session's episode counters (steps, hold
# run, track errors, verdict) and a copy of the graded rows.
_SESSION_EPISODE = ("steps", "done", "success", "reason", "solved_run", "last_info", "track_ok", "track_max", "_track0")
_MJ_STATE = mujoco.mjtState.mjSTATE_INTEGRATION


def _datas() -> list:
    b = S.base
    return [b.robot.mj_data, S.data] + ([b.obsd_mj_data] if b.obsd_mj_data is not S.data else [])


def ops_reset() -> None:
    S.reset()
    ROWS.clear()
    E.update(t_end=None, episode_control_steps=None, dumped=False)


def ops_save():
    datas = []
    for d in _datas():
        state = np.empty(mujoco.mj_stateSize(MD, _MJ_STATE))
        mujoco.mj_getState(MD, d, state, _MJ_STATE)
        datas.append((state, d.sensordata.copy(), [(w.number, w.lastinfo) for w in d.warning]))
    frozen = S._snapshot["model"]
    return {"datas": datas,
            "model": {k: v.copy() for k, v in M._model_arrays(MD).items() if not np.array_equal(v, frozen.get(k))},
            "attrs": {k: copy.deepcopy(v) for k, v in vars(S.base).items()
                      if not k.startswith("__") and M._plain(v) and k != "mujoco_render_frames"},
            "extras": S._copy_extras(), "sensor_cache": copy.deepcopy(getattr(S.base.robot, "_sensor_cache", None)),
            "rng": copy.deepcopy(S._rng().bit_generator.state),
            "episode": copy.deepcopy({k: getattr(S, k) for k in _SESSION_EPISODE if hasattr(S, k)}),
            "rows": [r.copy() for r in ROWS]}


def ops_load(snap) -> None:
    live = M._model_arrays(MD)
    for k, v in {**S._snapshot["model"], **snap["model"]}.items():
        if k in live and live[k].shape == v.shape:
            live[k][...] = v
    for d, (state, sensors, warnings) in zip(_datas(), snap["datas"]):
        mujoco.mj_setState(MD, d, state, _MJ_STATE)
        mujoco.mj_forward(MD, d)
        d.sensordata[:] = sensors
        for w, (number, info) in zip(d.warning, warnings):
            w.number, w.lastinfo = number, info
    for k, v in {**snap["attrs"], **snap["extras"]}.items():
        setattr(S.base, k, copy.deepcopy(v))
    if snap["sensor_cache"] is not None:
        S.base.robot._sensor_cache = copy.deepcopy(snap["sensor_cache"])
    S._rng().bit_generator.state = copy.deepcopy(snap["rng"])
    for k, v in copy.deepcopy(snap["episode"]).items():
        setattr(S, k, v)
    ROWS[:] = [r.copy() for r in snap["rows"]]
    E.update(t_end=None, episode_control_steps=None, dumped=False)


OPS = L.StateOps(budget=RESET_BUDGET, reset=ops_reset, save=ops_save, load=ops_load) if L.STATE_OPS else None


def ops_fields() -> dict:
    """episode.json's record of the state ops (RCB_STATE_OPS=1; {} without): it overrides episodes / resets."""
    if OPS is None:
        return {}
    return OPS.fields(bool(E["success"]), E["reason"], EP.events(), E.get("t_first"), EP.rows)


# A resume replays the journal from the frozen instance and resets where the committed resets did: their journal rows
# are in the last checkpoint (`persist`), the one Episode.begin resumes from.
_CPS = [e for e in EP.events() if e.get("ev") == "checkpoint"]
RESUME = {"at": list((_CPS[-1].get("persist") or {}).get("reset_at") or []) if _CPS else [], "i": 0}


def _resume_resets() -> None:
    while RESUME["at"] and int(RESUME["at"][0]) <= RESUME["i"]:
        RESUME["at"].pop(0)
        S.reset()
        ROWS.clear()


def replay_row(row) -> None:
    """A journaled row during a resume."""
    _resume_resets()
    S.step(row)
    ROWS.append(np.asarray(row, dtype=np.float64))
    RESUME["i"] += 1


def resume_sha() -> str:
    _resume_resets()  # a reset after the last replayed row (or before the first)
    return S.state_sha()


# The frozen initial state, checked at the episode's start (the verdict is the live episode's: no later replay checks it).
# A fresh start whose scene does not hash to the instance's recorded sha is no trial of this task: an infrastructure
# failure, before the agent is served. A resume is checked against its own checkpoint by Episode.begin.
if not any(e.get("ev") == "start" for e in EP.events()) and S.state_sha() != S.instance_sha:
    if not L.final_verdict(DATA):
        _fin = os.path.join(DATA, "final.json")
        L._write_json(_fin, {**L._read_json(_fin), "instance_mismatch": 1, "start_sha": S.state_sha(),
                             "instance_sha": S.instance_sha})
    EP.give_up(f"instance_mismatch: the scene started at {S.state_sha()}, the frozen instance records {S.instance_sha}",
               infra=True)   # writes the verdict (never overwrites one) and answers 409 until the container ends
RESUMED = EP.begin(
    state=E,
    step=replay_row,
    state_sha=resume_sha,
    proprio=proprio_row,
    spec=SPEC,
    on_end=end_from_signal,
    start_event={"task": TASK, "env": S.env_id, "cameras": CAMS, "instance_sha": S.instance_sha},
    ops=OPS,
)
if RESUMED and E["resets"]:  # Episode.begin looks for an "end" of any episode: only the current one's counts
    _ends = [e for e in EP.events() if e.get("ev") == "end" and int(e.get("episode") or 1) == E["resets"] + 1
             and int(e.get("journal_rows") or 0) <= EP.rows]
    if _ends:
        E.update(done=True, reason=_ends[-1].get("reason"), t_end=float(_ends[-1]["ts"]),
                 episode_control_steps=int(_ends[-1].get("control_steps") or ctrl_steps()))
    else:
        E.update(done=False, reason=None, t_end=None, episode_control_steps=None)
if E["done"]:
    dump()  # resumed after the episode had ended: the record is rewritten from the replay
E["ready"] = True
write_final()
open(PID_FILE, "w").write(str(os.getpid()))
print(
    f"{L.PROTOCOL} MyoSuite sim ready ({TASK}: {S.env_id}; startup {EP.n_startups}, resumes {EP.resumes})", flush=True
)


def make_handler(svc: L.Service):
    """The shared 2.2 handler, plus POST /reset when the run grants resets (the shared layer has no reset)."""
    base = L.make_handler(svc)

    class H(base):
        def do_POST(self):
            if self.path != "/reset" or not RESET_BUDGET or OPS is not None:
                return super().do_POST()
            try:
                meta, _ = self._body()
            except Exception as exc:  # noqa: BLE001
                return self._json(413 if "too large" in str(exc) else 400, {"error": str(exc)[:200]})
            with LOCK:
                if E.get("exiting"):
                    return self._json(409, {"error": "trial finished: your time is up", "reason": E.get("reason"),
                                            "resets_left": 0})
                cid = EP.call_begin(self.path, self.connection)
                out = blobs = err = None
                try:
                    out, blobs = do_reset(meta)
                except Refused as exc:
                    err = (409, exc.body)
                except ValueError as exc:
                    err = (400, {"error": str(exc)})
                except Exception as exc:  # noqa: BLE001
                    traceback.print_exc()
                    E["error"] = repr(exc)
                    EP.log({"ev": "error", "path": self.path, "error": repr(exc)})
                    try:
                        finish("error")
                    except Exception:  # noqa: BLE001
                        traceback.print_exc()
                    err = (500, {"error": "simulator error", "detail": repr(exc)[:200]})
                finally:
                    if out is not None or (err and err[0] == 500):
                        try:
                            EP.checkpoint(E, S.state_sha, proprio_row)
                        except Exception:  # noqa: BLE001
                            traceback.print_exc()
                    EP.call_end(cid, self.path)
            if err is not None:
                return self._json(*err)
            return self._rcb1({**out, "stopped": None}, blobs)

    return H


def serve(svc: L.Service, port: int) -> None:
    """L.serve with this service's handler (the same spec defaults and check, and L.bind: a port still held when the
    service restarts is waited for, not a crash)."""
    svc.spec.setdefault("on_service_error", L.ON_SERVICE_ERROR)
    svc.spec.setdefault("max_call_wall_s", EP.max_call_wall_s)
    L.check_spec(svc.spec)
    L.bind(port, make_handler(svc)).serve_forever()


try:
    serve(
        L.Service(
            spec=SPEC,
            lock=LOCK,
            state=E,
            submit=do_submit,
            observe=do_observe,
            kinematics=do_kinematics,
            status=status,
            on_submit_error=lambda exc: finish("error"),
            log=EP.log,
            episode=EP,
            state_sha=S.state_sha,
            proprio=proprio_row,
            state_ops=OPS,
            state_table=state_table() if L.STATE_QUERY else None,
        ),
        port=int(os.environ.get("RCB_PORT", "8000")),
    )
except Exception:  # noqa: BLE001
    traceback.print_exc()
    raise
