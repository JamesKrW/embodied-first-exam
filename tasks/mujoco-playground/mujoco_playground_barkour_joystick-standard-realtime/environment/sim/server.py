"""eai-standard/2.3 sim service for MuJoCo Playground's quadruped standard-realtime tasks -- the adapter behind the shared
realtime layer (`python /srv/rcb_realtime.py`, which imports this module and calls build(); docs/protocol/README.md §3,
docs/benchmarks/mujoco-playground.md).

The simulator-specific half only: `Quadruped` implements rcb_service.Sim (the contract documented in rcb_service.py) for
one frozen instance, on the benchmark's own tooling module rcb_quadruped.Session -- the same scene, command schedule,
pushes, termination rule and success check as the standard twin's eai-standard/2.2 service
(environment/sim/server.py of `-standard`). The protocol, the clock, the plan, the episode, crash recovery and the
finalize are rcb_realtime.py's and rcb_service.py's, unchanged copies of the HumanoidBench sources
(ci_checks/check-tooling-identity.sh).

Grading: the live episode's verdict (owner, 2026-10-17; nothing is replayed): step() reports the task's
check on the live simulator after every control step (rcb_quadruped.Session.check(): getup's 25 upright steps and the
stands' 100-step hold are inside it; a joystick episode passes only at the horizon), which is the rule the replay of the
recorded episode applied, so no live_verdict() is needed. `start_sha` is the frozen instance's recorded state hash:
rcb_service refuses a fresh episode that does not start there (instance_mismatch, an infrastructure failure).

What the agent gets is what the standard twin gives, minus the 2.2 additions the realtime layer does not serve
(`/kinematics`, per-frame camera poses, depth modes, the `done` action): the cameras (RGB and depth), the robot's own
proprioception (joint positions, velocities and torques, the IMU's gyro, accelerometer, gravity direction and
velocimeter, the feet's contacts) with the joystick command in force (the standard twin's `meta["command_now"]`) and joint position targets (clipped to the actuator ranges and reported). The joystick
command schedule and, for gait tracking, the gait are in /spec, as in standard mode; nothing about the pushes is.

An episode ends as the standard twin's does: after the horizon (judged then: success or failure), when the robot meets
the environment's termination rule (a failure), at a getup / stand task's success, when the agent's time is up, or on a
simulator error.

Rendering: MuJoCo on the GPU (EGL) when the container reaches the NVIDIA driver, else OSMesa. On a clock that does not
stop a render is time the simulator thread cannot step. The image's NVIDIA EGL ICD manifest may be missing (the toolkit
mounts the library under NVIDIA_DRIVER_CAPABILITIES=graphics but not always the manifest); this module writes one to
/tmp and points libglvnd at it before rcb_quadruped probes EGL (RCB_GL=egl|osmesa forces one).

Settings (environment/docker-compose.yaml): RCB_INSTANCE (the frozen instance.json), RCB_DATA, RCB_RESET_BUDGET,
RCB_WALL_CLOCK_S, RCB_SLOWDOWN (rcb_realtime / rcb_service), RCB_GL.

Ablations (rcb_service.STATE_OPS, rcb_service.STATE_QUERY; off unless the task sets them): save_state() / load_state()
keep every array of MuJoCo's data (the step's derived quantities included) and the episode's bookkeeping (targets,
control steps, the fall, the logs the check reads), exact; state_table() is the standard twin's: in MuJoCo's world frame
(quaternions w x y z), the robot's trunk pose, velocity and height above the ground, every joint, the IMU site, the feet
and the ground (the scene has no object), read from a scratch copy of the live state (MjRead) on the simulator thread
between two control steps.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def _egl_manifest() -> None:
    """Make the NVIDIA EGL driver reachable for rcb_quadruped's own EGL probe (inert without it)."""
    gl = os.environ.get("RCB_GL", "auto")
    if gl in ("egl", "osmesa"):
        os.environ["RCB_MUJOCO_GL"] = gl
        return
    if not os.path.exists("/dev/nvidiactl") or os.environ.get("__EGL_VENDOR_LIBRARY_FILENAMES"):
        return
    vendors = [p for d in ("/usr/share/glvnd/egl_vendor.d", "/etc/glvnd/egl_vendor.d") if os.path.isdir(d)
               for p in os.listdir(d)]
    if any("nvidia" in v for v in vendors):
        return
    os.makedirs("/tmp/rcb_egl", exist_ok=True)
    manifest = "/tmp/rcb_egl/10_nvidia.json"
    with open(manifest, "w") as f:
        f.write('{"file_format_version": "1.0.0", "ICD": {"library_path": "libEGL_nvidia.so.0"}}\n')
    mesa = "/usr/share/glvnd/egl_vendor.d/50_mesa.json"
    os.environ["__EGL_VENDOR_LIBRARY_FILENAMES"] = manifest + (f":{mesa}" if os.path.exists(mesa) else "")


_egl_manifest()
os.environ.pop("MUJOCO_GL", None)          # the image's default (osmesa) is not a choice: rcb_quadruped probes
os.environ.pop("PYOPENGL_PLATFORM", None)
os.environ["RCB_SESSION_LOG"] = ""
os.environ["RCB_AGENT_TRACE_DIR"] = ""
import rcb_quadruped as Q  # noqa: E402  (first: it picks the rendering backend before mujoco loads)
import mujoco  # noqa: E402

import rcb_service  # noqa: E402
from rcb_service import main  # noqa: E402

GL_BACKEND = os.environ.get("MUJOCO_GL", "?")
CAM_W, CAM_H = 320, 240                    # the standard twin's
GL2CV = np.diag([1.0, -1.0, -1.0])


def _mj_save(d) -> dict:
    """Every array of a MuJoCo MjData, as the last step left it (derived quantities too), and its time."""
    out = {"time": float(d.time)}
    for n in dir(d):
        if not n.startswith("_") and n != "plugin_data":
            v = getattr(d, n, None)
            if isinstance(v, np.ndarray):
                out[n] = v.copy()
    return out


def _mj_load(d, snap: dict) -> None:
    """_mj_save()'s arrays back into the same model's data, in place (arrays sized by the step's contacts and
    constraints are recomputed by the next step)."""
    for n, v in snap.items():
        if n != "time":
            a = getattr(d, n)
            if a.shape == v.shape and a.flags.writeable:
                a[...] = v
    d.time = snap["time"]


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



def quad_state_table(S):
    """The state-query ablation's table (RCB_STATE_QUERY=1) of Session S, built once after the scene, in MuJoCo's world
    frame (the standard twin's table): the
    quadruped (`robot`: its trunk's pose, world-frame velocity and height above the ground straight below it, by a ray
    cast that skips the robot's own geoms; every leg joint with its range), its IMU site and feet (robot_link), and the
    ground (`floor`: a plane or the rough terrain's height field). The scene has no object. Read by MjRead (a scratch
    copy of the live state, positions and velocities recomputed there)."""
    R = MjRead(mujoco, lambda: S.model, lambda: S.data).refresh()
    m = R.m
    root = int(S._root)
    robot = R.subtree(root)
    t = MjStateTable(rcb_service.StateTable(
        frame=("MuJoCo's world frame (metres, z up; the ground is the scene feature `floor`). The /kinematics trunk frame "
               "is this frame moved to robot.pos / robot.quat; proprio's IMU readings are in the frame of the IMU site, "
               "the robot_link `imu`"),
        quaternion_order="wxyz",
        note="every pose is a body's or site's frame origin; velocities are world-frame, at that origin"), R)
    joints = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in robot]
    t.add("robot", "robot", {**mj_body_fields(R, root, ground_exclude=robot), **mj_joint_fields(R, joints, ranges=True)},
          note=f"{S.robot['name']}: the trunk body {m.body(root).name!r} (floating base)")
    imu = int(S._imu)
    t.add("imu", "robot_link", mj_site_fields(R, imu), note=f"the IMU site {m.site(imu).name!r} on the trunk")
    for f in S.robot["feet"]:
        sid = m.site(f).id
        t.add(f if f.startswith("foot") else f"foot_{f}", "robot_link", mj_site_fields(R, sid), note=f"the foot site {f!r}")
    g = int(S._floor)
    kind = "hfield" if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_HFIELD) else "plane"
    size = m.hfield_size[int(m.geom_dataid[g])].copy() if kind == "hfield" else m.geom_size[g].copy()
    t.add("floor", "scene_feature",
          {"pos": ("m", lambda: R.d.geom_xpos[g].copy()), "type": ("plane or hfield", lambda: kind),
           "size": ("m (plane: half-sizes x, y and grid spacing; hfield: half-sizes x, y, the top's height, the base's "
                    "depth)", lambda: size)},
          note="the ground geom `floor`")
    return t


class Quadruped:
    """One frozen quadruped instance behind the rcb_service.Sim contract (see the module docstring)."""

    def __init__(self, instance: str):
        self.instance = instance
        self.s = Q.Session(instance, check_hash=False)            # the start is checked by the layer (start_sha), as a give-up
        self.start_sha = str(self.s.inst.get("state_sha") or "")  # the frozen instance's recorded t = 0 state hash
        m = self.m = self.s.model
        self.control_dt = float(self.s.control_dt)
        self.max_control_steps = int(self.s.horizon)
        self.raw_dim = Q.N
        self.cameras = tuple(self.s.cameras)
        self.depth_cameras = self.cameras
        feet = self.s.robot["feet"]
        j = self.s.joint_names
        self.proprio_layout = (list(j) + [f"{n}_vel" for n in j] + [f"{n}_effort" for n in j]
                               + ["gyro_x", "gyro_y", "gyro_z", "acc_x", "acc_y", "acc_z", "gravity_x", "gravity_y",
                                  "gravity_z", "linvel_x", "linvel_y", "linvel_z"] + [f"{f}_contact" for f in feet] + ["cmd_vx", "cmd_vy", "cmd_wz"])
        self._renderer = None
        self._depth = None
        self.cmd = self.s.targets.copy()
        self.images([self.cameras[0]], False)                     # warms up the GL context

    # ------------------------------------------------------------------ the rcb_service.Sim contract
    def spec(self) -> dict:
        s, m = self.s, self.m
        limits = dict(zip(s.joint_names, np.stack([s.lower, s.upper], axis=1).round(4).tolist()))
        sched = [] if s.commands is None else [
            {"from_s": round(int(c["start_step"]) * self.control_dt, 2), "from_step": int(c["start_step"]),
             "vx_mps": round(c["vx"], 4), "vy_mps": round(c["vy"], 4), "wz_radps": round(c["wz"], 4)}
            for c in s.inst["commands"]]
        episode = {
            "joystick": f"an episode lasts {s.horizon} control steps (the {s.horizon * self.control_dt:g} s command "
                        f"schedule) and is judged then; it ends early as a failure when the robot meets its termination "
                        f"rule ({s.spec.get('term_text')}), or when your time is up",
            "getup": f"an episode ends the moment the robot has stood upright for {Q.HOLD_STEPS} control steps in a row "
                     f"(success), after {s.horizon} control steps, or when your time is up",
            "stand_on_two": f"an episode ends the moment the pose has been held for {Q.POSE_HOLD_STEPS} control steps in "
                            f"a row (success), when the robot meets the termination rule (a failure), after {s.horizon} "
                            "control steps, or when your time is up",
        }[s.kind]
        cams = {}
        for c in self.cameras:
            i = m.camera(c).id
            f = 0.5 * CAM_H / math.tan(math.radians(float(m.cam_fovy[i])) / 2)
            follows = int(m.cam_mode[i]) != 0
            cams[c] = {"hw": [CAM_H, CAM_W], "intrinsic_cv": [[f, 0.0, CAM_W / 2], [0.0, f, CAM_H / 2], [0.0, 0.0, 1.0]],
                       "fovy_deg": round(float(m.cam_fovy[i]), 2),
                       "mounted": ("follows the robot's centre of mass at a fixed offset; it does not turn with the robot"
                                   if follows else "fixed to the robot's head: moves and turns with it")}
        out = {
            "simulator": "MuJoCo 3.3.7 (MuJoCo Playground's scene, plain MuJoCo on the CPU)",
            "robot": s.robot["name"],
            "task_text": s.inst["text"],
            "episode": episode,
            "action_types": {"joint_position": {
                "dim": Q.N, "layout": list(s.joint_names), "lower": s.lower.tolist(), "upper": s.upper.tolist(),
                "units": "rad",
                "semantics": "absolute targets of the 12 joint position actuators, one per control step; rows dt apart "
                             "(a whole multiple of control_dt), `hold` or `linear` between rows; values outside the "
                             "limits are clipped and reported"}},
            "cameras": cams,
            "proprio": {"joint_pos": "measured joint positions (rad), in the action's order",
                        "*_vel": "measured joint velocities (rad/s)", "*_effort": "actuator torques (N m)",
                        "gyro_*": "angular velocity of the trunk, IMU frame (rad/s)",
                        "acc_*": "specific force at the IMU, IMU frame (m/s^2; +9.81 up at rest)",
                        "gravity_*": "unit vector of gravity's direction in the IMU frame ([0, 0, -1] when level)",
                        "linvel_*": "linear velocity of the trunk at the IMU, IMU frame (m/s; the IMU's velocimeter)",
                        "*_contact": "1 when the foot touches the ground, else 0",
                        "cmd_*": "the joystick command in force at the next control step (vx, vy in m/s in the robot's "
                                 "own frame, yaw rate in rad/s; 0 without a schedule): command_schedule's row"},
            "command_schedule": sched,
            "world": {"frame": "z up; the robot starts where the episode puts it (no world coordinates are given)",
                      "up": "+z", "units": {"length": "m", "angle": "rad", "time": "s"}},
            "robot_spec": {"name": s.robot["name"], "joints": list(s.joint_names), "joint_limits": limits,
                           "joint_order": s.robot["joint_text"], "home_pose": dict(zip(s.joint_names, s.home.tolist())),
                           "feet": list(s.robot["feet"]),
                           "controller": {"type": "joint position actuators (PD)",
                                          "kp_nm_per_rad": float(m.actuator_gainprm[0, 0]),
                                          "kd_nms_per_rad": float(m.dof_damping[6] - m.actuator_biasprm[0, 2])},
                           "physics_substeps": int(s.substeps)},
            "depth": {"cameras": list(self.cameras), "units": "z-depth in metres, pixel for pixel with the camera's "
                      "image; noise-free"},
            "gl_backend": GL_BACKEND,
        }
        if s.gait:
            out["gait"] = {k: s.gait[k] for k in ("name", "frequency_hz", "foot_height_m", "phase0", "phase_dt")} | {
                "feet": list(s.robot["feet"]),
                "rule": "foot i's target height at control step t is get_rz(wrap(phase0[i] + t * phase_dt)) with "
                        "Playground's gait.get_rz (a cubic Bezier from 0 up to foot_height_m over the first half of the "
                        "phase, then down); phases in rad, wrapped to [-pi, pi)"}
        return out

    def interpolable(self, typ: str) -> bool:
        return True

    def images(self, cameras: list[str], depth: bool) -> tuple[dict, dict]:
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.m, CAM_H, CAM_W)
        r, rgb, dep = self._renderer, {}, {}
        for c in cameras:
            r.update_scene(self.s.data, camera=c)
            rgb[c] = r.render().copy()
            if depth and c in self.depth_cameras:
                r.enable_depth_rendering()
                r.update_scene(self.s.data, camera=c)
                z = np.asarray(r.render(), dtype=np.float32).copy()
                r.disable_depth_rendering()
                dep[c] = np.where(np.isfinite(z) & (z > 0), z, 0.0).astype(np.float32)
        return rgb, dep

    def proprio(self) -> np.ndarray:
        s, d = self.s, self.s.data
        return np.concatenate([d.qpos[7:], d.qvel[6:], d.actuator_force, s.sensor("gyro"), s.sensor("accelerometer"),
                               s.gravity(), s.sensor("local_linvel"), s.foot_contacts(), s.command_now()]).astype(np.float32)

    def clip(self, typ: str, rows: np.ndarray) -> tuple[np.ndarray, list]:
        lo, hi = self.s.lower, self.s.upper
        out = np.clip(rows, lo, hi)
        report = [{"row": int(i), "index": int(j), "reason": f"outside [{lo[j]:.4g}, {hi[j]:.4g}]"}
                  for i, j in np.argwhere(out != rows)]
        return out, report

    def to_sim(self, typ: str, row: np.ndarray) -> np.ndarray:
        return np.asarray(row, dtype=np.float64)

    def command(self, typ: str) -> np.ndarray:
        return self.cmd

    def hold(self) -> tuple[str, np.ndarray]:
        return "joint_position", self.cmd

    def step(self, row: np.ndarray) -> tuple[bool, bool, dict]:
        chk = self.s.step(row)
        self.cmd = np.asarray(row, dtype=np.float64).copy()
        s = self.s
        fell = s.kind in ("joystick", "stand_on_two") and s.fell_at is not None
        success = bool(chk["success"]) and (s.kind != "joystick" or s.done)
        return bool(fell or s.done or success), success, {}

    def reset(self) -> None:
        self.s.reset()
        self.cmd = self.s.targets.copy()

    def state_sha(self) -> str:
        return self.s.state_sha()

    # ------------------------------------------------------------------ ablations (rcb_service.STATE_OPS, STATE_QUERY)
    def save_state(self) -> dict:
        s = self.s
        return {"data": _mj_save(s.data), "targets": s.targets.copy(), "steps": int(s.steps), "fell_at": s.fell_at,
                "log": list(s.log), "gait_log": list(s.gait_log), "pose_log": list(s.pose_log),
                "trace": None if s._trace is None else list(s._trace), "cmd": self.cmd.copy()}

    def load_state(self, snap: dict) -> None:
        s = self.s
        _mj_load(s.data, snap["data"])
        s.targets, s.steps, s.fell_at = snap["targets"].copy(), int(snap["steps"]), snap["fell_at"]
        s.log, s.gait_log, s.pose_log = list(snap["log"]), list(snap["gait_log"]), list(snap["pose_log"])
        s._trace = None if snap["trace"] is None else list(snap["trace"])
        self.cmd = snap["cmd"].copy()

    def state_table(self):
        """RCB_STATE_QUERY: the robot, its IMU and feet, the ground (quad_state_table), read on the simulator thread."""
        return quad_state_table(self.s)

    def save_trajectory(self, rows: np.ndarray, path: str, meta: dict) -> None:
        self.s.save_trajectory(rows, path, meta)

    def render_video(self, rows: np.ndarray, path: str) -> None:
        """The episode's video, after the verdict: one pass over the rows from the frozen start on this simulator."""
        self.s.render_video(rows, path)


def build() -> Quadruped:
    return Quadruped(os.environ.get("RCB_INSTANCE", os.path.join(HERE, "instance.json")))


if __name__ == "__main__":
    main(build)
