"""eai-standard/2.1 sim service for HumanoidBench -- the sidecar's entry point (`python /srv/server.py`).

The simulator-specific half only: `HumanoidBench` implements rcb_service.Sim for one frozen instance. The protocol,
the episode, crash recovery and the finalize live in rcb_service.py, which imports no simulator. The verdict is the live
episode's: step() reports HumanoidBench's own rule (the episode's return reached the pass mark) and the
episode ends there, so nothing is replayed to grade it; render_video() re-runs the trajectory once, after the verdict, to
film it. start_sha is the frozen instance's starting state as instance.npz records it, checked when an episode begins.
Settings come from environment/docker-compose.yaml:
  RCB_TASK, RCB_INSTANCE   which HumanoidBench task, which frozen scene (the privileged twin's instance)
  RCB_TASK_TEXT            one sentence for /spec
  RCB_EPISODE              what ends an episode, for /spec (the instruction's words)
  RCB_OBS_PROPRIO          1 (standard): + joint angles and velocities; 0: images only
  RCB_OBS_IMU              1 (standard): + a pelvis IMU (angular velocity, gravity direction)
  RCB_OBS_ROOM_CAMERA      1 (standard): + `room_camera`, a camera fixed in the room
  RCB_OBS_DEPTH            0 (standard); 1: + depth from both head cameras, as from an RGB-D camera
  RCB_ROOM_CAMERA_POSE     where room_camera stands, "lookat_x,lookat_y,lookat_z,distance,azimuth,elevation" (a MuJoCo
                           free camera, metres and degrees); empty: the recording camera frozen at t=0
  RCB_IMAGE_HW             the side of every camera's image in pixels; empty: upstream's 256
  RCB_RESET_BUDGET, RCB_WALL_CLOCK_S, RCB_DATA   the service's own (rcb_service.main)

Ablations (off unless the task sets them; nothing here runs without them):
  rcb_service.STATE_QUERY   state_table(): the robot (its pelvis pose and velocity, height above the ground under it, the
                            centre of mass, every joint) and its key links (pelvis, torso, head, hands, feet, the
                            fingertips of a hand that has them); the task's free bodies (objects); its jointed bodies
                            (the door and its hatch: articulations, with joint ranges); its static bodies (barriers,
                            each hurdle, shelves, tables, the targets the instruction shows) and the floor as scene
                            features. MuJoCo's world frame, read off a private copy of the live state (forward
                            kinematics there): the live simulator is never written.
  rcb_service.STATE_OPS     save_state() / load_state(): every array of MuJoCo's data (derived quantities included), the
                            model arrays tasks write (body_pos, body_quat, geom_rgba), the task's own state (vars(task),
                            as Session's frozen instance keeps it), the time limit's step count, the episode's return
                            and step count, and the command in force: exact, in memory.
"""
from __future__ import annotations

import copy
import hashlib
import itertools
import math
import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# rcb_hb first: it picks MuJoCo's GL backend, which `import mujoco` binds once per process. Imported after it, the
# depth renderer's context ended up inside the viewer's and head-camera images came back as another camera's.
from rcb_hb import GL_BACKEND, Session  # noqa: E402
import mujoco  # noqa: E402
from rcb_rewards import restore  # noqa: E402

restore()  # the scoring formula lives in this sidecar only, and must be back before a Session is opened
import rcb_service  # noqa: E402
from rcb_service import InstanceMismatch, main  # noqa: E402
from humanoid_bench.tasks import Task  # noqa: E402

# HumanoidBench's own non-privileged sensors (humanoid_bench/wrappers.py, ObservationWrapper): `image` renders
# these two head cameras; `proprio` is the robot's joint angles and velocities, without the root pose.
CAMERAS = ("left_eye_camera", "right_eye_camera")
# The head cameras' near plane. MuJoCo's default is 1% of the scene's extent (5.5 cm in door), and the crouch agents
# brace against the door puts an eye 4 cm from it: the door was clipped away and the eye saw the floor behind it.
HEAD_NEAR_M = 0.01
DEPTH_MAX_M = 10.0   # how far the depth reaches (a RealSense D435i, the H1's head RGB-D camera, reads to about 10 m)
IMU = ("imu_gyro_x", "imu_gyro_y", "imu_gyro_z", "imu_gravity_x", "imu_gravity_y", "imu_gravity_z")
TASK_SHARED = ("_env", "robot", "unwrapped")   # the task's back-references (rcb_hb._TASK_STATE_SKIP): never copied


# ------------------------------------------------------------------ the state table (rcb_service.STATE_QUERY)
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


class _MirrorTable(rcb_service.StateTable):
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


class HumanoidBench:
    """One frozen HumanoidBench instance behind the rcb_service.Sim contract. The agent never gets the root pose,
    object or goal state, or the reward -- the reward is live here only so the service can judge."""

    def __init__(self, task: str | None, instance: str, *, proprio: bool = True, imu: bool = False,
                 room_camera: bool = False, room_pose: tuple | None = None, depth: bool = False, task_text: str = "",
                 episode: str = "", image_hw: int | None = None):
        self.task, self.instance = task, instance
        self.s = Session(task, instance=instance, enable_render=True, image_hw=image_hw)
        if not self.s.scored:
            raise RuntimeError("the reward was not restored: this service could not judge success")
        # The frozen start as instance.npz records it: the state the Session restores (state_sha() hashes the same
        # snapshot), checked by rcb_service when an episode begins; and the file's own digest of the whole instance
        # (pose, goal, task state), which a damaged or foreign file fails.
        with np.load(instance, allow_pickle=False) as inst:
            self.start_sha = hashlib.sha1(np.ascontiguousarray(np.asarray(inst["state"], dtype=np.float64))
                                          .tobytes()).hexdigest()[:12]
            recorded = str(inst["sha"]) if "sha" in inst.files else None
        if recorded is not None and recorded != self.s.instance_sha():
            raise InstanceMismatch(f"{instance} records instance sha {recorded}, its contents hash to "
                                   f"{self.s.instance_sha()}")
        self.task_text, self.episode = task_text, episode
        self.robot, m = self.s.low.robot, self.s.model
        assert m.jnt_type[0] == mujoco.mjtJoint.mjJNT_FREE and m.body(int(m.jnt_bodyid[0])).name == "pelvis"
        self.joints = [m.joint(j).name for j in range(m.njnt) if 7 <= m.jnt_qposadr[j] < self.robot.dof]
        self.nu = self.raw_dim = int(self.s.nu)
        self.control_dt, self.max_control_steps = float(self.s.low.dt), int(self.s.max_steps)
        self.lo, self.hi = (np.asarray(v, dtype=np.float64) for v in m.actuator_ctrlrange[:self.nu].T)
        self.proprio_on, self.imu = proprio, imu
        self.proprio_layout = ([f"{j}_pos" for j in self.joints] + [f"{j}_vel" for j in self.joints] if proprio else []) \
            + (list(IMU) if imu else [])
        self.depth_cameras = CAMERAS if depth else ()
        self._depth_renderer = None
        # reach and push override upstream's Task.render() to draw their goal as a marker; every image here goes
        # through the same viewer, so running that hook puts the goal in the head and room cameras too.
        self._marks = type(self.s.low.task).render is not Task.render
        self.cameras = CAMERAS + (("room_camera",) if room_camera else ())
        self._room = (room_pose or self._frozen_camera(self.s.low.camera_name or "cam_default")) if room_camera else None
        self._axes, self._anchors, self._links, self._camera_facts = self._hardware()
        self.image_hw = list(self._render(CAMERAS[0]).shape[:2])   # also warms up the GL context
        self.cmd = self._command_now()

    def _command_now(self) -> np.ndarray:
        """The servo targets the actuators hold now, in the action's normalised units."""
        return 2.0 * (np.asarray(self.s.data.ctrl[:self.nu], dtype=np.float64) - self.lo) / (self.hi - self.lo) - 1.0

    def _frozen_camera(self, name: str) -> tuple:
        """`room_camera`: the task's recording camera (which follows the robot) frozen where it is at the
        instance's initial state -- a tripod in the room. Kept as free-camera parameters for the shared
        offscreen viewer; pixel-identical to the recording camera at t=0 (checked on sit_simple)."""
        m = self.s.model
        d = mujoco.MjData(m); d.qpos[:], d.qvel[:] = self.s.data.qpos, self.s.data.qvel
        mujoco.mj_forward(m, d)                                       # a scratch copy: the live sim is untouched
        c, dist = m.camera(name).id, 3.0                              # the free camera orbits a point 3 m ahead
        fwd = -d.cam_xmat[c].reshape(3, 3)[:, 2]
        return d.cam_xpos[c] + dist * fwd, dist, math.degrees(math.atan2(fwd[1], fwd[0])), math.degrees(math.asin(fwd[2]))

    def _hardware(self) -> tuple[list, list, dict, dict]:
        """What a real robot's documentation (its URDF) gives, read off the model: each joint's axis and a point on
        it, each link's parent, mass and centre of mass, each foot's collision shape as a box, and each head
        camera's pose, all in the pelvis frame with every joint at 0; and where room_camera stands in the robot's
        starting frame (origin on the floor under the pelvis at t = 0, x the way the robot faces then, z up). The
        robot is the pelvis's subtree: the scene's objects are subtrees of their own, so none of them is here."""
        m, r3 = self.s.model, lambda v: [round(float(x), 3) for x in v]
        d = mujoco.MjData(m)
        d.qpos[:] = m.qpos0
        d.qpos[7:self.robot.dof] = 0                                  # a scratch copy: the live sim is untouched
        mujoco.mj_forward(m, d)
        pelvis = m.body("pelvis").id
        R, p = d.xmat[pelvis].reshape(3, 3), d.xpos[pelvis]
        axes = [r3(R.T @ d.xaxis[m.joint(n).id]) for n in self.joints]
        anchors = [r3(R.T @ (d.xanchor[m.joint(n).id] - p)) for n in self.joints]
        robot = [b for b in range(m.nbody) if m.body_rootid[b] == pelvis]

        def box(b):   # the link's colliding geoms' bounding boxes, corner by corner, in the pelvis frame
            pts = [d.geom_xpos[g] + d.geom_xmat[g].reshape(3, 3) @ (m.geom_aabb[g, :3] + m.geom_aabb[g, 3:] * np.array(c))
                   for g in range(m.ngeom) if m.geom_bodyid[g] == b and (m.geom_contype[g] or m.geom_conaffinity[g])
                   for c in itertools.product((-1, 1), repeat=3)]
            q = np.array([R.T @ (x - p) for x in pts])
            return [r3(q.min(0)), r3(q.max(0))]
        links = {"frame": "pelvis (x forward, y left, z up), every joint at 0", "names": [m.body(b).name for b in robot],
                 "parent": [None if b == pelvis else m.body(int(m.body_parentid[b])).name for b in robot],
                 "mass_kg": [round(float(m.body_mass[b]), 3) for b in robot],   # a massless link (the bare H1's hands):
                 "com_m": [r3(R.T @ ((d.xipos[b] if m.body_mass[b] > 0 else d.xpos[b]) - p)) for b in robot],   # its origin
                 "feet": {m.body(b).name: box(b) for b in robot if "ankle" in m.body(b).name},
                 "feet_units": "each foot's collision shape as a box: [min xyz, max xyz], m"}
        cams = {}
        for c in CAMERAS:
            i, C = m.camera(c).id, d.cam_xmat[m.camera(c).id].reshape(3, 3)   # a camera looks down its -z
            cams[c] = {"mounted": "head", "fovy_deg": round(float(m.cam_fovy[i]), 1),
                       "pose": {"frame": "pelvis (x forward, y left, z up), every joint at 0",
                                "position_m": r3(R.T @ (d.cam_xpos[i] - p)), "forward": r3(R.T @ -C[:, 2]),
                                "up": r3(R.T @ C[:, 1]), "right": r3(R.T @ C[:, 0])}}   # right: the image's +x
        if self._room is not None:
            lookat, dist, az, el = self._room
            a, e = math.radians(az), math.radians(el)                 # MuJoCo's free camera, as the viewer places it
            fwd = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), math.sin(e)])
            up = np.array([-math.sin(e) * math.cos(a), -math.sin(e) * math.sin(a), math.cos(e)])
            x0, R0 = self.s.data.xpos[pelvis], self.s.data.xmat[pelvis].reshape(3, 3)
            yaw = math.atan2(R0[1, 0], R0[0, 0])
            Y = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            cams["room_camera"] = {"mounted": "room (fixed)", "fovy_deg": round(float(m.vis.global_.fovy), 1), "pose": {
                "frame": "the robot's start: origin on the floor under the pelvis at t = 0, x the way it faces, y left, z up",
                "position_m": r3(Y.T @ (np.asarray(lookat) - dist * fwd - [x0[0], x0[1], 0])),
                "forward": r3(Y.T @ fwd), "up": r3(Y.T @ up), "right": r3(Y.T @ np.cross(fwd, up))}}
        return axes, anchors, links, cams

    def _render(self, camera: str) -> np.ndarray:
        r = self.s.low.mujoco_renderer
        if self._marks:
            if r.viewer is None or not r.viewer._markers:
                self.s.low.task.render()                              # adds the marker, which the viewer keeps
            for mk in r.viewer._markers:                              # and, like that hook on every call, puts it on the
                if mk["objid"] == 789:                                # goal: a reset gives task.goal a new array (push's
                    mk["pos"] = self.s.low.task.goal                  # reset_model first rewrites the old one in place)
        if camera != "room_camera":
            vis, znear = self.s.model.vis.map, self.s.model.vis.map.znear
            vis.znear = min(znear, HEAD_NEAR_M / self.s.model.stat.extent)   # the room camera keeps the default
            try:
                return r.render("rgb_array", camera_name=camera)
            finally:
                vis.znear = znear
        cam = r.viewer.cam                                            # the viewer exists: __init__ rendered once
        cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = self._room
        return r.render("rgb_array", camera_id=-1)                   # -1 = the free camera just set

    def _depth(self, camera: str) -> np.ndarray:
        """What an RGB-D camera in a head camera's place reads, noise-free: metres along its forward axis (planar
        depth), pixel for pixel with its image; 0 where nothing is within DEPTH_MAX_M. MuJoCo's own renderer:
        the viewer's gives no metric depth, and its markers (reach's and push's goal) are pictures, not things."""
        m = self.s.model
        if self._depth_renderer is None:
            self._depth_renderer = mujoco.Renderer(m, *self.image_hw)
            self._depth_renderer.enable_depth_rendering()
        vis, znear = m.vis.map, m.vis.map.znear
        vis.znear = min(znear, HEAD_NEAR_M / m.stat.extent)           # as the head cameras' images
        try:
            self._depth_renderer.update_scene(self.s.data, camera=camera)
            d = self._depth_renderer.render()
        finally:
            vis.znear = znear
            self.s.low.mujoco_renderer.viewer.make_context_current()  # the viewer's GL context again, for its images
        return np.where(d < DEPTH_MAX_M, d, 0.0).astype(np.float32)

    def _imu(self) -> list[float]:
        """What an IMU on the pelvis reads, noise-free: the angular velocity in the pelvis frame (the root free
        joint's qvel[3:6] already is that), and the gravity direction in that frame (tilt, not heading)."""
        rot = np.zeros(9); mujoco.mju_quat2Mat(rot, self.s.data.qpos[3:7])
        return [*self.s.data.qvel[3:6], *(rot.reshape(3, 3).T @ [0.0, 0.0, -1.0])]

    # ------------------------------------------------------------------ the rcb_service.Sim contract
    def spec(self) -> dict:
        m = self.s.model
        tendons = any(m.actuator_trntype[i] == mujoco.mjtTrn.mjTRN_TENDON for i in range(self.nu))
        return {
            "simulator": "humanoidbench", "robot": self.s.env_id.split("-")[0], "task_text": self.task_text,
            "episode": self.episode,
            "action_types": {"joint_position": {
                "dim": self.nu, "layout": [m.actuator(i).name for i in range(self.nu)],
                "lower": [-1.0] * self.nu, "upper": [1.0] * self.nu,
                "range_rad": [[round(float(lo), 4), round(float(hi), 4)] for lo, hi in zip(self.lo, self.hi)],
                "units": "HumanoidBench's own action: position-servo targets, -1 and +1 commanding the two ends of each "
                         "actuator's range (range_rad), linearly, so 0 is mid-range, not 0 rad"
                         + ("; a tendon actuator's range is the summed angle of the two joints it moves" if tendons else ""),
                "dtype": "float32: the simulator applies each target as a 32-bit float",
                "semantics": "rows dt apart (a multiple of control_dt); each control step's targets are the row (hold) or "
                             "a straight line from the previous row's (linear); values outside [-1, 1] are clipped and "
                             "reported"}},
            "cameras": {c: {**self._camera_facts[c], "hw": self.image_hw} for c in self.cameras},
            "proprio": {"joints": self.joints, "units": "*_pos rad, *_vel rad/s", "axis": self._axes,
                        "axis_frame": "the joint's rotation axis in the pelvis frame, every joint at 0; a positive "
                                      "angle turns the joint's child link about it by the right-hand rule: about an axis "
                                      "near +y (left), a link hanging below the joint swings backward",
                        "position_m": self._anchors, "position_frame": "a point on that axis (the joint's centre), m, same frame"}
            if self.proprio_on else None,
            "imu": {"mounted": "pelvis", "frame": "pelvis: x forward, y left, z up",
                    "gyro": "imu_gyro_*: angular velocity, rad/s, pelvis frame, right-hand rule: y > 0 while pitching "
                            "forward (nose down), x > 0 while rolling to the right, z > 0 while turning left",
                    "gravity": "imu_gravity_*: unit gravity direction, pelvis frame: (0, 0, -1) when upright; it tips "
                               "toward the way the pelvis leans: x > 0 leaning forward, y > 0 leaning left"} if self.imu else None,
            "links": self._links,
            "depth": {"cameras": list(CAMERAS), "units": "metres along the camera's forward axis (planar depth, as an "
                      "RGB-D camera reports it), pixel for pixel with that camera's image; noise-free",
                      "range_m": [HEAD_NEAR_M, DEPTH_MAX_M], "nothing_in_range": 0.0} if self.depth_cameras else None,
            "gl_backend": GL_BACKEND,
        }

    def interpolable(self, typ: str) -> bool:
        return True

    def images(self, cameras: list[str], depth: bool) -> tuple[dict, dict]:
        rgb = {c: self._render(c) for c in cameras}
        dep = {c: self._depth(c) for c in cameras if depth and c in self.depth_cameras}
        return rgb, dep

    def proprio(self) -> np.ndarray:
        own = [*self.robot.joint_angles(), *self.robot.joint_velocities()] if self.proprio_on else []
        return np.array(own + (self._imu() if self.imu else []), dtype=np.float32)

    def clip(self, typ: str, rows: np.ndarray) -> tuple[np.ndarray, list]:
        out = np.clip(rows, -1.0, 1.0)
        report = [{"row": int(i), "index": int(j), "reason": "outside [-1, 1]"} for i, j in np.argwhere(out != rows)]
        # At the precision the simulator applies (to_sim), which is also what command() gives back: a chunk then
        # interpolates from exactly the targets the same rows sent one by one would start from.
        return out.astype(np.float32).astype(np.float64), report

    def to_sim(self, typ: str, row: np.ndarray) -> np.ndarray:
        return np.asarray(row, dtype=np.float32).astype(np.float64)   # what Session.step applies

    def command(self, typ: str) -> np.ndarray:
        return self.cmd

    def hold(self) -> tuple[str, np.ndarray]:
        return "joint_position", self.cmd

    def step(self, row: np.ndarray) -> tuple[bool, bool, dict]:
        done = self.s.step(row)[2]
        self.cmd = np.asarray(row, dtype=np.float64)
        return done, bool(self.s.status()["success"]), {}

    def reset(self) -> None:
        self.s.reset()
        self.cmd = self._command_now()

    def state_sha(self) -> str:
        return self.s.state_sha()

    # ------------------------------------------------------------------ ablations (rcb_service.STATE_OPS, STATE_QUERY)
    def state_table(self) -> _MirrorTable:
        m = self.s.model
        t = _MirrorTable(_Mirror(m, lambda: self.s.data), quaternion_order="wxyz", frame=(
            "MuJoCo's world frame of the task: metres, z up. /spec's joint axes, links and IMU are in the pelvis "
            "frame: this frame moved to robot.pos and turned by robot.quat; room_camera's pose is in the robot's "
            "starting frame (this frame moved to the floor under the pelvis at t = 0 and turned to the way it faced)"))
        pelvis = m.body("pelvis").id
        robot = np.asarray(m.body_rootid) == pelvis
        skip = robot.copy()                         # height_above_ground looks past the robot and the free bodies
        sid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, n)  # noqa: E731
        bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)  # noqa: E731
        links = {"pelvis": ("body", pelvis), "torso": ("body", m.body("torso_link").id)}
        for label, site in (("head", "head"), ("left_hand", "left_hand"), ("right_hand", "right_hand"),
                            ("left_foot", "left_foot"), ("right_foot", "right_foot")):
            if sid(site) >= 0:
                links[label] = ("site", sid(site))
        for side in ("lh", "rh"):                   # the dexterous hands' fingertips (h1hand)
            for f in ("ff", "mf", "rf", "lf", "th"):
                if bid(f"{side}_{f}distal") >= 0:
                    links[f"{side}_{f}distal"] = ("body", bid(f"{side}_{f}distal"))
        _mj_robot(t, "robot", pelvis, robot, skip, {j: m.joint(j).id for j in self.joints}, links,
                  note="the H1: pos / quat / velocities are the pelvis's")
        skip |= _mj_scene(t, robot, expand=("hurdle_array",))
        return t

    def _shared(self) -> dict:
        """A deep copy's memo: what the task's state refers to but is not part of it (the env, the robot, MuJoCo)."""
        low = self.s.low
        return {id(o): o for o in (low, low.robot, low.task, self.s.env, self.s.model, self.s.data,
                                   getattr(low, "mujoco_renderer", None)) if o is not None}

    def _time_limits(self) -> list:
        """The gym wrappers with a step count (TimeLimit), outermost first."""
        out, e = [], self.s.env
        while e is not None and e is not self.s.low:
            if hasattr(e, "_elapsed_steps"):
                out.append(e)
            e = getattr(e, "env", None)
        return out

    def save_state(self) -> dict:
        s, m, task = self.s, self.s.model, self.s.low.task
        return {"data": _mj_save(s.data), "body_pos": m.body_pos.copy(), "body_quat": m.body_quat.copy(),
                "geom_rgba": m.geom_rgba.copy(),
                "task": copy.deepcopy({k: v for k, v in vars(task).items() if k not in TASK_SHARED}, self._shared()),
                "elapsed": [w._elapsed_steps for w in self._time_limits()], "ret": s._ret, "steps": s._steps,
                "last_info": copy.deepcopy(s._last_info, self._shared()), "cmd": self.cmd.copy()}

    def load_state(self, snap: dict) -> None:
        s, m = self.s, self.s.model
        m.body_pos[:], m.body_quat[:], m.geom_rgba[:] = snap["body_pos"], snap["body_quat"], snap["geom_rgba"]
        vars(s.low.task).update(copy.deepcopy(snap["task"], self._shared()))
        _mj_load(s.data, snap["data"])
        for w, n in zip(self._time_limits(), snap["elapsed"]):
            w._elapsed_steps = n
        s._ret, s._steps = snap["ret"], snap["steps"]
        s._last_info = copy.deepcopy(snap["last_info"], self._shared())
        self.cmd = snap["cmd"].copy()

    def save_trajectory(self, rows: np.ndarray, path: str, meta: dict) -> None:
        self.s.save_trajectory(rows, path, meta)

    def render_video(self, rows: np.ndarray, path: str) -> None:
        """The episode's video, after the verdict: the trajectory re-run once on this simulator, from the frozen start."""
        self.s.render_video(rows, path, every=5, fps=20)


def build() -> HumanoidBench:
    env = os.environ.get
    pose = [float(v) for v in env("RCB_ROOM_CAMERA_POSE", "").split(",") if v.strip()]
    if pose and len(pose) != 6:
        raise SystemExit("RCB_ROOM_CAMERA_POSE needs 6 numbers: lookat_x,lookat_y,lookat_z,distance,azimuth,elevation")
    return HumanoidBench(env("RCB_TASK"), env("RCB_INSTANCE", "/srv/instance.npz"),
                         proprio=env("RCB_OBS_PROPRIO", "1") == "1", imu=env("RCB_OBS_IMU", "1") == "1",
                         room_camera=env("RCB_OBS_ROOM_CAMERA", "1") == "1", depth=env("RCB_OBS_DEPTH", "0") == "1",
                         room_pose=(np.array(pose[:3]), *pose[3:]) if pose else None, task_text=env("RCB_TASK_TEXT", ""),
                         episode=env("RCB_EPISODE", ""), image_hw=int(env("RCB_IMAGE_HW") or 0) or None)


if __name__ == "__main__":
    main(build)
