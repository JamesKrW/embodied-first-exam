"""eai-standard/2.1 sim service for Embodied Olympics - Fun -- the sidecar's entry point (`python /srv/server.py`).

The simulator-specific half only: `RoboFun` implements rcb_service.Sim for one frozen instance, through rcb_rfo.Episode
(the Unitree G1 or H1-2 in the family's venue, with the family's referee). The protocol, the episode, crash recovery
and the finalize live in rcb_service.py, which imports no simulator.
Grading (owner, 2026-10-17): the live episode's referee call is the verdict; nothing is replayed to grade
it. The referee latches a success the moment it happens and ends the attempt, so the live rule is the one the replay
applied (no hold, no settle window). The frozen start is checked on the live simulator: build() refuses an image whose
world does not start at the instance's recorded state hash (InstanceMismatch), and `start_sha` makes the service void a
graded episode begun by a reset that did not return there (both instance_mismatch, infrastructure). live_verdict()
records the live referee's call (referee.json, final.json `referee`); the video is rendered from the trajectory after the
verdict.
Settings come from environment/docker-compose.yaml:
  RCB_INSTANCE     the frozen instance (the privileged twin's instance.json)
  RCB_RESET_BUDGET, RCB_WALL_CLOCK_S, RCB_DATA   the service's own
What leaves this process: the robot's own sensors (its two head cameras, with depth on the left one, its joint encoders
and motor torques, its two IMUs, whether each hand holds something), the venue's fixed camera, and, once an
attempt is over, the referee's call (meta["outcome"]: "success" or the rule that ended the attempt) with the number of
the task's stages reached. Never an object's pose, the robot's position in the venue, or the referee's state during an episode.

Ablations (off unless the task sets them; nothing here runs without them):
  rcb_service.STATE_QUERY   state_table(): the robot (its pelvis pose and velocity, height above the ground under it, the
                            centre of mass, every joint, what each hand grips) and its key links (pelvis, torso, head,
                            palms, feet, a real hand's fingertips); every free body of the venue (balls, implements,
                            cones, props; a rope as one chain); every jointed fixture (ball machines, gates, nets, the
                            board, the bag) with its joints; the field's static geometry (lines, mats, pits, beams,
                            bars, water) as scene features, decoration left out. World frame, read off a private copy of
                            the live state (forward kinematics there): the live simulator is never written. Never the
                            referee's state.
  rcb_service.STATE_OPS     save_state() / load_state(): the episode's sim rows since the frozen start, re-run from a
                            reset on load: the world's controllers, policies, grips, the family's own state and the
                            referee's latches and record are all part of what they rebuild (the same rows from the same
                            start give the same state, which the service's own crash recovery relies on). Exact (the
                            state hash, which covers the referee's record, is checked after the re-run); a load costs one
                            control step per saved row.
"""
from __future__ import annotations

import json
import os
import re
import sys

import mujoco
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import rcb_service  # noqa: E402
from rcb_rfo import IMAGE_HW, Episode, load_instance, replay  # noqa: E402
from rcb_service import main  # noqa: E402

# The referee's call belongs to every response once an attempt is over, not only to the /submit whose row ended it:
# an attempt may end while the agent watches (/observe with seconds). rcb_service builds every response's meta in
# Service.meta(); the shared file stays as it is, and the call is added here.
_service_meta = rcb_service.Service.meta


def _meta_with_call(self) -> dict:
    return {**_service_meta(self), **(self.sim.referee_call() if self.done else {})}


rcb_service.Service.meta = _meta_with_call


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


DEPTH_CAMERAS = ("head_left",)      # the G1's head depth camera (Intel RealSense D435i)
DEPTH_MAX_M = 10.0


class RoboFun:
    """One frozen Embodied Olympics - Fun instance behind the rcb_service.Sim contract."""

    def __init__(self, instance: str):
        self.instance = load_instance(instance)
        self.ep = Episode(self.instance)
        if not self.ep.instance_check(self.instance)["ok"]:
            raise rcb_service.InstanceMismatch("this image does not rebuild the frozen instance's starting state "
                                               f"({self.ep.t0_sha} != {self.instance['t0'].get('state_sha')})")
        self.start_sha = self.instance["t0"].get("state_sha") or self.ep.t0_sha   # the frozen start (rcb_service checks it)
        t0, texts = self.instance["t0"], self.instance["texts"]
        self.task_text = texts["sentence"]
        self.rules = texts["rules"]
        self.control_dt, self.max_control_steps = self.ep.control_dt, self.ep.horizon
        self.cameras = tuple(self.ep.cameras)
        self.depth_cameras = tuple(c for c in DEPTH_CAMERAS if c in self.cameras)
        self.proprio_layout = list(self.ep.proprio_layout)
        self.raw_dim = self.ep.raw_dim
        self.mounts, self.facts = t0["cameras"], t0["robot"]
        self.data = os.environ.get("RCB_DATA", "/data")
        self._rows: list = []                      # rcb_service.STATE_OPS: the sim rows since the frozen start

    # ------------------------------------------------------------------ the rcb_service.Sim contract
    def spec(self) -> dict:
        types = self.ep.actions.spec()
        for t in types.values():
            t["semantics"] = ("rows dt apart (a multiple of control_dt); each control step's command is the row (hold) "
                              "or a straight line from the command in force (linear); values outside lower/upper are "
                              "clipped and reported; a joint target moves at most the joint's speed limit per step; "
                              "joints a type does not name keep their current targets")
        return {
            "simulator": "Embodied Olympics - Fun simulator (MuJoCo 3.3.7 on the CPU, 1 ms physics, 10 ms control)",
            "robot": self.facts["model"],
            "task_text": self.task_text,
            "rules": self.rules,
            "episode": (f"one attempt: it ends at the success, at a rule violation (a fall, a foul; the rules say "
                        f"which), or after {self.instance['time_limit_s']:g} s ({self.max_control_steps} control steps)"),
            "action_types": types,
            "start_command": self.instance["t0"]["hold"],     # the command in force at t = 0 (type and row)
            "units": "vx, vy: m/s in the robot's own frame (x forward, y left); wz: rad/s (+ turns left); joint targets: "
                     "rad; *_dq: rad/s; *_tau: N m; *_kp: N m/rad; *_kd: N m s/rad; " + self._grip_units(),
            "cameras": {c: {"hw": list(IMAGE_HW), "fovy_deg": self.mounts[c]["fovy_deg"], "mounted": self.mounts[c]["mounted"],
                            "pose_t0": {"frame": "the robot at t = 0: origin on the floor under the pelvis, x the way it "
                                                 "faces, y left, z up",
                                        **{k: self.mounts[c][k] for k in ("position_m", "forward", "up", "right")}}}
                        for c in self.cameras},
            "depth": {"unit": "m", "max_m": DEPTH_MAX_M, "note": "distance along the camera's axis; 0 = nothing within max_m"},
            "robot_facts": self.facts,
            "proprio": {"units": "*_pos, *_target: rad; *_vel: rad/s; *_torque: N m (the motor torque applied); "
                                 "imu_*: orientation quaternion (w, x, y, z; world from sensor), angular velocity "
                                 "(rad/s) and acceleration (m/s^2, gravity included) in the sensor's frame; holding_*: "
                                 "1 while that hand grips something; mode: 0 stand, 1 walk, 2 run, 3 run_arms, 4 joints / "
                                 "lowcmd, 5 walk_falcon, 6 walk_agile (the G1's); cmd_*: the velocity command in force; time_s: "
                                 "the episode's time" + ("; jaw_opening_* (m), jaw_speed_* (m/s, + opening), jaw_force_* "
                                 "(N, the clamp force, + closing): a gripper's jaws; its holding_* is 1 while both jaws "
                                 "touch the same object" if self.facts.get("real_hands") else "")},
            "referee": "once an attempt is over, every response's meta carries the referee's call: meta['outcome'] "
                       "('success' or the rule that ended it) and meta['stages_reached'] of meta['stages_total']",
        }

    def _grip_units(self) -> str:
        real = self.facts.get("real_hands", {}).get("sides", [])
        if not real:
            return "grip_left, grip_right: >= 0.5 closes the hand's grip, < 0.5 opens it"
        out = [f"grip_{s}: the {s} gripper's closure command, 0 (open, 90 mm) to 1 (closed)" for s in real]
        out += [f"grip_{s}: >= 0.5 closes the {s} hand's grip, < 0.5 opens it" for s in ("left", "right")
                if s not in real]
        return "; ".join(out)

    def interpolable(self, typ: str) -> bool:
        return True

    def images(self, cameras: list[str], depth: bool) -> tuple[dict, dict]:
        rgb = {c: self.ep.image(c) for c in cameras}
        dep = {}
        if depth:
            for c in cameras:
                if c in self.depth_cameras:
                    z = self.ep.image(c, depth=True).astype(np.float32)
                    z[z > DEPTH_MAX_M] = 0.0
                    dep[c] = z
        return rgb, dep

    def proprio(self) -> np.ndarray:
        return self.ep.proprio()

    def clip(self, typ: str, rows: np.ndarray) -> tuple[np.ndarray, list]:
        t = self.ep.actions.types[typ]
        out = self.ep.clip(typ, rows)
        lo, hi = t["bounds"]
        report = [{"row": int(i), "index": int(j), "reason": f"{t['layout'][j]} outside [{lo[j]:g}, {hi[j]:g}]"}
                  for i, j in np.argwhere(out != rows)]
        return out, report

    def to_sim(self, typ: str, row: np.ndarray) -> np.ndarray:
        return self.ep.to_raw(typ, np.asarray(row, dtype=np.float64))

    def command(self, typ: str) -> np.ndarray:
        return self.ep.actions.current(typ, self.ep.world)

    def hold(self) -> tuple[str, np.ndarray]:
        return self.ep.hold()

    def referee_call(self) -> dict:
        """The referee's call on the attempt as it stands (given to the agent only once the attempt is over)."""
        v = self.ep.verdict()
        return {"outcome": self.ep.outcome(), "stages_reached": len(v["stages"]), "stages_total": v["stages_total"]}

    def step(self, row: np.ndarray) -> tuple[bool, bool, dict]:
        done = self.ep.step_raw(row)
        if rcb_service.STATE_OPS:
            self._rows.append(np.array(row, dtype=np.float64))
        return done, self.ep.success(), (self.referee_call() if done else {})

    def reset(self) -> None:
        self.ep.reset()
        if rcb_service.STATE_OPS:
            self._rows = []

    # ------------------------------------------------------------------ ablations (rcb_service.STATE_OPS, STATE_QUERY)
    def state_table(self) -> _MirrorTable:
        w = self.ep.world
        m, r = w.m, w.robot
        x0, y0, yaw0 = w.task.start
        t = _MirrorTable(_Mirror(m, lambda: w.d), quaternion_order="wxyz", frame=(
            f"the venue: metres, x and y level, z up, the floor at z = 0; the robot started at x = {x0:.3f}, "
            f"y = {y0:.3f}, facing {yaw0:.4f} rad from +x (+ turns left). /spec's camera pose_t0 frame is this frame "
            f"moved to ({x0:.3f}, {y0:.3f}, 0) and turned by that angle about z"))
        robot = np.asarray(m.body_rootid) == r.pelvis
        skip = robot.copy()                         # height_above_ground looks past the robot and the free bodies
        joints = {m.joint(j).name.removesuffix("_joint"): j for j in range(m.njnt)
                  if robot[m.jnt_bodyid[j]] and m.jnt_type[j] != 0}
        links = {"pelvis": ("body", r.pelvis), "torso": ("body", r.torso), "head": ("site", r.head_site),
                 **{f"{s}_hand": ("site", r.palm[s]) for s in ("left", "right")},
                 **{f"{s}_foot": ("site", r.foot_site[s]) for s in ("left", "right")}}
        parent = np.asarray(m.body_parentid)
        for side in w.real_hands:                   # a real hand's fingertips: the leaves under its wrist link
            wrist = int(parent[r.mount[side]])
            under = [b for b in range(m.nbody) if b not in (wrist, r.mount[side]) and self._under(parent, b, wrist)]
            for b in under:
                if not (parent[under] == b).any():
                    links[m.body(b).name] = ("body", b)
        _mj_robot(t, "robot", r.pelvis, robot, skip, joints, links, extra={
            "held": ("{side: the name of the body that hand grips, or null}", lambda: dict(w.held))},
            note=f"{self.facts['model']}: pos / quat / velocities are the pelvis's")
        skip |= _mj_scene(t, robot, skip_name=lambda n: n.startswith("rfo_"))
        return t

    @staticmethod
    def _under(parent: np.ndarray, b: int, top: int) -> bool:
        while b > 0:
            if b == top:
                return True
            b = int(parent[b])
        return False

    def save_state(self) -> dict:
        return {"rows": np.asarray(self._rows, dtype=np.float64).reshape(-1, self.raw_dim), "sha": self.state_sha()}

    def load_state(self, snap: dict) -> None:
        self.reset()
        for row in snap["rows"]:
            self.step(row)
        if self.state_sha() != snap["sha"]:
            raise RuntimeError("re-running the saved rows did not reach the saved state")

    def state_sha(self) -> str:
        return self.ep.state_sha()

    def live_verdict(self, rows: np.ndarray, success: bool) -> dict:
        """The live referee's record of the graded episode, for people and the dashboard: stages, the rule that ended
        it, metrics, events (referee.json). It adds no rule: the referee's live call is the verdict."""
        v = self.ep.verdict()
        rec = {**v, "outcome": self.ep.outcome(), "executed_rows": int(len(rows)), "events": self.ep.world.ref.events[-200:]}
        try:
            with open(os.path.join(self.data, "referee.json"), "w") as f:
                json.dump(rec, f, default=str)
        except OSError:
            pass
        return {"referee": {k: v[k] for k in ("success", "done", "failed", "stages_total", "progress", "time_s")}
                | {"stages_reached": len(v["stages"]), "outcome": self.ep.outcome()}}

    def save_trajectory(self, rows: np.ndarray, path: str, meta: dict) -> None:
        np.savez(path, actions=np.asarray(rows, dtype=np.float64).reshape(-1, self.raw_dim), family=self.ep.family,
                 **{k: str(v) for k, v in meta.items()})

    def render_video(self, rows: np.ndarray, path: str) -> None:
        replay(self.instance, rows, video=path)


def build() -> RoboFun:
    return RoboFun(os.environ.get("RCB_INSTANCE", "/srv/instance.json"))


if __name__ == "__main__":
    main(build)
