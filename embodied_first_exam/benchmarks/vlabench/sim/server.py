"""eai-standard/2.1 sim service for VLABench -- the sidecar's entry point (`python /srv/server.py`).

The simulator-specific half only: `VLABench` implements rcb_service.Sim for one frozen instance (vlb_tool.Session).
The protocol, the episode, crash recovery and the finalize live in rcb_service.py (copied from main's HumanoidBench
tasks), which imports no simulator.
Settings come from environment/docker-compose.yaml:
  RCB_INSTANCE            the frozen instance (the privileged twin's instance.json)
  RCB_TASK_TEXT           the task as the instruction states it, for /spec
  RCB_EPISODE             what ends an episode, for /spec (the instruction's words)
  RCB_MAX_CONTROL_STEPS   technical episode cap (9000 = 15 min of robot time): the video's pass over the episode
                          must finish inside the collect hook's timeout
  RCB_IMAGE_HW            the side of every camera's image in pixels (320)
  RCB_RESET_BUDGET, RCB_WALL_CLOCK_S, RCB_DATA   the service's own (rcb_service.main)

The verdict is the live episode's: step() reports VLABench's own success check (the task's termination
condition) at every control step and the episode ends there, so nothing is replayed to grade it; render_video() re-runs
the trajectory once, after the verdict, to film it. The frozen start: vlb_tool.Session refuses a scene whose initial
state or compiled model differs from instance.json (InstanceMismatch here), and start_sha (the instance's
initial_state_sha256) is checked by rcb_service when an episode begins.

Ablations (off unless the task sets them; nothing here runs without them):
  rcb_service.STATE_QUERY   state_table(): the privileged state in the world frame of /spec "frames" (quaternions
                            w x y z): the Panda's base pose and arm and finger joints, its tcp (proprio ee pose); the
                            task's entities (Session.state()["entities"] names, neutralised by entity_names()): free
                            ones as objects (pose, velocity, size, bbox), those with joints as articulations, the others
                            as scene features. A query reads
                            a scratch copy of the live MuJoCo state (MjRead); it takes no control step. Never an
                            entity's grasp keypoints, place point or the task's conditions.
  rcb_service.STATE_OPS     save_state() / load_state(): every array of MuJoCo's data (derived quantities included), the
                            plain values (flags, counters, numbers) of the environment, the native task, its entities and
                            its condition tree (the event flags state_sha hashes among them), and the Session's targets,
                            recorded rows and end flag. In memory, by name, so a state saved before a reset (which
                            rebuilds the scene) loads into the rebuilt one. MuJoCo's state is exact; the task's Python state
                            is exact as far as it is held in plain values.
"""

from __future__ import annotations

import os
import re
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import vlb_tool  # noqa: E402
import rcb_service  # noqa: E402
from rcb_service import InstanceMismatch, main  # noqa: E402

CAMERAS = ("forward", "left", "right", "wrist")
ROBOT_BASE = (0, -0.4, 0.78)  # VLABench's franka base; a task series may override it (the instance's episode.robot)


def frame(spec: dict) -> str:
    base = ", ".join(f"{float(v):g}" for v in spec["episode"].get("robot", {}).get("position", ROBOT_BASE))
    return (
        f"world frame: metres; the robot's base is at ({base}) facing +y: x to the robot's right, y forward "
        "(away from the base, across the table), z up; quaternions w x y z"
    )


# The Panda's documented joint limits (rad). VLABench's model sets none (its arm actuators accept any angle), so they
# are given as facts, not enforced.
PLAIN = (bool, int, float, np.bool_, np.number)   # the attribute values save_state() keeps (never an object or array)


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
    """_mj_save()'s arrays back into a data of the same model (reset() rebuilds an identical one), in place (arrays
    sized by the step's contacts and constraints are recomputed by the next step)."""
    for n, v in snap.items():
        if n != "time":
            a = getattr(d, n)
            if a.shape == v.shape and a.flags.writeable:
                a[...] = v
    d.time = snap["time"]


def _logic_objects(env) -> dict:
    """The objects whose plain attribute values are the native task's state, by a name stable across rebuilds: the
    environment, the task, each entity, each node of the task's condition tree (vlb_tool.logic_state's walk)."""
    out = {"env": env, "task": env.task, **{f"entity/{n}": e for n, e in sorted(env.task.entities.items())}}

    def walk(obj, prefix):
        if obj is None:
            return
        out[prefix] = obj
        for attr in ("conditions", "condition_sets", "_conditions"):
            children = getattr(obj, attr, [])
            if isinstance(children, (list, tuple)):
                for i, child in enumerate(children):
                    walk(child, f"{prefix}/{i}")

    walk(env.task.conditions, "conditions")
    return out


PANDA_LIMITS = [[-2.8973, 2.8973], [-1.7628, 1.7628], [-2.8973, 2.8973], [-3.0718, -0.0698], [-2.8973, 2.8973],
                [-0.0175, 3.7525], [-2.8973, 2.8973]]  # fmt: skip


# rcb_service.STATE_QUERY: the state table's entity names. VLABench's own names can state a role ("target_container",
# "cooked_food"), the asset split ("microwave_seen") or a card's face ("10_of_spades"). entity_names() keeps a name
# unless it does: a playing card is card_<k> and a card holder card_holder_<k> (VLABench's holder numbers follow its
# card order); an entity whose name states a role, and every entity of its class, is
# <class>_<k> (the class in snake case); k counts within the group in the order of the start positions (x, then y,
# then z); a "_seen" / "_unseen" suffix is dropped.
ROLE_WORDS = re.compile(r"target|distract|cooked|goal")
CLASS_NAMES = {"Poker": "card", "CardHolder": "card_holder", "PlaceMat": "placemat"}


def entity_names(ents: dict) -> dict:
    """{VLABench name: table name} for {VLABench name: (class name, start position [3])}."""
    snake = lambda c: CLASS_NAMES.get(c) or re.sub(r"(?<!^)(?=[A-Z])", "_", c).lower()  # noqa: E731
    group = {c for c, _ in ents.values() if c in ("Poker", "CardHolder")} | \
        {c for n, (c, _) in ents.items() if ROLE_WORDS.search(n)}
    out = {}
    for c in sorted(group):
        base = snake(c)
        members = sorted((n for n, (k, _) in ents.items() if k == c),
                         key=lambda n: tuple(round(float(v), 3) for v in ents[n][1]))
        for i, n in enumerate(members, 1):
            out[n] = f"{base}_{i}"
    used = set(out.values())
    for n in ents:
        if n not in out:
            new = re.sub(r"_(seen|unseen)$", "", n)
            if new in used or (new != n and new in ents):
                new = n
            k = 1
            while new in used:
                k += 1
                new = f"{n}_{k}"
            out[n] = new
            used.add(new)
    return out


# rcb_service.STATE_QUERY: the state table (VLABench.state_table), read by MjRead.
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


class VLABench:
    """One frozen VLABench instance behind the rcb_service.Sim contract. The agent never gets an entity's pose, the
    task's conditions or its success predicate; the predicate is live here only so the service can judge."""

    def __init__(self, instance: str, *, task_text: str = "", episode: str = "", image_hw: int = 320,
                 max_control_steps: int = 9000):  # fmt: skip
        self.instance = instance
        try:
            self.s = vlb_tool.Session(instance)
        except (RuntimeError, ValueError) as exc:   # the scene does not rebuild the frozen instance (vlb_tool's checks)
            refused = ("differs from the frozen instance", "another VLABench commit", "solved at the start")
            if any(k in str(exc) for k in refused):
                raise InstanceMismatch(str(exc)) from exc
            raise
        self.start_sha = self.s.spec["initial_state_sha256"]   # rcb_service checks it when an episode begins
        self.task_text = task_text or self.s.instruction
        self.episode = episode
        self.hw = int(image_hw)
        self.control_dt = float(self.s.dt)
        self.max_control_steps = int(max_control_steps)
        self.cameras = CAMERAS
        self.depth_cameras = CAMERAS
        self.raw_dim = vlb_tool.N
        self.lower, self.upper = self.s.lower.copy(), self.s.upper.copy()  # VLABench's own actuator bounds
        joints = vlb_tool.ACTION_NAMES[:7]
        self.proprio_layout = (
            [f"{j}_pos" for j in joints]
            + [f"{j}_vel" for j in joints]
            + ["finger1_pos", "finger2_pos", "tcp_x", "tcp_y", "tcp_z", "tcp_qw", "tcp_qx", "tcp_qy", "tcp_qz"]
            + [f"cmd_{a}" for a in vlb_tool.ACTION_NAMES]
        )

    # ---------------------------------------------------------------- what the agent may know
    def spec(self) -> dict:
        cams = {}
        for c in CAMERAS:
            cal = self.s.calibration(c, self.hw, self.hw)
            cams[c] = {"hw": [self.hw, self.hw], "intrinsic_cv": cal["intrinsic_cv"], "cam2world_gl": cal["cam2world_gl"],
                       "mounted": "on the hand (moves with it)" if c == "wrist" else "fixed"}  # fmt: skip
        return {
            "simulator": "VLABench (MuJoCo 3.2.2, dm_control)",
            "robot": "Franka Emika Panda (7 arm joints, 2-finger gripper)",
            "task_text": self.task_text,
            "episode": self.episode,
            "frames": frame(self.s.spec),
            "action_types": {
                "joint_position": {
                    "dim": vlb_tool.N,
                    "layout": vlb_tool.ACTION_NAMES,
                    "lower": self.lower.tolist(),
                    "upper": self.upper.tolist(),
                    "units": "joint1..joint7: rad; finger1, finger2: m (0 closed .. 0.04 open)",
                    "panda_joint_limits_rad": PANDA_LIMITS,
                    "dtype": "float64",
                    "semantics": (
                        "absolute position targets for VLABench's own position actuators, applied for one control "
                        "step (0.1 s, 100 physics substeps); clipped to [lower, upper]. The model enforces no arm joint limits: stay "
                        "inside panda_joint_limits_rad"
                    ),
                },
                "not_yet": ["ee_pose", "joint_delta"],
            },
            "cameras": cams,
        }

    def interpolable(self, type: str) -> bool:
        return True

    def images(self, cameras: list[str], depth: bool) -> tuple[dict, dict]:
        rgb = {c: self.s.render(c, self.hw, self.hw) for c in cameras}
        dep = {c: self.s.render(c, self.hw, self.hw, depth=True).astype(np.float32) for c in cameras} if depth else {}
        return rgb, dep

    def proprio(self) -> np.ndarray:
        st, ph = self.s.env, self.s.physics
        fingers = [st.robot.mjcf_model.find("joint", f"finger_joint{i}") for i in (1, 2)]
        return np.concatenate(
            [
                st.robot.get_qpos(ph),
                st.robot.get_qvel(ph),
                ph.bind(fingers).qpos,
                st.get_ee_pos(),
                st.get_ee_quat(),
                self.s.targets,
            ]
        ).astype(np.float32)

    # ---------------------------------------------------------------- actions
    def clip(self, type: str, rows: np.ndarray) -> tuple[np.ndarray, list]:
        out = np.clip(rows, self.lower, self.upper)
        clipped = [
            {"row": int(r), "index": int(i), "reason": "outside [lower, upper]"}
            for r, i in zip(*np.nonzero(out != rows), strict=True)
        ]
        return out, clipped

    def to_sim(self, type: str, row: np.ndarray) -> np.ndarray:
        return np.asarray(row, dtype=np.float64)

    def command(self, type: str) -> np.ndarray:
        return self.s.targets.copy()

    def hold(self) -> tuple[str, np.ndarray]:
        return "joint_position", self.s.targets.copy()

    def step(self, row: np.ndarray) -> tuple[bool, bool, dict]:
        st = self.s.step(row)
        return st["done"], st["success"], {}

    def reset(self) -> None:
        self.s.reset()

    def state_sha(self) -> str:
        return self.s.state_sha()

    # ---------------------------------------------------------------- ablations (rcb_service.STATE_OPS, STATE_QUERY)
    def state_table(self):
        """The state-query ablation's table, world frame of /spec "frames". Robot: the Panda's base link (fixed base)
        with its arm and finger joints; link: the tcp (env.get_ee_pos / get_ee_quat, the proprio ee pose); the task's
        entities, each its root body: free -> object, with hinge or slide joints -> articulation, else scene feature.
        Read by MjRead (a scratch copy of the live state); a reset rebuilds the scene with the same model."""
        import mujoco

        ph = self.s.env.physics
        R = MjRead(mujoco, lambda: self.s.env.physics.model.ptr, lambda: self.s.env.physics.data.ptr).refresh()
        m = R.m
        env = self.s.env

        def top(b):
            while int(m.body_parentid[b]) != 0:
                b = int(m.body_parentid[b])
            return b

        def root_of(entity) -> int:
            return int(ph.bind(entity.mjcf_model.worldbody).element_id)

        t = MjStateTable(rcb_service.StateTable(
            frame=frame(self.s.spec) + "; the frame of the proprio ee pose and the cameras' calibration",
            quaternion_order="wxyz",
            note="a pose is a body's frame origin (an entity's: its root, as VLABench's get_xpos / get_xqaut); "
                 "velocities are world-frame, at that origin"), R)
        rjoints = env.robot.mjcf_model.find_all("joint")
        rid = [int(ph.bind(j).element_id) for j in rjoints]
        root = int(m.body_parentid[int(m.jnt_bodyid[rid[0]])])   # the arm's base link (the body joint1 turns on)
        sub = R.subtree(top(root))
        jf = mj_joint_fields(R, rid, ranges=True)
        for k in list(jf):
            unit, get = jf[k]
            jf[k] = (unit, lambda get=get: {n.split("/")[-1]: v for n, v in get().items()})
        t.add("robot", "robot", {**mj_body_fields(R, root), **jf},
              note=f"Franka Panda, base body {m.body(root).name!r} (fixed base); joints by their element names "
                   "(proprio order: the arm's joints, then finger_joint1 / 2)")
        t.add("tcp", "robot_link", {"pos": ("m", lambda: np.asarray(self.s.env.get_ee_pos(), float)),
                                    "quat": ("1 (w x y z)", lambda: np.asarray(self.s.env.get_ee_quat(), float))},
              note="VLABench's end effector (env.get_ee_pos / get_ee_quat): proprio ee pose")
        ents = {n: (e, root_of(e)) for n, e in env.task.entities.items()}
        ents = {n: (e, b) for n, (e, b) in ents.items() if b not in sub and b > 0}
        names = entity_names({n: (type(e).__name__, R.d.xpos[b]) for n, (e, b) in ents.items()})
        def joint_fields(ids):                    # joint names "<entity>/<joint>" with the entity's table name
            out = mj_joint_fields(R, ids, ranges=True)
            path = lambda k: "/".join(names.get(c, c) for c in k.split("/"))  # noqa: E731
            return {f: (u, lambda get=get: {path(k): v for k, v in get().items()}) for f, (u, get) in out.items()}

        for n, (e, b) in sorted(ents.items(), key=lambda kv: names[kv[0]]):
            n = names[n]
            js = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in R.subtree(b)]
            free = any(int(m.jnt_type[j]) == 0 for j in js if int(m.jnt_bodyid[j]) == b)
            moving = [j for j in js if int(m.jnt_type[j]) in (2, 3)]
            cat = type(e).__name__
            if free:
                t.add(n, "object", {**mj_body_fields(R, b, size=True, bbox=True), **joint_fields(moving)},
                      note=f"VLABench {cat}, free")
            elif moving:
                t.add(n, "articulation", {**mj_body_fields(R, b, vel=False, size=True, bbox=True),
                                          **joint_fields(moving)}, note=f"VLABench {cat}")
            else:
                t.add(n, "scene_feature", mj_body_fields(R, b, vel=False, size=True, bbox=True), note=f"VLABench {cat}, fixed")
        return t

    def save_state(self) -> dict:
        s = self.s
        logic = {k: {a: v for a, v in vars(o).items() if isinstance(v, PLAIN)} for k, o in _logic_objects(s.env).items()}
        return {"data": _mj_save(s.physics.data.ptr), "logic": logic, "targets": s.targets.copy(),
                "actions": [a.copy() for a in s.actions], "done": bool(s.done)}

    def load_state(self, snap: dict) -> None:
        s = self.s
        objs = _logic_objects(s.env)
        for k, values in snap["logic"].items():
            for a, v in values.items():
                setattr(objs[k], a, v)
        _mj_load(s.physics.data.ptr, snap["data"])
        s.targets, s.actions, s.done = snap["targets"].copy(), [a.copy() for a in snap["actions"]], snap["done"]

    # ---------------------------------------------------------------- evidence (the verdict is the live episode's)
    def save_trajectory(self, rows: np.ndarray, path: str, meta: dict) -> None:
        np.savez(path, actions=np.asarray(rows, np.float64), meta=np.array(vlb_tool.canonical(meta)))

    def render_video(self, rows: np.ndarray, path: str) -> None:
        """The episode's video, after the verdict: the trajectory re-run once in a fresh scene, filmed."""
        vlb_tool.Session(self.instance).render_video(rows, path, every=max(1, len(rows) // 600))


def build() -> VLABench:
    env = os.environ.get
    return VLABench(
        env("RCB_INSTANCE", "/srv/instance.json"),
        task_text=env("RCB_TASK_TEXT", ""),
        episode=env("RCB_EPISODE", ""),
        image_hw=int(env("RCB_IMAGE_HW") or 320),
        max_control_steps=int(env("RCB_MAX_CONTROL_STEPS") or 9000),
    )


if __name__ == "__main__":
    main(build)
