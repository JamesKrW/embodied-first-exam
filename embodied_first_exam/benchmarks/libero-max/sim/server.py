"""eai-standard/2.1 sim service for LIBERO-MAX -- the sidecar's entry point (`python /srv/server.py`).

The simulator-specific half only: `LiberoMax` implements rcb_service.Sim for one frozen instance (lmx_tool.Session).
The protocol, the episode, crash recovery and the finalize live in rcb_service.py (a byte copy of main's HumanoidBench /
VLABench service), which imports no simulator.
The verdict is the live episode's (docs/protocol/README.md H.4; owner, 2026-10-17): the episode ends the moment
LIBERO's success check holds (step() reports it), and that is the success; nothing is replayed to grade it. The
frozen start is checked when the episode begins (start_sha: the instance's recorded start-state hash; a scene that
did not rebuild to it is refused as instance_mismatch, an infrastructure failure, before the agent is served).
live_verdict() adds LIBERO-MAX's event record of the live episode to final.json (event_fired, event_step).
The video is one video-only pass after the verdict is on disk: render_video() replays the trajectory in a fresh
process (one simulator per process, lmx_tool) with the camera on; it can neither change nor cost the verdict
(docs/benchmarks/libero-max.md, "Grading (standard mode)").
Settings come from environment/docker-compose.yaml:
  RCB_INSTANCE            the frozen instance (the privileged twin's instance.json)
  RCB_TASK_TEXT           the task as the instruction states it, for /spec
  RCB_EPISODE             what ends an episode, for /spec (the instruction's words)
  RCB_MAX_CONTROL_STEPS   technical episode cap (9000 = 450 s of robot time): the video pass of the longest episode
                          must finish inside the collect hook's wait (finalize.sh)
  LMX_VIDEO_BUDGET_S, LMX_VIDEO_MAX_FRAMES   the video's rendering bounds (lmx_tool: 240 s, 1200 frames)
  RCB_VIDEO_TIMEOUT_S     how long the video pass may take in all (5400 s, the collect hook's wait)
  RCB_IMAGE_HW            the side of every camera's image in pixels (256)
  RCB_RESET_BUDGET, RCB_WALL_CLOCK_S, RCB_DATA   the service's own (rcb_service.main)

Ablation (off unless the task sets it; nothing here runs without it): with rcb_service.STATE_QUERY, state_table() gives
the privileged state in the world frame of /spec "frames" (quaternions w x y z): the Panda's base pose and every arm and
finger joint, its eef site (proprio eef) and hand; the scene's movable objects (Session.state()["objects"] names: pose,
velocity, size, bbox) and fixtures (an articulation with its hinge and slide joints, else a scene feature), the arena's
table. A name that states a thing's role in the case loses that word (`neutral_name`: `black_book_occluder_1` is
`black_book_1`). A query reads a scratch copy of the live MuJoCo state (MjRead); it takes no control step. Never the
goal, the success check or the event's record. No save_state(): one episode per process, so no reset either.
"""

from __future__ import annotations

import contextlib
import json
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import lmx_tool  # noqa: E402
import rcb_service  # noqa: E402
from rcb_service import main  # noqa: E402

CAMERAS = ("agentview", "wrist")
# The video pass (render_video, after the verdict): a scene build, the episode's physics and the frames; at most this long
VIDEO_TIMEOUT_S = float(os.environ.get("RCB_VIDEO_TIMEOUT_S") or 5400)


def _result(line: str) -> dict | None:
    """A result line of `lmx_tool.py --replay` (JSON: the replay's outcome and its video), or None for anything else."""
    try:
        r = json.loads(line)
    except ValueError:
        return None
    return r if isinstance(r, dict) and "state_sha" in r and "success" in r else None


# rcb_service.STATE_QUERY: the state table (LiberoMax.state_table), read by MjRead.
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


_ROLE_WORDS = frozenset({"occluder", "distractor", "obstacle", "target", "goal", "decoy"})


def neutral_name(name: str, taken: set) -> str:
    """A scene name without a word that states the thing's role in the case (`black_book_occluder_1`: `black_book_1`),
    numbered on (`_2`, ...) when that name is taken."""
    parts = [p for p in name.split("_") if p not in _ROLE_WORDS] or [name]
    k = int(parts.pop()) if len(parts) > 1 and parts[-1].isdigit() else None
    stem = "_".join(parts)
    out = stem if k is None else f"{stem}_{k}"
    while out in taken:
        k = (k or 1) + 1
        out = f"{stem}_{k}"
    return out


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


class LiberoMax:
    """One frozen LIBERO-MAX case behind the rcb_service.Sim contract. The agent never gets an object's pose, the goal
    or the success check, nor whether or when the event happened; they are live here only so the service can judge."""

    def __init__(self, instance: str, *, task_text: str = "", episode: str = "", image_hw: int = 256,
                 max_control_steps: int = 9000):  # fmt: skip
        self.instance = instance
        # check=False: the frozen start is checked by rcb_service when the episode begins (start_sha below), which
        # records a mismatch as instance_mismatch (infrastructure) instead of failing every start of the process
        self.s = lmx_tool.Session(instance, check=False)
        self.start_sha = str(self.s.inst.get("initial_state_sha") or "")   # the frozen instance's recorded start state
        self._live_sha: str | None = None    # the live episode's final state (live_verdict), for the video's record
        self.task_text = task_text or self.s.instruction
        self.episode = episode
        self.hw = int(image_hw)
        ctl = self.s.controller()
        self.control_dt = 1.0 / ctl["control_freq"]
        self.max_control_steps = int(max_control_steps)
        self.cameras = CAMERAS
        self.depth_cameras = CAMERAS
        self.raw_dim = lmx_tool.N
        self.lower, self.upper = -np.ones(lmx_tool.N), np.ones(lmx_tool.N)
        self.out_max = ctl["output_max"]
        self.proprio_layout = (
            ["eef_x", "eef_y", "eef_z", "eef_qw", "eef_qx", "eef_qy", "eef_qz", "finger_left", "finger_right"]
            + [f"joint{i}_pos" for i in range(1, 8)]
            + [f"joint{i}_vel" for i in range(1, 8)]
            + [f"cmd_{a}" for a in lmx_tool.ACTION_NAMES]
        )

    # ---------------------------------------------------------------- what the agent may know
    def calibrations(self) -> dict:
        return {c: self.s.calibration(c, self.hw) for c in CAMERAS}

    def spec(self) -> dict:
        cams = {}
        for c, cal in self.calibrations().items():
            cams[c] = {"hw": [self.hw, self.hw], "intrinsic_cv": cal["intrinsic_cv"], "cam2world_gl": cal["cam2world_gl"],
                       "mounted": "on the hand (moves with it)" if c == "wrist" else "fixed",
                       "note": "the pose and intrinsics at the start; every submit response carries the current ones "
                               "(meta['cameras_now'])"}  # fmt: skip
        base = self.s.robot_base()["pos"]
        return {
            "simulator": "LIBERO (robosuite 1.4.0, MuJoCo 3.2.6) with LIBERO-PRO's task variant and LIBERO-MAX's event",
            "robot": "Franka Emika Panda on a fixed mount, 2-finger gripper, LIBERO's OSC_POSE controller",
            "task_text": self.task_text,
            "episode": self.episode,
            "frames": (
                f"world frame: metres; the robot's base is at ({', '.join(f'{v:g}' for v in base)}) facing +x: x forward "
                "(away from the base, across the table), y to the robot's left, z up; quaternions w x y z"
            ),
            "action_types": {
                "ee_delta": {
                    "dim": lmx_tool.N,
                    "layout": lmx_tool.ACTION_NAMES,
                    "lower": self.lower.tolist(),
                    "upper": self.upper.tolist(),
                    "units": (
                        f"dx, dy, dz: 1.0 = {self.out_max[0]:g} m of hand-target motion per control step; drx, dry, drz: "
                        f"1.0 = {self.out_max[3]:g} rad of rotation per control step (axis-angle, world frame); "
                        "grip: -1 open, +1 close"
                    ),
                    "dtype": "float64",
                    "semantics": (
                        "LIBERO's native action: a delta for the operational-space controller (OSC_POSE), applied for one "
                        "control step (1/20 s); values outside [-1, 1] are clipped"
                    ),
                },
                "not_yet": ["joint_position", "ee_pose"],
            },
            "cameras": cams,
        }

    def interpolable(self, type: str) -> bool:
        return True

    def images(self, cameras: list[str], depth: bool) -> tuple[dict, dict]:
        rgb, dep = {}, {}
        for c in cameras:
            if depth and c in self.depth_cameras:
                rgb[c], dep[c] = self.s.render(c, self.hw, depth=True)
            else:
                rgb[c] = self.s.render(c, self.hw)
        return rgb, dep

    def proprio(self) -> np.ndarray:
        return self.s.proprio()

    # ---------------------------------------------------------------- actions
    def clip(self, type: str, rows: np.ndarray) -> tuple[np.ndarray, list]:
        out = np.clip(rows, self.lower, self.upper)
        clipped = [
            {"row": int(r), "index": int(i), "reason": "outside [-1, 1]"}
            for r, i in zip(*np.nonzero(out != rows), strict=True)
        ]
        return out, clipped

    def to_sim(self, type: str, row: np.ndarray) -> np.ndarray:
        return np.asarray(row, dtype=np.float64)

    def command(self, type: str) -> np.ndarray:
        return self.s.targets.copy()

    def hold(self) -> tuple[str, np.ndarray]:
        return "ee_delta", np.r_[np.zeros(6), self.s.targets[6]]  # no motion, the gripper as commanded last

    def step(self, row: np.ndarray) -> tuple[bool, bool, dict]:
        r = self.s.step(row)
        return r["done"], r["success"], {"cameras_now": self.calibrations()}

    def reset(self) -> None:
        raise RuntimeError("this service runs one episode per process (no reset capability)")

    def state_sha(self) -> str:
        return self.s.state_sha()

    # ---------------------------------------------------------------- ablation (rcb_service.STATE_QUERY)
    def state_table(self):
        """The state-query ablation's table, world frame of /spec "frames". Robot: the Panda's root body (fixed mount)
        with every robot0_ / gripper0_ joint (prefix dropped); links: the eef site (proprio eef pose) and the hand;
        objects: Session.state()["objects"], each its root body; fixtures: articulations when they hold hinge or slide
        joints, else scene features; the arena's table. Read by MjRead (a scratch copy of the live state)."""
        import mujoco

        sim = self.s.sim
        R = MjRead(mujoco, lambda: sim.model._model, lambda: sim.data._data).refresh()
        m = R.m
        robot = self.s.env.robots[0]
        root = m.body(robot.robot_model.root_body).id
        sub = R.subtree(root)
        base = self.s.robot_base()["pos"]
        t = MjStateTable(rcb_service.StateTable(
            frame=f"the world frame of /spec frames: metres, the robot's base at ({', '.join(f'{v:g}' for v in base)}) "
                  "facing +x: x forward (across the table), y to the robot's left, z up; the frame of the proprio eef "
                  "pose and the cameras' cam2world_gl", quaternion_order="wxyz",
            note="a pose is a body's or site's frame origin (MuJoCo xpos / xquat); velocities are world-frame, at that "
                 "origin"), R)
        js = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in sub]
        jf = mj_joint_fields(R, js, ranges=True)
        strip = lambda n: n.split("_", 1)[1] if n.startswith(("robot0_", "gripper0_")) else n  # noqa: E731
        for k in list(jf):
            unit, get = jf[k]
            jf[k] = (unit, lambda get=get: {strip(n): v for n, v in get().items()})
        t.add("robot", "robot", {**mj_body_fields(R, root), **jf},
              note=f"Franka Panda, root body {m.body(root).name!r} (fixed mount); joints by name without the robot0_ / "
                   "gripper0_ prefix (joint1..7: proprio joint*_pos; finger_joint1 / 2: proprio finger_left / right)")
        site = int(getattr(robot, "eef_site_id", -1))
        if site < 0:
            site = m.site("gripper0_grip_site").id
        t.add("eef", "robot_link", mj_site_fields(R, site), note=f"site {m.site(site).name!r}: proprio eef_x .. eef_qz")
        for b in sorted(sub):
            if m.body(b).name.endswith("right_hand"):
                t.add("hand", "robot_link", mj_body_fields(R, b), note=f"body {m.body(b).name!r}")
        backend = self.s.wrapped.backend

        def root_of(n):
            entity, _ = backend._entity(n)
            return m.body(entity.root_body).id

        taken = set(t.entities()["entities"]) | set(self.s.env.objects_dict) | set(self.s.env.fixtures_dict)
        names = {}
        for n in [*self.s.env.objects_dict, *self.s.env.fixtures_dict]:
            names[n] = n if neutral_name(n, set()) == n else neutral_name(n, taken)
            taken.add(names[n])
        for n in self.s.env.objects_dict:
            t.add(names[n], "object", mj_body_fields(R, root_of(n), size=True, bbox=True),
                  note="a movable object of the scene")
        for n in self.s.env.fixtures_dict:
            b = root_of(n)
            inner = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in R.subtree(b) and int(m.jnt_type[j]) in (2, 3)]
            if inner:
                t.add(names[n], "articulation", {**mj_body_fields(R, b, vel=False, size=True, bbox=True),
                                                 **mj_joint_fields(R, inner, ranges=True)},
                      note="a fixture with moving parts: its root body and its hinge / slide joints")
            else:
                t.add(names[n], "scene_feature", mj_body_fields(R, b, vel=False, size=True, bbox=True), note="a fixture")
        have = set(t.entities()["entities"])
        for b in range(1, m.nbody):   # the arena's table (LIBERO's main_table, ...): not a fixture of the task
            n = m.body(b).name
            if int(m.body_parentid[b]) == 0 and "table" in n and n not in have and R.body_geoms(b):
                t.add(n, "scene_feature", mj_body_fields(R, b, vel=False, size=True, bbox=True), note="the arena's table")
        return t

    # ---------------------------------------------------------------- the verdict's record and the video
    def live_verdict(self, rows: np.ndarray, success: bool) -> dict:
        """LIBERO-MAX's record of the live episode, for final.json: whether, and at which control step (1-based; null if
        never), the case's event fired. No "success" key: LIBERO's rule is the live one (the episode ends the moment
        the success check holds, so no action can follow a success), applied by rcb_service as it ran."""
        live = self.s.report()
        self._live_sha = self.s.state_sha()
        return {"event_fired": bool(live["event_fired"]), "event_step": live["event_step"]}

    def save_trajectory(self, rows: np.ndarray, path: str, meta: dict) -> None:
        np.savez(path, actions=np.asarray(rows, np.float64), meta=np.array(lmx_tool.canonical(meta)))

    def render_video(self, rows: np.ndarray, path: str) -> None:
        """Called by rcb_service once the verdict is on disk: the episode's video, one replay of the trajectory in a fresh
        process (`lmx_tool.py --replay ... <video>`; one simulator per process) that renders agentview on the way. It
        decides nothing: what it says about success is not read, a failure raises (rcb_service then records video =
        false). Next to the video, replay.video.json says whether that replay ended in the live episode's final state
        (the video shows the live episode only then) and its stride (control steps per frame)."""
        tmp = tempfile.mkdtemp(prefix="video-")
        proc = None
        try:
            npy, video = os.path.join(tmp, "a.npy"), os.path.join(tmp, "replay.mp4")
            np.save(npy, lmx_tool.as_actions(rows))
            proc = subprocess.Popen(  # stderr: the service's own (the sidecar's log)
                [sys.executable, os.path.abspath(lmx_tool.__file__), "--replay", str(self.instance), npy, video],
                stdout=subprocess.PIPE, text=True,
            )  # fmt: skip
            try:
                out, _ = proc.communicate(timeout=VIDEO_TIMEOUT_S)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.communicate()
                raise RuntimeError(f"the video pass did not finish in {VIDEO_TIMEOUT_S:g} s") from None
            # The last result line says whether the video is complete (the exit code may be a teardown's).
            results = [r for r in map(_result, out.splitlines()) if r is not None]
            if not (results and results[-1].get("video") and os.path.exists(video)):
                raise RuntimeError(f"the video pass recorded no video (exit {proc.returncode})")
            r = results[-1]
            shutil.move(video, path)
            png = video.removesuffix(".mp4") + ".last.png"
            if os.path.exists(png):
                shutil.move(png, path.removesuffix(".mp4") + ".last.png")
            rec = {"source": "video-only replay of trajectory.npz after the verdict (it decides nothing)",
                   "state_sha": r.get("state_sha"), "live_state_sha": self._live_sha,
                   "ends_in_live_state": r.get("state_sha") == self._live_sha, "event_step": r.get("event_step"),
                   "video_stride": r.get("video_stride"), "video_frames": r.get("video_frames"),
                   "video_skipped": r.get("video_skipped")}  # fmt: skip
            with contextlib.suppress(OSError):
                with open(path.removesuffix(".mp4") + ".video.json", "w") as f:   # replay.first.mp4: its own record
                    json.dump(rec, f)
            if not rec["ends_in_live_state"]:
                print(f"[video] the video pass ended in {r.get('state_sha')}, the live episode in {self._live_sha}: "
                      "the video may not show the live episode", file=sys.stderr, flush=True)  # fmt: skip
        finally:
            if proc is not None and proc.poll() is None:
                proc.kill()
                proc.wait()
            shutil.rmtree(tmp, ignore_errors=True)


def build() -> LiberoMax:
    env = os.environ.get
    return LiberoMax(
        env("RCB_INSTANCE", "/srv/instance.json"),
        task_text=env("RCB_TASK_TEXT", ""),
        episode=env("RCB_EPISODE", ""),
        image_hw=int(env("RCB_IMAGE_HW") or 256),
        max_control_steps=int(env("RCB_MAX_CONTROL_STEPS") or 9000),
    )


if __name__ == "__main__":
    os.environ.setdefault("LMX_TRACE", os.path.join(tempfile.gettempdir(), "lmx-trace.jsonl"))
    main(build)
