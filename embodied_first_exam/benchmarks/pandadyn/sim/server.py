"""eai-standard/2.1 sim service for Panda Dynamic Scenes' standard tasks -- the adapter behind the shared standard
service layer (`python /srv/server.py`: rcb_service.main(build); docs/protocol/standard_mode.md Part 1, docs/benchmarks/pandadyn.md
"Standard mode").

The simulator-specific half only: `PandaDyn` implements rcb_service.Sim (the contract documented in rcb_service.py) for
one frozen instance, on the benchmark's own tooling rcb_pandadyn.Session (ManiSkill 3.0.1 / SAPIEN 3.0.3, CPU PhysX at
1000 Hz, one or two Franka Pandas behind their Ruckig + torque servos). The code below is the standard-realtime twin's
adapter, byte for byte (scripts/pandadyn/mk_tasks.py writes this copy): the two standard modes share the scene, the action,
the cameras, the proprioception, the success rule, the reset and the trajectory, and differ only in the clock (here the
world is frozen between the agent's calls; the realtime twin runs rcb_realtime.py on top of the same service layer). The
protocol, the episode, the resets, crash recovery and the finalize are rcb_service.py's, an unchanged copy of the
HumanoidBench source.

What the agent gets: the two static cameras (RGB, z-depth on request) with their calibration, the robot's own
proprioception (joints, fingers, hand TCP and tool poses, velocities, apertures, motor torques, the targets in force), the
`joint_position` action (per arm 7 absolute joint targets + the total aperture, clipped to the limits) and POST /reset
within the task's budget (RCB_RESET_BUDGET). Nothing about objects or fixtures leaves this process.

The verdict is the live episode's (owner, 2026-10-17; docs/benchmarks/pandadyn.md "Grading"): the task's
own check as the last episode ended in this process, judged by rcb_service after the agent's container is gone; nothing
is replayed to grade it. The task's check is the episode's end (the first control step where it succeeded or failed), so
there is no settle window to add (live_verdict below only records the phase and the end reason). The frozen start is
checked when the first episode begins and after every reset: `start_sha` is the instance's recorded `initial_sha`
(rcb_service refuses a scene that starts elsewhere: instance_mismatch, an infrastructure failure). The video is rendered
after the verdict from the graded episode's rows by this process's Session (render_video: the scene rebuilt, the rows
filmed).

Settings (environment/docker-compose.yaml): RCB_INSTANCE (the frozen instance), RCB_TEXTS (texts.json: the task text),
RCB_DATA, RCB_RESET_BUDGET, RCB_WALL_CLOCK_S (rcb_service).

Ablations (off unless the task sets them; nothing here runs without them):
  rcb_service.STATE_QUERY   state_table(): the privileged state in the scene's world frame (the frame of the TCP and
                            tool poses; quaternions w x y z): each arm's base pose and joints, its TCP and tool poses;
                            the task's objects (Session.objects(), its names: pose and velocity); its fixtures (base
                            pose, the joint's position and velocity); the static parts of the scene (pose, bbox). Read
                            off the live scene between calls (no step, nothing written); never the task's phase, goals,
                            events or debug target markers.
  rcb_service.STATE_OPS     save_state() / load_state(): Session.snapshot() / restore() (the environment's own exact
                            snapshot; restoring one that is not the current state re-runs its rows from a seeded reset)
                            and the command in force.
"""
from __future__ import annotations

import copy
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ["RCB_SESSION_LOG"] = ""
import rcb_pandadyn as M  # noqa: E402

M.SESSION_LOG = ""           # the sidecar's Session is not the agent's interaction log
import rcb_service  # noqa: E402
from rcb_service import main  # noqa: E402

JOINTS = [f"joint{i}" for i in range(1, 8)]
MOUNT = {"overview": "static, looking at the whole workspace from the front right, above the table",
         "closeup": "static, closer to the task's working area"}


# scene parts whose name states a role in the task, under a neutral name in the state table
NEUTRAL = {"goal_backstop": "backstop"}


class PandaDyn:
    """One frozen Panda Dynamic Scenes instance behind the rcb_service.Sim contract (see the module docstring)."""

    def __init__(self, instance: str, texts: dict):
        self.instance, self.texts = instance, texts
        self.I = json.load(open(instance))
        self.s = M.Session(self.I)               # not strict: rcb_service checks start_sha when the first episode begins
        self.start_sha = str(self.I["initial_sha"])  # the frozen instance's recorded start (rcb_pandadyn make-instance)
        self.arms = list(self.s.arm_ids)
        self.raw_dim = int(self.s.dim)
        self.control_dt = 1.0 / M.CONTROL_HZ
        self.max_control_steps = int(M.MAX_ROWS)                # the environment's cap; the task's horizon ends it first
        self.horizon_s = float(self.I["horizon_s"])
        self.cameras = tuple(M.CAMERAS)
        self.depth_cameras = self.cameras
        self.layout = [f"{a}_{j}" for a in self.arms for j in JOINTS + ["aperture"]]
        L = M._limits()
        self.lim = np.array([[lo, hi] for _a in self.arms for lo, hi in list(zip(L.LOWER, L.UPPER)) + [(0.0, M.APERTURE_MAX)]],
                            dtype=np.float64)
        qpos = [f"{a}_{j}" for a in self.arms for j in JOINTS + ["finger_left", "finger_right"]]
        self.proprio_layout = (qpos + [f"{a}_tcp_{k}" for a in self.arms for k in ("x", "y", "z", "qw", "qx", "qy", "qz")]
                               + [f"{q}_vel" for q in qpos] + [f"{a}_aperture_m" for a in self.arms]
                               + [f"{a}_{j}_torque" for a in self.arms for j in JOINTS]
                               + [f"{a}_tool_{k}" for a in self.arms for k in ("x", "y", "z", "qw", "qx", "qy", "qz")]
                               + [f"{a}_target_{j}" for a in self.arms for j in JOINTS + ["aperture"]])
        self.cam_info = self.s.camera_info(list(self.cameras))
        self.hw = {c: list(self.s.render_frame(c).shape[:2]) for c in self.cameras}   # warms up the renderer
        self.rs = self.s.robot_spec()
        self.cmd = np.asarray(self.s.hold_action(), dtype=np.float64)

    # ------------------------------------------------------------------ the rcb_service.Sim contract
    def spec(self) -> dict:
        na, nq = len(self.arms), 9 * len(self.arms)
        return {
            "simulator": "maniskill (Panda Dynamic Scenes on ManiSkill 3.0.1 / SAPIEN 3.0.3, CPU PhysX at 1000 Hz)",
            "robot": "franka-panda" if na == 1 else "franka-panda-bimanual",
            "task_text": self.texts["task_text"],
            "episode": ("an episode ends at the first control step where the task has succeeded (every phase in order) or "
                        "failed (a failure event, a forbidden contact, a safety-limit violation, an object out of bounds, or "
                        f"the task's horizon of {self.horizon_s:g} s of robot time, {int(round(self.horizon_s * M.CONTROL_HZ))} "
                        "control steps), when your time is up, or on a simulator error"),
            "action_types": {"joint_position": {
                "dim": self.raw_dim, "layout": self.layout,
                "lower": self.lim[:, 0].round(4).tolist(), "upper": self.lim[:, 1].round(4).tolist(),
                "units": "rad for <arm>_joint1..7 (absolute targets); <arm>_aperture: the total gripper opening target in "
                         "metres (0 = closed .. 0.08 = open)",
                "semantics": "the environment's own action, one row per 10 ms control step: each arm's servo turns its "
                             "latest targets into a jerk-limited reference motion (Ruckig, joints synchronised) within its "
                             "speed, acceleration and jerk limits (robot_spec.servo) and tracks it with torque control; rows "
                             "dt apart (a multiple of control_dt), `linear` (default: a straight line from the targets in "
                             "force) or `hold`; out-of-limit values are clipped and reported (meta clipped)"}},
            "cameras": {c: {"hw": self.hw[c], "mounted": MOUNT[c], "intrinsic_cv": self.cam_info[c]["intrinsic_cv"],
                            "extrinsic_cv": self.cam_info[c]["extrinsic_cv"], "cam2world_gl": self.cam_info[c]["cam2world_gl"],
                            "depth_range_m": self.cam_info[c]["depth_range_m"]} for c in self.cameras},
            "proprio_groups": {
                "joint_pos": [0, nq, "measured joint positions per arm (panda-0 first): joint1..7 (rad), finger_left, "
                                     "finger_right (m)"],
                "tcp_pose": [nq, 7 * na, "measured hand TCP pose per arm, x y z (m) + quaternion wxyz, world frame"],
                "joint_vel": [nq + 7 * na, nq, "measured joint velocities, the joints of joint_pos"],
                "gripper_opening": [2 * nq + 7 * na, na, "measured aperture per arm (m): 0 closed .. 0.08 open"],
                "joint_effort": [2 * nq + 8 * na, 7 * na, "the servo's commanded motor torques per arm, joint1..7 (N m)"],
                "tool_pose": [2 * nq + 15 * na, 7 * na, "each arm's task tool frame (the hand TCP without a tool), x y z + "
                                                        "wxyz, world frame"],
                "target": [2 * nq + 22 * na, 8 * na, "each arm's targets in force (joint1..7, aperture)"]},
            "static": {"arms": {a: {"role": self.rs["arms"][a]["role"], "tool": self.rs["arms"][a]["tool"]} for a in self.arms}},
            "world": {"frame": "the scene's world frame: z up, the table top is the plane z = 0; the arm bases are in "
                               "robot_spec.arms", "up": "+z", "units": {"length": "m", "angle": "rad", "time": "s"},
                      "quaternion_order": "wxyz",
                      "camera_axes": "intrinsic_cv / extrinsic_cv: OpenCV camera axes (x right, y down, z forward); "
                                     "cam2world_gl: OpenGL axes"},
            "robot_spec": self.rs,
            "depth": {"cameras": list(self.depth_cameras), "units": "z-depth in metres along the optical axis, pixel for "
                      "pixel with the camera's image; 0 = no reading"},
        }

    def interpolable(self, typ: str) -> bool:
        return True

    def images(self, cameras: list[str], depth: bool) -> tuple[dict, dict]:
        rgb, dep = {}, {}
        for c in cameras:
            rgb[c] = self.s.render_frame(c)                       # captures the camera (rendering never steps physics)
            if depth:
                dep[c] = self.s._depth_of(c)                      # the same capture's depth buffer
        return rgb, dep

    def proprio(self) -> np.ndarray:
        P, A = self.s.proprio(), self.arms
        cols = ([v for a in A for v in P[a]["q"]] + [v for a in A for v in P[a]["tcp_pose"]]
                + [v for a in A for v in P[a]["qd"]] + [P[a]["aperture"] for a in A]
                + [v for a in A for v in P[a]["torque"]] + [v for a in A for v in P[a]["tool_pose"]]
                + [v for a in A for v in P[a]["target"]])
        return np.array(cols, dtype=np.float32)

    def clip(self, typ: str, rows: np.ndarray) -> tuple[np.ndarray, list]:
        rows = np.asarray(rows, dtype=np.float64)
        out = np.clip(rows, self.lim[:, 0], self.lim[:, 1])
        report = [{"row": int(i), "index": int(j), "reason": f"{self.layout[j]} limit [{self.lim[j, 0]:.4f},{self.lim[j, 1]:.4f}]"}
                  for i, j in np.argwhere(out != rows)]
        return out, report

    def to_sim(self, typ: str, row: np.ndarray) -> np.ndarray:
        return np.asarray(row, dtype=np.float64)                  # clipped already: Session.step executes it as it is

    def command(self, typ: str) -> np.ndarray:
        return self.cmd

    def hold(self) -> tuple[str, np.ndarray]:
        return "joint_position", self.cmd

    def step(self, row: np.ndarray) -> tuple[bool, bool, dict]:
        over = self.s.step(row)
        self.cmd = np.asarray(row, dtype=np.float64).copy()
        return bool(over), bool(self.s.success()), ({"why": self.s.end_reason()} if over else {})

    def reset(self) -> None:
        self.s.reset()                                            # the scene rebuilt and reset with the instance's seed
        self.cmd = np.asarray(self.s.hold_action(), dtype=np.float64)

    def state_sha(self) -> str:
        return self.s.state_sha()

    # ------------------------------------------------------------------ ablations (rcb_service.STATE_OPS, STATE_QUERY)
    def state_table(self):
        """The state-query ablation's table, in the scene's world frame. Robots: each arm by its id (base pose, joint1..7
        and the two fingers, the joint limits of robot_spec); links: <arm>_tcp and <arm>_tool (the proprio poses);
        objects: Session.objects(); articulations: Session.fixtures() (pose of the fixture's base); scene features: the
        static actors (table, pedestals, rails, lanes, ...: pose and collision bbox), never a debug target marker."""
        s = self.s
        t = rcb_service.StateTable(
            frame="the scene's world frame (metres, z up, the table top is the plane z = 0): the frame of the proprio "
                  "TCP and tool poses, /kinematics and the cameras' extrinsics", quaternion_order="wxyz",
            note="an object's pose is its rigid body's frame (SAPIEN entity pose); velocities are world-frame")
        qn = JOINTS + ["finger_left", "finger_right"]
        L = M._limits()
        rng = {j: [float(lo), float(hi)] for j, lo, hi in zip(JOINTS, L.LOWER, L.UPPER)}
        for uid in self.arms:
            arm = s.u.arms[uid]
            q = lambda arm=arm: np.asarray(M._np(arm.robot.qpos), dtype=np.float64)[:9]   # noqa: E731
            v = lambda arm=arm: np.asarray(M._np(arm.robot.qvel), dtype=np.float64)[:9]   # noqa: E731
            t.add(uid, "robot", {
                "pos": ("m", lambda uid=uid: s.base_pose(uid)[:3]), "quat": ("1 (w x y z)", lambda uid=uid: s.base_pose(uid)[3:]),
                "joint_pos": ("rad (joint1..7), m (fingers)", lambda q=q: dict(zip(qn, q().tolist()))),
                "joint_vel": ("rad/s, m/s", lambda v=v: dict(zip(qn, v().tolist()))),
                "joint_range": ("rad ([lo, hi])", lambda: rng)},
                note=f"Franka Panda {uid!r} ({arm.role}), fixed base; joints as proprio joint_pos names them")
            t.add(f"{uid}_tcp", "robot_link", {"pos": ("m", lambda uid=uid: s.tcp_pose(uid)[:3]),
                                               "quat": ("1 (w x y z)", lambda uid=uid: s.tcp_pose(uid)[3:])},
                  note="the hand TCP (proprio tcp pose)")
            if arm.tool is not None:
                t.add(f"{uid}_tool", "robot_link", {"pos": ("m", lambda uid=uid: s.tool_pose(uid)[:3]),
                                                    "quat": ("1 (w x y z)", lambda uid=uid: s.tool_pose(uid)[3:])},
                      note=f"the tool ({arm.setup.tool}) mounted on the hand (proprio tool pose)")
        from panda_dynamics.props.primitives import body

        def obj(n, k):   # one value of one object, as Session.objects() reads it (the scene's current props: a reset rebuilds them)
            rb = body(s.u.objects[n])
            return M._np({"position": rb.entity_pose.p, "orientation_wxyz": rb.entity_pose.q, "velocity": rb.linear_velocity,
                          "angular_velocity": rb.angular_velocity}[k]).reshape(-1).tolist()

        for n in s.u.objects:
            t.add(n, "object", {"pos": ("m", lambda n=n: obj(n, "position")),
                                "quat": ("1 (w x y z)", lambda n=n: obj(n, "orientation_wxyz")),
                                "lin_vel": ("m/s", lambda n=n: obj(n, "velocity")),
                                "ang_vel": ("rad/s", lambda n=n: obj(n, "angular_velocity"))}, note="a prop of the task")
        for n in s.u.fixtures:
            art = lambda n=n: s.u.scene.articulations.get(f"{n}_fixture")   # noqa: E731
            f = {"joint_pos": ("rad or m", lambda n=n: {n: float(s.u.fixtures[n].q)}),
                 "joint_vel": ("rad/s or m/s", lambda n=n: {n: float(s.u.fixtures[n].v)})}
            if art() is not None:
                f = {"pos": ("m", lambda art=art: M._np(art().pose.p).reshape(-1).tolist()),
                     "quat": ("1 (w x y z)", lambda art=art: M._np(art().pose.q).reshape(-1).tolist()), **f}
            t.add(n, "articulation", f, note="a fixture of the task: its base pose and its one joint's position and velocity")
        skip = set(s.u.objects) | {"floor"}
        for n, a in s.u.scene.actors.items():   # the scene's static parts (table, pedestals, rails, lanes, ...)
            if n in skip or getattr(a, "px_body_type", "") != "static" or any(k in n for k in ("debug", "marker", "target")):
                continue
            f = {"pos": ("m", lambda p=M._np(a.pose.p).reshape(-1).tolist(): p),
                 "quat": ("1 (w x y z)", lambda q=M._np(a.pose.q).reshape(-1).tolist(): q)}
            try:
                lo, hi = np.asarray(a.get_first_collision_mesh(to_world_frame=True).bounds, dtype=np.float64)
                f["bbox"] = ("m ([min xyz, max xyz] of its collision shape, world axes)", lambda b=[lo.tolist(), hi.tolist()]: b)
            except Exception:   # an actor without a collision mesh: its pose only
                pass
            t.add(NEUTRAL.get(n, n), "scene_feature", f,
                  note="the table: its top is the plane z = 0" if n == "table" else "a static part of the scene")
        return t

    def save_state(self) -> dict:
        return {"session": self.s.snapshot(), "cmd": self.cmd.copy()}

    def load_state(self, snap: dict) -> None:
        self.s.restore(copy.deepcopy(snap["session"]))   # a copy: the snapshot can be loaded again
        self.cmd = snap["cmd"].copy()

    def live_verdict(self, rows: np.ndarray, success: bool) -> dict:
        """The benchmark's own record of the live episode as it ended (no "success": the task's check already is the
        episode's end, so nothing is taken away): the phases completed and why the episode ended."""
        return {"phase": int(self.s.u.phase), "end_reason": self.s.end_reason() or ""}

    def save_trajectory(self, rows: np.ndarray, path: str, meta: dict) -> None:
        self.s.save_trajectory(rows, path, meta)

    def render_video(self, rows: np.ndarray, path: str) -> None:
        from PIL import Image
        self.s.render_video(rows, path, fps=25, camera="overview")
        Image.fromarray(self.s.render_frame("overview")).save(os.path.splitext(path)[0] + ".last.png")


def build() -> PandaDyn:
    env = os.environ.get
    texts = json.load(open(env("RCB_TEXTS", os.path.join(HERE, "texts.json"))))
    return PandaDyn(env("RCB_INSTANCE", os.path.join(HERE, "instance.json")), texts)


if __name__ == "__main__":
    main(build)
