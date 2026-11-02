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
  rcb_service.OBJECT_POSES  object_poses(): the privileged twin's entities (Session.state()["entities"], its names),
                            position and orientation only, in the world frame of /spec "frames"; quaternions w x y z.
  rcb_service.STATE_OPS     save_state() / load_state(): every array of MuJoCo's data (derived quantities included), the
                            plain values (flags, counters, numbers) of the environment, the native task, its entities and
                            its condition tree (the event flags state_sha hashes among them), and the Session's targets,
                            recorded rows and end flag. In memory, by name, so a state saved before a reset (which
                            rebuilds the scene) loads into the rebuilt one. MuJoCo's state is exact; the task's Python state
                            is exact as far as it is held in plain values.
"""

from __future__ import annotations

import os
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

    # ---------------------------------------------------------------- ablations (rcb_service.STATE_OPS, OBJECT_POSES)
    def object_pose_spec(self) -> dict:
        return {"frame": frame(self.s.spec), "quaternion_order": "wxyz", "objects": list(self.s.env.task.entities)}

    def object_poses(self) -> dict:
        ph = self.s.physics
        return {n: rcb_service.pose_entry(e.get_xpos(ph), e.get_xqaut(ph)) for n, e in self.s.env.task.entities.items()}

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
