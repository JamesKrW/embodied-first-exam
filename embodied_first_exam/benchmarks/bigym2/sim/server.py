"""eai-standard/2.1 sim service for BiGym 2.0 -- the sidecar's entry point (`python /srv/server.py`; the standard-realtime
twin runs `python /srv/rcb_realtime.py`, which uses this adapter unchanged).

The simulator-specific half only: `BiGym2` implements rcb_service.Sim for one frozen instance, through rcb_bigym2.Episode
(upstream's official env and its coding-agent action contract). The protocol, the episode, crash recovery and the
finalize live in rcb_service.py / rcb_realtime.py, which import no simulator.
Settings come from environment/docker-compose.yaml:
  RCB_INSTANCE     the frozen instance (the privileged twin's instance.json)
  RCB_TASK_TEXT    the task, for /spec (the instruction's sentence)
  RCB_EPISODE      what ends an episode, for /spec (the instruction's words)
  RCB_IMAGE_HW     the side of every onboard camera's square image, pixels (default 84, upstream's)
  RCB_RESET_BUDGET, RCB_WALL_CLOCK_S, RCB_DATA, RCB_SLOWDOWN   the service's own
The verdict is the live episode's: step() reports upstream's is_success (the task's success held for 1 s,
no fall) and the episode ends where upstream ends it, so nothing is replayed to grade it; render_video() re-runs the
trajectory once, after the verdict, to film it. The frozen start is checked when the scene is built (instance_check: the
instance's t0 state exactly, or within upstream's tolerance on another CPU): a scene that does not rebuild it is
InstanceMismatch, and start_sha (the start this image accepts) is checked by rcb_service when an episode begins.
What leaves this process: the three onboard cameras' images and upstream's low-dimensional observation (the robot's
joint angles and velocities, gripper states, the pelvis pose its base controller tracks). Never an object's pose, the
success check or its parts.

Ablations (off unless the task sets them; nothing here runs without them):
  rcb_service.STATE_QUERY   state_table(): the robot (the pelvis: pose, velocity, height above the floor; every joint
                            with its range), its torso, wrists, feet and both grippers' points between the finger pads,
                            every free body outside the robot (the task's props: pose, velocity, extents), every body
                            outside the robot with hinge / slide joints (doors, drawers, racks: an articulation with its
                            joints) and every fixed body with collision geometry (counters, cabinets, walls: a scene
                            feature with pose and extents), in MuJoCo's world frame: the room's, the frame of proprio's
                            pelvis_x, pelvis_y, pelvis_z, pelvis_yaw; quaternions w x y z. A query copies the live joint
                            state into a private MjData and runs MuJoCo's kinematics there, between two control steps.
  rcb_service.STATE_OPS     save_state() / load_state(): the episode's sim rows since the frozen start, re-run from a
                            reset on load (upstream's env, its whole-body controller and its success timers keep state
                            no array snapshot reaches; the same rows from the same start give the same state, which the
                            service's own crash recovery relies on). Exact (the state hash is checked after the re-run);
                            a load costs one control step per saved row.
"""
from __future__ import annotations

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rcb_bigym2 import CAMERAS, Episode, load_instance, replay  # noqa: E402
import rcb_service  # noqa: E402
from rcb_service import InstanceMismatch, main  # noqa: E402

ACTION = "whole_body"


class BiGym2:
    """One frozen BiGym 2.0 instance behind the rcb_service.Sim contract."""

    def __init__(self, instance: str, task_text: str = "", episode: str = "", image_hw: int = 84):
        self.instance = load_instance(instance)
        self.ep = Episode(self.instance, image_hw=image_hw)
        check = self.ep.instance_check(self.instance)
        if not check["ok"]:
            raise InstanceMismatch(f"this image does not rebuild the frozen instance's starting state (state "
                                   f"{self.ep.t0_sha}, instance {self.instance['t0']['state_sha']}, max joint deviation "
                                   f"{check['max_joint_dev']:.2e})")
        self.start_sha = self.ep.t0_sha   # the instance's start (exact, or within tolerance): every episode begins there
        self.task_text, self.episode = task_text, episode
        self.control_dt, self.max_control_steps = self.ep.control_dt, self.ep.horizon
        self.cameras, self.depth_cameras = CAMERAS, ()
        self.proprio_layout = list(self.ep.proprio_layout)
        self.raw_dim = self.ep.action_dim
        self.hw = int(image_hw)
        self.mounts = self.instance["t0"]["cameras"]
        self.facts = self.instance["t0"]["robot"]  # the robot's description: joint axes, camera mounts
        self.cmd = self.ep.hold_row()
        self._rows: list = []                      # rcb_service.STATE_OPS: the sim rows since the frozen start

    # ------------------------------------------------------------------ the rcb_service.Sim contract
    def spec(self) -> dict:
        ep = self.ep
        return {
            "simulator": "bigym2 (BiGym 2.0, bigym 1.0.0 at 93ad4b3; MuJoCo 3.8.1)",
            "robot": "Unitree G1 29-DoF with Dex1-1 two-finger grippers; NVIDIA's GR00T-WBC drives the legs and waist",
            "task_text": self.task_text,
            "episode": self.episode,
            "action_types": {ACTION: {
                "dim": ep.action_dim, "layout": ep.layout,
                "lower": [round(float(x), 4) for x in ep.lo], "upper": [round(float(x), 4) for x in ep.hi],
                "units": "vx, vy: walking velocity command, m/s, the robot's frame (x forward, y left); height: pelvis "
                         "height command, m; wz: turning rate command, rad/s (+ turns left); pitch (when present): torso "
                         "lean command, rad (+ leans forward); *_joint: absolute arm joint targets, rad; left_gripper, "
                         "right_gripper: 0 open .. 1 closed",
                "semantics": "rows dt apart (a multiple of control_dt); each control step's command is the row (hold) or "
                             "a straight line from the previous row's (linear); values outside lower/upper are clipped "
                             "and reported. The whole-body controller turns the base commands into leg and waist "
                             "motion every control step; it keeps the robot balanced but does not stop it from "
                             "walking into things"}},
            "cameras": {c: {"mounted": "head" if c == "head" else "wrist", "hw": [self.hw, self.hw],
                            "fovy_deg": self.mounts[c]["fovy_deg"],
                            "pose_t0": {"frame": "the robot at t = 0: origin on the floor under the pelvis, x the way it "
                                                 "faces, y left, z up", **{k: self.mounts[c][k] for k in
                                                                           ("position_m", "forward", "up", "right")}},
                            "mount": {"frame": self.facts["camera_mount_frame"], **self.facts["camera_mounts"][c]}}
                        for c in CAMERAS},
            "joints": self.facts["joints"],
            "joint_frame": self.facts["frame"],
            "joint_sign": self.facts["joint_sign"],
            "grippers": {"meaning": self.facts["gripper_point"], **self.facts["grippers"]},
            "proprio": {"source": "upstream's low-dimensional observation (bigym), as its learned policies receive it",
                        "units": "*_pos: rad for the hinge joints, m for the grippers' finger joints (*_dex1_finger_joint_*, "
                                 "slides) and for pelvis_x_pos, pelvis_y_pos, pelvis_z_pos (pelvis_rz_pos: rad); *_vel: "
                                 "the same per second; *_gripper_state 0 open .. 1 closed; pelvis_x, pelvis_y, pelvis_z m "
                                 "and pelvis_yaw rad: the pelvis pose the base controller tracks, in the room's frame "
                                 "(z up)"},
        }

    def interpolable(self, typ: str) -> bool:
        return True

    def images(self, cameras: list[str], depth: bool) -> tuple[dict, dict]:
        return {c: self.ep.image(c) for c in cameras}, {}

    def proprio(self) -> np.ndarray:
        return self.ep.proprio()

    def clip(self, typ: str, rows: np.ndarray) -> tuple[np.ndarray, list]:
        if typ != ACTION:
            raise ValueError(f"the action type is {ACTION!r}")
        out = np.clip(rows, self.ep.lo, self.ep.hi)
        report = [{"row": int(i), "index": int(j), "reason": f"{self.ep.layout[j]} outside "
                   f"[{self.ep.lo[j]:g}, {self.ep.hi[j]:g}]"} for i, j in np.argwhere(out != rows)]
        # At the precision the simulator applies (float32, upstream's EnvTools), which command() also gives back.
        return out.astype(np.float32).astype(np.float64), report

    def to_sim(self, typ: str, row: np.ndarray) -> np.ndarray:
        return np.asarray(row, dtype=np.float32).astype(np.float64)

    def command(self, typ: str) -> np.ndarray:
        return self.cmd

    def hold(self) -> tuple[str, np.ndarray]:
        """Stand still: the last command with zero walking and turning velocity (rcb_bigym2.Episode.hold_row)."""
        return ACTION, self.ep.hold_row()

    def step(self, row: np.ndarray) -> tuple[bool, bool, dict]:
        done = self.ep.step(row)
        self.cmd = np.asarray(row, dtype=np.float64)
        if rcb_service.STATE_OPS:
            self._rows.append(self.cmd.copy())
        return done, self.ep.success(), ({"outcome": self.ep.outcome()} if done else {})

    def reset(self) -> None:
        self.ep.reset()
        self.cmd = self.ep.hold_row()
        if rcb_service.STATE_OPS:
            self._rows = []

    # ------------------------------------------------------------------ ablations (rcb_service.STATE_OPS, STATE_QUERY)
    def state_table(self) -> "MjStateTable":
        import mujoco

        m, ns = self.ep.model, getattr(self.ep.tools.robot, "namespace", "") or ""
        t = MjStateTable(self.ep, "MuJoCo's world frame, the room's: z up, the floor at z = 0 (the frame of proprio's "
                                  "pelvis_x, pelvis_y, pelvis_z, pelvis_yaw); metres, quaternions w x y z, velocities in "
                                  "world axes")
        name = lambda b: m.body(b).name  # noqa: E731
        robot = {b for b in range(1, m.nbody) if ns and name(b).startswith(ns)}
        if not robot:
            raise RuntimeError("the robot's bodies are not namespaced: the state table cannot tell them from the props")
        pelvis = m.body(ns + "pelvis").id
        rj = [j for j in range(m.njnt) if int(m.jnt_bodyid[j]) in robot and int(m.jnt_type[j]) in t.hinge_slide]
        t.add("robot", "robot", {**t.body(pelvis), "height_above_ground": ("m", lambda: float(t.data.xpos[pelvis][2])),
                                 **t.joints(rj, strip=ns)},
              note="the Unitree G1: pos / quat = its pelvis; joint_pos / joint_vel = every joint (its pelvis_x / y / z / "
                   "rz base slides, legs, waist, arms, grippers' fingers)")
        for side in ("left", "right"):
            t.add(f"{side}_gripper", "robot_link", t.site(m.site(f"{ns}{side}_end_effector").id),
                  note=f"the {side} gripper's point between its finger pads")
        for suffix in ("torso_link", "head_link", "left_wrist_yaw_link", "right_wrist_yaw_link", "left_ankle_roll_link",
                       "right_ankle_roll_link"):
            try:
                b = m.body(ns + suffix).id
            except KeyError:
                continue
            t.add(suffix, "robot_link", t.body(b))
        free = int(mujoco.mjtJoint.mjJNT_FREE)
        for b in range(1, m.nbody):
            if b in robot or not name(b) or m.body_mocapid[b] >= 0:
                continue
            own = list(range(int(m.body_jntadr[b]), int(m.body_jntadr[b]) + int(m.body_jntnum[b])))
            if any(int(m.jnt_type[j]) == free for j in own):
                t.add(name(b), "object", {**t.body(b), "size": ("m", lambda e=t.extent(b): e)})
            elif own:
                hs = [j for j in own if int(m.jnt_type[j]) in t.hinge_slide]
                if hs:
                    t.add(name(b), "articulation", {**t.body(b), **t.joints(hs)})
            elif int(m.body_weldid[b]) == 0 and int(m.body_parentid[b]) == 0 and t.colliding(b):
                t.add(name(b), "scene_feature", {**t.body(b, velocity=False), "size": ("m", lambda e=t.extent(b): e)},
                      note="fixed in the room")
        return t

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

    def save_trajectory(self, rows: np.ndarray, path: str, meta: dict) -> None:
        np.savez(path, actions=np.asarray(rows, dtype=np.float64).reshape(-1, self.raw_dim),
                 layout=np.array(self.ep.layout), task=self.ep.task,
                 **{k: str(v) for k, v in meta.items()})

    def render_video(self, rows: np.ndarray, path: str) -> None:
        """The episode's video, after the verdict: the trajectory re-run once from the frozen instance, filmed."""
        replay(self.instance, rows, video=path)
        try:  # the last frame, for the verifier's evidence
            import subprocess
            subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-sseof", "-0.1", "-i", path, "-frames:v", "1",
                            path[: -len(".mp4")] + ".last.png"], check=False, timeout=60)
        except Exception:  # noqa: BLE001
            pass


class MjStateTable:
    """rcb_service.STATE_QUERY's table over a private MjData: each query copies the live qpos / qvel / mocap poses into
    it and runs mj_kinematics, mj_comPos and mj_comVel there; the live MjData is never touched."""

    def __init__(self, ep, frame: str):
        import mujoco

        self.mj, self.ep = mujoco, ep
        self.model = ep.model
        self.data = mujoco.MjData(self.model)
        self.table = rcb_service.StateTable(frame=frame, quaternion_order="wxyz")
        self.add = self.table.add
        self.hinge_slide = (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE))

    def entities(self) -> dict:
        return self.table.entities()

    def query(self, plan: dict) -> dict:
        m, live = self.ep.model, self.ep.data
        if m is not self.model:                      # a rebuilt env: the same scene, a new model object
            self.model, self.data = m, self.mj.MjData(m)
        d = self.data
        d.qpos[:], d.qvel[:] = live.qpos, live.qvel
        d.mocap_pos[:], d.mocap_quat[:] = live.mocap_pos, live.mocap_quat
        self.mj.mj_kinematics(m, d)
        self.mj.mj_comPos(m, d)
        self.mj.mj_comVel(m, d)
        return self.table.query(plan)

    def vel(self, b: int) -> np.ndarray:
        out = np.zeros(6)
        self.mj.mj_objectVelocity(self.model, self.data, self.mj.mjtObj.mjOBJ_BODY, b, out, 0)
        return out                                   # [angular, linear], world axes, at the body's origin

    def body(self, b: int, velocity: bool = True) -> dict:
        f = {"pos": ("m", lambda: self.data.xpos[b].copy()), "quat": ("wxyz", lambda: self.data.xquat[b].copy())}
        if velocity:
            f["lin_vel"] = ("m/s", lambda: self.vel(b)[3:])
            f["ang_vel"] = ("rad/s", lambda: self.vel(b)[:3])
        return f

    def site(self, i: int) -> dict:
        def quat():
            q = np.zeros(4)
            self.mj.mju_mat2Quat(q, self.data.site_xmat[i])
            return q
        return {"pos": ("m", lambda: self.data.site_xpos[i].copy()), "quat": ("wxyz", quat)}

    def joints(self, ids: list, strip: str = "") -> dict:
        m = self.model
        names = [m.joint(j).name.removeprefix(strip) for j in ids]
        qa = [int(m.jnt_qposadr[j]) for j in ids]
        va = [int(m.jnt_dofadr[j]) for j in ids]
        kinds = {int(m.jnt_type[j]) for j in ids}
        slide = int(self.mj.mjtJoint.mjJNT_SLIDE)
        unit, vunit = (("rad or m (hinge / slide)", "rad/s or m/s (hinge / slide)") if len(kinds) > 1 else
                       (("m", "m/s") if kinds == {slide} else ("rad", "rad/s")))
        rng = {n: m.jnt_range[j].tolist() for n, j in zip(names, ids) if m.jnt_limited[j]}
        out = {"joint_pos": (unit, lambda: dict(zip(names, self.data.qpos[qa].tolist()))),
               "joint_vel": (vunit, lambda: dict(zip(names, self.data.qvel[va].tolist())))}
        if rng:
            out["joint_range"] = (unit, lambda: rng)
        return out

    def colliding(self, b: int) -> bool:
        m = self.model
        return any(int(m.geom_bodyid[g]) == b and (m.geom_contype[g] or m.geom_conaffinity[g]) for g in range(m.ngeom))

    def extent(self, b: int) -> list:
        """Full extents (m) of a body's geoms (its collision geoms when it has any) along its own axes."""
        m = self.model
        geoms = [g for g in range(m.ngeom) if int(m.geom_bodyid[g]) == b and int(m.geom_type[g]) != int(self.mj.mjtGeom.mjGEOM_PLANE)]
        geoms = [g for g in geoms if m.geom_contype[g] or m.geom_conaffinity[g]] or geoms
        pts = []
        for g in geoms:
            R = np.zeros(9)
            self.mj.mju_quat2Mat(R, m.geom_quat[g])
            c, h = m.geom_aabb[g][:3], m.geom_aabb[g][3:]
            for sg in np.array(np.meshgrid([-1, 1], [-1, 1], [-1, 1])).T.reshape(-1, 3):
                pts.append(m.geom_pos[g] + R.reshape(3, 3) @ (c + sg * h))
        if not pts:
            return [0.0, 0.0, 0.0]
        pts = np.asarray(pts)
        return (pts.max(axis=0) - pts.min(axis=0)).tolist()


def build() -> BiGym2:
    env = os.environ.get
    return BiGym2(env("RCB_INSTANCE", "/srv/instance.json"), task_text=env("RCB_TASK_TEXT", ""),
                  episode=env("RCB_EPISODE", ""), image_hw=int(env("RCB_IMAGE_HW") or 84))


if __name__ == "__main__":
    main(build)
