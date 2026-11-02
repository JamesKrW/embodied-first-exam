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
  rcb_service.OBJECT_POSES  object_poses(): every free body of the venue that is not the robot (World.free: balls, bars,
                            implements, cones, the family's props; a family without one gives {}), by body name, in the
                            robot's starting frame (the frame of /spec's camera pose_t0: origin on the floor under the
                            pelvis at t = 0, x the way it faced, y left, z up), read off qpos; quaternions w x y z. A prop
                            the robot carries (a pole, a rope's handle, a ski) moves with it. Never the referee's state.
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
import sys

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


def _qmul(a, b) -> np.ndarray:
    """Hamilton product of two w x y z quaternions."""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])

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

    # ------------------------------------------------------------------ ablations (rcb_service.STATE_OPS, OBJECT_POSES)
    def object_pose_spec(self) -> dict:
        return {"frame": "the robot at t = 0 (as /spec's camera pose_t0): origin on the floor under the pelvis, x the way "
                         "it faced, y left, z up", "quaternion_order": "wxyz", "objects": list(self.ep.world.free)}

    def object_poses(self) -> dict:
        w = self.ep.world
        x0, y0, yaw0 = w.task.start
        c, s = np.cos(yaw0), np.sin(yaw0)
        Y = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        turn = np.array([np.cos(yaw0 / 2), 0.0, 0.0, -np.sin(yaw0 / 2)])   # a turn by -yaw0 about z
        q, out = w.d.qpos, {}
        for name, (qa, _) in w.free.items():
            qo = np.asarray(q[qa + 3:qa + 7], dtype=np.float64)
            out[name] = rcb_service.pose_entry(Y.T @ (np.asarray(q[qa:qa + 3]) - [x0, y0, 0.0]),
                                               _qmul(turn, qo / np.linalg.norm(qo)))
        return out

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
