"""
rcb_tactilelab.py - task-local tooling for the TactileLab tasks (dual-arm X-Trainer whose right gripper carries two GelSight Mini
tactile sensors; Isaac Sim 5.1 / Isaac Lab 2.3.2): frozen instance, action recording, fresh-process replay.

Concepts
--------
instance   : one task in its fixed nominal scene (`seed`), the physics device it is frozen on (`device`: PhysX on the GPU,
             as the tasks were developed; a fresh process replays a scene bit for bit on the same machine, the Newton
             sidecar's deformable body included) and the
             hash of the full scene state after reset (`state_sha`: robot joint positions / velocities, every rigid
             object's pose and velocity, the deformable body's particles). On load, `instance_exact` = hash equality and
             `instance_matches` = equality, or the robot and the rigid objects within CORE_TOL and (a task with a
             deformable body) a fixed sample of its particles (`soft_sample`) within SOFT_TOL: another GPU can settle
             to slightly different positions.
trajectory : the (T, 22) rows handed to the robot, one per control step (60 Hz: physics 1/120 s, decimation 2):
               0-6   right end-effector target  x y z qw qx qy qz   (env frame, metres; the tactile gripper)
               7-13  left end-effector target   x y z qw qx qy qz
               14,15 right / left gripper: 0 = open, 1 = closed
               16,17 optional right / left finger gap override in metres (NaN = use the open / closed command)
               18-21 optional right finger drive settings, applied from this step on (NaN = unchanged): velocity limit
                     (m/s), effort limit (N), stiffness, damping
             The right gripper's final release is physical: an open command (1 -> 0 on column 14) after a close counts as
             a release once the fingers have opened; a new close command withdraws it.
             A NaN end-effector target holds that arm where it is. The targets are tracked by a differential IK and the
             joint PD drives. A trajectory holds at most MAX_ROWS rows; a replay holds the last row for SETTLE_STEPS more
             steps before the task is judged.
budget     : no step budget; MAX_STEPS (= MAX_ROWS + SETTLE_STEPS, a full trajectory and its settle) only stops runaway
             episodes.

CLI
---
  python rcb_tactilelab.py make-instance --task insert_hdmi --seed 0 --out instance.json
  python rcb_tactilelab.py replay        --task insert_hdmi --instance instance.json --traj traj.npz [--result r.json]
                                         [--video replay.mp4 [--fps 15] [--video-deadline-s S]] [--final-state f.npz]
  python rcb_tactilelab.py run-oracle    --task insert_hdmi --instance instance.json --code oracle.py --out traj.npz

Python (what the agent gets)
----------------------------
  from rcb_tactilelab import Session
  s = Session("insert_hdmi", instance="/app/instance.json")   # ~1 min: Isaac Sim launch + scene build
  s.reset()                       # back to the frozen initial state (unlimited)
  s.objects()                     # name -> {"position", "orientation_wxyz"}
  s.proprio()                     # joint_pos (16), right / left end-effector pose, gripper widths
  s.tactile()                     # per gel pad: height map (60, 80) mm, contact mask, press depth; RGB with tactile_rgb=True
  s.step(row)                     # one control step with a (22,) row (layout above; 16 or 18 numbers are padded with NaN)
  s.move_ee(arm, pose7, steps)    # straight-line interpolation of one arm's target over `steps` control steps
  s.set_gripper(arm, closed, steps)
  s.record_start(); ...; actions = s.record_stop()
  s.status()                      # {'success', 'sim_steps', 'resets', 'state_sha', 'metrics'}
  s.replay(actions)               # resets (not in a fresh process), replays open loop, returns status
  s.save_trajectory(actions, "/app/output/trajectory.npz")
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np

ACTION_DIM = 22
MAX_ROWS = 30000  # the most rows a trajectory may hold (the grader rejects a longer one)
SETTLE_STEPS = 60
MAX_STEPS = MAX_ROWS + SETTLE_STEPS  # runaway guard of one episode: a full trajectory and the replay's settle fit
CORE_TOL = 2e-3  # instance check: robot joints and rigid object poses / velocities
SOFT_TOL = 5e-3  # instance check: the deformable body's sampled particle / node coordinates (m)
SOFT_SAMPLE = 256  # particles / nodes in the instance's soft_sample
DEVICE = os.environ.get(
    "RCB_TACTILELAB_DEVICE", "cuda:0"
)  # PhysX on the GPU, as the tasks were developed (rendering and tactile on the GPU too)
os.environ.setdefault("TACTILELAB_ENHANCED_DETERMINISM", "0")  # the physics settings the tasks were developed with
_APP = None


def _log(*a: Any) -> None:
    print("[rcb_tactilelab]", *a, file=sys.stderr, flush=True)


def _launch_app(cameras: bool):
    global _APP
    if _APP is None:
        from tactilelab_tasks.backends.isaaclab_scene import launch_app

        _APP = launch_app(headless=True, enable_cameras=bool(cameras))
    return _APP


def _flat(tree: Any) -> np.ndarray:
    parts: list[np.ndarray] = []

    def walk(x: Any) -> None:
        if isinstance(x, dict):
            for k in sorted(x):
                walk(x[k])
        elif isinstance(x, (list, tuple)):
            for v in x:
                walk(v)
        else:
            try:
                parts.append(np.asarray(x, dtype=np.float64).ravel())
            except (TypeError, ValueError):
                pass

    walk(tree)
    return np.concatenate(parts) if parts else np.zeros(0)


class Session:
    def __init__(
        self,
        task: str,
        instance: str | Path | None = None,
        *,
        seed: int | None = None,
        cameras: bool = False,
        tactile: bool = True,
        tactile_rgb: bool = False,
    ):
        self.task_name = str(task)
        self.instance = json.loads(Path(instance).read_text()) if instance else {}
        if self.instance and self.instance.get("task") != self.task_name:
            raise ValueError(f"instance is for task {self.instance.get('task')!r}, not {self.task_name!r}")
        self.seed = int(self.instance.get("seed", 0) if seed is None else seed)
        self.device = str(self.instance.get("device", DEVICE))
        _launch_app(cameras)
        import tactilelab_tasks

        if self.task_name not in tactilelab_tasks.TASKS:
            raise KeyError(f"unknown task {self.task_name!r}; known: {sorted(tactilelab_tasks.TASKS)}")
        spec = tactilelab_tasks.TASKS[self.task_name]
        self.cfg = spec.load_cfg()
        obs_mode = "full" if (cameras or tactile_rgb) else "state"
        self.task = spec.task_cls(
            self.cfg, obs_mode=obs_mode, device=self.device, cameras=bool(cameras), tactile=bool(tactile)
        )
        self.task.build()
        self.cameras = bool(cameras)
        self.resets = -1
        self.sim_steps = 0
        self._rec: list[np.ndarray] | None = None
        self._last = None
        self._pending: dict[int, float] | None = None
        self._grip_prev: int | None = None
        self.obs = None
        # the task methods a scripted policy calls to change the gripper: the Session applies them from the action rows
        # (so a replay reproduces them); calls made directly on the task are turned into the next row's columns 18-21,
        # and the policy's own release announcements are replaced by the physical rule above
        self._orig = {
            n: getattr(self.task, n)
            for n in ("set_finger_limits", "set_finger_gains", "note_finish_release", "clear_finish_release")
            if hasattr(self.task, n)
        }
        if "set_finger_limits" in self._orig:
            self.task.set_finger_limits = self._capture_limits
        if "set_finger_gains" in self._orig:
            self.task.set_finger_gains = self._capture_gains
        if "note_finish_release" in self._orig:
            self.task.note_finish_release = lambda: None
            self.task.clear_finish_release = lambda: None
        self.reset()

    def _capture_limits(self, vel_limit: float = 0.0, effort_limit: float = 0.0) -> None:
        self._pending = dict(self._pending or {})
        if vel_limit > 0.0:
            self._pending[18] = float(vel_limit)
        if effort_limit > 0.0:
            self._pending[19] = float(effort_limit)

    def _capture_gains(self, stiffness: float = 0.0, damping: float = 0.0) -> None:
        self._pending = dict(self._pending or {})
        if stiffness > 0.0:
            self._pending[20] = float(stiffness)
        if damping > 0.0:
            self._pending[21] = float(damping)

    # ------------------------------------------------------------------ state
    def _state(self) -> np.ndarray:
        st = dict(self.task.get_state())
        st.pop("_task", None)
        return _flat(st)

    def _core_state(self) -> np.ndarray:
        """Robot and rigid objects only (the instance tolerance check)."""
        st = dict(self.task.get_state())
        return _flat({k: v for k, v in st.items() if k.startswith("robot") or k == "objects"})

    def _soft_points(self) -> np.ndarray | None:
        """(N, 3) positions of the deformable body (cloth / sleeve particles, cable nodes, dough / plug particles),
        or None for a task without one."""
        st = dict(self.task.get_state())
        soft = st.get("soft") or {}
        for k in ("particle_q", "cable_nodes"):
            if k in soft and soft[k] is not None:
                return np.asarray(soft[k], dtype=np.float64).reshape(-1, 3)
        dough = st.get("dough") or {}
        if dough.get("q") is not None:
            return np.asarray(dough["q"], dtype=np.float64).reshape(-1, 3)
        return None

    def _soft_sample(self) -> np.ndarray | None:
        pts = self._soft_points()
        if pts is None or len(pts) == 0:
            return None
        idx = np.unique(np.linspace(0, len(pts) - 1, min(SOFT_SAMPLE, len(pts))).round().astype(np.int64))
        return np.concatenate([[float(len(pts))], pts[idx].ravel()])

    def state_sha(self) -> str:
        return hashlib.sha1(self._state().tobytes()).hexdigest()

    def reset(self) -> None:
        self.obs = self.task.reset(seed=self.seed)
        self.resets += 1
        self.sim_steps = 0
        self._last = None
        self._pending = None
        self._grip_prev = None
        s0 = self._state()
        self.reset_sha = hashlib.sha1(s0.tobytes()).hexdigest()
        if (
            self.resets == 0
        ):  # the instance is checked on the first reset of the process (later resets are not bit-exact)
            want = self.instance.get("state_sha")
            self.instance_exact = (want is None) or (want == self.reset_sha)
            ref = self.instance.get("core_state")
            c0 = self._core_state()
            core_ok = ref is not None and len(ref) == c0.size and float(np.max(np.abs(np.asarray(ref) - c0))) <= CORE_TOL
            sref, s0 = self.instance.get("soft_sample"), self._soft_sample()
            if s0 is None:
                soft_ok = sref is None
            else:  # same particle count (the first entry) and the sampled coordinates within SOFT_TOL
                soft_ok = (
                    sref is not None
                    and len(sref) == s0.size
                    and sref[0] == s0[0]
                    and float(np.max(np.abs(np.asarray(sref[1:]) - s0[1:]))) <= SOFT_TOL
                )
            self.instance_matches = self.instance_exact or (core_ok and soft_ok)

    # ------------------------------------------------------------------ actions
    def _action(self, row):
        from tactilelab_tasks.core.types import Action

        r = np.asarray(row, dtype=np.float64).reshape(-1)
        if r.size in (16, 18):
            r = np.concatenate([r, np.full(ACTION_DIM - r.size, np.nan)])
        if r.size != ACTION_DIM:
            raise ValueError(f"a row has {ACTION_DIM} numbers (16 or 18 are padded with NaN), got {r.size}")
        if not np.all(np.isfinite(r[14:16])):
            raise ValueError("gripper commands must be finite (0 = open, 1 = closed)")
        gap = None if np.all(np.isnan(r[16:18])) else r[16:18].copy()
        return r, Action(r[:14].copy(), np.rint(r[14:16]).astype(np.int64), gap)

    def _apply_extras(self, r: np.ndarray) -> np.ndarray:
        """Finger drive settings (columns 18-21) and the release bookkeeping of the right gripper; returns the row as
        recorded (with any drive settings the task code requested since the last step merged in)."""
        if self._pending is not None:
            for i, v in self._pending.items():
                if np.isnan(r[i]):
                    r[i] = v
            self._pending = None
        t = self.task
        vel, eff, kp, kd = (float(x) for x in r[18:22])
        if (np.isfinite(vel) or np.isfinite(eff)) and hasattr(t, "set_finger_limits"):
            self._orig["set_finger_limits"](vel if np.isfinite(vel) else 0.0, eff if np.isfinite(eff) else 0.0)
        if (np.isfinite(kp) or np.isfinite(kd)) and hasattr(t, "set_finger_gains"):
            self._orig["set_finger_gains"](kp if np.isfinite(kp) else 0.0, kd if np.isfinite(kd) else 0.0)
        cmd = int(round(r[14]))
        if hasattr(t, "note_finish_release") and self._grip_prev is not None and cmd != self._grip_prev:
            if cmd == 0:
                self._orig["note_finish_release"]()
            else:
                self._orig["clear_finish_release"]()
        self._grip_prev = cmd
        return r

    def step(self, row) -> None:
        if self.sim_steps >= MAX_STEPS:
            raise RuntimeError(f"MAX_STEPS ({MAX_STEPS}) reached")
        r, act = self._action(row)
        r = self._apply_extras(r)
        self.obs, _info = self.task.step(act)
        self.sim_steps += 1
        self._last = r
        if self._rec is not None:
            self._rec.append(r)

    def hold_action(self) -> np.ndarray:
        """A row that holds both arms and keeps the current gripper commands (drive settings unchanged)."""
        if self._last is not None:
            row = self._last.copy()
            row[18:22] = np.nan
            return row
        row = np.full(ACTION_DIM, np.nan)
        row[14:16] = 0.0
        return row

    def move_ee(self, arm: int, pose7, steps: int = 60) -> None:
        """Interpolate one arm's target (position linearly, orientation by slerp) from its current pose to `pose7`."""
        cur = np.asarray(self.task.scene.robot.ee_pose(int(arm)), dtype=np.float64)
        goal = np.asarray(pose7, dtype=np.float64).reshape(7)
        q0, q1 = cur[3:] / np.linalg.norm(cur[3:]), goal[3:] / np.linalg.norm(goal[3:])
        if float(np.dot(q0, q1)) < 0.0:
            q1 = -q1
        for i in range(1, max(1, int(steps)) + 1):
            u = i / max(1, int(steps))
            row = self.hold_action()
            pos = (1.0 - u) * cur[:3] + u * goal[:3]
            d = float(np.clip(np.dot(q0, q1), -1.0, 1.0))
            if d > 0.9995:
                q = (1.0 - u) * q0 + u * q1
            else:
                th = math.acos(d)
                q = (math.sin((1.0 - u) * th) * q0 + math.sin(u * th) * q1) / math.sin(th)
            row[7 * arm : 7 * arm + 7] = np.concatenate([pos, q / np.linalg.norm(q)])
            self.step(row)

    def set_gripper(self, arm: int, closed: bool, steps: int = 30) -> None:
        for _ in range(max(1, int(steps))):
            row = self.hold_action()
            row[14 + int(arm)] = 1.0 if closed else 0.0
            self.step(row)

    # ------------------------------------------------------------------ recording / replay
    def record_start(self) -> None:
        self._rec = []

    def record_stop(self) -> np.ndarray:
        rows, self._rec = self._rec or [], None
        return np.stack(rows) if rows else np.zeros((0, ACTION_DIM))

    def save_trajectory(self, actions: np.ndarray, path: str | Path) -> None:
        a = np.asarray(actions, dtype=np.float64)
        if a.ndim != 2 or a.shape[1] != ACTION_DIM:
            raise ValueError(f"actions must be (T, {ACTION_DIM})")
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        np.savez(path, task=np.array(self.task_name), actions=a)

    def settle(self) -> None:
        row = self.hold_action()
        for _ in range(SETTLE_STEPS):
            self.step(row)

    def _frame(self) -> np.ndarray:
        """The two wrist cameras side by side, rendered now (only for a video: a graded replay renders nothing)."""
        obs = self.task.scene.render_cameras()
        return np.concatenate([np.asarray(obs["wrist_right_rgb"]), np.asarray(obs["wrist_left_rgb"])], axis=1)

    def replay(
        self,
        actions: np.ndarray,
        settle: bool = True,
        video: str | None = None,
        fps: int = 15,
        video_deadline: float | None = None,
        on_first_row=None,
    ) -> dict[str, Any]:
        """Resets (unless nothing has run since the first reset of the process: then the replay is that first episode,
        exactly what the grader's fresh-process replay runs), replays the rows open loop, holds the last row.
        With `video` (a Session with cameras=True), frames are rendered every 60/fps steps and streamed to the file;
        at the wall-clock time `video_deadline` (time.time()) the replay stops and the video is closed
        (`video_truncated` in the status). `on_first_row` is called once, before the first row is stepped."""
        if not (self.resets == 0 and self.sim_steps == 0):
            self.reset()
        every = max(1, int(round(60 / max(1, int(fps)))))  # 60 Hz control -> `fps` video frames per second
        rows = list(np.asarray(actions, dtype=np.float64))
        if settle and rows:
            hold = rows[-1].copy()
            hold[18:22] = np.nan  # the hold keeps the drive settings in force, as settle() does
            rows += [hold] * SETTLE_STEPS
        writer, truncated = None, False
        if video and self.cameras:
            import imageio.v2 as imageio

            writer = imageio.get_writer(video, fps=int(fps), codec="libx264", quality=7, macro_block_size=8)
        try:
            if on_first_row is not None:
                on_first_row()
            for i, r in enumerate(rows):
                if writer is not None and video_deadline is not None and time.time() > video_deadline:
                    truncated = True
                    break
                self.step(r)
                if writer is not None and i % every == 0:
                    writer.append_data(self._frame())  # streamed: no frame buffer in memory
        finally:
            if writer is not None:
                try:
                    writer.close()
                except Exception as exc:  # noqa: BLE001
                    _log(f"video not closed: {exc!r}")
        st = self.status()
        if writer is not None:
            st["video_truncated"] = bool(truncated)
        return st

    # ------------------------------------------------------------------ queries
    def sim_error(self) -> str | None:
        """The simulator error the task recorded in this episode (a Newton sidecar call or the soft-body judge that
        failed: the episode cannot succeed), or None."""
        e = getattr(self.task, "sim_error_reason", None)
        return str(e) if e else None

    def status(self) -> dict[str, Any]:
        m = {}
        try:
            for k, v in self.task.metrics().items():
                if np.isscalar(v) and not isinstance(v, str):
                    m[k] = float(v)
                elif isinstance(v, str) and v:
                    m[k] = v
        except Exception:
            pass
        err = self.sim_error()
        out = {
            "success": bool(self.task.is_success()) and err is None,
            "sim_steps": int(self.sim_steps),
            "resets": int(self.resets),
            "state_sha": self.state_sha(),
            "metrics": m,
        }
        if err is not None:
            out["sim_error"] = err
        return out

    def objects(self) -> dict[str, dict]:
        out = {}
        for o in self.cfg.get("scene", {}).get("objects", []):
            n = str(o["name"])
            try:
                p = np.asarray(self.task.object_pose(n), dtype=np.float64)
            except Exception:
                continue
            out[n] = {"position": p[:3].tolist(), "orientation_wxyz": p[3:7].tolist()}
        return out

    def proprio(self) -> dict[str, Any]:
        rob = self.task.scene.robot
        return {
            "joint_pos": np.asarray(rob.joint_pos(), dtype=np.float64).tolist(),
            "right_ee_pose": np.asarray(rob.ee_pose(0), dtype=np.float64).tolist(),
            "left_ee_pose": np.asarray(rob.ee_pose(1), dtype=np.float64).tolist(),
            "gripper_width": [float(rob.gripper_width(0)), float(rob.gripper_width(1))],
        }

    def tactile(self) -> dict[str, np.ndarray]:
        read = getattr(self.task, "sensors", {}).get("tactile")
        return {} if read is None else dict(read())

    def render_frame(self, camera: str = "wrist_right_rgb") -> np.ndarray:
        if not self.cameras:
            raise RuntimeError("cameras are off: Session(..., cameras=True)")
        obs = self.task.get_observation()
        return np.asarray(obs["policy"][camera])


# ---------------------------------------------------------------------- CLI
def _shutdown(code: int) -> None:
    import threading

    if _APP is not None:
        th = threading.Thread(target=_APP.close, daemon=True)
        th.start()
        th.join(timeout=20.0)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(code)


def _write_json(path: str | None, obj: dict) -> None:
    """Write obj to path atomically (the grader reads the file, never this process's stdout)."""
    if not path:
        return
    tmp = f"{path}.part"
    with open(tmp, "w") as f:
        json.dump(obj, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def _replay_cli(a) -> dict[str, Any]:
    """`replay`: the phases the grader tells apart through the --result file.
    setup  - Isaac start, scene build, reset and the instance check: a failure here is the host's (no agent row ran);
    agent  - written just before the first trajectory row is stepped: from here on, an exception, a dead process or a
             time-out is a simulator error on the agent's actions (the agent's 0, `sim_error`);
    done   - the verdict of this replay."""
    res: dict[str, Any] = {"phase": "setup"}
    _write_json(a.result, res)
    d = np.load(a.traj, allow_pickle=False)
    acts = np.asarray(d["actions"], dtype=np.float64)
    s = Session(a.task, instance=a.instance, tactile=False, cameras=bool(a.video))
    res.update(instance_exact=bool(s.instance_exact), instance_matches=bool(s.instance_matches))
    _write_json(a.result, res)
    if not s.instance_matches:  # the scene rebuilt here is not the frozen instance: the host's, before any agent row
        raise RuntimeError("the rebuilt scene does not match the instance (instance_mismatch)")
    if s.sim_error():  # the sidecar failed while the scene was built and reset: no agent row has run
        raise RuntimeError(f"simulator error before the first row: {s.sim_error()}")
    t0 = time.time()
    deadline = (t0 + float(a.video_deadline_s)) if (a.video and a.video_deadline_s) else None

    def first_row() -> None:
        res["phase"] = "agent"
        _write_json(a.result, res)

    try:
        st = s.replay(acts, video=a.video, fps=a.fps, video_deadline=deadline, on_first_row=first_row)
    except Exception as exc:  # noqa: BLE001 - raised while the agent's rows were stepped: the agent's result
        import traceback

        traceback.print_exc()
        if res["phase"] != "agent":
            raise
        res.update(
            phase="done", success=False, sim_error=f"{type(exc).__name__}: {exc}"[:2000], sim_steps=int(s.sim_steps)
        )
        res["wall_s"] = round(time.time() - t0, 1)
        _write_json(a.result, res)
        return res
    res.update(
        phase="done",
        success=bool(st["success"]),
        state_sha=st["state_sha"],
        sim_steps=st["sim_steps"],
        metrics=st["metrics"],
        wall_s=round(time.time() - t0, 1),
    )
    if st.get("sim_error"):
        res["sim_error"] = st["sim_error"]
    if "video_truncated" in st:
        res["video_truncated"] = st["video_truncated"]
    if a.final_state:  # the full final state vector (the grader measures the spread between replays)
        np.save(a.final_state, s._state(), allow_pickle=False)
    _write_json(a.result, res)
    return res


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    m = sub.add_parser("make-instance")
    m.add_argument("--task", required=True)
    m.add_argument("--seed", type=int, default=0)
    m.add_argument("--out", required=True)
    r = sub.add_parser("replay")
    r.add_argument("--task", required=True)
    r.add_argument("--instance", required=True)
    r.add_argument("--traj", required=True)
    r.add_argument("--result", default=None, help="write the phases and the verdict of this replay to this JSON file")
    r.add_argument("--video", default=None, help="render the two wrist cameras to this mp4 (cameras on: slower)")
    r.add_argument("--fps", type=int, default=15)
    r.add_argument("--video-deadline-s", type=float, default=None, help="with --video: stop after this many seconds")
    r.add_argument("--final-state", default=None, help="save the final state vector (.npy)")
    o = sub.add_parser("run-oracle")
    o.add_argument("--task", required=True)
    o.add_argument("--instance", required=True)
    o.add_argument("--code", required=True)
    o.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    code = 1
    try:
        if a.cmd == "make-instance":
            s = Session(a.task, seed=a.seed, tactile=False)
            st = s._state()
            inst = {
                "task": a.task,
                "seed": a.seed,
                "device": s.device,
                "state_sha": s.reset_sha,
                "core_state": s._core_state().tolist(),
            }
            ss = s._soft_sample()
            if ss is not None:
                inst["soft_sample"] = [float(x) for x in ss]
            Path(a.out).write_text(json.dumps(inst, indent=1) + "\n")
            print(json.dumps({"task": a.task, "state_sha": s.reset_sha}), flush=True)
        elif a.cmd == "replay":
            res = _replay_cli(a)
            print(json.dumps({k: v for k, v in res.items() if k != "metrics"}), flush=True)
        elif a.cmd == "run-oracle":
            s = Session(a.task, instance=a.instance)
            ns = {"s": s, "Session": Session, "np": np}
            exec(compile(Path(a.code).read_text(), a.code, "exec"), ns)
            acts = ns.get("actions")
            if acts is None:
                raise RuntimeError("the oracle code must leave its rows in `actions`")
            s.settle()  # the live first episode, as a replay would end it
            st = s.status()
            s.save_trajectory(acts, a.out)
            print(
                json.dumps({"rows": int(len(acts)), **{k: st[k] for k in ("success", "state_sha", "sim_steps")}}),
                flush=True,
            )
        code = 0
    except Exception:
        import traceback

        traceback.print_exc()
    _shutdown(code)
    return code


if __name__ == "__main__":
    main()
