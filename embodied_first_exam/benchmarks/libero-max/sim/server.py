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

Ablation (off unless the task sets it; nothing here runs without it): with rcb_service.OBJECT_POSES, object_poses() gives
the privileged twin's objects (Session.state()["objects"]: the scene's movable objects, its names; no fixture), position and
orientation only (Session.entity_pose), in the world frame of /spec "frames"; quaternions w x y z.

Ablation (off unless the task sets RCB_STATE_OPS=1 and a reset budget; build() returns LiberoMax itself without it):
/reset, /save_state and /load_state. LIBERO-MAX builds one simulator per process (lmx_tool: a second build in the same
process may differ), so under the switch the simulator lives in a worker process (`_worker`, one LiberoMax in it) and
`LiberoMaxOps` is the Sim the service holds, forwarding every call to the current worker:
  - a reset starts a new worker, a fresh build at the frozen start (the service checks its start state hash);
  - a save keeps the path (the journal rows from the frozen start) and its state hash; nothing live;
  - a load starts a new worker and runs the saved path in it from the frozen start: the physics, the controller, the
    case's event (whether and when it fired, what it changed: an object moved, the visual theme, the camera) and the
    task's goal state come back exactly as they were; the state hash is checked, else the load fails (a simulator error);
  - the worker it replaces is stopped once the new one stands; a worker that dies takes the service down with it
    (os.abort), so the sidecar restarts and resumes as after a crash of the in-process simulator.
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

    # ---------------------------------------------------------------- ablation (rcb_service.OBJECT_POSES)
    def object_pose_spec(self) -> dict:
        base = self.s.robot_base()["pos"]
        return {"frame": f"world frame: metres; the robot's base is at ({', '.join(f'{v:g}' for v in base)}) facing +x: x "
                         "forward, y to the robot's left, z up", "quaternion_order": "wxyz",
                "objects": list(self.s.env.objects_dict)}

    def object_poses(self) -> dict:
        out = {}
        for n in self.s.env.objects_dict:
            e = self.s.entity_pose(n)
            out[n] = rcb_service.pose_entry(e["pos"], e["quat_wxyz"])
        return out

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


# ---------------------------------------------------------------- the reset + save/load ablation (rcb_service.STATE_OPS)
WORKER_START_S = float(os.environ.get("RCB_WORKER_START_S") or 600)   # a scene build in a worker: at most this long
STATIC = ("instance", "start_sha", "task_text", "episode", "hw", "control_dt", "max_control_steps", "cameras",
          "depth_cameras", "raw_dim", "lower", "upper", "out_max", "proprio_layout")


def _worker(conn, instance: str, kw: dict) -> None:
    """A worker process: one LiberoMax (one simulator) driven by (op, args) messages; every answer is ("ok", value) or
    ("err", text). Ops: "attrs" (the static attributes), "call" (a LiberoMax method), "targets", "replay" (journal
    rows from the frozen start: the state hash), "state" ("entities" or "query" of the state table, when it has one)."""
    try:
        sim = LiberoMax(instance, **kw)
    except BaseException as exc:  # noqa: BLE001 -- the parent raises it
        conn.send(("err", repr(exc)[:1000]))
        return
    table = None
    conn.send(("ok", None))
    while True:
        try:
            op, args = conn.recv()
        except (EOFError, OSError):
            return
        try:
            if op == "attrs":
                out = {k: getattr(sim, k) for k in STATIC}
            elif op == "call":
                name, a, k = args
                out = getattr(sim, name)(*a, **k)
            elif op == "targets":
                out = sim.s.targets.copy()
            elif op == "replay":
                for row in args:
                    sim.step(np.asarray(row, dtype=np.float64))
                out = sim.state_sha()
            elif op == "state":
                if table is None:
                    table = sim.state_table()
                out = table.entities() if args[0] == "entities" else table.query(args[1])
            else:
                raise ValueError(f"unknown op {op!r}")
        except BaseException as exc:  # noqa: BLE001 -- a refusal or a simulator error: the parent raises it
            conn.send(("err", f"{type(exc).__name__}: {exc}"[:1000]))
            continue
        conn.send(("ok", out))


class _Worker:
    def __init__(self, instance: str, kw: dict):
        import multiprocessing as mp  # noqa: PLC0415
        ctx = mp.get_context("spawn")
        self.conn, child = ctx.Pipe()
        self.proc = ctx.Process(target=_worker, args=(child, instance, kw), daemon=True)
        self.proc.start()
        child.close()
        if not self.conn.poll(WORKER_START_S):
            self.stop()
            raise RuntimeError(f"the simulator worker did not start in {WORKER_START_S:g} s")
        self._answer()

    def _answer(self):
        try:
            tag, out = self.conn.recv()
        except (EOFError, OSError):
            # The worker died (a native crash): the service dies with it, as it would with the simulator in-process
            # (the sidecar restarts and resumes; the crash record names the call the service was serving).
            print("[worker] the simulator worker died", file=sys.stderr, flush=True)
            os.abort()
        if tag == "err":
            raise RuntimeError(out)
        return out

    def ask(self, op: str, args=None):
        self.conn.send((op, args))
        return self._answer()

    def call(self, name: str, *a, **k):
        return self.ask("call", (name, a, k))

    def stop(self) -> None:
        with contextlib.suppress(Exception):
            self.conn.close()
        if self.proc.is_alive():
            self.proc.kill()
        self.proc.join(30)


class _RemoteSession:
    """What LiberoMax's own methods read off its Session, from the worker."""

    def __init__(self, ops: "LiberoMaxOps"):
        self.ops = ops

    @property
    def targets(self) -> np.ndarray:
        return self.ops.w.ask("targets")


class _RemoteTable:
    """The state-query ablation's table (rcb_service.STATE_QUERY), the current worker's."""

    def __init__(self, ops: "LiberoMaxOps"):
        self.ops = ops

    def entities(self) -> dict:
        return self.ops.w.ask("state", ("entities", None))

    def query(self, plan: dict) -> dict:
        return self.ops.w.ask("state", ("query", plan))


class LiberoMaxOps:
    """LiberoMax with its simulator in a worker process, for /reset, /save_state and /load_state (rcb_service.STATE_OPS);
    see the module's docstring. The Sim contract's calls go to the current worker; clip, to_sim, interpolable,
    save_trajectory and render_video are LiberoMax's own, run here."""

    def __init__(self, instance: str, **kw):
        self._kw = dict(kw)
        self.instance = instance
        self.w = _Worker(instance, self._kw)
        for k, v in self.w.ask("attrs").items():
            setattr(self, k, v)
        self.s = _RemoteSession(self)
        self._live_sha: str | None = None
        self._path: list = []            # the journal rows run in the current worker since the frozen start

    clip, to_sim, interpolable, save_trajectory, render_video = (
        LiberoMax.clip, LiberoMax.to_sim, LiberoMax.interpolable, LiberoMax.save_trajectory, LiberoMax.render_video)

    def calibrations(self) -> dict:
        return self.w.call("calibrations")

    def spec(self) -> dict:
        return self.w.call("spec")

    def images(self, cameras: list[str], depth: bool) -> tuple[dict, dict]:
        return self.w.call("images", list(cameras), bool(depth))

    def proprio(self) -> np.ndarray:
        return self.w.call("proprio")

    def command(self, type: str) -> np.ndarray:
        return self.w.call("command", type)

    def hold(self) -> tuple[str, np.ndarray]:
        return self.w.call("hold")

    def step(self, row: np.ndarray) -> tuple[bool, bool, dict]:
        out = self.w.call("step", np.asarray(row, dtype=np.float64))
        self._path.append(np.array(row, dtype=np.float64))
        return out

    def state_sha(self) -> str:
        return self.w.call("state_sha")

    def state_table(self):
        return _RemoteTable(self)

    def live_verdict(self, rows: np.ndarray, success: bool) -> dict:
        out = self.w.call("live_verdict", rows, success)
        self._live_sha = self.w.call("state_sha")
        return out

    def _replace(self, w: _Worker, path: list) -> None:
        old, self.w, self._path = self.w, w, path
        old.stop()

    def reset(self) -> None:
        self._replace(_Worker(self.instance, self._kw), [])

    def save_state(self) -> dict:
        return {"rows": np.asarray(self._path, dtype=np.float64).reshape(-1, self.raw_dim), "sha": self.state_sha()}

    def load_state(self, snap: dict) -> None:
        w = _Worker(self.instance, self._kw)
        try:
            got = w.ask("replay", snap["rows"])
        except BaseException:
            w.stop()
            raise
        if got != snap["sha"]:
            w.stop()
            raise RuntimeError(f"load_state: the saved path rebuilt {got[:12]}, not the saved state {snap['sha'][:12]}")
        self._replace(w, [np.array(r) for r in snap["rows"]])


def build() -> LiberoMax:
    env = os.environ.get
    kw = dict(task_text=env("RCB_TASK_TEXT", ""), episode=env("RCB_EPISODE", ""),
              image_hw=int(env("RCB_IMAGE_HW") or 256), max_control_steps=int(env("RCB_MAX_CONTROL_STEPS") or 9000))
    if rcb_service.STATE_OPS and int(env("RCB_RESET_BUDGET") or 0):
        return LiberoMaxOps(env("RCB_INSTANCE", "/srv/instance.json"), **kw)
    return LiberoMax(env("RCB_INSTANCE", "/srv/instance.json"), **kw)


if __name__ == "__main__":
    os.environ.setdefault("LMX_TRACE", os.path.join(tempfile.gettempdir(), "lmx-trace.jsonl"))
    main(build)
