"""Verifier entry point: replay /app/output/trajectory.npz from the frozen instance and write Harbor rewards.

    python /tests/grade.py --family insert_hdmi --instance /tests/instance.json --traj /app/output/trajectory.npz \
        --repeat 2 --require-identical 1 --replay-limit-s 3600

Writes /logs/verifier/reward.txt (1/0) and reward.json (numeric fields only — Harbor rejects strings) first, then
renders a human-viewable video /logs/verifier/replay.mp4 in one more replay (after the verdict, never part of it).
Replays `--repeat` times, each in its own FRESH process (`rcb_tactilelab.py replay`, no cameras, no video): on Isaac, only
the first replay in a process is bit-exact (env.reset_to does not restore every PhysX internal), so fresh process ==
fresh process is the determinism check. Success requires every replay to complete the task, and with
`--require-identical 1` (PhysX alone, or with a Newton world on Warp's deterministic kernels: cloth, cables, the soft
plug) all final-state hashes to agree, so a lucky non-deterministic run cannot pass. With `--require-identical 0` (the
dough, whose MPM solver is not guaranteed to replay bit for bit) a task passes when every one of its replays succeeds;
`deterministic` (hashes equal) and `replay_spread` (the largest difference between the replays' final state vectors)
are reported either way. The frozen scene rebuilt here from
tests/instance.json must also match the recorded one (`instance_ok`: exact state hash, or every robot, object and
particle coordinate within tolerance on another host).

Whose failure a 0 is (every path writes a reward; `deterministic`, `sim_error`, `grader_error` and `infra_error` are
always present):
- the agent's: no file (`missing_trajectory`), another family's trajectory (`wrong_family`), a file that is not a
  trajectory (`bad_trajectory`: not an npz, no `task` / `actions` entry, a member too large, pickled or object arrays,
  the wrong shape, no rows, more rows than the task's cap (`--max-rows`), inf anywhere, NaN in a gripper column), a replay that does not
  complete the task or (rigid) is not deterministic, and a simulator error on the agent's actions (`sim_error` = 1:
  from the first trajectory row on, an exception, a Newton sidecar failure, a replay process that died, or a replay
  that ran out of its time limit, `replay_timed_out`);
- the verifier's (`grader_error` = 1, with `infra_error` = 1: no verdict on the agent): a replay process that fails
  before the first trajectory row (Isaac start, scene build, reset: `replay_failed`, or its time limit there), the
  rebuilt frozen scene not matching the recorded instance (`instance_ok` = 0, `instance_mismatch` = 1: the host, not
  the trajectory), or any other error in this script.

Time limits do not depend on the agent: each verdict replay process has `--replay-limit-s`, one number per task (Isaac
start + scene build, plus the task's most control steps at half the slowest replay rate measured for the task); the
video replay has `--video-limit-s` and stops recording 300 s before it (`video_truncated`). task.toml's [verifier]
timeout_sec fits `--repeat` verdict replays, the video replay and 600 s of overhead.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import traceback
import zipfile

import numpy as np

LOG_DIR = os.environ.get("RCB_VERIFIER_LOG_DIR", "/logs/verifier")
MAX_MEMBER_BYTES = 8 << 20  # one npz member, uncompressed: (30000, 22) float64 is 5.3 MB
MAX_MEMBERS = 8


def _npy_header(z: zipfile.ZipFile, name: str):
    """(shape, dtype) of an .npy member of the zip, read from its header only (nothing is decompressed beyond it)."""
    with z.open(name) as f:
        version = np.lib.format.read_magic(f)
        if version == (1, 0):
            shape, _fortran, dtype = np.lib.format.read_array_header_1_0(f)
        else:
            shape, _fortran, dtype = np.lib.format.read_array_header_2_0(f)
    return shape, dtype


def load_trajectory(path: str, family: str, action_dim: int, max_rows: int):
    """(actions, None), or (None, reward fields) when the file is not a gradable trajectory of this family. Everything
    here is the agent's deliverable: a file that cannot be read is the agent's 0 (`bad_trajectory`), not a grader error.
    The members' sizes and the array headers are checked before anything is loaded (a small file that inflates to
    gigabytes never reaches memory)."""
    try:
        with zipfile.ZipFile(path) as z:
            infos = z.infolist()
            names = {i.filename for i in infos}
            if (
                len(infos) > MAX_MEMBERS
                or any(i.file_size > MAX_MEMBER_BYTES for i in infos)
                or not {"task.npy", "actions.npy"} <= names
            ):
                return None, {"bad_trajectory": 1}
            shape, dtype = _npy_header(z, "actions.npy")
            tshape, tdtype = _npy_header(z, "task.npy")
        if dtype.hasobject or dtype.kind not in "fiu" or tdtype.kind != "U" or tshape != ():
            return None, {"bad_trajectory": 1}
        if len(shape) != 2 or shape[1] != action_dim or not (1 <= shape[0] <= max_rows):
            return None, {"bad_trajectory": 1, "n_actions": int(shape[0]) if len(shape) == 2 else 0}
        d = np.load(path, allow_pickle=False)
        task = str(d["task"])
        acts = np.asarray(d["actions"], dtype=np.float64)
    except Exception as exc:  # noqa: BLE001 - not a zip / not an npz / pickled content: the agent's file
        print(f"[grade] unreadable trajectory: {exc!r}", file=sys.stderr, flush=True)
        return None, {"bad_trajectory": 1}
    if task != family:
        return None, {"wrong_family": 1}
    if np.any(np.isinf(acts)) or not np.all(np.isfinite(acts[:, 14:16])):  # NaN = hold / no gap override
        return None, {"bad_trajectory": 1, "n_actions": int(acts.shape[0])}
    return acts, None


def write(ok: bool, out: dict) -> None:
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(os.path.join(LOG_DIR, "reward.txt"), "w") as f:
        f.write("1\n" if ok else "0\n")
    with open(os.path.join(LOG_DIR, "reward.json"), "w") as f:
        json.dump(out, f)
    print(json.dumps(out), flush=True)


def run_replay(tool: str, a, idx: str, limit: float, video: str | None = None) -> dict:
    """One fresh-process replay. Returns its --result record plus `rc`, `timed_out`, `wall_s`."""
    res_path = os.path.join("/tmp", f"rcb_replay_{idx}.json")
    state_path = os.path.join("/tmp", f"rcb_state_{idx}.npy")
    for p in (res_path, state_path):
        if os.path.exists(p):
            os.remove(p)
    cmd = [sys.executable, tool, "replay", "--task", a.family, "--instance", a.instance, "--traj", a.traj,
           "--result", res_path]
    if video:
        cmd += ["--video", video, "--fps", str(a.video_fps), "--video-deadline-s", str(max(60.0, limit - 300.0))]
    else:
        cmd += ["--final-state", state_path]
    t0 = time.time()
    timed_out, rc = False, None
    try:
        p = subprocess.run(cmd, stdout=sys.stderr, stderr=sys.stderr, timeout=limit,
                           env={**os.environ, "RCB_SESSION_LOG": ""})
        rc = p.returncode
    except subprocess.TimeoutExpired:
        timed_out = True
    rec: dict = {}
    try:
        with open(res_path) as f:
            rec = json.load(f)
    except Exception:  # noqa: BLE001 - no record: the process died before it wrote one
        rec = {}
    rec.update(rc=rc, timed_out=timed_out, proc_wall_s=round(time.time() - t0, 1), state_path=state_path)
    return rec


def classify(rec: dict) -> str:
    """'infra' (no agent row ran, or the scene is not the instance), 'sim_error' (the agent's), or 'done'."""
    phase = rec.get("phase")
    if rec.get("instance_matches") is False:  # the CLI stops before the first row when the scene is not the instance
        return "instance_mismatch"
    if phase == "done":
        return "sim_error" if rec.get("sim_error") else "done"
    if phase == "agent":  # died or timed out while the agent's rows were stepped
        return "sim_error"
    return "infra"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--family", required=True)
    ap.add_argument("--instance", required=True)
    ap.add_argument("--traj", default="/app/output/trajectory.npz")
    ap.add_argument("--repeat", type=int, default=2)
    ap.add_argument("--require-identical", type=int, default=1)
    ap.add_argument("--replay-limit-s", type=float, default=3600.0)
    ap.add_argument("--max-rows", type=int, default=0, help="the task's row cap (0: the CLI's MAX_ROWS)")
    ap.add_argument("--video-limit-s", type=float, default=0.0, help="the video replay's time limit (0: --replay-limit-s)")
    ap.add_argument("--no-video", action="store_true")
    ap.add_argument("--video-fps", type=int, default=15)
    a = ap.parse_args()
    os.environ["RCB_SESSION_LOG"] = ""  # the verifier's own sessions must not be counted
    ZERO = {"success": 0, "deterministic": 0, "sim_error": 0, "grader_error": 0, "infra_error": 0, "replays": 0}
    ERR = {**ZERO, "grader_error": 1, "infra_error": 1}  # no verdict on the agent
    if not os.path.isfile(a.traj):
        write(False, {**ZERO, "missing_trajectory": 1})
        return 0
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # the verifier's own copy of the tooling
        from rcb_tactilelab import ACTION_DIM, MAX_ROWS

        max_rows = min(MAX_ROWS, a.max_rows) if a.max_rows > 0 else MAX_ROWS
        acts, bad = load_trajectory(a.traj, a.family, ACTION_DIM, max_rows)
        if bad is not None:
            write(False, {**ZERO, **bad})
            return 0
        tool = os.path.join(os.path.dirname(os.path.abspath(__file__)), "rcb_tactilelab.py")
        t0 = time.time()
        limit = float(a.replay_limit_s)
        base = {"n_actions": int(len(acts)), "replay_timeout_s": round(limit, 1)}
        recs = []
        for i in range(max(1, a.repeat)):
            rec = run_replay(tool, a, str(i), limit)
            recs.append(rec)
            kind = classify(rec)
            print(f"[grade] replay {i + 1}: {kind} {json.dumps(rec)}", file=sys.stderr, flush=True)
            if kind != "done" or not rec.get("success"):
                break  # the verdict is settled (an infra failure, the agent's sim error, or a failed replay)
        kinds = [classify(r) for r in recs]
        common = {**base, "replays": len(recs), "replay_wall_s": round(time.time() - t0, 1)}
        if "infra" in kinds:
            r = recs[kinds.index("infra")]
            write(False, {**ERR, **common, "replay_failed": 1, "replay_timed_out": int(bool(r.get("timed_out")))})
            return 0
        if "instance_mismatch" in kinds:
            write(False, {**ERR, **common, "instance_ok": 0, "instance_mismatch": 1})
            return 0
        if "sim_error" in kinds:
            r = recs[kinds.index("sim_error")]
            print(f"[grade] simulator error on the agent's actions: {r.get('sim_error') or 'replay process ended'}",
                  file=sys.stderr, flush=True)
            write(False, {**ZERO, **common, "sim_error": 1, "replay_timed_out": int(bool(r.get("timed_out"))),
                          "replay_crashed": int(not r.get("sim_error") and not r.get("timed_out")),
                          "sim_steps": int(r.get("sim_steps") or 0)})
            return 0
        all_success = all(bool(r.get("success")) for r in recs) and len(recs) == max(1, a.repeat)
        hashes = {r.get("state_sha") for r in recs}
        deterministic = len(recs) >= 2 and len(hashes) == 1
        spread = 0.0
        if len(recs) >= 2:
            try:
                st = [np.load(r["state_path"], allow_pickle=False) for r in recs]
                spread = max(float(np.max(np.abs(s - st[0]))) if s.shape == st[0].shape else float("inf")
                             for s in st[1:])
            except Exception:  # noqa: BLE001 - a diagnostic
                spread = -1.0
        ok = all_success and (deterministic or not a.require_identical)
        out = {
            **ZERO,
            **common,
            "success": int(ok),
            "all_success": int(all_success),
            "deterministic": int(deterministic),
            "require_identical": int(bool(a.require_identical)),
            "replay_spread": spread if np.isfinite(spread) else -1.0,
            "instance_ok": 1,
            "instance_exact": int(all(bool(r.get("instance_exact")) for r in recs)),
            "sim_steps": int(recs[0].get("sim_steps") or 0),
        }
        write(ok, out)
    except Exception:  # noqa: BLE001
        traceback.print_exc(file=sys.stderr)
        write(False, {**ERR})
        return 0
    # the video: one more replay with the cameras on, after the verdict is written (never part of it)
    if not a.no_video:
        video = os.path.join(LOG_DIR, "replay.mp4")
        try:
            rec = run_replay(tool, a, "video", float(a.video_limit_s) or limit, video=video)
            out["video_rendered"] = int(os.path.isfile(video) and os.path.getsize(video) > 0)
            out["video_truncated"] = int(bool(rec.get("video_truncated")) or rec.get("phase") != "done")
            out["video_state_match"] = int(rec.get("state_sha") is not None and rec.get("state_sha") == recs[0].get("state_sha"))
            out["video_wall_s"] = float(rec.get("proc_wall_s") or 0.0)
        except Exception:  # noqa: BLE001
            traceback.print_exc(file=sys.stderr)
            out["video_rendered"] = 0
        with open(os.path.join(LOG_DIR, "reward.json"), "w") as f:  # the same verdict, with the video's fields
            json.dump(out, f)
    return 0


if __name__ == "__main__":
    sys.exit(main())
