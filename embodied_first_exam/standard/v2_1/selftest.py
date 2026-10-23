#!/usr/bin/env python3
"""The eai-standard/2.1 service's first-attempt record (docs/protocol/standard_mode.md "Evidence and grading"; PROTOCOL
§4), on a toy simulator (numpy only, no image, a few seconds), for every distinct rcb_service.py in
embodied_first_exam/benchmarks/ (the copies are byte-identical; a copy without the record is reported, not failed):

  - one attempt that succeeds: first_success = success = 1, no first-episode files;
  - fail (the task's rule), reset, success: success 1, first_success 0 ("failed"), trajectory.first.npz holds episode 1's
    rows, replay.first.mp4 is rendered after the graded video;
  - abandon (reset mid-episode), reset, fail: first_reason "reset";
  - one attempt that times out: first_success 0 ("timeout");
  - a first-episode video that fails to render: the verdict and the graded video are unchanged (evidence only).
"""
import hashlib
import importlib.util
import json
import os
import shutil
import sys
import tempfile
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "embodied_first_exam/standard"))   # rcb1.py
fails: list[str] = []


def check(cond, what):
    if not cond:
        fails.append(what)
        print("FAIL", what)


class Toy:
    """x moves by row[0]; x >= 1 is a success; row[1] > 0 ends the episode by the task's rule ("failed")."""
    raw_dim = 2
    control_dt = 0.01
    max_control_steps = 50
    proprio_layout = ["x"]
    cameras = ["c"]
    depth_cameras: list = []
    broken_first_video = False

    def __init__(self):
        self.reset()

    def spec(self):
        return {"cameras": {}, "action_types": {"joints": {"dim": 2}}}

    def interpolable(self, t):
        return True

    def images(self, cams, depth):
        return {c: np.zeros((2, 2, 3), np.uint8) for c in cams}, {}

    def proprio(self):
        return np.array([self.x], np.float32)

    def clip(self, t, rows):
        return rows, []

    def to_sim(self, t, row):
        return np.asarray(row, float)

    def command(self, t):
        return np.zeros(2)

    def hold(self):
        return "joints", np.zeros(2)

    def step(self, row):
        self.x += row[0]
        self.n += 1
        if self.x >= 1:
            return True, True, {}
        if row[1] > 0:
            return True, False, {}
        return False, False, {}

    def reset(self):
        self.x, self.n = 0.0, 0

    def state_sha(self):
        return hashlib.sha256(np.array([self.x, self.n]).tobytes()).hexdigest()[:12]

    def save_trajectory(self, rows, path, meta):
        np.savez(path, actions=rows, **{k: str(v) for k, v in meta.items()})

    def render_video(self, rows, path):
        if self.broken_first_video and "first" in os.path.basename(path):
            raise RuntimeError("no GL")
        with open(path, "wb") as f:
            f.write(b"mp4 %d" % len(rows))


def run(S, plan, budget=5, broken=False):
    d = tempfile.mkdtemp()
    try:
        sim = Toy()
        sim.broken_first_video = broken
        svc = S.Service(sim, data=d, reset_budget=budget)
        svc.start()
        for item in plan:
            if item == "reset":
                svc._reset({"cameras": []}, {})
                continue
            for r in item:
                if svc.done:
                    break
                svc._step("joints", np.array(r, float))
            svc._checkpoint("submit")
        svc.stopping = True
        svc.poll()
        fin = json.load(open(os.path.join(d, "final.json")))
        files = {f: open(os.path.join(d, f), "rb").read() for f in os.listdir(d) if f.startswith(("replay", "trajectory."))}
        first = os.path.join(d, "trajectory.first.npz")
        fin["_first_rows"] = np.load(first)["actions"].tolist() if os.path.exists(first) else None
        return fin, files
    finally:
        shutil.rmtree(d)


def test(path: Path) -> None:
    spec = importlib.util.spec_from_file_location(f"rcb_service_{hashlib.sha1(str(path).encode()).hexdigest()[:8]}", path)
    S = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(S)
    tag = path.relative_to(ROOT)
    check({"first_success", "first_reason", "first_n_actions", "first_video"} <= S.RESERVED, f"{tag}: first_* not RESERVED")

    fin, files = run(S, [[[0.5, 0], [0.6, 0]]])
    check(fin["success"] and fin["first_success"] is True and fin["first_reason"] == "success" and fin["resets"] == 0,
          f"{tag}: one success: {fin}")
    check("first_video" not in fin and "trajectory.first.npz" not in files and "replay.first.mp4" not in files,
          f"{tag}: one success left first-episode files: {sorted(files)}")

    fin, files = run(S, [[[0.2, 0], [0.1, 1]], "reset", [[0.5, 0], [0.6, 0]]])
    check(fin["success"] and fin["first_success"] is False and fin["first_reason"] == "failed" and fin["resets"] == 1,
          f"{tag}: fail, reset, success: {fin}")
    check(fin["first_n_actions"] == 2 and fin["_first_rows"] == [[0.2, 0.0], [0.1, 1.0]] and fin["n_actions"] == 2,
          f"{tag}: fail, reset, success: first rows {fin.get('_first_rows')} / {fin.get('first_n_actions')}")
    check(fin["first_video"] is True and files.get("replay.first.mp4") == b"mp4 2" and files.get("replay.mp4") == b"mp4 2"
          and fin["video"] is True and fin["video_pending"] is False, f"{tag}: fail, reset, success: videos {sorted(files)} {fin}")

    fin, files = run(S, [[[0.2, 0], [0.1, 0], [0.1, 0]], "reset", [[0.1, 1]]])
    check(not fin["success"] and fin["first_success"] is False and fin["first_reason"] == "reset"
          and fin["first_n_actions"] == 3 and fin["reason"] == "failed", f"{tag}: abandon, reset, fail: {fin}")

    fin, files = run(S, [[[0.1, 0]]])
    check(not fin["success"] and fin["first_success"] is False and fin["first_reason"] == "timeout"
          and "trajectory.first.npz" not in files, f"{tag}: timeout: {fin}")

    good, _ = run(S, [[[0.2, 0], [0.1, 1]], "reset", [[0.5, 0], [0.6, 0]]])
    fin, files = run(S, [[[0.2, 0], [0.1, 1]], "reset", [[0.5, 0], [0.6, 0]]], broken=True)
    same = {k: fin.get(k) for k in ("success", "live_success", "reason", "n_actions", "video", "graded_by")}
    check(same == {k: good.get(k) for k in same} and fin["first_video"] is False and "replay.first.mp4" not in files
          and files.get("replay.mp4") == b"mp4 2" and fin["video_pending"] is False,
          f"{tag}: a failed first video changed the verdict: {fin}")


def main() -> None:
    copies: dict[str, list[Path]] = {}
    for p in sorted((ROOT / "embodied_first_exam/benchmarks").glob("*/*/rcb_service.py")):
        copies.setdefault(hashlib.sha256(p.read_bytes()).hexdigest(), []).append(p)
    tested = 0
    for h, paths in copies.items():
        if "def _first_episode" not in paths[0].read_text():
            print(f"note: {len(paths)} copy(ies) without the first-attempt record ({h[:10]}): "
                  + ", ".join(str(p.relative_to(ROOT)) for p in paths))
            continue
        test(paths[0])
        tested += len(paths)
    check(tested > 0, "no rcb_service.py copy with the first-attempt record")
    if fails:
        print(f"{len(fails)} failure(s)")
        sys.exit(1)
    print(f"ok first-attempt record: {tested} rcb_service.py copies ({sum(1 for ps in copies.values())} distinct): one "
          "success, fail-reset-success, abandon-reset-fail, timeout, a failed first video")


if __name__ == "__main__":
    main()
