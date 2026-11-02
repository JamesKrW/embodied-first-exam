#!/usr/bin/env python3
"""The no-success-signal ablation of the shared eai-standard/2.1 service (RCB_HIDE_SUCCESS; docs/protocol/standard_mode.md
"No success signal"), on a toy simulator over the service's whole request surface (handle()), for every distinct
rcb_service.py in embodied_first_exam/benchmarks/ and for the 2.3 realtime layer on top of it:

  - switch off: the answers to a fixed request script are the ones the same script gets without the switch in the
    code (`--against <file>`: an older rcb_service.py), and /finish is a 404;
  - switch on: a success changes no answer (done, reason, executed rows, resets_left, 409s, the adapter's
    hidden_info_keys); /reset after a hidden success works as before one; POST /finish ends the episode, after it
    every call but GET /spec and /status is a 409; the verdict is the state's at /finish, at the agent's end and at
    the step cap (a success undone is 0, a success at the end is 1), an end by the task's failure rule is 0;
  - a crash after a hidden success resumes it hidden (the end state's success is restored from the checkpoint).
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
import rcb1  # noqa: E402

fails: list[str] = []
VOLATILE = {"time_s", "time_left_s", "elapsed_s", "lag_s"}


def check(cond, what):
    if not cond:
        fails.append(what)
        print("FAIL", what)


class Toy:
    """x moves by row[0]; x >= 1 is a success (step() says done with it); row[1] > 0 ends the episode by the task's
    rule ("failed"); info carries `progress` (declared hidden) and `blocked`."""
    raw_dim = 2
    control_dt = 0.01
    max_control_steps = 40
    proprio_layout = ["x"]
    cameras = ["c"]
    depth_cameras: list = []
    hidden_info_keys = ("progress",)

    def __init__(self):
        self.reset()

    def spec(self):
        return {"cameras": {}, "action_types": {"joints": {"dim": 2}}}

    def interpolable(self, t):
        return False

    def images(self, cams, depth):
        return {c: np.full((2, 2, 3), int(self.n) % 255, np.uint8) for c in cams}, {}

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
        info = {"progress": round(min(self.x, 1.0), 3)}
        if self.x >= 1:
            return True, True, info
        if row[1] > 0:
            return True, False, info
        return False, False, info

    def reset(self):
        self.x, self.n = 0.0, 0

    def state_sha(self):
        return hashlib.sha256(np.array([self.x, self.n]).tobytes()).hexdigest()[:12]

    def save_trajectory(self, rows, path, meta):
        np.savez(path, actions=rows)

    def render_video(self, rows, path):
        with open(path, "wb") as f:
            f.write(b"mp4 %d" % len(rows))


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def answer(reply) -> dict:
    code, ctype, body = reply
    if ctype == "application/x-rcb1":
        meta, parts = rcb1.decode(body)
        meta = {k: v for k, v in meta.items() if k not in VOLATILE}
        return {"code": code, "meta": meta, "parts": sorted(parts)}
    obj = json.loads(body)
    return {"code": code, "json": {k: v for k, v in obj.items() if k not in VOLATILE} if isinstance(obj, dict) else obj}


def submit(rows):
    return json.dumps({"type": "joints", "actions": rows, "cameras": ["c"], "video": "frames"}).encode()


class Run:
    def __init__(self, S, hide, budget=2, data=None, realtime=None):
        self.d = data or tempfile.mkdtemp()
        self.S, self.R = S, realtime
        kw = dict(data=self.d, reset_budget=budget, hide_success=hide)
        self.svc = (realtime.RealtimeService(Toy(), slowdown=1.0, **kw) if realtime else S.Service(Toy(), **kw))
        self.svc.start()

    def call(self, method, path, body=b""):
        if hasattr(self.svc, "dispatch"):                    # realtime: one job through the simulator thread's door
            job = self.R._Job(method, path, body)
            self.svc.dispatch(job)
            kind, *rest = job.reply
            if kind == "raw":
                return answer(tuple(rest))
            meta, payload = rest
            return answer(self.svc._reply_rcb1(meta, payload.parts() if kind == "rec" else payload))
        return answer(self.svc.handle(method, path, body))

    def end(self):
        self.svc.stopping = True
        self.svc.poll()
        fin = json.load(open(os.path.join(self.d, "final.json")))
        shutil.rmtree(self.d, ignore_errors=True)
        return fin


SCRIPT = [("GET", "/spec", b""), ("POST", "/observe", b"{}"), ("POST", "/submit", submit([[0.3, 0], [0.3, 0]])),
          ("GET", "/status", b""), ("POST", "/finish", b"{}"), ("POST", "/reset", b"{}"),
          ("POST", "/submit", submit([[0.6, 0], [0.6, 0], [0.1, 0]])), ("POST", "/submit", submit([[0.1, 0]])),
          ("POST", "/observe", b"{}"), ("POST", "/reset", b"{}"), ("POST", "/submit", submit([[0.2, 1]])),
          ("POST", "/reset", b"{}"), ("GET", "/status", b"")]


def script(S, hide, realtime=None):
    r = Run(S, hide, realtime=realtime)
    out = [r.call(*c) for c in SCRIPT]
    return out, r.end()


def test_off(S, tag, against: Path | None):
    out, fin = script(S, False)
    check(out[4]["code"] == 404, f"{tag}: switch off: /finish is not a 404: {out[4]}")
    check("success_signal" not in out[0]["json"] and "success_signal" not in fin, f"{tag}: switch off: hidden-mode keys")
    check(out[6]["meta"]["reason"] == "success" and out[6]["meta"]["executed_rows"] == 2,
          f"{tag}: switch off: a success no longer ends the episode: {out[6]}")
    if against is not None:
        O = load(against, "rcb_service_against")
        r = Run.__new__(Run)
        r.d = tempfile.mkdtemp()
        r.svc = O.Service(Toy(), data=r.d, reset_budget=2)
        r.svc.start()
        old = [r.call(*c) for c in SCRIPT]
        ofin = r.end()
        check(old == out, f"{tag}: switch off: answers differ from {against}: "
              + str([(i, a, b) for i, (a, b) in enumerate(zip(old, out)) if a != b][:2]))
        keep = {k: v for k, v in fin.items() if k not in VOLATILE}
        okeep = {k: v for k, v in ofin.items() if k not in VOLATILE}
        check(keep == okeep, f"{tag}: switch off: final.json differs: {set(keep.items()) ^ set(okeep.items())}")


def test_on(S, tag, realtime=None):
    out, fin = script(S, True, realtime)
    spec, obs, sub1, st1, fin1, rst1, sub2, sub3, obs2, rst2, sub4, rst3, st2 = out
    check("success_signal" in spec["json"] and "a success ends" not in json.dumps(spec["json"]),
          f"{tag}: hide: /spec has no success_signal or still says a success ends the trial")
    check(fin1["code"] == 200 and fin1["json"]["done"] is True and fin1["json"]["reason"] == "finished",
          f"{tag}: hide: /finish did not end the episode: {fin1}")
    check(rst1["code"] == 409 and sub2["code"] == 409, f"{tag}: hide: calls after /finish not refused: {rst1} {sub2}")
    if not realtime:
        check(obs2["code"] == 409, f"{tag}: hide: /observe after /finish not refused: {obs2}")
    check(st2["code"] == 200 and st2["json"]["reason"] == "finished", f"{tag}: hide: /status after /finish: {st2}")
    check(fin["success"] is False and fin["ended_by"] == "finished" and fin["hidden_success_seen"] is False
          and fin["success_signal"] == "hidden", f"{tag}: hide: finish at x=0.6: {fin}")

    # a success mid-chunk: nothing in the answers changes; reset after it works as before one; the end state is graded
    r = Run(S, True, realtime=realtime)
    a = r.call("POST", "/submit", submit([[0.6, 0], [0.6, 0], [0.1, 0]]))
    if not realtime:
        check(a["meta"]["done"] is False and a["meta"]["reason"] is None and a["meta"]["executed_rows"] == 3
              and "progress" not in a["meta"] and a["meta"]["resets_left"] == 2,
              f"{tag}: hide: a success shows in the answer: {a['meta']}")
    st = r.call("GET", "/status")
    check(st["json"]["done"] is False and st["json"]["reason"] is None and st["json"]["resets_left"] == 2,
          f"{tag}: hide: a success shows in /status: {st}")
    rs = r.call("POST", "/reset", b"{}")
    check(rs["code"] == 200 and rs["meta"]["resets_left"] == 1, f"{tag}: hide: reset after a hidden success: {rs}")
    r.svc._step("joints", np.array([0.5, 0.0]))
    r.svc._step("joints", np.array([0.6, 0.0]))          # x = 1.1: a success, held to the end
    f = r.call("POST", "/finish", b"{}")
    fin = r.end()
    check(f["code"] == 200 and fin["success"] is True and fin["ended_by"] == "finished" and fin["hidden_success_seen"]
          and fin["trial_hidden_success_seen"] and fin["first_success"] is False and fin["resets"] == 1,
          f"{tag}: hide: success at /finish after a reset: {fin}")

    # undone: success, then away from it, then the agent's end (collect hook)
    r = Run(S, True, realtime=realtime)
    r.svc._step("joints", np.array([1.2, 0.0]))
    r.svc._step("joints", np.array([-0.5, 0.0]))
    fin = r.end()
    check(fin["success"] is False and fin["hidden_success_seen"] is True and fin["ended_by"] == "timeout",
          f"{tag}: hide: a success undone was graded: {fin}")

    # success at the agent's end; and at the step cap
    r = Run(S, True, realtime=realtime)
    r.svc._step("joints", np.array([1.2, 0.0]))
    fin = r.end()
    check(fin["success"] is True and fin["first_success"] is True and fin["ended_by"] == "timeout",
          f"{tag}: hide: success at the agent's end: {fin}")
    r = Run(S, True, realtime=realtime)
    for _ in range(Toy.max_control_steps):
        if r.svc.done:
            break
        r.svc._step("joints", np.array([0.05, 0.0]))
    check(r.svc.done and r.svc.reason == "step_limit", f"{tag}: hide: step cap: {r.svc.reason}")
    fin = r.end()
    check(fin["success"] is True and fin["ended_by"] == "step_limit", f"{tag}: hide: success at the step cap: {fin}")

    # the task's failure rule after a hidden success: 0
    r = Run(S, True, budget=0, realtime=realtime)
    r.svc._step("joints", np.array([1.2, 0.0]))
    r.svc._step("joints", np.array([-0.5, 1.0]))
    fin = r.end()
    check(fin["success"] is False and fin["ended_by"] == "failed", f"{tag}: hide: failure after success: {fin}")

    # an adapter whose success is a latched event: the first success survives a later failure rule, and an end with the
    # state undone; a success before a reset does not count
    Toy.latched_success = True
    try:
        r = Run(S, True, budget=0, realtime=realtime)
        r.svc._step("joints", np.array([1.2, 0.0]))
        r.svc._step("joints", np.array([-0.5, 1.0]))
        fin = r.end()
        check(fin["success"] is True and fin["ended_by"] == "failed", f"{tag}: hide: latched, failure after it: {fin}")
        r = Run(S, True, budget=0, realtime=realtime)
        r.svc._step("joints", np.array([1.2, 0.0]))
        r.svc._step("joints", np.array([-0.5, 0.0]))
        fin = r.end()
        check(fin["success"] is True and fin["ended_by"] == "timeout", f"{tag}: hide: latched, undone: {fin}")
        r = Run(S, True, realtime=realtime)
        r.svc._step("joints", np.array([1.2, 0.0]))
        r.svc._reset({}, {})
        fin = r.end()
        check(fin["success"] is False, f"{tag}: hide: latched, then a reset: {fin}")
    finally:
        del Toy.latched_success

    # a crash after a hidden success: resumed hidden, the end state's success restored
    if not realtime:
        r = Run(S, True)
        r.svc._step("joints", np.array([1.2, 0.0]))
        r.svc._checkpoint("submit")
        with open(os.path.join(r.d, "startups.jsonl"), "a") as fh:
            fh.write("{}\n{}\n")
        r2 = Run(S, True, data=r.d)
        check(r2.svc.now_success is True and r2.svc.done is False and r2.svc.seen_step == 1,
              f"{tag}: hide: resume lost the hidden success: {r2.svc.now_success} {r2.svc.done}")
        fin = r2.end()
        check(fin["success"] is True, f"{tag}: hide: resumed hidden success not graded: {fin}")


def main() -> None:
    against = Path(sys.argv[sys.argv.index("--against") + 1]) if "--against" in sys.argv else None
    copies: dict[str, list[Path]] = {}
    for p in sorted((ROOT / "embodied_first_exam/benchmarks").glob("*/*/rcb_service.py")):
        copies.setdefault(hashlib.sha256(p.read_bytes()).hexdigest(), []).append(p)
    tested = 0
    for h, paths in copies.items():
        if "def _finish" not in paths[0].read_text():
            print(f"note: {len(paths)} copy(ies) without the no-success-signal switch ({h[:10]}): "
                  + ", ".join(str(p.relative_to(ROOT)) for p in paths))
            continue
        S = load(paths[0], f"rcb_service_{h[:8]}")
        tag = paths[0].relative_to(ROOT)
        test_off(S, tag, against)
        test_on(S, tag)
        sys.modules["rcb_service"] = S                    # the realtime layer on top of this copy
        R = load(ROOT / "embodied_first_exam/standard/realtime/rcb_realtime.py", "rcb_realtime_t")
        test_on(S, f"{tag} + rcb_realtime", realtime=R)
        tested += len(paths)
    check(tested > 0, "no rcb_service.py copy with the switch")
    if fails:
        print(f"{len(fails)} failure(s)")
        sys.exit(1)
    print(f"ok no-success-signal switch: {tested} rcb_service.py copies + rcb_realtime: off = unchanged, on = no "
          "success leak, /finish, end-state verdict, resume")


if __name__ == "__main__":
    main()
