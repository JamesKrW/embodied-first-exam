#!/usr/bin/env python3
"""The 2.3 realtime layer under RCB_STATE_OPS=1 (the unlimited-reset ablation), on the 2.1 toy simulator of
v2_1/selftest.py, in-process (numpy only, a few seconds): /save_state answers JSON with a handle while the clock runs on,
/load_state stops the clock and starts a new episode paused at the saved state, the frame cache follows the load, and
a crash resume replays the ops. With the switch off neither route exists.

    python3 embodied_first_exam/standard/realtime/ablation_selftest.py
"""
from __future__ import annotations
import hashlib, importlib.util, json, os, shutil, sys, tempfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
sys.path.insert(0, str(HERE.parent))                     # rcb1.py
sys.path.insert(0, str(HERE.parent / "v2_1"))
fails: list[str] = []


def check(cond, what):
    if not cond:
        fails.append(what); print("FAIL", what)


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod
    spec.loader.exec_module(mod)
    return mod


def layer(ops: bool):
    os.environ.pop("RCB_STATE_OPS", None)
    if ops:
        os.environ["RCB_STATE_OPS"] = "1"
    src = sorted((ROOT / "embodied_first_exam/benchmarks").glob("*/sim/rcb_service.py"))[0]
    S = load(src, "rcb_service")
    R = load(HERE / "rcb_realtime.py", f"rcb_realtime_{int(ops)}")
    os.environ.pop("RCB_STATE_OPS", None)
    return S, R


T = load(HERE.parent / "v2_1/selftest.py", "v21_selftest")   # its Toy (with save_state / load_state)


def main() -> int:
    S, R = layer(False)
    d = tempfile.mkdtemp()
    try:
        svc = R.RealtimeService(T.Toy(), data=d, reset_budget=5, slowdown=1.0)
        svc.start()
        r = svc.front("POST", "/save_state", 2, lambda n: b"{}")
        check(r[0] == "reply" and r[1] == 404, f"2.3 without the switch: /save_state is 404 ({r[:2]})")
        S2, R2 = layer(True)
        svc_on = R2.RealtimeService(T.Toy(), data=tempfile.mkdtemp(), reset_budget=5, slowdown=1.0)
        r = svc_on.front("POST", "/save_state", 2, lambda n: b"{}")
        check(r[0] == "/save_state", f"2.3 with the switch: /save_state is routed ({r[:1]})")
    finally:
        shutil.rmtree(d)

    S, R = layer(True)
    d = tempfile.mkdtemp()
    try:
        svc = R.RealtimeService(T.Toy(), data=d, reset_budget=5, slowdown=1.0)
        svc.start()
        for _ in range(3):
            svc._step("joints", np.array([0.2, 0.0])); svc.episode  # noqa: B018
        svc._checkpoint("submit")
        job = R._Job("POST", "/save_state")
        svc._rt_save({}, job)
        body = json.loads(job.reply[3])
        check(job.reply[0] == "raw" and job.reply[1] == 200 and body["handle"] == "s1" and body["step"] == 3,
              f"2.3 save: {job.reply[:2]} {body}")
        svc._step("joints", np.array([-0.5, 0.0])); svc._checkpoint("submit")
        svc.running = True                                   # as if the clock ran
        job = R._Job("POST", "/load_state")
        svc._rt_load({"handle": "s1", "cameras": []}, job)
        check(job.reply[0] == "rcb1" and job.reply[1]["loads"] == 1 and job.reply[1]["episode"] == 2 and not svc.running
              and abs(svc.sim.x - 0.6) < 1e-9 and len(svc.episode) == 3, f"2.3 load: {job.reply[1]} running={svc.running}")
        svc._step("joints", np.array([0.1, 0.0])); svc._checkpoint("submit")
        sha = svc.sim.state_sha()
        svc2 = R.RealtimeService(T.Toy(), data=d, reset_budget=5, slowdown=1.0)
        svc2.start()
        check(svc2.sim.state_sha() == sha and svc2.loads == 1 and "s1" in svc2.handles and not svc2.running,
              f"2.3 resume after ops: loads {svc2.loads}")
        job = R._Job("POST", "/load_state")
        try:
            svc2._rt_load({"handle": "nope", "cameras": []}, job)
            check(False, "2.3: an unknown handle loaded")
        except R.Refused as exc:
            check(exc.code == 409, f"2.3 unknown handle: {exc.code}")
    finally:
        shutil.rmtree(d)
    if fails:
        print(f"{len(fails)} failure(s)"); return 1
    print("ok 2.3 state ops: save while running, load pauses the clock, resume replays the ops, unknown handle 409")
    return 0


if __name__ == "__main__":
    sys.exit(main())
