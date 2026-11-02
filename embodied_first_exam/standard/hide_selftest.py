#!/usr/bin/env python3
"""The no-success-signal ablation of the shared eai-standard/2.2 and 2.4 layers (RCB_HIDE_SUCCESS; docs/protocol/
standard_mode.md "No success signal"), over HTTP, on toy_server.py (numpy only, CPU, about 20 s):

  - switch off: a fixed request script gets the same answers as from the code given with `--against DIR` (a directory
    holding an older toy_server.py, rcb_limited.py and v2_4/rcb_limited.py), and /finish is a 404;
  - switch on: a success changes no answer (done, reason, executed rows, 409s); POST /finish ends the episode and every
    call after it but GET /spec and /status is a 409 (2.4: GET /operations too); the verdict is the end state's (a
    success undone is 0; a success at /finish or at the agent's end is 1); final.json records the ablation;
  - switch on with an adapter that has no finish(): the service refuses to start (fail closed).
"""
from __future__ import annotations

import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import rcb1  # noqa: E402

VOLATILE = {"time_s", "time_left_s", "elapsed_s"}
fails: list[str] = []


def check(cond, what):
    if not cond:
        fails.append(what)
        print("FAIL", what, flush=True)


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


class Toy:
    """toy_server.py from `src` (with the layer `layer`) in a directory of its own, on a free port."""

    def __init__(self, src: str, layer: str, hide: bool | None):
        self.dir = tempfile.mkdtemp()
        for f, frm in (("toy_server.py", os.path.join(src, "toy_server.py")), ("rcb_limited.py", layer),
                       ("rcb1.py", os.path.join(HERE, "rcb1.py"))):
            shutil.copy(frm, os.path.join(self.dir, f))
        lay = os.path.join(self.dir, "rcb_limited.py")
        if "def agent_time" not in open(lay).read():      # the 2.4 layer: toy_server.py's clock helper from 2.2's
            with open(lay, "a") as fh:
                fh.write("\n\ndef agent_time(state):\n    t = state.get('t_first')\n"
                         "    return 0.0 if t is None else max(0.0, time.time() - float(t))\n")
        self.data = os.path.join(self.dir, "data")
        self.port = free_port()
        env = {**os.environ, "RCB_DATA": self.data, "RCB_TOY_PORT": str(self.port)}
        env.pop("RCB_HIDE_SUCCESS", None)
        if hide is not None:
            env["RCB_HIDE_SUCCESS"] = "1" if hide else "0"
        self.log = open(os.path.join(self.dir, "server.log"), "w")
        self.p = subprocess.Popen([sys.executable, os.path.join(self.dir, "toy_server.py")], env=env, stdout=self.log,
                                  stderr=subprocess.STDOUT)
        t = time.time()
        while time.time() - t < 30:
            if self.p.poll() is not None:
                return
            try:
                socket.create_connection(("127.0.0.1", self.port), timeout=0.5).close()
                return
            except OSError:
                time.sleep(0.1)

    def call(self, method: str, path: str, body: dict | None = None) -> dict:
        req = urllib.request.Request(f"http://127.0.0.1:{self.port}{path}", method=method,
                                     data=None if body is None else json.dumps(body).encode(),
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=60) as r:
                code, ctype, raw = r.status, r.headers.get("Content-Type"), r.read()
        except urllib.error.HTTPError as e:
            code, ctype, raw = e.code, e.headers.get("Content-Type"), e.read()
        if ctype == "application/x-rcb1":
            meta, parts = rcb1.decode(raw)
            return {"code": code, "meta": {k: v for k, v in meta.items() if k not in VOLATILE}, "parts": sorted(parts)}
        obj = json.loads(raw or b"{}")
        return {"code": code, "json": {k: v for k, v in obj.items() if k not in VOLATILE} if isinstance(obj, dict) else obj}

    def end(self) -> dict:
        fin = {}
        if self.p.poll() is None:
            self.p.send_signal(signal.SIGUSR1)
            t = time.time()
            while time.time() - t < 30:
                try:
                    fin = json.load(open(os.path.join(self.data, "final.json")))
                    if fin.get("finalized"):
                        break
                except (OSError, ValueError):
                    pass
                time.sleep(0.1)
            self.p.kill()
        self.p.wait()
        shutil.rmtree(self.dir, ignore_errors=True)
        return fin


def sub(rows):
    return {"type": "joint_position", "actions": rows, "video": "none"}


TO_BLOCK, PUSH, BACK = [[0.5, 0, 0]] * 12, [[1.0, 0, 0]] * 12, [[0.5, 0, 0]] * 12
SCRIPT = [("GET", "/spec", None), ("GET", "/status", None), ("POST", "/observe", {}), ("POST", "/submit", sub(TO_BLOCK)),
          ("POST", "/finish", {}), ("POST", "/submit", sub(PUSH)), ("GET", "/status", None), ("POST", "/observe", {}),
          ("POST", "/submit", sub(BACK)), ("POST", "/kinematics", {"op": "fk", "q": [0, 0, 0]}), ("POST", "/finish", {}),
          ("POST", "/submit", sub(BACK)), ("POST", "/observe", {}), ("POST", "/kinematics", {"op": "fk", "q": [0, 0, 0]}),
          ("GET", "/status", None)]


def script(src, layer, hide):
    t = Toy(src, layer, hide)
    out = [t.call(*c) for c in SCRIPT]
    return out, t.end()


def layer_test(tag: str, layer: str, against: str | None, against_layer: str | None) -> None:
    off, fin_off = script(HERE, layer, None)
    check(off[4]["code"] == 404 and off[10]["code"] == 404, f"{tag}: switch off: /finish is not a 404")
    check(off[5]["meta"]["reason"] == "success" and off[5]["meta"]["executed_rows"] < 12 and off[8]["code"] == 409,
          f"{tag}: switch off: a success does not end the episode: {off[5]} {off[8]}")
    check("success_signal" not in off[0]["json"] and "success_signal" not in fin_off, f"{tag}: switch off: ablation keys")
    off0, _ = script(HERE, layer, False)
    check(off0 == off, f"{tag}: RCB_HIDE_SUCCESS=0 differs from no switch")
    if against:
        old, fin_old = script(against, against_layer, None)
        check(old == off, f"{tag}: switch off: answers differ from {against}: "
              + str([(i, a, b) for i, (a, b) in enumerate(zip(old, off)) if a != b][:2]))
        keep = lambda f: {k: v for k, v in f.items() if k not in VOLATILE | {"resume_s", "t_end"}}  # noqa: E731
        check(keep(fin_old) == keep(fin_off), f"{tag}: switch off: final.json differs")

    on, fin = script(HERE, layer, True)
    spec, st0, obs0, s1, f1, s2, st1, obs1, s3, k1, f2, s4, obs2, k2, st2 = on
    check("success_signal" in spec["json"], f"{tag}: hide: /spec has no success_signal")
    check(f1["code"] == 200 and f1["json"]["done"] and f1["json"]["reason"] == "finished",
          f"{tag}: hide: the first /finish: {f1}")
    check(s2["code"] == 409 and obs1["code"] == 409 and k1["code"] == 409 and f2["code"] == 409 and st1["code"] == 200,
          f"{tag}: hide: calls after /finish: {s2['code']} {obs1['code']} {k1['code']} {f2['code']} {st1['code']}")
    check(fin.get("success") is False and fin.get("ended_by") == "finished" and fin.get("success_signal") == "hidden"
          and fin.get("hidden_success_seen") is False, f"{tag}: hide: finish before the block moved: {fin}")

    # a success mid-chunk, then undone, then /finish: nothing shown, graded 0
    t = Toy(HERE, layer, True)
    a = [t.call("POST", "/submit", sub(TO_BLOCK)), t.call("POST", "/submit", sub(PUSH))]
    st = t.call("GET", "/status")
    check(all(x["meta"]["done"] is False and x["meta"]["reason"] is None and x["meta"]["executed_rows"] == 12 for x in a)
          and st["json"]["done"] is False and st["json"]["reason"] is None,
          f"{tag}: hide: a success shows: {[x['meta'] for x in a]} {st}")
    b = t.call("POST", "/submit", sub(BACK))
    check(b["code"] == 200 and b["meta"]["executed_rows"] == 12, f"{tag}: hide: a call after a hidden success: {b}")
    f = t.call("POST", "/finish", {})
    fin = t.end()
    check(f["code"] == 200 and fin.get("success") is False and fin.get("hidden_success_seen") is True
          and fin.get("ended_by") == "finished", f"{tag}: hide: a success undone was graded: {fin}")

    # success held to /finish: 1; success held to the agent's end (SIGUSR1): 1
    for how in ("finish", "timeout"):
        t = Toy(HERE, layer, True)
        t.call("POST", "/submit", sub(TO_BLOCK)); t.call("POST", "/submit", sub(PUSH))
        if how == "finish":
            t.call("POST", "/finish", {})
        fin = t.end()
        check(fin.get("success") is True and fin.get("ended_by") == ("finished" if how == "finish" else "timeout"),
              f"{tag}: hide: a success held to the {how}: {fin}")

    # fail closed: an adapter without finish() does not start with the switch on
    old_src = tempfile.mkdtemp()
    s = open(os.path.join(HERE, "toy_server.py")).read().replace(', finish=lambda: finish("finished")', "")
    open(os.path.join(old_src, "toy_server.py"), "w").write(s)
    t = Toy(old_src, layer, True)
    time.sleep(1.0)
    check(t.p.poll() is not None and t.p.returncode != 0, f"{tag}: hide: an adapter without finish() served")
    t.end()
    shutil.rmtree(old_src, ignore_errors=True)


def main() -> None:
    against = sys.argv[sys.argv.index("--against") + 1] if "--against" in sys.argv else None
    layer_test("2.2", os.path.join(HERE, "rcb_limited.py"), against,
               against and os.path.join(against, "rcb_limited.py"))
    layer_test("2.4", os.path.join(HERE, "v2_4", "rcb_limited.py"), against,
               against and os.path.join(against, "v2_4", "rcb_limited.py"))
    if fails:
        print(f"{len(fails)} failure(s)")
        sys.exit(1)
    print("ok no-success-signal switch (2.2, 2.4 layers on toy_server.py): off = unchanged, on = no success leak, "
          "/finish, end-state verdict, fail closed")


if __name__ == "__main__":
    main()
