#!/usr/bin/env python3
"""CPU-only self-test of the shared 2.2 layer's ablation switches on the toy adapter (toy_server.py under run.sh, as
selftest.py runs it): RCB_STATE_QUERY (GET /state/entities, POST /state/query, state_queries) and RCB_STATE_OPS (/reset,
/save_state, /load_state, their counters, refusals, crash-resume replay and final.json fields). With both off the
service answers exactly as before: the new routes are 404 and no new key appears. Needs numpy + requests.

    python3 embodied_first_exam/standard/ablation_selftest.py
"""
from __future__ import annotations
import json, os, sys, time, traceback

import numpy as np
import requests

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import selftest as T  # noqa: E402  (its Compose, chunk, expect)

expect, chunk = T.expect, T.chunk
CASES = []


def case(fn):
    CASES.append(fn); return fn


def post(c, path, body=None):
    return requests.post(c.url + path, json=body if body is not None else {}, timeout=60)


@case
def switches_off_change_nothing():
    c = T.Compose("abl_off")
    try:
        r = c.robot()
        spec = r.spec()
        expect("state_query" not in spec and "state_ops" not in spec and spec["budget"]["reset"] == 0, f"spec {sorted(spec)}")
        res = r.submit("joint_position", chunk(3, 0.2), dt=0.05, video="none")
        expect("objects" not in res.meta and "saves" not in res.meta and "episode" not in res.meta, f"meta {sorted(res.meta)}")
        for p in ("/reset", "/save_state", "/load_state", "/state/query"):
            expect(post(c, p).status_code == 404, f"{p} without the switch")
        expect(requests.get(c.url + "/state/entities", timeout=60).status_code == 404, "entities without the switch")
        expect(c.end() == 2, "verdict")
        fin = c.final()
        expect(not ({"saves", "loads", "episodes_log", "resets_at_success", "state_queries"} & set(fin)),
               f"final {sorted(fin)}")
    finally:
        c.down()


@case
def state_query_reads_without_stepping():
    c = T.Compose("abl_query", RCB_STATE_QUERY="1")
    try:
        r = c.robot()
        expect("state_query" in r.spec() and "objects" not in r.get_obs(video="none").meta, "spec note, no pushed state")
        ents = requests.get(c.url + "/state/entities", timeout=60).json()
        expect(ents["quaternion_order"] == "wxyz" and ents["entities"]["block"]["kind"] == "object"
               and ents["entities"]["block"]["fields"]["pos"] == "m" and ents["entities"]["arm"]["kind"] == "robot",
               f"entities {ents}")
        q = post(c, "/state/query", {"names": ["block"], "fields": ["pos"]}).json()
        expect(q["state"] == {"block": {"pos": [0.5, 0.0, 0.0]}} and q["control_steps"] == 0 and q["state_queries"] == 1,
               f"query {q}")
        res = r.submit("joint_position", chunk(30, 0.6), dt=0.05, video="none")   # the tip reaches the block, pushes it
        q = post(c, "/state/query", {"names": ["block", "arm"]}).json()
        expect(q["state"]["block"]["pos"][0] > 0.5 and set(q["state"]["arm"]) == {"joint_pos", "tip_pos"}
               and q["state_queries"] == 2 and q["control_steps"] == int(res.meta["executed_rows"]),
               f"the state follows the block {q}")
        both = post(c, "/state/query", {"names": ["block", "arm"], "fields": ["quat"]}).json()
        expect(both["state"] == {"block": {"quat": [1.0, 0.0, 0.0, 0.0]}, "arm": {}} and both["control_steps"] == q["control_steps"],
               f"a field an entity lacks is left out, no step taken {both}")
        bad = post(c, "/state/query", {"names": ["blok"]})
        expect(bad.status_code == 400 and bad.json()["unknown"] == ["blok"] and "block" in bad.json()["names"], bad.text)
        bad = post(c, "/state/query", {"names": ["block"], "fields": ["velocity"]})
        expect(bad.status_code == 400 and bad.json()["fields"]["block"] == ["pos", "quat"], bad.text)
        for body in ({}, {"names": []}, {"names": ["block"], "field": ["pos"]}, {"names": "block"}):
            expect(post(c, "/state/query", body).status_code == 400, f"malformed {body}")
        expect(post(c, "/reset").status_code == 404, "no state ops without RCB_STATE_OPS")
        c.kill_service()                                   # the count survives a crash (each query is logged)
        r.wait_ready(60)
        expect(post(c, "/state/query", {"names": ["block"]}).json()["state_queries"] == 4, "counted across a resume")
        r.submit("joint_position", chunk(40, 0.95), dt=0.05, video="none")
        expect(r.status()["done"], "the episode ended")
        late = post(c, "/state/query", {"names": ["block"]})
        expect(late.status_code == 409, f"after the end {late.text}")
        expect(c.end() == 2, "verdict")
        fin = c.final()
        expect(fin["state_queries"] == 4, f"final {fin.get('state_queries')}")
        ev = [e for e in c.events() if e["ev"] == "state_query"]
        expect([e["n"] for e in ev] == [1, 2, 3, 4] and ev[0]["names"] == ["block"], f"logged {ev}")
    finally:
        c.down()


@case
def reset_save_load_and_their_rules():
    c = T.Compose("abl_ops", RCB_STATE_OPS="1", RCB_RESET_BUDGET="3", RCB_STATE_QUERY="1")
    try:
        r = c.robot()
        spec = r.spec()
        expect(spec["budget"]["reset"] == 3 and "reset" in spec and "state_ops" in spec, "spec")
        r.submit("joint_position", chunk(5, 0.3), dt=0.05, video="none")
        s1 = post(c, "/save_state").json()
        expect(s1["handle"] == "s1" and s1["step"] == 5 and s1["saves"] == 1 and s1["episode"] == 1, f"save {s1}")
        st_saved = r.status()
        r.submit("joint_position", chunk(5, -0.5), dt=0.05, video="none")
        bad = post(c, "/load_state", {"handle": "s9"})
        expect(bad.status_code == 409 and "unknown handle" in bad.json()["error"], f"unknown handle {bad.text}")
        expect(post(c, "/load_state", {}).status_code == 400, "load without a handle")
        ld = post(c, "/load_state", {"handle": "s1", "cameras": []})
        expect(ld.status_code == 200, f"load {ld.text}")
        m = T.rcb_client.rcb1.decode(ld.content)[0]
        expect(m["episode"] == 2 and m["loads"] == 1, f"load answer {m}")
        q = post(c, "/state/query", {"names": ["block"], "fields": ["pos"]}).json()
        expect(q["state"]["block"]["pos"] == [0.5, 0.0, 0.0] and q["control_steps"] == 0, f"the loaded state {q}")
        st = r.status()
        expect(st["episode"] == 2 and st["loads"] == 1 and st["resets_left"] == 3 and not st["done"], f"after load {st}")
        expect(st["control_steps_total"] == 10, f"journal keeps every row {st}")
        rs = post(c, "/reset", {"cameras": []})
        expect(rs.status_code == 200 and r.status()["episode"] == 3 and r.status()["resets_left"] == 2, "reset")
        # a crash after the ops: the resume replays rows, save, load and reset at their journal rows
        r.submit("joint_position", chunk(4, 0.2), dt=0.05, video="none")
        st0 = r.status()
        c.kill_service()
        st1 = r.wait_ready(60)
        expect((st1["episode"], st1["loads"], st1["saves"], st1["resets_left"], st1["control_steps_total"])
               == (st0["episode"], st0["loads"], st0["saves"], st0["resets_left"], st0["control_steps_total"]), f"resumed {st1} vs {st0}")
        ev = c.events(); res_ev = [e for e in ev if e["ev"] == "resumed"]
        expect(res_ev and res_ev[-1]["exact"] is True, f"replayed state matches the checkpoint {res_ev}")
        ld2 = post(c, "/load_state", {"handle": "s1", "cameras": []})       # the handle survives the crash (replayed save)
        expect(ld2.status_code == 200, f"load after resume {ld2.text}")
        # the loaded episode starts where s1 was saved: its graded path is s1's rows + the new ones
        res = r.submit("joint_position", chunk(40, 0.95), dt=0.05, video="none")
        expect(res.meta["done"] and res.meta["reason"] == "success", f"success {res.meta.get('reason')}")
        for p, b in (("/reset", {}), ("/save_state", {}), ("/load_state", {"handle": "s1"})):
            expect(post(c, p, b).status_code == 409, f"{p} after a success")
        expect(c.end() == 2, "verdict")
        fin = c.final()
        expect(fin["success"] and fin["resets_at_success"] == 1 and fin["loads_at_success"] == 2 and fin["saves"] == 1
               and fin["loads"] == 2 and fin["resets"] == 1 and fin["episodes"] == 4, f"final counts {fin}")
        log = fin["episodes_log"]
        expect([(e["episode"], e["began"], e["ended"], e["control_steps"]) for e in log]
               == [(1, "start", "load", 10), (2, "load", "reset", 0), (3, "reset", "load", 4),
                   (4, "load", "success", int(res.meta["executed_rows"]))], f"episodes_log {log}")
        traj = np.load(os.path.join(c.data, "trajectory.npy"))
        expect(len(traj) == 5 + int(res.meta["executed_rows"]), f"graded path = s1's 5 rows + the last episode's ({len(traj)})")
    finally:
        c.down()


@case
def handles_are_capped_lru():
    c = T.Compose("abl_lru", RCB_STATE_OPS="1", RCB_RESET_BUDGET="1", RCB_MAX_STATE_HANDLES="2")
    try:
        r = c.robot()
        r.submit("joint_position", chunk(1, 0.1), dt=0.05, video="none")
        hs = [post(c, "/save_state").json()["handle"] for _ in range(3)]
        expect(hs == ["s1", "s2", "s3"], f"handles {hs}")
        expect(post(c, "/load_state", {"handle": "s1"}).status_code == 409, "the oldest is dropped")
        expect(post(c, "/load_state", {"handle": "s2"}).status_code == 200, "a kept one loads")
        expect(post(c, "/reset").status_code == 200 and post(c, "/reset").status_code == 409, "the reset budget binds")
        r.submit("joint_position", chunk(2, -0.3), dt=0.05, video="none")
        expect(post(c, "/load_state", {"handle": "s3"}).status_code == 200, "a load after the budget is spent (loads are free)")
    finally:
        c.down()


def main() -> int:
    """`--layer 2.4`: the same cases on the 2.4 layer (the toy adapter imports v2_4/rcb_limited.py)."""
    want = sys.argv[1:]
    if "--layer" in want:
        i = want.index("--layer"); os.environ["RCB_TOY_LAYER"] = want[i + 1]; del want[i:i + 2]
    ok = 0
    for fn in CASES:
        if want and not any(w in fn.__name__ for w in want):
            continue
        t = time.time()
        try:
            fn(); ok += 1; print(f"ok   {fn.__name__} ({time.time() - t:.1f} s)")
        except Exception:  # noqa: BLE001
            print(f"FAIL {fn.__name__}"); traceback.print_exc()
    n = len([f for f in CASES if not want or any(w in f.__name__ for w in want)])
    print(f"{ok}/{n} passed; records in {T.ROOT}")
    return 0 if ok == n else 1


if __name__ == "__main__":
    sys.exit(main())
