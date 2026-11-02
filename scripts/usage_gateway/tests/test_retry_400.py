"""Tests for the gateway's one resend of an HTTP 400 (gateway.py module doc "One resend", RETRY_400).

    python3 scripts/usage_gateway/tests/test_retry_400.py     # stdlib only; starts gateway.py on loopback
"""
from __future__ import annotations

import http.client
import json
import pathlib
import subprocess
import sys
import tempfile
import threading
import time
import types
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import gateway  # noqa: E402

OK = json.dumps({"type": "response.completed", "response": {"model": "stub", "usage": {"input_tokens": 3,
                 "output_tokens": 2}}}).encode()


class Flaky(BaseHTTPRequestHandler):
    """Answers the n-th request (counted per path) by the plan for that path: "400" ({"detail": "Bad Request"}),
    "400b" (another 400 body), "drop" (closes the connection without an answer) or "200"."""
    plan: dict[str, list[str]] = {}
    seen: list[tuple[str, bytes, dict]] = []
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):
        pass

    def do_POST(self):
        body = self.rfile.read(int(self.headers.get("Content-Length") or 0))
        hdrs = {k.lower(): v for k, v in self.headers.items() if k.lower() not in ("connection",)}
        Flaky.seen.append((self.path, body, hdrs))
        n = sum(1 for p, _, _ in Flaky.seen if p == self.path) - 1
        steps = Flaky.plan.get(self.path, ["200"])
        what = steps[min(n, len(steps) - 1)]
        if what == "drop":
            self.close_connection = True
            return
        out = {"400": b'{"detail":"Bad Request"}', "400b": b'{"detail":"Bad Request","n":2}'}.get(what, OK)
        self.send_response(400 if what.startswith("400") else 200)
        self.send_header("Content-Type", "application/json"); self.send_header("X-Upstream-Try", str(n + 1))
        self.send_header("Content-Length", str(len(out))); self.end_headers(); self.wfile.write(out)


class Resend(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stub = ThreadingHTTPServer(("127.0.0.1", 0), Flaky)
        threading.Thread(target=cls.stub.serve_forever, daemon=True).start()
        cls.up = f"127.0.0.1:{cls.stub.server_address[1]}"

    @classmethod
    def tearDownClass(cls):
        cls.stub.shutdown(); cls.stub.server_close()

    def gateway(self, *extra):
        td = tempfile.TemporaryDirectory()
        self.addCleanup(td.cleanup)
        jobs = pathlib.Path(td.name)
        ready = jobs / "ready"
        proc = subprocess.Popen([sys.executable, str(HERE / "gateway.py"), "--jobs", str(jobs), "--bind", "127.0.0.1",
                                 "--ready-file", str(ready), "--http-host", self.up, *extra],
                                stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
        self.addCleanup(lambda: (proc.terminate(), proc.wait(10), proc.stderr.close()))
        for _ in range(100):
            if ready.exists() and ready.read_text():
                break
            if proc.poll() is not None:
                raise RuntimeError(proc.stderr.read().decode()[-2000:])
            time.sleep(0.1)
        return jobs, int(ready.read_text())

    @staticmethod
    def call(port, path, body=b'{"model": "m", "input": [{"role": "user", "content": "hi"}], "x": 1.846597}'):
        c = http.client.HTTPConnection("127.0.0.1", port, timeout=30)
        c.request("POST", path, body=body, headers={"Content-Type": "application/json", "Authorization": "Bearer K",
                                                    "Session-Id": "s-1"})
        r = c.getresponse(); out = r.read(); hdr = r.getheader("X-Upstream-Try"); c.close()
        return r.status, out, hdr

    @staticmethod
    def records(jobs):
        f = jobs / ".gateway" / "unattributed.jsonl"
        return [json.loads(x) for x in f.read_text().splitlines()] if f.exists() else []

    def setUp(self):
        Flaky.seen.clear(); Flaky.plan.clear()

    def test_a_400_goes_once_more_unchanged_and_the_second_answer_reaches_the_agent(self):
        jobs, port = self.gateway("--retry-400", f"{self.up}/v1/responses")
        Flaky.plan["/v1/responses"] = ["400", "200"]
        st, out, tr = self.call(port, f"/h/{self.up}/v1/responses")
        self.assertEqual((st, tr), (200, "2"))
        self.assertEqual(json.loads(out)["type"], "response.completed")
        self.assertEqual(len(Flaky.seen), 2)
        self.assertEqual(Flaky.seen[0][1], Flaky.seen[1][1])         # the same bytes
        self.assertEqual(Flaky.seen[0][2], Flaky.seen[1][2])         # the same headers
        recs = self.records(jobs)
        self.assertEqual([(r["status"], r.get("retried"), r.get("attempt")) for r in recs], [(400, True, 1), (200, None, 2)])
        self.assertEqual(recs[1]["retry_of_status"], 400)
        self.assertEqual(recs[0]["response_bytes"], 24)
        self.assertEqual(recs[1]["usage"], {"input": 3, "cached": 0, "output": 2})
        self.assertLessEqual(recs[0]["ts"], recs[1]["ts"])

    def test_a_second_400_is_passed_on_unchanged_and_there_is_no_third_try(self):
        jobs, port = self.gateway("--retry-400", f"{self.up}/v1/responses")
        Flaky.plan["/v1/responses"] = ["400", "400b", "200"]
        st, out, tr = self.call(port, f"/h/{self.up}/v1/responses")
        self.assertEqual((st, out, tr), (400, b'{"detail":"Bad Request","n":2}', "2"))
        self.assertEqual(len(Flaky.seen), 2)
        recs = self.records(jobs)
        self.assertEqual([(r["status"], r.get("attempt")) for r in recs], [(400, 1), (400, 2)])
        self.assertNotIn("retried", recs[1])

    def test_a_resend_that_gets_no_answer_passes_the_first_400_on(self):
        jobs, port = self.gateway("--retry-400", f"{self.up}/v1/responses")
        Flaky.plan["/v1/responses"] = ["400", "drop"]
        st, out, tr = self.call(port, f"/h/{self.up}/v1/responses")
        self.assertEqual((st, out, tr), (400, b'{"detail":"Bad Request"}', "1"))
        recs = self.records(jobs)
        self.assertEqual([(r["status"], r.get("attempt")) for r in recs], [(400, 1), (502, 2)])
        self.assertIs(recs[1]["forwarded"], False)
        self.assertEqual(recs[1]["passed_on"], "attempt 1")

    def test_other_paths_and_statuses_are_not_resent(self):
        jobs, port = self.gateway("--retry-400", f"{self.up}/v1/responses")
        Flaky.plan["/v1/messages"] = ["400", "200"]
        self.assertEqual(self.call(port, f"/h/{self.up}/v1/messages")[0], 400)        # not listed
        self.assertEqual(len(Flaky.seen), 1)
        self.assertEqual(self.call(port, f"/h/{self.up}/v1/responses")[0], 200)      # a 200 goes once
        self.assertEqual(len(Flaky.seen), 2)
        self.assertEqual([r.get("attempt") for r in self.records(jobs)], [None, None])

    def test_no_retry_400(self):
        jobs, port = self.gateway("--retry-400", f"{self.up}/v1/responses", "--no-retry-400")
        Flaky.plan["/v1/responses"] = ["400", "200"]
        self.assertEqual(self.call(port, f"/h/{self.up}/v1/responses")[0], 400)
        self.assertEqual(len(Flaky.seen), 1)

    def test_the_default_is_codex_responses_at_chatgpt_only(self):
        self.assertEqual(gateway.RETRY_400, {("chatgpt.com", "/backend-api/codex/responses")})


class Tally(unittest.TestCase):
    def test_retried_400_is_counted(self):
        saved = getattr(gateway, "ARGS", None)
        gateway.ARGS = types.SimpleNamespace(budget_input=0, budget_output=0)
        try:
            tot = {"requests": 0, "input": 0, "cached": 0, "output": 0, "no_usage": 0}
            gateway.tally(tot, {"ts": 1, "status": 400, "usage": {}, "retried": True, "attempt": 1})
            gateway.tally(tot, {"ts": 2, "status": 200, "usage": {"input": 5, "cached": 0, "output": 1}, "attempt": 2})
            self.assertEqual((tot["requests"], tot["retried_400"], tot["input"], tot["failures_in_a_row"]), (2, 1, 5, 0))
        finally:
            gateway.ARGS = saved


if __name__ == "__main__":
    unittest.main()
