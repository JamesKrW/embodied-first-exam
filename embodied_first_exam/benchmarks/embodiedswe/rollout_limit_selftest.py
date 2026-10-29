"""Self-test (CPU, no simulator): a solution still being served when the rollout limit passes is a graded timeout.

rcb_embodiedswe.run_solution serves the solution process over two pipes; at the deadline its watchdog kills the
process. A request still being answered then (a long env.step) used to end in BrokenPipeError on the reply, which the
rollout reported as the verifier's error (infra_error). Here a stand-in env whose step outlasts a 3 s limit checks
that run_solution returns solve_status "timeout" instead of raising, and that a solution that returns in time still
reports "returned". Needs Linux with Landlock ABI >= 4 (the solution's sandbox); otherwise it says so and passes.

    python3 embodied_first_exam/benchmarks/embodiedswe/rollout_limit_selftest.py
"""
from __future__ import annotations
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
os.environ.setdefault("ESW_BENCH_ROOT", "/nonexistent")    # no robobench here: the stand-in server needs none
import esw_proxy  # noqa: E402
import rcb_embodiedswe as R  # noqa: E402


class FakeServer:
    """The trusted side's request handler, without robobench: the env has one method, step, which sleeps `step_s`."""

    def __init__(self, env, on_step):
        self.env, self.on_step = env, on_step
        self.n_requests = self.n_refused = 0
        self.refusals: list = []

    def handle_of(self, obj) -> dict:
        return {"__h__": 0, "type": "BaseEnv", "callable": False}

    def handle(self, req: dict):
        self.n_requests += 1
        if req.get("op") == "step":
            return self.on_step(req.get("action"))
        raise R.Refused(f"not in this test: {req.get('op')}")


def run(step_s: float, limit_s: float) -> dict:
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        (d / "sol").mkdir()
        (d / "sol" / "solve.py").write_text("def solve(env):\n    env.step([0.0])\n")
        R.Server = FakeServer
        return R.run_solution(object(), d / "sol", d / "work", lambda a: time.sleep(step_s),
                              time.monotonic() + limit_s, d / "solve.log")


def sandboxed_python() -> str | None:
    """An interpreter the solution's sandbox can read (its stdlib under one of esw_proxy.READ_TREES), or None."""
    for exe in (sys.executable, "/usr/bin/python3"):
        try:
            prefix = subprocess.run([exe, "-I", "-c", "import sys; print(sys.base_prefix)"], capture_output=True,
                                    text=True, timeout=30).stdout.strip()
        except OSError:
            continue
        if prefix and any(prefix == t or prefix.startswith(t + "/") for t in esw_proxy.READ_TREES):
            return exe
    return None


def main() -> int:
    if not sys.platform.startswith("linux") or esw_proxy.landlock_abi() < esw_proxy.LANDLOCK_ABI_MIN:
        print("ok rollout limit: skipped (no Landlock ABI >= %d on this kernel)" % esw_proxy.LANDLOCK_ABI_MIN)
        return 0
    exe = sandboxed_python()
    if exe is None:
        print("ok rollout limit: skipped (no python whose stdlib the solution sandbox can read)")
        return 0
    sys.executable = exe                         # run_solution starts the sandboxed proxy with sys.executable
    t = time.monotonic()
    late = run(step_s=6.0, limit_s=3.0)          # the step outlasts the limit: the solution is killed mid-request
    assert late["solve_status"] == "timeout", late
    assert late.get("solve_error") is None, late
    assert time.monotonic() - t < 30, "run_solution did not return promptly"
    ok = run(step_s=0.1, limit_s=60.0)           # in time: returned
    assert ok["solve_status"] == "returned", ok
    print("ok rollout limit: a request outlasting the limit -> timeout (no broken pipe), in time -> returned")
    return 0


if __name__ == "__main__":
    sys.exit(main())
