#!/usr/bin/env python3
"""What the resource monitor can measure where it runs (capabilities(), written into its record), and a self-test of
the shared violation rule and the verifier step (no containers needed; ci_checks/check-monitor.py runs it):

    python3 images/monitor/rcb_monitor/selftest.py
"""
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))


def _rl(p):
    try:
        return os.readlink(p)
    except OSError:
        return None


def capabilities():
    """Facts the verifier and the results footnotes need: does the monitor see the host's processes (pid: host), the
    host's cgroup tree (cgroup: host; per-container memory and OOM counts), """
    c = {}
    try:
        pids = [d for d in os.listdir("/proc") if d.isdigit()]
    except OSError:
        pids = []
    try:
        c["pid_host"] = int(len(pids) > 1 and open("/proc/1/comm").read().strip() != open("/proc/self/comm").read().strip())
    except OSError:
        c["pid_host"] = 0
    own = ""
    try:
        own = [ln[3:].strip() for ln in open("/proc/self/cgroup") if ln.startswith("0::")][0]
    except (OSError, IndexError):
        pass
    c["cgroup_v2"] = int(os.path.exists("/sys/fs/cgroup/cgroup.controllers"))
    c["cgroup_host"] = int(bool(own) and own != "/" and os.path.isdir("/sys/fs/cgroup" + own))
    return c


# --------------------------------------------------------------------------------------------------- self-test
# limits.json exactly as inject.limits_json() writes it (test_monitor.py checks that it is that function's output for
# these caps, so the self-test never runs on a shape inject.py does not write): a privileged task whose main holds a GPU
# (TASK_COMPOSE_GPU, TASK_CFG), and the same task without the GPU
TASK_CFG = {"environment": {"cpus": 4, "memory_mb": 8192, "storage_mb": 10240}, "agent": {"timeout_sec": 3600}}
TASK_COMPOSE_GPU = ("services:\n  main:\n    pids_limit: 100\n    x-gpu-memory-gb: 4\n    deploy:\n      resources:\n"
                    "        reservations:\n          devices:\n            - driver: nvidia\n              count: 1\n"
                    "              capabilities: [gpu]\n")
TASK_COMPOSE_NOGPU = "services:\n  main:\n    pids_limit: 100\n"
LIMITS_GPU = {"format": "rcb-limits/2", "task": "selftest-gpu-i00-privileged", "mode": "privileged",
              "caps": {"main": {"cpus": 4.0, "memory_gb": 8.0, "pids": 100, "gpu_memory_gb": 4.0, "disk_gb": 10.0}},
              "scope_services": ["main"], "gpu_memory_gb": 4.0, "disk_gb": 10.0, "tick_sec": 5, "disk_tick_sec": 60,
              "gpu": True, "agent_timeout_sec": 3600}
LIMITS_NOGPU = {"format": "rcb-limits/2", "task": "selftest-nogpu-i00-privileged", "mode": "privileged",
                "caps": {"main": {"cpus": 4.0, "memory_gb": 8.0, "pids": 100, "disk_gb": 10.0}},
                "scope_services": ["main"], "disk_gb": 10.0, "tick_sec": 5, "disk_tick_sec": 60, "gpu": False,
                "agent_timeout_sec": 3600}


def _check(cond, msg):
    if not cond:
        raise AssertionError(msg)


def test_rule():
    sys.path.insert(0, HERE)
    import rcb_limits as r
    lim = LIMITS_GPU

    def sample(t, **c):
        base = {"kind": "scope", "role": "main", "service": "main", "cpu": 1.0, "memory_gb": 1.0, "pids": 10,
                "gpu_memory_gb": 0.5}
        return {"t": t, "scope": {"disk_gb": c.pop("disk_gb", None)}, "containers": [dict(base, **c)]}
    _check(r.violations([sample(0)], lim) == [], "within the caps")
    _check(r.violations([sample(0, memory_gb=8.3)], lim, tol=0.05) == [], "within the tolerance: not over")
    v = r.violations([sample(0, memory_gb=8.5), sample(5, memory_gb=9.0)], lim, tol=0.05)
    _check([x["metric"] for x in v] == ["memory_gb"] and v[0]["at"] == 0 and v[0]["peak"] == 9.0 and v[0]["samples"] == 2,
           f"memory over its cap at one sample is over: {v}")
    _check([x["metric"] for x in r.violations([sample(0, pids=101), sample(5, pids=101)], lim, tol=0.5)] == ["pids"],
           "counts are exact")
    _check(r.violations([sample(0, pids=101), sample(5, pids=99), sample(10, pids=101)], lim) == [],
           "a count over its cap in one sample alone is not over (rcb_limits.PIDS_RUN)")
    _check([x["metric"] for x in r.violations([sample(0, cpu=4.5)], lim)] == ["cpus"], "CPU over its cap")
    _check([x["metric"] for x in r.violations([sample(0, gpu_memory_gb=4.5)], lim)] == ["gpu_memory_gb"], "GPU memory")
    _check([x["metric"] for x in r.violations([sample(0, disk_gb=11.0)], lim)] == ["disk_gb"], "disk")
    _check(r.violations([sample(0, gpu_memory_gb=9.0)], LIMITS_NOGPU) == [],
           "no GPU in the service: GPU memory is recorded only")
    side = {"t": 0, "scope": {}, "containers": [{"kind": "observe", "role": "sidecar", "memory_gb": 99.0, "pids": 10 ** 6}]}
    _check(r.violations([side], lim) == [], "a benchmark-owned service is recorded only")
    _check(r.parse_tolerance("0.1") == (0.1, None) and r.parse_tolerance(None)[0] == r.TOLERANCE
           and r.parse_tolerance("x")[1] and r.parse_tolerance("2")[1], "the tolerance switch")


def test_verify():
    sys.path.insert(0, HERE)
    import rcb_limits as r
    lim = LIMITS_GPU
    for enforce, gpu, want in ((1, 6.0, "over"), (0, 6.0, "over"), (1, 3.0, "pass")):
        with tempfile.TemporaryDirectory() as d:
            rec, tests, logs = (os.path.join(d, x) for x in ("rec", "tests", "logs"))
            for x in (rec, tests, logs):
                os.makedirs(x)
            json.dump(lim, open(os.path.join(tests, "rcb_limits.json"), "w"))
            with open(os.path.join(rec, "samples.jsonl"), "w") as f:
                for i in range(40):
                    f.write(json.dumps({"t": 1000 + 5 * i, "scope": {"gpu_memory_gb": gpu if i > 5 else 0.5,
                                                                    "memory_gb": 2.0, "processes": 10,
                                                                    "disk_gb": 0.1 if i % 12 == 0 else None}}) + "\n")
            open(os.path.join(rec, "events.jsonl"), "w").close()
            json.dump({"format": r.FORMAT, "sealed": True, "enforce": enforce, "scope_seen": True},
                      open(os.path.join(rec, "summary.json"), "w"))
            json.dump({"success": 1, "infra_error": 0}, open(os.path.join(logs, "reward.json"), "w"))
            open(os.path.join(logs, "reward.txt"), "w").write("1\n")
            import shutil
            shutil.copy(os.path.join(HERE, "rcb_limits.py"), tests)
            env = dict(os.environ, RCB_MONITOR_RECORD=rec, RCB_VERIFIER_LOGS=logs)
            import subprocess
            subprocess.run([sys.executable, os.path.join(tests, "rcb_limits.py"), "verify"], env=env, check=True,
                           capture_output=True)
            out = json.load(open(os.path.join(logs, "reward.json")))
            txt = open(os.path.join(logs, "reward.txt")).read().strip()
            if want == "over":
                _check(out["success"] == 1 and out["limits_violation_observed"] == 1 and txt == "1"
                       and out["over_gpu_memory_gb"] == 1 and out["resource_limit_exceeded"] == 0
                       and out.get("infra_error", 0) == 0, f"a cap exceeded is recorded, the grade stands: {out}")
            else:
                _check(out["success"] == 1 and out["limits_violation_observed"] == 0 and out["resource_monitor_missing"] == 0,
                       f"within limits: {out}")
            _check(out["peak_gpu_memory_gb"] == gpu, "peak recorded")
    # no record at all: enforced = infra, observed = flagged only
    for enforce in (1, 0):
        with tempfile.TemporaryDirectory() as d:
            logs, tests = os.path.join(d, "logs"), os.path.join(d, "tests")
            os.makedirs(logs)
            os.makedirs(tests)
            json.dump(lim, open(os.path.join(tests, "rcb_limits.json"), "w"))
            json.dump({"success": 1}, open(os.path.join(logs, "reward.json"), "w"))
            import shutil
            import subprocess
            shutil.copy(os.path.join(HERE, "rcb_limits.py"), tests)
            env = dict(os.environ, RCB_MONITOR_RECORD=os.path.join(d, "none"), RCB_VERIFIER_LOGS=logs,
                       RCB_LIMITS_ENFORCE=str(enforce))
            subprocess.run([sys.executable, os.path.join(tests, "rcb_limits.py"), "verify"], env=env, check=True,
                           capture_output=True)
            out = json.load(open(os.path.join(logs, "reward.json")))
            _check(out["resource_monitor_missing"] == 1 and out.get("infra_error", 0) == enforce,
                   f"missing record (enforce={enforce}): {out}")


def main():
    if sys.argv[1:2] == ["caps"]:
        print(json.dumps(capabilities()))
        return 0
    test_rule()
    test_verify()
    print("rcb_monitor selftest: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
