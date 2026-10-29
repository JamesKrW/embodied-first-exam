#!/usr/bin/env python3
"""Tests of the resource monitor's own logic (monitor.py) and of the verifier step's edge cases (rcb_limits.py), on a
fake /proc tree and temporary directories: no containers, no GPU, no root. ci_checks/check-monitor.py runs it after
selftest.py. Not copied into tasks.

    python3 images/monitor/rcb_monitor/test_monitor.py
"""
import importlib.util
import itertools
import json
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ.setdefault("RCB_MONITOR_RECORD", tempfile.mkdtemp(prefix="rcb-monitor-test-"))
import inject  # noqa: E402
import monitor  # noqa: E402
import rcb_limits  # noqa: E402
import selftest  # noqa: E402
from pathlib import Path  # noqa: E402

GIB = float(2 ** 30)


def check(cond, msg):
    if not cond:
        raise AssertionError(msg)


# --------------------------------------------------------------------------------------------------- a fake /proc
class Fake:
    """A /proc tree, container roots and a beacon volume under one temporary directory."""

    def __init__(self):
        self.d = tempfile.mkdtemp(prefix="rcb-monitor-fake-")
        self.proc = os.path.join(self.d, "proc")
        self.volume = os.path.join(self.d, "volume")          # the trial's beacon volume
        self.host = os.path.join(self.d, "hostroot")          # the host's root: no beacon
        self.rec = os.path.join(self.d, "rec")
        self.observe = os.path.join(self.d, "volume-observe")  # the beacon of the benchmark's own services (recorded only)
        for x in (self.proc, self.volume, self.host, self.rec, self.observe):
            os.makedirs(x)
        self.proc_self()

    def proc_self(self):
        os.makedirs(os.path.join(self.proc, "self", "ns"))
        os.symlink("mnt:[1]", os.path.join(self.proc, "self", "ns", "mnt"))

    def root(self, name, beacon=True, main=True):
        """A container's root directory: the beacon volume at /run/rcb-limits, /logs/agent in the agent's container."""
        r = os.path.join(self.d, name)
        os.makedirs(os.path.join(r, "run"), exist_ok=True)
        if beacon and not os.path.exists(os.path.join(r, "run", "rcb-limits")):
            os.symlink(self.volume, os.path.join(r, "run", "rcb-limits"))
        if main:
            os.makedirs(os.path.join(r, "logs", "agent"), exist_ok=True)
        return r

    def sidecar_root(self, name):
        """The root of a benchmark-owned service (the sim sidecar of standard modes): the observe beacon, no /logs/agent."""
        r = self.root(name, beacon=False, main=False)
        if not os.path.exists(os.path.join(r, "run", "rcb-observe")):
            os.symlink(self.observe, os.path.join(r, "run", "rcb-observe"))
        return r

    def _entry(self, base, mnt, root, start, state="S", ppid=1, nspid=None, threads=1, rss_kb=1024, pss_kb=None,
               write_bytes=None, cpu_ticks=10, child_ticks=0):
        os.makedirs(os.path.join(base, "ns"), exist_ok=True)
        for link, target in ((os.path.join(base, "ns", "mnt"), mnt), (os.path.join(base, "root"), root)):
            if os.path.lexists(link):
                os.remove(link)
            if target is not None:
                os.symlink(target, link)
        pid = os.path.basename(base)
        fields = [state, str(ppid)] + ["0"] * 9 + [str(cpu_ticks // 2), str(cpu_ticks - cpu_ticks // 2)] \
            + [str(child_ticks // 2), str(child_ticks - child_ticks // 2)] + ["0"] * 4 + [str(start)] + ["0"] * 10
        with open(os.path.join(base, "stat"), "w") as f:
            f.write(f"{pid} (x) " + " ".join(fields) + "\n")
        with open(os.path.join(base, "status"), "w") as f:
            f.write(f"State:\t{state} ({'zombie' if state == 'Z' else 'sleeping'})\nPPid:\t{ppid}\n"
                    f"NSpid:\t{pid} {nspid if nspid is not None else pid}\nThreads:\t{threads}\nVmRSS:\t{rss_kb} kB\n")
        if pss_kb is not None:
            with open(os.path.join(base, "smaps_rollup"), "w") as f:
                f.write(f"Rss:\t{rss_kb} kB\nPss:\t{pss_kb} kB\nPss_Anon:\t{pss_kb} kB\nPss_Shmem:\t0 kB\n")
        if write_bytes is not None:
            with open(os.path.join(base, "io"), "w") as f:
                f.write(f"rchar: 0\nwchar: {write_bytes}\nread_bytes: 0\nwrite_bytes: {write_bytes}\n")

    def pid(self, pid, mnt, root, start=1000, **kw):
        self._entry(os.path.join(self.proc, str(pid)), mnt, root, start, **kw)

    def thread(self, pid, tid, mnt, root, start=1000):
        self._entry(os.path.join(self.proc, str(pid), "task", str(tid)), mnt, root, start)

    def gone(self, pid):
        shutil.rmtree(os.path.join(self.proc, str(pid)))

    def cgroup(self, name, pids=None, anon_gb=None, max_gb=None, **events):
        """A container's own cgroup v2 directory (under the fake host tree) and its processes' /proc/<pid>/cgroup.
        Only what is given is (re)written; events: the counters of memory.events (oom=, oom_kill=, max=)."""
        d = os.path.join(self.d, "cgroup", name)
        os.makedirs(d, exist_ok=True)
        if pids is not None:
            for pid in pids:
                with open(os.path.join(self.proc, str(pid), "cgroup"), "w") as fh:
                    fh.write(f"0::/{name}\n")
            with open(os.path.join(d, "cgroup.procs"), "w") as fh:
                fh.write("".join(f"{pid}\n" for pid in pids))
        if anon_gb is not None:
            with open(os.path.join(d, "memory.stat"), "w") as fh:
                fh.write(f"anon {int(anon_gb * GIB)}\nshmem 0\n")
            with open(os.path.join(d, "memory.current"), "w") as fh:
                fh.write(f"{int(anon_gb * GIB)}\n")
        if max_gb is not None:
            with open(os.path.join(d, "memory.max"), "w") as fh:
                fh.write("max\n" if max_gb == "max" else f"{int(max_gb * GIB)}\n")
        if events:
            ev = dict({"low": 0, "high": 0, "max": 0, "oom": 0, "oom_kill": 0, "oom_group_kill": 0}, **events)
            with open(os.path.join(d, "memory.events"), "r+" if os.path.exists(os.path.join(d, "memory.events")) else "w") as fh:
                fh.truncate(0)                       # in place: the monitor keeps the file open, as on a real cgroup
                fh.write("".join(f"{k} {v}\n" for k, v in ev.items()))
        return d

    def monitor(self, limits=None, enforce="0"):
        monitor.PROC, monitor.REC = self.proc, self.rec
        monitor.CGROUP = os.path.join(self.d, "cgroup")

        class NoNvml:
            def __init__(self):
                raise RuntimeError("no NVML in the test")

        monitor.Nvml = NoNvml
        os.environ["RCB_LIMITS_ENFORCE"] = enforce
        m = monitor.Monitor()
        m.lim = dict(limits or {})
        m.beacon = {"scope": monitor.stat_id(self.volume), "observe": monitor.stat_id(self.observe)}
        return m

    def events(self, kind=None):
        ev = rcb_limits.read_jsonl(os.path.join(self.rec, "events.jsonl"))
        return [e for e in ev if kind is None or e.get("event") == kind]

    def close(self):
        shutil.rmtree(self.d, ignore_errors=True)


# --------------------------------------------------------------------------------------------------- scope discovery
def test_classify_race():
    """M2: a container seen before its pivot_root (new mount namespace, still the host's root) is found at a later tick."""
    f = Fake()
    try:
        m = f.monitor()
        croot = f.root("main")
        f.pid(100, "mnt:[111]", f.host)                       # runc init before pivot_root
        f.pid(50, "mnt:[2]", f.host)                          # some host service with its own mount namespace
        check(m.groups() == {}, "nothing is watched before the container's root is in place")
        check(not m.cls, "a namespace that does not show the beacon is not remembered")
        f.pid(100, "mnt:[111]", croot, nspid=1)               # pivot_root done
        f.pid(101, "mnt:[111]", croot, start=1100, ppid=100, nspid=7)
        g = m.groups()
        check(set(g) == {"mnt:[111]"} and g["mnt:[111]"]["kind"] == "scope" and g["mnt:[111]"]["role"] == "main"
              and sorted(g["mnt:[111]"]["pids"]) == [100, 101], f"the container is found after its pivot_root: {g}")
        check(m.groups() == g, "and stays found")
        # the witness of the namespace ends, another of its processes proves it again
        f.gone(100)
        g = m.groups()
        check(g["mnt:[111]"]["pids"] == [101], f"still watched through another process: {g}")
        # the namespace ends and its number is given to a container of another trial: not watched
        f.gone(101)
        other = f.root("other", beacon=False)
        f.pid(200, "mnt:[111]", other, start=2000)
        check(m.groups() == {} and not m.cls, "a reused namespace number is asked again, not trusted")
    finally:
        f.close()


def test_dead_leader_and_zombies():
    """m4: a process whose first thread has exited is still found (through a living thread); zombies count."""
    f = Fake()
    try:
        m = f.monitor()
        croot = f.root("main")
        f.pid(100, "mnt:[111]", croot, nspid=1)
        f.pid(300, None, None, start=1300, state="Z", ppid=100, threads=2)     # leader gone, thread 301 runs on
        f.thread(300, 300, None, None, start=1300)
        f.thread(300, 301, "mnt:[111]", croot, start=1301)
        f.pid(400, None, None, start=1400, state="Z", ppid=100)                # a zombie child of the container's init
        f.thread(400, 400, None, None, start=1400)
        f.pid(500, None, None, start=1500, state="Z", ppid=50)                 # a zombie of a host process
        f.pid(50, "mnt:[2]", f.host)
        g = m.groups()["mnt:[111]"]
        check(sorted(g["pids"]) == [100, 300] and g["zombies"] == 1, f"dead leader counted as a process, zombie counted: {g}")
        check(m.base(300).endswith("/300/task/301"), "the process is read through its living thread")
        ms = m.measure(g, {300: 2048}, 10.0)
        check(ms["processes"] == 3 and ms["zombies"] == 1 and ms["gpu_memory_gb"] == 2.0,
              f"its GPU memory is the scope's, zombies count as processes: {ms}")
    finally:
        f.close()


# --------------------------------------------------------------------------------------------------- disk
def _fill(path, mib):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "wb") as fh:
        fh.write(os.urandom(1024) * (1024 * mib))


def test_disk_walk():
    f = Fake()
    try:
        croot = f.root("main")
        _fill(os.path.join(croot, "app", "a.bin"), 2)
        _fill(os.path.join(croot, "app", "sub", "deep", "b.bin"), 1)
        os.link(os.path.join(croot, "app", "a.bin"), os.path.join(croot, "app", "sub", "a-again.bin"))
        _fill(os.path.join(croot, "app", "skip", "c.bin"), 4)
        _fill(os.path.join(croot, "opt", "d.bin"), 4)                          # not under disk_paths
        w = monitor.DiskWalk(croot, ["/app", "/tmp"], ["/app/skip"])
        check(w.step(60) and w.done and w.why is None, "a small tree is walked in one step")
        mib = w.bytes / 2 ** 20
        check(3.0 <= mib < 3.5, f"2 + 1 MiB: the hard link once, the excluded and the other paths not at all ({mib:.2f})")
        w = monitor.DiskWalk(croot, ["/app"], [])
        check(not w.step(0) and not w.done, "a walk out of budget is not done")
        check(w.step(60) and w.done and 7.0 <= w.bytes / 2 ** 20 < 7.5, "and goes on where it stopped")
    finally:
        f.close()


def test_disk_baseline():
    """M5: only a complete walk sets the baseline or counts as a measurement; a restart keeps the baseline."""
    f = Fake()
    keep = (monitor.DISK_SLICE_SEC, monitor.DISK_WALK_MAX_SEC)
    try:
        lim = {"disk_gb": 0.004, "disk_tick_sec": 60, "tick_sec": 5}
        m = f.monitor(lim)
        m.disk_tick = 60.0
        croot = f.root("main")
        f.pid(100, "mnt:[111]", croot, nspid=1)
        _fill(os.path.join(croot, "app", "sub", "base.bin"), 2)
        groups = m.groups()
        # 1. the first walk does not finish: no baseline, no value, an event
        monitor.DISK_SLICE_SEC, monitor.DISK_WALK_MAX_SEC = 0.0, -1.0
        check(m.disk_step(1000.0, groups) is None and m.disk_base is None, "a partial walk never becomes the baseline")
        check(len(f.events("disk_partial")) == 1, "and is recorded as a gap")
        check(m.disk_step(1005.0, groups) is None and len(f.events("disk_partial")) == 1, "the next walk waits for the disk tick")
        # 2. a complete walk: the baseline
        monitor.DISK_SLICE_SEC, monitor.DISK_WALK_MAX_SEC = keep
        check(m.disk_step(1060.0, groups) == 0.0 and abs(m.disk_base * 1024 - 2.0) < 0.3, f"baseline {m.disk_base}")
        check(len(f.events("disk_baseline")) == 1, "the baseline is recorded")
        # 3. growth is measured against it
        _fill(os.path.join(croot, "app", "sub", "new.bin"), 3)
        v = m.disk_step(1120.0, groups)
        check(v is not None and abs(v * 1024 - 3.0) < 0.3, f"3 MiB of growth: {v}")
        # 4. a walk that does not finish and has seen less than the limit: a gap (None), never "under"
        monitor.DISK_SLICE_SEC, monitor.DISK_WALK_MAX_SEC = 0.0, -1.0
        check(m.disk_step(1180.0, groups) is None, "a partial walk below the limit is not a measurement")
        # 5. ... but one that has already seen more than baseline + limit proves the violation
        _fill(os.path.join(croot, "app", "big.bin"), 8)                        # in the first directory read
        v = m.disk_step(1240.0, groups)
        check(v is not None and v > lim["disk_gb"], f"a lower bound over the limit counts: {v}")
        # 6. the container's process ends during the walk: nothing is measured
        monitor.DISK_SLICE_SEC, monitor.DISK_WALK_MAX_SEC = keep
        f.pid(100, "mnt:[111]", croot, start=9999, nspid=1)                    # another process under the same PID
        m.walk = (monitor.DiskWalk(croot, ["/app"], []), "mnt:[111]", f"{f.proc}/100", 1000)
        n = len(f.events("disk_partial"))
        check(m.disk_step(1300.0, groups) is None and len(f.events("disk_partial")) == n + 1, "a vanished container: a gap")
        # 7. a restarted monitor keeps the baseline and the backstop's counts, and lets go of an open throttle: the
        #    processes the previous instance throttled (same start time) get SIGCONT, a quota it wrote its old value
        import signal
        base = m.disk_base
        for r in ("memory", "memory", "pids"):
            monitor.event("kill", resource=r, pid=7, value=2.0, cap=1.0)
        monitor.event("throttle_start", resource="cpu", value=3.0, cap=1.0)
        quota = os.path.join(f.d, "cpu.max")
        open(quota, "w").write("100000 100000")
        f.pid(105, "mnt:[111]", croot, start=7000, ppid=100, nspid=12)
        st = rcb_limits.read_json(os.path.join(f.rec, "state.json")) or {}
        st["throttle"] = [{"procs": [[105, 7000], [106, 7001]]}, {"file": quota, "orig": "max 100000"}]
        monitor.write_json(os.path.join(f.rec, "state.json"), st)
        m2 = f.monitor(lim)
        check(m2.disk_base is None, "a fresh monitor has no baseline")
        with Kills() as k:
            m2.restore()
        check(m2.disk_base == base and m2.killed == 3 and m2.kills["memory"] == 2 and m2.kills["pids"] == 1,
              f"restore() brings the state back: {m2.kills}")
        check(k.calls == [(105, signal.SIGCONT)] and open(quota).read() == "max 100000"
              and f.events("throttle_end")[-1]["processes"] == 1 and not m2.resume
              and not (rcb_limits.read_json(os.path.join(f.rec, "state.json")) or {}).get("throttle"),
              f"and lets go of the previous throttle (a PID no longer there is not signalled): {k.calls}")
    finally:
        monitor.DISK_SLICE_SEC, monitor.DISK_WALK_MAX_SEC = keep
        f.close()


# --------------------------------------------------------------------------------------------------- the backstop
def _agent_container(f, per=None, mnt="mnt:[111]", root=None):
    per = per or {}
    """The agent's container: PID 1 (100), its keepalive (103, started with it), and the agent's processes 101, 102
    (exec'd into it) and 104 (an orphan PID 1 adopted later). per: {pid: kwargs of Fake.pid}."""
    root = root or f.root("main")
    f.pid(100, mnt, root, start=1000, nspid=1, **per.get(100, {}))
    f.pid(103, mnt, root, start=1001, ppid=100, nspid=7, **per.get(103, {}))
    f.pid(101, mnt, root, start=5000, ppid=77, nspid=9, **per.get(101, {}))
    f.pid(102, mnt, root, start=5001, ppid=101, nspid=10, **per.get(102, {}))
    f.pid(104, mnt, root, start=6000, ppid=100, nspid=11, **per.get(104, {}))
    return root


class Kills:
    """os.kill recorded instead of sent."""

    def __enter__(self):
        self.calls, self.real = [], os.kill
        os.kill = lambda pid, sig: self.calls.append((pid, sig))
        return self

    def __exit__(self, *a):
        os.kill = self.real

    def killed(self):
        import signal
        return [p for p, s in self.calls if s == signal.SIGKILL]


GPU_DEVICE = ("    deploy:\n      resources:\n        reservations:\n          devices:\n            - driver: nvidia\n"
              "              count: 1\n              capabilities: [gpu]\n")


def _limits(compose, cfg, task="test-i00-privileged"):
    """limits.json exactly as inject.py writes it for a task (never a hand-written dict: the monitor must read the
    shape the tasks really carry)."""
    return inject.limits_json(Path(task), cfg, inject.applied_caps(cfg, compose))


# main: 2 CPUs, 1 GiB, 5 processes, a GPU with a 4 GiB cap, 4 MiB of disk; blenderkit (a service the agent drives):
# 1 CPU, 0.5 GiB, 2; scorer (the benchmark's own: observed)
LIM_COMPOSE = ("services:\n  main:\n    pids_limit: 5\n    x-gpu-memory-gb: 4\n" + GPU_DEVICE +
               "  blenderkit:\n    cpus: 1\n    mem_limit: 512m\n    pids_limit: 2\n"
               "  scorer:\n    cpus: 1\n    mem_limit: 100m\n    pids_limit: 1\n")
LIM_CFG = {"environment": {"cpus": 2, "memory_mb": 1024, "storage_mb": 4}, "agent": {"timeout_sec": 3600}}
LIM = _limits(LIM_COMPOSE, LIM_CFG)
MIB = 1024                                                     # kB


def _one(m, t=100.0, gpu_per=None, gpu_unknown=()):
    """One sample of the scope: (groups, measures), the backstop run on each container of the scope."""
    groups = m.groups()
    ms = [m.measure(g, gpu_per or {}, t, gpu_unknown) for g in groups.values()]
    for g, x in zip(groups.values(), ms):
        if g["kind"] == "scope":
            m.backstop(g, x, t)
    return groups, ms


def test_backstop_memory():
    """Memory over the container's cap (no kernel cap here: PSS): the process with the largest PSS is killed, one per
    tick (the next tick measures again); never PID 1 or its keepalive; nothing when what may be killed cannot cover the
    excess, or holds next to nothing; 0 kills nothing; within the tolerance nothing."""
    f = Fake()
    try:
        per = {100: {"pss_kb": 50 * MIB, "rss_kb": 50 * MIB}, 103: {"pss_kb": MIB, "rss_kb": MIB},
               101: {"pss_kb": 600 * MIB, "rss_kb": 650 * MIB}, 102: {"pss_kb": 300 * MIB, "rss_kb": 320 * MIB},
               104: {"pss_kb": 200 * MIB, "rss_kb": 210 * MIB}}
        _agent_container(f, per)                             # 1151 MiB of a 1 GiB cap
        for enforce in ("0", "1"):
            shutil.rmtree(f.rec)
            os.makedirs(f.rec)
            with Kills() as k:
                m = f.monitor(LIM, enforce=enforce)
                _, ms = _one(m)
            check(ms[0]["service"] == "main" and ms[0]["memory_gb"] > 1.05, f"measured: {ms}")
            over = f.events("over")
            check([(e["resource"], e["service"]) for e in over] == [("memory", "main")], f"over, recorded: {over}")
            if enforce == "0":
                check(k.calls == [] and not f.events("kill"), f"0: nothing is killed: {k.calls}")
            else:
                ev = f.events("kill")
                check(k.killed() == [101] and len(ev) == 1 and ev[0]["resource"] == "memory" and ev[0]["pid"] == 101
                      and m.kills["memory"] == 1, f"1: the largest process, once (then under the cap): {k.calls} {ev}")
        # still over after the largest: one kill per tick, the next one at the next tick, after measuring again
        per = {100: {"pss_kb": 10 * MIB}, 103: {"pss_kb": MIB}, 101: {"pss_kb": 610 * MIB},
               102: {"pss_kb": 605 * MIB}, 104: {"pss_kb": 600 * MIB}}         # 1826 MiB: 750 MiB over 1.05 GiB
        _agent_container(f, per)
        shutil.rmtree(f.rec)
        os.makedirs(f.rec)
        m = f.monitor(LIM, enforce="1")
        with Kills() as k:
            _one(m)
        check(k.killed() == [101], f"one kill per tick, the largest: {k.killed()}")
        f.gone(101)
        with Kills() as k:
            _one(m, 105.0)
        check(k.killed() == [102], f"the next tick, measured again, the next largest: {k.killed()}")
        f.gone(102)
        with Kills() as k:
            _one(m, 110.0)
        check(k.calls == [] and m.kills["memory"] == 2, f"under the cap again: no more: {k.calls}")
        # PID 1's own memory is never a reason to kill the others: what they hold does not cover the excess
        per = {100: {"pss_kb": 1500 * MIB}, 103: {"pss_kb": MIB}, 101: {"pss_kb": 600 * MIB},
               102: {"pss_kb": 300 * MIB}, 104: {"pss_kb": 200 * MIB}}
        _agent_container(f, per)
        shutil.rmtree(f.rec)
        os.makedirs(f.rec)
        with Kills() as k:
            _one(f.monitor(LIM, enforce="1"))
        check(k.calls == [] and [e["resource"] for e in f.events("no_victim")] == ["memory"],
              f"the killable processes cannot cover the excess: nothing is killed, recorded: {k.calls}")
        # processes that hold next to nothing are never killed for memory (killing them frees nothing)
        per = {100: {"pss_kb": 1100 * MIB}, 103: {"pss_kb": MIB}, 101: {"pss_kb": 5 * MIB},
               102: {"pss_kb": 5 * MIB}, 104: {"pss_kb": 5 * MIB}}
        _agent_container(f, per)
        shutil.rmtree(f.rec)
        os.makedirs(f.rec)
        with Kills() as k:
            _one(f.monitor(LIM, enforce="1"))
        check(k.calls == [] and f.events("no_victim"), f"near-zero PSS: never a victim: {k.calls}")
        # within the tolerance (5 %): not over
        per = {p: {"pss_kb": 10 * MIB, "rss_kb": 10 * MIB} for p in (100, 103, 102, 104)}
        per[101] = {"pss_kb": 1030 * MIB, "rss_kb": 1030 * MIB}   # 1070 MiB = 1.045 GiB
        _agent_container(f, per)
        shutil.rmtree(f.rec)
        os.makedirs(f.rec)
        with Kills() as k:
            _one(f.monitor(LIM, enforce="1"))
        check(k.calls == [] and not f.events("over"), f"within the tolerance: {k.calls}")
    finally:
        f.close()


def test_backstop_pids():
    """Processes and threads over the cap in rcb_limits.PIDS_RUN samples in a row: the newest processes are killed
    until the count is back at the cap; one sample over alone is not over (a fork burst between two samples)."""
    f = Fake()
    try:
        _agent_container(f, {101: {"threads": 3}})            # 1 + 1 + 3 + 1 + 1 = 7 of 5
        for enforce in ("0", "1"):
            shutil.rmtree(f.rec)
            os.makedirs(f.rec)
            with Kills() as k:
                m = f.monitor(LIM, enforce=enforce)
                _, ms = _one(m)
                check(ms[0]["pids"] == 7 and k.calls == [] and not f.events("over"),
                      f"one sample over: not yet: {ms[0]} {k.calls}")
                _, ms = _one(m, 102.0)
            check(ms[0]["pids"] == 7, f"processes and threads counted: {ms[0]}")
            check([e["resource"] for e in f.events("over")] == ["pids"], f"over: {f.events('over')}")
            if enforce == "0":
                check(k.calls == [], "0: nothing is killed")
            else:
                check(k.killed() == [104, 102] and m.kills["pids"] == 2,
                      f"1: newest first, until at the cap (7 - 1 - 1 = 5): {k.killed()}")
                # the victims are zombies until their parent reaps them: they count (as for pids.max), but never
                # make the backstop kill again
                for p, start in ((104, 6000), (102, 5001)):
                    f.pid(p, None, None, start=start, state="Z", ppid=101)
                    f.thread(p, p, None, None, start=start)
                with Kills() as k2:
                    _one(m, 105.0)
                    _, ms = _one(m, 110.0)
                check(ms[0]["pids"] == 7 and ms[0]["zombies"] == 2 and k2.calls == [],
                      f"its own victims' zombies: no second kill: {ms[0]} {k2.calls}")
                _agent_container(f, {101: {"threads": 3}})
        # over, under, over: never two samples in a row
        _agent_container(f, {101: {"threads": 3}})
        shutil.rmtree(f.rec)
        os.makedirs(f.rec)
        m = f.monitor(LIM, enforce="1")
        with Kills() as k:
            _one(m)
            _agent_container(f, {101: {"threads": 1}})
            _one(m, 105.0)
            _agent_container(f, {101: {"threads": 3}})
            _one(m, 110.0)
        check(k.calls == [] and not f.events("over"), f"over, under, over: not over: {k.calls}")
    finally:
        f.close()


def test_backstop_cpu():
    """CPU over the cap: never a kill; the container is duty-cycled (SIGCONT, run for cap / demand of the period,
    SIGSTOP), parents stopped first and continued last; released when its demand fits under the cap again."""
    import signal
    f = Fake()
    try:
        _agent_container(f)
        for enforce in ("0", "1"):
            shutil.rmtree(f.rec)
            os.makedirs(f.rec)
            with Kills() as k:
                m = f.monitor(LIM, enforce=enforce)
                groups = m.groups()
                g = next(iter(groups.values()))
                mnt = g["mnt"]
                x = m.measure(g, {}, 100.0)
                x["cpu"] = 4.0                                # twice the cap of 2 CPUs
                m.backstop(g, x, 100.0)
                check([e["resource"] for e in f.events("over")] == ["cpu"], "over, recorded")
                if enforce == "0":
                    check(not m.throttle.active and k.calls == [], "0: no throttle")
                    continue
                check(m.throttle.throttled(mnt) and abs(m.throttle.active[mnt]["frac"] - 0.5) < 1e-9,
                      f"1: throttled to half of each period: {m.throttle.active}")
                check(set(m.throttle.active[mnt]["procs"]) == {101, 102, 104}, "never PID 1 or its keepalive")
                slept, vt = [], [0.0]

                def vsleep(d):
                    slept.append(d)
                    vt[0] += d
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                check(k.calls == [(101, signal.SIGSTOP), (102, signal.SIGSTOP), (104, signal.SIGSTOP)],
                      f"first period: nothing to resume (they run), parents stopped first: {k.calls}")
                check(abs(sum(slept) - monitor.THROTTLE_PERIOD_SEC) < 1e-9 and abs(slept[0] - 0.05) < 1e-9,
                      f"runs half the period: {slept}")
                check(abs(m.throttle.active[mnt]["landed"] - monitor.THROTTLE_PERIOD_SEC) < 1e-9,
                      "the period counts: its stops reached the processes")
                del k.calls[:]
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                order = [(p, sg) for p, sg in k.calls]
                cont = [p for p, sg in order if sg == signal.SIGCONT]
                stop = [p for p, sg in order if sg == signal.SIGSTOP]
                check(cont == [104, 102, 101] and stop == [101, 102, 104] and not k.killed(),
                      f"children continued first, parents stopped first, no kill: {order}")
                ev = f.events("throttle_start")
                check(len(ev) == 1 and ev[0]["cap"] == 2.0 and ev[0]["fraction"] == 0.5, f"recorded: {ev}")
                x["cpu"] = 2.0                                 # used 2 at a share of 0.5: the demand is still 4
                m.backstop(g, x, 105.0)
                check(m.throttle.throttled(mnt), "demand still over the cap: still throttled")
                del k.calls[:]
                x["cpu"] = 0.8                                 # demand 1.6 now: fits under 2
                m.backstop(g, x, 110.0)
                check(not m.throttle.throttled(mnt) and set(p for p, sg in k.calls if sg == signal.SIGCONT) == {101, 102, 104},
                      f"released, every process continued: {k.calls}")
                ev = f.events("throttle_end")
                check(len(ev) == 1 and abs(ev[0]["sec"] - 0.2) < 1e-9 and ev[0]["engaged_sec"] >= 0,
                      f"recorded: held by signals for two periods: {ev}")
                check(rcb_limits.backstop(f.events())["throttles"] == 1, "counted")
                # a throttle left open is closed at the seal
                m.throttle.engage(mnt, g, 2.0, 4.0, 120.0)
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                del k.calls[:]
                m.seal()
                check(not m.throttle.active and {p for p, sg in k.calls if sg == signal.SIGCONT} == {101, 102, 104},
                      "the seal lets every throttled process go")
    finally:
        f.close()


def test_backstop_gpu():
    """GPU memory over the container's cap: the process holding the most is killed (one per tick), when the processes
    the backstop may kill can cover the excess; a container whose GPU memory NVML does not tell is never killed for it
    (recorded); 0 kills nothing. The limits are inject.py's: the cap is there when the service holds a GPU (an older
    monitor looked for a per-service `gpu` flag, which limits.json does not carry, and never killed)."""
    check(LIM["caps"]["main"].get("gpu_memory_gb") == 4.0 and "gpu" not in LIM["caps"]["main"] and LIM["gpu"] is True,
          f"limits.json as inject.py writes it: {LIM}")
    f = Fake()
    try:
        _agent_container(f)
        for enforce in ("0", "1"):
            shutil.rmtree(f.rec)
            os.makedirs(f.rec)
            with Kills() as k:
                m = f.monitor(LIM, enforce=enforce)
                _, ms = _one(m, gpu_per={101: 3000, 102: 2000, 100: 100})
            check(ms[0]["gpu_memory_gb"] > 4.2 and [e["resource"] for e in f.events("over")] == ["gpu_memory"],
                  f"over: {ms[0]}")
            check(k.killed() == ([] if enforce == "0" else [101]), f"enforce={enforce}: {k.killed()}")
        shutil.rmtree(f.rec)
        os.makedirs(f.rec)
        with Kills() as k:
            _one(f.monitor(LIM, enforce="1"), gpu_per={101: 3000, 102: 3000}, gpu_unknown={104})
        check(k.calls == [] and [e["resource"] for e in f.events("unobservable")] == ["gpu_memory"],
              f"not observable: nothing is killed, recorded: {k.calls}")
        # a service without a GPU has no GPU-memory cap (inject.py writes none)
        lim = _limits("services:\n  main:\n    pids_limit: 5\n", LIM_CFG)
        check("gpu_memory_gb" not in lim["caps"]["main"] and lim["gpu"] is False, f"no GPU: no cap: {lim}")
        shutil.rmtree(f.rec)
        os.makedirs(f.rec)
        with Kills() as k:
            _one(f.monitor(lim, enforce="1"), gpu_per={101: 9000})
        check(k.calls == [] and not f.events("over"), "no GPU in the service: recorded only")
    finally:
        f.close()


def _fd(f, pid, n, target, flags="0100001"):
    """An open file of a fake process: /proc/<pid>/fd/<n> -> target (a path in its container), fdinfo flags (octal)."""
    base = os.path.join(f.proc, str(pid))
    os.makedirs(os.path.join(base, "fd"), exist_ok=True)
    os.makedirs(os.path.join(base, "fdinfo"), exist_ok=True)
    link = os.path.join(base, "fd", str(n))
    if os.path.lexists(link):
        os.remove(link)
    os.symlink(target, link)
    with open(os.path.join(base, "fdinfo", str(n)), "w") as fh:
        fh.write(f"pos:\t0\nflags:\t{flags}\nmnt_id:\t1\n")


def test_backstop_disk():
    """Disk over the cap at a measurement: the process that holds the most bytes open for writing in files under the
    measured paths is killed; PID 1, read-only and deleted files never count; no second kill until the disk grew since
    the last one; nobody holding such a file (or fd tables that cannot be read): recorded only; 0 kills nothing."""
    tag = "rcb-test-" + os.urandom(4).hex()            # paths that exist in the fake container only, never on the host
    f = Fake()
    try:
        croot = _agent_container(f)
        for name, mib in (("w102", 30), ("w101", 12), ("r104", 40), ("w100", 50), ("w103", 45), ("log", 1)):
            _fill(os.path.join(croot, "app", tag, name + ".bin"), mib)
        _fd(f, 102, 3, f"/app/{tag}/w102.bin")
        _fd(f, 102, 4, f"/app/{tag}/w102.bin", flags="0100002")                # the same file twice: once
        _fd(f, 101, 3, f"/app/{tag}/w101.bin", flags="02")
        _fd(f, 104, 3, f"/app/{tag}/r104.bin", flags="0100000")                # read only
        _fd(f, 104, 4, f"/app/{tag}/w102.bin (deleted)")                       # deleted: not counted
        _fd(f, 104, 5, f"/opt/{tag}/w100.bin")                                 # not under the measured paths
        _fd(f, 100, 3, f"/app/{tag}/w100.bin")                                 # PID 1: spared
        _fd(f, 103, 3, f"/app/{tag}/w103.bin")                                 # its keepalive: spared
        for enforce in ("0", "1"):
            shutil.rmtree(f.rec)
            os.makedirs(f.rec)
            with Kills() as k:
                m = f.monitor(LIM, enforce=enforce)
                c = LIM["disk_gb"] * (1.0 + m.tol)            # the cap, with the tolerance
                groups, _ = _one(m)
                g = next(iter(groups.values()))
                m.disk_backstop(g, 0.001)                      # under the cap: nothing
                m.disk_backstop(g, round(c + 0.02, 6))         # 20 MiB over: 102 holds 30
            check([e["resource"] for e in f.events("over")] == ["disk"], f"over: {f.events('over')}")
            if enforce == "0":
                check(k.calls == [] and m.by[g["mnt"]]["disk"] == "none", "0: nothing is killed")
                continue
            ev = f.events("kill")
            check(k.killed() == [102] and ev[0]["resource"] == "disk" and m.kills["disk"] == 1
                  and 29 < ev[0]["held_gb"] * 1024 < 31 and m.disk_kill_at == round(c + 0.02, 6),
                  f"1: the largest writer, PID 1 and its keepalive spared: {k.killed()} {ev}")
            with Kills() as k:
                m.disk_backstop(g, round(c + 0.025, 6))
            check(k.calls == [] and f.events("no_victim")[0]["resource"] == "disk",
                  f"no growth since the last kill: no second kill: {k.calls}")
            f.gone(102)
            with Kills() as k:
                m.disk_backstop(g, round(c + 0.035, 6))         # 35 MiB over: 101 holds 12, killing it frees too little
            nv = [e for e in f.events("no_victim") if e["resource"] == "disk"]
            check(k.calls == [] and len(nv) == 2 and 11 < nv[1]["held_gb"] * 1024 < 13,
                  f"the largest writer holds less than the excess: nobody is killed: {k.calls} {nv}")
            # under the cap again (the agent cleaned up): the last kill is forgotten, a refill is new growth
            m.disk_backstop(g, 0.001)
            check(m.disk_kill_at is None, "under the cap: disk_kill_at reset")
            with Kills() as k:
                m.disk_backstop(g, round(c + 0.011, 6))         # 11 MiB over, at or below the last kill's value
            check(k.killed() == [101], f"a refill after a clean-up: the writer that covers it: {k.killed()}")
            check((rcb_limits.read_json(os.path.join(f.rec, "state.json")) or {}).get("disk_kill_at")
                  == round(c + 0.011, 6), "the last kill's value survives a restart")
            # a small log held open covers no excess, and is under the minimum: never the victim (the harness's own)
            f.gone(101)
            _fd(f, 104, 6, f"/app/{tag}/log.bin")
            m.disk_kill_at = None
            with Kills() as k:
                m.disk_backstop(g, round(c + 0.0005, 6))
            check(k.calls == [], f"under DISK_MIN_VICTIM_GB: never killed: {k.calls}")
        # nobody holds a file there: recorded only
        f2 = Fake()
        try:
            _agent_container(f2)
            with Kills() as k:
                m = f2.monitor(LIM, enforce="1")
                groups, _ = _one(m)
                m.disk_backstop(next(iter(groups.values())), 0.2)
            check(k.calls == [] and [e["resource"] for e in f2.events("no_victim")] == ["disk"],
                  f"no process holds a file open for writing: nobody is killed: {k.calls}")
            # an fd table that cannot be read: the writer cannot be told
            open(os.path.join(f2.proc, "101", "fd"), "w").close()
            m.disk_kill_at = None
            with Kills() as k:
                m.disk_backstop(next(iter(groups.values())), 0.3)
            check(k.calls == [] and [e["resource"] for e in f2.events("unobservable")] == ["disk"],
                  f"unreadable open files: nothing is killed: {k.calls}")
        finally:
            f2.close()
    finally:
        f.close()


def _cg_files(d, **files):
    for name, v in files.items():
        with open(os.path.join(d, name.replace("_", ".", 1)), "w") as fh:
            fh.write(f"{v}\n")


def test_runtime_first():
    """A cap the container's own cgroup already holds (cpu.max, memory.max, pids.max at the cap or below: Docker's) is
    the runtime's: the monitor only records it; a limit missing or higher than the cap is the monitor's to enforce.
    Who enforces what is recorded (an `enforcement` event, `by` in `over`, usage.json `enforced_by`)."""
    f = Fake()
    keep_beacon = monitor.SCOPE_BEACON
    try:
        per = {100: {"pss_kb": 10 * MIB}, 101: {"pss_kb": 900 * MIB, "threads": 3}, 102: {"pss_kb": 300 * MIB},
               103: {"pss_kb": MIB}, 104: {"pss_kb": 200 * MIB}}
        _agent_container(f, per)                                             # 7 pids of 5
        pids = [100, 101, 102, 103, 104]
        d = f.cgroup("rt", pids=pids, anon_gb=1.5, max_gb=1.0)               # 1.5 GiB of 1 (an OOM kill is due)
        _cg_files(d, cpu_max="200000 100000", pids_max=5)
        m = f.monitor(LIM, enforce="1")
        with Kills() as k:
            m.tick_once(100.0)
            m.tick_once(105.0)
            monitor.SCOPE_BEACON = f.volume              # usage.json is written there (the beacon volume)
            m.write_usage(105.0)
            monitor.SCOPE_BEACON = keep_beacon
        want = {"cpu": "runtime", "memory": "runtime", "pids": "runtime", "gpu_memory": "monitor", "disk": "monitor"}
        g = next(iter(m.by))
        check(k.calls == [] and m.by[g] == want, f"the runtime holds cpu, memory, pids: the monitor kills nothing: "
                                                 f"{k.calls} {m.by}")
        ev = f.events("enforcement")
        check(len(ev) == 1 and ev[0]["by"] == want and ev[0]["runtime"] == {"cpus": 2.0, "memory_gb": 1.0, "pids": 5},
              f"recorded once: {ev}")
        over = {e["resource"]: e["by"] for e in f.events("over")}
        check(over == {"memory": "runtime", "pids": "runtime"}, f"over, and who holds it: {over}")
        u = rcb_limits.read_json(os.path.join(f.volume, "usage.json"))
        check(u["enforced_by"] == want and u["violations"] == u["over"] and u["emergency_kills"] == 0,
              f"usage.json: {u}")
        # a runtime limit above the cap, or none: the monitor's
        _cg_files(d, memory_max=2 * 2 ** 30, pids_max="max")
        with Kills() as k:
            m.tick_once(110.0)
            m.tick_once(115.0)
        check(m.by[g]["memory"] == "monitor" and m.by[g]["pids"] == "monitor" and m.by[g]["cpu"] == "runtime"
              and 101 in k.killed() and len(f.events("enforcement")) == 2, f"now the monitor's: {m.by} {k.killed()}")
        check(m.by_svc["main"] == m.by[g], f"per service: {m.by_svc}")
        # CPU: no quota (cpu.max "max"), writable: the monitor writes the cap there, and puts back the old value
        _cg_files(d, cpu_max="max 100000", memory_max=2 ** 30, pids_max=5)
        m2 = f.monitor(LIM, enforce="1")
        groups = m2.groups()
        g2 = next(iter(groups.values()))
        x = m2.measure(g2, {}, 100.0)
        x["cpu"] = 4.0
        with Kills() as k:
            m2.backstop(g2, x, 100.0)
            check(m2.throttle.active[g2["mnt"]]["method"] == "cgroup" and open(d + "/cpu.max").read() == "200000 100000",
                  f"the cap is the cgroup's quota: {open(d + '/cpu.max').read()!r}")
            x = m2.measure(g2, {}, 105.0)
            x["cpu"] = 2.0
            m2.backstop(g2, x, 105.0)
            check(m2.throttle.throttled(g2["mnt"]) and m2.by[g2["mnt"]]["cpu"] == "monitor",
                  "the monitor's own quota is not the runtime's: it stays")
            st = rcb_limits.read_json(os.path.join(f.rec, "state.json"))
            check(st["throttle"] == [{"file": d + "/cpu.max", "orig": "max 100000", "pfile": None, "porig": None}], f"state.json: {st['throttle']}")
            m2.seal()
        check(open(d + "/cpu.max").read() == "max 100000" and k.calls == [] and
              f.events("throttle_start")[-1]["method"] == "cgroup", f"released at the seal: the old value, no signal: {open(d + '/cpu.max').read()!r} {k.calls}")
    finally:
        monitor.SCOPE_BEACON = keep_beacon
        f.close()


def test_throttle_signals():
    """The SIGSTOP fallback: a process already stopped (state T: the agent's own) is never signalled, so never resumed;
    a share too small for one run window runs every k-th period; a process that leaves the set gets SIGCONT."""
    import signal
    f = Fake()
    try:
        _agent_container(f, {102: {"state": "T"}})
        m = f.monitor(LIM, enforce="1")
        groups = m.groups()
        g = next(iter(groups.values()))
        mnt = g["mnt"]
        x = m.measure(g, {}, 100.0)
        x["cpu"] = 400.0                                       # share 0.005: 0.5 ms of a 100 ms period
        with Kills() as k:
            m.backstop(g, x, 100.0)
            e = m.throttle.active[mnt]
            check(e["method"] == "signal" and set(e["procs"]) == {101, 104} and {p for p, _ in e["foreign"]} == {102},
                  f"the stopped process is not ours: {e['procs']} {e['foreign']}")
            vt = [0.0]

            def vsleep(dt):
                vt[0] += dt
            k_need = __import__("math").ceil(monitor.THROTTLE_MIN_RUN_SEC / (e["frac"] * monitor.THROTTLE_PERIOD_SEC))
            conts = []
            for i in range(2 * k_need):
                n0 = len(k.calls)
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                conts.append(sum(1 for _, sg in k.calls[n0:] if sg == signal.SIGCONT))
            check(conts == ([0] * (k_need - 1) + [2]) * 2, f"run every {k_need}th period: {conts}")
            check(all(p != 102 for p, _ in k.calls), f"never the process stopped before: {k.calls}")
            # 104 leaves the container (another mount namespace, no longer the trial's): continued, then dropped
            del k.calls[:]
            f.pid(104, "mnt:[999]", f.root("main"), start=6000, ppid=100, nspid=11)
            m.throttle._refresh(mnt, e)
            check(k.calls == [(104, signal.SIGCONT)] and set(e["procs"]) == {101}, f"dropped: {k.calls} {e['procs']}")
            # 101's first thread exits, another runs on: still in the set, still let go at the end
            root = f.root("main")
            f.pid(101, None, None, start=5000, state="Z", ppid=77)
            f.thread(101, 1011, "mnt:[111]", root, start=5000)
            del k.calls[:]
            m.throttle._refresh(mnt, e)
            check(k.calls == [] and set(e["procs"]) == {101}, f"a dead leader with live threads stays: {e['procs']}")
            m.throttle.release_all("test")
        check(k.calls == [(101, signal.SIGCONT)], f"released: only what it stopped: {k.calls}")
    finally:
        f.close()


def test_cgroup_container():
    """A container is its cgroup when it has one of its own (docker-<id>.scope): a process of it in another mount
    namespace (unshare -m, maybe pivoted away from the beacon) still counts and can be killed; two namespaces of one
    container cgroup are one container. A kill needs the start time the process was measured with."""
    f = Fake()
    try:
        root = _agent_container(f)
        esc = f.root("escaped", beacon=False, main=False)
        f.pid(150, "mnt:[555]", esc, start=9000, ppid=101, nspid=20)        # no beacon: found by its cgroup alone
        f.pid(160, "mnt:[666]", f.root("unshared", main=False), start=9500, ppid=101, nspid=21)
        name = "system.slice/docker-" + "ab" * 32 + ".scope"
        f.cgroup(name, pids=[100, 101, 102, 103, 104, 150, 160], anon_gb=0.1)
        other = f.root("other", beacon=False, main=False)
        f.pid(170, "mnt:[777]", other, start=9600, nspid=1)                  # another container: not the trial's
        f.cgroup("system.slice/docker-" + "cd" * 32 + ".scope", pids=[170], anon_gb=0.1)
        m = f.monitor(LIM, enforce="1")
        groups = m.groups()
        check(len(groups) == 1, f"one container: {list(groups)}")
        g = next(iter(groups.values()))
        check(g["mnt"] == "mnt:[111]" and set(g["pids"]) == {100, 101, 102, 103, 104, 150, 160}
              and g["mnts"] == {"mnt:[111]", "mnt:[555]", "mnt:[666]"} and g["role"] == "main",
              f"named by its PID 1's namespace, every process of the cgroup: {g}")
        check(m.in_trial(150) and not m.in_trial(170), "the cgroup is the trial's, the other one is not")
        with Kills() as k:
            _one(m)
            _one(m, 105.0)
        check(k.killed() == [160, 150], f"7 of 5 pids: the newest, in whatever namespace: {k.killed()}")
        check(m.kill(g, 999, "pids", 7, 5) is False and m.kill(dict(g, info={}), 101, "pids", 7, 5) is False,
              "no start time measured: never killed")
    finally:
        f.close()


def test_runtime_while_starting():
    """The runtime's limits are read from the container's own cgroup even while it holds a process the monitor does not
    see yet (a container starting): its memory is then measured by PSS, its caps still the runtime's."""
    f = Fake()
    try:
        _agent_container(f)
        name = "system.slice/docker-" + "ef" * 32 + ".scope"
        d = f.cgroup(name, pids=[100, 101, 102, 103, 104], anon_gb=0.1, max_gb=1.0)
        f.pid(999, "mnt:[2]", f.host, start=9900)                # runc's init: still in the host's mount namespace
        with open(os.path.join(d, "cgroup.procs"), "a") as fh:
            fh.write("999\n")
        _cg_files(d, cpu_max="200000 100000", pids_max=5)
        m = f.monitor(LIM, enforce="1")
        with Kills():
            _, ms = _one(m)
        check(ms[0]["cgroup"] == 0 and ms[0]["runtime"] == {"cpus": 2.0, "memory_gb": 1.0, "pids": 5}
              and m.by["mnt:[111]"]["memory"] == "runtime", f"the runtime's limits: {ms[0]} {m.by}")
        # a fork between the /proc walk and the read of cgroup.procs (in the container's namespace), and one gone since:
        # still the container's own cgroup
        f.gone(999)
        f.pid(998, "mnt:[111]", f.root("main"), start=9950, ppid=101, nspid=30)
        with open(os.path.join(d, "cgroup.procs"), "w") as fh:
            fh.write("".join(f"{p}\n" for p in (100, 101, 102, 103, 104, 997, 998)))
        check(m.cgroup_of([100, 101, 102, 103, 104], {"mnt:[111]"}) == d, "the fork race: still its own cgroup")
        f.pid(996, "mnt:[777]", f.root("other", beacon=False, main=False), start=9960, nspid=1)
        with open(os.path.join(d, "cgroup.procs"), "a") as fh:
            fh.write("996\n")
        check(m.cgroup_of([100, 101, 102, 103, 104], {"mnt:[111]"}) is None, "another container's process: shared")
        # a limit on a parent that carries the container's id (crun's `container` subgroup) holds the container
        sub = f.cgroup(name + "/container", pids=[100, 101, 102, 103, 104])
        _cg_files(sub, cpu_max="max 100000", pids_max="max", memory_max="max")
        check(m.runtime_limits(sub, {}) == {"cpus": 2.0, "memory_gb": 1.0, "pids": 5},
              f"the parent's limits: {m.runtime_limits(sub, {})}")
    finally:
        f.close()


def test_bad_tolerance():
    f = Fake()
    try:
        os.environ["RCB_LIMITS_TOLERANCE"] = "lots"
        m = f.monitor(LIM, enforce="1")
        check(m.enforce == 0 and "lots" in (m.switch_error or ""), f"a bad tolerance enforces nothing: {m.switch_error}")
    finally:
        os.environ.pop("RCB_LIMITS_TOLERANCE", None)
        f.close()


def test_selftest_fixtures():
    """The self-test's limits.json fixtures are inject.limits_json's output for its task (so neither suite can test a
    shape the tasks do not carry)."""
    for compose, want in ((selftest.TASK_COMPOSE_GPU, selftest.LIMITS_GPU), (selftest.TASK_COMPOSE_NOGPU, selftest.LIMITS_NOGPU)):
        got = _limits(compose, selftest.TASK_CFG, want["task"])
        check(got == want, f"selftest fixture != inject.limits_json: {got} vs {want}")


def test_backstop_services():
    """Each container gets its own service's caps: the agent's `main`, the one other scope service (a service the
    agent drives); a benchmark-owned (observed) service is recorded only; caps that cannot be told enforce nothing."""
    f = Fake()
    try:
        _agent_container(f)
        broot = f.root("blenderkit", main=False)
        f.pid(300, "mnt:[333]", broot, start=1000, nspid=1)
        f.pid(301, "mnt:[333]", broot, start=5000, ppid=300, nspid=5)
        f.pid(302, "mnt:[333]", broot, start=5001, ppid=300, nspid=6)   # 3 processes of a cap of 2
        sroot = f.sidecar_root("scorer")
        for p in (200, 201, 202):
            f.pid(p, "mnt:[222]", sroot, start=5000 + p, ppid=200, nspid=p - 199)
        with Kills() as k:
            m = f.monitor(LIM, enforce="1")
            _one(m)
            groups, ms = _one(m, 105.0)
        svc = sorted((x["kind"], x["role"], x["service"] or "") for x in ms)
        check(svc == [("observe", "sidecar", ""), ("scope", "main", "main"), ("scope", "sidecar", "blenderkit")], f"{svc}")
        check(k.killed() == [302] and f.events("kill")[0]["service"] == "blenderkit",
              f"the driven service's own cap (2): its newest process; the sim (cap 1, 3 processes) untouched: {k.killed()}")
        lim = dict(LIM, scope_services=["main", "blenderkit", "other"])
        shutil.rmtree(f.rec)
        os.makedirs(f.rec)
        with Kills() as k:
            _one(f.monitor(lim, enforce="1"))
        check(k.calls == [] and f.events("unobservable")[0]["resource"] == "caps", "several candidates: nothing enforced")
    finally:
        f.close()


def test_backstop_record():
    """The verifier's view of a record with backstop actions: per-resource counts, over_<metric>, the grade untouched."""
    ok = {"format": rcb_limits.FORMAT, "sealed": True, "scope_seen": True}
    lim = {"format": rcb_limits.LIMITS_FORMAT, "tick_sec": 5, "caps": {"main": {"memory_gb": 2.0, "pids": 100, "cpus": 2}}}
    ev = [{"t": 1010, "event": "kill", "resource": "memory", "service": "main", "pid": 7, "value": 2.5, "cap": 2.0},
          {"t": 1011, "event": "kill", "resource": "memory", "service": "main", "pid": 8, "value": 2.2, "cap": 2.0},
          {"t": 1020, "event": "kill", "resource": "pids", "service": "main", "pid": 9, "value": 120, "cap": 100},
          {"t": 1030, "event": "throttle_start", "resource": "cpu", "service": "main", "value": 4.0, "cap": 2},
          {"t": 1060, "event": "throttle_end", "resource": "cpu", "service": "main", "sec": 30.0}]
    for enforce in (0, 1):
        out, txt, det = _verify(_samples(), ev, dict(ok, enforce=enforce), limits=lim)
        check(out["success"] == 1 and txt == "1" and out.get("infra_error", 0) == 0 and out["limits_violation_observed"] == 1
              and out["resource_memory_kills"] == 2 and out["resource_pids_kills"] == 1 and out["resource_monitor_kills"] == 3
              and out["resource_emergency_kills"] == 3 and out["resource_cpu_throttles"] == 1
              and out["resource_cpu_throttled_sec"] == 30.0 and out["over_memory_gb"] == 1 and out["over_pids"] == 1
              and out["over_cpus"] == 1 and out["resource_limit_exceeded"] == 0, f"enforce={enforce}: {out}")
        check(len(det["backstop_events"]) == 5 and "graded normally" in det["verdict"], f"detail: {det['verdict']}")
    c = {"kind": "scope", "role": "main", "service": "main", "memory_gb": 3.0, "pids": 10, "cpu": 1.0}
    samples = [dict(s, containers=[c]) for s in _samples()]
    out, txt, _ = _verify(samples, (), dict(ok, enforce=1, tolerance=0.6), limits=lim)
    check(out["limits_violation_observed"] == 0, f"the record's tolerance is the verifier's: {out}")
    out, txt, _ = _verify(samples, (), dict(ok, enforce=1, tolerance=0.05), limits=lim)
    check(out["limits_violation_observed"] == 1 and out["over_memory_gb"] == 1 and out["success"] == 1 and txt == "1",
          f"over in a sample, enforced: recorded, graded normally: {out}")


# --------------------------------------------------------------------------------------------------- the switch, the verifier step
def test_switch():
    for v, want in (("0", 0), ("1", 1), ("true", 1), ("False", 0), ("TRUE", 1), ("", 0), (None, 0), (1, 1), (0, 0), (True, 1)):
        check(rcb_limits.parse_switch(v) == (want, None), f"RCB_LIMITS_ENFORCE={v!r} -> {want}")
    for v in ("yes", "on", "2", "enforce", "-1"):
        e, why = rcb_limits.parse_switch(v)
        check(e == 0 and why and repr(v) in why, f"RCB_LIMITS_ENFORCE={v!r} is an error, not a silent off")
    f = Fake()
    try:
        m = f.monitor(enforce="yes")
        check(m.enforce == 0 and m.switch_error, "the monitor does not guess")
    finally:
        f.close()


def _verify(samples=None, events=(), summary=None, env=None, limits=None, reward=None, no_limits=False, txt="1"):
    """Run the verifier step on a made-up record; returns (reward.json, reward.txt, resource_limits.json)."""
    lim = limits if limits is not None else {"format": rcb_limits.LIMITS_FORMAT, "gpu_memory_gb": 4.0, "disk_gb": 10.0,
                                             "tick_sec": 5}
    with tempfile.TemporaryDirectory() as d:
        rec, tests, logs = (os.path.join(d, x) for x in ("rec", "tests", "logs"))
        for x in (tests, logs):
            os.makedirs(x)
        if not no_limits:
            json.dump(lim, open(os.path.join(tests, "rcb_limits.json"), "w"))
        if samples is not None:
            os.makedirs(rec)
            with open(os.path.join(rec, "samples.jsonl"), "w") as fh:
                for s in samples:
                    fh.write(json.dumps(s) + "\n")
            with open(os.path.join(rec, "events.jsonl"), "w") as fh:
                for e in events:
                    fh.write(json.dumps(e) + "\n")
            if summary is not None:
                json.dump(summary, open(os.path.join(rec, "summary.json"), "w"))
        json.dump(reward if reward is not None else {"success": 1, "infra_error": 0}, open(os.path.join(logs, "reward.json"), "w"))
        open(os.path.join(logs, "reward.txt"), "w").write(txt + "\n")
        shutil.copy(os.path.join(HERE, "rcb_limits.py"), tests)
        e = dict(os.environ, RCB_MONITOR_RECORD=rec, RCB_VERIFIER_LOGS=logs)
        e.pop("RCB_LIMITS_ENFORCE", None)
        e.update(env or {})
        r = subprocess.run([sys.executable, os.path.join(tests, "rcb_limits.py"), "verify"], env=e, capture_output=True, text=True)
        check(r.returncode == 0, f"the step never changes the verifier's exit status: {r.stderr}")
        det = os.path.join(logs, "resource_limits.json")
        return (json.load(open(os.path.join(logs, "reward.json"))), open(os.path.join(logs, "reward.txt")).read().strip(),
                json.load(open(det)) if os.path.exists(det) else None)


def _samples(n=40, gpu=1.0, **scope):
    return [{"t": 1000 + 5 * i, "scope": dict({"gpu_memory_gb": gpu, "memory_gb": 2.0, "processes": 10, "disk_gb": None},
                                              **scope)} for i in range(n)]


def test_verify_edges():
    ok = {"format": rcb_limits.FORMAT, "sealed": True, "scope_seen": True}
    # a backstop kill: recorded, the grader's verdict stands (enforced)
    ev = [{"t": 1010, "event": "kill", "resource": "gpu_memory", "service": "main", "pid": 5, "value": 9.0, "cap": 4.0}]
    out, txt, det = _verify(_samples(), ev, dict(ok, enforce=1))
    check(out["success"] == 1 and txt == "1" and out["resource_gpu_memory_kills"] == 1 and out["limits_violation_observed"] == 1
          and out["over_gpu_memory_gb"] == 1 and out.get("resource_limit_exceeded", 0) == 0 and out.get("infra_error", 0) == 0,
          f"a backstop kill: {out}")
    check(len(det["backstop_events"]) == 1, "and listed in resource_limits.json")
    # the tolerance: unreadable -> not a result, like the switch
    out, txt, _ = _verify(_samples(), (), dict(ok, enforce=1, switch_error="RCB_LIMITS_TOLERANCE='x' is not a number in [0, 1]"))
    check(out["infra_error"] == 1 and out["resource_switch_error"] == 1 and txt == "0", f"tolerance error: {out}")
    # the switch: unreadable -> not a result, in the record or in the verifier's own environment
    out, txt, det = _verify(_samples(), (), dict(ok, enforce=0, switch_error="RCB_LIMITS_ENFORCE='yes' is not one of 0, 1, true, false"))
    check(out["infra_error"] == 1 and out["resource_switch_error"] == 1 and out["success"] == 0 and txt == "0"
          and out["success_before_limits"] == 1, f"switch error in the record: not a result, and no success beside it: {out}")
    out, _, _ = _verify(None, env={"RCB_LIMITS_ENFORCE": "yes"})
    check(out["infra_error"] == 1 and out["resource_switch_error"] == 1 and out["resource_monitor_missing"] == 1,
          f"switch error, no record: {out}")
    for v, want in (("true", 1), ("false", 0), ("1", 1), ("0", 0)):
        out, _, _ = _verify(None, env={"RCB_LIMITS_ENFORCE": v})
        check(out["resource_enforced"] == want and out.get("infra_error", 0) == want and out["resource_monitor_missing"] == 1,
              f"no record, RCB_LIMITS_ENFORCE={v}: infra only when enforced: {out}")
    # a record that is there but not usable, enforcement off: reported only
    out, txt, det = _verify(_samples(), [{"t": 1001, "event": "fatal", "why": "x"}], dict(ok, enforce=0))
    check(out["resource_monitor_missing"] == 1 and out.get("infra_error", 0) == 0 and out["success"] == 1, f"observe only: {out}")
    # NVML does not tell the scope's GPU memory: the GPU limit was not measured
    blind = _samples(gpu=0.0, gpu_unmeasured=1)
    out, _, det = _verify(blind, (), dict(ok, enforce=1))
    check(out["infra_error"] == 1 and out["resource_monitor_missing"] == 1 and "NVML" in " ".join(det["record_problems"]),
          f"GPU memory not measured, enforced: {out}")
    out, _, _ = _verify(blind, (), dict(ok, enforce=0))
    check(out.get("infra_error", 0) == 0 and out["resource_monitor_missing"] == 1, f"GPU memory not measured, observed: {out}")
    nogpu = {"format": rcb_limits.LIMITS_FORMAT, "disk_gb": 10.0}   # no GPU in the scope: no GPU-memory cap (key absent)
    out, _, _ = _verify(blind, (), dict(ok, enforce=1), limits=nogpu)
    check(out["resource_monitor_missing"] == 0, "no GPU limit: nothing was missed")
    out, _, det = _verify(_samples(gpu=9.0), (), dict(ok, enforce=1), limits=nogpu)
    check(out["limits_violation_observed"] == 0 and out["success"] == 1 and out["peak_gpu_memory_gb"] == 9.0,
          f"GPU memory seen in a scope without a GPU cap: recorded, never a violation: {out}")
    # limits of another format: no cap can be read -> an unusable record (not a result when enforced)
    old = {"gpu_memory_gb": 4.0, "disk_gb": 10.0, "tick_sec": 5}
    out, _, det = _verify(_samples(), (), dict(ok, enforce=1), limits=old)
    check(out["infra_error"] == 1 and out["resource_monitor_missing"] == 1
          and "limits format" in " ".join(det["record_problems"]), f"limits without rcb-limits/2, enforced: {out}")
    out, _, _ = _verify(_samples(), (), dict(ok, enforce=0), limits=old)
    check(out.get("infra_error", 0) == 0 and out["resource_monitor_missing"] == 1 and out["success"] == 1,
          f"limits without rcb-limits/2, observed: {out}")
    # the step itself fails (a broken limits file): reported in reward.json, as infra only when enforcement was asked for
    out, txt, det = _verify(_samples(), (), dict(ok, enforce=0), limits=["broken"], env={"RCB_LIMITS_ENFORCE": "1"})
    check(out["resource_step_error"] == 1 and out["infra_error"] == 1 and out["success"] == 0 and txt == "0"
          and out["success_before_limits"] == 1, f"step failed, enforcement asked for: not a result: {out}")
    out, txt, _ = _verify(_samples(), (), dict(ok, enforce=0), limits=["broken"])
    check(out["resource_step_error"] == 1 and out.get("infra_error", 0) == 0 and out["success"] == 1 and txt == "1",
          f"step failed, not enforced: {out}")
    # R2 m1: an unusable record with enforcement on is not a result, and leaves no success beside the infra flag
    out, txt, _ = _verify(_samples(), [{"t": 1001, "event": "fatal", "why": "x"}], dict(ok, enforce=1))
    check(out["infra_error"] == 1 and out["success"] == 0 and out["success_before_limits"] == 1 and txt == "0"
          and out.get("resource_limit_exceeded", 0) == 0, f"unusable record, enforced: {out}")
    out, txt, _ = _verify(_samples(), [{"t": 1001, "event": "fatal", "why": "x"}], dict(ok, enforce=1),
                          reward={"success": 1, "reward": 1.0, "infra_error": 0})
    check(out["reward"] == 0 and out["success"] == 0 and out["infra_error"] == 1, f"... the reward field too: {out}")
    # the verifier's copy of the limits is missing: reported (before: "within limits")
    out, txt, det = _verify(_samples(), (), dict(ok, enforce=0), limits=None, no_limits=True)
    check(out["resource_monitor_missing"] == 1 and out["success"] == 1 and txt == "1" and out.get("infra_error", 0) == 0
          and "rcb_limits.json" in " ".join(det["record_problems"]), f"no limits file, not enforced: {out}")
    out, txt, _ = _verify(_samples(), (), dict(ok, enforce=1), limits=None, no_limits=True)
    check(out["infra_error"] == 1 and out["success"] == 0 and txt == "0", f"no limits file, enforced: not a result: {out}")


# --------------------------------------------------------------------------------------------------- OOM kills
def test_oom_watch():
    """R2 M3 / M2: an OOM kill is the container's own cap's only when its cgroup's `oom` counter rose with it; the
    host's OOM killer leaves `oom` alone. A benchmark-owned service's kills are recorded as such. Counters that start
    again (the container was restarted) are compared with 0."""
    f = Fake()
    keep = monitor.OOM_CAP_WINDOW_SEC
    try:
        m = f.monitor({"caps": {"main": {"memory_gb": 24.0}}, "tick_sec": 5})
        f.pid(100, "mnt:[111]", f.root("main"), nspid=1)
        f.pid(101, "mnt:[111]", f.root("main"), start=5000, ppid=77, nspid=9)
        f.pid(200, "mnt:[222]", f.sidecar_root("sim"), nspid=1)
        cg = f.cgroup("docker-main.scope", pids=[100, 101], anon_gb=3.0, max_gb=29.0, oom=0, oom_kill=0)
        f.cgroup("docker-sim.scope", pids=[200], anon_gb=20.0, max_gb=24.0, oom=4, oom_kill=2)   # before it was watched
        m.tick_once(1000.0)
        g = m.groups()
        check(sorted((x["kind"], x["role"]) for x in g.values()) == [("observe", "sidecar"), ("scope", "main")], f"both watched: {g}")
        check(f.events("oom") == [], "the first reading is the baseline: nothing from before the container was watched")
        # 1. the reviewer's case: the trial uses 3 of 29 GiB, the host runs out of memory (other trials) and its OOM
        #    killer takes a process of this container: oom_kill rises, oom (the cgroup's own limit) does not
        f.cgroup("docker-main.scope", oom=0, oom_kill=1)
        m.tick_once(1005.0)
        ev = f.events("oom")
        check(len(ev) == 1 and ev[0]["cause"] == "host" and ev[0]["scope"] is True and ev[0]["role"] == "main"
              and ev[0]["count"] == 1 and ev[0]["memory_cap_gb"] == 29.0 and ev[0]["memory_current_gb"] == 3.0,
              f"a kill without the cgroup's own oom event is the host's: {ev}")
        check(rcb_limits.violations([], m.lim, ev) == [], "and not a violation")
        # 2. the container reaches its own cap: oom and oom_kill rise together
        f.cgroup("docker-main.scope", max=7, oom=1, oom_kill=2)
        check(m.oomw.scan() is True, "a change is reported (the watch then reads faster)")
        ev = f.events("oom")
        check(len(ev) == 2 and ev[1]["cause"] == "cap" and ev[1]["cap_hits"] == 1, f"at its own cap: {ev}")
        check([v["metric"] for v in rcb_limits.violations([], m.lim, ev)] == ["oom"], "which is the violation")
        check(m.oomw.scan() is False and len(f.events("oom")) == 2, "nothing new: no event, no change")
        # 3. the oom event is read first, the kill a moment later: still the cap's; long after: the host's
        f.cgroup("docker-main.scope", max=9, oom=2, oom_kill=2)
        m.oomw.scan()
        f.cgroup("docker-main.scope", max=9, oom=2, oom_kill=3)
        m.oomw.scan()
        check(f.events("oom")[-1]["cause"] == "cap", "the kill that follows the cgroup's oom event is the cap's")
        m.oomw.w[cg]["cap_t"] -= monitor.OOM_CAP_WINDOW_SEC + 1
        f.cgroup("docker-main.scope", max=9, oom=2, oom_kill=4)
        m.oomw.scan()
        check(f.events("oom")[-1]["cause"] == "host", "a kill long after the last oom event is not")
        # 4. the benchmark's own service is killed at its cap: recorded as the service's (scope false)
        f.cgroup("docker-sim.scope", oom=5, oom_kill=3)
        m.oomw.scan()
        e = f.events("oom")[-1]
        check(e["scope"] is False and e["role"] == "sidecar" and e["cause"] == "cap" and e["count"] == 1, f"the sidecar's: {e}")
        # 5. the container was restarted: a new cgroup under the same path, its counters start at 0 and are all the trial's
        n = len(f.events("oom"))
        f.cgroup("docker-sim.scope", oom=0, oom_kill=0)
        m.oomw.scan()
        check(len(f.events("oom")) == n, "counters that went back: no event")
        f.cgroup("docker-sim.scope", oom=1, oom_kill=1)
        m.oomw.scan()
        check(len(f.events("oom")) == n + 1 and f.events("oom")[-1]["count"] == 1, "then counted from 0")
        # ... also when the old cgroup's file stopped answering (ENODEV on a real cgroup) and the path is there again:
        # another directory under the same name
        sim_cg = os.path.join(f.d, "cgroup", "docker-sim.scope")
        os.close(m.oomw.w[sim_cg]["fd"])
        shutil.rmtree(sim_cg)
        import time
        time.sleep(0.02)                                       # (a new creation time, whatever inode it gets)
        f.cgroup("docker-sim.scope", pids=[200], anon_gb=20.0, max_gb=24.0, oom=1, oom_kill=1)   # already lost a process
        m.oomw.scan()
        e = f.events("oom")
        check(len(e) == n + 2 and e[-1]["count"] == 1 and e[-1]["cause"] == "cap" and e[-1]["scope"] is False,
              f"a new cgroup under the same path is compared with 0: {e[-2:]}")
        # ... but the same cgroup opened again (its file descriptor was lost, the cgroup was not) counts nothing twice
        os.close(m.oomw.w[sim_cg]["fd"])
        check(m.oomw.scan() is False and len(f.events("oom")) == n + 2, "the same cgroup reopened: no kill counted twice")
        f.cgroup("docker-sim.scope", oom=2, oom_kill=2)
        m.oomw.scan()
        check(len(f.events("oom")) == n + 3 and f.events("oom")[-1]["count"] == 1, "and goes on counting from where it was")
        # 6. a cgroup without a memory cap cannot be at its cap; a file that is gone or unreadable stops nothing
        f.cgroup("docker-main.scope", max_gb="max", oom=2, oom_kill=5)
        m.oomw.scan()
        e = f.events("oom")[-1]
        check(e["cause"] == "host" and "memory_cap_gb" not in e, f"no cap: {e}")
        os.remove(os.path.join(f.d, "cgroup", "docker-sim.scope", "memory.events"))
        os.close(m.oomw.w[os.path.join(f.d, "cgroup", "docker-sim.scope")]["fd"])
        n = len(f.events("oom"))
        check(m.oomw.scan() is False and len(f.events("oom")) == n, "a cgroup that is gone: nothing")
        with open(os.path.join(f.d, "cgroup", "docker-main.scope", "memory.events"), "w") as fh:
            fh.write("garbage\noom_kill x\n")
        check(m.oomw.scan() is False and len(f.events("oom")) == n, "unreadable counters: nothing")
        # 7. the watch thread reads on its own, between the samples
        f.cgroup("docker-main.scope", max_gb=29.0, oom=2, oom_kill=5)
        m.oomw.scan()
        monitor.OOM_CAP_WINDOW_SEC = keep
        m.oomw.start()
        f.cgroup("docker-main.scope", oom=3, oom_kill=6)
        end = time.time() + 5
        while time.time() < end and len(f.events("oom")) == n:
            time.sleep(0.01)
        check(len(f.events("oom")) == n + 1 and f.events("oom")[-1]["cause"] == "cap", "the thread records a kill by itself")
        # the seal closes the record: the counters are read once more, then the thread ends (what happens to a sidecar
        # in the collect hooks after it is the grader's to judge)
        f.cgroup("docker-main.scope", oom=4, oom_kill=7)
        m.seal()
        check(len(f.events("oom")) == n + 2 and m.oomw.stopped, "a kill just before the seal is in the sealed record")
        m.oomw.thread.join(5)
        check(not m.oomw.thread.is_alive(), "the watch thread ends with the seal")
        f.cgroup("docker-main.scope", oom=5, oom_kill=8)
        time.sleep(0.2)
        check(len(f.events("oom")) == n + 2, "nothing is recorded after the seal")
        summary = rcb_limits.read_json(os.path.join(f.rec, "summary.json"))
        check(summary["sealed"] and any(v["metric"] == "oom" for v in summary["violations"]), f"the summary has the cap kills: {summary['violations']}")
    finally:
        monitor.OOM_CAP_WINDOW_SEC = keep
        f.close()


def test_oom_verdicts():
    """R2 M2 / M3: what the verifier step makes of the kernel's OOM kills."""
    ok = {"format": rcb_limits.FORMAT, "sealed": True, "scope_seen": True}
    agent0 = {"success": 0, "infra_error": 0, "finalized": 1}        # e.g. a standard grader on reason = sim_crashed_on_call
    side = [{"t": t, "event": "oom", "role": "sidecar", "scope": False, "count": 1, "cause": "cap"} for t in (1100, 1150, 1190)]
    for enforce in (0, 1):
        # M2: the simulator sidecar was OOM-killed at its docker cap and the episode ended without success: not the agent's 0
        out, txt, det = _verify(_samples(), side, dict(ok, enforce=enforce), reward=agent0, txt="0")
        check(out["infra_error"] == 1 and out["resource_service_oom"] == 1 and out["resource_service_oom_kills"] == 3
              and out["success"] == 0 and txt == "0" and out.get("resource_limit_exceeded", 0) == 0
              and out["limits_violation_observed"] == 0 and "benchmark-owned service" in det["verdict"],
              f"sidecar OOM, no success, enforce={enforce}: infrastructure, not the agent's failure: {out}")
        check(len(det["oom_kills"]) == 3, "the events are in resource_limits.json")
        # ... the same when the host's killer took it
        host_side = [dict(e, cause="host") for e in side]
        out, _, _ = _verify(_samples(), host_side, dict(ok, enforce=enforce), reward=agent0, txt="0")
        check(out["infra_error"] == 1 and out["resource_service_oom"] == 1, f"sidecar killed by the host: {out}")
        # ... but a success stands: the service recovered and the grader accepted the episode
        out, txt, _ = _verify(_samples(), side, dict(ok, enforce=enforce))
        check(out["success"] == 1 and txt == "1" and out.get("infra_error", 0) == 0 and out["resource_service_oom_kills"] == 3
              and "resource_service_oom" not in out, f"sidecar OOM, success, enforce={enforce}: the verdict stands: {out}")
        # no kill in the record: nothing changes
        out, txt, _ = _verify(_samples(), (), dict(ok, enforce=enforce), reward=agent0, txt="0")
        check(out["infra_error"] == 0 and out["resource_service_oom_kills"] == 0 and txt == "0", f"no OOM: the agent's 0: {out}")
    # no record at all (a host that cannot run the monitor): the step stays a no-op, as before
    out, txt, _ = _verify(None, reward=agent0, txt="0")
    check(out["infra_error"] == 0 and out["resource_monitor_missing"] == 1 and out["resource_service_oom_kills"] == 0,
          f"no record, not enforced: reported only: {out}")
    # M3: a process in the scope killed by the host's OOM killer (not at its container's cap)
    host = [{"t": 1100, "event": "oom", "role": "main", "scope": True, "count": 1, "cause": "host"}]
    out, txt, det = _verify(_samples(), host, dict(ok, enforce=1))
    check(out["infra_error"] == 1 and out["resource_host_oom"] == 1 and out["resource_host_oom_kills"] == 1
          and out.get("resource_limit_exceeded", 0) == 0 and "over_oom" not in out and out["limits_violation_observed"] == 0
          and out["success"] == 0 and out["success_before_limits"] == 1 and txt == "0",
          f"host OOM kill, enforced: not a result, never the agent's violation: {out}")
    out, txt, _ = _verify(_samples(), host, dict(ok, enforce=0))
    check(out["success"] == 1 and txt == "1" and out.get("infra_error", 0) == 0 and out["resource_host_oom_kills"] == 1
          and "resource_host_oom" not in out, f"host OOM kill, observed only: reported: {out}")
    # ... at its own cap: over the cap, recorded; the grade stands in both settings (like Docker's OOM kill)
    cap = [dict(host[0], cause="cap")]
    for enforce in (0, 1):
        out, txt, _ = _verify(_samples(), cap, dict(ok, enforce=enforce))
        check(out["success"] == 1 and txt == "1" and out["limits_violation_observed"] == 1 and out["over_oom"] == 1
              and out["resource_oom_kills"] == 1 and out.get("infra_error", 0) == 0 and out.get("resource_limit_exceeded", 0) == 0,
              f"OOM at the cap, enforce={enforce}: recorded, graded normally: {out}")
    # the agent's own OOM kill and the sidecar's in one episode without success: the sidecar's makes it not a result
    out, _, _ = _verify(_samples(), cap + side, dict(ok, enforce=1), reward=agent0, txt="0")
    check(out["infra_error"] == 1 and out["resource_service_oom"] == 1, f"cap and sidecar OOM: {out}")


# --------------------------------------------------------------------------------------------------- the agent's files, before the grader
def test_sanitize():
    """R2 m6: what the agent left in /app/output is untrusted input. A link out of it (rcb_session_log.jsonl ->
    /dev/zero made a grader read without end), a FIFO (opening it blocks) and an endless session log are dealt with
    before the grader runs; everything a grader could read safely stays byte for byte."""
    d = tempfile.mkdtemp(prefix="rcb-sanitize-")
    try:
        out = os.path.join(d, "app", "output")
        os.makedirs(os.path.join(out, "sub"))
        outside = os.path.join(d, "secret.txt")
        open(outside, "w").write("the verifier's own file\n")
        open(os.path.join(out, "trajectory.npz"), "wb").write(b"PK\x03\x04 deliverable")
        open(os.path.join(out, "sub", "notes.txt"), "w").write("notes\n")
        os.symlink("/dev/zero", os.path.join(out, "rcb_session_log.jsonl"))            # the reviewer's case
        os.symlink(outside, os.path.join(out, "sub", "peek.txt"))                       # a link out of the directory
        os.symlink("trajectory.npz", os.path.join(out, "latest.npz"))                   # a link to the agent's own file: stays
        os.symlink(os.path.join(d, "app"), os.path.join(out, "up"))                     # a link to a directory (a loop)
        os.symlink("nowhere", os.path.join(out, "dangling"))
        os.mkfifo(os.path.join(out, "pipe.jsonl"))
        done = rcb_limits.sanitize([out])
        left = sorted(os.path.relpath(os.path.join(dp, n), out) for dp, dn, fn in os.walk(out) for n in dn + fn)
        check(left == ["latest.npz", "sub", "sub/notes.txt", "trajectory.npz"], f"what is left: {left}")
        check(len(done) == 5, f"five things removed: {done}")
        check(open(os.path.join(out, "latest.npz"), "rb").read() == b"PK\x03\x04 deliverable"
              and open(outside).read() == "the verifier's own file\n", "the deliverable and the link's target are untouched")
        check(rcb_limits.sanitize([out]) == [], "nothing more to do the second time")
        # an endless session log is cut; a normal one is not touched
        log = os.path.join(out, "rcb_session_log.jsonl")
        open(log, "w").write('{"ev": "session_start"}\n' * 10)
        check(rcb_limits.sanitize([out], max_log=4096) == [] and os.path.getsize(log) == 240, "a normal log stays")
        open(log, "w").write("x" * 10000)
        done = rcb_limits.sanitize([out], max_log=4096)
        check(os.path.getsize(log) == 4096 and len(done) == 1, f"a log over the cap is cut: {done}")
        # the grader's reading loop (as in the privileged graders) now ends on the reviewer's input
        os.remove(log)
        os.symlink("/dev/zero", log)
        rcb_limits.sanitize([out])
        n = 0
        try:
            with open(log) as fh:
                for n, _line in enumerate(fh):
                    pass
        except OSError:
            pass
        check(n == 0 and not os.path.lexists(log), "the session log that pointed to /dev/zero is gone")
        # a session log linked to a file of the agent's own inside the directory (one huge line) is removed, not followed;
        # the file it pointed to stays (it is the agent's), and a grader that opens the log by name finds none
        big = os.path.join(out, "sub", "big.bin")
        open(big, "wb").write(b"x" * 10000)
        os.symlink(os.path.join("sub", "big.bin"), log)
        done = rcb_limits.sanitize([out], max_log=4096)
        check(not os.path.lexists(log) and any("session log is a symbolic link" in x for x in done)
              and os.path.getsize(big) == 10000, f"a session log linked inside the directory is removed: {done}")
        os.remove(big)
        # a root that is missing or itself a link, and the command line the generated test.sh line runs: never an error
        check(rcb_limits.sanitize([os.path.join(d, "missing")]) == [], "no directory: nothing")
        os.symlink("/", os.path.join(d, "rootlink"))
        check(len(rcb_limits.sanitize([os.path.join(d, "rootlink")])) == 1 and os.path.isdir("/tmp"), "a root that is a link is removed, not walked")
        os.mkfifo(os.path.join(out, "again"))
        r = subprocess.run([sys.executable, os.path.join(HERE, "rcb_limits.py"), "sanitize", out, os.path.join(d, "missing")],
                           capture_output=True, text=True)
        check(r.returncode == 0 and "again" in r.stderr and not os.path.lexists(os.path.join(out, "again")), f"the command: {r.stderr}")
    finally:
        shutil.rmtree(d, ignore_errors=True)


# --------------------------------------------------------------------------------------------------- the default changes no verdict
def _step_in_process(reward_json, reward_txt, record, switch):
    """The verifier step (run_step) in this process, on a temporary /tests, record and /logs/verifier."""
    d = tempfile.mkdtemp(prefix="rcb-verify-")
    try:
        rec, tests, logs = (os.path.join(d, x) for x in ("rec", "tests", "logs"))
        for x in (tests, logs):
            os.makedirs(x)
        json.dump({"format": rcb_limits.LIMITS_FORMAT, "gpu_memory_gb": 4.0, "disk_gb": 10.0, "tick_sec": 5},
                  open(os.path.join(tests, "rcb_limits.json"), "w"))
        shutil.copy(os.path.join(HERE, "rcb_limits.py"), tests)
        if reward_json is not None:
            open(os.path.join(logs, "reward.json"), "w").write(reward_json)
        if reward_txt is not None:
            open(os.path.join(logs, "reward.txt"), "w").write(reward_txt)
        if record is not None:
            os.makedirs(rec)
            samples, events, summary = record
            with open(os.path.join(rec, "samples.jsonl"), "w") as fh:
                for x in samples:
                    fh.write((x if isinstance(x, str) else json.dumps(x)) + "\n")
            with open(os.path.join(rec, "events.jsonl"), "w") as fh:
                for x in events:
                    fh.write((x if isinstance(x, str) else json.dumps(x)) + "\n")
            if summary is not None:
                json.dump(summary, open(os.path.join(rec, "summary.json"), "w"))
        keep = {k: os.environ.get(k) for k in ("RCB_MONITOR_RECORD", "RCB_VERIFIER_LOGS", "RCB_LIMITS_ENFORCE")}
        os.environ.update(RCB_MONITOR_RECORD=rec, RCB_VERIFIER_LOGS=logs)
        os.environ.pop("RCB_LIMITS_ENFORCE", None)
        if switch is not None:
            os.environ["RCB_LIMITS_ENFORCE"] = switch
        err = sys.stderr
        try:
            sys.stderr = open(os.devnull, "w")
            spec = importlib.util.spec_from_file_location("rcb_limits_under_test", os.path.join(tests, "rcb_limits.py"))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            rc = mod.run_step()
        finally:
            sys.stderr.close()
            sys.stderr = err
            for k, v in keep.items():
                os.environ.pop(k, None) if v is None else os.environ.__setitem__(k, v)

        def rd(name):
            try:
                return open(os.path.join(logs, name)).read()
            except OSError:
                return None
        return rc, rd("reward.json"), rd("reward.txt")
    finally:
        shutil.rmtree(d, ignore_errors=True)


def test_default_is_neutral():
    """With the default (RCB_LIMITS_ENFORCE unset / 0 / false / empty) the step never changes the grader's verdict:
    reward.txt, success and reward stay what they were for every grader output x every record x every spelling of
    "off", and it never creates a reward.json. infra_error changes in exactly one case, in both modes (R2 M2): a
    benchmark-owned service was OOM-killed and the grader's verdict is not a success. (The reviewer's battery of round
    2, 1320 combinations, extended by the records with OOM kills.)"""
    def samples(gpu=6.0, n=40):
        return [{"t": 1000 + 5 * i, "scope": {"gpu_memory_gb": gpu if i > 5 else 0.5, "memory_gb": 2.0, "processes": 10,
                                               "disk_gb": None}} for i in range(n)]
    ok = {"format": rcb_limits.FORMAT, "sealed": True, "enforce": 0, "scope_seen": True}
    oom = {"event": "oom", "t": 1010, "count": 1}
    rewards = {"ok1": '{"success": 1, "infra_error": 0}', "ok0": '{"success": 0}', "nosuccess": '{"reward": 1}',
               "list": "[1]", "invalid": '{"success": 1', "empty": "", "none": None, "nan": '{"success": 1, "x": NaN}',
               "infra": '{"success": 0, "infra_error": 1}', "str": '{"success": 1, "note": "abc"}',
               "fail_reward": '{"success": 0, "reward": 0.0, "infra_error": 0}', "false": '{"success": false, "infra_error": 0}'}
    records = {
        "norecord": None,
        "violating_obs": (samples(), [], ok),
        "oom_obs": (samples(1), [dict(oom, scope=True, role="main", cause="cap")], ok),
        "oom_old_format": (samples(1), [dict(oom, scope=True, role="main")], ok),
        "oom_host": (samples(1), [dict(oom, scope=True, role="main", cause="host")], ok),
        "fatal_obs": (samples(1), [{"event": "fatal", "why": "x"}], ok),
        "unsealed": (samples(), [], None),
        "garbage_samples": (['{"t": "abc", "scope": 5}', '{"scope": {}}', "[]", "5", '{"t": 3}'], [], ok),
        "garbage_events": (samples(1), ["[]", "5", '"oom"', '{"event": "oom", "count": "x", "scope": "yes"}', "{"], ok),
        "weird_values": ([{"t": 1000 + 5 * i, "scope": {"gpu_memory_gb": "9", "memory_gb": None, "processes": [1],
                                                        "disk_gb": {}}} for i in range(30)], [], ok),
        "summary_list": (samples(), [], []),
        "enforce_str0": (samples(), [], dict(ok, enforce="0")),
        "enforce_false": (samples(), [], dict(ok, enforce=False)),
        "enforce_null": (samples(), [], dict(ok, enforce=None)),
        # a benchmark-owned service OOM-killed: the one case that sets infra_error (when there is no success)
        "service_oom_cap": (samples(1), [dict(oom, scope=False, role="sidecar", cause="cap")], ok),
        "service_oom_host": (samples(1), [dict(oom, scope=False, role="sidecar", cause="host")] * 3, ok),
        "service_oom_norecordsummary": (samples(1), [dict(oom, scope=False, role="sidecar")], None),
    }
    n = bad = 0
    for (rn, rj), (cn, rec), sw, txt in itertools.product(rewards.items(), records.items(), (None, "0", "false", ""),
                                                          ("1\n", "0\n", None)):
        n += 1
        rc, rj2, rt2 = _step_in_process(rj, txt, rec, sw)
        problems = []
        if rc != 0:
            problems.append(f"rc={rc}")
        if rt2 != txt:
            problems.append(f"reward.txt {txt!r} -> {rt2!r}")
        try:
            before = json.loads(rj) if rj else None
        except ValueError:
            before = None
        try:
            after = json.loads(rj2) if rj2 else None
        except ValueError:
            after = "INVALID"
        if isinstance(before, dict):
            if not isinstance(after, dict):
                problems.append(f"reward.json broke: {rj2!r}")
            else:
                service_oom = cn.startswith("service_oom") and not before.get("success")
                for k in ("success", "infra_error", "reward"):
                    want = 1 if (k == "infra_error" and service_oom) else before.get(k)
                    if after.get(k) != want and not (k == "success" and k not in before and after.get(k) == 0):
                        problems.append(f"{k}: {before.get(k)} -> {after.get(k)}")
                if after.get("resource_service_oom", 0) != int(service_oom):
                    problems.append(f"resource_service_oom = {after.get('resource_service_oom')}")
                nonnum = [k for k, v in after.items() if k not in before and (isinstance(v, bool) or not isinstance(v, (int, float)))]
                if nonnum:
                    problems.append(f"non-numeric fields added: {nonnum}")
        elif rj2 != rj:
            problems.append(f"reward.json created or changed from a non-object: {rj!r} -> {rj2!r}")
        if problems:
            bad += 1
            print("NOT NEUTRAL", rn, cn, repr(sw), repr(txt), problems)
    check(n == len(rewards) * len(records) * 12 and n >= 1320, f"the battery ran ({n} combinations)")
    check(bad == 0, f"{bad} of {n} combinations changed a verdict with enforcement off")


def test_clock_and_beacon():
    """usage.json carries the agent's clock (from the monitor's start, never more time than the agent has),
    after a failed tick too, and `monitor.py beacon` writes it with `"measured": false` where no monitor can run."""
    c = monitor.clock(1000.0, 400.0, 3600)
    check(c["elapsed_s"] == 600.0 and c["time_left_s"] == 3000.0 and c["time_limit_s"] == 3600, f"clock: {c}")
    check(c["deadline_utc"] == "1970-01-01T01:06:40Z", f"deadline: {c}")
    check(monitor.clock(5000.0, 400.0, 3600)["time_left_s"] == 0.0, "time left never negative")
    check("time_left_s" not in monitor.clock(1000.0, 400.0, 0), "no time limit: elapsed only")
    beacon_dir = tempfile.mkdtemp(prefix="rcb-beacon-test-")
    real = (monitor.SCOPE_BEACON, monitor.LIMITS, monitor.REC)
    try:
        monitor.SCOPE_BEACON, monitor.REC = beacon_dir, os.path.join(beacon_dir, "rec")
        monitor.LIMITS = os.path.join(beacon_dir, "limits.json")
        with open(monitor.LIMITS, "w") as fh:
            json.dump({"format": rcb_limits.LIMITS_FORMAT, "disk_gb": 20.0, "tick_sec": 5, "agent_timeout_sec": 600,
                       "caps": {"main": {"cpus": 4.0, "memory_gb": 8.0, "pids": 8192, "disk_gb": 20.0}}}, fh)
        monitor.beacon(rounds=1)
        u = rcb_limits.read_json(os.path.join(beacon_dir, "usage.json"))
        check(u["measured"] is False and u["enforced"] == 0 and u["caps"]["memory_gb"] == 8.0 and u["caps"]["cpus"] == 4.0
              and u["caps"]["gpu_memory_gb"] is None and u["limits"]["gpu_memory_gb"] is None
              and u["limits"]["disk_gb"] == 20.0 and u["limits"]["tolerance"] == rcb_limits.TOLERANCE,
              f"beacon: the agent's caps and the enforceable ones: {u}")
        check(u["time_limit_s"] == 600 and 590 <= u["time_left_s"] <= 600, f"beacon clock: {u}")
        started = rcb_limits.read_json(os.path.join(monitor.REC, "beacon.json"))["started"]
        monitor.beacon(rounds=1)                       # a restarted beacon keeps the first start
        check(rcb_limits.read_json(os.path.join(monitor.REC, "beacon.json"))["started"] == started, "beacon restart")
        m = Fake().monitor({"caps": {"main": {"memory_gb": 8.0}}, "tick_sec": 5, "agent_timeout_sec": 600})
        m.clock_t0 = monitor.now() - 100
        m.write_usage(monitor.now())                   # before any sample (or after a failed tick): the clock still runs
        u = rcb_limits.read_json(os.path.join(beacon_dir, "usage.json"))
        check(u["measured"] is False and u["memory_gb"] is None and 495 <= u["time_left_s"] <= 500, f"usage: {u}")
        m._usage_vals = {"gpu_memory_gb": 0.0, "memory_gb": 1.5, "processes": 3, "disk_gb": None}
        m.write_usage(monitor.now())
        u = rcb_limits.read_json(os.path.join(beacon_dir, "usage.json"))
        check(u["measured"] is True and u["memory_gb"] == 1.5 and u["processes"] == 3, f"usage: {u}")
    finally:
        monitor.SCOPE_BEACON, monitor.LIMITS, monitor.REC = real
        shutil.rmtree(beacon_dir, ignore_errors=True)


def test_session_log_copy():
    """The monitor copies the agent's session log append-only; the verifier gives the grader that copy when the
    agent's file is gone or rewritten, and the agent's own file when it is what was written (plus a last tick)."""
    d = tempfile.mkdtemp(prefix="rcb-slog-test-")
    real_rec = monitor.REC
    try:
        rec, root = os.path.join(d, "rec"), os.path.join(d, "root")
        os.makedirs(rec)
        os.makedirs(os.path.join(root, "app", "output"))
        monitor.REC = rec
        log = os.path.join(root, "app", "output", "rcb_session_log.jsonl")
        c = monitor.SessionLogCopy()
        c.step(root)
        check(not os.path.exists(os.path.join(rec, "session_log")), "no log yet: nothing copied")
        with open(log, "w") as f:
            f.write('{"e":"session","n":1}\n')
        c.step(root)
        with open(log, "a") as f:
            f.write('{"e":"reset","n":2}\n')
        c.step(root)
        out = os.path.join(d, "out")
        os.makedirs(out)
        vlogs = os.path.join(d, "vlogs")
        os.environ["RCB_VERIFIER_LOGS"] = vlogs
        agent = os.path.join(out, "rcb_session_log.jsonl")
        with open(agent, "w") as f:                    # the agent's file: what was written, and one line more
            f.write('{"e":"session","n":1}\n{"e":"reset","n":2}\n{"e":"step","n":3}\n')
        check(rcb_limits.session_log_source(agent, record=rec) is None, "an untouched log stays the grader's input")
        st = rcb_limits.read_json(os.path.join(vlogs, "session_log.json"))
        check(st["tampered"] == 0 and st["from_monitor"] == 0, f"status: {st}")
        with open(log, "w") as f:                      # the agent rewrites it: one line left, one forged
            f.write('{"e":"session","n":1}\n{"e":"forged"}\n')
        c.step(root)
        os.unlink(log)
        c.step(root)
        ev = [e["event"] for e in rcb_limits.read_jsonl(os.path.join(rec, "events.jsonl"))]
        check("session_log_rewritten" in ev and "session_log_removed" in ev, f"events: {ev}")
        os.unlink(agent)
        msg = rcb_limits.session_log_source(agent, record=rec)
        got = open(agent).read().splitlines()
        check(msg and got == ['{"e":"session","n":1}', '{"e":"reset","n":2}', '{"e":"forged"}'], f"merged: {got}")
        st = rcb_limits.read_json(os.path.join(vlogs, "session_log.json"))
        check(st["tampered"] == 1 and st["from_monitor"] == 1, f"status: {st}")
        os.symlink("/etc/hostname", log)               # a link is never followed (it would resolve in the monitor)
        c2 = monitor.SessionLogCopy()
        c2.step(root)
        check(c2.s["ino"] is None, "a symbolic link is not copied")
    finally:
        monitor.REC = real_rec
        os.environ.pop("RCB_VERIFIER_LOGS", None)
        shutil.rmtree(d, ignore_errors=True)


def test_stop_without_cgroup():
    """A container without a cgroup of its own (rootless podman, the user's cgroup not delegated) shares its parent's
    (conmon's), and its runtime cannot stop it (`pid: host`: podman signals the cgroup's processes, which are none):
    the monitor then ends itself after the seal, once the seal hook has read the summary (seal.done), or after the wait."""
    d = tempfile.mkdtemp(prefix="rcb-stop-test-")
    real = (monitor.PROC, monitor.REC)
    try:
        monitor.PROC, monitor.REC = os.path.join(d, "proc"), os.path.join(d, "rec")
        os.makedirs(monitor.REC)
        for who in ("self", str(os.getppid())):
            os.makedirs(os.path.join(monitor.PROC, who))

        def cg(who, text):
            with open(os.path.join(monitor.PROC, who, "cgroup"), "w") as f:
                f.write(text)
        cg("self", "0::/user/libpod-1234\n")
        cg(str(os.getppid()), "0::/user/session\n")
        check(monitor.own_cgroup(), "a cgroup of its own (docker, podman with a delegated cgroup)")
        cg("self", "0::/user/session\n")
        check(not monitor.own_cgroup(), "the parent's cgroup: none of its own")
        os.unlink(os.path.join(monitor.PROC, "self", "cgroup"))
        check(monitor.own_cgroup(), "unreadable: the runtime's stop is trusted")
        m = Fake().monitor({"caps": {"main": {"memory_gb": 8.0}}, "tick_sec": 5})
        t = monitor.now()
        m.linger(0.6)
        check(0.5 <= monitor.now() - t < 5, "no seal hook: it ends after the wait")
        with open(os.path.join(monitor.REC, "summary.json"), "w") as f:
            json.dump({"sealed": True, "n_samples": 1}, f)
        check(monitor.seal_cmd(1) == 0 and os.path.exists(os.path.join(monitor.REC, "seal.done")),
              "the seal hook marks that it has read the summary")
        t = monitor.now()
        m.linger(30)
        check(monitor.now() - t < 2, "after the seal hook: it ends at once")
    finally:
        monitor.PROC, monitor.REC = real
        shutil.rmtree(d, ignore_errors=True)


def test_throttle_restart_and_ownership():
    """The signal throttle after review 6: a change of the throttled set is in state.json before its new process is
    first stopped; a restarted monitor resumes every stopped process of that container whose stop is not the agent's
    own (fail open); a process the agent stops during a run window is never resumed by the monitor; without pidfds the
    throttle still signals (checked again before every signal) and says so once; a share under one run window runs
    k times its share every k-th period; only periods its stops reached count as throttled."""
    import errno
    import signal
    f = Fake()
    keep_open = monitor.open_pidfd
    try:
        root = _agent_container(f)
        m = f.monitor(LIM, enforce="1")
        groups = m.groups()
        g = next(iter(groups.values()))
        mnt = g["mnt"]

        def nopidfd(pid):
            raise OSError(errno.ENOSYS, "Function not implemented")
        monitor.open_pidfd = nopidfd
        vt = [0.0]

        def vsleep(dt):
            vt[0] += dt
        with Kills() as k:
            m.throttle.engage(mnt, g, 2.0, 4.0, 100.0)
            e = m.throttle.active[mnt]
            check(set(e["procs"]) == {101, 102, 104} and len(f.events("backstop_error")) == 1,
                  f"no pidfd: the processes are still throttled, said once: {e['procs']} {f.events('backstop_error')}")
            m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
            check(sorted(p for p, sg in k.calls if sg == signal.SIGSTOP) == [101, 102, 104]
                  and e["landed"] > 0, f"signals by PID land, counted: {k.calls} {e['landed']}")
            # a new process joins: state.json has it before the monitor first stops it
            f.pid(105, mnt, root, start=7000, ppid=101, nspid=12)
            st0 = []
            real_save = m.save_state

            def save():
                real_save()
                st0.append([p for p, _ in (rcb_limits.read_json(os.path.join(f.rec, "state.json")) or {})["throttle"][0]["procs"]])
            m.save_state = save
            n0 = len(k.calls)
            e["refreshed"] = 0.0
            m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
            first_stop = next(i for i, (p, sg) in enumerate(k.calls[n0:]) if p == 105 and sg == signal.SIGSTOP)
            check(st0 and 105 in st0[0] and first_stop >= 0, f"saved before 105's first stop: {st0}")
            m.save_state = real_save
            # the agent stops 102 during a run window: never resumed by the monitor again
            n0 = len(k.calls)
            m.throttle._signal(e, signal.SIGCONT)
            f.pid(102, mnt, root, start=5001, ppid=101, nspid=10, state="T")
            m.throttle._signal(e, signal.SIGSTOP)
            check(102 not in e["procs"] and (102, 5001) in e["foreign"] and
                  all(p != 102 for p, _ in k.calls[n0 + 4:]), f"the agent's own stop is left alone: {k.calls[n0:]}")
            m.throttle._signal(e, signal.SIGCONT)
            check(all(p != 102 for p, _ in k.calls[n0 + 4:]), "and never resumed")
            m.throttle._signal(e, signal.SIGSTOP)
            st = rcb_limits.read_json(os.path.join(f.rec, "state.json"))["throttle"][0]
            check([102, 5001] in st["not_ours"] and st["mnt"] == mnt, f"state.json knows it is not ours: {st}")
            # the monitor dies (SIGKILL): a restart resumes what it may have stopped, not the agent's own stop
            f.pid(101, mnt, root, start=5000, ppid=77, nspid=9, state="T")
            f.pid(104, mnt, root, start=6000, ppid=100, nspid=11, state="T")
            f.pid(106, mnt, root, start=7100, ppid=101, nspid=13, state="T")   # stopped after the last save
            st["procs"] = [[101, 5000]]
            monitor.write_json(os.path.join(f.rec, "state.json"), {"throttle": [st]})
        m2 = f.monitor(LIM, enforce="1")
        with Kills() as k:
            m2.restore()
        check(sorted(p for p, sg in k.calls if sg == signal.SIGCONT) == [101, 104, 106] and all(p != 102 for p, _ in k.calls),
              f"restart: every stopped process the monitor may have stopped, never the agent's: {k.calls}")
        for p, start, pp, nsp in ((101, 5000, 77, 9), (102, 5001, 101, 10), (104, 6000, 100, 11)):
            f.pid(p, mnt, root, start=start, ppid=pp, nspid=nsp)
        f.gone(105)
        f.gone(106)
        # a share under one run window: k times its share, every k-th period (0.015: 3 ms every 2nd period)
        m3 = f.monitor(LIM, enforce="1")
        g3 = next(iter(m3.groups().values()))
        with Kills() as k:
            m3.throttle.engage(mnt, g3, 2.0, 2.0 / 0.015, 100.0)
            e3 = m3.throttle.active[mnt]
            runs = []
            for _ in range(4):
                slept = []
                vt3 = [0.0]

                def vs(dt):
                    slept.append(dt)
                    vt3[0] += dt
                m3.throttle.cycle(sleep=vs, clock=lambda: vt3[0])
                runs.append(round(slept[0], 4) if len(slept) == 2 else None)
            check(runs == [None, 0.003, None, 0.003], f"the share asked for: {runs}")
            m3.throttle.release_all("test")
            check(f.events("throttle_end")[-1]["sec"] == 0.4, f"held every period: {f.events('throttle_end')[-1]}")
    finally:
        monitor.open_pidfd = keep_open
        f.close()


def test_review6_minor():
    """pids over the cap that killing cannot cover (zombies nobody reaps): nothing is killed; a process in the
    container's cgroup that is not of it (conmon in a shared scope) is never merged in; NVML's PIDs outside the host's
    PID namespace are not matched; cgroup v1: the period the monitor wrote is put back."""
    f = Fake()
    try:
        _agent_container(f)
        for z in range(30, 36):                                  # 6 zombies of PID 1 (sleep infinity never reaps)
            f.pid(z + 200, None, None, start=8000 + z, state="Z", ppid=100)
            f.thread(z + 200, z + 200, None, None, start=8000 + z)
        m = f.monitor(LIM, enforce="1")
        with Kills() as k:
            _one(m)
            _, ms = _one(m, 105.0)
        check(ms[0]["pids"] == 11 and k.calls == [] and [e["resource"] for e in f.events("no_victim")] == ["pids"],
              f"6 zombies + 5 of a cap of 5: killing every agent process cannot help: {ms[0]} {k.calls}")
    finally:
        f.close()
    f = Fake()
    try:
        _agent_container(f)
        name = "machine.slice/libpod-conmon-" + "ab" * 32 + ".scope"
        f.pid(180, "mnt:[888]", f.host, start=900, ppid=1, nspid=180)         # conmon: in the scope, not a descendant
        f.cgroup(name, pids=[100, 101, 102, 103, 104, 180], anon_gb=0.1)
        m = f.monitor(LIM, enforce="1")
        g = next(iter(m.groups().values()))
        check(180 not in g["pids"] and not m.in_trial(180) and not m.trial_cg,
              f"conmon is not the container's, the shared cgroup not the trial's: {g['pids']} {m.trial_cg}")
        # NVML's PIDs: the host's; matched only when the monitor's /proc is the host's PID namespace
        class FakeNvml:
            handles = []

            def sample(self):
                return {101: 6000}, {}, set()
        m.nvml = FakeNvml()
        with Kills() as k:
            m.pidns_host = False
            m.tick_once(100.0)
            check(m._last_measures[0]["gpu_memory_gb"] == 0 and
                  [e["resource"] for e in f.events("unobservable")] == ["gpu_memory"] and k.calls == [],
                  f"another PID namespace: not matched, nothing killed: {m._last_measures[0]}")
            m.pidns_host = True
            m.tick_once(105.0)
            check(m._last_measures[0]["gpu_memory_gb"] > 5.8, "the host's: matched")
        # cgroup v1: the quota and the period it had come back
        d = os.path.join(f.d, "v1cpu")
        os.makedirs(d)
        _cg_files(d, **{"cpu_cfs_quota_us": "-1", "cpu_cfs_period_us": "50000"})
        e = {"cg": d, "method": "cgroup"}
        r = m.throttle._cgroup_quota(e, 1.5)
        check(open(d + "/cpu.cfs_quota_us").read() == "150000" and open(d + "/cpu.cfs_period_us").read() == "100000",
              "v1: the cap written")
        e.update(file=r[0], orig=r[1], pfile=r[2], porig=r[3])
        m.throttle._let_go(e)
        check(open(d + "/cpu.cfs_quota_us").read() == "-1" and open(d + "/cpu.cfs_period_us").read() == "50000",
              "v1: the quota and the period put back")
    finally:
        f.close()


class Clock:
    """monitor.now() moved by hand."""

    def __init__(self, t=1000.0):
        self.t, self.real = t, monitor.now

    def __enter__(self):
        monitor.now = lambda: self.t
        return self

    def __exit__(self, *a):
        monitor.now = self.real


def test_pids_zombies_for_good():
    """The backstop's own victims stay discounted for as long as they are zombies (a parent that never reaps keeps
    them for good: no ratchet after any time); a reaped victim's PID taken by a new process is that process's."""
    f = Fake()
    try:
        _agent_container(f, {101: {"threads": 3}})            # 7 of 5
        m = f.monitor(LIM, enforce="1")
        with Clock() as c, Kills() as k:
            _one(m)
            c.t += 5
            _one(m, 105.0)
            check(k.killed() == [104, 102], f"the first kills: {k.killed()}")
            for p, start in ((104, 6000), (102, 5001)):
                f.pid(p, None, None, start=start, state="Z", ppid=101)
                f.thread(p, p, None, None, start=start)
            for i in range(4):                                  # 0 s, 700 s, 1400 s, 2100 s later
                c.t += 700
                _, ms = _one(m, c.t)
                c.t += 5
                _, ms = _one(m, c.t)
            check(len(k.killed()) == 2 and ms[0]["zombies"] == 2, f"no second kill after 2100 s: {k.killed()} {ms[0]}")
            # 104 is reaped and its PID taken by a new process: not discounted any more (102 still is)
            f.pid(104, "mnt:[111]", f.root("main"), start=9000, ppid=101, nspid=12)
            _one(m, c.t + 5)
            check(set(m.recent) == {102}, f"only the zombie with its start time is the kill's: {m.recent}")
        # a victim that has not died yet (SIGKILL sent, exit under way) is neither counted nor killed again
        f2 = Fake()
        try:
            _agent_container(f2, {101: {"threads": 3}})
            m2 = f2.monitor(LIM, enforce="1")
            with Kills() as k2:
                _one(m2)
                _one(m2, 105.0)
                _one(m2, 110.0)
                _one(m2, 115.0)
            check(k2.killed() == [104, 102], f"killed, still there (Kills sends nothing): never again: {k2.killed()}")
        finally:
            f2.close()
    finally:
        f.close()


def _hb_gaps(conts):
    """The longest run of periods without a SIGCONT, in periods."""
    gap = best = 0
    for c in conts:
        gap = 0 if c else gap + 1
        best = max(best, gap)
    return best + 1


def test_throttle_churn():
    """Processes that start while a container is throttled are stopped in the same period (they are looked for twice
    in every period, between the full refreshes), with a cgroup (its process list) and without one (new PIDs of
    /proc); the ones gone are dropped."""
    import signal
    for with_cg in (True, False):
        f = Fake()
        try:
            _agent_container(f)
            if with_cg:
                f.cgroup("docker-" + "cd" * 32 + ".scope", pids=[100, 101, 102, 103, 104], anon_gb=0.1)
            m = f.monitor(LIM, enforce="1")
            groups = m.groups()
            g = next(iter(groups.values()))
            mnt = g["mnt"]
            x = m.measure(g, {}, 100.0)
            check(bool(g.get("cg")) == with_cg, f"cgroup: {g.get('cg')}")
            x["cpu"] = 8.0                                      # share 0.25
            vt = [0.0]

            def vsleep(dt):
                vt[0] += dt
            with Kills() as k:
                m.backstop(g, x, 100.0)
                e = m.throttle.active[mnt]
                if with_cg:
                    e["method"] = "signal"                      # the cgroup's quota not writable: signals
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                check(set(e["procs"]) == {101, 102, 104}, f"the set: {e['procs']}")
                # a compiler job starts: in the same period it is stopped with the rest, not a second later
                f.pid(105, "mnt:[111]", f.root("main"), start=7000, ppid=102, nspid=13)
                if with_cg:
                    f.cgroup("docker-" + "cd" * 32 + ".scope", pids=[100, 101, 102, 103, 104, 105])
                del k.calls[:]
                e["refreshed"] = monitor.now()                  # well inside THROTTLE_REFRESH_SEC
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                check((105, signal.SIGSTOP) in k.calls and 105 in e["procs"],
                      f"with_cg={with_cg}: the new process stopped in its first period: {k.calls}")
                # it ends: dropped at the next look, without a signal
                f.gone(105)
                if with_cg:
                    f.cgroup("docker-" + "cd" * 32 + ".scope", pids=[100, 101, 102, 103, 104])
                del k.calls[:]
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                check(105 not in e["procs"] and all(p != 105 for p, _ in k.calls), f"dropped: {k.calls}")
                # another trial's process (another root, no beacon) is never added
                f.pid(106, "mnt:[111]", f.host, start=7100, ppid=1, nspid=106)
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                check(106 not in e["procs"] and all(p != 106 for p, _ in k.calls), "not the trial's: never signalled")
                m.throttle.release_all("test")
        finally:
            f.close()


def test_throttle_controller():
    """The share under churn: what new processes used before they were first seen ran free (escape) and is taken off
    the cap; the rest is the throttled processes' demand. The share never goes under the floor, so a long-lived
    process (a heartbeat) is never held for more than 0.4 s; when the escape alone is over the cap,
    `throttle_insufficient` once, and no release while the container still uses more than the cap."""
    import signal
    f = Fake()
    try:
        _agent_container(f)
        m = f.monitor(LIM, enforce="1")
        groups = m.groups()
        g = next(iter(groups.values()))
        mnt = g["mnt"]
        x = m.measure(g, {}, 100.0)
        check(monitor.Throttler.floor(0.25, 1) == 0.25 and monitor.Throttler.floor(0.25, 64) == 0.005 and
              monitor.Throttler.floor(2.0, 16) == 0.125, "the floor: the cap over the CPUs, at least 0.005")
        cap, HZ = 2.0, monitor.HZ
        with Clock(1000.0) as c, Kills() as k:
            x["cpu"] = 8.0
            m.backstop(g, x, 100.0)
            e = m.throttle.active[mnt]
            check(abs(e["frac"] - 0.25) < 1e-9 and e["floor"] == monitor.THROTTLE_MIN_FRACTION, f"engaged: {e['frac']}")

            def tick(escape):                                   # 5 s: the set wants 8 CPUs, new processes ran free
                c.t += 5                                        # for `escape` CPUs before they were seen
                e["escape_ticks"] = int(escape * 5 * HZ)
                x["cpu"] = 8.0 * e["frac"] + escape
                m.backstop(g, x, c.t)
            for _ in range(6):
                tick(0.5)
            held = 8.0 * e["frac"] + 0.5
            check(m.throttle.throttled(mnt) and abs(held - cap) < 0.02, f"the cap is held: {held} {e['frac']}")
            check(not f.events("throttle_insufficient"), "the floor not reached")
            for _ in range(5):                                  # the churn alone uses more than the cap
                tick(3.0)
            check(m.throttle.throttled(mnt) and e["frac"] == e["floor"], f"at the floor: {e['frac']}")
            ev = f.events("throttle_insufficient")
            check(len(ev) == 1 and ev[0]["escape"] == 3.0 and ev[0]["cap"] == cap, f"said once: {ev}")
            # the set idle (or frozen), the churn still over the cap: not "demand 0.0", still throttled
            c.t += 5
            e["escape_ticks"] = int(2.5 * 5 * HZ)
            x["cpu"] = 2.5
            m.backstop(g, x, c.t)
            check(m.throttle.throttled(mnt), "the escape alone over the cap: no release")
            # at the floor a long-lived process runs at least every 0.4 s
            vt = [0.0]

            def vsleep(dt):
                vt[0] += dt
            conts = []
            for _ in range(24):
                n0 = len(k.calls)
                m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
                conts.append(any(p == 101 and sg == signal.SIGCONT for p, sg in k.calls[n0:]))
            gap = _hb_gaps(conts) * m.throttle.period
            check(gap <= 0.4 + 1e-9, f"the longest hold of a process: {gap} s")
            # the demand falls: released, with the record saying the cap was not held
            c.t += 5
            e["escape_ticks"] = 0
            x["cpu"] = 8.0 * e["frac"] * 0.1
            m.backstop(g, x, c.t)
            check(not m.throttle.throttled(mnt), "released")
            check(f.events("throttle_end")[-1].get("insufficient") is True, f"{f.events('throttle_end')}")
        # a process that joins the set brings what it used before it was seen into the escape
        f.pid(105, "mnt:[111]", f.root("main"), start=7000, ppid=102, nspid=13, cpu_ticks=3 * HZ)
        with Kills():
            x["cpu"] = 8.0
            m.backstop(g, x, 200.0)
            e = m.throttle.active[mnt]
            m.throttle._catch_up(mnt, e)
            check(105 in e["procs"] and e["escape_ticks"] == 3 * HZ, f"escape: {e.get('escape_ticks')}")
            m.throttle.release_all("test")
    finally:
        f.close()


def test_emergency_never_blocks():
    """SIGTERM while another thread holds the state lock and the throttle's lock: the handler lets every process the
    monitor stopped go at once, writes no state.json, and no SIGSTOP goes out after it."""
    import signal
    import threading
    f = Fake()
    try:
        _agent_container(f)
        m = f.monitor(LIM, enforce="1")
        groups = m.groups()
        g = next(iter(groups.values()))
        mnt = g["mnt"]
        x = m.measure(g, {}, 100.0)
        x["cpu"] = 8.0
        vt = [0.0]

        def vsleep(dt):
            vt[0] += dt
        with Kills() as k:
            m.backstop(g, x, 100.0)
            m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
            f.pid(102, "mnt:[111]", f.root("main"), start=5001, ppid=101, nspid=10, state="T")   # 102: T, ours
            f.pid(101, "mnt:[111]", f.root("main"), start=5000, ppid=77, nspid=9, state="T")
            m.throttle.active[mnt]["ours"].pop(101)        # 101 stopped by the agent since: dropped as not ours
            hold, done = threading.Event(), threading.Event()

            def holder():
                with m.state_lock, m.throttle.lock:
                    hold.set()
                    done.wait(10)
            th = threading.Thread(target=holder, daemon=True)
            th.start()
            hold.wait(5)
            os.remove(os.path.join(f.rec, "state.json")) if os.path.exists(os.path.join(f.rec, "state.json")) else None
            del k.calls[:]
            t0 = monitor.now()
            em = threading.Thread(target=m.throttle.emergency, daemon=True)
            em.start()
            em.join(5)
            check(not em.is_alive() and monitor.now() - t0 < 2, "never waits for a lock")
            done.set()
            th.join(5)
            check({p for p, sg in k.calls if sg == signal.SIGCONT} == {102, 104} and
                  not os.path.exists(os.path.join(f.rec, "state.json")),
                  f"what it stopped is let go, no state written: {k.calls}")
            del k.calls[:]
            m.throttle.cycle(sleep=vsleep, clock=lambda: vt[0])
            check(all(sg != signal.SIGSTOP for _, sg in k.calls), f"no SIGSTOP after it: {k.calls}")
    finally:
        f.close()


def test_cumulative_cpu():
    """No cgroup: a process that starts and ends between two samples is counted (through its parent's count of reaped
    children, or through all it used when it started after the last sample), and never twice."""
    f = Fake()
    try:
        with open(os.path.join(f.proc, "stat"), "w") as fh:
            fh.write("cpu 0 0 0 0\nbtime 0\n")
        _agent_container(f)
        m = f.monitor(LIM, enforce="0")
        HZ = monitor.HZ
        with Clock(1000.0) as c:
            _, ms = _one(m, 100.0)
            check(ms[0]["cpu"] == 0, "the first sample: nothing to compare with")
            # 10 s: 102 used 2 s of its own; a make job (child of 102) ran 5 s and was reaped by 102
            c.t += 10
            f.pid(102, "mnt:[111]", f.root("main"), start=5001, ppid=101, nspid=10, cpu_ticks=10 + 2 * HZ,
                  child_ticks=5 * HZ)
            _, ms = _one(m, 110.0)
            check(abs(ms[0]["cpu"] - 0.7) < 0.011, f"2 + 5 s over 10 s: {ms[0]['cpu']}")
            # 10 s: a new process, started after the last sample, used 3 s and is still there
            c.t += 10
            f.pid(105, "mnt:[111]", f.root("main"), start=int(1015 * HZ), ppid=102, nspid=13, cpu_ticks=3 * HZ)
            _, ms = _one(m, 120.0)
            check(abs(ms[0]["cpu"] - 0.3) < 0.011, f"the new one's 3 s: {ms[0]['cpu']}")
            # 10 s: 105 ends after 1 s more; 102 reaps it (its count rises by all 4 s): only the 1 s is new
            c.t += 10
            f.gone(105)
            f.pid(102, "mnt:[111]", f.root("main"), start=5001, ppid=101, nspid=10, cpu_ticks=10 + 2 * HZ,
                  child_ticks=9 * HZ)
            _, ms = _one(m, 130.0)
            check(abs(ms[0]["cpu"] - 0.1) < 0.011, f"not counted twice: {ms[0]['cpu']}")
            # an old process seen for the first time (it started long before) is not counted with all it used
            c.t += 10
            f.pid(106, "mnt:[111]", f.root("main"), start=10, ppid=100, nspid=14, cpu_ticks=500 * HZ)
            _, ms = _one(m, 140.0)
            check(ms[0]["cpu"] == 0, f"started before the last sample: {ms[0]['cpu']}")
    finally:
        f.close()


def test_write_errors():
    """A record write that fails is counted (summary.json `write_errors`), never raised; the fd limit is raised."""
    soft0 = None
    try:
        import resource
        soft0 = resource.getrlimit(resource.RLIMIT_NOFILE)
    except ImportError:
        pass
    n = monitor.raise_nofile()
    if soft0:
        check(n is None or n >= soft0[0], f"the soft limit raised: {soft0} -> {n}")
        import resource
        resource.setrlimit(resource.RLIMIT_NOFILE, soft0)
    f = Fake()
    try:
        m = f.monitor(LIM, enforce="1")
        before = dict(monitor.IO_ERRORS["write"])
        monitor.REC = os.path.join(f.d, "missing")
        monitor.event("x")
        monitor.append("samples.jsonl", {})
        m.save_state()
        w = monitor.IO_ERRORS["write"]
        check(w.get("events.jsonl", 0) == before.get("events.jsonl", 0) + 1 and
              w.get("samples.jsonl", 0) == before.get("samples.jsonl", 0) + 1 and
              w.get("state.json", 0) == before.get("state.json", 0) + 1, f"counted: {w}")
        monitor.REC = f.rec
        _write_meta(f)
        m.seal()
        s = rcb_limits.read_json(os.path.join(f.rec, "summary.json"))
        check(s["write_errors"].get("events.jsonl", 0) >= 1 and "fd_exhausted" in s, f"in the summary: {s}")
    finally:
        f.close()


def _write_meta(f):
    with open(os.path.join(f.rec, "meta.json"), "w") as fh:
        json.dump({"started": 0}, fh)


def test_real_pidfd():
    """The shipped path, on real processes and this kernel's /proc: pidfds held while throttled, SIGSTOP / SIGCONT
    through them, a process that dies is skipped, a new one is stopped in its first period; beyond the fd budget a
    process is signalled by PID; every fd is closed when the throttle ends."""
    import signal
    import time
    if not hasattr(os, "pidfd_open") or not os.path.isdir("/proc/self"):
        print("test_real_pidfd: skipped (no os.pidfd_open here)")
        return
    try:
        os.close(os.pidfd_open(os.getpid()))
    except OSError:
        print("test_real_pidfd: skipped (pidfd_open refused here)")
        return
    saved = monitor.PROC, monitor.REC
    rec = tempfile.mkdtemp(prefix="rcb-monitor-real-")
    kids = []

    def state(p):
        return monitor.state_of(f"/proc/{p}")

    def wait_state(p, want, sec=2.0):
        t = time.time()
        while time.time() - t < sec:
            if state(p) == want:
                return True
            time.sleep(0.005)
        return False
    try:
        monitor.PROC, monitor.REC = "/proc", rec
        kids = [subprocess.Popen(["sleep", "60"]) for _ in range(3)]
        ids = {k.pid for k in kids}

        class Mon:                                              # the trial: these children only
            threads = False
            members = set(ids)

            def in_trial(self, p):
                return p in self.members

            def live_thread(self, p):
                return None

            def spared(self, pids):
                return set()

            def save_state(self):
                pass
        mon = Mon()
        th = monitor.Throttler(mon)
        mnt = os.readlink("/proc/self/ns/mnt")
        g = {"pids": sorted(ids), "role": "main", "service": "main", "cg": None}
        th.engage(mnt, g, 0.25, 1.0, 0.0)
        e = th.active[mnt]
        check(set(e["procs"]) == ids and all(isinstance(v[1], int) and isinstance(v[2], int) for v in e["procs"].values()) and
              th.fds == 6,
              f"a pidfd per process: {e['procs']}")
        th.cycle()
        check(all(wait_state(p, "T") for p in ids), "stopped through the pidfds")
        # one dies while stopped: its pidfd says so, the others go on
        kids[0].kill()
        kids[0].wait()
        th.cycle()
        # a new process of the trial: stopped in its first period, through a pidfd of its own
        new = subprocess.Popen(["sleep", "60"])
        kids.append(new)
        mon.members.add(new.pid)
        e["refreshed"] = monitor.now()
        th.cycle()
        check(new.pid in e["procs"] and wait_state(new.pid, "T"), "a new process: stopped in its first period")
        # beyond the budget: by PID, checked before every signal
        th.fd_budget = th.fds + 1
        more = subprocess.Popen(["sleep", "60"])
        kids.append(more)
        mon.members.add(more.pid)
        th.cycle()
        check(e["procs"][more.pid][1] is None and e["procs"][more.pid][2] is None and wait_state(more.pid, "T"), "by PID beyond the budget")
        check(any("open-file limit" in (x.get("why") or "") for x in
                  rcb_limits.read_jsonl(os.path.join(rec, "events.jsonl")) if x.get("event") == "backstop_error"),
              "said once")
        th.release_all("test")
        live = [k.pid for k in kids[1:]]
        check(all(wait_state(p, "S") for p in live) and th.fds == 0, f"all let go, every fd closed: {th.fds}")
    finally:
        for k in kids:
            try:
                k.kill()
                k.wait(5)
            except Exception:
                pass
        monitor.PROC, monitor.REC = saved
        shutil.rmtree(rec, ignore_errors=True)


def main():
    for t in (test_classify_race, test_dead_leader_and_zombies, test_disk_walk, test_disk_baseline, test_backstop_memory,
              test_backstop_pids, test_backstop_cpu, test_backstop_gpu, test_backstop_disk, test_runtime_first,
              test_throttle_signals, test_cgroup_container, test_runtime_while_starting, test_bad_tolerance, test_selftest_fixtures, test_backstop_services,
              test_backstop_record, test_switch, test_verify_edges, test_oom_watch, test_oom_verdicts, test_sanitize, test_default_is_neutral,
              test_clock_and_beacon, test_session_log_copy, test_stop_without_cgroup,
              test_throttle_restart_and_ownership, test_review6_minor, test_pids_zombies_for_good, test_throttle_churn,
              test_throttle_controller, test_emergency_never_blocks, test_cumulative_cpu, test_write_errors,
              test_real_pidfd):
        t()
    shutil.rmtree(os.environ["RCB_MONITOR_RECORD"], ignore_errors=True)
    print("rcb_monitor test_monitor: ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
