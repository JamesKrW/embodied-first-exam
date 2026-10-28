#!/usr/bin/env python3
"""Resources (docs/protocol/README.md §6): the over-cap rule, shared by the monitor sidecar (monitor.py) and the verifier
step, and the verifier step itself.

Every resource number of a task is an upper bound for one container (limits.json `caps`, per compose service, format
rcb-limits/2, written by inject.py; any other format is an unusable record). A measurement is over its cap when it is
above cap x (1 + tolerance) (RCB_LIMITS_TOLERANCE, default TOLERANCE: sampling noise; process and thread counts are
compared exactly): CPU (`cpus`), memory (`memory_gb`), processes and threads (`pids`) and GPU memory (`gpu_memory_gb`)
of each container in the scope, disk (the growth of the files under DISK_PATHS in the agent's container, `disk_gb`).
A kernel OOM kill in the scope at the container's own memory cap counts too. Who acts on it is docs/protocol/README.md
§6: Docker / Harbor as far as the host lets them, and, with RCB_LIMITS_ENFORCE=1, the monitor as the backstop (it kills
or throttles like Docker would, monitor.py). Neither changes the grade: an agent whose process was killed ends like one
whose process died; this step only records.

The pinned monitor image carries it at /opt/rcb_monitor/ (inject.py IMAGE and CONTENTS), and every task's tests/Dockerfile
copies it from there to /tests/ (ci_checks/check-monitor.py). Python 3.8+, standard library only: it runs inside
benchmark verifier images.

    python3 /tests/rcb_limits.py verify      # test.sh runs it on exit (trap), after the grader
    python3 /tests/rcb_limits.py sanitize /app/output    # test.sh runs it before the grader (see sanitize())

The session log: sanitize() also compares the agent's /app/output/rcb_session_log.jsonl with
the monitor's append-only copy of it (session_log/ in the record) and, when the agent's file is missing or is not what
was written (removed, truncated, rewritten), gives the grader the copy instead (session_log_source()); verify() adds
session_log_tampered (0/1) and session_log_from_monitor (0/1) to reward.json. Only the graders' cost fields read the log.

The verifier step reads the monitor's sealed record (/var/rcb-monitor/, collected from the rcb_monitor sidecar, a
channel the agent cannot write), recomputes what was over a cap from the raw samples with its own copy of the caps
(/tests/rcb_limits.json), and adds to /logs/verifier/reward.json:
  always:   resource_enforced, limits_violation_observed (a cap exceeded at some sample, or a backstop action),
            resource_monitor_missing, peak_gpu_memory_gb, peak_memory_gb, peak_processes, peak_disk_gb, peak_cpus,
            peak_pids; over_<metric>=1 for every metric over its cap (gpu_memory_gb, memory_gb, pids, cpus, disk_gb,
            oom); the monitor's backstop actions: resource_memory_kills, resource_pids_kills, resource_disk_kills,
            resource_gpu_memory_kills (processes killed per resource), resource_monitor_kills (their sum; also under its
            0.4 name resource_emergency_kills), resource_cpu_throttles (times a container's CPU was duty-cycled) and
            resource_cpu_throttled_sec; the kernel's OOM kills: resource_oom_kills (in the scope, at the container's
            own memory cap), resource_host_oom_kills (in the scope, not at its cap: the host's OOM killer or an
            enclosing cgroup's), resource_service_oom_kills (in a benchmark-owned service that is only recorded: the
            simulator sidecar of standard modes, RoboPaint's scorer)
  enforced (the record's mode, RCB_LIMITS_ENFORCE=1) and no usable record: infra_error=1 (the backstop that was asked
            for is not shown to have run: not a result). Not enforced (the default): a missing or unusable record is
            only reported (resource_monitor_missing=1); the grader's verdict stands.
  enforced, and a process in the scope was killed by an OOM killer that is not its own cap's: infra_error=1,
            resource_host_oom=1 (not a result: another trial's or the host's doing, never the agent's)
  RCB_LIMITS_ENFORCE neither 0/1/true/false, or RCB_LIMITS_TOLERANCE not a number in [0, 1]: infra_error=1,
            resource_switch_error=1 (nobody knows what was asked for)
  in both modes: the kernel OOM-killed a benchmark-owned service and the grader's verdict is not a success:
            infra_error=1, resource_service_oom=1 (PROTOCOL §6: such a service is recorded, never a failure; an
            episode that ended without success after its simulator was killed for memory is not the agent's 0). A
            success stands: the service recovered and the grader accepted the episode.
Wherever this step says "not a result" (infra_error=1) it also sets success=0 and reward.txt 0 and keeps the grader's
verdict in success_before_limits, so that no reader sees a success next to an infrastructure error. A cap exceeded never
changes the grade by itself, in either setting.
The reasons, in words, go to /logs/verifier/resource_limits.json. If this step itself fails, it says so in reward.json
(resource_step_error=1, resource_monitor_missing=1; infra_error=1 when the switch asks for enforcement).
"""
import json
import os
import stat
import sys

FORMAT = "rcb-monitor/1"
LIMITS_FORMAT = "rcb-limits/2"   # limits.json / rcb_limits.json (inject.py FORMAT)
RECORD = os.environ.get("RCB_MONITOR_RECORD", "/var/rcb-monitor")
METRICS = ("gpu_memory_gb", "memory_gb", "processes", "disk_gb", "cpus", "pids")   # the scope's sums (peaks)
# a container's measurement (the monitor's sample, `containers`) -> its cap in limits.json `caps.<service>`
CAP_OF = {"cpus": "cpus", "memory_gb": "memory_gb", "pids": "pids", "gpu_memory_gb": "gpu_memory_gb"}
RESOURCES = ("memory", "pids", "cpu", "disk", "gpu_memory")   # what the backstop acts on (kill / throttle events)
TOLERANCE = 0.05             # RCB_LIMITS_TOLERANCE's default: over a cap = above cap x (1 + this); counts are exact
DISK_PATHS, DISK_EXCLUDE = ("/app", "/tmp", "/root"), ()   # what disk counts in the agent's container: new files here
MAX_GAP_FRACTION = 0.10      # more of the agent phase than this unobserved: the record is not usable


def parse_switch(value):
    """RCB_LIMITS_ENFORCE -> (1 or 0, None), or (0, why) for a value that is neither on nor off (never a silent off)."""
    s = str("" if value is None else value).strip().lower()
    if s in ("", "0", "false"):
        return 0, None
    if s in ("1", "true"):
        return 1, None
    return 0, f"RCB_LIMITS_ENFORCE={value!r} is not one of 0, 1, true, false"


def parse_tolerance(value):
    """RCB_LIMITS_TOLERANCE -> (fraction, None); unset or empty: TOLERANCE; not a number in [0, 1]: (TOLERANCE, why)."""
    s = str("" if value is None else value).strip()
    if not s:
        return TOLERANCE, None
    try:
        v = float(s)
    except ValueError:
        v = -1.0
    if not 0.0 <= v <= 1.0:      # NaN fails both comparisons
        return TOLERANCE, f"RCB_LIMITS_TOLERANCE={value!r} is not a number in [0, 1]"
    return v, None


def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool) and v == v


def over(value, cap, tol, exact=False):
    """Whether a measurement is over its cap: above cap x (1 + tol); a count (exact) above the cap itself. A cap that is
    not set (None, 0) or a value that is not a number is never over."""
    if not _num(value) or not _num(cap) or cap <= 0:
        return False
    return value > cap if exact else value > cap * (1.0 + tol)


def service_caps(limits, c):
    """The caps of the container `c` of a sample (its `service`; a record before 0.5 names only the role: main's)."""
    caps = (limits or {}).get("caps") or {}
    svc = c.get("service") or ("main" if c.get("role") == "main" else None)
    out = caps.get(svc) if isinstance(caps, dict) else None
    return svc, (out if isinstance(out, dict) else {})


def violations(samples, limits, events=(), tol=TOLERANCE):
    """Every cap exceeded in a record: [{metric, limit, at, peak, samples, services}] (empty: none), one entry per
    metric (at: the first sample over, peak: the largest value over, samples: how many). Per container of the scope
    (the samples' `containers`, kind "scope", against the caps of its service): memory_gb, pids (processes and threads:
    exact), cpus, gpu_memory_gb (only a service that holds a GPU has that cap); for the scope: disk_gb (the agent's disk
    against limits.json disk_gb) and gpu_memory_gb (the scope's sum against limits.json gpu_memory_gb, for a record
    without per-container values); and `oom`: a kernel OOM kill in the scope at the container's own memory cap (the
    monitor's `oom` event with cause "cap"). A kill by the host's OOM killer is not the trial's doing and never counts:
    oom_kills() counts it. A backstop action of the monitor (`kill`, `throttle` events) is evidence of the same kind:
    its resource counts as over even when no sample shows it (the monitor acted between the samples' writes)."""
    limits = limits if isinstance(limits, dict) else {}
    found = {}

    def add(metric, lim, t, v, svc):
        e = found.get(metric)
        if e is None:
            e = found[metric] = {"metric": metric, "limit": lim, "at": t, "peak": v, "samples": 0, "services": []}
        e["samples"] += 1
        if _num(v) and (not _num(e["peak"]) or v > e["peak"]):
            e["peak"] = v
        if svc and svc not in e["services"]:
            e["services"].append(svc)

    for s in samples:
        if not isinstance(s, dict):
            continue
        t = s.get("t") if _num(s.get("t")) else None
        sc = s.get("scope") if isinstance(s.get("scope"), dict) else {}
        if over(sc.get("disk_gb"), limits.get("disk_gb"), tol):
            add("disk_gb", limits.get("disk_gb"), t, sc.get("disk_gb"), "main")
        cs = [c for c in (s.get("containers") or []) if isinstance(c, dict) and c.get("kind") == "scope"]
        gpu_seen = False
        for c in cs:
            svc, caps = service_caps(limits, c)
            for m, k in CAP_OF.items():
                if m == "gpu_memory_gb" and not caps.get("gpu"):
                    continue                          # no GPU in this service: no GPU-memory cap (recorded only)
                v = c.get(m)
                if m == "pids" and v is None and _num(c.get("threads")):
                    v = c["threads"] + (c.get("zombies") or 0)
                if m == "cpus" and v is None:
                    v = c.get("cpu")
                if over(v, caps.get(k), tol, exact=(m == "pids")):
                    add(m, caps.get(k), t, v, svc)
                    gpu_seen |= m == "gpu_memory_gb"
        if not cs and not gpu_seen and over(sc.get("gpu_memory_gb"), limits.get("gpu_memory_gb"), tol):
            add("gpu_memory_gb", limits.get("gpu_memory_gb"), t, sc.get("gpu_memory_gb"), None)
    metric_of = {"memory": "memory_gb", "pids": "pids", "cpu": "cpus", "disk": "disk_gb", "gpu_memory": "gpu_memory_gb"}
    for e in events:
        if not isinstance(e, dict):
            continue
        k = e.get("event")
        if k == "oom" and e.get("scope") and e.get("cause") == "cap":
            add("oom", None, e.get("t"), e.get("count"), e.get("service") or e.get("role"))
        elif k in ("kill", "throttle_start") and e.get("resource") in metric_of and metric_of[e["resource"]] not in found:
            add(metric_of[e["resource"]], e.get("cap"), e.get("t"), e.get("value"), e.get("service"))
    return sorted(found.values(), key=lambda v: v["at"] if _num(v["at"]) else 0)


def oom_kills(events):
    """The kernel's OOM kills in a record: {"cap": in the scope at the container's own memory cap,
    "host": in the scope by another OOM killer (the host's, an enclosing cgroup's; also an event that does not say),
    "service": in a benchmark-owned service that is only recorded (whatever the cause)}."""
    out = {"cap": 0, "host": 0, "service": 0}
    for e in events:
        if not isinstance(e, dict) or e.get("event") != "oom":
            continue
        n = e.get("count")
        n = int(n) if isinstance(n, (int, float)) and not isinstance(n, bool) and n == n and 1 <= n < 1e9 else 1
        out["service" if not e.get("scope") else "cap" if e.get("cause") == "cap" else "host"] += n
    return out


def backstop(events):
    """What the monitor's backstop did (RCB_LIMITS_ENFORCE=1): {"kills": {resource: processes killed},
    "throttles": times a container's CPU was duty-cycled, "throttled_sec": seconds it was}."""
    kills = {r: 0 for r in RESOURCES if r != "cpu"}
    thr, sec = 0, 0.0
    for e in events:
        if not isinstance(e, dict):
            continue
        k = e.get("event")
        if k == "kill" and e.get("resource") in kills:
            kills[e["resource"]] += 1
        elif k == "throttle_start":
            thr += 1
        elif k == "throttle_end" and _num(e.get("sec")) and 0 <= e["sec"] < 1e7:
            sec += float(e["sec"])
    return {"kills": kills, "throttles": thr, "throttled_sec": round(sec, 1)}


def peaks(samples):
    p = {}
    for m in METRICS:
        vals = [(s.get("scope") or {}).get(m) for s in samples if isinstance(s, dict) and isinstance(s.get("scope"), dict)]
        vals = [v for v in vals if _num(v)]
        p[m] = round(max(vals), 3) if vals else 0.0
    return p


def read_jsonl(path):
    out = []
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line:
                    try:
                        out.append(json.loads(line))
                    except ValueError:
                        pass             # a torn last line (the monitor was killed mid-write)
    except OSError:
        pass
    return out


def read_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def load_record(record=None):
    """(summary or None, samples, events)."""
    record = record or os.environ.get("RCB_MONITOR_RECORD", RECORD)
    return (read_json(os.path.join(record, "summary.json")), read_jsonl(os.path.join(record, "samples.jsonl")),
            [e for e in read_jsonl(os.path.join(record, "events.jsonl")) if isinstance(e, dict)])


def record_problems(summary, samples, events, limits):
    """Why a record cannot be used ([] = usable)."""
    why = []
    if not summary:
        why.append("no sealed record (/var/rcb-monitor/summary.json missing: the monitor did not run, crashed, or was not collected)")
    elif summary.get("format") != FORMAT:
        why.append(f"record format {summary.get('format')!r}, expected {FORMAT}")
    if (limits or {}).get("format") != LIMITS_FORMAT:
        why.append(f"limits format {(limits or {}).get('format')!r}, expected {LIMITS_FORMAT}: the caps cannot be read")
    if not samples:
        why.append("no samples")
    fatal = [e for e in events if e.get("event") == "fatal"]
    if fatal:
        why.append("monitor fatal: " + "; ".join(str(e.get("why")) for e in fatal)[:300])
    if samples and summary and not summary.get("scope_seen"):
        why.append("the monitor never found the containers it watches (no beacon seen)")
    if float(limits.get("gpu_memory_gb") or 0) > 0 and samples:
        blind = sum(1 for s in samples if (s.get("scope") or {}).get("gpu_unmeasured"))
        if blind / len(samples) > MAX_GAP_FRACTION:
            why.append(f"NVML did not tell the GPU memory of processes in scope in {blind} of {len(samples)} samples "
                       "(MIG / vGPU?): the GPU-memory cap was not measured")
    if len(samples) >= 2:
        tick = float(limits.get("tick_sec", 5))
        span = samples[-1]["t"] - samples[0]["t"]
        gaps = sum(max(0.0, b["t"] - a["t"] - 3 * tick) for a, b in zip(samples, samples[1:]))
        if span > 0 and gaps / span > MAX_GAP_FRACTION:
            why.append(f"{gaps:.0f} s of {span:.0f} s unobserved")
    return why


def switch_of(summary):
    """(enforce, why the switch cannot be read or None): the record's mode when there is a record, else this step's
    own environment (the same run-time variable, task.toml [verifier.env])."""
    if summary and summary.get("switch_error"):
        return 0, str(summary["switch_error"])
    if summary and "enforce" in summary:
        return parse_switch(summary.get("enforce"))
    return parse_switch(os.environ.get("RCB_LIMITS_ENFORCE"))


def not_a_result(reward, add):
    """This step found that the trial is not a result (infra_error=1): no success is left beside it. The grader's
    verdict stays readable in success_before_limits."""
    add["infra_error"] = 1
    add["success_before_limits"] = int(bool(reward.get("success", 0)))
    add["success"] = 0
    if "reward" in reward:
        add["reward"] = 0


def tolerance_of(summary):
    """The tolerance the monitor used (its record), else this step's own environment's, else TOLERANCE."""
    if summary and _num(summary.get("tolerance")) and 0 <= summary["tolerance"] <= 1:
        return float(summary["tolerance"])
    return parse_tolerance(os.environ.get("RCB_LIMITS_TOLERANCE"))[0]


def verify(argv=()):
    log = os.environ.get("RCB_VERIFIER_LOGS", "/logs/verifier")
    tests = os.path.dirname(os.path.abspath(__file__))
    limits_file = read_json(os.path.join(tests, "rcb_limits.json"))
    limits = limits_file or {}
    summary, samples, events = load_record()
    summary = summary if isinstance(summary, dict) else None
    enforce, switch_error = switch_of(summary)
    rj_path, rt_path = os.path.join(log, "reward.json"), os.path.join(log, "reward.txt")
    reward = read_json(rj_path)
    have_grade = isinstance(reward, dict)
    reward = reward if have_grade else {}
    problems = record_problems(summary, samples, events, limits)
    if limits_file is None:
        problems.append("the verifier's copy of the caps (tests/rcb_limits.json) is missing or unreadable")
    if switch_error and not any(switch_error in p for p in problems):
        problems.append(switch_error)
    tol = tolerance_of(summary)
    viol = violations(samples, limits, events, tol) if samples or events else []
    pk = peaks(samples)
    bs = backstop(events)
    oom = oom_kills(events)
    detail = {"format": FORMAT, "enforced": enforce, "tolerance": tol, "limits": {k: v for k, v in limits.items()},
              "peaks": pk, "over_caps": viol, "record_problems": problems, "backstop": bs,
              "backstop_events": [e for e in events if e.get("event") in ("kill", "throttle_start", "throttle_end",
                                                                         "no_victim", "unobservable")][:200],
              "oom_kills": [e for e in events if e.get("event") == "oom"],
              "monitor": {k: (summary or {}).get(k) for k in ("started", "ended", "n_samples", "capabilities", "killed",
                                                               "restarts", "disk_baseline_gb", "disk_partial_walks")}}
    kills = sum(bs["kills"].values())
    add = {"resource_enforced": enforce, "limits_violation_observed": int(bool(viol)),
           "resource_monitor_missing": int(bool(problems)),
           "peak_gpu_memory_gb": pk["gpu_memory_gb"], "peak_memory_gb": pk["memory_gb"],
           "peak_processes": pk["processes"], "peak_disk_gb": pk["disk_gb"], "peak_cpus": pk["cpus"],
           "peak_pids": pk["pids"],
           "resource_memory_kills": bs["kills"]["memory"], "resource_pids_kills": bs["kills"]["pids"],
           "resource_disk_kills": bs["kills"]["disk"], "resource_gpu_memory_kills": bs["kills"]["gpu_memory"],
           "resource_monitor_kills": kills, "resource_emergency_kills": kills,
           "resource_cpu_throttles": bs["throttles"], "resource_cpu_throttled_sec": bs["throttled_sec"],
           "resource_oom_kills": oom["cap"], "resource_host_oom_kills": oom["host"],
           "resource_service_oom_kills": oom["service"]}
    for v in viol:
        add["over_" + v["metric"]] = 1
    sl = read_json(os.path.join(log, SESSION_LOG_STATUS))       # sanitize()'s session-log comparison
    if isinstance(sl, dict):
        add["session_log_tampered"] = int(bool(sl.get("tampered")))
        add["session_log_from_monitor"] = int(bool(sl.get("from_monitor")))
        detail["session_log"] = sl
    void = False                      # this step says "not a result" (infra_error=1) and takes the success away
    over_words = ", ".join(v["metric"] for v in viol)
    if switch_error:
        # nobody knows whether enforcement was asked for: not a result in either mode
        add["resource_switch_error"] = 1
        void = True
        detail["verdict"] = "not a result: " + switch_error
    elif enforce and problems:
        void = True
        detail["verdict"] = "not a result: the resource monitor's record is not usable"
    elif enforce and oom["host"]:
        # a process the limits bind was killed for memory, but not at its container's own cap: the host ran out of
        # memory (other trials), or an enclosing cgroup did. Never the agent's doing, and not a clean run either.
        add["resource_host_oom"] = 1
        void = True
        detail["verdict"] = ("not a result: an out-of-memory killer other than the container's own cap (the host's) "
                             f"killed {oom['host']} process(es) of the trial")
    elif oom["service"] and have_grade and not reward.get("success"):
        # in both modes: the kernel killed a benchmark-owned service (the simulator sidecar, a scorer) for memory and
        # the episode did not end in a success. Such a service is recorded, never a failure (PROTOCOL §6): whatever
        # the grader made of the crash, this is not the agent's 0.
        add["resource_service_oom"] = 1
        add["infra_error"] = 1
        detail["verdict"] = (f"not a result: the kernel OOM-killed a benchmark-owned service ({oom['service']} kill(s)) "
                             "and the episode did not end in a success")
    else:
        # the grade stands: a cap exceeded is recorded (and, enforced, the monitor acted on it like Docker would)
        mode = "enforced: the monitor acted" if enforce else "recorded only (RCB_LIMITS_ENFORCE=0)"
        detail["verdict"] = (f"graded normally; over a cap ({mode}): {over_words}" if viol else
                             "within the caps" if not problems else "graded normally; no usable record")
    if void:
        not_a_result(reward, add)
    os.makedirs(log, exist_ok=True)
    if have_grade:
        reward.update(add)
        reward.setdefault("success", 0)
        tmp = rj_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(reward, f, indent=1)
        os.replace(tmp, rj_path)
        if void:
            with open(rt_path, "w") as f:
                f.write("0\n")
    with open(os.path.join(log, "resource_limits.json"), "w") as f:
        json.dump(detail, f, indent=1)
    print("resource limits: " + detail["verdict"], file=sys.stderr)
    return 0


def step_failed(err):
    """This step raised: say so in the grader's reward.json instead of leaving the trial without its resource fields
    (and, when the switch asks for enforcement or cannot be read, as an infrastructure error: nothing was judged)."""
    log = os.environ.get("RCB_VERIFIER_LOGS", "/logs/verifier")
    rj_path = os.path.join(log, "reward.json")
    reward = read_json(rj_path)
    enforce, switch_error = parse_switch(os.environ.get("RCB_LIMITS_ENFORCE"))
    add = {"resource_step_error": 1, "resource_monitor_missing": 1, "resource_enforced": enforce}
    if isinstance(reward, dict):
        if enforce or switch_error:
            not_a_result(reward, add)
        reward.update(add)
        tmp = rj_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(reward, f, indent=1)
        os.replace(tmp, rj_path)
        if add.get("infra_error"):
            with open(os.path.join(log, "reward.txt"), "w") as f:
                f.write("0\n")
    with open(os.path.join(log, "resource_limits.json"), "w") as f:
        json.dump({"format": FORMAT, "enforced": enforce, "verdict": "the verifier's resource step failed: " + err,
                   "record_problems": [err]}, f, indent=1)


MAX_SESSION_LOG = 64 * 2 ** 20     # bytes of /app/output/rcb_session_log.jsonl a grader is given to read


def sanitize(roots=("/app/output",), max_log=MAX_SESSION_LOG):
    """Before the grader: the files the agent left (PROTOCOL §2.6: collected from its container as they are, links and
    all) are untrusted input, and a grader opens some of them by name. Remove what is not a file of the agent's own
    under the root: a symbolic link that leaves it or does not end at a regular file (`rcb_session_log.jsonl ->
    /dev/zero` made a grader read until the verifier ran out of memory or time: no verdict), every FIFO, device and
    socket (opening a FIFO blocks), and a session log that is a symbolic link wherever it points (a link to a file of
    the agent's own, gigabytes without a newline, is as bad: graders open the log by name). A session log larger than
    `max_log` is cut there (graders read it line by line, and one endless line is as bad). Regular files, directories
    and other links to regular files inside the root stay as they are: nothing a grader could have read safely changes.
    Returns what it removed or cut (also printed)."""
    done = []
    for root in roots:
        real_root = os.path.realpath(root)
        if not os.path.isdir(real_root) or os.path.islink(root):
            if os.path.islink(root):                 # the root itself must be a directory of its own
                try:
                    os.unlink(root)
                    done.append(f"removed {root}: a symbolic link")
                except OSError:
                    pass
            continue
        log = os.path.join(real_root, "rcb_session_log.jsonl")
        for dirpath, dirnames, filenames in os.walk(real_root, followlinks=False):
            for name in list(dirnames) + filenames:
                path = os.path.join(dirpath, name)
                try:
                    st = os.lstat(path)
                    if stat.S_ISLNK(st.st_mode):
                        if path == log:              # the session log is read by name: a link to any file (a huge one
                            why = "the session log is a symbolic link"   # without a newline) is removed, never followed
                        else:
                            target = os.path.realpath(path)
                            inside = target.startswith(real_root + os.sep)
                            if inside and stat.S_ISREG(os.stat(target).st_mode):
                                continue
                            why = "a symbolic link that leaves the directory or does not end at a regular file"
                    elif stat.S_ISREG(st.st_mode) or stat.S_ISDIR(st.st_mode):
                        continue
                    else:
                        why = "not a regular file (FIFO, device or socket)"
                    os.unlink(path)
                    if name in dirnames:
                        dirnames.remove(name)
                    done.append(f"removed {path}: {why}")
                except OSError as e:                 # a dangling link, a file that is gone: remove what can be removed
                    try:
                        if stat.S_ISLNK(os.lstat(path).st_mode):
                            os.unlink(path)
                            done.append(f"removed {path}: a symbolic link that cannot be resolved ({type(e).__name__})")
                    except OSError:
                        pass
        try:
            if os.path.isfile(log) and not os.path.islink(log) and os.path.getsize(log) > max_log:
                os.truncate(log, max_log)
                done.append(f"cut {log} to {max_log} bytes")
        except OSError:
            pass
        try:
            msg = session_log_source(log, max_log)
            if msg:
                done.append(msg)
        except Exception as e:                       # never keep the grader from running
            done.append(f"session log: not compared with the monitor's copy ({type(e).__name__}: {e})")
    for d in done:
        print("verifier input: " + d, file=sys.stderr)
    return done


SESSION_LOG_STATUS = "session_log.json"   # in /logs/verifier: what session_log_source() did, read back by verify()


def session_log_source(log, max_log=MAX_SESSION_LOG, record=None):
    """The session log the grader reads. The monitor copies what the agent's tooling appends to it, append-only,
    into its record (session_log/<segment>.jsonl; a new segment each time the file was truncated, replaced or
    rewritten). The agent's file stays when it is the copy plus at most what was appended after the last tick; else
    (removed, truncated, rewritten, or the copy has segments) the grader gets the copy: the segments' lines in order,
    in a later segment a line an earlier one already gave left out (a rewrite copies the file again). No copy (no monitor on the runner, or no log
    written): nothing changes, nothing is written. Else writes /logs/verifier/session_log.json; returns a line for the
    log, or None."""
    record = record or os.environ.get("RCB_MONITOR_RECORD", RECORD)
    d = os.path.join(record, "session_log")
    segs = sorted((int(n.split(".")[0]) for n in (os.listdir(d) if os.path.isdir(d) else ())
                   if n.endswith(".jsonl") and n.split(".")[0].isdigit()))
    events = [e for e in read_jsonl(os.path.join(record, "events.jsonl")) if isinstance(e, dict)]
    tamper = [e for e in events if e.get("event") in ("session_log_rewritten", "session_log_removed")]
    status = {"from_monitor": 0, "tampered": int(bool(tamper)), "monitor_copy": bool(segs), "events": tamper[:20]}
    msg = None
    if segs:
        copies = []
        for n in segs:
            with open(os.path.join(d, "%d.jsonl" % n), "rb") as f:
                copies.append(f.read(max_log))
        agent = None
        if os.path.isfile(log) and not os.path.islink(log):
            with open(log, "rb") as f:
                agent = f.read(max_log)
        if len(segs) == 1 and not tamper and agent is not None and agent.startswith(copies[0]):
            pass                                     # the agent's file is what was written (and maybe a last tick more)
        else:
            given, out = {}, []                      # line -> most times one earlier segment had it
            for c in copies:
                again, lines, here = dict(given), c.splitlines(True), {}
                for line in lines:
                    here[line] = here.get(line, 0) + 1
                    if again.get(line):
                        again[line] -= 1             # the rewrite's copy of a line given before
                        continue
                    out.append(line)
                for line, k in here.items():
                    given[line] = max(given.get(line, 0), k)
            data = b"".join(out)[:max_log]
            if os.path.lexists(log):
                os.unlink(log)
            with open(log, "wb") as f:
                f.write(data)
            status.update(from_monitor=1, tampered=1, agent_bytes=None if agent is None else len(agent),
                          copy_bytes=len(data))
            msg = (f"replaced {log} with the monitor's copy ({len(data)} bytes; the agent's file was "
                   + ("missing" if agent is None else f"{len(agent)} bytes, not what was written") + ")")
    if not segs and not tamper:
        return None                                  # no copy (no monitor, or no log written): nothing to say
    logs = os.environ.get("RCB_VERIFIER_LOGS", "/logs/verifier")
    try:
        os.makedirs(logs, exist_ok=True)
        with open(os.path.join(logs, SESSION_LOG_STATUS), "w") as f:
            json.dump(status, f, indent=1)
    except OSError:
        pass
    return msg


def run_step(argv=()):
    """verify(), and whatever it raises reported instead of breaking the grader's own verdict. Always 0."""
    try:
        verify(argv)
    except Exception as e:
        msg = f"{type(e).__name__}: {e}"
        print(f"resource limits: verifier step failed: {msg}", file=sys.stderr)
        try:
            step_failed(msg)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    if sys.argv[1:2] == ["verify"]:
        sys.exit(run_step(sys.argv[2:]))
    if sys.argv[1:2] == ["sanitize"]:
        try:
            sanitize(sys.argv[2:] or ("/app/output",))
        except Exception as e:                       # never keep the grader from running
            print(f"verifier input: sanitize failed: {type(e).__name__}: {e}", file=sys.stderr)
        sys.exit(0)
    print(__doc__)
