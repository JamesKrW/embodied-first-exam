"""Verifier entry point of an EmbodiedSWE task: run the delivered program /app/output/solution/ (`solve(env)`) against a
fresh simulator built from the frozen instance and grade it once, as EmbodiedSWE does; then re-execute its recorded
actions in another fresh process (a diagnostic and the video's source, never part of the verdict).

    python /tests/grade.py --instance /tests/instance.json --grader /tests/grader/<scene>.py

Every step runs in its own process (`rcb_embodiedswe.py`, the image's tooling; the grader comes from this directory):
1. rollout  - the trusted simulator process builds the env from the frozen instance (seeded reset, PhysX on the CPU),
              checks its state hash, attaches the task's grader (EmbodiedSWE's rubric, recorded after every step) and
              serves the solution, which runs in ANOTHER process as `nobody` without network or GPU, through a proxy that
              answers reads with copies, executes `env.step(action)` and refuses everything else (an exception from
              `env.step` is raised in `solve`, as EmbodiedSWE's GradedEnv lets it propagate). It ends when `solve`
              returns or raises, its process dies, or the rollout limit (30 min of solve time) passes; every process of
              `nobody` is killed, then the verdict is read on the state at that moment: success = the grader's end
              predicate (`check_success`), progress = its rubric score (peak recorded progress). The actions are
              recorded. This is the verdict (a run in which no action reached the simulator scores 0).
2. replay   - a fresh process rebuilds the frozen instance and re-executes the recorded actions (no solution code):
              a diagnostic only (`deterministic` = the same final-state hash, `replay_success`, `replay_progress`).
              A replay that differs, fails or overruns leaves the verdict as it is and is noted in grade.log.
3. video    - only after reward.txt / reward.json are written: a third process renders the recorded actions;
              video.json says whether its final state matches the rollout's (`video_mismatch`; `video_ok` 0 then).

Whose failure a 0 is (every path writes a reward; `deterministic`, `grader_error` and `infra_error` are always present):
- the agent's: no solution/solve.py (`missing_solution`), no action at all, a final state that does not meet the
  goal (whether solve returned, raised (`solve_error`), timed out (`solve_timeout`) or sent a malformed action
  (`bad_action`: wrong shape, NaN or inf)), or, in a task with a tool-only rule, the robot's hand touching the
  payload (`disqualified`, `tool_contact`: success 0 and progress 0);
- the verifier's (`infra_error` = 1, no verdict on the agent): the rebuilt instance not matching the frozen one
  (`instance_mismatch`), a rollout process that ends without a result or overruns its limit, any error here.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
import traceback

LOG_DIR = os.environ.get("RCB_VERIFIER_LOG_DIR", "/logs/verifier")
TOOL = "/opt/efe/rcb_embodiedswe.py"
WORK = "/var/lib/esw"
ROLLOUT_S = 1800.0                  # the solve's wall-clock limit (EmbodiedSWE's rollout limit)
BOOT_S = 900.0                      # Isaac Sim start + scene build, generous (measured ~1-3 min)
LOAD_S = 300.0                      # importing solve.py (the tooling's own limit)


ZERO = {"success": 0, "deterministic": 0, "grader_error": 0, "infra_error": 0}
ERR = {"success": 0, "deterministic": 0, "grader_error": 1, "infra_error": 1}


def write(ok: bool, out: dict) -> None:
    os.makedirs(LOG_DIR, exist_ok=True)
    open(os.path.join(LOG_DIR, "reward.txt"), "w").write("1\n" if ok else "0\n")
    json.dump(out, open(os.path.join(LOG_DIR, "reward.json"), "w"))
    print(json.dumps(out), flush=True)


def run(cmd: list[str], timeout: float, log_name: str) -> tuple[dict | None, bool]:
    """(the last JSON object the command printed, timed_out). Isaac writes to stdout too; its noise goes to the log."""
    with open(os.path.join(LOG_DIR, log_name), "ab") as lf:
        try:
            p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=lf, timeout=timeout, env={**os.environ},
                               start_new_session=True)
        except subprocess.TimeoutExpired:
            return None, True
        lf.write(p.stdout[-200_000:])
    res = None
    for line in p.stdout.decode(errors="replace").splitlines():
        if line.startswith("{"):
            try:
                v = json.loads(line)
            except ValueError:
                continue
            if isinstance(v, dict):
                res = v
    return res, False


def _verify(a, ro: dict, out_dir: str, status: str, n: int, t0: float) -> dict:
    """Score the rollout (the verdict), then replay the recorded actions in a fresh process as a diagnostic; writes
    reward.txt / reward.json, returns the fields."""
    t1 = time.time()
    # The verdict: the rollout's own reading of the grader on its final state (EmbodiedSWE's verdict). No action
    # reached the sim: nothing was done (nop = 0 is an acceptance check), so that scores 0 whatever the predicate says.
    ok = n > 0 and bool(ro.get("success"))
    progress = float(ro.get("progress") or 0.0) if n > 0 else 0.0
    diag = {"replay_failed": 0, "replay_timed_out": 0}
    if n > 0:
        rp, rp_timeout = run([sys.executable, TOOL, "replay", "--instance", a.instance, "--grader", a.grader,
                              "--traj", os.path.join(out_dir, "actions.npz")],
                             BOOT_S + 2.0 * float(ro.get("solve_wall_s", ROLLOUT_S)) + 600.0, "replay.log")
        if rp_timeout or rp is None or "error" in rp or not rp.get("instance_ok"):
            print(f"[grade] replay gave no result (diagnostic only, the verdict stands): "
                  f"{json.dumps(rp)[:2000] if rp else ('timed out' if rp_timeout else 'no result')}", file=sys.stderr)
            diag = {"replay_failed": 1, "replay_timed_out": int(rp_timeout)}
            rp = {}
    else:
        rp = {"success": False, "final_sha": ro.get("final_sha"), "progress": 0.0}
    deterministic = bool(rp) and rp.get("final_sha") == ro.get("final_sha")
    if n > 0 and rp and not deterministic:
        print(f"[grade] the replay did not reach the rollout's final state (rollout {ro.get('final_sha')}, replay "
              f"{rp.get('final_sha')}; replay success {int(bool(rp.get('success')))}): the verdict stands, the "
              f"video (a replay) does not show this run", file=sys.stderr)
    out = {**ZERO, "success": int(ok), "deterministic": int(deterministic),
           "rollout_success": int(bool(ro.get("success"))), "replay_success": int(bool(rp.get("success"))),
           "progress": round(progress, 6), "replay_progress": round(float(rp.get("progress") or 0.0), 6), **diag,
           "instance_ok": 1, "n_actions": n, "sim_steps": n,
           "solve_returned": int(status == "returned"), "solve_error": int(status in ("error", "process_ended")),
           "solve_timeout": int(status == "timeout"), "bad_action": int(int(ro.get("bad_actions", 0)) > 0),
           "step_errors": int(ro.get("step_errors", 0)), "disqualified": int(ro.get("disqualified", 0)),
           **({"tool_contact": int(ro.get("tool_contact", 0)),
               "replay_tool_contact": int(rp.get("tool_contact", 0) or 0)} if ro.get("tool_only") else {}),
           "proxy_requests": int(ro.get("requests", 0)), "proxy_refused": int(ro.get("refused", 0)),
           "solve_wall_s": float(ro.get("solve_wall_s", 0.0)), "rollout_ms_per_step": float(ro.get("ms_per_step", 0.0)),
           "rollout_wall_s": round(t1 - t0, 1), "replay_wall_s": round(time.time() - t1, 1)}
    write(ok, out)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", required=True)
    ap.add_argument("--grader", required=True)
    ap.add_argument("--solution", default="/app/output/solution")
    ap.add_argument("--no-video", action="store_true")
    a = ap.parse_args()
    entry = os.path.join(a.solution, "solve.py")
    if not (os.path.isdir(a.solution) and not os.path.islink(a.solution)
            and os.path.isfile(entry) and not os.path.islink(entry)):
        write(False, {**ZERO, "missing_solution": 1, "progress": 0.0})
        return 0
    try:
        shutil.rmtree(WORK, ignore_errors=True)
        os.makedirs(WORK, mode=0o755)
        out_dir = os.path.join(WORK, "rollout")
        t0 = time.time()
        limit = BOOT_S + LOAD_S + ROLLOUT_S + 300.0
        ro, timed_out = run([sys.executable, TOOL, "rollout", "--instance", a.instance, "--grader", a.grader,
                             "--solution", a.solution, "--out", out_dir, "--timeout", str(ROLLOUT_S)], limit, "rollout.log")
        if timed_out or ro is None or ro.get("infra_error") or ro.get("phase") != "done":
            fields = {"rollout_timed_out": int(timed_out), "rollout_failed": int(not timed_out),
                      "instance_ok": int(bool((ro or {}).get("instance_ok", 1)))}
            if ro and ro.get("phase") == "instance_mismatch":
                fields["instance_mismatch"] = 1
            print(f"[grade] rollout gave no verdict: {json.dumps(ro)[:2000] if ro else 'no result'}", file=sys.stderr)
            write(False, {**ERR, **fields, "rollout_wall_s": round(time.time() - t0, 1)})
            return 0
        shutil.copyfile(os.path.join(out_dir, "actions.npz"), os.path.join(LOG_DIR, "trajectory.npz"))
        for f in ("rollout.json", "solve.log"):
            if os.path.isfile(os.path.join(out_dir, f)):
                shutil.copyfile(os.path.join(out_dir, f), os.path.join(LOG_DIR, f))
        status = ro.get("solve_status", "")
        n = int(ro.get("n_actions", 0))
        if ro.get("step_errors"):
            print(f"[grade] env.step raised {ro.get('step_errors')} time(s), first at action "
                  f"{ro.get('step_error_at')}: "
                  f"{str(ro.get('step_error'))[:1000]}", file=sys.stderr)
        out = _verify(a, ro, out_dir, status, n, t0)
    except Exception:  # noqa: BLE001
        traceback.print_exc(file=sys.stderr)
        write(False, {**ERR})
        return 0
    if not a.no_video and os.path.isfile(os.path.join(LOG_DIR, "trajectory.npz")):
        v, _ = run([sys.executable, TOOL, "video", "--instance", a.instance, "--traj",
                    os.path.join(LOG_DIR, "trajectory.npz"), "--video", os.path.join(LOG_DIR, "replay.mp4")],
                   BOOT_S + 3.0 * float(out.get("solve_wall_s") or ROLLOUT_S) + 600.0, "video.log")
        rendered = bool(v and v.get("video_ok"))
        # The video is a replay: it shows this run only if it reaches the rollout's final state.
        mismatch = not (rendered and v.get("final_sha") and v.get("final_sha") == ro.get("final_sha"))
        if rendered and mismatch:
            print(f"[grade] the video's replay did not reach the rollout's final state (rollout {ro.get('final_sha')}, "
                  f"video {v.get('final_sha')}): video_mismatch, the video does not show this run", file=sys.stderr)
        json.dump({"video_source": "replay", "video_ok": int(rendered and not mismatch),
                   "video_rendered": int(rendered), "video_mismatch": int(mismatch),
                   "frames": int((v or {}).get("frames", 0))},
                  open(os.path.join(LOG_DIR, "video.json"), "w"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
