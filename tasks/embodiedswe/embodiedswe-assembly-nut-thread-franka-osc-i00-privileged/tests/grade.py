"""Verifier entry point of an EmbodiedSWE task, as EmbodiedSWE's harbor verifier grades: every submitted checkpoint
(`submit`, /app/output/submissions/NNNNNN/) and the final /app/output/solution/ are candidates; each distinct one runs
its `solve(env)` once against a fresh simulator built from the frozen instance; the best one is the result.

    python /tests/grade.py --instance /tests/instance.json --grader /tests/grader/<scene>.py

1. candidates - the submissions in name order (at most 64), then `final`. Each is checked (a solve.py; for a submission,
                its files equal its receipt, submission.json) and frozen into a root-owned copy before any of them
                runs; one that fails a check scores 0 (`invalid_submission`, `missing_solution`). Byte-identical
                candidates (the same content hash) are graded once: a later one takes the earlier one's result.
2. rollout    - per distinct candidate, a fresh process (`rcb_embodiedswe.py rollout`, the image's tooling; the grader
                comes from this directory): the trusted simulator builds the env from the frozen instance (GPU PhysX),
                compares its start state, attaches the task's grader (EmbodiedSWE's rubric, recorded after every step)
                and serves the candidate's `solve`, which runs in ANOTHER process as `nobody` without network or GPU,
                through a proxy that answers reads with copies, executes `env.step(action)` and refuses everything
                else. The candidate's limit is EmbodiedSWE's: ROLLOUT_S of wall clock for the whole process (start-up
                and import count); then SIGTERM: the solve is ended and the verdict read on the state at that moment;
                GRACE_S later, SIGKILL. success = the grader's end predicate (`check_success`), progress = its rubric
                score (peak recorded progress). A process that ends without a result while solving scores 0 (the
                candidate's); in any other phase it is the verifier's error.
3. selection  - the candidate with the highest progress (EmbodiedSWE's score); ties go to higher success, then to a
                graded candidate over one that never ran, then to the earliest (`final` is last). reward.txt and
                reward.json give that one candidate's success and progress; results.json lists every candidate.
4. video      - only after reward.txt / reward.json are written: the selected candidate's trajectory (its actions and
                recorded states, trajectory.npz) is rendered by posing its states (replay.mp4), after a camera replay
                of its actions as a diagnostic (video.json). Neither changes the verdict.

Whose failure a 0 is (`grader_error` and `infra_error` are always present):
- the candidate's: no solve.py, a receipt that does not match, no action, a final state that does not meet the goal
  (whether solve returned, raised, timed out or sent a malformed action), or, in a task with a tool-only rule, the
  robot's hand touching the payload (`disqualified`);
- the verifier's (`infra_error` = 1, no verdict on the agent): the rebuilt instance not matching the frozen one on the
  GPU model that froze it (`instance_mismatch`), a rollout that ends without a result outside its solve, any error here.

Timeout of the whole verifier (task.toml): (64 + 1) x (ROLLOUT_S + GRACE_S) + VIDEO_S + 300 = 122,550 s.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import signal
import stat
import subprocess
import sys
import time
import traceback

LOG_DIR = os.environ.get("RCB_VERIFIER_LOG_DIR", "/logs/verifier")
TOOL = "/opt/efe/rcb_embodiedswe.py"
WORK = "/var/lib/esw"
ROLLOUT_S = 1800.0                  # EmbodiedSWE's rollout_timeout_sec: the whole candidate process
GRACE_S = 30.0                      # EmbodiedSWE's termination_grace_sec: SIGTERM, then SIGKILL
MAX_SUBMISSIONS = 64                # EmbodiedSWE's max_submissions
VIDEO_S = 900.0 + 1800.0 + 600.0    # the video process: start-up with cameras + camera replay budget + posing
REPLAY_BUDGET_S = 1800.0
NOBODY = 65534
ERR = {"success": 0, "progress": 0.0, "grader_error": 1, "infra_error": 1}


class EvaluationError(RuntimeError):
    pass


def write(ok: bool, out: dict) -> None:
    os.makedirs(LOG_DIR, exist_ok=True)
    open(os.path.join(LOG_DIR, "reward.txt"), "w").write("1\n" if ok else "0\n")
    json.dump(out, open(os.path.join(LOG_DIR, "reward.json"), "w"))
    print(json.dumps(out), flush=True)


def note(msg: str) -> None:
    print(f"[grade] {msg}", file=sys.stderr, flush=True)


# ------------------------------------------------------------------------------------------------ candidates
def inventory(root: str) -> dict[str, str]:
    """{relative path: sha256} of every regular file (esw_submit.py's receipt format). Links and special files are not
    part of a solution (the rollout copies regular files only), so they are left out here too."""
    out = {}
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        for name in sorted(filenames):
            p = os.path.join(dirpath, name)
            if not stat.S_ISREG(os.lstat(p).st_mode):
                continue
            h = hashlib.sha256()
            with open(p, "rb") as f:
                for chunk in iter(lambda: f.read(1 << 20), b""):
                    h.update(chunk)
            out[os.path.relpath(p, root).replace(os.sep, "/")] = h.hexdigest()
    return dict(sorted(out.items()))


def content_hash(files: dict[str, str]) -> str:
    return hashlib.sha256(json.dumps(files, sort_keys=True, separators=(",", ":")).encode()).hexdigest()


def candidates(submissions: str, final: str) -> list[dict]:
    """The submissions in name order, then `final`; every entry says where its solution is and whether it may run."""
    found = []
    if os.path.isdir(submissions) and not os.path.islink(submissions):
        for name in sorted(os.listdir(submissions)):
            if not name.isdecimal():
                continue
            path = os.path.join(submissions, name)
            e = {"submission": name, "path": os.path.join(path, "solution")}
            if len(name) != 6 or os.path.islink(path) or not os.path.isdir(path):
                e["status"] = "invalid_submission"
                e["error"] = "not a NNNNNN directory"
            if len(found) >= MAX_SUBMISSIONS:
                e["status"], e["error"] = "over_limit", f"more than {MAX_SUBMISSIONS} submissions: not graded"
            found.append(e)
    found.append({"submission": "final", "path": final})
    return found


def freeze(entries: list[dict], inputs: str) -> None:
    """Check and freeze every candidate (root-owned, read-only copies) before any of them runs."""
    for e in entries:
        e.update(score=0.0, success=0)
        if e.get("status"):
            continue
        sol = e["path"]
        entry = os.path.join(sol, "solve.py")
        if not (os.path.isdir(sol) and not os.path.islink(sol) and os.path.isfile(entry) and not os.path.islink(entry)):
            e["status"] = "missing_solution"
            continue
        try:
            files = inventory(sol)
            digest = content_hash(files)
            if e["submission"] != "final":
                receipt = json.load(open(os.path.join(os.path.dirname(sol), "submission.json")))
                if not isinstance(receipt, dict) or receipt.get("schema_version") != 1:
                    raise ValueError("invalid submission receipt")
                if receipt.get("files") != files or receipt.get("sha256") != digest:
                    raise ValueError("the snapshot's files differ from its receipt")
                e["note"] = str(receipt.get("note", ""))[:500]
                e["created_at"] = str(receipt.get("created_at", ""))[:64]
            if len(files) > 20_000:
                raise ValueError("more than 20,000 files")
        except (OSError, ValueError) as exc:
            e.update(status="invalid_submission", error=str(exc)[:500])
            continue
        frozen = os.path.join(inputs, e["submission"])
        shutil.copytree(sol, frozen, symlinks=True)   # a storage failure here is the verifier's error
        if inventory(frozen) != files:
            raise EvaluationError("a solution changed while it was frozen")
        for dirpath, _, filenames in os.walk(frozen):
            os.chmod(dirpath, 0o755)
            for f in filenames:
                os.chmod(os.path.join(dirpath, f), 0o644)
        e.update(sha256=digest, frozen=frozen)


# ------------------------------------------------------------------------------------------------ one rollout
def kill_uid(uid: int = NOBODY) -> None:
    """End every process of `uid` (a solution the rollout process could not end itself), from a child of that uid."""
    if os.geteuid() != 0:
        return
    pid = os.fork()
    if pid == 0:
        try:
            os.setgid(uid)
            os.setuid(uid)
            for _ in range(3):
                try:
                    os.kill(-1, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                time.sleep(0.2)
        finally:
            os._exit(0)
    os.waitpid(pid, 0)


def _killpg(pid: int, sig: int) -> None:
    try:
        os.killpg(pid, sig)
    except ProcessLookupError:
        pass


def rollout(a, e: dict, out_dir: str) -> dict:
    """One candidate's graded run, as EmbodiedSWE's run_rollout: ROLLOUT_S, then SIGTERM, GRACE_S, then SIGKILL."""
    os.makedirs(out_dir)
    cmd = [sys.executable, TOOL, "rollout", "--instance", a.instance, "--grader", a.grader, "--solution", e["frozen"],
           "--out", out_dir, "--timeout", str(ROLLOUT_S), "--label", e["submission"]]
    timed_out = False
    t0 = time.time()
    with open(os.path.join(LOG_DIR, "rollout.log"), "ab") as lf:
        lf.write(f"\n===== candidate {e['submission']}\n".encode())
        lf.flush()
        proc = subprocess.Popen(cmd, stdout=lf, stderr=lf, start_new_session=True, env={**os.environ})
        try:
            try:
                code = proc.wait(timeout=ROLLOUT_S)
            except subprocess.TimeoutExpired:
                timed_out = True
                _killpg(proc.pid, signal.SIGTERM)
                try:
                    code = proc.wait(timeout=GRACE_S)
                except subprocess.TimeoutExpired:
                    _killpg(proc.pid, signal.SIGKILL)
                    code = proc.wait()
        finally:
            _killpg(proc.pid, signal.SIGKILL)         # nothing of this candidate bleeds into the next one
            kill_uid()
            proc.wait()
    res = None
    try:
        res = json.load(open(os.path.join(out_dir, "rollout.json")))
    except (OSError, ValueError):
        pass
    meta = {"exit_code": code, "timed_out": int(timed_out), "process_wall_s": round(time.time() - t0, 1)}
    if not isinstance(res, dict):
        try:
            phase = json.load(open(os.path.join(out_dir, "phase.json")))["phase"]
        except (OSError, ValueError, KeyError, TypeError):
            phase = "unknown"
        if phase == "solve" and (timed_out or code >= 0):
            why = "exceeded its time limit" if timed_out else f"exited ({code})"
            return {**meta, "status": "no_verdict", "score": 0.0, "success": 0, "phase": phase,
                    "solution_error": f"the candidate's run {why} without a verdict"}
        raise EvaluationError(f"candidate {e['submission']}: the rollout process ended in {phase} without a result "
                              f"(exit {code}, timed out {timed_out})")
    if res.get("infra_error") or res.get("phase") != "done":
        raise EvaluationError(f"candidate {e['submission']}: rollout gave no verdict: {json.dumps(res)[:2000]}")
    return {**meta, "status": "graded", "score": round(float(res.get("progress") or 0.0), 4),
            "success": int(bool(res.get("success"))), "rollout": res}


# ------------------------------------------------------------------------------------------------ the verdict
def numbers(e: dict) -> dict:
    """reward.json's fields for the selected candidate (numbers only)."""
    ro = e.get("rollout") or {}
    status = ro.get("solve_status", "")
    out = {"selected_submission": 0 if e["submission"] == "final" else int(e["submission"]),
           "selected_final": int(e["submission"] == "final"),
           "missing_solution": int(e.get("status") == "missing_solution"),
           "invalid_submission": int(e.get("status") == "invalid_submission"),
           "no_verdict": int(e.get("status") == "no_verdict"),
           "no_actions": int(e.get("status") == "graded" and int(ro.get("n_actions", 0)) == 0),
           "solve_error": int(status == "error"), "solve_timeout": int(status == "timeout"),
           "solve_process_ended": int(status == "process_ended"), "bad_action": int(ro.get("bad_actions", 0) > 0),
           "n_actions": int(ro.get("n_actions", 0)), "step_errors": int(ro.get("step_errors", 0)),
           "solve_wall_s": float(ro.get("solve_wall_s", 0.0)), "boot_s": float(ro.get("boot_s", 0.0)),
           "rollout_ms_per_step": float(ro.get("ms_per_step", 0.0)),
           "instance_ok": int(ro.get("instance_ok", 0)), "instance_same_gpu": int(ro.get("instance_same_gpu", 0)),
           "instance_within_tol": int(ro.get("instance_within_tol", 0)),
           "instance_max_abs_diff": float(ro.get("instance_max_abs_diff", 0.0)),
           "proxy_requests": int(ro.get("requests", 0)), "proxy_refused": int(ro.get("refused", 0))}
    if ro.get("tool_only"):
        out.update(tool_only=1, disqualified=int(ro.get("disqualified", 0) or 0),
                   tool_contact=int(ro.get("tool_contact", 0) or 0))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--instance", required=True)
    ap.add_argument("--grader", required=True)
    ap.add_argument("--solution", default="/app/output/solution")
    ap.add_argument("--submissions", default="/app/output/submissions")
    ap.add_argument("--no-video", action="store_true")
    a = ap.parse_args()
    t0 = time.time()
    try:
        shutil.rmtree(WORK, ignore_errors=True)
        os.makedirs(os.path.join(WORK, "inputs"), mode=0o755)
        entries = candidates(a.submissions, a.solution)
        freeze(entries, os.path.join(WORK, "inputs"))
        seen: dict[str, dict] = {}
        for e in entries:
            if not e.get("frozen"):
                continue
            prev = seen.get(e["sha256"])
            if prev is not None:
                e.update(status="duplicate", duplicate_of=prev["submission"], score=prev["score"],
                         success=prev["success"])
                continue
            r = rollout(a, e, os.path.join(WORK, "rollouts", e["submission"]))
            e.update(r)
            seen[e["sha256"]] = e
            note(f"candidate {e['submission']}: {e['status']} progress {e['score']} success {e['success']}")
            json.dump({"candidates": [{k: v for k, v in x.items() if k != "rollout"} for x in entries]},
                      open(os.path.join(LOG_DIR, "progress.json"), "w"), indent=1)
        # EmbodiedSWE's selection: the highest score, then success; the first such (the earliest; final is last).
        # A tie between a candidate that ran and one that never did goes to the one that ran (its run is the video).
        best = max(entries, key=lambda x: (x["score"], x["success"], x.get("status") in ("graded", "duplicate")))
        source = seen.get(best.get("sha256", ""), best) if best.get("status") == "duplicate" else best
        out = {"success": int(best["success"]), "progress": float(best["score"]), "grader_error": 0, "infra_error": 0,
               "n_candidates": len(entries), "n_graded": len(seen), **numbers(source)}
        out["selected_submission"] = 0 if best["submission"] == "final" else int(best["submission"])
        out["selected_final"] = int(best["submission"] == "final")
        results = {"selected_submission": best["submission"], "video_from": source["submission"],
                   "reward": {"success": out["success"], "progress": out["progress"]},
                   "candidates": [{k: v for k, v in x.items() if k not in ("path", "frozen")} for x in entries],
                   "limits": {"rollout_timeout_sec": ROLLOUT_S, "termination_grace_sec": GRACE_S,
                              "max_submissions": MAX_SUBMISSIONS}}
        json.dump(results, open(os.path.join(LOG_DIR, "results.json"), "w"), indent=1)
        src_dir = os.path.join(WORK, "rollouts", source["submission"])
        for f in ("trajectory.npz", "rollout.json", "solve.log"):
            if os.path.isfile(os.path.join(src_dir, f)):
                shutil.copyfile(os.path.join(src_dir, f), os.path.join(LOG_DIR, f))
        for x in entries:
            ro = x.get("rollout") or {}
            if ro.get("step_errors"):
                note(f"candidate {x['submission']}: env.step raised {ro['step_errors']} time(s), first at action "
                     f"{ro.get('step_error_at')}: {str(ro.get('step_error'))[:1000]}")
            if ro and not ro.get("instance_ok"):
                note(f"candidate {x['submission']}: start state differs from the frozen hash on GPU "
                     f"{ro.get('gpu_model')!r} (frozen on {ro.get('instance_gpu')!r}): max abs difference "
                     f"{ro.get('instance_max_abs_diff')}")
        out["verifier_wall_s"] = round(time.time() - t0, 1)
        write(bool(out["success"]), out)
    except Exception:  # noqa: BLE001 — EvaluationError or anything else: the verifier's, no verdict on the agent
        traceback.print_exc(file=sys.stderr)
        write(False, {**ERR, "verifier_wall_s": round(time.time() - t0, 1)})
        return 0
    traj = os.path.join(LOG_DIR, "trajectory.npz")
    if a.no_video or not os.path.isfile(traj):
        return 0
    v = None
    with open(os.path.join(LOG_DIR, "video.log"), "ab") as lf:
        try:
            p = subprocess.run([sys.executable, TOOL, "video", "--instance", a.instance, "--traj", traj,
                                "--video", os.path.join(LOG_DIR, "replay.mp4"),
                                "--final-sha", str((source.get("rollout") or {}).get("final_sha") or ""),
                                "--replay-budget", str(REPLAY_BUDGET_S)],
                               stdout=subprocess.PIPE, stderr=lf, timeout=VIDEO_S, start_new_session=True)
            for line in p.stdout.decode(errors="replace").splitlines():
                if line.startswith("{"):
                    try:
                        v = json.loads(line)
                    except ValueError:
                        pass
            lf.write(p.stdout[-200_000:])
        except subprocess.TimeoutExpired:
            note("the video process overran its limit: no video")
    v = v if isinstance(v, dict) else {}
    if v.get("error"):
        note(f"the video process failed: {str(v['error'])[-1500:]}")
    json.dump({"video_source": "recorded_states", "video_from": source["submission"],
               "video_ok": int(v.get("video_ok", 0) or 0), "frames": int(v.get("frames", 0) or 0),
               "camera_replay": v.get("camera_replay", "not_run"),
               "camera_replay_match": int(v.get("camera_replay_match", 0) or 0),
               "camera_replay_success": int(v.get("camera_replay_success", 0) or 0),
               "camera_replay_s": float(v.get("camera_replay_s", 0.0) or 0.0),
               "camera_instance_ok": int(v.get("camera_instance_ok", 0) or 0),
               "frames_nonfinite": int(v.get("frames_nonfinite", 0) or 0)},
              open(os.path.join(LOG_DIR, "video.json"), "w"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
