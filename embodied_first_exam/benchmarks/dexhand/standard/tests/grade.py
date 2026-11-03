"""Grade the trusted sidecar's record of the live episode (final.json); no agent-produced success claim is read.

success = the live episode's success under the task's predicate, judged by the sim service (graded_by = "live"; owner,
2026-10-17): nothing is replayed. No verdict on the agent is an infrastructure failure (infra_error = 1),
never the agent's 0: final.json missing, unreadable or not finalized; the service did not end and dump its episode in
time; it gave up after crashes that were not all inside the agent's calls (sim_crashed, resume_failed); the frozen
instance did not build at the episode's start (instance_mismatch); a hand the task does not offer. A crash inside the
agent's calls (sim_crashed_on_call), a simulator error from the agent's actions (reason "error"), a timeout and a
failed predicate are the agent's 0. The video is rendered after the verdict and never decides it."""

from __future__ import annotations

import json
import os
from pathlib import Path
import shutil

BASE = {"success": 0, "live_success": 0, "graded_by_live": 1, "replays": 0, "infra_error": 0, "sim_error": 0,
        "grader_error": 0, "finalized": 0, "missing_log": 0}


def state_ops_fields(data: dict, success: bool) -> dict:
    """The unlimited-reset ablation's record (the service writes it with RCB_STATE_OPS=1): saves and loads, and for a
    success resets_at_success / loads_at_success; first_success = a success with no reset and no load. Numbers only; {}
    without the switch."""
    if "loads" not in data:
        return {}
    count = lambda key: int(data.get(key) or 0)  # noqa: E731
    out = {"saves": count("saves"), "loads": count("loads"),
           "first_success": int(success and count("resets") + count("loads") == 0)}
    if success:
        out.update({key: count(key) for key in ("resets_at_success", "loads_at_success") if key in data})
    return out


def state_query_fields(data: dict) -> dict:
    """The state-query ablation's count (the service writes it with RCB_STATE_QUERY=1): state_queries, the /state/query
    calls answered; {} without the switch. Never part of the verdict."""
    return {"state_queries": int(data.get("state_queries") or 0)} if "state_queries" in data else {}


def verdict(data: dict) -> dict:
    live = data.get("graded_by") == "live"
    gave_up = bool(data.get("gave_up"))
    no_verdict = not data.get("finalized") or not (live or gave_up)
    mismatch = bool(data.get("instance_mismatch"))
    infra = bool(data.get("infra_error")) or no_verdict or mismatch
    # an episode a simulator error ended is never a success
    success = data.get("success") is True and live and not gave_up and not infra and data.get("reason") != "error"
    return {
        **BASE,
        "success": int(success),
        "live_success": int(data.get("success") is True),
        "graded_by_live": int(live),
        "replays": int(data.get("replays") or 0),
        "infra_error": int(infra),
        "sim_error": int(data.get("reason") == "error"),
        "finalized": int(bool(data.get("finalized"))),
        "no_verdict": int(no_verdict),
        "gave_up": int(gave_up),
        "instance_mismatch": int(mismatch),
        "video_rendered": int(bool(data.get("video"))),
        "n_actions": int(data.get("n_actions") or 0),
        "sim_steps": int(data.get("control_steps") or data.get("steps") or 0),
        "control_steps_total": int(data.get("control_steps_total") or 0),
        "calls": int(data.get("calls") or 0),
        "elapsed_s": float(data.get("elapsed_s") or 0),
        "sim_startups": int(data.get("sim_startups") or 0),
        "resumes": int(data.get("resumes") or 0),
        "resume_state_mismatch": int(data.get("resume_state_mismatch") or 0),
        "hand_wuji": int(data.get("hand") == "wuji"),
        "resets": int(data.get("resets") or 0),
        "reset_budget": int(data.get("reset_budget") or 0),
        **state_ops_fields(data, success),
        **state_query_fields(data),
    }


def main() -> None:
    directory = Path(os.environ.get("RCB_DATA", "/data"))
    log = Path(os.environ.get("RCB_VERIFIER_LOG_DIR", "/logs/verifier"))
    log.mkdir(parents=True, exist_ok=True)
    path = directory / "final.json"
    if not path.is_file():
        result = {**BASE, "infra_error": 1, "missing_log": 1}
    else:
        try:
            data = json.loads(path.read_text())
            if not isinstance(data, dict):
                raise ValueError("final.json must be an object")
            result = verdict(data)
        except (OSError, ValueError, TypeError):
            result = {**BASE, "infra_error": 1, "grader_error": 1, "corrupt_log": 1}
    for name in ("trajectory.npz", "episode.jsonl", "replay.mp4"):   # evidence for people; never the verdict
        if (directory / name).is_file():
            shutil.copyfile(directory / name, log / name)
    (log / "reward.txt").write_text(f"{result['success']}\n")
    (log / "reward.json").write_text(json.dumps(result, sort_keys=True))


if __name__ == "__main__":
    main()
