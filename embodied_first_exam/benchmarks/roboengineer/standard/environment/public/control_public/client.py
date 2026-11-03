"""Agent-side policy client. Preserve the exact service bytes plus concise revision diagnoses."""

import base64, json, hashlib, os, urllib.request, urllib.error
from pathlib import Path


class Client:
    def __init__(self, url="http://127.0.0.1:8000", out="/app/output/trace", timeout_s=None):
        self.url = url
        self.timeout_s = float(
            timeout_s if timeout_s is not None else os.environ.get("CONTROL_DEPLOY_TIMEOUT_S", "1800")
        )
        if not 0 < self.timeout_s <= 129600:
            raise ValueError("Client request timeout")
        self.out = Path(out)
        self.out.mkdir(parents=True, exist_ok=True)

    def call(self, operation, arguments):
        request = json.dumps(arguments, allow_nan=False).encode()
        req = urllib.request.Request(
            self.url + "/" + operation, data=request, headers={"Content-Type": "application/json"}
        )
        # Allocate by requests so failed calls also retain distinct trace entries.
        indices = [
            int(p.name.split("-", 1)[0]) for p in self.out.glob("*.request.json") if p.name.split("-", 1)[0].isdigit()
        ]
        index = max(indices, default=-1) + 1
        prefix = self.out / f"{index:03d}-{operation.replace('/', '_')}"
        prefix.with_suffix(".request.json").write_bytes(request)
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as response:
                raw = response.read()
        except urllib.error.HTTPError as error:
            raw = error.read()
            prefix.with_suffix(".response.json").write_bytes(raw)
            prefix.with_suffix(".error.json").write_text(json.dumps({"http_status": error.code}))
            raise
        except urllib.error.URLError as error:
            prefix.with_suffix(".error.json").write_text(json.dumps({"transport_error": str(error.reason)}))
            raise
        prefix.with_suffix(".response.json").write_bytes(raw)
        result = json.loads(raw)
        # describe.feedback is documentation; only deploy returns episode payloads.
        episodes = result.get("feedback", []) if operation in ("deploy", "result") else []
        if not isinstance(episodes, list):
            raise ValueError("Invalid deployment feedback")
        for i, episode in enumerate(episodes):
            for j, frame in enumerate(episode["images"]):
                b = base64.b64decode(frame["base64"], validate=True)
                if hashlib.sha256(b).hexdigest() != frame["sha256"]:
                    raise ValueError("Frame checksum")
                (self.out / f"{index:03d}-case-{i}-frame-{j}.jpg").write_bytes(b)
        return result

    def describe(self):
        return self.call("describe", {})

    def deploy(self, policy, diagnosis):
        return self.call("deploy", {"policy": policy, "diagnosis": diagnosis})

    def seal(self, round_id, summary):
        return self.call("seal", {"round_id": round_id, "summary": summary})

    def result(self, round_id):
        """The stored response of a completed deployment round (when a deploy response was lost); runs nothing."""
        return self.call("result", {"round_id": round_id})


    # The ablations' calls (a run that offers them says so in its instruction and in /spec; elsewhere they are 404).
    def finish(self, round_id=None, summary=None):
        """No-success-signal runs: end adaptation; the given round (else the latest completed one) is graded."""
        args = {k: v for k, v in (("round_id", round_id), ("summary", summary)) if v is not None}
        return self.call("finish", args)

    def state_entities(self):
        """State-query runs: the entities and fields /state/query answers."""
        with urllib.request.urlopen(self.url + "/state/entities", timeout=self.timeout_s) as response:
            return json.loads(response.read())

    def state_query(self, names, fields=None, round_id=None, case=None, t=None):
        """State-query runs: the true state of a completed round's development episodes at their telemetry times."""
        args = {"names": list(names)}
        for k, v in (("fields", fields), ("round_id", round_id), ("case", case), ("t", t)):
            if v is not None:
                args[k] = list(v) if k == "fields" else v
        return self.call("state/query", args)

    def save_state(self, round_id, case, t):
        """Save/load runs: a handle for the state of a development episode at its telemetry time at or before t."""
        return self.call("save_state", {"round_id": round_id, "case": case, "t": t})

    def load_state(self, handle):
        """Save/load runs: the next deploy() runs that case from the saved state (None cancels)."""
        return self.call("load_state", {"handle": handle})


def artifact(path):
    return {"npz_base64": base64.b64encode(Path(path).read_bytes()).decode()}
