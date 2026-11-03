"""python -m rcb_ablation.serve (cwd /srv): core.serve's main() with the ablation's service class and HTTP layer."""
import core
import core.serve as S

from . import ANY, OPS
from . import httpx
from . import service

if OPS and not str(getattr(core, "VERSION", "")).startswith("0.2."):
    # reset, save and load restart the core-0.2 service's world thread and built-in seats; core 0.5 is not covered
    raise SystemExit(f"RCB_STATE_OPS needs core 0.2 (this is core {getattr(core, 'VERSION', '?')})")
if ANY:
    S.RealtimeService = service.make(S.RealtimeService)
    S.TurnService = service.make(S.TurnService)
    _make_server = S.make_server

    def make_server(svc, seats, hosts, port=8000):
        return httpx.patch(_make_server(svc, seats, hosts, port), svc, seats)

    S.make_server = make_server
    if hasattr(S, "builtin_seat"):
        service.patch_builtin(S.builtin_seat)

if __name__ == "__main__":
    S.main()
