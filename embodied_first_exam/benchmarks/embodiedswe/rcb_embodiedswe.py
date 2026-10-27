"""EmbodiedSWE tasks in Embodied First Exam: frozen instances, the graded rollout of a delivered `solve(env)`, replay
and video. One file, in the image at /opt/efe/rcb_embodiedswe.py (with esw_proxy.py beside it).

Development (the agent, privileged mode: the full simulator):

    from rcb_embodiedswe import make_env
    env = make_env()                    # launches Isaac Sim (headless, PhysX on the CPU), builds and resets the task's
                                        # env from /app/instance.json exactly as grading does, checks the start state
    import solve; solve.solve(env)      # anything goes in development

    python /opt/efe/rcb_embodiedswe.py check --solution /app/output/solution
                                        # the graded flow without the hidden grader: solve runs as `nobody` in its own
                                        # process against a proxy env; prints the task's success predicate at the end

Grading (the verifier; root; no network):

    rcb_embodiedswe.py rollout --instance I --grader G --solution DIR --out OUT     the graded run (trusted sim process)
    rcb_embodiedswe.py replay  --instance I --grader G --traj OUT/actions.npz       a fresh process re-executes it
    rcb_embodiedswe.py video   --instance I --traj OUT/actions.npz --video V.mp4    the video, after the verdict

Building the task (maintainers): `rcb_embodiedswe.py make-instance --preset P --seed S --out instance.json`.

The rollout: build the env from the frozen instance (EmbodiedSWE's own build + seeded reset, `device="cpu"`: PhysX
on the CPU), check its state hash, construct the task's grader on it (EmbodiedSWE's rubric, recorded after every
step as their GradedEnv does), then start the solution process (esw_proxy.py) as `nobody` and serve its requests:
reads answered with copies, `env.step(action)` executed and recorded, everything else refused. The rollout ends when
`solve` returns or raises, its process dies, or the rollout time limit passes (the verdict is then read from the state
at that moment, as EmbodiedSWE's grader does at its budget kill). Every process of `nobody` is killed before the
verdict is read. Verdict: success = the grader's `check_success` on the final state (their end predicate); progress =
the grader's rubric score (its peak recorded progress).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

HERE = Path(__file__).resolve().parent
BENCH_ROOT = os.environ.get("ESW_BENCH_ROOT", "/opt/embodiedswe")
NOBODY = 65534
DEVICE = "cpu"                     # PhysX on the CPU: a fresh process replays bit for bit (measured per task)
MAX_STEPS = 400_000                # runaway guard on control steps in one rollout (a technical cap, not a budget)
ROLLOUT_TIMEOUT_S = 1800.0         # EmbodiedSWE's rollout limit (30 min of solve wall clock)
LOAD_TIMEOUT_S = 300.0             # importing solve.py
INSTANCE_FORMAT = "esw-instance/1"

# The tasks of this benchmark: preset -> the module and class of the grader its tests/ ship, and the scene's own
# public success function (what `check` reports without the hidden grader; equal to the grader's check_success).
TASKS = {
    "assembly.bulb.franka.osc": {"grader": "BulbAssemblyGrader", "check": "success"},
    "puzzle.stack_blocks.g1.pink_ik": {"grader": "StackBlocksGrader", "check": "success"},
    "locomanip.fruit_delivery.g1.loco_pink_ik": {"grader": "FruitDeliveryGrader", "check": "success"},
    "assembly.allen_bolt.franka.osc": {"grader": "AllenBoltAssemblyGrader", "check": "success"},
    "assembly.ikea_table.bimanual_franka.osc": {"grader": "IkeaTableAssemblyGrader", "check": "success"},
    "assembly.nut_thread.franka.osc": {"grader": "NutThreadAssemblyGrader", "check": "success"},
    "assembly.pc_gpu.franka.osc": {"grader": "PcGpuAssemblyGrader", "check": "success"},
    "assembly.pc_gpu_ram.franka.osc": {"grader": "PcGpuRamAssemblyGrader", "check": "success"},
    "assembly.pc_motherboard.franka.osc": {"grader": "PcMotherboardAssemblyGrader", "check": "success"},
    "assembly.pc_ram.franka.osc": {"grader": "PcRamAssemblyGrader", "check": "success"},
    "packing.pen_holder.franka.osc": {"grader": "PenHolderGrader", "check": "success"},
    "packing.tool_packing.franka.osc": {"grader": "ToolPackingGrader", "check": "success"},
    "packing.egg_carton.g1.pink_ik": {"grader": "EggCartonGrader", "check": "success"},
    "packing.clear_organic_objects.g1.pink_ik": {"grader": "ClearOrganicObjectsGrader", "check": "success"},
    "puzzle.coffee.franka.osc": {"grader": "CoffeeServiceGrader", "check": "success"},
    "puzzle.syringe.bimanual_franka.osc": {"grader": "SyringeDosingGrader", "check": "success"},
    "puzzle.push_shapes.g1.pink_ik": {"grader": "PushShapesGrader", "check": "success"},
    "puzzle.classify_objects.g1.pink_ik": {"grader": "ClassifyObjectsGrader", "check": "success"},
    "cutting.slice.franka.osc": {"grader": "SliceFoodGrader", "check": "cut_success"},   # the scene has no success()
    "cutting.dice.franka.osc": {"grader": "DiceFoodGrader", "check": "cut_success"},
    "locomanip.box_to_bin.g1.loco_pink_ik": {"grader": "BoxToBinGrader", "check": "success"},
    "locomanip.wheel_carry.g1.loco_pink_ik": {"grader": "WheelCarryGrader", "check": "success"},
}


def cut_success(scene):
    """The cutting scenes' public end predicate (they have no success()): the food is in at least its target number
    of pieces (`cfg.target_pieces`, 0 = every piece) and every piece's root is still on the chopping board, inside its
    footprint (`cfg.board_size`, centred on the env origin) and above its top. `(num_envs,)` bool."""
    import torch
    target = scene.cfg.target_pieces or len(scene.pieces)
    hx, hy = scene.cfg.board_size[0] / 2, scene.cfg.board_size[1] / 2
    ok = torch.ones(scene.env.num_envs, dtype=torch.bool, device=scene.env.device)
    for pc in scene.pieces:
        rel = pc.data.root_pos_w - scene.env_origins
        ok &= (rel[:, 0].abs() < hx) & (rel[:, 1].abs() < hy) & (rel[:, 2] > scene._board_top)
    return (scene.pieces_count() >= target) & ok


CHECKS = {"cut_success": cut_success}


def task_success(env, preset: str):
    """The task's public end predicate, `(num_envs,)` bool: the scene's own method, or the tooling's for scenes
    without one (CHECKS)."""
    name = TASKS[preset]["check"]
    return CHECKS[name](env.scene) if name in CHECKS else getattr(env.scene, name)()


def log(*a) -> None:
    print("[esw]", *a, file=sys.stderr, flush=True)


# ------------------------------------------------------------------------------------------------ simulator
_APP = None


def launch(cameras: bool = False):
    """Isaac Sim headless (once per process). pinocchio first: the pink_ik presets need it imported before Kit."""
    global _APP
    if _APP is not None:
        return _APP
    try:
        import pinocchio  # noqa: F401
    except ImportError as exc:
        log(f"pinocchio not importable ({exc}): a pink_ik preset will fail to build")
    from isaaclab.app import AppLauncher
    kw = dict(headless=True, device=DEVICE)
    if cameras:
        kw["enable_cameras"] = True
    _APP = AppLauncher(**kw).app
    return _APP


def build(instance: dict):
    """The env exactly as EmbodiedSWE's grader builds it (registered preset, seed, one trajectory), PhysX on the CPU."""
    if BENCH_ROOT not in sys.path:
        sys.path.insert(0, BENCH_ROOT)
    import robobench
    robobench.discover()
    from robobench.core.registries import ENVS
    seed = int(instance["seed"])
    if instance.get("device", DEVICE) != DEVICE:
        raise ValueError(f"instance built on {instance.get('device')!r}; these tasks run PhysX (and the G1 locomotion "
                         f"policy) on {DEVICE!r} only: another device gives another start state and other bits")
    env = ENVS.get(instance["preset"])().build(num_envs=1, seed=seed, device=DEVICE)
    if str(env.device) != DEVICE:
        raise RuntimeError(f"the env was built on {env.device}, not {DEVICE}")
    env.reset(seed=seed)
    return env


def _leaves(obj, prefix=""):
    """(path, tensor-or-scalar) leaves of a nested state, in a fixed order."""
    if isinstance(obj, dict):
        for k in sorted(obj, key=str):
            yield from _leaves(obj[k], f"{prefix}/{k}")
    elif isinstance(obj, (list, tuple)):
        for i, x in enumerate(obj):
            yield from _leaves(x, f"{prefix}/{i}")
    else:
        yield prefix, obj


def full_state(env) -> dict:
    """Everything that defines where the episode is: every asset's state in Isaac Lab's scene (root poses and
    velocities, joint positions and velocities) and robobench's own `get_states()` (scene + robot)."""
    return {"iscene": env.iscene.get_state(is_relative=False), "robobench": env.get_states()}


def state_sha(env) -> str:
    import numpy as np
    import torch
    h = hashlib.sha256()
    for path, v in _leaves(full_state(env)):
        h.update(path.encode())
        if isinstance(v, torch.Tensor):
            a = v.detach().to("cpu").contiguous()
            h.update(str(a.dtype).encode() + str(tuple(a.shape)).encode())
            h.update(a.numpy().tobytes() if a.dtype != torch.bfloat16 else a.float().numpy().tobytes())
        elif isinstance(v, np.ndarray):
            h.update(np.ascontiguousarray(v).tobytes())
        else:
            h.update(repr(v).encode())
    return h.hexdigest()


def load_instance(path) -> dict:
    inst = json.loads(Path(path).read_text())
    if inst.get("format") != INSTANCE_FORMAT or inst.get("preset") not in TASKS:
        raise ValueError(f"{path}: not an instance of this benchmark")
    return inst


def load_grader(path: str, preset: str):
    """The task's grader module (from the verifier's tests/, never from the agent's image), by file path."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("esw_task_grader", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return getattr(mod, TASKS[preset]["grader"])


def stop_app(rc: int = 0):
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(rc)                    # Kit can hang on close; everything is written by now


# ------------------------------------------------------------------------------------------------ the proxy server
WRITE_RE = re.compile(r"^_*(set|write|reset|apply|step|update|bind|post_step|close|clear|spawn|add|remove|delete|"
                      r"create|enable|disable|load|save|play|pause|stop|render|teleport|register|attach|detach|define|"
                      r"init|initialize|sim_cfg|assets|compute|record|verdict|seal|kill|exec|run|start|open|"
                      r"to_isaaclab|build|fetch|ensure|prepare|wait|override|patch|swap|replace|move|push|pop|"
                      r"insert|append|extend|fill|zero|copy_|scale|mul|div|sub|clamp_|index_|scatter|put|resize)",
                      re.IGNORECASE)
READ_RE = re.compile(r"^_*(get|is|has|find|describe|seated|success|score|tower_height|level_counts|slots|upright|"
                     r"block_tensors|fruits_|nonfruits_|on_plate|placed|speed|ground_z|yaw_quat|grid_names|slot_xy|"
                     r"fixed_pose|radial|plate_|bulb_|in_socket|stand_xy|keys|values|items|names|count|num|name|"
                     r"prim_name|action_dim|control_period|control_dt|actuator_limits|joint_names|body_names)",
                     re.IGNORECASE)
# Calls the proxy answers: a method whose name says it reads (READ_RE) and not that it writes (WRITE_RE), on any object
# reachable from the env, plus ENV_CALLS on the env itself. A whitelist: an unknown verb is refused. Audited against
# the three tasks' scene and robot classes (and BaseScene, BaseRobot); on_bind / actuator_sink / build_controller are
# refused (they rebind or hand out the write path).
ENV_CALLS = {"describe", "describe_stage", "get_states"}

# Strict, audited read-method allowlist for the privileged objects that hold ground truth (scene, robot).
# Default deny: a method on a scene/robot that is not listed here is refused even if its name looks like a
# read (the regex above is kept only as a secondary gate for leaf objects such as configs and articulations).
# Maintainers EXTEND these per task after auditing the new scene/robot class (not a code change for each run).
READ_METHODS_SCENE = {
    "get_state", "describe", "seated", "success", "score", "tower_height",
    "_bulb_offsets_in_socket", "_bulb_axis_cos", "_block_tensors", "_upright", "_slots", "_level_counts",
    # assembly scenes (allen_bolt, ikea_table, nut_thread, pc_gpu, pc_gpu_ram, pc_motherboard, pc_ram)
    "grasp_sites", "engaged", "gpu_engaged", "gpu_seated", "ram_engaged", "ram_seated",
    "_bolt_offsets_in_platform", "_bolt_axis_cos", "_bolt_offsets_in_case", "_leg_offsets_in_table", "_leg_axis_cos",
    "_weld_targets", "_nut_offsets_in_bolt", "_nut_axis_cos", "_card_offset_in_case", "_ram_offsets_in_case",
    "_axis_cos", "_part_axis_cos",
    # packing scenes (pen_holder, tool_packing, egg_carton, clear_organic_objects)
    "holder_up", "inserted", "settled", "counted", "all_inserted", "holder_placed", "_pen_tensors", "_pen_ends_local",
    "drawer_pos", "door_pos", "stowed", "drawers_closed", "doors_closed", "_slot_xy", "_item_in_tray",
    "lid_pos", "lid_closed", "occupied", "_seat_candidates", "_stages_complete",
    "cleared", "placed_on_tray", "stage_flags", "organics_cleared", "distractors_in_bin", "organics_present",
    "_in_bin", "_on_tray", "_speed",
    # puzzle scenes (coffee, syringe, push_shapes, classify_objects); syringe's fixture() writes and stays refused
    "cover_pos", "cover_closed", "cover_open", "button_depth", "pod_seated", "cup_under_spout", "cup_on_tray",
    "cup_settled", "running", "filled", "stage_flags",
    "barrel_axis", "travel", "tip_pos", "plunger_top_w", "seated_reservoir", "well_mouths", "seated_well", "liquid",
    "doses", "spilled", "drawn", "doses_ok", "parked",
    "position_error", "orientation_error", "block_bottom_z", "not_lifted", "dropped", "yaw_matched", "stage_live",
    "sorted_mask", "n_sorted",
    # cutting scenes (slice, dice); knife_edge only fills a lazy cache of edge samples
    "pieces_count", "knife_edge", "food_height", "plane_aim_w", "plane_axis_w",
    # locomanip scenes (box_to_bin, wheel_carry)
    "placed", "carried_fraction", "box_pose", "wheel_pose",
}
READ_METHODS_ROBOT = {
    "get_state", "describe", "name", "prim_name", "action_dim", "control_period", "control_dt",
    "actuator_limits", "joint_names", "body_names",
}
MAX_HANDLES = 500_000              # cap the trusted-side handle table (a solution cannot grow it without bound)


class Refused(Exception):
    pass


class Server:
    """Answers one solution process's requests on the trusted env. Objects stay here; the other side gets copies of
    data and integer handles for everything else."""

    def __init__(self, env, on_step):
        self.env = env
        self.on_step = on_step           # callable(action tensor) -> None; raises StopRollout to end
        self.objs: dict[int, tuple] = {0: ("obj", env)}
        self.ids: dict[int, int] = {id(env): 0}
        self.method_ids: dict[tuple, int] = {}
        self.n_requests = 0
        self.n_refused = 0
        self.refusals: list[str] = []
        from robobench.core.scene import BaseScene
        from robobench.core.robot import BaseRobot
        self._scene_types = (BaseScene,)
        self._robot_types = (BaseRobot,)

    # -- handles
    def handle_of(self, obj) -> dict:
        import types
        if isinstance(obj, types.ModuleType):
            raise Refused("modules are not reachable from the graded env")
        key = id(obj)
        h = self.ids.get(key)
        if h is None or self.objs[h][1] is not obj:
            if len(self.objs) >= MAX_HANDLES:
                raise StopRollout(f"handle_cap ({MAX_HANDLES} handles)")
            h = len(self.objs)
            self.objs[h] = ("obj", obj)
            self.ids[key] = h
        return {"__h__": h, "type": type(obj).__name__, "callable": callable(obj)}

    def method_handle(self, owner, name: str) -> dict:
        key = (id(owner), name)                       # dedup: reading the same method twice reuses its handle
        h = self.method_ids.get(key)
        if h is None or self.objs.get(h) != ("method", owner, name):
            if len(self.objs) >= MAX_HANDLES:
                raise StopRollout(f"handle_cap ({MAX_HANDLES} handles)")
            h = len(self.objs)
            self.objs[h] = ("method", owner, name)
            self.method_ids[key] = h
        return {"__h__": h, "type": f"{type(owner).__name__}.{name}", "callable": True}

    def resolve(self, d: dict):
        h = int(d["__h__"])
        if h not in self.objs:
            raise Refused(f"unknown handle {h}")
        ent = self.objs[h]
        if ent[0] == "method":
            raise Refused("a method cannot be passed as an argument")
        return ent[1]

    def value(self, v):
        from esw_proxy import encode
        return encode(v, self.handle_of)

    # -- policy
    def allowed_call(self, owner, name: str) -> bool:
        if owner is self.env:
            return name in ENV_CALLS
        if name in ("on_bind", "actuator_sink", "build_controller"):
            return False
        # privileged ground-truth objects: the audited exact-name allowlist alone decides (a listed read such as
        # `settled` or `inserted` must not fall to the write-verb regex)
        if isinstance(owner, self._scene_types):
            return name in READ_METHODS_SCENE
        if isinstance(owner, self._robot_types):
            return name in READ_METHODS_ROBOT
        if WRITE_RE.match(name):                       # leaf objects (configs, articulations): name heuristic
            return False
        return bool(READ_RE.match(name))

    def getattr(self, obj, name: str):
        if not isinstance(name, str) or (name.startswith("__") and name.endswith("__")) or name in ("_grader", "_env"):
            raise Refused(f"attribute {name!r} is not readable during grading")
        v = getattr(obj, name)
        import inspect
        if inspect.ismethod(v) or inspect.isbuiltin(v) or inspect.ismethoddescriptor(v) or inspect.isfunction(v) \
                or type(v).__name__ in ("method-wrapper", "builtin_function_or_method"):
            return self.method_handle(obj, name)
        return self.value(v)

    def handle(self, req: dict):
        from esw_proxy import decode
        self.n_requests += 1
        op = req.get("op")
        if op == "step":
            import torch
            a = decode(req.get("action"), self.resolve)
            a = torch.as_tensor(a, dtype=torch.float32, device="cpu")
            return self.on_step(a)
        h = int(req.get("h", -1))
        if h not in self.objs:
            raise Refused(f"unknown handle {h}")
        ent = self.objs[h]
        if op == "call":
            if ent[0] != "method":
                raise Refused(f"{type(ent[1]).__name__} objects are not callable during grading")
            owner, name = ent[1], ent[2]
            if owner is self.env and name == "step":
                args = decode(req.get("args"), self.resolve)
                return self.on_step(__import__("torch").as_tensor(args[0], dtype=__import__("torch").float32))
            if owner is self.env and name in ("reset", "set_states"):
                raise PermissionError("blocked during grading — solve may only step forward")
            if not self.allowed_call(owner, name):
                raise PermissionError(f"blocked during grading: {type(owner).__name__}.{name}() is not a read")
            args = decode(req.get("args"), self.resolve)
            kwargs = decode(req.get("kwargs"), self.resolve)
            return self.value(getattr(owner, name)(*args, **kwargs))
        if ent[0] == "method":
            raise Refused("a method handle supports calls only")
        obj = ent[1]
        if op == "getattr":
            return self.getattr(obj, req.get("name"))
        if op == "getitem":
            return self.value(obj[decode(req.get("key"), self.resolve)])
        if op == "len":
            return len(obj)
        if op == "iter":
            import itertools
            return self.value(list(itertools.islice(obj, 100_000)))  # never materialize the whole iterable
        if op == "contains":
            return decode(req.get("item"), self.resolve) in obj
        if op == "bool":
            return bool(obj)
        if op == "repr":
            return repr(obj)[:20000]
        if op == "dir":
            return self.value([n for n in dir(obj) if not (n.startswith("__") and n.endswith("__"))])
        raise Refused(f"unknown operation {op!r}")


class StopRollout(Exception):
    pass


def _hide_other_procs() -> None:
    """Belt to Landlock's suspenders: give the solution a private mount namespace with /proc remounted
    hidepid=2, so other processes' /proc entries (cmdline, environ, fd, mem of the trusted sim) are hidden
    even on a kernel where Landlock is weaker or absent. Best effort: a verifier container without
    CAP_SYS_ADMIN gets EPERM and this is a no-op (Landlock stays the guarantee, re-checked by
    esw_proxy._verify_sandbox). Runs while still root, before the setuid below."""
    import ctypes
    libc = ctypes.CDLL(None, use_errno=True)
    CLONE_NEWNS = 0x00020000
    MS_REC, MS_PRIVATE = 0x4000, 1 << 18
    if libc.unshare(CLONE_NEWNS) != 0:
        return
    libc.mount(None, b"/", None, MS_REC | MS_PRIVATE, None)
    libc.mount(b"proc", b"/proc", b"proc", 0, b"hidepid=2")


def _preexec_nobody():
    os.setsid()
    try:
        _hide_other_procs()
    except Exception:  # noqa: BLE001 — best effort; Landlock + _verify_sandbox are the guarantee
        pass
    os.setgroups([])
    os.setgid(NOBODY)
    os.setuid(NOBODY)


def kill_uid(uid: int = NOBODY) -> None:
    """End every process of `uid` (the solution and anything it started), from a child that runs as that uid."""
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


def prepare_solution(src: Path, dst: Path) -> None:
    """Copy the solution's regular files (no links, no special files) to a root-owned tree `nobody` can only read."""
    import shutil
    if dst.exists():
        shutil.rmtree(dst)
    dst.mkdir(parents=True)
    n = size = 0
    for root, dirs, files in os.walk(src, followlinks=False):
        rel = Path(root).relative_to(src)
        for d in dirs:
            p = Path(root) / d
            if not p.is_symlink():
                (dst / rel / d).mkdir(exist_ok=True)
        for f in files:
            p = Path(root) / f
            if p.is_symlink() or not p.is_file():
                continue
            n += 1
            size += p.stat().st_size
            if n > 20_000 or size > (2 << 30):
                raise ValueError("solution larger than 20,000 files or 2 GiB")
            shutil.copyfile(p, dst / rel / f)
    for root, dirs, files in os.walk(dst):
        os.chmod(root, 0o755)
        for f in files:
            os.chmod(Path(root) / f, 0o644)


def run_solution(env, solution: Path, work: Path, on_step, timeout_s: float, log_path: Path) -> dict:
    """Start the solution process as `nobody` and serve it until solve ends, it dies, or `timeout_s` passes."""
    from esw_proxy import LANDLOCK_ABI_MIN, Wire, landlock_abi
    if landlock_abi() < LANDLOCK_ABI_MIN:
        raise RuntimeError(f"this kernel offers no Landlock ABI >= {LANDLOCK_ABI_MIN}: the solution cannot be sandboxed")
    sol = work / "solution"
    prepare_solution(solution, sol)
    scratch = work / "scratch"
    scratch.mkdir(parents=True, exist_ok=True)
    root = os.geteuid() == 0
    if root:
        os.chown(scratch, NOBODY, NOBODY)
        os.chmod(work, 0o711)                         # `nobody` may pass through to solution/ and scratch/, not list
    # the solution's two pipes: it reads requests' answers on c_in, writes requests on c_out
    p_r, c_w = os.pipe()        # child -> parent
    c_r, p_w = os.pipe()        # parent -> child
    envv = {"PATH": "/opt/robolab/.venv/bin:/usr/local/bin:/usr/bin:/bin", "HOME": str(scratch),
            "TMPDIR": str(scratch), "LANG": "C.UTF-8", "PYTHONDONTWRITEBYTECODE": "1", "PYTHONHASHSEED": "0",
            "CUDA_VISIBLE_DEVICES": "", "NVIDIA_VISIBLE_DEVICES": "", "OMP_NUM_THREADS": "4",
            "MKL_NUM_THREADS": "4", "OPENBLAS_NUM_THREADS": "4"}
    logf = open(log_path, "ab")
    os.chmod(log_path, 0o600)
    cmd = [sys.executable, "-I", "-B", str(HERE / "esw_proxy.py"), str(c_r), str(c_w), str(sol), BENCH_ROOT]
    proc = subprocess.Popen(cmd, stdin=subprocess.DEVNULL, stdout=logf, stderr=logf, env=envv, cwd=str(scratch),
                            pass_fds=(c_r, c_w), preexec_fn=_preexec_nobody if root else os.setsid)
    os.close(c_r)
    os.close(c_w)
    wire = Wire(p_r, p_w)
    srv = Server(env, on_step)
    out = {"solve_status": "running", "requests": 0, "refused": 0}
    deadline = [time.monotonic() + LOAD_TIMEOUT_S]
    timer_fired = threading.Event()

    def watchdog():
        while proc.poll() is None:
            if time.monotonic() > deadline[0]:
                timer_fired.set()
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except (ProcessLookupError, PermissionError):
                    pass
                kill_uid()
                return
            time.sleep(0.2)
    threading.Thread(target=watchdog, daemon=True).start()
    t0 = None
    try:
        wire.send({"op": "hello", "root": srv.handle_of(env), "protocol": "esw-proxy/1", "scratch": str(scratch)})
        while True:
            try:
                req = wire.recv()
            except (EOFError, ValueError, OSError) as exc:
                out["solve_status"] = "timeout" if timer_fired.is_set() else "process_ended"
                out["solve_error"] = None if timer_fired.is_set() else f"connection: {exc}"
                break
            if not isinstance(req, dict):
                wire.send({"error": "malformed request", "kind": "ValueError"})
                continue
            op = req.get("op")
            if op == "loaded":
                t0 = time.monotonic()
                deadline[0] = t0 + timeout_s
                out["load_s"] = None
                wire.send({"value": None})
                continue
            if op == "done":
                if req.get("sandbox_failed"):
                    out["solve_status"] = "sandbox_failed"
                    out["sandbox_error"] = str(req["sandbox_failed"])[:500]
                    break
                out["solve_status"] = "error" if req.get("error") else "returned"
                out["solve_error"] = req.get("error")
                out["sandbox"] = req.get("sandbox")
                break
            try:
                v = srv.handle(req)
                wire.send({"value": v if op != "step" else None})
            except StopRollout as exc:
                out["solve_status"] = str(exc)
                wire.send({"error": str(exc), "kind": "RuntimeError", "stop": True})
                break
            except (Refused, PermissionError) as exc:
                srv.n_refused += 1
                if len(srv.refusals) < 50:
                    srv.refusals.append(f"{req.get('op')}:{req.get('name', '')}: {exc}"[:300])
                wire.send({"error": str(exc), "kind": "PermissionError"})
            except Exception as exc:  # noqa: BLE001 — the solve's error (a bad attribute, a bad index) is its own
                wire.send({"error": f"{type(exc).__name__}: {exc}"[:4000], "kind": type(exc).__name__})
            if time.monotonic() > deadline[0]:
                out["solve_status"] = "timeout"
                break
    finally:
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError):
            pass
        kill_uid()                                    # every process the solution started ends before the verdict
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass
        logf.close()
    out.update(requests=srv.n_requests, refused=srv.n_refused, refusals=srv.refusals,
               solve_wall_s=round(time.monotonic() - t0, 3) if t0 else 0.0)
    if out["solve_status"] == "running":
        out["solve_status"] = "process_ended"
    return out


# ------------------------------------------------------------------------------------------------ commands
def cmd_make_instance(a) -> int:
    launch()
    inst = {"format": INSTANCE_FORMAT, "preset": a.preset, "seed": int(a.seed), "device": DEVICE}
    try:
        env = build(inst)
    except Exception:  # noqa: BLE001 — a failed build writes no instance and exits non-zero
        traceback.print_exc()
        stop_app(1)
    inst["state_sha"] = state_sha(env)
    inst["action_dim"] = int(env.robot.action_dim)
    inst["control_dt"] = round(float(env.dt * env.robot.control_period), 6)
    inst["physics_dt"] = float(env.dt)
    Path(a.out).write_text(json.dumps(inst, indent=1, sort_keys=True) + "\n")
    if a.describe:
        Path(a.describe).write_text(env.describe().strip() + "\n")
    print(json.dumps(inst), flush=True)
    stop_app()
    return 0


def _graded_env(inst: dict, grader_path: str | None):
    env = build(inst)
    sha = state_sha(env)
    grader = load_grader(grader_path, inst["preset"])(env) if grader_path else None
    return env, grader, sha


def _final(env, grader, inst: dict) -> dict:
    if grader is not None:
        v = grader.verdict()[0]
        success, score = bool(v["success"]), float(v["score"])
    else:
        success, score = bool(task_success(env, inst["preset"])[0]), None
    return {"success": success, "progress": score, "final_sha": state_sha(env)}


def cmd_rollout(a) -> int:
    """The graded run. Prints one JSON line (also written to OUT/rollout.json); the actions go to OUT/actions.npz."""
    import numpy as np
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res: dict = {"phase": "boot"}
    t_boot = time.monotonic()
    try:
        launch()
        inst = load_instance(a.instance)
        env, grader, sha = _graded_env(inst, a.grader)
        res.update(instance_sha=sha, instance_ok=sha == inst["state_sha"], boot_s=round(time.monotonic() - t_boot, 1))
        if not res["instance_ok"]:
            res.update(phase="instance_mismatch", infra_error=1)
            raise StopRollout("instance_mismatch")
        actions: list = []
        step_s = [0.0]
        bad = [0]
        step_err = [None]

        def on_step(act):
            if act.numel() != env.num_envs * env.robot.action_dim:
                bad[0] += 1
                raise ValueError(f"action has {act.numel()} values; the env takes ({env.num_envs}, "
                                 f"{env.robot.action_dim})")
            if not bool(__import__("torch").isfinite(act).all()):
                bad[0] += 1
                raise ValueError("action contains NaN or inf")
            if len(actions) >= MAX_STEPS:
                raise StopRollout(f"step_cap ({MAX_STEPS} control steps)")
            act = act.reshape(env.num_envs, env.robot.action_dim).contiguous()
            actions.append(act[0].numpy().copy())     # recorded BEFORE the step: a step that raises is in the record
            t = time.perf_counter()
            try:
                env.step(act.to(env.device))
                if grader is not None:
                    grader.record()
            except Exception as exc:  # noqa: BLE001 — the env could not take this action (e.g. a controller
                step_err[0] = f"{type(exc).__name__}: {exc}"[:1000]   # singularity): the rollout ends here
                raise StopRollout("step_error") from exc
            step_s[0] += time.perf_counter() - t
            return None

        res["phase"] = "solve"
        run = run_solution(env, Path(a.solution), out / "work", on_step, float(a.timeout), Path(a.solve_log))
        res.update(run)
        if run.get("solve_status") == "sandbox_failed":
            res.update(phase="sandbox_failed", infra_error=1)
            raise StopRollout("sandbox_failed")
        if step_err[0] is not None:
            res["step_error"] = step_err[0]
            try:
                res.update(_final(env, grader, inst))
            except Exception as exc:  # noqa: BLE001 — the env may be unusable after the failed step; no verdict
                res.update(success=False, progress=0.0, final_sha=None, final_error=f"{type(exc).__name__}: {exc}"[:500])
        else:
            res.update(_final(env, grader, inst))
        A = np.stack(actions).astype(np.float32) if actions else np.zeros((0, env.robot.action_dim), np.float32)
        np.savez(out / "actions.npz", task=np.array(inst["preset"]), seed=np.array(inst["seed"]), actions=A)
        res.update(phase="done", n_actions=int(len(A)), bad_actions=bad[0], sim_step_s=round(step_s[0], 3),
                   ms_per_step=round(1000 * step_s[0] / max(1, len(A)), 3), infra_error=0)
    except StopRollout:
        pass
    except Exception:  # noqa: BLE001
        res.update(infra_error=1, error=traceback.format_exc()[-4000:])
    res["wall_s"] = round(time.monotonic() - t_boot, 1)
    (out / "rollout.json").write_text(json.dumps(res, indent=1) + "\n")
    print(json.dumps(res), flush=True)
    stop_app()
    return 0


def _load_actions(path: str, preset: str, action_dim: int):
    import numpy as np
    d = np.load(path, allow_pickle=False)
    if str(d["task"]) != preset:
        raise ValueError("trajectory of another task")
    A = np.asarray(d["actions"], dtype=np.float32)
    if A.ndim != 2 or A.shape[1] != action_dim or len(A) > MAX_STEPS or not np.isfinite(A).all():
        raise ValueError("malformed trajectory")
    return A


def cmd_replay(a) -> int:
    """Re-execute a recorded rollout in this fresh process (no solution code). One JSON line."""
    import torch
    res: dict = {}
    t0 = time.monotonic()
    try:
        launch()
        inst = load_instance(a.instance)
        env, grader, sha = _graded_env(inst, a.grader)
        res.update(instance_sha=sha, instance_ok=sha == inst["state_sha"])
        A = _load_actions(a.traj, inst["preset"], env.robot.action_dim)
        t1 = time.perf_counter()
        for i, row in enumerate(A):
            try:
                env.step(torch.from_numpy(row).reshape(1, -1).to(env.device))
                if grader is not None:
                    grader.record()
            except Exception as exc:  # noqa: BLE001 — the recorded action the rollout's env also failed on
                res.update(step_error=f"{type(exc).__name__}: {exc}"[:1000], step_error_at=i)
                break
        res.update(_final(env, grader, inst), n_actions=int(len(A)), replay_step_s=round(time.perf_counter() - t1, 3))
    except Exception:  # noqa: BLE001
        res.update(error=traceback.format_exc()[-4000:])
    res["wall_s"] = round(time.monotonic() - t0, 1)
    print(json.dumps(res), flush=True)
    stop_app()
    return 0


def cmd_video(a) -> int:
    """Render a recorded rollout (a fresh replay with a camera) to an mp4. Never part of a verdict."""
    import numpy as np
    import torch
    res: dict = {}
    t0 = time.monotonic()
    try:
        launch(cameras=True)
        import imageio.v2 as imageio
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import Camera, CameraCfg
        inst = load_instance(a.instance)
        if BENCH_ROOT not in sys.path:
            sys.path.insert(0, BENCH_ROOT)
        import robobench
        robobench.discover()
        from robobench.core.registries import ENVS
        seed = int(inst["seed"])
        env = ENVS.get(inst["preset"])().build(num_envs=1, seed=seed, device=DEVICE)
        cam = Camera(CameraCfg(prim_path="/World/esw_cam", update_period=0.0, height=a.height, width=a.width,
                               data_types=["rgb"], spawn=sim_utils.PinholeCameraCfg(focal_length=18.0,
                                                                                  clipping_range=(0.01, 100.0))))
        env.sim.reset()                                     # re-parse so the camera exists, then the graded reset
        env.reset(seed=seed)
        res["instance_ok"] = state_sha(env) == inst["state_sha"]
        eye, target = (env.room or {}).get("camera") or ((1.6, -1.6, 1.4), (0.3, 0.0, 0.5))
        if a.eye and a.target:
            eye, target = [float(v) for v in a.eye.split(",")], [float(v) for v in a.target.split(",")]
        cam.set_world_poses_from_view(torch.tensor([list(eye)]), torch.tensor([list(target)]))
        A = _load_actions(a.traj, inst["preset"], env.robot.action_dim)
        control_dt = float(env.dt * env.robot.control_period)
        every = max(1, int(round(1.0 / (a.fps * control_dt))))
        if len(A) / every > a.max_frames:                   # a long rollout becomes a time lapse
            every = int(np.ceil(len(A) / a.max_frames))
        w = imageio.get_writer(a.video, fps=a.fps, codec="libx264", quality=7, macro_block_size=8)
        n = 0
        for i, row in enumerate(A):
            grab = (i % every == 0) or i == len(A) - 1
            try:
                env.step(torch.from_numpy(row).reshape(1, -1), render=grab)
            except Exception as exc:  # noqa: BLE001 — the rollout ended on this step error too; the video stops here
                res.update(step_error=f"{type(exc).__name__}: {exc}"[:500], step_error_at=i)
                break
            if grab:
                cam.update(env.dt)
                f = cam.data.output["rgb"][0].detach().cpu().numpy()[..., :3]
                if f.size and f.any():
                    w.append_data(f)
                    n += 1
        w.close()
        try:
            fsha = state_sha(env)
        except Exception:  # noqa: BLE001
            fsha = None
        res.update(video=a.video, frames=n, every=every, final_sha=fsha, video_ok=n > 0)
    except Exception:  # noqa: BLE001
        res.update(video_ok=False, error=traceback.format_exc()[-4000:])
    res["wall_s"] = round(time.monotonic() - t0, 1)
    print(json.dumps(res), flush=True)
    stop_app()
    return 0


def cmd_check(a) -> int:
    """The graded flow without the hidden grader (development): solve as `nobody` through the proxy."""
    a.grader = None
    a.out = a.out or "/tmp/esw_check"
    a.solve_log = a.solve_log or str(Path(a.out) / "solve.log")
    Path(a.out).mkdir(parents=True, exist_ok=True)
    return cmd_rollout(a)


# ------------------------------------------------------------------------------------------------ development API
def make_env(instance: str = "/app/instance.json", cameras: bool = False):
    """The task's env for development, built and reset exactly as grading builds it (then yours to use freely).
    Launches Isaac Sim (headless, PhysX on the CPU) first if this process has not."""
    launch(cameras=cameras)
    inst = load_instance(instance)
    env = build(inst)
    sha = state_sha(env)
    if sha != inst["state_sha"]:
        log(f"warning: the start state differs from the frozen instance ({sha[:12]} != {inst['state_sha'][:12]})")
    return env


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("make-instance")
    p.add_argument("--preset", required=True, choices=sorted(TASKS))
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", required=True)
    p.add_argument("--describe", default=None, help="also write env.describe() here (the task's instruction text)")
    for name in ("rollout", "check"):
        p = sub.add_parser(name)
        p.add_argument("--instance", default="/app/instance.json")
        if name == "rollout":
            p.add_argument("--grader", required=True)
        p.add_argument("--solution", default="/app/output/solution")
        p.add_argument("--out", default=None if name == "check" else "/var/lib/esw/rollout")
        p.add_argument("--timeout", type=float, default=ROLLOUT_TIMEOUT_S)
        p.add_argument("--solve-log", default=None)
    p = sub.add_parser("replay")
    p.add_argument("--instance", default="/app/instance.json")
    p.add_argument("--grader", default=None)
    p.add_argument("--traj", required=True)
    p = sub.add_parser("video")
    p.add_argument("--instance", default="/app/instance.json")
    p.add_argument("--traj", required=True)
    p.add_argument("--video", required=True)
    p.add_argument("--fps", type=float, default=10.0)
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    p.add_argument("--max-frames", type=int, default=9000)
    p.add_argument("--eye", default=None, help="camera eye x,y,z (env frame); default: the room's camera")
    p.add_argument("--target", default=None, help="camera target x,y,z")
    a = ap.parse_args(argv)
    if a.cmd == "rollout" and not a.solve_log:
        a.solve_log = str(Path(a.out) / "solve.log")
    if HERE.as_posix() not in sys.path:
        sys.path.insert(0, HERE.as_posix())
    return {"make-instance": cmd_make_instance, "rollout": cmd_rollout, "replay": cmd_replay, "video": cmd_video,
            "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
