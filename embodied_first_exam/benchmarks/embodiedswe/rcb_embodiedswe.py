"""EmbodiedSWE tasks in Embodied First Exam: frozen instances, the graded rollout of a delivered `solve(env)` and the
video. One file, in the image at /opt/efe/rcb_embodiedswe.py (with esw_proxy.py and esw_submit.py beside it).

Development (the agent, privileged mode: the full simulator):

    from rcb_embodiedswe import make_env
    env = make_env()                    # launches Isaac Sim (headless, GPU PhysX as EmbodiedSWE grades), builds and
                                        # resets the task's env from /app/instance.json exactly as grading does
    import solve; solve.solve(env)      # anything goes in development

    python /opt/efe/rcb_embodiedswe.py check --solution /app/output/solution
                                        # the graded flow without the hidden grader: solve runs as `nobody` in its own
                                        # process against a proxy env; prints the task's success predicate at the end
    submit "note"                       # freeze a checkpoint of solution/ (esw_submit.py; every checkpoint is graded)

Grading (the verifier, tests/grade.py; root; no network), once per distinct candidate, each in a fresh process:

    rcb_embodiedswe.py rollout --instance I --grader G --solution DIR --out OUT     the graded run (trusted sim process)
    rcb_embodiedswe.py video   --instance I --traj OUT/trajectory.npz --video V.mp4    the selected one's video

Building the task (maintainers): `rcb_embodiedswe.py make-instance --preset P --seed S --out instance.json`.

The rollout: build the env from the frozen instance (EmbodiedSWE's own build + seeded reset, `device="cuda:0"`: GPU
PhysX, as EmbodiedSWE grades), compare its start state with the frozen one (bit for bit on the GPU model that froze
it; on another model the difference is recorded, never an error), construct the task's grader on it (EmbodiedSWE's
rubric, recorded after every step as their GradedEnv does), then start the solution process (esw_proxy.py) as
`nobody` and serve its requests: reads answered with copies, `env.step(action)` executed and recorded, everything else
refused. The candidate's time limit (EmbodiedSWE's rollout limit, 1800 s) runs from the start of this process, so the
simulator's start-up and the import of solve.py count, as in EmbodiedSWE; at the limit (or on SIGTERM from the
verifier) the solution's processes are killed and the verdict is read from the state at that moment, as EmbodiedSWE's
grader does at its budget kill. Every process of `nobody` is killed before the verdict is read. Verdict: success = the
grader's `check_success` on the final state (their end predicate); progress = the grader's rubric score (its peak
recorded progress); a task with a tool-only rule (puzzle.spatula: the hand or a finger touching the bread disqualifies
the episode) scores success 0 and progress 0 once PhysX reports such a contact (ToolOnlyMonitor, read after every
physics substep). An exception from `env.step` is raised in `solve`, as EmbodiedSWE's GradedEnv lets it propagate: the
rollout goes on if `solve` handles it. Actions are applied as given (robobench's controllers scale task-space deltas by
pos_scale / rot_scale without clipping); only the shape and finiteness of an action are checked. The trajectory
(OUT/trajectory.npz) holds every action and the scene's state every few control steps (Recorder). The video poses
those recorded states on the bodies and draws them (no physics); it never changes the verdict.
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
DEVICE = "cuda:0"                  # GPU PhysX, EmbodiedSWE's device
MAX_STEPS = 400_000                # runaway guard on control steps in one rollout (a technical cap, not a budget)
ROLLOUT_TIMEOUT_S = 1800.0         # EmbodiedSWE's rollout limit: the whole candidate process (start-up, import, solve)
START_TOL = 1e-4                   # start-state tolerance on another GPU model (max abs difference; a diagnostic)
MAX_FRAMES = 3000                  # recorded states per trajectory (the cadence doubles when full)
FRAME_DT = 0.1                     # their target spacing in simulated seconds (the video's 10 fps)
T_START = time.monotonic()         # this process's start: the candidate's time limit runs from here
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
    "assembly.so101.bimanual_franka.osc": {"grader": "SO101AssemblyGrader", "check": "so101_success"},  # no success()
    # the scene states a TOOL-ONLY rule (finger or gripper contact with the bread disqualifies the episode) and leaves
    # it to the harness (its module docstring); the tooling enforces it (ToolOnlyMonitor)
    "puzzle.spatula.franka.osc": {"grader": "SpatulaFlipServeGrader", "check": "success",
                                  "tool_only": {"payload": "Bread_", "hand": ("panda_hand", "panda_leftfinger",
                                                                              "panda_rightfinger")}},
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


def so101_success(scene):
    """The SO101 scene's end predicate (it has no success()), exactly EmbodiedSWE's grader's (SO101AssemblyGrader.
    check_success): every loose screw fastened (`fastened >= 0` for all twelve; the scene's `state == 2`), the servo
    in the upper arm's pocket (its body frame on the upper_arm link frame within `cfg.motor_align_pos` and
    `cfg.motor_align_deg`) and the forearm on the horn (the lower_arm at its elbow seat on the motor within
    `motor_align_pos`, its orientation off the horn axis within `motor_align_deg`; any elbow angle). `(num_envs,)`."""
    import math

    from isaaclab.utils.math import quat_error_magnitude
    c = scene.cfg
    ap, aq = scene.upper_arm_pose()
    motor = (((scene.motor.data.root_pos_w - ap).norm(dim=-1) < c.motor_align_pos)
             & (quat_error_magnitude(scene.motor.data.root_quat_w, aq) < math.radians(c.motor_align_deg)))
    lp, lq = scene.lower_arm_pose()
    seat_p, seat_q = scene.lower_arm_seat_w()
    _, off_axis = scene._elbow_angle_split(lq, seat_q)
    fork = ((lp - seat_p).norm(dim=-1) < c.motor_align_pos) & (off_axis < math.radians(c.motor_align_deg))
    return (scene.fastened >= 0).all(dim=1) & motor & fork


CHECKS = {"cut_success": cut_success, "so101_success": so101_success}


class ToolOnlyMonitor:
    """A task's TOOL-ONLY rule, in the trusted process: contact between the robot's hand (`spec["hand"]`: the hand
    body and its fingers) and the payload (the scene's present bread) disqualifies the episode. PhysX reports the
    contacts of the payload's rigid bodies (PhysxContactReportAPI, threshold 0, plus omni.physx's contact processing,
    both set before the simulation starts; they report, they do not change the dynamics), and after EVERY physics
    substep (the scene's post_step) the monitor reads the contact-force matrix between them and the hand's links:
    any pair with a nonzero force is a touch. Contacts with the absent (parked) bread sizes do not count."""

    def __init__(self, env, spec: dict):
        import torch
        self.env, self.spec = env, spec
        self.scene = env.scene
        self.payload = list(self.scene.breads.values())
        from pxr import UsdPhysics
        root = env.stage.GetPrimAtPath(f"/World/envs/env_0/{env.robot.prim_name}")
        links, todo = [], [root] if root.IsValid() else []
        while todo:                              # the hand's rigid links under the robot's prim
            p = todo.pop(0)
            if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName() in spec["hand"]:
                links.append(str(p.GetPath()))
            todo += p.GetChildren()
        if sorted(p.rsplit("/", 1)[-1] for p in links) != sorted(spec["hand"]):
            raise RuntimeError(f"tool-only rule: hand links {spec['hand']} not all found ({links})")
        sensors = [p for b in self.payload for p in b.root_physx_view.prim_paths]
        if len(sensors) != len(self.payload):
            raise RuntimeError(f"tool-only rule: expected one rigid body per payload, got {sensors}")
        self.view = env.sim.physics_sim_view.create_rigid_contact_view(
            sensors, filter_patterns=[links] * len(sensors), max_contact_data_count=64 * len(sensors) * len(links))
        self.links, self.sensors = links, sensors
        self.substeps = 0
        self.touch_substeps = 0
        self.first_touch_step = None             # control step (0-based) of the first touch
        self.max_force = 0.0
        self.touched_links: set = set()
        orig = self.scene.post_step
        period = int(env.robot.control_period)

        def post_step(*a, **k):                  # the scene's own mechanics first, then the contact read
            orig(*a, **k)
            f = self.view.get_contact_force_matrix(dt=env.dt)
            f = torch.as_tensor(f).reshape(len(sensors), len(links), 3).norm(dim=-1)
            present = self.scene._present[0].reshape(-1).to(torch.bool).cpu()
            hit = f.cpu()[present]
            if bool((hit > 0).any()):
                self.touch_substeps += 1
                if self.first_touch_step is None:
                    self.first_touch_step = self.substeps // period
                self.max_force = max(self.max_force, float(hit.max()))
                for j in torch.nonzero((hit > 0).any(dim=0)).reshape(-1).tolist():
                    self.touched_links.add(links[j].rsplit("/", 1)[-1])
            self.substeps += 1
        self.scene.post_step = post_step

    @property
    def touched(self) -> bool:
        return self.touch_substeps > 0

    def report(self) -> dict:
        return {"tool_only": 1, "tool_contact": int(self.touched), "tool_contact_substeps": self.touch_substeps,
                "tool_contact_first_step": self.first_touch_step, "tool_contact_max_force": round(self.max_force, 6),
                "tool_contact_links": sorted(self.touched_links)}


def _enable_contact_reports(payload_prefix: str) -> None:
    """Before the simulation starts: PhysX contact reports (threshold 0) on every rigid body of every prim of each env
    whose name starts with `payload_prefix`, and omni.physx's contact processing on (Isaac Lab turns it off for CPU
    physics and back on when it creates a contact sensor, as here). Only reporting: no sleep threshold or other
    physics attribute changes."""
    import carb
    import omni.usd
    from pxr import PhysxSchema, UsdPhysics
    carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
    stage = omni.usd.get_context().get_stage()
    n = 0
    envs = stage.GetPrimAtPath("/World/envs")
    for e in (envs.GetChildren() if envs.IsValid() else []):
        for top in e.GetChildren():
            if not top.GetName().startswith(payload_prefix):
                continue
            todo = [top]
            while todo:
                p = todo.pop(0)
                if p.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(p).CreateThresholdAttr().Set(0.0)
                    n += 1
                else:
                    todo += p.GetChildren()
    if not n:
        raise RuntimeError(f"tool-only rule: no rigid body named {payload_prefix}* on the stage")


def tool_contact(env) -> dict | None:
    """The tool-only rule's record of this env so far (None for a task without the rule)."""
    mon = getattr(env, "_esw_tool_only", None)
    return mon.report() if mon is not None else None


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


def build(instance: dict, before_reset=None, reset_impl=None):
    """The env exactly as EmbodiedSWE's grader builds it (registered preset, seed, one trajectory), GPU PhysX. A task
    with a tool-only rule gets its contact monitor (ToolOnlyMonitor). `before_reset` / `reset_impl`: the video's
    camera (see _build_with_camera)."""
    if BENCH_ROOT not in sys.path:
        sys.path.insert(0, BENCH_ROOT)
    import robobench
    robobench.discover()
    from robobench.core.registries import ENVS
    seed = int(instance["seed"])
    if instance.get("device", DEVICE) != DEVICE:
        raise ValueError(f"instance built on {instance.get('device')!r}; these tasks run PhysX (and the G1 locomotion "
                         f"policy) on {DEVICE!r}, as EmbodiedSWE grades them")
    spec = TASKS[instance["preset"]].get("tool_only")
    hooks = [lambda: _enable_contact_reports(spec["payload"])] if spec else []
    if before_reset:
        hooks.append(before_reset)
    if hooks or reset_impl is not None:
        # run the hooks right before the env's one `sim.reset()` (what must be on the stage when PhysX starts), and
        # take that reset from `reset_impl` if given; nothing else of the build changes
        from isaaclab.sim import SimulationContext
        orig = SimulationContext.reset
        pending = list(hooks)

        def reset(self, soft: bool = False):
            while pending:
                pending.pop(0)()
            return (reset_impl or orig)(self, soft=soft)

        SimulationContext.reset = reset
        try:
            env = ENVS.get(instance["preset"])().build(num_envs=1, seed=seed, device=DEVICE)
        finally:
            SimulationContext.reset = orig
        if pending:
            raise RuntimeError("the env build did not reset the simulation")
    else:
        env = ENVS.get(instance["preset"])().build(num_envs=1, seed=seed, device=DEVICE)
    if str(env.device) != DEVICE:
        raise RuntimeError(f"the env was built on {env.device}, not {DEVICE}")
    if spec:
        env._esw_tool_only = ToolOnlyMonitor(env, spec)
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


def scene_vector(env) -> tuple[list, "object"]:
    """Isaac Lab's scene state (every asset's root pose and velocity, joint positions and velocities, world frame) as
    ([(path, shape)], one float64 vector)."""
    import numpy as np
    import torch
    layout, parts = [], []
    for path, v in _leaves(env.iscene.get_state(is_relative=False)):
        if isinstance(v, torch.Tensor):
            layout.append((path, list(v.shape)))
            parts.append(v.detach().to("cpu").double().reshape(-1).numpy())
    return layout, (np.concatenate(parts) if parts else np.zeros(0))


def gpu_model() -> str:
    try:
        import torch
        return torch.cuda.get_device_name(0)
    except Exception:  # noqa: BLE001
        return "unknown"


def check_start(env, inst: dict) -> dict:
    """The start state against the frozen instance: `instance_ok` = the same state hash (expected on the GPU model that
    froze it, `instance_gpu`); otherwise `instance_max_abs_diff` over the scene state vector and `instance_within_tol`
    (START_TOL), a diagnostic. `instance_mismatch` = 1 only when the hash differs on the freezing GPU model."""
    import numpy as np
    sha = state_sha(env)
    model = gpu_model()
    out = {"instance_sha": sha, "instance_ok": int(sha == inst["state_sha"]), "gpu_model": model,
           "instance_gpu": inst.get("gpu_model"), "instance_same_gpu": int(model == inst.get("gpu_model"))}
    if out["instance_ok"]:
        out.update(instance_max_abs_diff=0.0, instance_within_tol=1, instance_mismatch=0)
        return out
    layout, vec = scene_vector(env)
    ref = inst.get("start_state") or {}
    if [list(x) for x in ref.get("layout", [])] == [list(x) for x in layout] and len(ref.get("vector", [])) == len(vec):
        d = float(np.max(np.abs(vec - np.asarray(ref["vector"], dtype=np.float64)))) if len(vec) else 0.0
    else:
        d = float("inf")                         # another scene layout
    out.update(instance_max_abs_diff=d if d != float("inf") else -1.0, instance_within_tol=int(d <= START_TOL),
               instance_mismatch=int(bool(out["instance_same_gpu"])))
    return out


class Recorder:
    """The run's trajectory: every action (recorded before its step) and the scene state (scene_vector, float32) every
    `every` control steps, the start and the end included. At most MAX_FRAMES states: when full, the cadence doubles and
    every other kept state goes. Written by `save` to an npz (trajectory.npz)."""

    def __init__(self, env, preset: str, seed: int):
        import numpy as np
        self.np = np
        self.env, self.preset, self.seed = env, preset, seed
        self.control_dt = float(env.dt * env.robot.control_period)
        self.every = max(1, int(round(FRAME_DT / self.control_dt)))
        self.actions: list = []
        self.steps: list[int] = []
        self.rows: list = []
        self.layout = None

    def frame(self) -> None:
        n = len(self.actions)
        if self.steps and self.steps[-1] == n:
            return
        layout, vec = scene_vector(self.env)
        if self.layout is None:
            self.layout = layout
        elif layout != self.layout:
            raise RuntimeError("the scene state changed layout during the run")
        self.rows.append(vec.astype(self.np.float32))
        self.steps.append(n)
        if len(self.rows) > MAX_FRAMES:
            self.every *= 2
            keep = [i for i, s in enumerate(self.steps) if s % self.every == 0 or i == len(self.steps) - 1]
            self.rows = [self.rows[i] for i in keep]
            self.steps = [self.steps[i] for i in keep]

    def after_step(self) -> None:
        if len(self.actions) % self.every == 0:
            self.frame()

    def save(self, path: Path, **extra) -> None:
        np = self.np
        A = np.stack(self.actions).astype(np.float32) if self.actions else np.zeros((0, self.env.robot.action_dim),
                                                                                    np.float32)
        S = np.stack(self.rows) if self.rows else np.zeros((0, 0), np.float32)
        np.savez(path, task=np.array(self.preset), seed=np.array(self.seed), actions=A,
                 frame_step=np.asarray(self.steps, dtype=np.int64), frame_state=S,
                 state_layout=np.array(json.dumps(self.layout or [])), control_dt=np.array(self.control_dt),
                 **{k: np.array(v) for k, v in extra.items()})


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
                     r"prim_name|action_dim|control_period|control_dt|actuator_limits|joint_names|body_names|"
                     r"bread_r|bread_h)",
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
    # assembly.so101: frames and seats (pure reads); set_elbow_target and the fastening mechanics stay refused
    "upper_arm_pose", "lower_arm_pose", "lower_arm_seat_w", "elbow_screw_seats_w", "elbow_horn_screw_seats_w",
    "_hole_frames_w", "_seats_w", "_elbow_angle_split",
    # puzzle.spatula: the instantaneous predicates and the blade-frame helpers (pure reads)
    "tool_lifted_now", "on_blade", "on_pan", "flipped_now", "on_plate", "on_bare_surface", "loaded_now",
    "served_now", "carry_tilt_deg", "_bread_tensors", "_bread_up", "_blade_up", "_bread_in_blade_frame",
    "_stage_table",
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


TERM = threading.Event()             # SIGTERM from the verifier at the candidate's time limit (cmd_rollout)


def run_solution(env, solution: Path, work: Path, on_step, deadline: float, log_path: Path) -> dict:
    """Start the solution process as `nobody` and serve it until solve ends, it dies, the monotonic clock passes
    `deadline` or TERM is set (status "timeout" for the last two)."""
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
    t_start = time.monotonic()
    timer_fired = threading.Event()

    def watchdog():
        while proc.poll() is None:
            if time.monotonic() > deadline or TERM.is_set():
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

    def ended(exc) -> None:
        """The solution's side of the pipe is gone: at the time limit (the watchdog killed it, or TERM) that is the
        rollout's timeout, a graded end like any other; otherwise the process ended by itself."""
        late = timer_fired.is_set() or TERM.is_set() or time.monotonic() > deadline
        out["solve_status"] = "timeout" if late else "process_ended"
        out["solve_error"] = None if late else f"connection: {exc}"

    def reply(msg) -> bool:
        """Answer the solution; False (and the end recorded) when it can no longer be answered: a solution killed at
        the time limit while its request was being served leaves a broken pipe, never a grader error."""
        try:
            wire.send(msg)
            return True
        except (BrokenPipeError, ConnectionError, OSError) as exc:
            ended(exc)
            return False

    try:
        ok = reply({"op": "hello", "root": srv.handle_of(env), "protocol": "esw-proxy/1", "scratch": str(scratch)})
        while ok:
            try:
                req = wire.recv()
            except (EOFError, ValueError, OSError) as exc:
                ended(exc)
                break
            if not isinstance(req, dict):
                if not reply({"error": "malformed request", "kind": "ValueError"}):
                    break
                continue
            op = req.get("op")
            if op == "loaded":
                t0 = time.monotonic()
                out["load_s"] = round(t0 - t_start, 3)
                if not reply({"value": None}):
                    break
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
                msg = {"value": v if op != "step" else None}
            except StopRollout as exc:
                out["solve_status"] = str(exc)
                reply({"error": str(exc), "kind": "RuntimeError", "stop": True})
                out["solve_status"] = str(exc)            # the step cap stands even if the answer cannot be sent
                break
            except (Refused, PermissionError) as exc:
                srv.n_refused += 1
                if len(srv.refusals) < 50:
                    srv.refusals.append(f"{req.get('op')}:{req.get('name', '')}: {exc}"[:300])
                msg = {"error": str(exc), "kind": "PermissionError"}
            except Exception as exc:  # noqa: BLE001 — the solve's error (a bad attribute, a bad index) is its own
                msg = {"error": f"{type(exc).__name__}: {exc}"[:4000], "kind": type(exc).__name__}
            if time.monotonic() > deadline or TERM.is_set():
                out["solve_status"] = "timeout"
                break
            if not reply(msg):
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
    """Freeze an instance: the preset, the seed, the device, the start state's hash, its scene state vector (for the
    comparison on another GPU model) and the GPU model it was frozen on."""
    launch()
    inst = {"format": INSTANCE_FORMAT, "preset": a.preset, "seed": int(a.seed), "device": DEVICE}
    try:
        env = build(inst)
    except Exception:  # noqa: BLE001 — a failed build writes no instance and exits non-zero
        traceback.print_exc()
        stop_app(1)
    inst["state_sha"] = state_sha(env)
    inst["gpu_model"] = gpu_model()
    layout, vec = scene_vector(env)
    inst["start_state"] = {"layout": layout, "vector": [float(x) for x in vec]}
    inst["action_dim"] = int(env.robot.action_dim)
    inst["control_dt"] = round(float(env.dt * env.robot.control_period), 6)
    inst["physics_dt"] = float(env.dt)
    Path(a.out).write_text(json.dumps(inst, indent=1, sort_keys=True) + "\n")
    if a.describe:
        Path(a.describe).write_text(env.describe().strip() + "\n")
    print(json.dumps({k: v for k, v in inst.items() if k != "start_state"}), flush=True)
    stop_app()
    return 0


def _graded_env(inst: dict, grader_path: str | None):
    env = build(inst)
    grader = load_grader(grader_path, inst["preset"])(env) if grader_path else None
    return env, grader


def _final(env, grader, inst: dict) -> dict:
    if grader is not None:
        v = grader.verdict()[0]
        success, score = bool(v["success"]), float(v["score"])
    else:
        success, score = bool(task_success(env, inst["preset"])[0]), None
    out = {"success": success, "progress": score, "final_sha": state_sha(env)}
    tc = tool_contact(env)
    if tc is not None:
        out.update(tc, disqualified=tc["tool_contact"])
        if tc["tool_contact"]:                   # the episode is disqualified: no success, no progress
            out.update(success=False, progress=0.0 if score is not None else None)
    return out


def _unreadable_final(env, grader, exc) -> dict:
    """The verdict when the grader cannot read the state (after a failed step): success 0, progress = the rubric's
    peak recorded before (0 when a tool-only rule disqualified the episode)."""
    out = {"success": False, "progress": _peak(grader), "final_sha": None,
           "final_error": f"{type(exc).__name__}: {exc}"[:500]}
    tc = tool_contact(env)
    if tc is not None:
        out.update(tc, disqualified=tc["tool_contact"])
        if tc["tool_contact"]:
            out["progress"] = 0.0
    return out


def _peak(grader) -> float:
    try:
        return max((float(step[0]["progress"]) for step in grader.history), default=0.0)
    except Exception:  # noqa: BLE001
        return 0.0


_PHASE = ["boot"]


def _set_phase(out: Path, phase: str) -> None:
    """The rollout's phase, in OUT/phase.json: the verifier reads it when the process ends without a result (killed at
    the time limit): in "solve" that is the candidate's 0, in any other phase the verifier's error."""
    _PHASE[0] = phase
    (out / "phase.json").write_text(json.dumps({"phase": phase}) + "\n")


def _on_term(signum, frame) -> None:
    if _PHASE[0] != "solve":
        return                     # finalizing: the verdict is on its way (the verifier allows a grace period)
    TERM.set()                     # run_solution ends the solve; the verdict is read on the state at this moment


def cmd_rollout(a) -> int:
    """The graded run of one candidate. Prints one JSON line (also written to OUT/rollout.json); the trajectory goes to
    OUT/trajectory.npz. The time limit (`--timeout`) runs from this process's start (T_START)."""
    import torch
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    res: dict = {"phase": "boot", "label": a.label or ""}
    deadline = T_START + float(a.timeout)
    _set_phase(out, "boot")
    try:
        launch()
        inst = load_instance(a.instance)
        env, grader = _graded_env(inst, a.grader)
        res.update(check_start(env, inst), boot_s=round(time.monotonic() - T_START, 1))
        if res["instance_mismatch"]:
            res.update(phase="instance_mismatch", infra_error=1)
            raise StopRollout("instance_mismatch")
        if time.monotonic() > deadline:
            res.update(phase="boot_timeout", infra_error=1)
            raise StopRollout("boot_timeout")
        rec = Recorder(env, inst["preset"], int(inst["seed"]))
        rec.frame()                                   # the start state
        step_s = [0.0]
        bad = [0]
        step_err: list = []                           # (index, message) of every env.step that raised

        def on_step(act):
            if act.numel() != env.num_envs * env.robot.action_dim:
                bad[0] += 1
                raise ValueError(f"action has {act.numel()} values; the env takes ({env.num_envs}, "
                                 f"{env.robot.action_dim})")
            if not bool(torch.isfinite(act).all()):
                bad[0] += 1
                raise ValueError("action contains NaN or inf")
            if len(rec.actions) >= MAX_STEPS:
                raise StopRollout(f"step_cap ({MAX_STEPS} control steps)")
            act = act.reshape(env.num_envs, env.robot.action_dim).contiguous()
            rec.actions.append(act[0].numpy().copy())     # recorded BEFORE the step: a step that raises is in the record
            t = time.perf_counter()
            try:
                env.step(act.to(env.device))
                if grader is not None:
                    grader.record()
            except Exception as exc:  # noqa: BLE001 — the env could not take this action (e.g. a controller
                # singularity). As in EmbodiedSWE's GradedEnv, the error reaches solve: the rollout goes on if solve
                # handles it, and ends (verdict on the state then) if it does not.
                step_err.append((len(rec.actions) - 1, f"{type(exc).__name__}: {exc}"[:1000]))
                raise
            finally:
                step_s[0] += time.perf_counter() - t
            rec.after_step()
            return None

        res["phase"] = "solve"
        _set_phase(out, "solve")
        signal.signal(signal.SIGTERM, _on_term)
        run = run_solution(env, Path(a.solution), out / "work", on_step, deadline, Path(a.solve_log))
        _set_phase(out, "finalize")
        res.update(run, terminated=int(TERM.is_set()))
        if run.get("solve_status") == "sandbox_failed":
            res.update(phase="sandbox_failed", infra_error=1)
            raise StopRollout("sandbox_failed")
        if step_err:
            res.update(step_errors=len(step_err), step_error_at=step_err[0][0], step_error=step_err[0][1])
            try:
                res.update(_final(env, grader, inst))
            except Exception as exc:  # noqa: BLE001 — the env may be unusable after the failed step: success 0,
                # progress = the rubric's peak recorded before it (the grader's own record of the run)
                res.update(_unreadable_final(env, grader, exc))
        else:
            res.update(_final(env, grader, inst))
        try:
            rec.frame()                               # the final state
        except Exception as exc:  # noqa: BLE001 — an env broken by a failed step: the trajectory ends at its last frame
            res["final_frame_error"] = f"{type(exc).__name__}: {exc}"[:300]
        rec.save(out / "trajectory.npz", label=a.label or "", final_sha=res.get("final_sha") or "")
        n = len(rec.actions)
        res.update(phase="done", n_actions=n, n_frames=len(rec.steps), frame_every=rec.every, bad_actions=bad[0],
                   sim_step_s=round(step_s[0], 3), ms_per_step=round(1000 * step_s[0] / max(1, n), 3), infra_error=0)
        _set_phase(out, "done")
    except StopRollout:
        pass
    except Exception:  # noqa: BLE001
        res.update(infra_error=1, error=traceback.format_exc()[-4000:])
    res["wall_s"] = round(time.monotonic() - T_START, 1)
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


def _build_with_camera(inst: dict, make_camera):
    """`build(inst)` exactly as grading builds it, plus a camera: the camera prim is added just before the env's own
    (single) `sim.reset()`, and that reset skips Isaac Lab's two warm-up renders (they and an extra `sim.reset()`
    change later physics bits), so the camera's env starts from the graded start state. Frames drawn between steps can
    still change GPU PhysX's later bits (measured in the stack scene; not in SO101's). `make_camera()` is called once
    and its result returned with the env."""
    from isaaclab.sim import SimulationContext
    made: list = []

    def reset(self, soft: bool = False):        # Isaac Lab's reset minus the warm-up renders
        self._disable_app_control_on_stop_handle = True
        super(SimulationContext, self).reset(soft=soft)
        if self.physics_sim_view:
            self.physics_sim_view._backend.initialize_kinematic_bodies()
        self._disable_app_control_on_stop_handle = False

    env = build(inst, before_reset=lambda: made or made.append(make_camera()), reset_impl=reset)
    if not made:
        raise RuntimeError("the env build did not reset the simulation: no camera")
    return env, made[0]


def _unflatten(row, layout: list, device):
    """Isaac Lab's scene-state dict from one recorded state vector (Recorder) for `InteractiveScene.reset_to`."""
    import torch
    st: dict = {}
    i = 0
    for path, shape in layout:
        n = 1
        for d in shape:
            n *= int(d)
        node = st
        keys = path.strip("/").split("/")
        for k in keys[:-1]:
            node = node.setdefault(k, {})
        node[keys[-1]] = torch.as_tensor(row[i:i + n], dtype=torch.float32, device=device).reshape(shape)
        i += n
    if i != len(row):
        raise ValueError("a recorded state does not match its layout")
    return st


def load_trajectory(path: str, preset: str, action_dim: int) -> dict:
    import numpy as np
    d = np.load(path, allow_pickle=False)
    A = _load_actions(path, preset, action_dim)
    steps = np.asarray(d["frame_step"], dtype=np.int64)
    S = np.asarray(d["frame_state"], dtype=np.float32)
    layout = json.loads(str(d["state_layout"]))
    if S.ndim != 2 or len(S) != len(steps) or len(S) > MAX_FRAMES + 1 \
            or (len(steps) and (steps.min() < 0 or steps.max() > len(A) or (np.diff(steps) <= 0).any())):
        raise ValueError("malformed trajectory states")
    return {"actions": A, "steps": steps, "states": S, "layout": layout,
            "label": str(d["label"]) if "label" in d else "", "final_sha": str(d["final_sha"]) if "final_sha" in d else ""}


def cmd_video(a) -> int:
    """The selected candidate's video, from its trajectory, in this fresh process (never part of a verdict):
    1. the camera replay, a diagnostic: the recorded actions re-executed with the camera drawing at the recorded
       frames, as a live recording would (drawing can change GPU PhysX's later bits, so it may end elsewhere):
       `camera_replay` (done / budget / error), `camera_replay_match` (its final state hash equals the graded run's),
       `camera_replay_success` (the task's public predicate at its end); at most `--replay-budget` seconds;
    2. the video: each recorded state posed on the bodies (InteractiveScene.reset_to, no physics step) and drawn, so
       the video shows the graded run's own states (Isaac Lab's scene state: poses and joints; a scene's non-physical
       visuals are not part of it); a recorded state with a non-finite value is skipped (`frames_nonfinite`)."""
    import numpy as np
    import torch
    res: dict = {}
    try:
        launch(cameras=True)
        import imageio.v2 as imageio
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import Camera, CameraCfg
        inst = load_instance(a.instance)
        env, cam = _build_with_camera(inst, lambda: Camera(CameraCfg(
            prim_path="/World/esw_cam", update_period=0.0, height=a.height, width=a.width, data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(focal_length=18.0, clipping_range=(0.01, 100.0)))))
        st = check_start(env, inst)
        res.update(camera_instance_ok=st["instance_ok"], camera_instance_max_abs_diff=st["instance_max_abs_diff"])
        eye, target = (env.room or {}).get("camera") or ((1.6, -1.6, 1.4), (0.3, 0.0, 0.5))
        if a.eye and a.target:
            eye, target = [float(v) for v in a.eye.split(",")], [float(v) for v in a.target.split(",")]
        cam.set_world_poses_from_view(torch.tensor([list(eye)], device=env.device),
                                      torch.tensor([list(target)], device=env.device))
        tr = load_trajectory(a.traj, inst["preset"], env.robot.action_dim)
        A, steps = tr["actions"], tr["steps"]
        final_sha = a.final_sha or tr["final_sha"]
        res["label"] = tr["label"]
        drawn = [0]

        def draw():
            for _ in range(1 if drawn[0] else 3):      # the renderer's first 2 frames are dark
                env.sim.render()
                cam.update(env.dt)
            drawn[0] += 1
            return cam.data.output["rgb"][0].detach().cpu().numpy()[..., :3]

        # 1. the camera replay
        status, t1 = "done", time.monotonic()
        try:
            at = set(int(s) for s in steps)
            if 0 in at:
                draw()
            for i, row in enumerate(A):
                if time.monotonic() - t1 > a.replay_budget:
                    status = "budget"
                    break
                try:
                    env.step(torch.from_numpy(row).reshape(1, -1).to(env.device))
                except Exception:  # noqa: BLE001 — the graded run raised on this step too and went on
                    res.setdefault("camera_replay_step_error_at", i)
                    continue
                if i + 1 in at:
                    draw()
            if status == "done":
                fsha = state_sha(env)
                res.update(camera_replay_match=int(bool(final_sha) and fsha == final_sha),
                           camera_replay_success=int(bool(task_success(env, inst["preset"])[0])))
        except Exception as exc:  # noqa: BLE001
            status = "error"
            res["camera_replay_error"] = f"{type(exc).__name__}: {exc}"[:500]
        res.update(camera_replay=status, camera_replay_s=round(time.monotonic() - t1, 1),
                   camera_replay_frames=drawn[0])
        # 2. the video: the recorded states, posed and drawn
        w = imageio.get_writer(a.video, fps=a.fps, codec="libx264", quality=7, macro_block_size=8)
        n = skipped = 0
        t2 = time.monotonic()
        for row in tr["states"]:
            if not np.isfinite(row).all():            # a state the run's physics blew up (NaN): nothing to pose
                skipped += 1
                continue
            env.iscene.reset_to(_unflatten(row, tr["layout"], env.device), is_relative=False)
            env.sim.forward()
            f = draw()
            if f.size and f.any():
                w.append_data(f)
                n += 1
        w.close()
        res.update(video=a.video, frames=n, video_ok=int(n > 0), video_source="recorded_states",
                   frames_nonfinite=skipped, pose_s=round(time.monotonic() - t2, 1))
    except Exception:  # noqa: BLE001
        res.update(video_ok=0, error=traceback.format_exc()[-4000:])
    res["wall_s"] = round(time.monotonic() - T_START, 1)
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
    Launches Isaac Sim (headless, GPU PhysX) first if this process has not."""
    launch(cameras=cameras)
    inst = load_instance(instance)
    env = build(inst)
    st = check_start(env, inst)
    if not st["instance_ok"]:
        log(f"note: the start state differs from the frozen instance's hash (GPU {st['gpu_model']!r}, frozen on "
            f"{st['instance_gpu']!r}; max abs difference {st['instance_max_abs_diff']:.3g})")
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
        p.add_argument("--timeout", type=float, default=ROLLOUT_TIMEOUT_S,
                       help="the time limit from this process's start (start-up and import count)")
        p.add_argument("--solve-log", default=None)
        p.add_argument("--label", default="", help="the candidate's name, kept in the trajectory")
    p = sub.add_parser("video")
    p.add_argument("--instance", default="/app/instance.json")
    p.add_argument("--traj", required=True)
    p.add_argument("--video", required=True)
    p.add_argument("--final-sha", default="", help="the graded run's final state hash (default: the trajectory's)")
    p.add_argument("--replay-budget", type=float, default=1800.0, help="seconds for the camera replay (0: skip it)")
    p.add_argument("--fps", type=float, default=10.0)
    p.add_argument("--width", type=int, default=960)
    p.add_argument("--height", type=int, default=540)
    p.add_argument("--eye", default=None, help="camera eye x,y,z (env frame); default: the room's camera")
    p.add_argument("--target", default=None, help="camera target x,y,z")
    a = ap.parse_args(argv)
    if a.cmd == "rollout" and not a.solve_log:
        a.solve_log = str(Path(a.out) / "solve.log")
    if HERE.as_posix() not in sys.path:
        sys.path.insert(0, HERE.as_posix())
    return {"make-instance": cmd_make_instance, "rollout": cmd_rollout, "video": cmd_video, "check": cmd_check}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
