"""eai-standard/2.2 (formerly rcb-limited/2.2): the HTTP layer, the protocol helpers and the crash recovery that every
2.2 sim service shares (docs/protocol/standard_mode.md, Part 2; the 2.1 base is its Part 1).

The source is scripts/eai_standard/rcb_limited.py; the generators copy it unchanged into every standard task's
environment/sim/ (ci_checks/check-tooling-identity.sh). A service keeps its simulator to itself (scene, action
execution, cameras, kinematic model) and hands this module plain callables:

    import rcb_limited as L
    EP = L.Episode(DATA, dim=N, resume="exact")   # before the scene is built; "approx" for a nondeterministic simulator
    EP.startup()                                  # start count, back-off after failed starts, startup watchdog
    S = <build the scene>; EP.scene_built()       # the watchdog stands down
    def ctrl_step(a, rec):                        # every executed control step
        S.step(a); stop = EP.journal(a); rec.tick(); return E["done"] or stop
    rec = L.Recorder(cams, every, grab, proprio_row, extrinsics, clock, depth=L.depth_mode(meta), clock_hz=CTRL_HZ)
    rec.tick(force=True)            # first frame (before the chunk / watch)
    ...; rec.tick(); ...            # after every control step: a frame every `every` steps
    rec.tick(force=True, last=True) # last frame (+ its depth)
    parts = rec.parts(video, fps)   # RCB1 parts: video/ frames/ last/ extrinsic_cv/ last_depth/ depth_frames/ depth/ proprio frame_*
    S.record_start()
    EP.begin(state=E, step=S.step, state_sha=S.state_sha, proprio=proprio_row, spec=SPEC, start_event={...}, on_end=end_fn)
    L.serve(L.Service(spec=SPEC, lock=LOCK, state=E, submit=do_submit, observe=do_observe, kinematics=do_kinematics,
                      status=status, on_submit_error=lambda exc: finish("error"), log=EP.log,
                      episode=EP, state_sha=S.state_sha, proprio=proprio_row))

    grab(cams, want_depth) -> ({cam: uint8 (H,W,3)}, {cam: float32 (H,W) z-depth in metres, 0 = no reading})
    extrinsics(cams)       -> {cam: (3,4) world -> camera, OpenCV axes (x right, y down, z forward)}
    proprio_row()          -> float32 (len(spec["proprio_layout"]),)
    clock()                -> int, control steps since the episode start (the frame's step index)
    do_kinematics(req)     -> dict; `req` is parse_kinematics(body, spec["kinematics"]); it must never step physics

None of the callables may step physics. check_spec(SPEC) runs in serve(): a service whose /spec misses a 2.2 field, or
carries a field the protocol forbids (segmentation, object identities or sizes, physical parameters), does not start.
Crash recovery (class Episode; PROTOCOL §4.7): every executed control step is journaled, every completed call
checkpointed (rows, state hash, the robot's proprio row); after a crash compose restarts the container and the service
replays the journal and continues the SAME episode, never a reset. `python rcb_limited.py end <data dir>` is the shared
first half of a collect hook.
The agent's clock (meta time_s / time_left_s = agent_time(E)) starts at its first request to the service, any endpoint
(the HTTP layer notes it in E["t_first"] and logs `first_request`, which a resume reads back): the sidecar is up before
the agent's session begins (the harness installs the agent first), so the service's own start is not that moment."""
from __future__ import annotations
import errno, faulthandler, json, math, os, re, select, signal, socket, sys, tempfile, threading, time, traceback, zlib
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Callable

import numpy as np

import rcb1

PROTOCOL = "eai-standard/2.2"
COMPAT = ["rcb-limited/2.0", "rcb-limited/2.1", "eai-standard/2.1"]   # requests written for these are served unchanged
MAX_BODY = 64 * 1024 * 1024
MAX_RESP = 256 * 1024 * 1024
DEPTH_MODES = ("none", "last", "frames")
MAX_FRAMES_PER_CALL = 3000   # images one response may carry (cameras x frames, per-frame depth counted again): the 2.1 services' bound


def check_frames(n_cameras: int, n_frames: int, depth_frames: bool = False, limit: int = MAX_FRAMES_PER_CALL) -> int:
    """The images one call would return (cameras x frames; per-frame depth counted again), refused above `limit`
    (ValueError -> 400, for the adapter to raise before it executes or renders anything): every frame is held in memory
    until the response is sent, so a call without a bound can exhaust the service's memory. The adapter passes the frames
    its Recorder will capture (frames_per_call(), with the Recorder's own `every`) and the limit its /spec states as
    `max_frames_per_call`. A depth frame counts as one image: the Recorder keeps it as it is sent, uint16 mm or float16
    m, at most the bytes of an RGB frame (docs/protocol/README.md §3, the per-call image budget)."""
    n = int(n_cameras) * int(n_frames) * (2 if depth_frames else 1)
    if n > limit:
        raise ValueError(f"this call would return {n} images (cameras x frames, per-frame depth counted again); the limit is "
                         f"max_frames_per_call = {limit}: ask for fewer cameras, a lower video_fps or a shorter call")
    return n


def frames_per_call(steps: int, every: int) -> int:
    """The frames per camera a Recorder filming every `every` clock ticks captures over a call of `steps` control
    steps: the forced first and last frame and one every `every` steps, at most steps // every + 2. `every` is the
    stride the Recorder is given (round(control_hz / video_fps)), not the requested fps: the count is of real captures."""
    return int(steps) // max(1, int(every)) + 2
DEPTH_MAX_M = 65.535                                        # uint16 millimetres; a farther surface reads 0 (no reading)
DEPTH_NOISE = os.environ.get("RCB_DEPTH_NOISE", "off").strip().lower()   # off | realsense (as rcb-limited/2.1)
_RNG = np.random.default_rng(int(os.environ.get("RCB_DEPTH_NOISE_SEED", "0")))
PROPRIO_GROUPS_REQUIRED = ("joint_pos", "joint_vel", "gripper_opening", "tcp_pose", "joint_effort")
PART_PREFIXES = ("video/", "frames/", "last/", "last_depth/", "depth_frames/", "depth/", "extrinsic_cv/")
PART_NAMES = ("proprio", "frame_step", "frame_time_s")


# ---------------------------------------------------------------- depth
def depth_mode(meta: dict) -> str:
    """The request's depth mode: "none" | "last" | "frames" (2.2 field `depth`), or "legacy" for a 2.0/2.1 request that
    sets `include_depth: true` and no `depth` (answered as 2.1 did: `depth/<cam>` float16 (N,H,W) metres)."""
    d = meta.get("depth")
    if d is None or d is False:
        return "legacy" if meta.get("include_depth") is True else "none"
    if d is True:
        return "last"
    if isinstance(d, str) and d in DEPTH_MODES:
        return d
    raise ValueError(f"depth must be one of {'|'.join(DEPTH_MODES)} (got {d!r})")


def clean_depth(z) -> np.ndarray:
    """float32 z-depth in metres: non-finite, negative and beyond-range pixels read 0 (no reading); the optional
    RCB_DEPTH_NOISE=realsense model (sigma = 2 mm + 2.5 mm * z^2, 1 % dropouts) is applied here, after rendering, so it
    never touches physics."""
    z = np.asarray(z, dtype=np.float32).copy()
    if z.ndim == 3 and z.shape[-1] == 1:
        z = z[..., 0]
    bad = ~np.isfinite(z) | (z <= 0) | (z > DEPTH_MAX_M)
    z[bad] = 0.0
    if DEPTH_NOISE == "realsense":
        valid = z > 0
        z = z + _RNG.normal(0.0, 1.0, z.shape).astype(np.float32) * (0.002 + 0.0025 * z * z)
        z[_RNG.random(z.shape) < 0.01] = 0.0
        z[~valid] = 0.0
        z[(z < 0) | (z > DEPTH_MAX_M)] = 0.0
    return z.astype(np.float32)


def depth_mm(z) -> np.ndarray:
    """Metres -> C-order uint16 millimetres (0 = no reading), one frame at a time: what depth_frames/<cam> carries."""
    mm = np.rint(np.asarray(z, dtype=np.float64) * 1000.0)
    mm[(mm < 0) | (mm > 65535)] = 0
    return np.ascontiguousarray(mm.astype("<u2"))


class Deflate:
    """depth_frames/<cam> not compressed yet: a list of depth_mm() frames that encode_response() compresses only while
    they still fit in the response (compression stops as soon as its output passes the room left). Its zlib bytes are
    the same as zlib.compress of the stacked frames; until then len() is an upper bound of their length."""

    def __init__(self, frames: list):
        self.frames = frames
        n = sum(int(f.nbytes) for f in frames)
        self.bound = n + n // 1000 + 64            # zlib's worst case: stored blocks (5 bytes per 64 KiB) and its framing

    def __len__(self) -> int:
        return self.bound

    def compress(self, limit: int):
        """The zlib bytes, or None (the frames let go of) once they pass `limit` bytes: then they cannot be sent."""
        z, out, a = zlib.compressobj(1), bytearray(), self.frames
        a.reverse()                                # each frame is let go of once it is compressed
        while a:
            out += z.compress(a.pop())
            if len(out) > limit:
                a.clear()
                return None
        out += z.flush()
        return out if len(out) <= limit else None


def depth_frames_part(name: str, stack) -> tuple[str, str, bytes, dict]:
    """(N,H,W) metres, or a list of N depth_mm() frames (Deflate: compressed in encode_response, if it is sent) -> an
    RCB1 "bytes" part: zlib of C-order uint16 millimetres (0 = no reading). Clients that do not know it keep the raw
    bytes and ignore it; rcb_client decodes it back to float32 metres."""
    if not isinstance(stack, list):
        a = depth_mm(stack)
        return (name, "bytes", zlib.compress(a.tobytes(), 1), {"codec": "zlib", "dtype": "uint16", "shape": list(a.shape), "unit": "mm"})
    shape = [len(stack), *stack[0].shape] if stack else [0]
    return (name, "bytes", Deflate(stack), {"codec": "zlib", "dtype": "uint16", "shape": shape, "unit": "mm"})


def encode_mp4(frames, fps: int) -> bytes:
    """H.264 mp4 (yuv420p, faststart) of uint8 (N,H,W,3) frames (an array or a list of (H,W,3) frames): the same
    encoding every 2.0 service used."""
    import imageio.v2 as imageio
    with tempfile.NamedTemporaryFile(suffix=".mp4", delete=False) as tf:
        path = tf.name
    try:
        with imageio.get_writer(path, format="FFMPEG", mode="I", fps=fps, codec="libx264", macro_block_size=1,
                                output_params=["-movflags", "+faststart", "-pix_fmt", "yuv420p"]) as w:
            for f in frames:
                w.append_data(np.ascontiguousarray(f))
        return open(path, "rb").read()
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


# ---------------------------------------------------------------- recording what the cameras see during one call
class Recorder:
    """Frames of the requested cameras every `every` clock ticks, plus forced first/last frames; for every frame the
    proprio row, the clock (step index), each camera's extrinsics and, if asked, its depth. All arrays share one frame
    axis N: frames/<cam>, the mp4's frames, proprio, frame_step, frame_time_s, extrinsic_cv/<cam>, depth_frames/<cam>."""

    def __init__(self, cameras, every: int, grab: Callable, proprio_row: Callable, extrinsics: Callable, clock: Callable,
                 depth: str = "none", clock_hz: float = 1.0, max_images: int | None = None):
        if depth not in DEPTH_MODES + ("legacy",):
            raise ValueError(f"bad depth mode {depth!r}")
        self.cams = list(dict.fromkeys(cameras)); self.every = max(1, int(every)); self.mode = depth; self.hz = float(clock_hz)
        # max_images: a call whose length is known only while it runs (ee_pose rows, a watch that ends with the
        # episode) stops filming at the budget; the forced first and last frames are always taken (`skipped` counts
        # the frames not taken)
        self.max_images, self.skipped = max_images, 0
        self.per_frame = len(self.cams) * (2 if depth in ("frames", "legacy") else 1)
        self._grab, self._prop, self._ext, self._clock = grab, proprio_row, extrinsics, clock
        self.frames: dict = {c: [] for c in self.cams}; self.depth: dict = {c: [] for c in self.cams}
        self.ext: dict = {c: [] for c in self.cams}; self.last_depth: dict = {}
        self.prop: list = []; self.steps: list = []; self.n = 0; self.missing = 0

    def tick(self, force: bool = False, last: bool = False) -> None:
        if force or last or self.n % self.every == 0:
            if (not (force or last) and self.max_images is not None
                    and (len(self.steps) + 2) * self.per_frame > self.max_images):
                self.skipped += 1                # the budget keeps room for the last frame
                self._skip()
            else:
                self._capture(last)
        self.n += 1

    def _skip(self) -> None:
        """A frame not taken past the budget (max_images). Nothing here; a service whose live video records only the
        states a call rendered overrides it to render the frame anyway (not kept in the response)."""

    def _capture(self, last: bool) -> None:
        want = self.mode in ("frames", "legacy") or (last and self.mode == "last")
        rgb, dep = self._grab(self.cams, want)
        ext = self._ext(self.cams)
        for c in self.cams:
            img = rgb.get(c)
            if img is not None:
                img = np.ascontiguousarray(np.asarray(img)[..., :3].astype(np.uint8))
            else:                  # a camera that failed to render: a black frame (sized in parts()) keeps one frame axis
                self.missing += 1
            self.frames[c].append(img)
            e = ext.get(c)
            self.ext[c].append(np.full((3, 4), np.nan) if e is None else np.asarray(e, dtype=np.float64).reshape(3, 4))
            if want:
                d = dep.get(c)
                if d is None:
                    d = np.zeros(img.shape[:2] if img is not None else (1, 1), np.float32)
                else:
                    d = clean_depth(d)
                if img is not None and d.shape != img.shape[:2]:
                    raise RuntimeError(f"depth of {c} is {d.shape}, its RGB {img.shape[:2]}: they must be pixel-aligned")
                if self.mode == "frames":    # kept as it is sent: uint16 mm (2 bytes a pixel, an RGB frame has 3)
                    self.depth[c].append(depth_mm(d))
                elif self.mode == "legacy":  # float16 metres
                    self.depth[c].append(d.astype(np.float16))
                if last or self.mode == "frames":
                    self.last_depth[c] = d
        self.prop.append(np.asarray(self._prop(), dtype=np.float32).reshape(-1))
        self.steps.append(int(self._clock()))

    def parts(self, video: str, fps: int) -> list:
        if self.mode == "last" and not self.last_depth and self.prop:   # no last=True tick: the state has not moved since
            _, dep = self._grab(self.cams, True)
            self.last_depth = {c: clean_depth(dep[c]) for c in self.cams if c in dep}
        out = []
        for c in self.cams:
            frames = self.frames[c]
            good = [f for f in frames if f is not None]
            hw = good[-1].shape[:2] if good else (1, 1)
            if len(good) < len(frames):
                frames[:] = [f if f is not None else np.zeros(hw + (3,), np.uint8) for f in frames]
                self.depth[c] = [d if d.shape == hw else np.zeros(hw, d.dtype) for d in self.depth[c]]
                if c in self.last_depth and self.last_depth[c].shape != hw:
                    self.last_depth[c] = np.zeros(hw, np.float32)
            del good
            # One copy of the frames: no np.stack; frames/<cam> is written into the body from this list (rcb1.Frames,
            # which empties it), the mp4 is encoded from it, and a list nothing sends is let go of here.
            if video in ("mp4", "both") and frames:
                out.append((f"video/{c}", "mp4", encode_mp4(frames, fps), {"fps": fps, "n": int(len(frames))}))
            last = frames[-1] if frames else None
            if video in ("frames", "both"):
                out.append(rcb1.frames_part(f"frames/{c}", frames, empty=np.zeros((0, 1, 1, 3), np.uint8)))
            self.frames[c] = []
            if last is not None:
                out.append(rcb1.array_part(f"last/{c}", last))
            out.append(rcb1.array_part(f"extrinsic_cv/{c}", np.stack(self.ext[c]) if self.ext[c] else np.zeros((0, 3, 4))))
            if self.mode in ("last", "frames") and c in self.last_depth:
                out.append(rcb1.array_part(f"last_depth/{c}", self.last_depth[c]))
            if self.mode == "frames" and self.depth[c]:
                out.append(depth_frames_part(f"depth_frames/{c}", self.depth[c]))
            if self.mode == "legacy" and self.depth[c]:
                out.append(rcb1.frames_part(f"depth/{c}", self.depth[c]))
            self.depth[c] = []
        out.append(rcb1.array_part("proprio", np.stack(self.prop) if self.prop else np.zeros((0, 0), np.float32)))
        st = np.asarray(self.steps, dtype=np.int64)
        out.append(rcb1.array_part("frame_step", st))
        out.append(rcb1.array_part("frame_time_s", st.astype(np.float64) / self.hz))
        return out


def encode_response(meta: dict, parts: list, max_bytes: int = MAX_RESP) -> bytes:
    """The RCB1 body; over `max_bytes` the per-frame depth goes first, then the raw frames (mp4, last frames, last
    depth, proprio and the per-frame arrays always stay), and meta["degraded"] says what was dropped. The size is known
    from the parts before anything is built (rcb1.size), so the body is built once. The per-frame depth (Deflate) is
    compressed only while it fits in the room the rest of the response leaves: compression stops as soon as its output
    passes it, and the depth is then dropped. A dropped part is let go of at once (`parts` is edited in place), so
    the peak is the captured frames plus one body."""
    if not isinstance(parts, list):
        parts = list(parts)
    if any(isinstance(p[2], Deflate) for p in parts):
        room = max_bytes - rcb1.size(meta, [(n, k, b"" if isinstance(b, Deflate) else b, e) for n, k, b, e in parts])
        for i, (n, k, b, e) in enumerate(parts):
            if isinstance(b, Deflate):
                z = b.compress(room) if room >= 0 else None
                if z is None:                        # over the limit: kept as its upper bound, dropped below
                    b.frames.clear()
                    room = -1
                else:
                    parts[i] = (n, k, z, e)
                    room -= len(z)
    dropped, m = [], meta
    for prefixes in (("depth_frames/", "depth/"), ("frames/",)):
        if rcb1.size(m, parts) <= max_bytes:
            break
        keep = [p for p in parts if not p[0].startswith(prefixes)]
        if len(keep) < len(parts):
            for p in parts:
                if p[0].startswith(prefixes) and isinstance(p[2], rcb1.Frames):
                    p[2].arrays.clear()              # not sent: let go of its frames now
                elif p[0].startswith(prefixes) and isinstance(p[2], Deflate):
                    p[2].frames.clear()
            parts[:] = keep; dropped += [x.rstrip("/") for x in prefixes]
            m = {**meta, "degraded": f"response over {max_bytes} bytes: {', '.join(dropped)} dropped"}
    return rcb1.encode(m, parts)


# ---------------------------------------------------------------- kinematics
def _vec(x, n: int, what: str) -> list:
    a = np.asarray(x, dtype=np.float64).reshape(-1) if x is not None else None
    if a is None or a.size != n or not np.all(np.isfinite(a)):
        raise ValueError(f"{what} must be {n} finite numbers")
    return a.tolist()


def _pose(x, what: str) -> list:
    p = _vec(x, 7, what)
    nq = math.sqrt(sum(v * v for v in p[3:]))
    if nq < 1e-6:
        raise ValueError(f"{what}: quaternion has zero norm")
    return p[:3] + [v / nq for v in p[3:]]


def parse_kinematics(body: dict, kin: dict) -> dict:
    """Validate a /kinematics request against spec["kinematics"] (ValueError -> 400). Returns
    {"op": "fk", "q", "links", "base_pose"} or {"op": "ik", "targets": {frame: pose7}, "seed", "base_pose", "tol_pos_m",
    "tol_rot_rad", "max_iters"}; poses are x y z + a unit quaternion in the spec's order; q follows kin["joint_names"]."""
    if not isinstance(body, dict):
        raise ValueError("body must be a JSON object")
    op = body.get("op"); n = len(kin["joint_names"])
    base = None
    if body.get("base_pose") is not None:
        if kin.get("frame") != "base":
            raise ValueError("base_pose is only accepted by a mobile robot (spec kinematics.frame == 'base')")
        base = _pose(body["base_pose"], "base_pose")
    if op == "fk":
        links = body.get("links") or list(kin["links"])
        if not isinstance(links, list) or any(l not in kin["links"] for l in links):
            raise ValueError(f"links must be a list of names from spec kinematics.links: {kin['links']}")
        return {"op": "fk", "q": _vec(body.get("q"), n, "q"), "links": links, "base_pose": base}
    if op == "ik":
        tg = body.get("targets")
        if not isinstance(tg, dict) or not tg or any(k not in kin["ik_targets"] for k in tg):
            raise ValueError(f"targets must map frames from spec kinematics.ik_targets {kin['ik_targets']} to poses")
        ik = kin.get("ik", {})
        seed = None if body.get("seed") is None else _vec(body["seed"], n, "seed")
        tol_p = float(body.get("tol_pos_m", ik.get("tol_pos_m", 0.005)))
        tol_r = float(body.get("tol_rot_rad", ik.get("tol_rot_rad", 0.05)))
        it = int(body.get("max_iters", ik.get("max_iters", 200)))
        if not (1e-5 <= tol_p <= 0.5 and 1e-4 <= tol_r <= 3.2 and 1 <= it <= 5000):
            raise ValueError("tol_pos_m in [1e-5, 0.5], tol_rot_rad in [1e-4, 3.2], max_iters in [1, 5000]")
        return {"op": "ik", "targets": {k: _pose(v, f"targets[{k}]") for k, v in tg.items()}, "seed": seed, "base_pose": base,
                "tol_pos_m": tol_p, "tol_rot_rad": tol_r, "max_iters": it}
    raise ValueError("op must be 'fk' or 'ik'")


def fk_response(poses: dict, frame: str) -> dict:
    return {"ok": True, "op": "fk", "frame": frame, "poses": {k: [float(v) for v in np.asarray(p, dtype=np.float64).reshape(7)] for k, p in poses.items()}}


def ik_response(q, ok: bool, errors: dict, frame: str, note: str | None = None) -> dict:
    """errors: {frame: (position error m, rotation error rad)} of the returned q (None / non-finite -> null: no solution)."""
    fin = lambda v: float(v) if v is not None and math.isfinite(float(v)) else None  # noqa: E731
    out = {"ok": bool(ok), "op": "ik", "frame": frame, "q": None if q is None else [float(v) for v in np.asarray(q).reshape(-1)],
           "error": {k: {"pos_m": fin(e[0]), "rot_rad": fin(e[1])} for k, e in errors.items()}}
    if note:
        out["note"] = note
    return out


def quat_angle(q1, q2) -> float:
    """Rotation angle (rad) between two unit quaternions of the same order."""
    d = abs(float(np.dot(np.asarray(q1, dtype=np.float64), np.asarray(q2, dtype=np.float64))))
    return 2.0 * math.acos(min(1.0, d))


# ---------------------------------------------------------------- /spec self-check
FORBIDDEN_KEY = re.compile(r"mask|segment|semantic(?!s$)|instance_?id|bbox|bounding|categor|object|\bobj_|label|mass|friction|"
                           r"restitution|inertia|density|damping|stiffness", re.I)
GAIN_PATHS = ("robot_spec.controller",)      # joint drive gains (stiffness, damping) are the robot's own, and allowed there
POSITION_KEY = re.compile(r"(?:^|_)(?:pos|position|positions|pose|poses|origin|location|coords?|coordinates|world|translation)(?:_|$)|"
                          r"(?:^|_)cent(?:er|re)s?(?:_|$)|(?:^|_)xyz?$|(?:^|_)(?:top|surface|floor|table|plane|height|elevation)?_?z$", re.I)
WORLD_TEXT = re.compile(r"\bworld\s+[xyz]\s*=", re.I)
COORDS = re.compile(r"\(\s*-?\d+(?:\.\d+)?\s*,\s*-?\d+(?:\.\d+)?\s*(?:,\s*-?\d+(?:\.\d+)?\s*)?\)")


def world_text(s: str) -> bool:
    """A text that gives a world coordinate: a pixel-to-world formula, or a coordinate tuple with a decimal point that is
    not the robot's own pose ("robot base at (0, 0, 1.08)" stays)."""
    return bool(WORLD_TEXT.search(s)) or any("." in m.group(0) and not re.search(r"base|robot|origin", s[max(0, m.start() - 40):m.start()], re.I)
                                             for m in COORDS.finditer(s))
CONVENTIONS = {"static.floor_z": 0.0}       # BEHAVIOR: the floor is the plane z = 0 (the world frame's definition)


def static_positions(obj: Any, path: str = "static") -> list[str]:
    """Paths in /spec static that say WHERE something is (keys naming positions, heights, centres; texts with world
    coordinates). Names, inventories, sizes and colours may stay in static; positions never (docs § Never provided)."""
    out = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            p = f"{path}.{k}"
            if POSITION_KEY.search(str(k)) and CONVENTIONS.get(p, object()) != v:
                out.append(p)
            else:
                out += static_positions(v, p)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            out += static_positions(v, f"{path}[{i}]")
    elif isinstance(obj, str) and world_text(obj):
        out.append(path)
    return out


SPEC_REQUIRED = ("protocol", "protocol_compat", "simulator", "robot", "task_text", "control_hz", "cameras", "default_cameras",
                 "action_types", "proprio_layout", "proprio_groups", "static", "world", "budget", "robot_spec", "kinematics", "depth")


def forbidden_keys(obj: Any, path: str = "", skip: tuple = ()) -> list[str]:
    """Key paths that name something the protocol never provides (key paths in `skip` are not searched)."""
    out = []
    if isinstance(obj, dict):
        for k, v in obj.items():
            p = f"{path}.{k}" if path else str(k)
            if p in skip:
                continue
            if FORBIDDEN_KEY.search(str(k)) and not (re.search(r"damping|stiffness", str(k), re.I) and p.startswith(GAIN_PATHS)):
                out.append(p)
            out += forbidden_keys(v, p, skip)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            out += forbidden_keys(v, f"{path}[{i}]", skip)
    return out


def check_spec(spec: dict) -> None:
    """Raise if /spec is not a complete 2.2 spec or names anything forbidden (run at startup, before binding :8000)."""
    miss = [k for k in SPEC_REQUIRED if k not in spec]
    if miss:
        raise RuntimeError(f"/spec misses {miss}")
    if spec["protocol"] != PROTOCOL:
        raise RuntimeError(f"/spec protocol {spec['protocol']!r} != {PROTOCOL}")
    bad = forbidden_keys(spec, skip=())
    if bad:
        raise RuntimeError(f"/spec names forbidden fields: {bad}")
    pos = static_positions(spec["static"])
    if pos:
        raise RuntimeError(f"/spec static says where things are (never provided): {pos}")
    lay = spec["proprio_layout"]
    if len(set(lay)) != len(lay):
        raise RuntimeError("proprio_layout names are not unique")
    for g in PROPRIO_GROUPS_REQUIRED:
        if g not in spec["proprio_groups"]:
            raise RuntimeError(f"proprio_groups misses {g}")
    for g, (first, n, _meaning) in spec["proprio_groups"].items():
        if not (0 <= first and first + n <= len(lay)):
            raise RuntimeError(f"proprio group {g} [{first}, +{n}) outside the {len(lay)} columns")
    kin = spec["kinematics"]
    for k in ("endpoint", "joint_names", "frame", "quaternion_order", "links", "tcp_frames", "ee_frames", "ik_targets"):
        if k not in kin:
            raise RuntimeError(f"kinematics misses {k}")
    if any(j not in lay for j in kin["joint_names"]):
        raise RuntimeError("kinematics.joint_names must be proprio_layout columns (the measured joint positions)")
    if any(f not in kin["links"] for f in list(kin["tcp_frames"]) + list(kin["ee_frames"]) + list(kin["ik_targets"])):
        raise RuntimeError("kinematics tcp_frames / ee_frames / ik_targets must be listed in kinematics.links")
    if spec["proprio_groups"]["tcp_pose"][1] != 7 * len(kin["tcp_frames"]):
        raise RuntimeError("proprio group tcp_pose must hold 7 columns per kinematics.tcp_frames entry")
    for c, cam in spec["cameras"].items():
        for k in ("rgb", "depth", "mounted", "resolution", "intrinsic_cv", "mount"):
            if k not in cam:
                raise RuntimeError(f"cameras.{c} misses {k}")
        if cam["mount"].get("type") not in ("static", "link"):
            raise RuntimeError(f"cameras.{c}.mount.type must be static|link")
    if any(c not in spec["cameras"] for c in spec["default_cameras"]):
        raise RuntimeError("default_cameras must be cameras")


# ---------------------------------------------------------------- crash recovery (PROTOCOL §4.7; docs/protocol/standard_mode.md)
MAX_RESUMES = int(os.environ.get("RCB_MAX_RESUMES", "2"))
STARTUP_TIMEOUT_S = float(os.environ.get("RCB_STARTUP_TIMEOUT_S", "900"))     # a task with a slower scene build sets more
MAX_CALL_WALL_S = float(os.environ.get("RCB_MAX_CALL_WALL_S", "900"))         # < the client's 1800 s /submit read timeout
RESUME_MAX_DEV_M = float(os.environ.get("RCB_RESUME_MAX_DEV_M", "0.05"))      # approx resumes: the gross-divergence guard
ON_SERVICE_ERROR = ("a connection error means the service crashed: it restarts and resumes the SAME episode (never a reset) "
                    "in the state after the last call it completed, which you may not have received. Wait until GET /status "
                    "answers again (rcb_client: Robot.wait_ready(); resuming replays the episode so far, which can take "
                    "minutes in a long episode, and the wall clock keeps running), then check calls and control_steps "
                    "before you continue. A 500 means the simulator failed: the episode is over (reason \"error\"). A call "
                    "that runs longer than max_call_wall_s seconds stops at the next control step and returns what it "
                    "executed (meta.stopped, executed_rows).")


CLOCK_NOTE = ("meta time_s / time_left_s count from your first request to the service (any endpoint), not from the "
              "service's start, and keep running through a restart of the service; time_left_s = budget.wall_clock_s - "
              "time_s. The harness counts the same limit from the start of your session, so time_left_s never runs out "
              "before the harness stops you: your real time left is time_left_s minus the time you spent before that "
              "first request. Plan by time_left_s.")


def agent_time(state: dict) -> float:
    """Seconds since the agent's first request to the service (0.0 before it): the clock of meta time_s / time_left_s.
    Harbor counts the agent's cutoff around the agent's run only, and the sidecar starts with the environment, a
    minute or more before that (the harness is installed in between): a clock started with the service, or when the
    scene is ready, ran ahead of the agent's and time_left_s reached 0 while the agent still had that time."""
    t = state.get("t_first")
    return 0.0 if t is None else max(0.0, time.time() - float(t))


def _jsonable(v: Any) -> Any:
    if isinstance(v, np.ndarray):
        return v.tolist()
    if isinstance(v, (np.floating, np.integer, np.bool_)):
        return v.item()
    if isinstance(v, (list, tuple)):
        return [_jsonable(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _jsonable(x) for k, x in v.items()}
    return v


def _peer_closed(sock) -> bool:
    """True when the client of an HTTP request has closed its connection (it gave up on the call): the socket reads as
    end-of-file. A client never sends anything while it waits for the response, so readable means closed."""
    try:
        r, _, _ = select.select([sock], [], [], 0)
        return bool(r) and sock.recv(1, socket.MSG_PEEK) == b""
    except (OSError, ValueError):
        return True


def _read_json(path: str) -> dict:
    try:
        with open(path) as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _write_json(path: str, obj: dict) -> None:
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "w") as f:
        json.dump(obj, f)
    os.replace(tmp, path)


# ---------------------------------------------------------------- ablations (off by default: nothing changes when off)
# RCB_STATE_OPS=1 (the unlimited-reset ablation): the layer serves POST /reset (RCB_RESET_BUDGET times), /save_state and
# /load_state (StateOps), counts them, logs one entry per episode (a reset or a load begins a new one) and replays them on
# a crash resume; final.json adds resets_at_success, loads_at_success, saves, loads and episodes_log (state_ops_fields).
STATE_OPS = os.environ.get("RCB_STATE_OPS") == "1"
MAX_HANDLES = int(os.environ.get("RCB_MAX_STATE_HANDLES", "32"))
# The state-query ablation (off unless the task sets RCB_STATE_QUERY=1; nothing changes when off): GET /state/entities
# lists what the agent may ask about (the adapter's StateTable: every entity's kind and fields, with units), and POST
# /state/query {"names": [...], "fields": [...]} answers their current values in the world frame. A query only reads:
# no control step, the clock neither advances nor pauses (on a realtime clock it answers at the current simulation
# time), served only while the episode runs (409 after its end). episode.jsonl logs each answered query, final.json
# adds state_queries (their count). A service without a state table refuses to start with the switch on.
STATE_QUERY = os.environ.get("RCB_STATE_QUERY") == "1"
STATE_KINDS = ("robot", "robot_link", "object", "articulation", "scene_feature")
MAX_QUERY_NAMES = 512
STATE_QUERY_NOTE = (
    "privileged state (an ablation of this run): GET /state/entities lists every entity you may query, {\"frame\", "
    "\"quaternion_order\", \"entities\": {name: {\"kind\", \"fields\": {field: unit}}}}; kinds are robot, robot_link, "
    "object, articulation and scene_feature. POST /state/query {\"names\": [names], \"fields\": [fields] (optional; "
    "default: every field of each name)} answers JSON {\"state\": {name: {field: value}}, \"control_steps\", "
    "\"sim_time_s\", \"state_queries\"}: the current values in the world frame `frame`, in the units listed; a "
    "requested field that an entity does not have is left out of its entry. An unknown name or field is a 400 that "
    "lists the valid ones. A query only reads the simulator: it takes no control step and does not advance or pause "
    "the clock (on a realtime clock it answers at the current simulation time). It is served while the episode runs "
    "(409 after it ended). Every query is counted (state_queries)")


def state_value(v, digits: int = 6):
    """A queried value as JSON holds it: numbers rounded (NaN / inf as null), arrays as (nested) lists, dicts, lists and
    tuples taken apart, strings, bools and None as they are."""
    if isinstance(v, dict):
        return {str(k): state_value(x, digits) for k, x in v.items()}
    if v is None or isinstance(v, str):
        return v
    if isinstance(v, (bool, np.bool_)):
        return bool(v)
    if isinstance(v, (int, np.integer)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        f = float(v)
        return round(f, digits) if np.isfinite(f) else None
    if isinstance(v, np.ndarray):
        return state_value(v.tolist(), digits)
    if isinstance(v, (list, tuple)):
        return [state_value(x, digits) for x in v]
    raise TypeError(f"a state value must be numbers, arrays, strings or dicts of them, not {type(v).__name__}")


class StateTable:
    """The state-query ablation's entities (RCB_STATE_QUERY=1), built by an adapter once its scene is built:
        add(name, kind, {field: (unit, getter)}, note=None)
    for each entity, kind one of STATE_KINDS; getter() reads the field's current value from the live simulator, in the
    world frame (an array, a number, a bool, a string or a dict of them). The usual fields: pos [x, y, z] m, quat [4]
    (quaternion_order), lin_vel [3] m/s, ang_vel [3] rad/s, height_above_ground m, joint_pos / joint_vel {joint: rad or
    m (/s)}, size [3] m (full extents along the body axes), bbox [[x, y, z] min, [x, y, z] max] m; a benchmark adds its
    own (a coverage fraction, a line's endpoints). The entities and their fields are fixed for the episode. Never a
    task predicate, a success flag, a progress or a grader value."""

    def __init__(self, frame: str, quaternion_order: str = "wxyz", note: str | None = None):
        if quaternion_order not in ("wxyz", "xyzw"):
            raise ValueError("quaternion_order is wxyz or xyzw")
        self.frame, self.quaternion_order, self.note = str(frame), quaternion_order, note
        self._ents: dict = {}

    def add(self, name: str, kind: str, fields: dict, note: str | None = None) -> "StateTable":
        if kind not in STATE_KINDS:
            raise ValueError(f"kind {kind!r} is not one of {STATE_KINDS}")
        if not isinstance(name, str) or not name or name in self._ents:
            raise ValueError(f"entity name {name!r} empty or added twice")
        if not fields:
            raise ValueError(f"entity {name!r} has no fields")
        for f, spec in fields.items():
            if not (isinstance(f, str) and isinstance(spec, tuple) and len(spec) == 2 and isinstance(spec[0], str)
                    and callable(spec[1])):
                raise ValueError(f"{name}.{f}: a field is (unit text, getter)")
        self._ents[name] = {"kind": kind, "fields": dict(fields), "note": note}
        return self

    def entities(self) -> dict:
        ents = {n: {"kind": e["kind"], "fields": {f: u for f, (u, _) in e["fields"].items()},
                    **({"note": e["note"]} if e["note"] else {})} for n, e in self._ents.items()}
        return {"frame": self.frame, "quaternion_order": self.quaternion_order, **({"note": self.note} if self.note else {}),
                "entities": ents}

    def query(self, plan: dict) -> dict:
        """plan = {name: [fields]}, already checked (plan_state_query) -> {name: {field: value}}."""
        return {n: {f: state_value(self._ents[n]["fields"][f][1]()) for f in fs} for n, fs in plan.items()}


def check_state_table(table) -> dict:
    """The table's entities() as GET /state/entities serves it, checked (a malformed table refuses to start)."""
    if not (callable(getattr(table, "entities", None)) and callable(getattr(table, "query", None))):
        raise TypeError("RCB_STATE_QUERY=1: the state table must have entities() and query(plan)")
    ents = table.entities()
    if not isinstance(ents, dict) or not isinstance(ents.get("entities"), dict) or not ents["entities"] \
            or not isinstance(ents.get("frame"), str) or ents.get("quaternion_order") not in ("wxyz", "xyzw"):
        raise TypeError("RCB_STATE_QUERY=1: entities() must give frame, quaternion_order and a non-empty entities dict")
    for n, e in ents["entities"].items():
        if e.get("kind") not in STATE_KINDS or not isinstance(e.get("fields"), dict) or not e["fields"] \
                or not all(isinstance(u, str) for u in e["fields"].values()):
            raise TypeError(f"RCB_STATE_QUERY=1: entity {n!r} needs a kind in {STATE_KINDS} and fields {{field: unit}}")
    json.dumps(ents, allow_nan=False)
    return ents


def plan_state_query(ents: dict, body: dict) -> tuple[dict | None, dict | None]:
    """A /state/query body -> ({name: [fields]}, None), or (None, the 400's JSON body)."""
    table = ents["entities"]
    extra = sorted(set(body) - {"names", "fields"})
    if extra:
        return None, {"error": f"unknown keys {extra}: the body is {{\"names\": [...], \"fields\": [...]}}"}
    names = body.get("names")
    if not isinstance(names, list) or not names or not all(isinstance(n, str) for n in names):
        return None, {"error": "names must be a non-empty list of entity names (GET /state/entities)",
                      "names": list(table)}
    names = list(dict.fromkeys(names))
    if len(names) > MAX_QUERY_NAMES:
        return None, {"error": f"at most {MAX_QUERY_NAMES} names per query"}
    unknown = [n for n in names if n not in table]
    if unknown:
        return None, {"error": "unknown names", "unknown": unknown, "names": list(table)}
    fields = body.get("fields")
    if fields is None:
        return {n: list(table[n]["fields"]) for n in names}, None
    if not isinstance(fields, list) or not fields or not all(isinstance(f, str) for f in fields):
        return None, {"error": "fields must be a non-empty list of field names, or left out for all of them"}
    fields = list(dict.fromkeys(fields))
    have = {f for n in names for f in table[n]["fields"]}
    bad = [f for f in fields if f not in have]
    if bad:
        return None, {"error": "unknown fields for these names", "unknown": bad,
                      "fields": {n: list(table[n]["fields"]) for n in names}}
    return {n: [f for f in fields if f in table[n]["fields"]] for n in names}, None


class Refusal(Exception):
    """A state op the layer turns down before anything changes (HTTP code, JSON body)."""

    def __init__(self, code: int, body: dict):
        super().__init__(body.get("error")); self.code, self.body = code, body


class StateOps:
    """The reset ablation's state operations (RCB_STATE_OPS=1), served by the layer for an adapter that hands it:
        reset()        the simulator back to the frozen start AND the adapter's per-episode state cleared (its graded rows,
                       recorder, latches, counters); no journal row
        save() -> obj  an in-memory snapshot of the full simulator state AND that per-episode state (a copy of the graded
                       rows from the frozen start included, so the graded episode stays a replayable path); optional
        load(obj)      restore one; optional (save and load together, or neither: reset-only)
    The layer owns the rest: the budget (RCB_RESET_BUDGET resets; saves and loads are not counted against it), the
    handles (opaque "s<k>", at most MAX_HANDLES held, least recently used dropped: loading a dropped one is a 409), the
    409s (after a success, once the agent's time is up, after the trial was given up; a save while the episode has ended),
    the episode's E keys (done, reason, success, t_end cleared by a reset or a load), the counters, the episode log and
    the replay of every op after a crash (an op is logged with the journal row it happened at)."""

    def __init__(self, *, budget: int, reset: Callable, save: Callable | None = None, load: Callable | None = None,
                 max_handles: int = MAX_HANDLES):
        if (save is None) != (load is None):
            raise ValueError("StateOps: save and load come together")
        self.budget, self.reset_fn, self.save_fn, self.load_fn = int(budget), reset, save, load
        self.max_handles = int(max_handles)
        self.resets = self.saves = self.loads = 0
        self.handles: dict = {}              # handle -> snapshot, oldest use first
        self.episode = 1
        self.ep_rows0 = 0                    # the journal row the running episode began at

    @property
    def can_save(self) -> bool:
        return self.save_fn is not None

    def spec(self) -> dict:
        out = {"reset": ("POST /reset (JSON body as /observe's: cameras, video, depth) abandons the running episode, or follows "
                         f"one that has ended, and starts a new one from the same starting state, {self.budget} times in "
                         "all; only the last episode is graded, and a success ends the trial (409 after it). It answers "
                         "as /observe with seconds 0 does. meta and /status add episode, resets_left, saves, loads and "
                         "control_steps_total")}
        if self.can_save:
            out["state_ops"] = ("POST /save_state ({}) stores the current state of the running episode in this service and "
                                "answers {\"handle\": \"s<k>\", \"step\": control steps of the episode, \"episode\"}: the "
                                "handle is all you get, never the state's contents. POST /load_state {\"handle\": h, cameras, "
                                "video, depth} restores a state saved earlier in this trial and starts a new episode there "
                                "(answered as /observe with seconds 0); nothing else can be loaded. The task's success rule "
                                f"is checked again from your next command on. At most {self.max_handles} handles are kept: "
                                "saving more drops the least recently used one (loading it is a 409). Saves and loads do "
                                "not count against the reset budget; a success ends the trial (409 after it)")
        return out

    def counters(self, state: dict) -> dict:
        over = bool(state.get("exiting")) or bool(state.get("success") and state.get("done"))
        return {"episode": self.episode, "resets_left": 0 if over else self.budget - self.resets, "saves": self.saves,
                "loads": self.loads}

    def check(self, op: str, state: dict, body: dict) -> Any:
        """Every refusal before anything changes; returns the snapshot for a load."""
        if state.get("exiting") or state.get("finalized"):
            raise Refusal(409, {"error": "trial finished: your time is up", "reason": state.get("reason"), "resets_left": 0})
        if state.get("success") and state.get("done"):
            raise Refusal(409, {"error": "trial finished: an episode succeeded", "reason": state.get("reason"),
                                "resets_left": 0})
        if op == "reset" and self.resets >= self.budget:
            raise Refusal(409, {"error": "reset budget exhausted", "budget": self.budget, "resets_left": 0})
        if op in ("save", "load") and not self.can_save:
            raise Refusal(404, {"error": "no such endpoint"})
        if op == "save" and state.get("done"):
            raise Refusal(409, {"error": "the episode has ended: reset or load a saved state first",
                                "reason": state.get("reason")})
        if op == "load":
            h = body.get("handle")
            if not isinstance(h, str):
                raise ValueError("load_state needs {\"handle\": \"s<k>\"} from an earlier /save_state")
            if h not in self.handles:
                raise Refusal(409, {"error": f"unknown handle {h!r}: not saved in this trial, or dropped (at most "
                                             f"{self.max_handles} are kept)", "handle": h})
            return self.handles[h]
        return None

    def _end_episode(self, state: dict, log: Callable, rows: int, why: str) -> None:
        log({"ev": "episode_end", "episode": self.episode, "ended": state.get("reason") if state.get("done") else why,
             "control_steps": rows - self.ep_rows0})

    def apply(self, op: str, state: dict, rows: int, log: Callable | None, handle: str | None = None,
              snap: Any = None) -> dict:
        """Run one op (live, or replayed after a crash with log=None). rows = the journal rows so far."""
        if op == "save":
            obj = self.save_fn()
            self.saves += 1
            h = handle or f"s{self.saves}"
            self.handles[h] = obj
            while len(self.handles) > self.max_handles:
                self.handles.pop(next(iter(self.handles)))
            if log:
                log({"ev": "op", "op": "save", "at": rows, "handle": h, "episode": self.episode})
            return {"handle": h}
        if log:
            self._end_episode(state, log, rows, op)
        if op == "reset":
            self.reset_fn()
            self.resets += 1
        else:
            self.handles[handle] = self.handles.pop(handle)       # a load counts as a use
            self.load_fn(snap if snap is not None else self.handles[handle])
            self.loads += 1
        self.episode += 1
        self.ep_rows0 = rows
        state.update(done=False, reason=None, t_end=None)
        if "success" in state:
            state["success"] = False
        if log:
            log({"ev": "op", "op": op, "at": rows, "handle": handle, "episode": self.episode})
        return {}

    def replay(self, e: dict, state: dict, rows: int) -> None:
        self.apply(e["op"], state, rows, None, handle=e.get("handle"))

    def fields(self, success: bool, reason: str | None, events: list, t_first: float | None, rows: int) -> dict:
        """final.json / episode.json: resets_at_success and loads_at_success (only for a success), saves, loads and
        episodes_log (per episode: its start and end on the agent's clock, control steps, how it began and ended)."""
        base = t_first if t_first is not None else next((float(e["ts"]) for e in events if e.get("ev") == "start"), 0.0)
        t = lambda e: round(max(0.0, float(e["ts"]) - base), 2)  # noqa: E731
        log, cur = [], {"episode": 1, "began": "start", "t_start_s": 0.0}
        for e in events:
            if e.get("ev") == "episode_end" and e.get("episode") == cur["episode"]:
                log.append({**cur, "t_end_s": t(e), "control_steps": int(e.get("control_steps") or 0), "ended": e.get("ended")})
            elif e.get("ev") == "op" and e.get("op") in ("reset", "load") and e.get("episode") == cur["episode"] + 1:
                cur = {"episode": int(e["episode"]), "began": e["op"], "t_start_s": t(e)}
        log.append({**cur, "t_end_s": round(max(0.0, time.time() - base), 2), "control_steps": rows - self.ep_rows0,
                    "ended": reason})
        out = {"resets": self.resets, "saves": self.saves, "loads": self.loads, "reset_budget": self.budget,
               "episodes": self.episode, "episodes_log": log}
        if success:
            out.update(resets_at_success=self.resets, loads_at_success=self.loads)
        return out


def final_verdict(data: str) -> dict | None:
    """final.json if it already holds a verdict that nothing may overwrite: the grading result (finalized: the live
    episode's, `graded_by` = "live"; before 2026-10-17 a replay's, with `deterministic`) or the end of an
    episode the service gave up after crashes (`gave_up`)."""
    fin = _read_json(os.path.join(data, "final.json"))
    if fin.get("finalized") and (fin.get("gave_up") or fin.get("graded_by") == "live"
                                 or fin.get("deterministic") is not None):
        return fin
    return None


class Episode:
    """Crash-safe episode bookkeeping, the same rules as the RoboTwin and MuJoCo services (PROTOCOL §4.7):

    - startup(): before the scene is built. One line in startups.jsonl; a back-off of min(120, 20 (n - 1)) s when the
      earlier starts never began an episode (they crashed while building, e.g. GPU memory taken by other trials); the
      startup watchdog (faulthandler, a C thread: it fires even when a native scene build holds the GIL) dumps every
      thread's traceback and exits 1 after RCB_STARTUP_TIMEOUT_S, so compose restarts the build. Until begin() a SIGUSR1
      (the collect hook's "the agent phase is over") is only remembered.
    - journal(row): every executed control step (the exact row given to the simulator's step) goes to journal.f64.
    - checkpoint(): after every completed call (the HTTP layer calls it): the journal is flushed, then the call's
      checkpoint is logged in episode.jsonl (rows, control steps, state hash, the robot's proprio row, `persist` keys).
    - begin(): a fresh episode, or after a crash (run.sh exits non-zero, compose restarts the container, `restart:
      on-failure:N`) the SAME episode resumed: the journal is replayed up to the last checkpoint, the journal truncated
      there (the call that was in flight is lost as a whole: its caller got an error, not a result), the state hash
      compared, the service's counters restored; the agent's clock keeps running from its first request (the logged
      `first_request`: a resume never sets it back to full), the episode's own (state["t0"]) from the first start.
    - Resume check, by the simulator's determinism class: `exact` (bit-exact in a fresh process): the hash must match,
      else the episode ends `resume_failed` (infra_error). `approx` (a simulator declared nondeterministic): a
      mismatch is recorded (resume_state_mismatch, and the robot's deviation from its checkpointed proprio: joint_pos and
      tcp_pose) and the episode continues, unless a TCP is more than RCB_RESUME_MAX_DEV_M from where it was
      (resume_failed, infra_error). The verdict is the live episode's (PROTOCOL H.4), so it is judged on the state the
      resumed episode continued from; the mismatch stays in final.json for audit.
    - More than RCB_MAX_RESUMES crashes end the episode: `sim_crashed_on_call` if every crash happened while a call was
      being served (the agent's failure), else `sim_crashed` (infra_error = 1). A verdict already on disk is never
      overwritten; the service then idles with its PID file for the collect hook.
    - Per call, two bounds keep one call from holding the single-threaded service: a call that runs longer than
      max_call_wall_s, or whose client has hung up, stops at the next control step and returns what it executed
      (journal() returns True; meta.stopped says why). Neither bounds the episode: the wall clock is its only budget."""

    def __init__(self, data: str, dim: int, resume: str = "exact", persist: tuple = ()):
        if resume not in ("exact", "approx"):
            raise ValueError("resume must be 'exact' or 'approx'")
        self.data, self.dim, self.resume_class, self.persist = data, int(dim), resume, tuple(persist)
        os.makedirs(data, exist_ok=True)
        p = lambda n: os.path.join(data, n)  # noqa: E731
        self.f_events, self.f_journal, self.f_final, self.f_pid, self.f_startups = (
            p("episode.jsonl"), p("journal.f64"), p("final.json"), p("server.pid"), p("startups.jsonl"))
        self.rows = 0                       # control steps executed in this episode (journaled)
        self.max_call_wall_s = MAX_CALL_WALL_S
        self.n_startups = 0; self.resumes = 0; self.mismatches = 0; self.max_tcp_dev = 0.0; self.max_joint_dev = 0.0
        self._jf = None; self._call: dict | None = None; self._ids = 0; self._ck: tuple | None = None
        self._on_end: Callable | None = None; self._end_pending = False

    # ---- startup
    def startup(self) -> None:
        signal.signal(signal.SIGUSR1, self._usr1)
        try:
            os.remove(self.f_pid)            # a PID file from before a crash would send the collect hook to a dead process
        except OSError:
            pass
        with open(self.f_startups, "a") as f:
            f.write(json.dumps({"ts": time.time(), "pid": os.getpid()}) + "\n")
        with open(self.f_startups) as f:
            self.n_startups = sum(1 for line in f if line.strip())
        if self.n_startups > 1 and not any(e.get("ev") == "start" for e in self.events()):
            time.sleep(min(120.0, 20.0 * (self.n_startups - 1)))
        faulthandler.dump_traceback_later(STARTUP_TIMEOUT_S, exit=True)

    def scene_built(self) -> None:
        faulthandler.cancel_dump_traceback_later()   # the scene exists: a resume replay may take long, it is not a hang

    def _usr1(self, *_):
        if self._on_end is None:
            self._end_pending = True
        else:
            threading.Thread(target=self._on_end, daemon=True).start()

    # ---- the record
    def log(self, ev: dict) -> None:
        with open(self.f_events, "a") as f:
            f.write(json.dumps(_jsonable({"ts": time.time(), **ev})) + "\n")

    def events(self) -> list:
        out = []
        try:
            with open(self.f_events) as f:
                for line in f:
                    try:
                        out.append(json.loads(line))
                    except ValueError:      # a line cut by a crash
                        pass
        except OSError:
            pass
        return [e for e in out if isinstance(e, dict)]

    def journal(self, row) -> bool:
        """One executed control step. True: the call in progress must stop at this step boundary (meta.stopped)."""
        a = np.asarray(row, dtype=np.float64).reshape(-1)
        if a.size != self.dim:
            raise ValueError(f"journal rows have {self.dim} entries, got {a.size}")
        if self._jf is None:
            self._jf = open(self.f_journal, "ab")
        self._jf.write(a.tobytes()); self.rows += 1
        c = self._call
        if c is None:
            return False
        if c["stopped"] is None:
            now = time.monotonic()
            if now - c["t0"] > self.max_call_wall_s:
                c["stopped"] = "max_call_wall_s"
            elif c["sock"] is not None and now - c["polled"] >= 0.2:
                c["polled"] = now
                if _peer_closed(c["sock"]):
                    c["stopped"] = "client_disconnected"
        return c["stopped"] is not None

    @property
    def stopping(self) -> bool:
        """The call in progress must stop (adapters whose loops test E["done"] test this too)."""
        return bool(self._call and self._call["stopped"])

    @property
    def stop_reason(self) -> str | None:
        return self._call["stopped"] if self._call else None

    def call_begin(self, path: str, sock=None) -> int:
        self._ids += 1
        if path != "/kinematics":
            self._call = {"id": self._ids, "t0": time.monotonic(), "sock": sock, "polled": 0.0, "stopped": None}
        self.log({"ev": "call_begin", "id": self._ids, "path": path})
        return self._ids

    def call_end(self, cid: int, path: str, **extra) -> None:
        self.log({"ev": "call_end", "id": cid, "path": path, **extra})
        if path != "/kinematics":
            self._call = None

    def checkpoint(self, state: dict, state_sha: Callable | None, proprio: Callable | None) -> None:
        if self._jf is not None:
            self._jf.flush()                 # the rows reach the file before the checkpoint that counts them
        if self._ck is not None and self._ck[0] == self.rows:
            sha, pr = self._ck[1], self._ck[2]                          # no physics since the last checkpoint
        else:
            try:
                sha = state_sha() if state_sha else None
            except Exception:  # noqa: BLE001
                traceback.print_exc(); sha = None
            pr = self._proprio_row(proprio)
        self._ck = (self.rows, sha, pr)
        self.log({"ev": "checkpoint", "call": int(state.get("calls") or 0), "step": int(state.get("step") or 0),
                  "control_steps": self.rows, "success": bool(state.get("success")), "done": bool(state.get("done")),
                  "reason": state.get("reason"), "state_sha": sha, "proprio": pr,
                  **({"persist": {k: state.get(k) for k in self.persist}} if self.persist else {})})

    @staticmethod
    def _proprio_row(proprio: Callable | None) -> list | None:
        try:
            return [round(float(v), 6) for v in np.asarray(proprio(), dtype=np.float64).reshape(-1)] if proprio else None
        except Exception:  # noqa: BLE001
            traceback.print_exc(); return None

    def final_fields(self) -> dict:
        """Crash-recovery fields for final.json / episode.json (all numeric, for reward.json)."""
        return {"sim_startups": self.n_startups, "resumes": self.resumes, "resume_state_mismatch": int(self.mismatches > 0),
                "resume_max_tcp_dev_m": round(self.max_tcp_dev, 6), "resume_max_joint_dev": round(self.max_joint_dev, 6),
                **({"state_queries": self.state_queries()} if STATE_QUERY else {})}

    def state_queries(self) -> int:
        """RCB_STATE_QUERY: the /state/query calls answered in this trial (each is logged before it is answered)."""
        return sum(1 for e in self.events() if e.get("ev") == "state_query")

    # ---- start or resume
    def begin(self, *, state: dict, step: Callable, state_sha: Callable, proprio: Callable, spec: dict,
              start_event: dict | None = None, on_end: Callable | None = None, ops: StateOps | None = None) -> bool:
        """Start the episode, or resume it after a crash. `step(row)` executes one journaled row on the simulator (the
        recorder on, no rendering needed); `on_end` is what the collect hook's SIGUSR1 runs (in a thread). Returns True
        if the episode was resumed."""
        try:                                 # a line cut by the crash must not swallow the next record
            with open(self.f_events, "rb+") as f:
                f.seek(0, os.SEEK_END)
                if f.tell():
                    f.seek(-1, os.SEEK_END)
                    if f.read(1) != b"\n":
                        f.write(b"\n")
        except OSError:
            pass
        ev = self.events()
        starts = [e for e in ev if e.get("ev") == "start"]
        if not starts:
            try:
                os.remove(self.f_journal)
            except OSError:
                pass
            sha = state_sha()
            self._ck = (0, sha, self._proprio_row(proprio))
            self.log({**(start_event or {}), "ev": "start", "state_sha": sha, "protocol": PROTOCOL, "resume": self.resume_class,
                      "journal_dim": self.dim})
            state["t0"] = time.time()
            self._arm(on_end)
            return False
        tries = [e for e in ev if e.get("ev") == "resuming"]        # earlier resumes (a crash during one counts too)
        resumed = [e for e in ev if e.get("ev") == "resumed"]
        self.resumes = len(tries)                    # the record so far, also for a give-up below (final_fields)
        self.mismatches = sum(1 for e in resumed if e.get("exact") is False)
        self.max_tcp_dev = max([float(e.get("tcp_dev_m") or 0) for e in resumed] + [0.0])
        self.max_joint_dev = max([float(e.get("joint_dev") or 0) for e in resumed] + [0.0])
        if final_verdict(self.data):
            self.give_up("restarted after finalize", infra=False)
        k = max(i for i, e in enumerate(ev) if e.get("ev") in ("start", "resuming", "resumed"))
        tail = ev[k + 1:]
        in_call = bool({e.get("id") for e in tail if e.get("ev") == "call_begin"} - {e.get("id") for e in tail if e.get("ev") == "call_end"})
        if len(tries) >= MAX_RESUMES:
            agent = sum(1 for e in tries if e.get("crashed_in_call")) + int(in_call) == len(tries) + 1
            self.give_up("sim_crashed_on_call" if agent else "sim_crashed", infra=not agent)
        self.log({"ev": "resuming", "attempt": len(tries) + 1, "crashed_in_call": in_call,
                  "downtime_s": round(time.time() - float(ev[-1].get("ts") or time.time()), 1)})
        self.resumes = len(tries) + 1                # this attempt counts, whether or not it succeeds
        cps = [e for e in ev if e.get("ev") == "checkpoint"]
        last = cps[-1] if cps else {"call": 0, "step": 0, "control_steps": 0, "success": False, "state_sha": starts[0].get("state_sha")}
        n = int(last.get("control_steps") or 0)
        try:
            R = np.fromfile(self.f_journal, dtype=np.float64)
        except (OSError, ValueError):
            R = np.zeros(0)
        if R.size < n * self.dim:
            self.give_up("resume_failed: journal shorter than the log", infra=True)
        R = R[: n * self.dim].reshape(n, self.dim)
        t = time.time()
        # the state ops (RCB_STATE_OPS) of the calls that completed (logged before the last checkpoint), at their rows
        ck = max((i for i, e in enumerate(ev) if e.get("ev") == "checkpoint"), default=-1)
        opq = [e for i, e in enumerate(ev) if e.get("ev") == "op" and i < ck] if ops is not None else []
        j = 0
        for i, row in enumerate(R):
            while j < len(opq) and int(opq[j]["at"]) <= i:
                ops.replay(opq[j], state, i); j += 1
            step(row)
        while j < len(opq):
            ops.replay(opq[j], state, n); j += 1
        replay_s = time.time() - t
        R.tofile(self.f_journal)            # drop the rows of the call that was in flight
        self.rows = n
        sha = state_sha(); want = last.get("state_sha")
        exact = None if want is None else sha == want
        dev = self._deviation(last.get("proprio"), proprio, spec)
        if exact is False:                           # recorded first: a give-up below reports it too
            self.mismatches += 1
            self.max_tcp_dev = max(self.max_tcp_dev, dev.get("tcp_m") or 0.0)
            self.max_joint_dev = max(self.max_joint_dev, dev.get("joint") or 0.0)
            if self.resume_class == "exact":
                self.give_up("resume_failed: replayed state differs from the logged one", infra=True)
            if dev.get("tcp_m") is not None and dev["tcp_m"] > RESUME_MAX_DEV_M:
                self.give_up(f"resume_failed: a replayed TCP is {dev['tcp_m']:.3f} m from its checkpoint (> {RESUME_MAX_DEV_M} m)", infra=True)
        state["calls"] = int(last.get("call") or 0); state["step"] = int(last.get("step") or 0)
        if "success" in state:
            state["success"] = bool(last.get("success"))
        for key, v in (last.get("persist") or {}).items():
            state[key] = v
        state["t0"] = float(starts[0]["ts"])
        first = [e for e in ev if e.get("ev") == "first_request"]
        if first:                                    # the agent's clock keeps running through a crash, never back to full
            state["t_first"] = float(first[0].get("t_first") or first[0]["ts"])
        ended = [e for e in ev if e.get("ev") == "end" and int(e.get("control_steps") or 0) <= n]   # not the lost call's
        if ops is not None:                          # with state ops the last checkpoint says how the episode stands
            state["done"] = bool(last.get("done")); state["reason"] = last.get("reason")
            if "success" in state:
                state["success"] = bool(last.get("success"))
            ended = []
        if ended:
            state["done"] = True; state["reason"] = ended[-1].get("reason"); state["t_end"] = float(ended[-1]["ts"])
            state["episode_control_steps"] = int(ended[-1].get("control_steps") or n)
        self._ck = (n, sha, self._proprio_row(proprio))           # the replayed state's row (it may differ under approx)
        self.log({"ev": "resumed", "attempt": self.resumes, "control_steps": n, "calls": state["calls"], "state_sha": sha,
                  "logged_sha": want, "exact": exact, "class": self.resume_class, "tcp_dev_m": dev.get("tcp_m"),
                  "joint_dev": dev.get("joint"), "crashed_in_call": in_call, "replay_s": round(replay_s, 1),
                  "downtime_s": round(time.time() - float(ev[-1].get("ts") or time.time()), 1)})
        self._arm(on_end)
        return True

    @staticmethod
    def _deviation(ref, proprio: Callable, spec: dict) -> dict:
        """The replayed robot against its checkpointed proprio row: max |joint_pos| difference (rad or m) and the largest
        TCP position error (m) over the tcp_pose frames (7 columns each: x y z + quaternion)."""
        if ref is None:
            return {}
        try:
            now = np.asarray(proprio(), dtype=np.float64).reshape(-1); ref = np.asarray(ref, dtype=np.float64).reshape(-1)
        except Exception:  # noqa: BLE001
            return {}
        if now.shape != ref.shape:
            return {}
        g, out = spec.get("proprio_groups") or {}, {}
        if "joint_pos" in g:
            f, n = int(g["joint_pos"][0]), int(g["joint_pos"][1])
            out["joint"] = float(np.max(np.abs(now[f:f + n] - ref[f:f + n]))) if n else 0.0
        if "tcp_pose" in g:
            f, n = int(g["tcp_pose"][0]), int(g["tcp_pose"][1]); m = n // 7
            d = (now[f:f + 7 * m] - ref[f:f + 7 * m]).reshape(m, 7)[:, :3]
            out["tcp_m"] = float(np.max(np.linalg.norm(d, axis=1))) if m else 0.0
        return out

    def _arm(self, on_end: Callable | None) -> None:
        self._on_end = on_end
        signal.signal(signal.SIGUSR1, self._usr1)
        if on_end is not None and self._end_pending:
            self._end_pending = False
            threading.Thread(target=on_end, daemon=True).start()

    def give_up(self, reason: str, infra: bool) -> None:
        """End the trial's episode without a new one (never a reset): log it, write the verdict unless one is on disk,
        leave the PID file for the collect hook, and answer every request with the refusal until the container is
        removed (refuse_forever)."""
        faulthandler.cancel_dump_traceback_later()
        self.log({"ev": "restart_refused", "reason": reason})
        if not final_verdict(self.data):
            fin = _read_json(self.f_final)
            fin.update({"success": False, "live_success": False, "done": True, "reason": reason, "finalized": True,
                        "gave_up": True, "replays": 0, "protocol": PROTOCOL, **({} if infra else {"graded_by": "live"}),
                        **self.final_fields(), **({"infra_error": 1} if infra else {})})
            _write_json(self.f_final, fin)
        with open(self.f_pid, "w") as f:
            f.write(str(os.getpid()))
        signal.signal(signal.SIGUSR1, lambda *_: None)
        print(f"[{PROTOCOL}] restart refused: {reason}", flush=True)
        refuse_forever(reason, infra)


def refuse_forever(reason: str, infra: bool, port: int | None = None) -> None:
    """After a give-up: answer every request 409 {"error", "reason", "gave_up": true, "infra_error"} until the container
    is removed. Before, nothing listened once the service had given up -- whether it refused the episode at its start
    (DexJoCo with fewer than 4 CPUs, a frozen instance that does not rebuild) or after its crashes -- and the client's
    Robot() and wait_ready() waited out their whole timeout, an hour, for a simulator that would never answer. The
    client raises EpisodeOver on this answer at once. The verdict is on disk already: nothing here reads or changes it."""
    body = json.dumps({"error": "the sim service gave up on this episode and will not serve it", "reason": reason,
                       "gave_up": True, "infra_error": int(bool(infra))}).encode()

    class Refusal(BaseHTTPRequestHandler):
        timeout = 30                         # a client that stops sending must not hold the only thread

        def _refuse(self):
            try:
                n = int(self.headers.get("Content-Length") or 0)
                if 0 < n <= MAX_BODY:        # take the request first: a client still sending would see a reset, not the answer
                    self.rfile.read(n)
                self.send_response(409); self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body))); self.end_headers(); self.wfile.write(body)
            except (OSError, ValueError):
                pass
            self.close_connection = True

        do_GET = do_POST = _refuse

        def log_message(self, *a):
            pass

    def answer():
        try:
            bind(int(os.environ.get("RCB_PORT", "8000")) if port is None else port, Refusal).serve_forever()
        except Exception:  # noqa: BLE001 -- the refusal is a courtesy to the client; the give-up stands without it
            traceback.print_exc()

    threading.Thread(target=answer, daemon=True).start()
    while True:
        time.sleep(3600)


# ---------------------------------------------------------------- the HTTP layer
def _finite(v) -> bool:
    try:
        return math.isfinite(float(v))
    except (TypeError, ValueError, OverflowError):
        return False


def parse_request(path: str, meta, parts: dict):
    """What every adapter needs checked before it touches the simulator (ValueError -> 400; PROTOCOL §4.6: a malformed
    request steps no physics and never ends the episode, whatever is wrong with it: a JSON object where rows are
    expected, a number too large for a float, NaN, a field of the wrong JSON type). Returns the action rows of a
    /submit as a float64 array (shape and limits are the adapter's to check), None for /observe."""
    if not isinstance(meta, dict):
        raise ValueError("the request's meta must be a JSON object")
    for k in ("dt", "seconds", "video_fps"):
        if meta.get(k) is not None and not _finite(meta[k]):
            raise ValueError(f"{k} must be a finite number (got {meta[k]!r:.60})")
    for k in ("type", "interpolate", "video"):
        if meta.get(k) is not None and not isinstance(meta[k], str):
            raise ValueError(f"{k} must be a string (got {meta[k]!r:.60})")
    cams = meta.get("cameras")
    if cams is not None and not (isinstance(cams, (list, tuple)) and all(isinstance(c, str) for c in cams)):
        raise ValueError("cameras must be a list of camera names")
    if cams is not None:   # each camera once, in the order asked for: N copies of a name were rendered and returned N * N times
        meta["cameras"] = list(dict.fromkeys(cams))
    if path != "/submit":
        return None
    try:
        src = parts["actions"]["data"] if "actions" in parts else meta.get("actions", [])
        if src is None or isinstance(src, (dict, str, bytes)):
            raise TypeError(f"got {type(src).__name__}")
        A = np.asarray(src, dtype=np.float64)
    except (TypeError, ValueError, OverflowError, AttributeError, KeyError) as exc:
        raise ValueError("actions must be an RCB1 array part `actions` or a JSON list of rows of numbers "
                         f"({type(exc).__name__}: {str(exc)[:120]})") from None
    if not np.all(np.isfinite(A)):
        raise ValueError("actions contain NaN/inf")
    return A


class Service:
    """What a sim service hands to serve(): spec (dict), lock, state (the service's E dict: done, reason, exiting, error),
    submit(meta, actions) and observe(meta) -> (meta_out, rcb1 parts), kinematics(req) -> dict, status() -> dict,
    on_submit_error(exc) (a simulator error in /submit or /observe: it ends the episode, reason "error"), log(event dict),
    and for crash recovery the Episode with the simulator's state_sha() and proprio row()."""

    def __init__(self, *, spec: dict, lock, state: dict, submit: Callable, observe: Callable, kinematics: Callable,
                 status: Callable, on_submit_error: Callable, log: Callable, max_resp: int = MAX_RESP,
                 episode: Episode | None = None, state_sha: Callable | None = None, proprio: Callable | None = None,
                 state_table: Any = None, state_ops: StateOps | None = None, observe_now: Callable | None = None):
        self.spec, self.lock, self.state, self.submit, self.observe = spec, lock, state, submit, observe
        self.kinematics, self.status, self.on_submit_error, self.log, self.max_resp = kinematics, status, on_submit_error, log, max_resp
        self.episode, self.state_sha, self.proprio = episode, state_sha, proprio
        self.kin_calls = 0
        # the ablations' hooks (module comments above STATE_OPS and STATE_QUERY): state_table: a StateTable (or any
        # object with entities() and query(plan)); state_ops: StateOps; observe_now(meta) -> (meta_out, parts), the
        # answer to a reset / load (default: observe with seconds 0)
        self.state_table, self.ops = state_table, state_ops
        self.observe_now = observe_now or (lambda m: observe({**m, "seconds": 0}))
        if STATE_QUERY and state_table is None:
            raise RuntimeError("RCB_STATE_QUERY=1 but this service hands no state_table")
        self.state_ents = check_state_table(state_table) if STATE_QUERY else None
        self.state_queries = None            # RCB_STATE_QUERY: counted from the event log at the first query
        if STATE_OPS and state_ops is None:
            raise RuntimeError("RCB_STATE_OPS=1 but this service hands no StateOps")

    def touch(self) -> None:
        """Every HTTP request passes here: the first one starts the agent's clock (agent_time) and is logged, so that
        a resume keeps it. A probe that must not start it connects to the port without sending a request."""
        if self.state.get("t_first") is None:
            self.state["t_first"] = t = time.time()
            (self.episode.log if self.episode is not None else self.log)({"ev": "first_request", "t_first": t})


def make_handler(svc: Service):
    if STATE_QUERY:
        svc.spec["state_query"] = STATE_QUERY_NOTE
    if svc.ops is not None:
        svc.spec.update(svc.ops.spec())
        if isinstance(svc.spec.get("budget"), dict):
            svc.spec["budget"]["reset"] = svc.ops.budget
    if isinstance(svc.spec.get("budget"), dict):
        svc.spec["budget"].setdefault("clock", CLOCK_NOTE)   # what time_s / time_left_s count from, in every 2.2 /spec

    class H(BaseHTTPRequestHandler):
        def _send(self, code: int, ctype: str, body: bytes) -> None:
            try:
                self.send_response(code); self.send_header("Content-Type", ctype); self.send_header("Content-Length", str(len(body)))
                self.end_headers(); self.wfile.write(body)
            except (BrokenPipeError, ConnectionResetError):   # the client hung up (its call was abandoned)
                pass

        def _json(self, code: int, obj: dict) -> None:
            self._send(code, "application/json", json.dumps(obj).encode())

        def _rcb1(self, meta: dict, parts: list) -> None:
            self._send(200, "application/x-rcb1", encode_response(meta, parts, svc.max_resp))

        def log_message(self, *a):
            pass

        def parse_request(self):             # every request, of any method and path (a subclass's endpoints too)
            ok = super().parse_request()
            if ok:
                svc.touch()
            return ok

        def _body(self):
            try:
                n = int(self.headers.get("Content-Length", 0))
            except ValueError:
                raise ValueError("bad Content-Length")
            if n < 0:                        # read(-1) would read until the client hangs up, without any bound
                raise ValueError("bad Content-Length")
            if n > MAX_BODY:
                raise ValueError("request too large")
            raw = self.rfile.read(n) if n else b""
            if raw[:4] == rcb1.MAGIC:
                return rcb1.decode(raw)
            body = json.loads(raw or b"{}")
            if not isinstance(body, dict):
                raise ValueError("body must be a JSON object or an RCB1 frame")
            return body, {}

        def do_GET(self):
            if STATE_QUERY and self.path == "/state/entities":     # fixed for the episode
                return self._json(200, svc.state_ents)
            if self.path == "/spec":
                return self._json(200, svc.spec)
            if self.path == "/status":
                st = svc.status()
                if svc.ops is not None:
                    st = {**st, **svc.ops.counters(svc.state), "control_steps_total": svc.episode.rows if svc.episode else None}
                if STATE_QUERY:
                    st = {**st, "state_queries": svc.episode.state_queries() if svc.episode else svc.state_queries or 0}
                return self._json(200, {**st, "resumes": svc.episode.resumes} if svc.episode else st)
            return self._json(404, {"error": "no such endpoint"})

        def _state_op(self):
            """/reset, /save_state, /load_state (StateOps; RCB_STATE_OPS=1)."""
            op = {"/reset": "reset", "/save_state": "save", "/load_state": "load"}[self.path]
            try:
                body, _ = self._body()
            except Exception as exc:  # noqa: BLE001
                return self._json(413 if "too large" in str(exc) else 400, {"error": str(exc)[:200]})
            E, ep, ops = svc.state, svc.episode, svc.ops
            with svc.lock:
                try:
                    snap = ops.check(op, E, body)
                    view = {k: body[k] for k in ("cameras", "video_fps", "video", "depth", "include_depth") if k in body}
                    if op != "save":
                        parse_request("/observe", {**view, "seconds": 0}, {})          # a malformed view: 400 first
                except Refusal as r:
                    return self._json(r.code, r.body)
                except ValueError as exc:
                    return self._json(400, {"error": str(exc)})
                cid = ep.call_begin(self.path, self.connection) if ep else 0
                try:
                    E["calls"] = int(E.get("calls") or 0) + 1
                    res = ops.apply(op, E, ep.rows if ep else 0, ep.log if ep else svc.log,
                                    handle=body.get("handle") if op == "load" else None, snap=snap)
                    if ep:
                        ep._ck = None                    # the state changed without a journal row: hash it anew
                    if op == "save":
                        out, blobs = None, None
                    else:
                        calls = E["calls"]                 # the op is one call, whether observe_now counts one or not
                        out, blobs = svc.observe_now({**view, "video": view.get("video", "frames")})
                        E["calls"] = calls
                        if "calls" in out:
                            out = {**out, "calls": calls}
                except Exception as exc:  # noqa: BLE001 -- a simulator that cannot start the new episode: infrastructure
                    traceback.print_exc(); E["error"] = repr(exc)
                    svc.log({"ev": "error", "path": self.path, "error": repr(exc)})
                    E.update(done=True, reason="error", state_op_failed=True)
                    if "success" in E:
                        E["success"] = False
                    return self._json(500, {"error": "simulator error", "detail": repr(exc)[:200]})
                finally:
                    if ep:
                        try:
                            ep.checkpoint(E, svc.state_sha, svc.proprio)
                        except Exception:  # noqa: BLE001
                            traceback.print_exc()
                        ep.call_end(cid, self.path)
                counters = {**ops.counters(E), "control_steps_total": ep.rows if ep else None}
                if op == "save":
                    return self._json(200, {**res, "step": (ep.rows if ep else 0) - ops.ep_rows0, **counters})
                out = {**out, **counters}
            return self._rcb1(out, blobs)

        def _state_query(self):
            """POST /state/query (RCB_STATE_QUERY=1): reads the simulator between calls; no control step, no clock."""
            try:
                body, _ = self._body()
            except Exception as exc:  # noqa: BLE001
                return self._json(413 if "too large" in str(exc) else 400, {"error": str(exc)[:200]})
            E, ep = svc.state, svc.episode
            with svc.lock:
                if E.get("exiting") or E.get("finalized") or E.get("done"):
                    return self._json(409, {"error": "the episode has ended: state queries are served only while it runs"})
                plan, err = plan_state_query(svc.state_ents, body)
                if err:
                    return self._json(400, err)
                log = ep.log if ep else svc.log
                try:
                    state = svc.state_table.query(plan)
                    json.dumps(state, allow_nan=False)
                except Exception as exc:  # noqa: BLE001 -- the query only reads: nothing ends
                    traceback.print_exc(); log({"ev": "state_query_error", "error": repr(exc)[:300]})
                    return self._json(500, {"error": "state query failed", "detail": repr(exc)[:200]})
                if svc.state_queries is None:
                    svc.state_queries = ep.state_queries() if ep else 0
                svc.state_queries += 1
                steps = (ep.rows - (svc.ops.ep_rows0 if svc.ops is not None else 0)) if ep else None
                log({"ev": "state_query", "n": svc.state_queries, "names": list(plan),
                     "fields": sorted({f for fs in plan.values() for f in fs}), "control_steps": steps})
                hz = float(svc.spec.get("control_hz") or 0)
                return self._json(200, {"state": state, "control_steps": steps,
                                        "sim_time_s": round(steps / hz, 4) if steps is not None and hz else None,
                                        "state_queries": svc.state_queries})

        def do_POST(self):
            if STATE_QUERY and self.path == "/state/query":
                return self._state_query()
            if svc.ops is not None and self.path in ("/reset", "/save_state", "/load_state"):
                return self._state_op()
            if self.path not in ("/submit", "/observe", "/kinematics"):
                return self._json(404, {"error": "no such endpoint"})
            try:
                meta, parts = self._body()
            except Exception as exc:  # noqa: BLE001
                return self._json(413 if "too large" in str(exc) else 400, {"error": str(exc)[:200]})
            E, ep = svc.state, svc.episode
            if self.path == "/kinematics":          # stateless: never steps physics, never waits for the episode lock
                cid = ep.call_begin(self.path) if ep else 0
                try:
                    try:
                        req = parse_kinematics(meta, svc.spec["kinematics"])
                    except (TypeError, OverflowError) as exc:   # wrong-typed fields (a string for q, a list for max_iters)
                        raise ValueError(f"malformed kinematics request: {exc}") from None
                    out = svc.kinematics(req); svc.kin_calls += 1
                except ValueError as exc:
                    return self._json(400, {"error": str(exc)[:300]})
                except Exception as exc:  # noqa: BLE001
                    traceback.print_exc(); return self._json(500, {"error": "kinematics error", "detail": repr(exc)[:200]})
                finally:
                    if ep:
                        ep.call_end(cid, self.path)
                return self._json(200, out)
            with svc.lock:
                if E.get("exiting") or (self.path == "/submit" and E.get("done")):
                    return self._json(409, {"error": "episode finished", "reason": E.get("reason")})
                cid = ep.call_begin(self.path, self.connection) if ep else 0
                rows0 = ep.rows if ep else 0
                out = blobs = err = None; stopped = None
                try:
                    A = parse_request(self.path, meta, parts)   # ValueError -> 400, before the adapter runs
                    if self.path == "/submit":
                        out, blobs = svc.submit(meta, A)
                    else:
                        out, blobs = svc.observe(meta)
                except ValueError as exc:
                    err = (400, {"error": str(exc)})
                except Exception as exc:  # noqa: BLE001
                    traceback.print_exc(); E["error"] = repr(exc); svc.log({"ev": "error", "path": self.path, "error": repr(exc)})
                    try:
                        svc.on_submit_error(exc)    # PROTOCOL §4.6: a simulator error ends the episode (reason "error")
                    except Exception:  # noqa: BLE001
                        traceback.print_exc()
                    err = (500, {"error": "simulator error", "detail": repr(exc)[:200]})
                finally:
                    if ep:
                        stopped = ep.stop_reason
                        if err is None or ep.rows != rows0:
                            try:
                                ep.checkpoint(E, svc.state_sha, svc.proprio)
                            except Exception:  # noqa: BLE001
                                traceback.print_exc()
                        ep.call_end(cid, self.path, **({"stopped": stopped} if stopped else {}))
                if err is None and out is not None:
                    if svc.ops is not None:
                        out = {**out, **svc.ops.counters(E), "control_steps_total": ep.rows if ep else None}
            if err is not None:
                return self._json(*err)
            return self._rcb1({**out, "stopped": stopped}, blobs)
    return H


def serve(svc: Service, port: int = 8000) -> None:
    """Check the spec, then serve forever (one request at a time, as every 2.x service)."""
    svc.spec.setdefault("on_service_error", ON_SERVICE_ERROR)
    if svc.episode is not None:
        svc.spec.setdefault("max_call_wall_s", svc.episode.max_call_wall_s)
    check_spec(svc.spec)
    bind(port, make_handler(svc)).serve_forever()


def bind(port: int, handler) -> HTTPServer:
    """The listening server. Under Harbor's egress control the agent's container shares this network namespace, so a
    process of the agent (`python -m http.server` listens on 8000 by default) may hold the port while the scene was
    built. Wait for the port instead of failing: an exception here would count as a crash of the service, and three of
    them end the episode. The time spent waiting is the agent's own (the collect hook's signal is still served)."""
    said = False
    while True:
        try:
            return HTTPServer(("0.0.0.0", port), handler)
        except OSError as e:
            if e.errno != errno.EADDRINUSE:
                raise
        if not said:
            said = True
            print(f"[sim] port {port} is in use (a process in the agent's container?): waiting for it to be free", flush=True)
        time.sleep(1.0)


# ---------------------------------------------------------------- the collect hook's first half
def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    try:                                     # a zombie (exited, not yet reaped) counts as gone
        with open(f"/proc/{pid}/stat") as f:
            return f.read().rsplit(")", 1)[-1].split()[0] != "Z"
    except OSError:
        return True


def end_episode(data: str, timeout: float = 21600.0, wait_key: str | None = None, resend_s: float = 30.0) -> int:
    """Tell the service that the agent phase is over and wait until it has ended the episode. The service may be
    resuming after a crash (no PID file until it is ready again), so there is no fixed start-up wait, and a signal goes
    only to the process that wrote the newest line of startups.jsonl (its SIGUSR1 handler is installed by then).
    Returns 2: final.json already holds the verdict (the service gave up after crashes, or graded in-process; with
    `wait_key`, also waits for that final.json key, e.g. video_done): keep it, run no replays.
    Returns 0: the service dumped the episode (episode.json) and exited: the hook merges its live verdict next.
    Returns 1: timed out (with a verdict on disk still waiting for `wait_key`, e.g. a slow video: 2, the verdict stands)."""
    t0, sent = time.time(), 0.0
    f_pid, f_dump, f_starts = (os.path.join(data, n) for n in ("server.pid", "episode.json", "startups.jsonl"))
    while time.time() - t0 < timeout:
        fin = final_verdict(data)
        if fin is not None:
            if not wait_key or fin.get(wait_key) or fin.get("gave_up"):
                return 2
            if time.time() - t0 + 2 >= timeout:
                return 2                     # the verdict is on disk: a video that is not ready never voids it
            time.sleep(2); continue
        try:
            with open(f_pid) as f:
                pid = int(f.read().strip())
        except (OSError, ValueError):
            pid = None
        try:
            with open(f_starts) as f:
                cur = json.loads([x for x in f if x.strip()][-1]).get("pid")
        except (OSError, ValueError, IndexError):
            cur = None                       # a service without Episode.startup(): signal what the PID file names
        if pid is not None and (cur is None or pid == cur):
            if not _alive(pid):
                if os.path.exists(f_dump):
                    return 0
            elif time.time() - sent >= resend_s:
                try:
                    os.kill(pid, signal.SIGUSR1); sent = time.time()
                except OSError:
                    pass
        time.sleep(2)
    return 2 if final_verdict(data) is not None else 1   # a verdict that landed during the last wait stands


if __name__ == "__main__":
    a = sys.argv[1:]
    if len(a) >= 2 and a[0] == "end":
        wk = a[a.index("--wait-key") + 1] if "--wait-key" in a else None
        to = float(a[a.index("--timeout") + 1]) if "--timeout" in a else 21600.0
        rc = end_episode(a[1], timeout=to, wait_key=wk)
        print({0: "[collect] episode dumped by the service", 1: "[collect] timed out waiting for the service",
               2: "[collect] final.json already holds the verdict"}[rc], flush=True)
        sys.exit(rc)
    if len(a) == 2 and a[0] == "final-verdict":
        fin = final_verdict(a[1])
        print(json.dumps(fin) if fin else "no final verdict")
        sys.exit(0 if fin else 1)
    print("usage: rcb_limited.py end <data dir> [--wait-key KEY] [--timeout S] | final-verdict <data dir>")
    sys.exit(2)
