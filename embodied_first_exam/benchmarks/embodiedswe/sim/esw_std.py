"""EmbodiedSWE standard mode: the robot side of the `sim` service (server.py). One class per kind of robot in the 24
tasks, all built on EmbodiedSWE's own env (rcb_embodiedswe.build: the preset, the seed, GPU PhysX):

- FrankaRig: one or two Franka Pandas under EmbodiedSWE's operational-space control (osc). An `ee_pose` row per arm is
  the pose target of the OSC's own control body (panda_hand) in the world frame plus the two finger position targets.
  Each control step turns the target in force into EmbodiedSWE's native OSC action (6 pose deltas in action units +
  2 fingers) so that the target the controller latches IS the row's pose: the delta is (target - measured pose) /
  pos_scale (rot_scale), and the controller's own action smoothing (an EMA) is inverted with its own buffer. The torque
  law, its gains and its 100 N m clamp are EmbodiedSWE's, unchanged.
- G1Rig: the Unitree G1 with Dex3 hands under Pink IK (pink_ik: pelvis fixed; loco_pink_ik: walking, a frozen
  locomotion policy). The `ee_pose` row is EmbodiedSWE's native action: the two wrist poses (world frame), the 14 hand
  joint targets, and in loco mode the base command [vx, vy, wz, hip_height].

Each rig gives the service: the action layout and its interpolation, the native action for one control step, the
proprioception row (measured joints, velocities, efforts, tool poses, gripper openings), robot_spec, and a stateless
kinematic model (/kinematics: pinocchio on the kinematics URDF the robot's own Pink IK uses) whose frames are checked
against the simulator's link poses at start-up. Cameras (Cameras): the views EmbodiedSWE itself declares (the scene's
CAMERAS, its room's overview camera, the Franka's wrist camera) and, for the G1, which declares none, the real robot's
head camera mount; created right before the env's one sim.reset() so the frozen start state is unchanged."""
from __future__ import annotations

import math
import os

import numpy as np

W, H = 640, 480                         # every camera's resolution (EmbodiedSWE's replay / VLA default)
G1_HEAD = {"link": "torso_link", "xyz": (0.0576235, 0.01753, 0.42987), "rpy": (0.0, 0.8307767, 0.0), "focal": 16.0}
FRANKA_TCP_Z = 0.1034                   # panda_hand -> the centre between the fingertips, along the approach axis (m)
EE_BOX_M = 3.0                          # an ee_pose position further than this from the robot's base on any axis is a 400
BENCH = os.environ.get("ESW_BENCH_ROOT", "/opt/embodiedswe")


# ---------------------------------------------------------------- small math (wxyz quaternions)
def qnorm(q):
    q = np.asarray(q, dtype=np.float64)
    return q / max(1e-12, float(np.linalg.norm(q)))


def qmul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def qconj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def q2R(q):
    w, x, y, z = qnorm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def R2q(R):
    R = np.asarray(R, dtype=np.float64); t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2; q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2; q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2; q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2; q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = qnorm(q)
    return q if q[0] >= 0 else -q


def T_of(p, q):
    T = np.eye(4); T[:3, :3] = q2R(q); T[:3, 3] = p
    return T


def pose_of(T):
    return list(np.asarray(T[:3, 3], dtype=np.float64)) + list(R2q(T[:3, :3]))


def axis_angle(q):
    """The rotation vector (axis * angle, angle in [0, pi]) of a wxyz quaternion."""
    q = qnorm(q)
    if q[0] < 0:
        q = -q
    s = float(np.linalg.norm(q[1:])); ang = 2.0 * math.atan2(s, q[0])
    return np.zeros(3) if s < 1e-12 else q[1:] / s * ang


def slerp(a, b, t):
    a, b = qnorm(a), qnorm(b); d = float(np.dot(a, b))
    if d < 0:
        b, d = -b, -d
    if d > 0.9995:
        return qnorm(a + t * (b - a))
    th = math.acos(d)
    return (math.sin((1 - t) * th) * a + math.sin(t * th) * b) / math.sin(th)


def rpy2q(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2), math.cos(y / 2), math.sin(y / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def lookat_q(eye, target):
    """EmbodiedSWE's own look-at (data_engine/engine/replay.py _lookat_quat): +x forward, +z up, wxyz."""
    f = np.asarray(target, dtype=np.float64) - np.asarray(eye, dtype=np.float64); f = f / max(1e-9, float(np.linalg.norm(f)))
    up = np.array([0.0, 0.0, 1.0]); left = np.cross(up, f)
    if np.linalg.norm(left) < 1e-6:
        left = np.array([0.0, 1.0, 0.0])
    left = left / np.linalg.norm(left); u = np.cross(f, left)
    return R2q(np.stack([f, left, u], axis=1))


def t2np(x):
    return x.detach().to("cpu").double().numpy()


# ---------------------------------------------------------------- pinocchio models
class Chain:
    """A stateless kinematic model (pinocchio) of one robot's kinematics URDF. `frames` maps exposed names to URDF frame
    names (+ an optional fixed offset), all relative to `base_frame`; `joints` are the URDF joints the request's q sets
    (others stay 0)."""

    def __init__(self, urdf: str, base_frame: str, frames: dict, joints: list, ik_joints: list):
        import pinocchio as pin
        self.pin = pin
        self.m = pin.buildModelFromUrdf(urdf); self.d = self.m.createData()
        self.base = self.m.getFrameId(base_frame)
        self.frames = {k: (self.m.getFrameId(v[0]), v[1]) for k, v in frames.items()}
        for k, (fid, _) in self.frames.items():
            if fid >= len(self.m.frames):
                raise RuntimeError(f"kinematics: frame {frames[k][0]} not in {urdf}")
        self.joints = list(joints)
        miss = [j for j in joints if not self.m.existJointName(j)]
        if miss:
            raise RuntimeError(f"kinematics: joints {miss} not in {urdf}")
        self.qidx = [self.m.idx_qs[self.m.getJointId(j)] for j in joints]
        self.vidx = [self.m.idx_vs[self.m.getJointId(j)] for j in joints]
        self.ik_cols = [self.joints.index(j) for j in ik_joints]
        self.lo = np.array([self.m.lowerPositionLimit[i] for i in self.qidx]); self.hi = np.array([self.m.upperPositionLimit[i] for i in self.qidx])

    def _q(self, q):
        full = self.pin.neutral(self.m)
        for i, v in zip(self.qidx, q):
            full[i] = v
        return full

    def fk(self, q, names=None) -> dict:
        pin = self.pin
        pin.framesForwardKinematics(self.m, self.d, self._q(q))
        Tb_inv = np.linalg.inv(self.d.oMf[self.base].homogeneous)
        out = {}
        for k in (names or self.frames):
            fid, off = self.frames[k]
            T = Tb_inv @ self.d.oMf[fid].homogeneous
            if off is not None:
                T = T @ off
            out[k] = T
        return out

    def ik(self, targets: dict, seed, tol_p=1e-4, tol_r=1e-3, iters=300):
        """Damped least squares over the ik joints; targets {name: 4x4 in the base frame}. -> (q, {name: (pos_err, rot_err)})."""
        pin = self.pin
        q = np.array(seed, dtype=np.float64)
        cols = self.ik_cols
        for _ in range(int(iters)):
            full = self._q(q)
            pin.computeJointJacobians(self.m, self.d, full); pin.framesForwardKinematics(self.m, self.d, full)
            Tb = self.d.oMf[self.base].homogeneous; Rb = Tb[:3, :3]
            errs, Js, rep = [], [], {}
            for k, Tt in targets.items():
                fid, off = self.frames[k]
                Tf = np.linalg.inv(Tb) @ self.d.oMf[fid].homogeneous
                Tc = Tf @ off if off is not None else Tf
                ep = Tt[:3, 3] - Tc[:3, 3]
                er = axis_angle(R2q(Tt[:3, :3] @ Tc[:3, :3].T))
                rep[k] = (float(np.linalg.norm(ep)), float(np.linalg.norm(er)))
                J = pin.getFrameJacobian(self.m, self.d, fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)   # world-aligned at the frame
                if off is not None:                       # the offset point's linear velocity: v + w x r
                    r = Tf[:3, :3] @ off[:3, 3]
                    rw = Rb @ r
                    J = J.copy(); J[:3] = J[:3] - np.cross(rw, J[3:].T).T
                Jb = np.vstack([Rb.T @ J[:3], Rb.T @ J[3:]])                        # into the base frame
                Js.append(Jb[:, [self.vidx[c] for c in cols]]); errs.append(np.concatenate([ep, er]))
            if all(p < tol_p and r < tol_r for p, r in rep.values()):
                return q, rep, True
            J = np.vstack(Js); e = np.concatenate(errs); lam = 1e-4
            dq = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(J.shape[0]), e)
            n = float(np.linalg.norm(dq))
            if n > 0.2:
                dq *= 0.2 / n
            for i, c in enumerate(cols):
                q[c] = float(np.clip(q[c] + dq[i], self.lo[c], self.hi[c]))
        return q, rep, all(p < tol_p * 30 and r < tol_r * 30 for p, r in rep.values())


# ---------------------------------------------------------------- cameras
GL2CV = np.diag([1.0, -1.0, -1.0, 1.0])     # OpenGL camera axes (x right, y up, z back) <-> OpenCV (x right, y down, z forward)


class Cameras:
    """The service's cameras: Isaac Lab `Camera` sensors (rgb + z-depth, 640x480), made right before the env's one
    sim.reset(). Static views from EmbodiedSWE's declarations (the scene's CAMERAS at their nominal pose, on the work
    surface; the room's overview camera), aimed once the env exists; robot-mounted views are children of their link."""

    def __init__(self, views: list):
        self.views = views
        self.cams: dict = {}

    def make(self):
        import isaaclab.sim as sim_utils
        from isaaclab.sensors import Camera, CameraCfg
        for v in self.views:
            kw = dict(update_period=0.0, update_latest_camera_pose=True, height=H, width=W, data_types=["rgb", "distance_to_image_plane"],
                      spawn=sim_utils.PinholeCameraCfg(focal_length=v["focal"], clipping_range=(0.01, 100.0)))
            if v.get("link_prim"):
                cfg = CameraCfg(prim_path=f"{v['link_prim']}/esw_{v['name']}",
                                offset=CameraCfg.OffsetCfg(pos=tuple(v["pos"]), rot=tuple(v["quat"]), convention="world"), **kw)
            else:
                cfg = CameraCfg(prim_path=f"/World/esw_{v['name']}", **kw)
            self.cams[v["name"]] = Camera(cfg)
        return self

    def aim(self, env):
        import torch
        o = t2np(env.iscene.env_origins[0])
        if float(np.max(np.abs(o))) > 1e-9:
            raise RuntimeError(f"env origin {o.tolist()} is not the world origin: the service's frames assume it is")
        for v in self.views:
            if not v.get("link_prim"):
                self.cams[v["name"]].set_world_poses_from_view(torch.tensor([list(v["eye"])], device=env.device, dtype=torch.float32),
                                                               torch.tensor([list(v["target"])], device=env.device, dtype=torch.float32))


def _find_prim(stage, root: str, name: str) -> str:
    todo = [stage.GetPrimAtPath(root)]
    while todo:
        p = todo.pop(0)
        if not p.IsValid():
            continue
        if p.GetName() == name:
            return str(p.GetPath())
        todo += list(p.GetChildren())
    raise RuntimeError(f"camera mount: no prim {name} under {root}")


def views_before_reset(scene, robot, room, stage) -> list:
    """EmbodiedSWE's declared views of this preset, read from the env under construction: the scene's CAMERAS
    (nominal pose, relative to the work surface; their randomisation bands are not used), the room's overview camera,
    then the robot's own (the Franka wrist camera robobench declares; the G1, which declares none, gets the real
    robot's head camera mount)."""
    views = []
    sz = float(getattr(getattr(scene, "cfg", None), "surface_z", 0.0) or 0.0)
    decl = getattr(type(scene), "CAMERAS", {}) or {}
    # robobench declares these views on the work surface (its replay adds surface_z), but wheel_carry's are already
    # world heights (their targets sit on its 0.694 m table tops): such declarations are used as they are
    if decl and sz > 0 and min(float(v["target"][2]) for v in decl.values()) >= 0.5 * sz:
        sz = 0.0
    for n, spec in decl.items():
        e, t = spec["eye"], spec["target"]
        views.append({"name": n, "eye": (e[0], e[1], e[2] + sz), "target": (t[0], t[1], t[2] + sz), "focal": float(spec.get("focal", 16.0)),
                      "what": "static: EmbodiedSWE's own view of this scene"})
    if isinstance(room, dict) and room.get("camera"):
        views.append({"name": "overview", "eye": tuple(room["camera"][0]), "target": tuple(room["camera"][1]), "focal": 16.0,
                      "what": "static: the room's overview of the workspace"})
    kind = type(robot).__name__
    if kind == "G1Robot":
        h = G1_HEAD
        views.append({"name": "head", "link_prim": _find_prim(stage, f"/World/envs/env_0/{robot.prim_name}", h["link"]), "link": h["link"],
                      "arm": "", "pos": h["xyz"], "quat": tuple(rpy2q(*h["rpy"])), "focal": h["focal"],
                      "what": "on the head (the G1's head camera mount on torso_link), looking forward and down; moves with the torso"})
    else:
        kids = list((getattr(robot, "robots", None) or {}).items()) or [("", robot)]
        for name, k in kids:
            spec = type(k).CAMERAS["wrist"]; pfx = f"{name}_" if name else ""
            views.append({"name": f"{pfx}wrist", "link_prim": _find_prim(stage, f"/World/envs/env_0/{k.prim_name}", spec["link"]),
                          "link": spec["link"], "arm": pfx, "pos": tuple(spec["eye"]), "quat": tuple(lookat_q(spec["eye"], spec["target"])),
                          "focal": float(spec["focal"]),
                          "what": f"on the {name + ' ' if name else ''}gripper (EmbodiedSWE's Franka wrist camera); moves with the hand"})
    return views


def calibrate(cams: Cameras, rig) -> dict:
    """Per camera: intrinsics, and its mount: a static camera's extrinsics, a mounted one's parent link and the fixed
    link_T_cam_cv, measured from the rendered sensor's pose and the link's pose (the cameras were just rendered)."""
    out = {}
    for v in cams.views:
        c = cams.cams[v["name"]]; d = c.data
        K = t2np(d.intrinsic_matrices[0])
        T = T_of(t2np(d.pos_w[0]), t2np(d.quat_w_ros[0]))          # camera -> world, OpenCV axes
        m = {"K": K}
        if v.get("link_prim"):
            body = rig.body_ref(v)
            m["parent_body"] = body
            m["link_T_cv"] = np.linalg.inv(rig.link_T(body)) @ T
            m["mount"] = {"type": "link", "parent_link": v["arm"] + v["link"], "link_T_cam_cv": m["link_T_cv"].tolist()}
        else:
            m["world_T_cv"] = T
            m["mount"] = {"type": "static", "extrinsic_cv": np.linalg.inv(T)[:3].tolist(), "cam2world_gl": (T @ GL2CV).tolist()}
        out[v["name"]] = m
    return out


# ---------------------------------------------------------------- Franka (one or two arms, osc)
class _Arm:
    def __init__(self, env, robot, prefix: str):
        self.env, self.robot, self.p = env, robot, prefix
        self.art = robot.articulation
        subs = robot.controller.controllers
        self.osc, self.grip = subs[0], subs[1]
        if type(self.osc).__name__ != "OperationalSpaceController":
            raise RuntimeError(f"expected EmbodiedSWE's osc controller, got {type(self.osc).__name__}")
        if getattr(self.osc.cfg, "unidirectional_rot", False):
            raise RuntimeError("unidirectional_rot is not supported by the ee_pose conversion")
        self.arm_ids = list(self.osc.joint_ids); self.finger_ids = list(self.grip.joint_ids)
        self.ee = int(self.osc._ee_idx)
        if self.art.body_names[self.ee] != "panda_hand":
            raise RuntimeError(f"the OSC drives {self.art.body_names[self.ee]}, not panda_hand")
        names = self.art.joint_names
        self.arm_names = [names[i] for i in self.arm_ids]; self.finger_names = [names[i] for i in self.finger_ids]
        self.base_T = T_of(t2np(self.art.data.root_pos_w[0]), t2np(self.art.data.root_quat_w[0]))

    def hand_pose(self):
        d = self.art.data
        return t2np(d.body_pos_w[0, self.ee]), t2np(d.body_quat_w[0, self.ee])

    def native(self, row):
        """9 values (pose target of panda_hand, world, + 2 finger targets) -> the 8-value native action whose latched
        OSC target is exactly that pose (the controller's EMA inverted with its own buffer)."""
        c = self.osc.cfg
        p, q = self.hand_pose()
        tq = qnorm(row[3:7])
        if float(np.dot(tq, q)) < 0:
            tq = -tq
        eff = np.concatenate([(np.asarray(row[0:3]) - p) / c.pos_scale, axis_angle(qmul(tq, qconj(q))) / c.rot_scale])
        if self.osc._prev_action is not None and c.ema_factor < 1.0:
            prev = t2np(self.osc._prev_action[0])
            eff = (eff - (1.0 - c.ema_factor) * prev) / c.ema_factor
        return np.concatenate([eff, np.asarray(row[7:9], dtype=np.float64)])

    def hold(self):
        p, q = self.hand_pose()
        ft = t2np(self.art.data.joint_pos_target[0, self.finger_ids])
        return np.concatenate([p, q, ft])


class FrankaRig:
    kind = "franka"

    def __init__(self, env):
        self.env = env
        r = env.robot
        kids = list((getattr(r, "robots", None) or {}).items())
        self.arms = [_Arm(env, k, f"{n}_") for n, k in kids] if kids else [_Arm(env, r, "")]
        self.ctrl_hz = 1.0 / (env.dt * r.control_period); self.substeps = int(r.control_period)
        self.native_dim = int(r.action_dim)
        assert self.native_dim == 8 * len(self.arms), (self.native_dim, len(self.arms))
        L = []
        for a in self.arms:
            L += [a.p + k for k in ("x", "y", "z", "qw", "qx", "qy", "qz")] + [a.p + "finger_1", a.p + "finger_2"]
        self.layout = L; self.dim = len(L)
        self.quat_slices = [(9 * i + 3, 9 * i + 7) for i in range(len(self.arms))]
        self.pos_slices = [(9 * i, 9 * i + 3) for i in range(len(self.arms))]
        self.step_cols: list = []                    # columns switched at the row (none here)
        urdf = os.path.join(BENCH, "robobench/robots/assets/franka/panda_kinematics.urdf")
        off = np.eye(4); off[2, 3] = FRANKA_TCP_Z
        frames = {f"panda_link{i}": (f"panda_link{i}", None) for i in range(8)}
        frames.update({"panda_hand": ("panda_hand", None), "tcp": ("panda_hand", off)})
        J = [f"panda_joint{i}" for i in range(1, 8)]
        self.chain = Chain(urdf, "panda_link0", frames, J, J)
        self.links = [a.p + f for a in self.arms for f in frames]
        self.tcp_frames = [a.p + "tcp" for a in self.arms]
        self.ee_frames = [a.p + "panda_hand" for a in self.arms]

    # actions
    def native(self, row):
        return np.concatenate([a.native(row[9 * i: 9 * i + 9]) for i, a in enumerate(self.arms)])

    def hold(self):
        return np.concatenate([a.hold() for a in self.arms])

    def check_rows(self, A):
        for ri, row in enumerate(A):
            for i, a in enumerate(self.arms):
                p = row[9 * i: 9 * i + 3]
                if float(np.max(np.abs(p - a.base_T[:3, 3]))) > EE_BOX_M:
                    raise ValueError(f"ee_pose row {ri}: {a.p or 'the '}panda_hand position {[round(float(v), 4) for v in p]} is more than "
                                     f"{EE_BOX_M} m from the arm's base on an axis (metres, world frame)")
                if float(np.linalg.norm(row[9 * i + 3: 9 * i + 7])) < 1e-6:
                    raise ValueError(f"ee_pose row {ri}: a quaternion (wxyz) of zero length")

    def body_ref(self, view):
        a = next(x for x in self.arms if x.p == view["arm"])
        return (a, a.art.body_names.index(view["link"]))

    def action_limits(self):
        out = {}
        for i, a in enumerate(self.arms):
            lim = t2np(a.art.data.soft_joint_pos_limits[0, a.finger_ids])
            for j in range(2):
                out[self.layout[9 * i + 7 + j]] = [round(float(lim[j, 0]), 5), round(float(lim[j, 1]), 5)]
        return out

    def controller_text(self):
        c = self.arms[0].osc.cfg
        return (f"EmbodiedSWE's operational-space controller latches that panda_hand pose as its target (its 6-D action = the pose error "
                f"in units of {c.pos_scale} m and {c.rot_scale} rad, its action smoothing inverted) and computes the joint torques toward it "
                f"every physics step (task stiffness {list(c.task_prop_gains)}, a nullspace pull towards the home posture, each torque "
                f"clamped to {c.torque_limit} N m); the two finger targets go to the fingers' position drives. A target far from the hand "
                "is pulled towards at the torque clamp: stream intermediate rows for a controlled motion")

    def after_replay(self, row):
        pass

    def link_T(self, body):
        a, idx = body
        d = a.art.data
        return T_of(t2np(d.body_pos_w[0, idx]), t2np(d.body_quat_w[0, idx]))

    # proprioception
    def proprio_spec(self):
        lay, g = [], {}
        def grp(name, cols, what):
            g[name] = [len(lay), len(cols), what]; lay.extend(cols)
        grp("joint_pos", [a.p + n for a in self.arms for n in a.arm_names + a.finger_names],
            "measured joint positions: panda_joint1..7 (rad), panda_finger_joint1..2 (m)" + (" (left arm, then right)" if len(self.arms) > 1 else ""))
        grp("tcp_pose", [f"{a.p}tcp_{k}" for a in self.arms for k in ("x", "y", "z", "qw", "qx", "qy", "qz")],
            f"measured fingertip-centre pose (panda_hand moved {FRANKA_TCP_Z} m along its approach axis), world frame, wxyz")
        grp("ee_pose", [f"{a.p}hand_{k}" for a in self.arms for k in ("x", "y", "z", "qw", "qx", "qy", "qz")],
            "measured panda_hand pose (the frame ee_pose actions command), world frame, wxyz")
        grp("gripper_opening", [f"{a.p}gripper_opening_m" for a in self.arms], "measured distance between the two finger pads (m), about 0.08 fully open")
        grp("finger_target", [a.p + n + "_target" for a in self.arms for n in a.finger_names], "the finger position targets in force (m)")
        grp("joint_vel", [a.p + n + "_vel" for a in self.arms for n in a.arm_names + a.finger_names], "measured joint velocities (rad/s, m/s)")
        grp("joint_effort", [a.p + n + "_effort" for a in self.arms for n in a.arm_names],
            "the joint torques the operational-space controller applies to panda_joint1..7 (N m, after its clamp)")
        return lay, g

    def proprio(self):
        out = {k: [] for k in ("jp", "tcp", "ee", "op", "ft", "jv", "ef")}
        for a in self.arms:
            d = a.art.data
            ids = a.arm_ids + a.finger_ids
            out["jp"] += list(t2np(d.joint_pos[0, ids])); out["jv"] += list(t2np(d.joint_vel[0, ids]))
            p, q = a.hand_pose(); T = T_of(p, q); Tt = T.copy(); Tt[:3, 3] = T[:3, 3] + T[:3, 2] * FRANKA_TCP_Z
            out["tcp"] += pose_of(Tt); out["ee"] += list(p) + list(qnorm(q) if q[0] >= 0 else -qnorm(q))
            fp = t2np(d.joint_pos[0, a.finger_ids]); out["op"].append(float(np.sum(fp)))
            out["ft"] += list(t2np(d.joint_pos_target[0, a.finger_ids])); out["ef"] += list(t2np(d.applied_torque[0, a.arm_ids]))
        return np.array(out["jp"] + out["tcp"] + out["ee"] + out["op"] + out["ft"] + out["jv"] + out["ef"], dtype=np.float32)

    # robot spec + kinematics
    def robot_spec(self):
        joints, gains = [], {}
        for a in self.arms:
            d = a.art.data
            lim = t2np(d.soft_joint_pos_limits[0]); vl = t2np(d.joint_vel_limits[0]); el = t2np(d.joint_effort_limits[0])
            st = t2np(d.joint_stiffness[0]); dm = t2np(d.joint_damping[0])
            for i in a.arm_ids + a.finger_ids:
                n = a.art.joint_names[i]; fing = i in a.finger_ids
                joints.append({"name": a.p + n, "type": "prismatic" if fing else "revolute", "unit": "m" if fing else "rad",
                               "position_limits": [round(float(lim[i, 0]), 5), round(float(lim[i, 1]), 5)],
                               "velocity_limit": float(vl[i]) if np.isfinite(vl[i]) and vl[i] > 0 else None,
                               "effort_limit": float(el[i]) if np.isfinite(el[i]) and el[i] > 0 else None,
                               "group": (a.p + "fingers") if fing else (a.p + "arm")})
                gains[a.p + n] = {"stiffness": float(st[i]), "damping": float(dm[i])}
        c = self.arms[0].osc.cfg
        return {"name": "franka-panda" + (" x2" if len(self.arms) > 1 else ""),
                "description": ("two " if len(self.arms) > 1 else "") + "Franka Emika Panda arm(s) with the parallel-jaw Franka Hand, fixed bases",
                "base": {"type": "fixed", "pose": pose_of(self.arms[0].base_T),
                         **({"poses": {a.p.rstrip("_"): pose_of(a.base_T) for a in self.arms}} if len(self.arms) > 1 else {})},
                "joints": joints,
                "grippers": {a.p + "hand": {"joints": [a.p + n for n in a.finger_names], "opening_m": [0.0, 0.08],
                                            "command": "the two finger position targets of each ee_pose row (m each, 0 closed, 0.04 open); a PD drive "
                                                       "tracks them, so a closed command on an object squeezes it", "tcp_frame": a.p + "tcp"}
                             for a in self.arms},
                "controller": {"type": "EmbodiedSWE operational-space control (osc): joint torques from the pose error of panda_hand, "
                                       "shaped by the operational-space inertia, plus a nullspace posture term, clamped per joint",
                               "control_hz": self.ctrl_hz, "physics_hz": 1.0 / self.env.dt, "gains": gains,
                               "task_stiffness": list(c.task_prop_gains), "torque_limit_nm": float(c.torque_limit),
                               "note": "the arm joints have no position drive (stiffness 0): the OSC torque moves them; the fingers are PD position drives"},
                "tcp": {a.p + "tcp": {"parent_link": a.p + "panda_hand", "link_T_frame": T_of([0, 0, FRANKA_TCP_Z], [1, 0, 0, 0]).tolist(),
                                      "definition": f"the centre between the fingertips: panda_hand moved {FRANKA_TCP_Z} m along its z (approach) axis; "
                                                    "ee_pose rows command panda_hand itself"} for a in self.arms}}

    def kin_joint_names(self):
        return [a.p + n for a in self.arms for n in a.arm_names]

    def kinematics_spec(self):
        return {"endpoint": "/kinematics", "joint_names": self.kin_joint_names(), "frame": "world", "quaternion_order": "wxyz",
                "links": self.links, "tcp_frames": self.tcp_frames, "ee_frames": self.ee_frames, "ik_targets": self.ee_frames + self.tcp_frames,
                "ik": {"solver": "damped least squares (pinocchio, the arm's kinematics URDF)", "tol_pos_m": 1e-4, "tol_rot_rad": 1e-3, "max_iters": 300},
                "stateless": True, "note": "the arm model only (no collisions, no fingers); world frame: the arm's fixed base pose is applied"}

    def kinematics(self, req):
        out_frame = "world"
        if req["op"] == "fk":
            q = np.asarray(req["q"]); names = req.get("links") or self.links; poses = {}
            for i, a in enumerate(self.arms):
                Ts = self.chain.fk(q[7 * i: 7 * i + 7])
                for n in names:
                    if n.startswith(a.p) and n[len(a.p):] in Ts:
                        poses[n] = pose_of(a.base_T @ Ts[n[len(a.p):]])
            return {"ok": True, "op": "fk", "frame": out_frame, "poses": poses}
        if req.get("seed") is not None:
            seed = np.asarray(req["seed"], dtype=np.float64)
        else:
            seed = np.concatenate([t2np(a.art.data.default_joint_pos[0, a.arm_ids]) for a in self.arms])
        q = seed.copy(); err = {}; ok = True
        for i, a in enumerate(self.arms):
            tg = {n[len(a.p):]: np.linalg.inv(a.base_T) @ T_of(p[:3], p[3:]) for n, p in req["targets"].items() if n.startswith(a.p)}
            if not tg:
                continue
            qi, rep, oki = self.chain.ik(tg, seed[7 * i: 7 * i + 7], tol_p=req.get("tol_pos_m") or 1e-4, tol_r=req.get("tol_rot_rad") or 1e-3,
                                         iters=req.get("max_iters") or 300)
            q[7 * i: 7 * i + 7] = qi; ok = ok and oki
            err.update({a.p + k: {"pos_m": round(v[0], 6), "rot_rad": round(v[1], 6)} for k, v in rep.items()})
        return {"ok": bool(ok), "op": "ik", "frame": out_frame, "q": [float(v) for v in q], "error": err}

    def kin_check(self):
        """FK of the measured joints against the simulator's link poses (start-up self-check)."""
        dev = 0.0
        for a in self.arms:
            q = t2np(a.art.data.joint_pos[0, a.arm_ids])
            T = a.base_T @ self.chain.fk(q, ["panda_hand"])["panda_hand"]
            p, _ = a.hand_pose(); dev = max(dev, float(np.linalg.norm(T[:3, 3] - p)))
        return dev

    def describe_action(self):
        per = "x y z qw qx qy qz finger_1 finger_2"
        return (f"{self.dim} values: " + (f"left arm [{per}], then right arm [{per}]" if len(self.arms) > 1 else per) +
                ": the pose target of panda_hand (world frame, metres, wxyz) and the two finger position targets (m, 0 closed, 0.04 open)")


# ---------------------------------------------------------------- G1 (pink_ik / loco_pink_ik)
G1_PFX = "g1_29dof_with_hand_rev_1_0_"


class G1Rig:
    kind = "g1"

    def __init__(self, env):
        self.env = env
        r = env.robot; self.robot = r; self.art = r.articulation
        self.loco = r.control_mode.startswith("loco_")
        subs = r.controller.controllers
        self.ik_ctrl, self.hand_ctrl = subs[0], subs[1]
        self.ctrl_hz = 1.0 / (env.dt * r.control_period); self.substeps = int(r.control_period)
        self.native_dim = int(r.action_dim)
        names = self.art.joint_names
        self.hand_ids = list(self.hand_ctrl.joint_ids); self.hand_names = [names[i] for i in self.hand_ids]
        self.upper_ids = list(self.ik_ctrl.joint_ids); self.upper_names = [names[i] for i in self.upper_ids]
        self.wrist = [self.art.body_names.index("left_wrist_yaw_link"), self.art.body_names.index("right_wrist_yaw_link")]
        self.pelvis = self.art.body_names.index("pelvis")
        L = [f"{s}_wrist_{k}" for s in ("left", "right") for k in ("x", "y", "z", "qw", "qx", "qy", "qz")] + list(self.hand_names)
        if self.loco:
            L += ["base_vx", "base_vy", "base_wz", "base_hip_height"]
        self.layout = L; self.dim = len(L)
        assert self.dim == self.native_dim, (self.dim, self.native_dim)
        self.quat_slices = [(3, 7), (10, 14)]; self.pos_slices = [(0, 3), (7, 10)]
        self.step_cols = list(range(28, 32)) if self.loco else []
        self.last_cmd = np.array([0.0, 0.0, 0.0, 0.72])
        urdf = os.path.join(BENCH, "robobench/robots/assets/g1/g1_29dof_with_hand_only_kinematics.urdf")
        body = [b for b in self.art.body_names if not b.endswith("palm_link")]
        legs = ("hip", "knee", "ankle")
        self.link_names = [b for b in body if not any(k in b for k in legs)]
        frames = {}
        import pinocchio as pin
        m = pin.buildModelFromUrdf(urdf)
        for b in self.link_names:
            for cand in (G1_PFX + b, b):
                if m.existFrame(cand):
                    frames[b] = (cand, None); break
        self.link_names = [b for b in self.link_names if b in frames]
        self.kin_joints = [n for n in names if not any(k in n for k in legs)]
        ik_j = [n for n in self.upper_names]
        self.chain = Chain(urdf, G1_PFX + "pelvis", frames, self.kin_joints, ik_j)
        self.ee_frames = ["left_wrist_yaw_link", "right_wrist_yaw_link"]
        self.base_T0 = self.pelvis_T()

    def pelvis_T(self):
        d = self.art.data
        return T_of(t2np(d.body_pos_w[0, self.pelvis]), t2np(d.body_quat_w[0, self.pelvis]))

    def native(self, row):
        """The row as EmbodiedSWE's native action (a quaternion is renormalised only when it is not unit to float32
        precision: rows of a recorded native trajectory pass bit for bit)."""
        r = np.asarray(row, dtype=np.float64).copy()
        for a, b in self.quat_slices:
            if abs(float(np.linalg.norm(r[a:b])) - 1.0) > 1e-6:
                r[a:b] = qnorm(r[a:b])
        if self.loco:
            self.last_cmd = r[28:32].copy()
        return r

    def hold(self):
        d = self.art.data
        out = []
        for b in self.wrist:
            p, q = t2np(d.body_pos_w[0, b]), qnorm(t2np(d.body_quat_w[0, b]))
            out += list(p) + list(q if q[0] >= 0 else -q)
        out += list(t2np(d.joint_pos_target[0, self.hand_ids]))
        if self.loco:
            out += list(self.last_cmd)
        return np.array(out)

    def check_rows(self, A):
        base = self.pelvis_T()[:3, 3]
        for ri, row in enumerate(A):
            for (a, b), (c, e) in zip(self.pos_slices, self.quat_slices):
                if float(np.max(np.abs(row[a:b] - base))) > EE_BOX_M:
                    raise ValueError(f"ee_pose row {ri}: a wrist position {[round(float(v), 4) for v in row[a:b]]} is more than {EE_BOX_M} m "
                                     "from the pelvis on an axis (metres, world frame)")
                if float(np.linalg.norm(row[c:e])) < 1e-6:
                    raise ValueError(f"ee_pose row {ri}: a quaternion (wxyz) of zero length")

    def body_ref(self, view):
        return self.art.body_names.index(view["link"])

    def action_limits(self):
        lim = t2np(self.art.data.soft_joint_pos_limits[0, self.hand_ids])
        out = {self.layout[14 + j]: [round(float(lim[j, 0]), 5), round(float(lim[j, 1]), 5)] for j in range(14)}
        if self.loco:
            out["base_hip_height"] = [0.5, 0.8]
        return out

    def controller_text(self):
        s = ("EmbodiedSWE's Pink IK takes the two wrist poses as its frame targets (one QP per control step over the 17 waist and arm "
             "joints, whose position drives track its solution) and the 14 hand joint targets go to the hands' position drives")
        if self.loco:
            s += ("; the base command goes to the frozen locomotion policy that walks the legs ([0, 0, 0, 0.72] stands still). The wrist "
                  "targets are world poses: while the robot walks, move them with it")
        return s

    def after_replay(self, row):
        if self.loco:
            self.last_cmd = np.asarray(row[28:32], dtype=np.float64).copy()

    def link_T(self, idx):
        d = self.art.data
        return T_of(t2np(d.body_pos_w[0, idx]), t2np(d.body_quat_w[0, idx]))

    def proprio_spec(self):
        lay, g = [], {}
        def grp(name, cols, what):
            g[name] = [len(lay), len(cols), what]; lay.extend(cols)
        names = self.art.joint_names
        grp("joint_pos", list(names), "measured joint positions (rad) of every joint: legs, waist, arms, then the Dex3 hand joints (articulation order)")
        grp("tcp_pose", [f"{s}_wrist_meas_{k}" for s in ("left", "right") for k in ("x", "y", "z", "qw", "qx", "qy", "qz")],
            "measured wrist poses (left_wrist_yaw_link, right_wrist_yaw_link: the frames ee_pose actions command), world frame, wxyz")
        grp("gripper_opening", ["left_hand_opening_m", "right_hand_opening_m"],
            "per hand: the distance (m) between the thumb's tip link and the midpoint of the index and middle fingers' tip links")
        grp("base_pose", ["pelvis_x", "pelvis_y", "pelvis_z", "pelvis_qw", "pelvis_qx", "pelvis_qy", "pelvis_qz"],
            "measured pelvis pose, world frame, wxyz" + ("" if self.loco else " (fixed: the pelvis is welded to the world)"))
        grp("base_vel", ["pelvis_vx", "pelvis_vy", "pelvis_vz", "pelvis_wx", "pelvis_wy", "pelvis_wz"], "measured pelvis velocity, world frame (m/s, rad/s)")
        grp("hand_target", [n + "_target" for n in self.hand_names], "the hand joint targets in force (rad)")
        if self.loco:
            grp("base_command", ["cmd_vx", "cmd_vy", "cmd_wz", "cmd_hip_height"], "the base command in force")
        grp("joint_vel", [n + "_vel" for n in names], "measured joint velocities (rad/s)")
        grp("joint_effort", [n + "_effort" for n in names], "applied joint torques of the PD drives (N m)")
        return lay, g

    def _opening(self, side):
        b = self.art.body_names; d = self.art.data
        P = lambda n: t2np(d.body_pos_w[0, b.index(n)])  # noqa: E731
        return float(np.linalg.norm(P(f"{side}_hand_thumb_2_link") - 0.5 * (P(f"{side}_hand_index_1_link") + P(f"{side}_hand_middle_1_link"))))

    def proprio(self):
        d = self.art.data
        v = list(t2np(d.joint_pos[0]))
        for b in self.wrist:
            p, q = t2np(d.body_pos_w[0, b]), qnorm(t2np(d.body_quat_w[0, b])); v += list(p) + list(q if q[0] >= 0 else -q)
        v += [self._opening("left"), self._opening("right")]
        v += pose_of(self.pelvis_T())
        v += list(t2np(d.body_lin_vel_w[0, self.pelvis])) + list(t2np(d.body_ang_vel_w[0, self.pelvis]))
        v += list(t2np(d.joint_pos_target[0, self.hand_ids]))
        if self.loco:
            v += list(self.last_cmd)
        v += list(t2np(d.joint_vel[0])) + list(t2np(d.applied_torque[0]))
        return np.array(v, dtype=np.float32)

    def robot_spec(self):
        d = self.art.data
        lim = t2np(d.soft_joint_pos_limits[0]); vl = t2np(d.joint_vel_limits[0]); el = t2np(d.joint_effort_limits[0])
        st = t2np(d.joint_stiffness[0]); dm = t2np(d.joint_damping[0])
        joints, gains = [], {}
        for i, n in enumerate(self.art.joint_names):
            grp = "hand" if i in self.hand_ids else ("arm_waist" if i in self.upper_ids else "leg")
            joints.append({"name": n, "type": "revolute", "unit": "rad", "position_limits": [round(float(lim[i, 0]), 5), round(float(lim[i, 1]), 5)],
                           "velocity_limit": float(vl[i]) if np.isfinite(vl[i]) and vl[i] > 0 else None,
                           "effort_limit": float(el[i]) if np.isfinite(el[i]) and el[i] > 0 else None, "group": grp})
            gains[n] = {"stiffness": float(st[i]), "damping": float(dm[i])}
        base = {"type": "mobile", "root_link": "pelvis", "proprio": "base_pose, base_vel",
                "command": "the last 4 ee_pose columns [vx, vy, wz, hip_height]: forward / sideways speed (m/s) and turn rate (rad/s) in the "
                           "robot's own frame, standing height of the pelvis (m); a frozen locomotion policy walks the legs; the policy clips "
                           "vx, vy, wz and hip_height to its own ranges (hip_height 0.50-0.80)"} if self.loco else \
               {"type": "fixed", "pose": pose_of(self.base_T0), "note": "the pelvis is welded to the world (legs locked)"}
        return {"name": "unitree-g1-dex3", "description": "Unitree G1 humanoid (29 body joints) with two three-finger Dex3 hands",
                "base": base, "joints": joints,
                "grippers": {f"{s}_hand": {"joints": [n for n in self.hand_names if n.startswith(s)], "opening_m": [0.0, 0.12],
                                           "command": "the 7 hand joint targets of that hand in each ee_pose row (rad); PD position drives",
                                           "tcp_frame": f"{s}_wrist_yaw_link"} for s in ("left", "right")},
                "controller": {"type": "EmbodiedSWE Pink IK (whole upper body: waist + both arms solved for the two wrist poses, nullspace "
                                       "holding shoulders and waist near home) + direct hand joint targets" + (" + a frozen locomotion policy for the legs" if self.loco else ""),
                               "control_hz": self.ctrl_hz, "physics_hz": 1.0 / self.env.dt, "gains": gains,
                               "note": "every joint is a PD position drive; Pink IK writes the arm and waist targets"},
                "tcp": {f: {"parent_link": f, "link_T_frame": np.eye(4).tolist(),
                            "definition": "the wrist link itself (the frame Pink IK and the ee_pose action drive); the fingers extend beyond it"}
                        for f in self.ee_frames}}

    def kinematics_spec(self):
        return {"endpoint": "/kinematics", "joint_names": list(self.kin_joints), "frame": "base" if self.loco else "world",
                **({"base_pose_proprio": ["pelvis_x", "pelvis_y", "pelvis_z", "pelvis_qw", "pelvis_qx", "pelvis_qy", "pelvis_qz"]} if self.loco else {}),
                "quaternion_order": "wxyz", "links": self.link_names, "tcp_frames": self.ee_frames, "ee_frames": self.ee_frames,
                "ik_targets": self.ee_frames,
                "ik": {"solver": "damped least squares (pinocchio, the kinematics URDF the robot's Pink IK uses) over the 17 arm and waist joints",
                       "tol_pos_m": 1e-4, "tol_rot_rad": 1e-3, "max_iters": 300},
                "stateless": True,
                "note": "the upper body from the pelvis (waist, arms, hands; legs are not modelled); poses are in the pelvis frame unless the "
                        "request gives base_pose (the pelvis pose, x y z qw qx qy qz in the world)" if self.loco else
                        "the upper body from the pelvis (waist, arms, hands); world frame: the fixed pelvis pose is applied"}

    def _base(self, req):
        bp = req.get("base_pose")
        if bp is not None:
            return T_of(bp[:3], bp[3:]), "world"
        return (np.eye(4), "base") if self.loco else (self.base_T0, "world")

    def kinematics(self, req):
        Tb, fr = self._base(req)
        if req["op"] == "fk":
            Ts = self.chain.fk(np.asarray(req["q"]), req.get("links") or self.link_names)
            return {"ok": True, "op": "fk", "frame": fr, "poses": {k: pose_of(Tb @ T) for k, T in Ts.items()}}
        seed = np.asarray(req["seed"], dtype=np.float64) if req.get("seed") is not None else \
            t2np(self.art.data.default_joint_pos[0, [self.art.joint_names.index(n) for n in self.kin_joints]])
        tg = {k: np.linalg.inv(Tb) @ T_of(p[:3], p[3:]) for k, p in req["targets"].items()}
        q, rep, ok = self.chain.ik(tg, seed, tol_p=req.get("tol_pos_m") or 1e-4, tol_r=req.get("tol_rot_rad") or 1e-3, iters=req.get("max_iters") or 300)
        return {"ok": bool(ok), "op": "ik", "frame": fr, "q": [float(v) for v in q],
                "error": {k: {"pos_m": round(v[0], 6), "rot_rad": round(v[1], 6)} for k, v in rep.items()}}

    def kin_check(self):
        q = t2np(self.art.data.joint_pos[0, [self.art.joint_names.index(n) for n in self.kin_joints]])
        Ts = self.chain.fk(q, self.ee_frames)
        Tb = self.pelvis_T()
        return max(float(np.linalg.norm((Tb @ Ts[f])[:3, 3] - t2np(self.art.data.body_pos_w[0, b]))) for f, b in zip(self.ee_frames, self.wrist))

    def describe_action(self):
        s = (f"{self.dim} values: left wrist x y z qw qx qy qz, right wrist x y z qw qx qy qz (world frame, metres, wxyz; the wrist_yaw_link "
             "frames), then the 14 hand joint targets (rad) in the order of spec.action_types.ee_pose.layout")
        return s + (", then the base command vx vy wz hip_height (switched at the row, never interpolated)" if self.loco else "")


def make_rig(env):
    name = type(env.robot).__name__
    if name in ("FrankaRobot", "BimanualFranka"):
        return FrankaRig(env)
    if name == "G1Robot":
        return G1Rig(env)
    raise RuntimeError(f"no standard-mode rig for {name}")


def interp_row(rig, a, b, t):
    """The target at fraction t (0..1) from row a to row b: positions and joint values linear, quaternions slerped, the
    command columns (rig.step_cols) switched to b."""
    out = a + (b - a) * t
    for i, j in rig.quat_slices:
        out[i:j] = slerp(a[i:j], b[i:j], t)
    for c in rig.step_cols:
        out[c] = b[c]
    return out
