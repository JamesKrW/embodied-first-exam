"""MuJoCo readers for the state tables: a body's pose and velocity, a free joint's state, joint positions and
velocities. Only reads (mj_objectVelocity reads the data's last forward pass; nothing is stepped)."""
from __future__ import annotations

import numpy as np

POSE = {"pos": "m", "quat": "wxyz", "lin_vel": "m/s", "ang_vel": "rad/s"}
JOINTS = {"joint_pos": "rad or m", "joint_vel": "rad/s or m/s"}


def r(x, n=5):
    return [round(float(v), n) for v in np.asarray(x, float).ravel()]


def body_id(model, name):
    import mujoco as mj
    i = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, name)
    if i < 0:
        raise KeyError(f"no body {name!r}")
    return i


def body(model, data, name_or_id) -> dict:
    """World pose and velocity of a body (its frame's origin)."""
    import mujoco as mj
    i = name_or_id if isinstance(name_or_id, (int, np.integer)) else body_id(model, name_or_id)
    v = np.zeros(6)
    mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_BODY, int(i), v, 0)      # [angular, linear], world frame
    return {"pos": r(data.xpos[i]), "quat": r(data.xquat[i], 6), "lin_vel": r(v[3:]), "ang_vel": r(v[:3])}


def site(model, data, name) -> dict:
    import mujoco as mj
    i = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, name)
    if i < 0:
        raise KeyError(f"no site {name!r}")
    v = np.zeros(6)
    mj.mj_objectVelocity(model, data, mj.mjtObj.mjOBJ_SITE, i, v, 0)
    q = np.zeros(4)
    mj.mju_mat2Quat(q, data.site_xmat[i])
    return {"pos": r(data.site_xpos[i]), "quat": r(q, 6), "lin_vel": r(v[3:]), "ang_vel": r(v[:3])}


def free_joint(model, data, joint) -> dict:
    """A free joint's state from qpos / qvel (position, wxyz, linear velocity, angular velocity: MuJoCo's convention
    for a free joint, the angular velocity in the body's local frame is converted to the world frame)."""
    import mujoco as mj
    j = joint if isinstance(joint, (int, np.integer)) else mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, joint)
    if j < 0:
        raise KeyError(f"no joint {joint!r}")
    a, b = model.jnt_qposadr[j], model.jnt_dofadr[j]
    quat = np.asarray(data.qpos[a + 3:a + 7], float)
    w_local = np.asarray(data.qvel[b + 3:b + 6], float)
    R = np.zeros(9)
    mj.mju_quat2Mat(R, quat)
    return {"pos": r(data.qpos[a:a + 3]), "quat": r(quat, 6), "lin_vel": r(data.qvel[b:b + 3]),
            "ang_vel": r(R.reshape(3, 3) @ w_local)}


def joints(model, data, names) -> dict:
    import mujoco as mj
    pos, vel = {}, {}
    for n in names:
        j = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, n)
        if j < 0:
            continue
        pos[n] = round(float(data.qpos[model.jnt_qposadr[j]]), 6)
        vel[n] = round(float(data.qvel[model.jnt_dofadr[j]]), 6)
    return {"joint_pos": pos, "joint_vel": vel}


def joint_names(model, pattern=None):
    import re
    import mujoco as mj
    out = []
    for j in range(model.njnt):
        n = mj.mj_id2name(model, mj.mjtObj.mjOBJ_JOINT, j) or ""
        if model.jnt_type[j] in (2, 3) and (pattern is None or re.search(pattern, n)):     # slide, hinge
            out.append(n)
    return out


def turn(x):
    """A readout turned 180 degrees about z (world -> a frame facing the other way: (x, y, z) -> (-x, -y, z)), the
    turn every two-ended sport here uses between its world and a player's frame. Joint values are unchanged."""
    if not isinstance(x, dict):
        return x
    out = {}
    for k, v in x.items():
        if k in ("pos", "lin_vel", "ang_vel", "vel") and isinstance(v, list) and len(v) >= 2:
            out[k] = [-v[0], -v[1], *v[2:]]
        elif k == "quat" and isinstance(v, list) and len(v) == 4:
            w, a, b, c = v                                  # (0, 0, 0, 1) * q, wxyz
            out[k] = [round(-c, 6), round(-b, 6), round(a, 6), round(w, 6)]
        elif k == "yaw" and isinstance(v, (int, float)):
            out[k] = round((float(v) + 2 * np.pi) % (2 * np.pi) - np.pi, 6)
        else:
            out[k] = v
    return out
