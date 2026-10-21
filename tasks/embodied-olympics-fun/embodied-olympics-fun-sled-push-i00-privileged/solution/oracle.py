"""sled-push reference solution (privileged; the 5 kg sled at the official turf friction): the walking policy walks into the sled while the arms, by a
differential IK on the robot's own model, keep each palm pad pressed on the rear of its handle (the handle's position
from s.state()["task"]["poles"]); the walk command keeps the pelvis a set distance behind the handles and steers the
robot and the sled along the lane's centre line."""
import math

import numpy as np
import mujoco

GAP = 0.42            # m: the pelvis this far behind the handles' axis
PRESS = 0.04          # m: each palm's target this far inside the handle's rear surface (the push)
PALM_Z = 0.94         # m: where on the handle the palms push
PAD = 0.022           # m: the palm pad's half thickness (its broad face from the palm point)


class ArmIK:
    def __init__(self, w):
        self.w, self.m = w, w.m
        self.kd = mujoco.MjData(w.m)
        r, L = w.robot, w.L
        names = [j[:-6] for j in L.JOINTS]
        self.cols = {s: [i for i, n in enumerate(names) if n.startswith(s + "_") and i >= 12] for s in ("left", "right")}
        self.mount = {s: w.m.body(f"{s}_mount").id for s in ("left", "right")}
        self.q = r.rest.copy()

    def solve(self, targets, iters=8, normal=(1.0, 0.0, 0.0), w_rot=0.3):
        w, m, kd, r = self.w, self.m, self.kd, self.w.robot
        kd.qpos[:] = w.d.qpos
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        q = self.q.copy()
        for _ in range(iters):
            kd.qpos[r.qadr] = q
            mujoco.mj_kinematics(m, kd)
            mujoco.mj_comPos(m, kd)
            for s, t in targets.items():
                sid = r.palm[s]
                e = t - kd.site_xpos[sid]
                ax = kd.xmat[self.mount[s]].reshape(3, 3)[:, 0]      # the pad's broad face's normal (the palm)
                er = np.cross(ax, np.asarray(normal))
                mujoco.mj_jacSite(m, kd, jp, jr, sid)
                c = self.cols[s]
                J = np.vstack([jp[:, r.vadr[c]], w_rot * jr[:, r.vadr[c]]])
                E = np.r_[e, w_rot * er]
                dq = J.T @ np.linalg.solve(J @ J.T + 1e-3 * np.eye(6), E)
                q[c] = np.clip(q[c] + np.clip(dq, -0.2, 0.2), r.lo[c], r.hi[c])
        self.q = q
        return q


def solve(s, gap=GAP, press=PRESS, palm_z=PALM_Z, vmax=0.6, ramp_s=1.5, log=None, mode="walk", k_yaw=0.3, k_y=0.5):
    w = s.world
    r, L = w.robot, w.L
    ik = ArmIK(w)
    while not s.done:
        st = s.state()
        poles = np.array(st["task"]["poles"])
        x, y, z, yaw = st["pelvis_xyz_yaw"]
        rad = w.task.p["pole_diameter_m"] / 2
        pr = -0.03 + (press + 0.03) * min(1.0, w.t / ramp_s)      # the palms come to the handles, then press
        syaw = float(-np.arctan2(poles[0][0] - poles[1][0], poles[0][1] - poles[1][1]))   # the sled's yaw
        mid = poles.mean(axis=0)
        h_des = float(np.clip(-k_y * mid[1], -0.2, 0.2))         # the sled's heading back to the lane's centre
        turn = float(np.clip(k_yaw * (syaw - h_des), -0.04, 0.04))   # press more on one side: the sled turns
        fwd = np.array([math.cos(syaw), math.sin(syaw), 0.0])
        tg = {"left": np.r_[poles[0][:2], palm_z] + fwd * (-rad - PAD + pr + turn),
              "right": np.r_[poles[1][:2], palm_z] + fwd * (-rad - PAD + pr - turn)}
        q = ik.solve(tg, normal=fwd)
        ex = (mid[0] - x) - gap
        vx = float(np.clip(1.5 * ex + 0.25, -0.2, vmax)) if w.t > ramp_s else 0.0
        sy = float(mid[1])
        vy = float(np.clip(-1.5 * (y - sy) - 0.8 * sy, -0.25, 0.25))
        wz = float(np.clip(1.5 * (syaw - yaw), -0.4, 0.4))
        s.step(mode, np.r_[vx, vy, wz, q[12:], 0.0, 0.0])
        if log is not None and w.k % 25 == 0:
            log(w)
    return s.status()
