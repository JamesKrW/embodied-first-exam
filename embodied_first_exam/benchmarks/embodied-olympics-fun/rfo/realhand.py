"""The real hands: holding objects by contact and friction only (no weld: what a hand holds can slip, twist and drop).
Opt-in per side from a family's build():

    sc.real_hands["right"] = "dex1"            # and/or "left"; default: none (the palm pad and the weld grip)
    sc.real_hands["left"] = "dex3"             # the G1 only: Unitree's Dex3-1 three-finger hand (FINGER HANDS, below)
    sc.real_hands["left"] = "inspire"          # the H1-2 only: its own Inspire RH56DFX five-finger hand, fingers moving
    sc.real_hand_options["left"] = {"grip_scale": 2.0}   # optional, finger hands: see FINGER HANDS

The kinds of one world may differ per side; a side without one keeps its palm pad (or fist) bit for bit.

THE DEX1-1. Unitree's Dex1-1 two-finger parallel gripper on a wrist mount.
The Dex1-1 (https://www.unitree.com/Dex1-1, product page 2026-10): 143 x 78 x 67 mm, 0.55 kg; stroke 90 mm; jaw length
80 mm; grip force 5-120 N (adjustable); speed 50-375 mm/s; full-stroke close 0.24 s; width 78 mm half open, 120 mm
fully open; friction-grasp load 5 kg; sold for the G1, the H2 and the R1 (not for the H1 / H1-2: on the H1-2 it is a
deviation, mounted where the Inspire hand's palm ends, on the same wrist mount as every other hand tool).

The model, in the wrist mount's frame (+z along the forearm away from the wrist; +x the palm's normal):
  housing   a box 78 (x) x 67 (y) x 63 (z) mm from the mount's origin outwards, 0.43 kg
  jaws      two fingers 80 mm long (z, from 63 to 143 mm), 15 mm thick (x), 34 mm wide (y), 0.06 kg each, on slide
            joints along x: jaw "a" on the palm side (+x), jaw "b" on the back of the hand (-x); the opening between
            their pads is 90 mm open, 0 closed; an equality couples the two (the rack and pinion: one DoF)
  pads      the inner face of each jaw is a rubber pad of 3 x 2 overlapping segments (2 mm proud, 80 x 34 mm in all) so
            a handle gets a contact patch, not one point: friction 1.0, torsional friction 5 mm (condim 4), priority 2
            (the pad's contact parameters win over the held object's), stiff (solref 0.004 1, solimp 0.99 0.999)
  friction  a world with a real hand gets elliptic friction cones and impratio 100 (unless the family set them): MuJoCo's
            friction is soft, and with its defaults a clamped handle creeps under a steady torque (IMPRATIO)
  centre    a site `<side>_jaw_centre` (also the hand's palm site) midway between the pads and along them, mount axes
The housing and the jaws are part `<side>_hand` for the referee, contype 4 / conaffinity 0 like the body's hulls (they
touch every prop, contype 1 / conaffinity 13 or 5, not the robot itself), plus explicit pairs with the hulls of the
rest of the body (the other arm, the trunk, the legs) like a hand tool, and between two grippers.

The controller (each 1 ms physics step; RAW_DIM unchanged): the sim row's `grip_<side>` is the closure command c in
[0, 1] (clipped): the commanded opening is 90 mm x (1 - c). A setpoint per jaw moves towards 45 mm x c at most
187.5 mm/s (the opening at 375 mm/s: full stroke in 0.24 s), and each jaw is a force-limited PD servo on it:
force = clip(KP (setpoint - q) - KV dq, -120 N, 120 N), KP = 20 000 N/m, KV = 80 N s/m. A command that opens starts
from where the jaws are (the setpoint of a clamp, which ran on past the object, comes back to the jaws at once): a
clamp eases or lets go within a physics step, then the jaws open at the speed limit. On an object the jaws stop on,
the clamp force is therefore min(120 N, 10 N per mm that the commanded opening is below the object's width): a fully
closed command (c = 1) clamps anything wider than 12 mm at the full 120 N; a command a few mm under the width clamps
lightly (and the object slips).

What the hand holds (`World.held[side]`): the body that touches both jaws now (by contact, at every physics step);
None otherwise. A family puts an object between the open jaws at reset and calls `w.close_on(side, name)`.

The speed limit is the setpoint's: the jaws themselves follow it a little faster when nothing holds them (about
0.39 m/s opening freely) and, for a few milliseconds, up to about 0.45 m/s when a clamp springs open (the pads'
stored compression).

Deviations from the device: one closure command per hand (the real one also takes a speed and a force setting: here
the speed is the maximum and the force follows from the commanded opening); no 5 N minimum force; two pads at
friction 1.0 and 120 N hold about 240 N of shear, some five times the device's rated 5 kg friction-grasp load; the
drive's inertia (ARMATURE) is an estimate; the gripper's 0.55 kg comes on top of the wrist link's own mass, which
keeps the hand it replaces (the G1's rubber fist: only its mesh goes; the H1-2's Inspire hand: its finger bodies,
about 0.05 kg per hand, go, its palm stays); on the H1-2 (for which Unitree does not sell it) the gripper sits where
the Inspire palm ends, which makes that forearm longer than a real installation would.
"""
from __future__ import annotations

import base64
import math
import xml.etree.ElementTree as ET
import zlib

import numpy as np

import mujoco

KINDS = ("dex1", "dex3", "inspire")
FINGER_KINDS = ("dex3", "inspire")       # the finger hands (FINGER HANDS, the end of this file)
HAND_ROBOT = {"dex3": "unitree_g1", "inspire": "unitree_h1"}    # each finger hand is its robot's own (dex1: either)
STROKE = 0.090                 # m: the opening between the pads, fully open
HALF = STROKE / 2              # m: one jaw's travel
JAW_LEN, JAW_THICK, JAW_WIDE = 0.080, 0.015, 0.034
HOUSING = (0.039, 0.0335, 0.0315)          # half sizes (x, y, z): 78 x 67 x 63 mm
PAD = 0.002                    # m: the pad's thickness (proud of the finger)
CENTRE_Z = 2 * HOUSING[2] + JAW_LEN / 2    # 0.103 m: the jaws' centre along the mount's z
F_MAX = 120.0                  # N: the grip force limit (per jaw: the clamp force)
SPEED = 0.375                  # m/s: the opening's speed limit (each jaw half of it)
KP, KV = 20000.0, 80.0         # N/m, N s/m: the jaw servo
MASS_HOUSING, MASS_JAW = 0.43, 0.06
ARMATURE = 0.05                # kg: the drive's reflected inertia on each jaw (an estimate)
PAD_FRICTION = (1.0, 0.005, 0.0001)
PAD_SOLREF, PAD_SOLIMP = "0.004 1", "0.99 0.999 0.0005"
IMPRATIO = 100.0               # a world with a real hand: elliptic friction cones and impratio 100. MuJoCo's friction is
#                                soft: with the default pyramidal cones a 120 N grasp let a 0.84 N m handle torque turn it
#                                34 deg in 3 s; elliptic + impratio 10: 0.5 deg/s; 100: 0.05 deg/s
JAWS = ("a", "b")              # a: the palm side (+x), b: the back (-x)
FREEZE_ARMATURE = 1e5          # kg (m^2): the rest of the world's inertia while close_on settles a grasp


def _f(v) -> str:
    return " ".join(f"{float(x):.6g}" for x in v)


def build(root: ET.Element, mount: ET.Element, side: str, robot: str) -> list[str]:
    """Mount a Dex1-1 on `mount` (an ElementTree body) and add its motors and coupling. Returns the collision geoms that
    get explicit pairs with the rest of the robot's body (housing and fingers)."""
    col = dict(contype="4", conaffinity="0")
    pre = f"{side}_hand"
    ET.SubElement(mount, "site", name=f"{side}_jaw_centre", pos=_f((0, 0, CENTRE_Z)), size="0.006", group="4")
    ET.SubElement(mount, "geom", name=f"{pre}_housing", type="box", pos=_f((0, 0, HOUSING[2])), size=_f(HOUSING),
                  mass=f"{MASS_HOUSING}", rgba="0.18 0.18 0.2 1", friction="0.8 0.005 0.0001", group="2", **col)
    out = [f"{pre}_housing"]
    for j, s in zip(JAWS, (1, -1)):
        x0 = s * (HALF + PAD + JAW_THICK / 2)
        b = ET.SubElement(mount, "body", name=f"{pre}_jaw_{j}", pos=_f((x0, 0, CENTRE_Z)))
        ET.SubElement(b, "joint", name=f"{pre}_jaw_{j}", type="slide", axis=_f((-s, 0, 0)), range=f"0 {HALF:.6g}",
                      armature=f"{ARMATURE}", damping="0", limited="true")
        ET.SubElement(b, "geom", name=f"{pre}_jaw_{j}", type="box", size=_f((JAW_THICK / 2, JAW_WIDE / 2, JAW_LEN / 2)),
                      mass=f"{MASS_JAW * 0.8:.6g}", rgba="0.75 0.76 0.8 1", friction="0.8 0.005 0.0001", group="2",
                      **col)
        out.append(f"{pre}_jaw_{j}")
        k = 0
        for iz in range(3):                       # the pad: 3 x 2 segments on the inner face (-s x side), overlapping
            #                                       by 1 mm (a gap between them would give a handle lying on it tilted
            #                                       edge normals that squeeze it sideways)
            for iy in range(2):
                pz = (iz - 1) * JAW_LEN / 3
                py = (iy - 0.5) * JAW_WIDE / 2
                ET.SubElement(b, "geom", name=f"{pre}_pad_{j}{k}", type="box",
                              pos=_f((-s * (JAW_THICK / 2 + PAD / 2), py, pz)),
                              size=_f((PAD / 2, JAW_WIDE / 4 + 0.0005, JAW_LEN / 6 + 0.0005)),
                              mass=f"{MASS_JAW * 0.2 / 6:.6g}", rgba="0.08 0.08 0.08 1", condim="4", priority="2",
                              friction=_f(PAD_FRICTION), solref=PAD_SOLREF, solimp=PAD_SOLIMP, group="2", **col)
                k += 1
    act = root.find("actuator")
    for j in JAWS:
        ET.SubElement(act, "general", name=f"{pre}_jaw_{j}", joint=f"{pre}_jaw_{j}", gaintype="fixed",
                      gainprm=f"{KP:g}", biastype="affine", biasprm=f"0 {-KP:g} {-KV:g}", ctrllimited="true",
                      ctrlrange=f"0 {HALF:.6g}", forcelimited="true", forcerange=f"{-F_MAX:g} {F_MAX:g}")
    ET.SubElement(root.find("equality"), "joint", name=f"{pre}_couple", joint1=f"{pre}_jaw_b", joint2=f"{pre}_jaw_a",
                  polycoef="0 1 0 0 0", solref="0.002 1", solimp="0.95 0.99 0.001")
    return out


def strip_hand(root: ET.Element, side: str, robot: str) -> None:
    """The robot's own hand on that side goes where the gripper replaces it: the G1's rubber fist (a visual mesh) and
    the H1-2's Inspire fingers (visual-only bodies). The hand's collision shell of the H1-2 (part of the wrist link's
    hull and of the robot's self-collision) stays."""
    wl = next(b for b in root.iter("body") if b.get("name") == f"{side}_wrist_yaw_link")
    for g in list(wl.findall("geom")):
        if "rubber_hand" in (g.get("mesh") or ""):
            wl.remove(g)
    for b in list(wl.findall("body")):
        if (b.get("name") or "")[:2] in ("L_", "R_"):
            wl.remove(b)


class RealHands:
    """The grippers of one world: addresses, the servo's setpoints, the contacts that say what each holds."""

    def __init__(self, w, sides: dict):
        m = w.m
        self.w, self.sides = w, [s for s in ("left", "right") if s in sides]
        self.kind = {s: sides[s] for s in self.sides}
        self.jq = {s: np.array([int(m.jnt_qposadr[m.joint(f"{s}_hand_jaw_{j}").id]) for j in JAWS]) for s in self.sides}
        self.jv = {s: np.array([int(m.jnt_dofadr[m.joint(f"{s}_hand_jaw_{j}").id]) for j in JAWS]) for s in self.sides}
        self.act = {s: np.array([m.actuator(f"{s}_hand_jaw_{j}").id for j in JAWS]) for s in self.sides}
        self.centre = {s: m.site(f"{s}_jaw_centre").id for s in self.sides}
        # geom -> code: 0 not a jaw, 1 + 2 i + k for side i, jaw k (the finger and its pads)
        code = np.zeros(m.ngeom, dtype=np.int64)
        for i, s in enumerate(self.sides):
            for k, j in enumerate(JAWS):
                b = m.body(f"{s}_hand_jaw_{j}").id
                code[m.geom_bodyid == b] = 1 + 2 * i + k
        self.jaw_code = code
        self.robot_body = np.zeros(m.nbody, dtype=bool)
        self.robot_body[list(w.robot.bodies)] = True
        # what a hand can hold: a body that moves, i.e. hangs under a free joint (a prop, a racket, a rope's link),
        # never the world body, a static body or the robot itself
        free_root = np.zeros(m.nbody, dtype=bool)
        for j in range(m.njnt):
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                free_root[int(m.jnt_bodyid[j])] = True
        movable = np.zeros(m.nbody, dtype=bool)
        for b in range(1, m.nbody):
            p = b
            while p > 0 and not free_root[p]:
                p = int(m.body_parentid[p])
            movable[b] = p > 0
        self.holdable = movable & ~self.robot_body
        self._scratch = None
        self.reset()

    def reset(self) -> None:
        self.cmd = {s: 0.0 for s in self.sides}          # the closure command in force (0 open .. 1 closed)
        self.setpoint = {s: 0.0 for s in self.sides}     # each jaw's servo setpoint (m of travel from open)

    # ------------------------------------------------------------------ control
    def command(self, grips) -> None:
        """The sim row's grip values (left, right): the closure command of each real hand."""
        for i, s in enumerate(("left", "right")):
            if s in self.cmd:
                self.cmd[s] = float(np.clip(grips[i], 0.0, 1.0))

    def substep(self) -> None:
        """Before each physics step: the setpoints one step on (rate limited), into the jaw servos."""
        d = self.w.d
        step = SPEED / 2 * self.w.m.opt.timestep
        for s in self.sides:
            tgt = HALF * self.cmd[s]
            sp = self.setpoint[s]
            if tgt < sp:                          # opening: from where the jaws are (no wind-up of a blocked clamp,
                q = float(d.qpos[self.jq[s]].mean())      # whose setpoint ran on past the object)
                sp = max(tgt, min(sp, q))
            sp = sp + min(max(tgt - sp, -step), step)
            self.setpoint[s] = sp
            d.ctrl[self.act[s]] = sp

    def write_ctrl(self) -> None:
        """The setpoints in force into the servos (after a reset cleared the controls)."""
        for s in self.sides:
            self.w.d.ctrl[self.act[s]] = self.setpoint[s]

    def update_held(self) -> None:
        """After each physics step: what each hand holds (a movable body, under a free joint, touching both jaws),
        into World.held."""
        w, d, m = self.w, self.w.d, self.w.m
        n = d.ncon
        touch = {}
        if n:
            g = d.contact.geom[:n]
            keep = d.contact.dist[:n] <= 0.0
            g1, g2 = g[keep, 0], g[keep, 1]
            c1, c2 = self.jaw_code[g1], self.jaw_code[g2]
            b1, b2 = m.geom_bodyid[g1], m.geom_bodyid[g2]
            for ca, ob in ((c1, b2), (c2, b1)):
                sel = (ca > 0) & self.holdable[ob]
                for c, b in zip(ca[sel], ob[sel]):
                    touch.setdefault(int(b), set()).add(int(c))
        for i, s in enumerate(self.sides):
            both = sorted(b for b, cs in touch.items() if {1 + 2 * i, 2 + 2 * i} <= cs)
            names = [m.body(b).name for b in both]
            if w.held[s] in names:                # several between the jaws: the one already held, else the lowest id
                continue
            w.held[s] = names[0] if names else None

    # ------------------------------------------------------------------ what it senses
    def opening(self, s: str) -> float:
        return float(STROKE - self.w.d.qpos[self.jq[s]].sum())

    def opening_speed(self, s: str) -> float:
        return float(-self.w.d.qvel[self.jv[s]].sum())

    def force(self, s: str) -> float:
        """The clamp force (N): the mean of the two jaw servos' forces, positive closing."""
        return float(self.w.d.actuator_force[self.act[s]].mean())

    def layout(self) -> list[str]:
        return [f"{k}_{s}" for s in self.sides for k in ("jaw_opening", "jaw_speed", "jaw_force")]

    def proprio(self) -> list[float]:
        return [v for s in self.sides for v in (self.opening(s), self.opening_speed(s), self.force(s))]

    def state(self) -> dict:
        """For privileged tooling: per real hand the command, the opening (m), its speed (m/s), the clamp force (N),
        what it holds, the jaw centre's position and axes (world)."""
        d = self.w.d
        out = {}
        for s in self.sides:
            R = d.site_xmat[self.centre[s]].reshape(3, 3)
            out[s] = {"kind": self.kind[s], "command": self.cmd[s], "opening_m": self.opening(s),
                      "opening_speed_m_s": self.opening_speed(s), "force_n": self.force(s), "holding": self.w.held[s],
                      "centre": d.site_xpos[self.centre[s]].tolist(), "closing_axis": R[:, 0].tolist(),
                      "finger_axis": R[:, 2].tolist()}
        return out

    def facts(self) -> dict:
        """The grippers' description for the task texts and spec()["robot_facts"]: the device, the controller, the
        start (closure command and what each holds at t = 0)."""
        return {"kind": "dex1", "model": "Unitree Dex1-1 two-finger parallel gripper (modelled: 0.55 kg, jaws 80 mm "
                "long, 90 mm stroke, rubber pads)", "sides": list(self.sides), "stroke_m": STROKE,
                "jaw_length_m": JAW_LEN, "force_limit_n": F_MAX, "setpoint_opening_speed_m_s": SPEED,
                "jaw_speed_note": "the limit is the servo setpoint's; the jaws can move briefly up to about 0.45 m/s "
                                  "when a clamp springs open",
                "servo": {"kp_n_per_m": KP, "kd_n_s_per_m": KV, "rule": "each jaw: a setpoint moves towards 45 mm x "
                          "closure at most half setpoint_opening_speed_m_s (opening, it starts from the jaws' position); force = clip(kp (setpoint - q) - kd dq, "
                          "-force_limit_n, force_limit_n); the clamp on an object is min(force_limit_n, 10 N per mm "
                          "that the commanded opening 90 mm x (1 - closure) is below the object's width)"},
                "pad": {"friction": list(PAD_FRICTION), "condim": 4},
                "start": {s: {"closure": round(self.cmd[s], 4), "holding": self.w.held[s]} for s in self.sides}}

    def memory(self) -> np.ndarray:
        return np.array([v for s in self.sides for v in (self.cmd[s], self.setpoint[s])], float)

    # ------------------------------------------------------------------ the start
    def jaw_frame(self, s: str) -> tuple[np.ndarray, np.ndarray]:
        """(position, 3 x 3 rotation) of the jaw centre in the world: columns x = the closing axis (towards jaw a, the
        palm side), y = across the jaws, z = along the fingers away from the wrist."""
        m, d = self.w.m, self.w.d
        if self._scratch is None:               # the kinematics on a scratch copy: reading the frame between control
            self._scratch = mujoco.MjData(m)    # steps must not change the live data (the next step reads it)
        k = self._scratch
        k.qpos[:] = d.qpos
        k.mocap_pos[:], k.mocap_quat[:] = d.mocap_pos, d.mocap_quat
        mujoco.mj_kinematics(m, k)
        return k.site_xpos[self.centre[s]].copy(), k.site_xmat[self.centre[s]].reshape(3, 3).copy()

    def close_on(self, s: str, name: str, closure: float = 1.0, settle_s: float = 0.3) -> dict:
        """At reset: close this hand's jaws on the free body `name` (already placed between the open jaws) and settle
        the grasp, with the rest of the world held still: the jaws close at their speed limit with the body held in
        place until both jaws touch it, then the body is let go and the grasp settles under gravity for settle_s.
        The closure command then stays in force (the start row's grip value). Deterministic (a function of the state).
        Raises if the body is not held at the end. Returns the grasp: opening, force, how far the body moved."""
        w = self.w
        m, d = w.m, w.d
        if name not in w.free:
            raise KeyError(f"{name!r} is not a free body")
        qa, va = w.free[name]
        closure = float(np.clip(closure, 0.0, 1.0))
        keep_q = np.ones(m.nq, dtype=bool)
        keep_v = np.ones(m.nv, dtype=bool)
        keep_q[self.jq[s]] = keep_v[self.jv[s]] = False
        mujoco.mj_forward(m, d)
        q0, v0 = d.qpos.copy(), d.qvel.copy()
        v0[~keep_v] = 0.0
        v0[va:va + 6] = 0.0
        p_start = d.qpos[qa:qa + 3].copy()
        self.cmd[s] = closure
        i = 1 + 2 * self.sides.index(s)
        dt = m.opt.timestep
        n_close = int(math.ceil((HALF / (SPEED / 2) + 0.05) / dt))
        n_settle = int(round(settle_s / dt))
        # the rest of the world held still: put back after every step, and inert within it (a huge armature on its
        # degrees of freedom: an arm left to fall under gravity during the step would drag the body down the jaws)
        arm0 = m.dof_armature.copy()
        m.dof_armature[keep_v] = FREEZE_ARMATURE
        m.dof_armature[va:va + 6] = arm0[va:va + 6]
        try:
            self._settle(s, name, closure, keep_q, keep_v, q0, v0, i, n_close, n_settle, qa, va)
        finally:
            m.dof_armature[:] = arm0
        free = self._free
        d.qvel[self.jv[s]] = 0.0
        d.qvel[va:va + 6] = 0.0
        mujoco.mj_forward(m, d)
        self.update_held()
        if w.held[s] != name:
            raise RuntimeError(f"close_on({s!r}, {name!r}): the jaws do not hold it (opening {self.opening(s):.4f} m, "
                               f"{'both jaws touched it' if free else 'the jaws never touched it on both sides'})")
        return {"opening_m": self.opening(s), "force_n": self.force(s),
                "moved_m": float(np.linalg.norm(d.qpos[qa:qa + 3] - p_start))}

    def _settle(self, s, name, closure, keep_q, keep_v, q0, v0, i, n_close, n_settle, qa, va) -> None:
        m, d = self.w.m, self.w.d
        free, settled = False, 0
        for _ in range(n_close + n_settle):
            self.substep()
            mujoco.mj_step(m, d)
            d.qpos[keep_q] = q0[keep_q]
            d.qvel[keep_v] = v0[keep_v]
            if not free:
                d.qpos[qa:qa + 7] = q0[qa:qa + 7]
                d.qvel[va:va + 6] = 0.0
                n = d.ncon
                g = d.contact.geom[:n]
                body = m.body(name).id
                on = set()
                for a, b in zip(g[:, 0], g[:, 1]):
                    if m.geom_bodyid[b] == body:
                        on.add(int(self.jaw_code[a]))
                    elif m.geom_bodyid[a] == body:
                        on.add(int(self.jaw_code[b]))
                if {i, i + 1} <= on and abs(self.setpoint[s] - HALF * closure) < 1e-12:
                    free = True                   # both jaws on it and the servo at its command: the full clamp
                    keep_q[qa:qa + 7] = False
                    keep_v[va:va + 6] = False
            else:
                settled += 1
                if settled >= n_settle:
                    break
        self._free = free


# ================================================================================================ FINGER HANDS
# Two real multi-finger hands, each its robot's own, holding by the contact and friction of their fingers, thumb and
# palm (no weld). The agent's grip column of that side is one closure command c in [0, 1] (clipped) for the whole hand,
# through a synergy (as the makers' own drivers close a hand with one command): every drive's target is
# open + c x (closed - open). A family may set the synergy's closed pose per drive (real_hand_options "closed").
#
# THE DEX3-1 ("dex3", the G1). Unitree's three-finger hand (https://www.unitree.com/Dex3-1, product page 2026-10):
# 0.71 kg; 175 x 88 x 77 mm (flat); 7 joints: thumb 3 (0: -60..60 deg, 1: -35..60, 2: 0..100), index 2 and middle 2
# (0: 0..90, 1: 0..100); "6 motors drive directly, 1 motor drive with gears": joint motors F-1515-108 and F-1515-214,
# maximum actual torque with the torque in the direction of the speed 0.49 / 0.86 N m, against it (back-driven)
# 1.37 / 3.1 N m, speed 23 / 11 rad/s; rated grasp 0.5 kg (a 5 cm hard ball, palm down). The geared joint is taken to
# be the thumb's rotation (thumb 0; Menagerie gives it the larger torque limit, 2.45 N m against 1.4); the six
# others are direct (F-1515-108). The model is MuJoCo Menagerie's unitree_g1/g1_with_hands.xml (BSD-3-Clause, from
# Unitree's g1 URDF): its link frames, joint axes and ranges, link masses and inertias; its meshes (decimated to about
# 250 faces a link, 1100 the palm: DEX3_MESH at the end of this file; the right hand's are the left's mirrored) are the
# visual and the collision shapes (each a convex hull, as Menagerie collides them; thumb 1 a box, as Menagerie's).
# The palm takes the rest of the 0.71 kg (0.386 kg, a box's inertia). Written in Menagerie's wrist_yaw_link frame (the
# G1's here: x along the hand, the left palm facing -y, the right +y), placed under the wrist mount by the inverse of
# the mount's pose; the rubber fist it replaces goes (its mass stays in the wrist link: about 0.1 kg, not separable).
#
# THE INSPIRE RH56DFX ("inspire", the H1-2). The H1-2's own five-finger hand (Inspire Robots RH56DFX: 6 DoF, 12
# joints, 6 linear servo drives, self-locking; fingertip force 10 N per finger, 15 N the thumb; four-finger flexion
# 260 deg/s, thumb rotation 107 deg/s, thumb flexion 70 deg/s, closing 0.8 s; 540 g; passive load 8 kg per finger;
# the maker's listing via Knoxlabs, 2026-10). The H1-2's model already carries its finger links (Unitree unitree_rl_gym
# h1_2.xml, commit 276801e, and unitree_ros h1_2.urdf: their frames, meshes, masses, inertias), fixed there: here they
# get Unitree's own joints (axes, ranges) and the URDF's couplings (the mimic joints: each finger's intermediate =
# 1.0 x its proximal; the thumb's intermediate = 1.6 x, its distal = 2.4 x its pitch), the couplings as joint
# equalities; each finger link collides as its mesh's convex hull, the palm as the hand base's (the robot model's own
# collision geom). The thumb's pitch drive is held to 0..0.5 rad (0.5 x 1.6 and x 2.4 are its two couplings' limits).
#
# THE DRIVE (both). Each drive is a joint servo on a setpoint (the motor's position): the setpoint moves towards its
# target at the drive's speed limit, but closing it stops where the servo would push harder than the drive's ACTIVE
# limit (a motor stalls at its rated torque); the joint follows by torque = clip(kp (setpoint - q) + kd (setpoint
# speed - dq), -PASSIVE, PASSIVE), computed at every 1 ms physics step; a load beyond the PASSIVE limit (back-driving
# the gear: Dex3-1's 1.37 / 3.1 N m; the Inspire's self-locking screw: its 8 kg passive load) drags the setpoint
# with the joint (the drive slips). So a closed hand squeezes with the active limit, and holds against a load up to
# the passive one. Inspire forces are fingertip forces: the drive's torque is that force times the fingertip's lever
# (INSPIRE_LEVER, the fingertip's speed per drive speed at half closure, measured on this model).
#
# THE GRIP-FORCE FACTOR (a family's declared deviation): real_hand_options[side]["grip_scale"] = k (default 1.0)
# multiplies the drives' active and passive limits, kp and kd (a stronger drive of the same speed), as the motor-scale
# block does for the body's motors; robot_facts report the scaled values and k.
#
# What a hand holds (World.held[side]): a movable body (under a free joint) that touches its fingers and its thumb or
# palm now (by contact, at every physics step). The pads: the finger links and the palm collide with friction 1.0
# (rubber pads; the Dex1-1's), torsional 5 mm (condim 4), priority 2 (their contact parameters win), stiff (the
# Dex1-1 pads' solref / solimp); the robot model's palm hull (the Inspire's) keeps the robot's own contact.
# Collision: the hand's geoms are part `<side>_hand` (contype 4, conaffinity 0: every prop, not the robot), plus
# explicit pairs with the rest of the body's hulls (like a hand tool), between the two hands, and the thumb with the
# fingers. Grasp frame (grasp_frame): a site `<side>_grasp`, where a handle's axis sits in a closed power grasp:
# columns x out of the palm, z along the fingers, y = z x x (the left hand: towards the little finger, the right:
# towards the index finger).
FINGER_FRICTION = (1.0, 0.005, 0.0001)
FINGER_SOLREF, FINGER_SOLIMP = PAD_SOLREF, PAD_SOLIMP
SCALE_MAX = 10.0                  # a grip-force factor above this is refused (families state theirs and why)

# --- the Dex3-1: Menagerie's inertials (pos of the left link: the right mirrors y; quat left, right; mass; diaginertia)
_DEX3_IN = {
    "thumb_0": ("-0.000884246 -0.00863407 0.000944293", ("0.462991 0.643965 -0.460173 0.398986",
                "0.643965 0.462991 -0.398986 0.460173"), 0.0862366, "1.6546e-05 1.60058e-05 1.43741e-05"),
    "thumb_1": ("-0.000827888 -0.0354744 -0.0003809", ("0.685598 0.705471 -0.15207 0.0956069",
                "0.705471 0.685598 -0.0956069 0.15207"), 0.0588507, "1.28514e-05 1.22902e-05 5.9666e-06"),
    "thumb_2": ("-0.00171735 -0.0262819 0.000107789", ("0.703174 0.710977 -0.00017564 -0.00766553",
                "0.710977 0.703174 0.00766553 0.00017564"), 0.0203063, "4.61314e-06 3.86645e-06 1.53495e-06"),
    "f_0": ("0.0354744 0.000827888 0.0003809", ("0.391313 0.552395 0.417187 0.606373",
            "0.606373 0.417187 0.552395 0.391313"), 0.0588507, "1.28514e-05 1.22902e-05 5.9666e-06"),
    "f_1": ("0.0262819 0.00171735 -0.000107789", ("0.502612 0.491799 0.502639 0.502861",
            "0.502861 0.502639 0.491799 0.502612"), 0.0203063, "4.61314e-06 3.86645e-06 1.53495e-06"),
}
# (joint, parent, pos (left; the right mirrors y), inertial key, axis, range left, range right, closed left, right)
DEX3_LINKS = [
    ("thumb_0", None, (0.067, 0.003, 0.0), "thumb_0", "0 1 0", (-1.0472, 1.0472), (-1.0472, 1.0472), 0.0, 0.0),
    ("thumb_1", "thumb_0", (-0.0025, -0.0193, 0.0), "thumb_1", "0 0 1", (-0.724312, 1.0472), (-1.0472, 0.724312),
     1.0472, -1.0472),
    ("thumb_2", "thumb_1", (0.0, -0.0458, 0.0), "thumb_2", "0 0 1", (0.0, 1.74533), (-1.74533, 0.0), 1.74533,
     -1.74533),
    ("middle_0", None, (0.1192, 0.0046, -0.0285), "f_0", "0 0 1", (-1.5708, 0.0), (0.0, 1.5708), -1.5708, 1.5708),
    ("middle_1", "middle_0", (0.0458, 0.0, 0.0), "f_1", "0 0 1", (-1.74533, 0.0), (0.0, 1.74533), -1.74533, 1.74533),
    ("index_0", None, (0.1192, 0.0046, 0.0285), "f_0", "0 0 1", (-1.5708, 0.0), (0.0, 1.5708), -1.5708, 1.5708),
    ("index_1", "index_0", (0.0458, 0.0, 0.0), "f_1", "0 0 1", (-1.74533, 0.0), (0.0, 1.74533), -1.74533, 1.74533),
]
DEX3_MASS = 0.71
DEX3_PALM_POS = (0.0415, 0.003, 0.0)            # the palm mesh's frame in the wrist link (left; the right mirrors y)
DEX3_THUMB1_BOX = ((-0.001, -0.032, 0.0), (0.01, 0.015, 0.01))   # Menagerie's collision box of thumb 1 (left)
DEX3_GEARED = ("thumb_0",)                      # the F-1515-214 (gears); the others F-1515-108 (direct)
DEX3_DRIVE = {"108": dict(active=0.49, passive=1.37, speed=23.0), "214": dict(active=0.86, passive=3.1, speed=11.0)}
DEX3_KP, DEX3_KD, DEX3_ARMATURE, DEX3_DAMPING = 20.0, 0.1, 0.001, 0.02
DEX3_GRASP = (0.094, -0.031, 0.0)               # (left, wrist frame) a 28 mm bar's axis in the closed hand (hbar study)
DEX3_FINGERS = ("middle_0", "middle_1", "index_0", "index_1")
DEX3_THUMB = ("thumb_0", "thumb_1", "thumb_2")

# --- the Inspire RH56DFX (Unitree's h1_2 joints; L_ / R_ bodies of the H1-2 model)
# drive: (name, driven joint, coupled joints [(joint, ratio)], open, closed, speed rad/s, fingertip force N, group)
INSPIRE_FINGERS = ("index", "middle", "ring", "pinky")
INSPIRE_DRIVES = [
    ("thumb_yaw", "thumb_proximal_yaw", [], 0.0, 1.10, math.radians(107.0), 15.0, "thumb"),
    ("thumb_pitch", "thumb_proximal_pitch", [("thumb_intermediate", 1.6), ("thumb_distal", 2.4)], 0.0, 0.5,
     math.radians(70.0), 15.0, "thumb"),
] + [(f, f"{f}_proximal", [(f"{f}_intermediate", 1.0)], 0.0, 1.7, math.radians(260.0), 10.0, "fingers")
     for f in INSPIRE_FINGERS]
# joint: (body, axis left, axis right, range)
INSPIRE_JOINTS = {
    "thumb_proximal_yaw": ("thumb_proximal_base", "0 0 1", "0 0 -1", (-0.1, 1.3)),
    "thumb_proximal_pitch": ("thumb_proximal", "0 0 -1", "0 0 1", (0.0, 0.5)),
    "thumb_intermediate": ("thumb_intermediate", "0 0 -1", "0 0 1", (0.0, 0.8)),
    "thumb_distal": ("thumb_distal", "0 0 -1", "0 0 1", (0.0, 1.2)),
    **{f"{f}_{k}": (f"{f}_{k}", "0 0 -1", "0 0 1", (0.0, 1.7)) for f in INSPIRE_FINGERS
       for k in ("proximal", "intermediate")},
}
INSPIRE_PASSIVE_N = 8.0 * 9.81                  # the passive load per finger (8 kg): the self-locking drive's hold
INSPIRE_LEVER = {"thumb_yaw": 0.053, "thumb_pitch": 0.224, "index": 0.117, "middle": 0.122, "ring": 0.116, "pinky": 0.101}
INSPIRE_KP, INSPIRE_KD, INSPIRE_ARMATURE, INSPIRE_DAMPING = 200.0, 0.5, 0.001, 0.01
INSPIRE_COUPLE_SOLREF, INSPIRE_COUPLE_SOLIMP = "0.002 1", "0.95 0.99 0.001"
INSPIRE_GRASP = (0.160, -0.046, -0.006)         # (left, wrist frame) a 34 mm handle's axis on the palm, under the
#                                                 fingers (the best hold of a search, tests/hands inspire_grasp.py)
INSPIRE_MASS = 0.54                             # the maker's; the model keeps Unitree's link masses (see facts)


# Collision shapes: each link collides as its mesh's convex hull (Menagerie's choice for the Dex3-1; the Inspire's palm
# is the robot model's own hull). Primitives fitted to the meshes (boxes, capsules) were measured and saved nothing:
# a finger-hand world's extra cost is its degrees of freedom (the H1-2 with two Inspire hands: 67 instead of 47), not
# its contacts (tests/hands variants.py, 2026-10-21).


def _wrist_frame_site(parent: ET.Element, name: str, pos, side: str, mount: ET.Element | None) -> None:
    """The grasp site: at pos (left, wrist frame; the right mirrors y), x out of the palm, z along the fingers.
    Under `mount` (its frame) when given, else directly in the wrist link (`parent`)."""
    sg = 1.0 if side == "left" else -1.0
    p = np.array([pos[0], sg * pos[1], pos[2]], float)
    x, z = np.array([0.0, -sg, 0.0]), np.array([1.0, 0.0, 0.0])
    y = np.cross(z, x)
    if mount is not None:
        pi, qi = _mount_inverse(mount)
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, qi)
        R = R.reshape(3, 3)
        p, x, y = R @ p + pi, R @ x, R @ y
    ET.SubElement(parent, "site", name=name, pos=_f(p), xyaxes=_f(np.r_[x, y]), size="0.006", group="4")


def _mount_inverse(mount: ET.Element) -> tuple[np.ndarray, np.ndarray]:
    """(pos, quat) of the wrist link's frame in the mount's frame: the inverse of the mount's pose."""
    p = np.array([float(x) for x in (mount.get("pos") or "0 0 0").split()])
    q = np.array([float(x) for x in (mount.get("quat") or "1 0 0 0").split()])
    qi = np.array([q[0], -q[1], -q[2], -q[3]]) / np.linalg.norm(q)
    pi = np.zeros(3)
    mujoco.mju_rotVecQuat(pi, -p, qi)
    return pi, qi


def grasp_point_in_mount(mount: ET.Element, side: str, kind: str) -> str:
    """The grasp point of a finger hand in the mount's frame (the palm site's position), as a string."""
    g = DEX3_GRASP if kind == "dex3" else INSPIRE_GRASP
    sg = 1.0 if side == "left" else -1.0
    pi, qi = _mount_inverse(mount)
    out = np.zeros(3)
    mujoco.mju_rotVecQuat(out, np.array([g[0], sg * g[1], g[2]], float), qi)
    return _f(out + pi)


def _mesh_asset(root: ET.Element, name: str, v: np.ndarray, f: np.ndarray) -> None:
    asset = root.find("asset")
    if asset is None:
        asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "mesh", name=name, vertex=" ".join(f"{x:.5g}" for x in v.ravel()),
                  face=" ".join(str(int(i)) for i in f.ravel()), inertia="convex")


def build_dex3(root: ET.Element, mount: ET.Element, side: str, robot: str, opts: dict) -> tuple[list[str], list]:
    """Mount a Dex3-1 on `mount` and add its seven drives (motors driven by FingerHands) and the thumb-finger pairs.
    Returns (the collision geoms that get explicit pairs with the rest of the body, the thumb-finger pairs)."""
    if robot != "unitree_g1":
        raise ValueError("the Dex3-1 is the G1's hand (sc.real_hands = 'dex3' needs robot unitree_g1)")
    sg = 1.0 if side == "left" else -1.0
    pre = f"{side}_hand"
    meshes = dex3_meshes()
    for part, (v, f) in meshes.items():
        if side == "right":
            v, f = v * np.array([1.0, -1.0, 1.0]), f[:, ::-1]
        _mesh_asset(root, f"{pre}_{part}_mesh", v, f)
    col = dict(contype="4", conaffinity="0", condim="4", priority="2", friction=_f(FINGER_FRICTION),
               solref=FINGER_SOLREF, solimp=FINGER_SOLIMP, group="2", rgba="0.22 0.22 0.24 1", mass="0")
    pi, qi = _mount_inverse(mount)
    base = ET.SubElement(mount, "body", name=f"{pre}_base", pos=_f(pi), quat=_f(qi))
    _wrist_frame_site(base, f"{side}_grasp", DEX3_GRASP, side, None)
    m_links = sum(_DEX3_IN[k][2] for (_, _, _, k, *_r) in DEX3_LINKS)
    m_palm = DEX3_MASS - m_links
    a, b, c = 0.0867, 0.042, 0.0878                 # the palm mesh's extent (x, y, z)
    pp = (DEX3_PALM_POS[0] + a / 2, sg * DEX3_PALM_POS[1], 0.0)
    ET.SubElement(base, "inertial", pos=_f(pp), mass=f"{m_palm:.6g}",
                  diaginertia=_f((m_palm * (b * b + c * c) / 12, m_palm * (a * a + c * c) / 12,
                                  m_palm * (a * a + b * b) / 12)))
    look = dict(contype="0", conaffinity="0", group="2", rgba=col["rgba"], mass="0")
    ET.SubElement(base, "geom", name=f"{pre}_palm", type="mesh", mesh=f"{pre}_palm_mesh",
                  pos=_f((DEX3_PALM_POS[0], sg * DEX3_PALM_POS[1], 0.0)), **col)
    out = [f"{pre}_palm"]
    bodies = {None: base}
    for (j, parent, pos, ik, axis, rl, rr, _cl, _cr) in DEX3_LINKS:
        b_ = ET.SubElement(bodies[parent], "body", name=f"{pre}_{j}", pos=_f((pos[0], sg * pos[1], pos[2])))
        bodies[j] = b_
        _ipos, iquat, mass, dinert = _DEX3_IN[ik]
        ET.SubElement(b_, "inertial", pos=_dex3_ipos(ik, side), quat=iquat[0 if side == "left" else 1],
                      mass=f"{mass:g}", diaginertia=dinert)
        lo, hi = rl if side == "left" else rr
        ET.SubElement(b_, "joint", name=f"{pre}_{j}", type="hinge", axis=axis, range=f"{lo:.6g} {hi:.6g}",
                      limited="true", armature=f"{DEX3_ARMATURE:g}", damping=f"{DEX3_DAMPING:g}")
        if j == "thumb_1":                             # Menagerie collides thumb 1 as a box; its mesh is a look
            bp, bs = DEX3_THUMB1_BOX
            ET.SubElement(b_, "geom", name=f"{pre}_{j}_look", type="mesh", mesh=f"{pre}_{j}_mesh", **look)
            ET.SubElement(b_, "geom", name=f"{pre}_{j}", type="box", pos=_f((bp[0], sg * bp[1], bp[2])), size=_f(bs),
                          **dict(col, group="3"))
        else:
            ET.SubElement(b_, "geom", name=f"{pre}_{j}", type="mesh", mesh=f"{pre}_{j}_mesh", **col)
        out.append(f"{pre}_{j}")
    act = root.find("actuator")
    for (j, *_r) in DEX3_LINKS:
        d = DEX3_DRIVE["214" if j in DEX3_GEARED else "108"]
        t = d["passive"] * _scale(opts)
        ET.SubElement(act, "motor", name=f"{pre}_{j}", joint=f"{pre}_{j}", gear="1", ctrllimited="true",
                      ctrlrange=f"{-t:g} {t:g}")
    pairs = [(f"{pre}_{a_}", f"{pre}_{f_}") for a_ in ("thumb_1", "thumb_2") for f_ in DEX3_FINGERS]
    return out, pairs


def _dex3_ipos(key: str, side: str) -> str:
    """Menagerie's inertial position of a Dex3-1 link (the right hand mirrors y)."""
    v = [float(x) for x in _DEX3_IN[key][0].split()]
    if side == "right":
        v[1] = -v[1]
    return _f(v)


def _scale(opts: dict) -> float:
    k = float((opts or {}).get("grip_scale", 1.0))
    if not (0.0 < k <= SCALE_MAX):
        raise ValueError(f"real_hand_options grip_scale = {k:g}: a factor in (0, {SCALE_MAX:g}]")
    return k


def build_inspire(root: ET.Element, mount: ET.Element, side: str, robot: str, opts: dict) -> tuple[list[str], list]:
    """Give the H1-2's Inspire fingers (the model's L_ / R_ bodies under the wrist link) Unitree's joints, the URDF's
    couplings, collision hulls and six drives. Returns (the collision geoms, the thumb-finger
    pairs). The finger bodies: inspire_bodies() (no look-only shells on them: world.py)."""
    if robot != "unitree_h1":
        raise ValueError("the Inspire hand is the H1-2's (sc.real_hands = 'inspire' needs robot unitree_h1)")
    pre, tag = f"{side}_hand", ("L_" if side == "left" else "R_")
    wl = next(b for b in root.iter("body") if b.get("name") == f"{side}_wrist_yaw_link")
    bodies = {b.get("name"): b for b in wl.iter("body")}
    _wrist_frame_site(wl, f"{side}_grasp", INSPIRE_GRASP, side, None)
    col = dict(contype="4", conaffinity="0", condim="4", priority="2", friction=_f(FINGER_FRICTION),
               solref=FINGER_SOLREF, solimp=FINGER_SOLIMP, group="3", mass="0")
    out = []
    for j, (bn, axl, axr, rng) in INSPIRE_JOINTS.items():
        b = bodies[tag + bn]
        vis = next(g for g in b.findall("geom") if g.get("mesh"))
        jt = ET.Element("joint", name=f"{pre}_{j}", type="hinge", axis=axl if side == "left" else axr,
                        range=f"{rng[0]:g} {rng[1]:g}", limited="true", armature=f"{INSPIRE_ARMATURE:g}",
                        damping=f"{INSPIRE_DAMPING:g}")
        b.insert(1 if b.find("inertial") is not None else 0, jt)
        g = ET.SubElement(b, "geom", name=f"{pre}_{bn}", type="mesh", mesh=vis.get("mesh"), **col)
        for a in ("pos", "quat"):
            if vis.get(a) is not None:
                g.set(a, vis.get(a))
        out.append(f"{pre}_{bn}")
    eq = root.find("equality")
    act = root.find("actuator")
    k = _scale(opts)
    for (name, joint, couples, _o, _c, _sp, f_tip, _grp) in INSPIRE_DRIVES:
        for cj, ratio in couples:
            ET.SubElement(eq, "joint", name=f"{pre}_{cj}_couple", joint1=f"{pre}_{cj}", joint2=f"{pre}_{joint}",
                          polycoef=f"0 {ratio:g} 0 0 0", solref=INSPIRE_COUPLE_SOLREF, solimp=INSPIRE_COUPLE_SOLIMP)
        t = INSPIRE_PASSIVE_N * INSPIRE_LEVER[name] * k
        ET.SubElement(act, "motor", name=f"{pre}_{name}", joint=f"{pre}_{joint}", gear="1", ctrllimited="true",
                      ctrlrange=f"{-t:g} {t:g}")
    thumb = [f"{pre}_thumb_{x}" for x in ("intermediate", "distal")]
    fing = [f"{pre}_{f}_{x}" for f in ("index", "middle") for x in ("proximal", "intermediate")]
    pairs = [(a, b) for a in thumb for b in fing]
    return out, pairs


def inspire_bodies(side: str) -> set[str]:
    """The Inspire finger bodies of a side (the H1-2 model's names)."""
    tag = "L_" if side == "left" else "R_"
    return {tag + bn for (bn, *_r) in INSPIRE_JOINTS.values()}


def inspire_palm_geom(root: ET.Element, side: str) -> str:
    """The name of the H1-2 model's own collision geom of the Inspire hand base (the palm)."""
    wl = next(b for b in root.iter("body") if b.get("name") == f"{side}_wrist_yaw_link")
    for g in wl.findall("geom"):
        if "hand_base_link" in (g.get("mesh") or "") and g.get("name") and (g.get("contype") or "1") != "0":
            return g.get("name")
    raise RuntimeError(f"the {side} Inspire hand base has no collision geom")


def finger_spec(kind: str, side: str) -> list[dict]:
    """The drives of a finger hand on a side: name, joint (driven), coupled joints, open, closed, speed, active and
    passive limits (N m), group ("thumb" / "fingers"), kp, kd."""
    out = []
    if kind == "dex3":
        for (j, _p, _pos, _ik, _ax, _rl, _rr, cl, cr) in DEX3_LINKS:
            d = DEX3_DRIVE["214" if j in DEX3_GEARED else "108"]
            out.append(dict(name=j, joint=j, couples=[], open=0.0, closed=cl if side == "left" else cr,
                            speed=d["speed"], active=d["active"], passive=d["passive"],
                            group="thumb" if j.startswith("thumb") else "fingers", kp=DEX3_KP, kd=DEX3_KD,
                            motor="F-1515-214" if j in DEX3_GEARED else "F-1515-108"))
    else:
        for (name, joint, couples, o, c, sp, f_tip, grp) in INSPIRE_DRIVES:
            L = INSPIRE_LEVER[name]
            out.append(dict(name=name, joint=joint, couples=couples, open=o, closed=c, speed=sp, active=f_tip * L,
                            passive=INSPIRE_PASSIVE_N * L, group=grp, kp=INSPIRE_KP, kd=INSPIRE_KD,
                            fingertip_force_n=f_tip, lever_m=L))
    return out


class FingerHands:
    """The finger hands of one world (the interface of RealHands): addresses, the drives' setpoints, what each holds."""

    def __init__(self, w, sides: dict, options: dict | None = None):
        m = w.m
        options = options or {}
        self.w, self.sides = w, [s for s in ("left", "right") if s in sides]
        self.kind = {s: sides[s] for s in self.sides}
        self.scale = {s: _scale(options.get(s)) for s in self.sides}
        self.spec, self.jq, self.jv, self.act = {}, {}, {}, {}
        self.open, self.span, self.sgn, self.speed = {}, {}, {}, {}
        self.f_act, self.f_pas, self.kp, self.kd, self.hq, self.hv = {}, {}, {}, {}, {}, {}
        for s in self.sides:
            sp = finger_spec(self.kind[s], s)
            closed = dict(options.get(s, {}).get("closed", {}) or {})
            unknown = set(closed) - {d["name"] for d in sp}
            if unknown:
                raise KeyError(f"real_hand_options[{s!r}]['closed']: unknown drive(s) {sorted(unknown)}")
            for d in sp:
                if d["name"] in closed:
                    d["closed"] = float(closed[d["name"]])
            k = self.scale[s]
            self.spec[s] = sp
            jid = [m.joint(f"{s}_hand_{d['joint']}").id for d in sp]
            self.jq[s] = np.array([int(m.jnt_qposadr[i]) for i in jid])
            self.jv[s] = np.array([int(m.jnt_dofadr[i]) for i in jid])
            self.act[s] = np.array([m.actuator(f"{s}_hand_{d['name']}").id for d in sp])
            o = np.array([d["open"] for d in sp])
            c = np.array([d["closed"] for d in sp])
            self.open[s], self.span[s] = o, np.abs(c - o)
            self.sgn[s] = np.where(c >= o, 1.0, -1.0)
            self.speed[s] = np.array([d["speed"] for d in sp])
            self.f_act[s] = np.array([d["active"] for d in sp]) * k
            self.f_pas[s] = np.array([d["passive"] for d in sp]) * k
            self.kp[s] = np.array([d["kp"] for d in sp]) * k
            self.kd[s] = np.array([d["kd"] for d in sp]) * k
            pre = f"{s}_hand_"
            hj = [j for j in range(m.njnt) if m.joint(j).name.startswith(pre)]
            self.hq[s] = np.array([int(m.jnt_qposadr[j]) for j in hj])
            self.hv[s] = np.array([int(m.jnt_dofadr[j]) for j in hj])
        self.centre = {s: m.site(f"{s}_grasp").id for s in self.sides}
        # both sides' drives in one vector (the 1 kHz loop in numpy at once): per side a slice of it
        cat = lambda dd: np.concatenate([dd[s] for s in self.sides])  # noqa: E731
        n = [len(self.spec[s]) for s in self.sides]
        self._sl = {s: slice(sum(n[:i]), sum(n[:i + 1])) for i, s in enumerate(self.sides)}
        self._n = n
        self._jq, self._jv, self._act = cat(self.jq), cat(self.jv), cat(self.act)
        self._open, self._span, self._sgn, self._speed = cat(self.open), cat(self.span), cat(self.sgn), cat(self.speed)
        self._fa, self._fp, self._kp, self._kd = cat(self.f_act), cat(self.f_pas), cat(self.kp), cat(self.kd)
        self._ca, self._cp = self._fa / self._kp, self._fp / self._kp
        self._geom_body = m.geom_bodyid.copy()
        # geom -> code: 0 not a hand, 1 + 2 i the fingers of side i, 2 + 2 i its thumb and palm
        code = np.zeros(m.ngeom, dtype=np.int64)
        for i, s in enumerate(self.sides):
            for g in self.group_geoms(w, s, "fingers"):
                code[g] = 1 + 2 * i
            for g in self.group_geoms(w, s, "thumb_palm"):
                code[g] = 2 + 2 * i
        self.jaw_code = code
        self.robot_body = np.zeros(m.nbody, dtype=bool)
        self.robot_body[list(w.robot.bodies)] = True
        free_root = np.zeros(m.nbody, dtype=bool)
        for j in range(m.njnt):
            if m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE:
                free_root[int(m.jnt_bodyid[j])] = True
        movable = np.zeros(m.nbody, dtype=bool)
        for b in range(1, m.nbody):
            p = b
            while p > 0 and not free_root[p]:
                p = int(m.body_parentid[p])
            movable[b] = p > 0
        self.holdable = movable & ~self.robot_body
        self._scratch = None
        self.reset()

    # ------------------------------------------------------------------ geometry
    def group_geoms(self, w, s: str, group: str) -> list[int]:
        """The collision geoms of a hand's group: "fingers" (all but the thumb), "thumb_palm" (the thumb and the
        palm), "hand" (both)."""
        m = w.m
        kind = self.kind[s] if hasattr(self, "kind") else None
        pre = f"{s}_hand_"
        out = []
        for g in range(m.ngeom):
            n = m.geom(g).name
            if not n.startswith(pre) or n.endswith("_look") or not (m.geom_contype[g] or m.geom_conaffinity[g]):
                continue
            rest = n[len(pre):]
            thumb = rest.startswith("thumb") or rest == "palm"
            if group == "hand" or (group == "thumb_palm") == thumb:
                out.append(g)
        if kind == "inspire" and group in ("thumb_palm", "hand"):
            out.append(m.geom(self.palm_geom[s]).id)
        return out

    # ------------------------------------------------------------------ control
    def reset(self) -> None:
        self.cmd = {s: 0.0 for s in self.sides}                               # the closure command in force
        self._sp = np.zeros(sum(self._n))                                      # each drive's setpoint (rad from open)
        self._vs = np.zeros(sum(self._n))                                      # ... and its speed
        self.setpoint = {s: self._sp[self._sl[s]] for s in self.sides}        # (views per side)
        self.sp_speed = {s: self._vs[self._sl[s]] for s in self.sides}

    def command(self, grips) -> None:
        for i, s in enumerate(("left", "right")):
            if s in self.cmd:
                self.cmd[s] = float(np.clip(grips[i], 0.0, 1.0))

    def substep(self) -> None:
        """Before each physics step: the setpoints one step on (speed limit; closing stops at the active limit, a
        load beyond the passive limit drags them), and the drives' torques into the motors."""
        d, dt = self.w.d, self.w.m.opt.timestep
        sg, kp = self._sgn, self._kp
        x = sg * (d.qpos[self._jq] - self._open)                    # each drive's joint, rad from open (closing +)
        v = sg * d.qvel[self._jv]
        tgt = np.repeat([self.cmd[s] for s in self.sides], self._n) * self._span
        sp0 = self._sp.copy()
        step = self._speed * dt
        close = np.minimum(np.minimum(sp0 + step, tgt), np.maximum(sp0, x + self._ca))
        sp = np.where(tgt > sp0, close, np.maximum(sp0 - step, tgt))
        sp = np.minimum(np.maximum(sp, x - self._cp), x + self._cp)
        vs = (sp - sp0) / dt
        tau = np.minimum(np.maximum(kp * (sp - x) + self._kd * (vs - v), -self._fp), self._fp)
        self._sp[:], self._vs[:] = sp, vs
        d.ctrl[self._act] = sg * tau

    def write_ctrl(self) -> None:
        """After a reset cleared the controls: the drives' torques of the state in force (zero at a fresh reset)."""
        d = self.w.d
        for s in self.sides:
            x = self.sgn[s] * (d.qpos[self.jq[s]] - self.open[s])
            tau = np.clip(self.kp[s] * (self.setpoint[s] - x), -self.f_pas[s], self.f_pas[s])
            d.ctrl[self.act[s]] = self.sgn[s] * tau

    def update_held(self) -> None:
        """After each physics step: what each hand holds (a movable body touching its fingers and its thumb or palm),
        into World.held."""
        w, d, m = self.w, self.w.d, self.w.m
        n = d.ncon
        both = {s: np.zeros(0, dtype=np.int64) for s in self.sides}
        if n:
            g = d.contact.geom[:n]
            keep = d.contact.dist[:n] <= 0.0
            g1, g2 = g[keep, 0], g[keep, 1]
            c = np.concatenate([self.jaw_code[g1], self.jaw_code[g2]])
            b = np.concatenate([self._geom_body[g2], self._geom_body[g1]])
            sel = (c > 0) & self.holdable[b]
            c, b = c[sel], b[sel]
            if len(c):
                for i, s in enumerate(self.sides):
                    both[s] = np.intersect1d(b[c == 1 + 2 * i], b[c == 2 + 2 * i])
        for s in self.sides:
            names = [m.body(int(x)).name for x in both[s]]
            if w.held[s] in names:
                continue
            w.held[s] = names[0] if names else None

    # ------------------------------------------------------------------ what it senses
    def positions(self, s: str) -> np.ndarray:
        return self.w.d.qpos[self.jq[s]].copy()

    def torques(self, s: str) -> np.ndarray:
        return self.w.d.ctrl[self.act[s]].copy()

    def closure(self, s: str) -> float:
        """How far the closing drives are closed, on average (0 open, 1 at their closed positions)."""
        x = self.sgn[s] * (self.w.d.qpos[self.jq[s]] - self.open[s])
        f = self.span[s] > 0
        return float(np.mean(x[f] / self.span[s][f])) if f.any() else 0.0

    def closure_speed(self, s: str) -> float:
        v = self.sgn[s] * self.w.d.qvel[self.jv[s]]
        f = self.span[s] > 0
        return float(np.mean(v[f] / self.span[s][f])) if f.any() else 0.0

    def force(self, s: str) -> float:
        """The closing drives' mean torque as a fraction of their active limits (1: squeezing at the limit)."""
        t = self.sgn[s] * self.w.d.ctrl[self.act[s]]
        f = self.span[s] > 0
        return float(np.mean(t[f] / self.f_act[s][f])) if f.any() else 0.0

    def layout(self) -> list[str]:
        out = []
        for s in self.sides:
            out += [f"hand_closure_{s}", f"hand_closure_speed_{s}", f"hand_force_{s}"]
            out += [f"{s}_hand_{d['name']}_pos" for d in self.spec[s]]
            out += [f"{s}_hand_{d['name']}_torque" for d in self.spec[s]]
        return out

    def proprio(self) -> list[float]:
        out = []
        for s in self.sides:
            out += [self.closure(s), self.closure_speed(s), self.force(s)]
            out += self.positions(s).tolist() + self.torques(s).tolist()
        return out

    def state(self) -> dict:
        d = self.w.d
        out = {}
        for s in self.sides:
            R = d.site_xmat[self.centre[s]].reshape(3, 3)
            names = [x["name"] for x in self.spec[s]]
            out[s] = {"kind": self.kind[s], "command": self.cmd[s], "closure": self.closure(s),
                      "drives_rad": dict(zip(names, self.positions(s).tolist())),
                      "drive_torques_nm": dict(zip(names, self.torques(s).tolist())),
                      "holding": self.w.held[s], "grasp_centre": d.site_xpos[self.centre[s]].tolist(),
                      "palm_normal": R[:, 0].tolist(), "finger_axis": R[:, 2].tolist()}
        return out

    def facts(self) -> dict:
        out = {"kind": "+".join(sorted(set(self.kind.values()))), "sides": list(self.sides), "hands": {}}
        for s in self.sides:
            k, kind = self.scale[s], self.kind[s]
            drives = []
            for i, d in enumerate(self.spec[s]):
                e = {"name": d["name"], "joint": d["joint"], "open_rad": d["open"], "closed_rad": round(d["closed"], 5),
                     "speed_rad_s": round(d["speed"], 4), "active_torque_nm": round(self.f_act[s][i], 4),
                     "passive_torque_nm": round(self.f_pas[s][i], 4), "group": d["group"]}
                if d["couples"]:
                    e["coupled"] = {cj: r for cj, r in d["couples"]}
                if "fingertip_force_n" in d:
                    e["fingertip_force_n"] = round(d["fingertip_force_n"] * k, 3)
                    e["passive_fingertip_force_n"] = round(INSPIRE_PASSIVE_N * k, 2)
                if "motor" in d:
                    e["motor"] = d["motor"]
                drives.append(e)
            model = ("Unitree Dex3-1 three-finger hand (0.71 kg; thumb 3 + index 2 + middle 2 joints, each its own "
                     "motor; MuJoCo Menagerie g1_with_hands kinematics)" if kind == "dex3" else
                     "the H1-2's Inspire RH56DFX five-finger hand (6 self-locking linear drives for 12 joints: the "
                     "thumb's rotation and flexion, one drive per finger with its two joints coupled; Unitree's "
                     "h1_2 kinematics)")
            out["hands"][s] = {
                "kind": kind, "model": model, "grip_scale": k,
                "drives": drives,
                "servo": {"kp_nm_per_rad": round(float(self.kp[s][0]), 4), "kd_nm_s_per_rad": round(float(self.kd[s][0]), 4),
                          "rule": "each drive: a setpoint moves towards open + closure x (closed - open) at most "
                                  "speed_rad_s, but closing stops where kp (setpoint - q) would exceed "
                                  "active_torque_nm; torque = clip(kp (setpoint - q) + kd (setpoint speed - dq), "
                                  "-passive, passive) at every physics step; a load beyond passive_torque_nm drags "
                                  "the setpoint (the drive slips)"},
                "holding_rule": "a free body that touches the hand's fingers and its thumb or palm",
                "pad_friction": list(FINGER_FRICTION),
                "start": {"closure": round(self.cmd[s], 4), "holding": self.w.held[s]}}
            if k != 1.0:
                out["hands"][s]["grip_scale_note"] = (f"this family's {s} hand's drives are {k:g} times the device's "
                                                      "(a declared deviation of the task)")
        return out

    def memory(self) -> np.ndarray:
        return np.array([v for s in self.sides for v in (self.cmd[s], *self.setpoint[s], *self.sp_speed[s])], float)

    # ------------------------------------------------------------------ the start
    def grasp_frame(self, s: str) -> tuple[np.ndarray, np.ndarray]:
        """(position, 3 x 3 rotation) of the grasp site in the world: columns x out of the palm, y = z x x, z along
        the fingers (on a scratch copy of the kinematics: the live data is not touched)."""
        m, d = self.w.m, self.w.d
        if self._scratch is None:
            self._scratch = mujoco.MjData(m)
        k = self._scratch
        k.qpos[:] = d.qpos
        k.mocap_pos[:], k.mocap_quat[:] = d.mocap_pos, d.mocap_quat
        mujoco.mj_kinematics(m, k)
        return k.site_xpos[self.centre[s]].copy(), k.site_xmat[self.centre[s]].reshape(3, 3).copy()

    jaw_frame = grasp_frame

    def close_on(self, s: str, name: str, closure: float = 1.0, settle_s: float = 0.3) -> dict:
        """At reset: close this hand on the free body `name` (placed in the hand) and settle the grasp, the rest of the
        world held still: the drives close at their speed limits with the body held in place until it touches the
        fingers and the thumb or palm and every drive has reached its target or stalled at its active limit; then
        the body is let go and the grasp settles under gravity for settle_s. The closure command stays in force.
        Deterministic. Raises if the body is not held at the end. Returns the grasp: closure, force, how far the body
        moved."""
        w = self.w
        m, d = w.m, w.d
        if name not in w.free:
            raise KeyError(f"{name!r} is not a free body")
        qa, va = w.free[name]
        closure = float(np.clip(closure, 0.0, 1.0))
        keep_q = np.ones(m.nq, dtype=bool)
        keep_v = np.ones(m.nv, dtype=bool)
        keep_q[self.hq[s]] = keep_v[self.hv[s]] = False
        mujoco.mj_forward(m, d)
        q0, v0 = d.qpos.copy(), d.qvel.copy()
        v0[~keep_v] = 0.0
        v0[va:va + 6] = 0.0
        p_start = d.qpos[qa:qa + 3].copy()
        self.cmd[s] = closure
        dt = m.opt.timestep
        n_close = int(math.ceil((float(np.max(self.span[s] / self.speed[s])) + 0.3) / dt))
        n_settle = int(round(settle_s / dt))
        arm0 = m.dof_armature.copy()
        m.dof_armature[keep_v] = FREEZE_ARMATURE
        m.dof_armature[va:va + 6] = arm0[va:va + 6]
        i = self.sides.index(s)
        free, settled, still = False, 0, 0
        try:
            for _ in range(n_close + n_settle):
                sp_before = self.setpoint[s].copy()
                self.substep()
                mujoco.mj_step(m, d)
                d.qpos[keep_q] = q0[keep_q]
                d.qvel[keep_v] = v0[keep_v]
                if not free:
                    d.qpos[qa:qa + 7] = q0[qa:qa + 7]
                    d.qvel[va:va + 6] = 0.0
                    still = still + 1 if np.allclose(self.setpoint[s], sp_before, atol=1e-9) else 0
                    self.update_held()
                    if w.held[s] == name and still >= 20:
                        free = True
                        keep_q[qa:qa + 7] = False
                        keep_v[va:va + 6] = False
                else:
                    settled += 1
                    if settled >= n_settle:
                        break
        finally:
            m.dof_armature[:] = arm0
        d.qvel[self.hv[s]] = 0.0
        d.qvel[va:va + 6] = 0.0
        self.sp_speed[s][:] = 0.0
        mujoco.mj_forward(m, d)
        self.update_held()
        if w.held[s] != name:
            raise RuntimeError(f"close_on({s!r}, {name!r}): the hand does not hold it (closure {self.closure(s):.3f}, "
                               f"{'it was held before the settling' if free else 'it never touched the fingers and the thumb or palm together'}; side {i})")
        return {"closure": self.closure(s), "force": self.force(s),
                "moved_m": float(np.linalg.norm(d.qpos[qa:qa + 3] - p_start))}


def make_hands(w, sides: dict, options: dict | None = None):
    """The real hands of a world: the Dex1-1 grippers (RealHands) or the finger hands (FingerHands), one family of
    kinds per world."""
    kinds = set(sides.values())
    if kinds <= {"dex1"}:
        return RealHands(w, sides)
    if kinds <= set(FINGER_KINDS):
        h = FingerHands.__new__(FingerHands)
        h.palm_geom = {s: f"{s}_hand_palm" for s in sides}
        h.palm_geom.update(getattr(w, "_inspire_palm", {}))
        h.kind = dict(sides)
        h.__init__(w, sides, options)
        return h
    raise ValueError(f"real_hands: the Dex1-1 and the finger hands in one world are not supported, got {sorted(kinds)}")


def dex3_meshes() -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """The Dex3-1's link meshes (left hand, its wrist link's frames): Menagerie's unitree_g1 STL meshes, decimated
    (quadric simplification, then vertex clustering) and quantised to 0.01 mm."""
    raw = zlib.decompress(base64.b64decode("".join(DEX3_MESH.split())))
    out, o = {}, 0
    for name, nv, nf in DEX3_MESH_INDEX:
        v = np.frombuffer(raw, np.int16, nv * 3, o).reshape(-1, 3).astype(float) * 1e-5
        o += nv * 6
        f = np.frombuffer(raw, np.uint16, nf * 3, o).reshape(-1, 3).astype(np.int64)
        o += nf * 6
        out[name] = (v, f)
    return out


# The Dex3-1's meshes (dex3_meshes): Menagerie unitree_g1/assets/left_hand_*_link.STL (BSD-3-Clause), decimated by
# tests/hands tooling (fast_simplification, then vertex clustering), int16 vertices at 0.01 mm then uint16 faces per
# part, zlib, base64.
DEX3_MESH_INDEX = [('palm', 552, 1113), ('thumb_0', 90, 176), ('thumb_1', 130, 256), ('thumb_2', 119, 243), ('middle_0', 130, 256), ('middle_1', 119, 243), ('index_0', 130, 256), ('index_1', 119, 243)]
DEX3_MESH = """
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
"""
