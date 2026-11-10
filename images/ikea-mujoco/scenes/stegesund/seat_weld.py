"""Weld a threaded fastener to its host once it has physically seated, and remove all its contacts (user decisions 2026-10-15).

One shared implementation for all IKEA scenes. Purpose: speed. Once seated, a fastener is part of its host, so it
needs no contacts at all ("after weld, there shouldn't be any contact on that screw").

A scene declares, per threaded fastener (wherever geom names are asked for, a body name means all geoms of that body):
  - `body`: the fastener; `host`: the body its thread is in;
  - `thread_holes`: the host's internal-thread geoms of that hole (they stop colliding too);
  - `head_geoms`: the head/flange/collar geoms, used only to detect seating;
  - `thread_geoms` (optional): for a load-bearing threaded part (a leg, a screw-in foot) whose other geoms must keep
    colliding, list only its thread geoms (may sit on child bodies); those go off. Omitted (small screws, bolts):
    every geom of the fastener body goes off, head included;
  - `clamped`: the bodies the head clamps against the host (e.g. a leg plate). Since the head stops colliding, each
    clamped body is welded to the host at the same moment, at its current pose, so the joint stays closed.
and authors the inactive welds that `weld_xml()` writes (MuJoCo cannot add an equality at run time).

Arming: a fastener lying head-down on the table looks "seated", so create it with `armed=False` and call
`welder.arm(name)` when its drive starts (it is in its hole); only armed fasteners are checked. Default: armed.

Seated = a head geom touches something, the fastener's velocity relative to its host is below `v_eps` (m/s) and
`w_eps` (rad/s), and this has held for `hold_s` of simulated time. Then each weld is set to the current relative pose
(no snap) and activated, and every geom of the fastener plus its hole's thread geoms lose contype/conaffinity.

Welds and contact changes live in model fields, not in mjSTATE_INTEGRATION, so each event is recorded in
`welder.events`; save them with the trajectory and call `SeatWelder.apply_event` at the same sim time on replay.
Every weld of a fastener (its own and its clamped parts') pins the same point, the fastener's origin, i.e. the joint
(anchor = that point in the host frame, relpose position = that point in body1's frame), with the current relative
orientation. Body origins far from the joint cannot then turn small rotational give into drift (fixed 09:27 for the
host after UPPSTÅ's 2.9 mm off-axis, 10:05 for clamped parts after SKÅDIS's 0.25 mm spring-back).

Welded-group contact switch-off (speed; opt-in per scene: `SeatWelder(..., group_contacts_off=True)`, default off, and
then the welder never touches any other collision bit). A clamped part welded to its host is one rigid body with it, as
the seated fastener is, but it keeps its own contacts with the host (UPPSTÅ: an end block's 114 pieces on the board,
~900 contacts, for the rest of the episode). With the switch on, the bodies joined by latched welds (each welded
fastener's host and its clamped parts, transitively) form one rigid group, and contacts between two bodies of the same
group are off; the group's geoms keep every other contact exactly as before. The bits are the scheme of
ikea_runtime.RetentionMechanism's switch-off (kyrre): a group's geoms get contype = the group's own bit (16 and up) and
conaffinity = their own bits plus the other groups' bits; the build checks that the scene's collision bits make this
exact. They are a pure function of the weld state (`sync_contacts()`, called on every weld event, reset and set_state).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import mujoco
import numpy as np

# RoboARC's weld class (profiles.py _RIGID["weld"])
WELD_SOLREF = ".00125 1"
WELD_SOLIMP = ".9 .95 .001 .5 2"
GROUP_BIT0 = 16                   # collision bits 16.. mark the welded groups (no scene geom uses them)


def _weld(name, b1, b2):
    return (f'<weld name="{name}" body1="{b1}" body2="{b2}" active="false" '
            f'solref="{WELD_SOLREF}" solimp="{WELD_SOLIMP}"/>')


def weld_xml(name: str, fastener_body: str, host_body: str, clamped: list[str] = ()) -> str:
    """The inactive welds a scene must author for one fastener (put inside <equality>)."""
    return "".join([_weld(f"seat__{name}", fastener_body, host_body)] +
                   [_weld(f"seat__{name}__{c}", c, host_body) for c in clamped])


@dataclass
class Fastener:
    name: str
    body: str
    host: str
    thread_holes: list[str]
    head_geoms: list[str]
    clamped: list[str] = field(default_factory=list)
    thread_geoms: list[str] | None = None
    armed: bool = True
    bid: int = -1
    hid: int = -1
    welds: list[tuple[int, int]] = field(default_factory=list)   # (equality id, body1 id)
    off_geoms: list[int] = field(default_factory=list)
    head_ids: list[int] = field(default_factory=list)
    quiet_since: float | None = None
    welded: bool = False


def _rel_pose(d, b1, b2):
    """Pose of body2 in body1's frame (MuJoCo weld relpose), position and wxyz quaternion."""
    q1inv = np.zeros(4); mujoco.mju_negQuat(q1inv, d.xquat[b1])
    pos = np.zeros(3); mujoco.mju_rotVecQuat(pos, d.xpos[b2] - d.xpos[b1], q1inv)
    quat = np.zeros(4); mujoco.mju_mulQuat(quat, q1inv, d.xquat[b2])
    return pos, quat


def _rel_vel(m, d, b1, b2):
    """Linear (at body1's origin) and angular speed of body1 relative to body2."""
    v1 = np.zeros(6); v2 = np.zeros(6)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, b1, v1, 0)
    mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_BODY, b2, v2, 0)
    v2_at_b1 = v2[3:] + np.cross(v2[:3], d.xpos[b1] - d.xpos[b2])
    return float(np.linalg.norm(v1[3:] - v2_at_b1)), float(np.linalg.norm(v1[:3] - v2[:3]))


def _geoms(m, names):
    """Geom ids for a list of names; a body name stands for all geoms of that body. Unknown names -> ValueError."""
    out, bad = [], []
    for n in names:
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, n)
        if g >= 0:
            out.append(g); continue
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        if b >= 0:
            out += [i for i in range(m.ngeom) if m.geom_bodyid[i] == b]; continue
        bad.append(n)
    if bad:
        raise ValueError(f"seat_weld: no geom or body named {bad}")
    return out


class SeatWelder:
    def __init__(self, m, d, fasteners, hold_s=0.05, v_eps=1e-4, w_eps=0.05, group_contacts_off=False):
        self.m, self.d, self.f = m, d, list(fasteners)
        self.group_contacts_off = bool(group_contacts_off)
        self.hold_s, self.v_eps, self.w_eps = hold_s, v_eps, w_eps
        self.events: list[dict] = []
        name2id = lambda kind, n: mujoco.mj_name2id(m, kind, n)
        G, B, E = mujoco.mjtObj.mjOBJ_GEOM, mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_EQUALITY
        for f in self.f:
            f.bid, f.hid = name2id(B, f.body), name2id(B, f.host)
            weld_names = [(f"seat__{f.name}", f.body)] + [(f"seat__{f.name}__{c}", c) for c in f.clamped]
            f.welds = [(name2id(E, w), name2id(B, b1)) for w, b1 in weld_names]
            own = (_geoms(m, f.thread_geoms) if f.thread_geoms is not None
                   else [g for g in range(m.ngeom) if m.geom_bodyid[g] == f.bid])
            f.off_geoms = own + _geoms(m, f.thread_holes)
            f.head_ids = _geoms(m, f.head_geoms)
            bad = ([w for (e, _), (w, _) in zip(f.welds, weld_names) if e < 0] +
                   [n for n in [f.body, f.host] + f.clamped if name2id(B, n) < 0])
            if bad:
                raise ValueError(f"seat_weld: fastener {f.name}: not in the model: {bad}")

        if self.group_contacts_off:
            self._prepare_groups()

    # -- welded-group contact switch-off (opt-in)
    def _prepare_groups(self):
        m = self.m
        self._orig = (m.geom_contype.copy(), m.geom_conaffinity.copy())
        self._body_orig = (m.body_contype.copy(), m.body_conaffinity.copy())
        ct, ca = self._orig
        live = (ct | ca) != 0
        if ((ct | ca) >> GROUP_BIT0).any():
            raise ValueError("seat_weld: a geom uses collision bits >= %d" % GROUP_BIT0)
        members = {b for f in self.f for b in [f.hid] + [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, c) for c in f.clamped]}
        off = {g for f in self.f for g in f.off_geoms}
        cls = {(int(ct[g]), int(ca[g])) for g in np.flatnonzero(live)}
        own = {(int(ct[g]), int(ca[g])) for g in np.flatnonzero(live) if int(m.geom_bodyid[g]) in members and g not in off}
        hit = lambda a, b: bool((a[0] & b[1]) | (b[0] & a[1]))
        for e in own:      # exactness: a group geom vs any other geom collides iff (its ct & our ca); two groups always
            if any(hit(e, o) != bool(o[0] & e[1]) for o in cls) or not all(hit(e, f) for f in own):
                raise ValueError("seat_weld: the collision bits %s do not allow an exact group switch-off" % (e,))
        self._members = sorted(members)
        self._geoms = {b: np.flatnonzero(live & (m.geom_bodyid == b)) for b in members}

    def groups(self):
        """The rigid groups of the latched welds: sorted lists of body ids (2 or more), ordered by their first body."""
        root = {}
        def find(x):
            while root.get(x, x) != x:
                x = root[x]
            return x
        for f in self.f:
            if f.welded:
                for c in f.clamped:
                    a, b = find(f.hid), find(mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, c))
                    if a != b:
                        root[max(a, b)] = min(a, b)
        comp = {}
        for b in self._members:
            comp.setdefault(find(b), set()).add(b)
        return sorted((sorted(c) for c in comp.values() if len(c) > 1), key=lambda c: c[0])

    def sync_contacts(self):
        """The model's collision bits from the weld state: the original bits, the welded groups' bits, and every welded
        fastener's own and thread-hole geoms off. A no-op unless the scene opted in."""
        if not self.group_contacts_off:
            return
        m = self.m; ct, ca = self._orig
        nct, nca = ct.copy(), ca.copy()
        groups = self.groups()
        if len(groups) > 31 - GROUP_BIT0:
            raise ValueError("seat_weld: more than %d welded groups" % (31 - GROUP_BIT0))
        bits = [1 << (GROUP_BIT0 + k) for k in range(len(groups))]
        allbits = sum(bits)
        for grp, bit in zip(groups, bits):
            g = np.concatenate([self._geoms[b] for b in grp])
            nct[g] = bit
            nca[g] = ca[g] | (allbits & ~bit)
        for f in self.f:
            if f.welded:
                nct[f.off_geoms] = 0
                nca[f.off_geoms] = 0
        m.geom_contype[:] = nct
        m.geom_conaffinity[:] = nca
        # MuJoCo's body-pair pre-filter reads body_contype / body_conaffinity (OR over the body's geoms): keep a superset
        bct, bca = self._body_orig[0].copy(), self._body_orig[1].copy()
        if groups:
            np.bitwise_or.at(bct, m.geom_bodyid, nct)
            np.bitwise_or.at(bca, m.geom_bodyid, nca)
        m.body_contype[:] = bct
        m.body_conaffinity[:] = bca

    def arm(self, name):
        """Start checking this fastener for seating (call when its drive starts)."""
        next(x for x in self.f if x.name == name).armed = True

    def _head_in_contact(self, f):
        c = self.d.contact[: self.d.ncon]
        return bool(np.isin(c.geom1, f.head_ids).any() or np.isin(c.geom2, f.head_ids).any())

    def step(self):
        """Call after every mj_step. Returns the weld events made this step."""
        new, t = [], float(self.d.time)
        for f in self.f:
            if f.welded or not f.armed:
                continue
            v, w = _rel_vel(self.m, self.d, f.bid, f.hid)
            if v < self.v_eps and w < self.w_eps and self._head_in_contact(f):
                f.quiet_since = t if f.quiet_since is None else f.quiet_since
                if t - f.quiet_since >= self.hold_s:
                    ev = {"time": t, "fastener": f.name, "host": f.host, "clamped": list(f.clamped)}
                    self.apply_event(self.m, self.d, ev, self.f)
                    self.sync_contacts()
                    self.events.append(ev); new.append(ev)
            else:
                f.quiet_since = None
        return new

    @staticmethod
    def apply_event(m, d, ev, fasteners):
        """Weld the fastener (and its clamped parts) to the host at the current pose and drop all the fastener's
        contacts and its hole's thread contacts. Used live and on replay."""
        f = next(x for x in fasteners if x.name == ev["fastener"])
        for eq, b1 in f.welds:
            pos, quat = _rel_pose(d, b1, f.hid)
            # The weld pins one point: anchor (in body2 = host frame) to relpose pos (in body1 frame). Every weld of a
            # fastener (its own and its clamped parts') pins the fastener's origin, i.e. the joint itself; a body
            # origin far from the joint (a board centre 120-240 mm off) would let small rotational give show as drift.
            anchor, _ = _rel_pose(d, f.hid, f.bid)
            relpos, _ = _rel_pose(d, b1, f.bid)
            m.eq_data[eq, :3] = anchor
            m.eq_data[eq, 3:6] = relpos
            m.eq_data[eq, 6:10] = quat
            m.eq_data[eq, 10] = 1.0
            d.eq_active[eq] = 1
        for g in f.off_geoms:
            m.geom_contype[g] = 0
            m.geom_conaffinity[g] = 0
        f.welded = True
