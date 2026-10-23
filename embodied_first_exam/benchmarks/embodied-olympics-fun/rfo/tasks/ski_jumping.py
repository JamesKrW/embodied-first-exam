"""Ski jumping (FIS International Competition Rules, Ski Jumping; FIS Jumping Hills Construction Norm 2018): one jump
on a small hill (K 10 m) built to the norm's formulas, on the robot's own jumping skis: down the in-run, off the
take-off, through the air, a landing on the skis, and through the transition to the fall line without a fall."""
import math
import xml.etree.ElementTree as ET

import numpy as np

import mujoco

from rfo.task import TaskBase

# ==================================================================================== BEGIN ski-snow block
# The equipment and the snow (the same block in alpine_slalom, giant_slalom, ski_jumping and snowboard; its master
# copy is new-snow's block_skisnow.py). Ported, with its authors' permission, from the ski-snow model of the
# RoboOlympics freeski sport (github.com/CHIGUI0/roboolympics, sports/freeski/skisnow.py, MIT): frictionless
# condim-1 contacts of the planks' edge spheres with the snow, and, every 1 ms physics step, the tangential forces of
# the snow on every contact in the plank's own frame, plus the air's drag on every link. Changes in the port: the
# forces are written to d.xfrc_applied (at the bodies' centres of mass, torque included) instead of mj_applyFT; a
# plank is a free body held to the boot by a weld (the binding: rigid, as a real binding is until it releases)
# because the core mounts the robot after the family's build(); the plank is rigid (freeski's later flexing
# fore- and afterbody are not ported).
#
# Why a model of its own: MuJoCo's contact friction is isotropic in the tangent plane, a ski is not: it glides along
# its length and holds across it when it is edged. Per contact of a plank segment with the snow (Fn its normal force):
#   along   -mu_k(v) Fn sat(v_long): kinetic ski-snow friction, mu_k(v) after Wolfsperger, Meyer & Gilgien 2021
#           (Front. Mech. Eng. 7:728722): 0.043 at its optimum (6.5-11.5 m/s), up to +0.02 towards a standstill,
#           +0.003 per m/s above 11.5 m/s
#   across  -C sat(v_lat), C = mu_k Fn + min(Fn tan(phi), S_f e l): the snow carries the edged plank by the volume it
#           displaces (Mossner et al. 2013, J. Glaciol. 59(218):1170; Federolf's race-piste hardness H_V 0.21 N/mm^3 and
#           shear strength S_f 0.52 N/mm^2), so an edge at angle phi sinks e = sqrt(2 Fn tan(phi) / (H_V l)) and the snow
#           can hold its platform only up to its shear strength over the groove; a flat plank skids on its base
#   sidecut the "along" direction follows the arc of radius R_sc cos(phi) the edged plank is bent onto (Howe 2001;
#           Lieu & Mote 1985): an edged plank carves a turn of that radius, a flat one runs straight
# sat(v) = v / sqrt(v^2 + V0^2): each force opposes its own segment's sliding and never does positive work (an edged
# plank cannot climb, a sliding one cannot push itself). The snow's normal reaction is MuJoCo's soft contact.
# Air: drag 0.5 rho Cd A(u) |v| v on every link of the robot and on the planks, A(u) the link's area projected across
# its velocity (|u_x| A_x + |u_y| A_y + |u_z| A_z, the convex hulls of its visual shapes along its own axes), times the
# overlap factor (the robot's true silhouette over the links' summed areas, measured once standing), Cd 1.0 (a bluff
# body); the venue's air density is that of a mountain venue at about 1500 m, 1.052 kg/m^3 (ISA). MuJoCo's own fluid
# model is switched off in these worlds (it would count the air twice).
SNOW_BIT, PLANK_BIT = 1, 2                   # the snow accepts the planks' edge spheres (2), the feet (1), the hulls (4)
SNOW_CONAFF = 1 | 2 | 4 | 8
SNOW_RGBA = (0.94, 0.95, 0.98, 1)            # groomed snow (drawn flat white: looks.py leaves white surfaces alone)
SEG = 7                                      # contact spheres per edge along the running surface
SPHERE_R = 0.012
PLANK_SOLREF = "0.03 1.3"                   # the snow's normal compliance and damping at the planks' contacts (freeski)
PLANK_SOLIMP = "0.9 0.95 0.001"
BIND_SOLREF = "0.002 1"                     # the binding (a weld): as stiff as the 1 ms step allows
AIR_RHO = 1.052


class Plank:
    """A ski or a snowboard: metres and kilograms. `mount`: the binding centre's offset from the middle of the running
    surface (negative: behind); widths at the tip, the waist and the tail; `radius` the sidecut radius. For a
    snowboard `stance` is the distance between the two bindings' centres (0 for a ski)."""

    def __init__(self, model, length, tip_w, waist_w, tail_w, radius, mass, mount, binding_mass, stand_h,
                 shovel=0.16, tail=0.10, tip_rise=0.06, tail_rise=0.025, thick=0.016, boot_mass=0.0, stance=0.0):
        self.model, self.length, self.tip_w, self.waist_w, self.tail_w = model, length, tip_w, waist_w, tail_w
        self.radius, self.mass, self.mount, self.binding_mass, self.stand_h = radius, mass, mount, binding_mass, stand_h
        self.shovel, self.tail, self.tip_rise, self.tail_rise, self.thick = shovel, tail, tip_rise, tail_rise, thick
        self.boot_mass, self.stance = boot_mass, stance

    @property
    def run_len(self):
        return self.length - self.shovel - self.tail

    def half_width(self, s):
        """Half the width at s along the running surface (s = 0 its middle, + forward): the sidecut's parabola."""
        h = 0.5 * self.run_len
        w = self.tip_w if s >= 0 else self.tail_w
        return 0.5 * self.waist_w + 0.5 * (w - self.waist_w) * min((s / h) ** 2, 1.0)

    def tip_x(self):
        """The front end of the plank (the shovel's tip) along it from the middle of the running surface."""
        return 0.5 * self.run_len + self.shovel


def _f6(v):
    return " ".join(f"{float(x):.6g}" for x in v)


def add_plank(sc, name, pk: Plank, rgba=(0.85, 0.12, 0.10, 1), top_rgba=(0.95, 0.95, 0.95, 1), n_bind=1):
    """A free plank body `name` (its frame: the middle of the running surface, x forward along it, z up from its base)
    with its binding(s) and boot shell(s) as mass, looks, two rows of edge spheres (the snow contacts) and raised
    spheres at the shovel and the tail, and a solid plate (`<name>_plate`) for what the plank hits besides the snow
    (the other plank, gate poles; explicit pairs). Returns the body element."""
    b = sc.body(name, (0.0, 0.0, -50.0), free=True)
    L, H, T = pk.run_len, 0.5 * pk.run_len, pk.thick
    mb = (pk.binding_mass + pk.boot_mass) * n_bind
    m_tot = pk.mass + mb
    w = pk.waist_w
    Ixx = pk.mass * (w ** 2 + T ** 2) / 12 + mb * (0.07 ** 2 + 0.06 ** 2) / 12
    Iyy = pk.mass * (pk.length ** 2 + T ** 2) / 12 + mb * ((0.3 ** 2 + 0.06 ** 2) / 12 + pk.mount ** 2)
    Izz = pk.mass * (pk.length ** 2 + w ** 2) / 12 + mb * ((0.3 ** 2 + 0.07 ** 2) / 12 + pk.mount ** 2)
    ET.SubElement(b, "inertial", pos=_f6((pk.mount * mb / m_tot, 0, T + 0.02)), mass=f"{m_tot:.5g}",
                  diaginertia=_f6((Ixx, Iyy, Izz)))
    n = 12
    for i in range(n):                         # the running surface with its sidecut (looks only)
        s0, s1 = -H + L * i / n, -H + L * (i + 1) / n
        sm = 0.5 * (s0 + s1)
        hw = pk.half_width(sm)
        sc.geom(b, f"{name}_v{i}", type="box", pos=(sm, 0, 0.5 * T), size=(0.5 * (s1 - s0) + 2e-4, hw, 0.5 * T),
                rgba=rgba, group=1, mass=0, contype=0, conaffinity=0)
    sc.geom(b, f"{name}_top", type="box", pos=(0, 0, T + 0.0006), size=(H, 0.45 * w, 0.0007), rgba=top_rgba, group=1,
            mass=0, contype=0, conaffinity=0)
    for nm, sgn, ln, rise, wd in (("tip", 1, pk.shovel, pk.tip_rise, pk.tip_w), ("tail", -1, pk.tail, pk.tail_rise,
                                                                                   pk.tail_w)):
        ang = math.atan2(rise, ln)
        sc.geom(b, f"{name}_{nm}", type="box", pos=(sgn * (H + 0.5 * ln), 0, 0.5 * T + 0.5 * rise),
                euler=(0, -sgn * ang, 0), size=(0.5 * math.hypot(ln, rise), 0.45 * wd, 0.5 * T), rgba=rgba, group=1,
                mass=0, contype=0, conaffinity=0)
    # the solid plate: what the plank hits besides the snow (explicit pairs only)
    sc.geom(b, f"{name}_plate", type="box", pos=(0, 0, 0.5 * T + 0.004), size=(H + 0.5 * pk.shovel, 0.5 * w, 0.5 * T),
            rgba=(0, 0, 0, 0), group=4, mass=0, contype=0, conaffinity=0)
    offs = [0.0] if n_bind == 1 else [-0.5 * pk.stance, 0.5 * pk.stance]
    for k, off in enumerate(offs):             # binding(s): looks
        x0 = pk.mount + off
        sc.geom(b, f"{name}_bind{k}", type="box", pos=(x0, 0, T + 0.5 * pk.stand_h),
                size=(0.15, 0.03, 0.5 * pk.stand_h - 0.002) if n_bind == 1 else (0.04, 0.13, 0.5 * pk.stand_h - 0.002),
                rgba=(0.18, 0.18, 0.2, 1), group=1, mass=0, contype=0, conaffinity=0)
    r = SPHERE_R
    for e, sgn in (("l", 1), ("r", -1)):
        pts = [(f"{e}{i}", -H + L * i / (SEG - 1), 0.0, pk.half_width(-H + L * i / (SEG - 1))) for i in range(SEG)]
        pts += [(f"{e}tipc", H + 0.55 * pk.shovel, 0.55 * pk.tip_rise, 0.42 * pk.tip_w),
                (f"{e}tailc", -H - 0.55 * pk.tail, 0.55 * pk.tail_rise, 0.42 * pk.tail_w)]
        for nm, s_, z, hw in pts:
            sc.geom(b, f"{name}_e{nm}", type="sphere", size=(r,), pos=(s_, sgn * (hw - r), z + r),
                    contype=PLANK_BIT, conaffinity=0, condim=1, priority=2, group=4, mass=0, solref=PLANK_SOLREF,
                    solimp=PLANK_SOLIMP, rgba=(0.3, 0.3, 0.9, 0.5))
    return b


def bind(sc, plank, foot, relpos, relquat=(1, 0, 0, 0), name=None):
    """The binding: a weld of the plank to the foot body; relpos / relquat: the plank's frame in the foot's frame."""
    ET.SubElement(sc.root.find("equality"), "weld", name=name or f"binding_{plank}_{foot}", body1=foot, body2=plank,
                  relpose=_f6(tuple(relpos) + tuple(relquat)), solref=BIND_SOLREF, solimp="0.95 0.99 0.001")


def no_fluid(sc):
    """The world's air is this block's drag model: MuJoCo's own fluid forces off (they would count the air twice)."""
    opt = sc.root.find("option")
    for k in ("density", "viscosity"):
        opt.attrib.pop(k, None)


def _hull_poly(pts2):
    p = np.unique(np.round(pts2, 6), axis=0)
    if len(p) < 3:
        return p
    p = p[np.lexsort((p[:, 1], p[:, 0]))]

    def cross(o, a, b):
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    lower, upper = [], []
    for q in p:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], q) <= 0:
            lower.pop()
        lower.append(tuple(q))
    for q in p[::-1]:
        while len(upper) >= 2 and cross(upper[-2], upper[-1], q) <= 0:
            upper.pop()
        upper.append(tuple(q))
    return np.array(lower[:-1] + upper[:-1])


def _hull_area(pts2):
    h = _hull_poly(pts2)
    if len(h) < 3:
        return 0.0
    x, y = h[:, 0], h[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))


def _geom_points(m, g):
    t, size = m.geom_type[g], m.geom_size[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        mid = int(m.geom_dataid[g])
        P = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        P = np.array([[a, b, c] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)]) * size
    else:
        r = size[0]
        hz = size[1] if t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER) else 0.0
        a = np.linspace(0, 2 * np.pi, 16, endpoint=False)
        ring = np.stack([r * np.cos(a), r * np.sin(a), 0 * a], 1)
        P = np.concatenate([ring + [0, 0, z] for z in (-hz, hz)] + [np.array([[0, 0, -hz - r], [0, 0, hz + r]])])
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, m.geom_quat[g])
    return P @ R.reshape(3, 3).T + m.geom_pos[g]


class SnowModel:
    """The snow's tangential forces on the planks and the air's drag on the robot and the planks, written to
    d.xfrc_applied after every physics step from that step's contacts (the forces of the next step)."""

    def __init__(self, w, planks: dict, snow_geoms, H_V=0.21e9, S_f=0.52e6, mu_opt=0.043, cd=1.0, v0=0.03,
                 phi_max_deg=70.0):
        m = w.m
        self.m, self.H_V, self.S_f, self.mu_opt, self.cd, self.v0 = m, H_V, S_f, mu_opt, cd, v0
        self.phi_max = math.radians(phi_max_deg)
        self.is_snow = np.zeros(m.ngeom, bool)
        self.is_snow[[m.geom(n).id for n in snow_geoms]] = True
        self.seg_body = np.full(m.ngeom, -1, int)
        self.seg_s, self.seg_len, self.seg_w = np.zeros(m.ngeom), np.zeros(m.ngeom), np.zeros(m.ngeom)
        self.radius = np.ones(m.nbody) * 1e9
        self.plank_ids = []
        for name, pk in planks.items():
            bid = m.body(name).id
            self.plank_ids.append(bid)
            self.radius[bid] = pk.radius
            L = pk.run_len
            for e in ("l", "r"):
                for i in range(SEG):
                    g = m.geom(f"{name}_e{e}{i}").id
                    s = float(m.geom_pos[g][0])
                    self.seg_body[g], self.seg_s[g] = bid, s
                    self.seg_len[g] = L / (SEG - 1) * (0.5 if i in (0, SEG - 1) else 1.0)
                    self.seg_w[g] = 2 * pk.half_width(s)
                for nm in ("tipc", "tailc"):
                    g = m.geom(f"{name}_e{e}{nm}").id
                    self.seg_body[g], self.seg_s[g] = bid, float(m.geom_pos[g][0])
                    self.seg_len[g] = 0.5 * L / (SEG - 1)
                    self.seg_w[g] = pk.tip_w if nm == "tipc" else pk.tail_w
        self.is_seg = self.seg_body >= 0
        # the drag's links: the robot's bodies with visual shapes, and the planks
        bs, cs, As, pts = [], [], [], []
        for b in sorted(set(w.robot.bodies) | set(self.plank_ids)):
            gs = [g for g in range(m.ngeom) if m.geom_bodyid[g] == b and m.geom_group[g] in (1, 2)]
            if not gs:
                continue
            P = np.concatenate([_geom_points(m, g) for g in gs])
            c = 0.5 * (P.min(0) + P.max(0))
            bs.append(b)
            cs.append(c)
            As.append([_hull_area(P[:, [1, 2]]), _hull_area(P[:, [0, 2]]), _hull_area(P[:, [0, 1]])])
            pts.append(P)
        self.drag_b, self.drag_c, self.drag_A = np.array(bs, int), np.array(cs), np.array(As)
        self.drag_root = m.body_rootid[self.drag_b]
        self.overlap = self._overlap(w, pts)
        self.last = {}

    def _overlap(self, w, pts, res=0.01) -> float:
        """The robot's true silhouette seen from the front over its links' summed areas (standing as at t = 0, before
        the family places it; the planks excluded)."""
        m = self.m
        d0 = mujoco.MjData(m)
        mujoco.mj_kinematics(m, d0)
        rob = [i for i, b in enumerate(self.drag_b) if b not in self.plank_ids]
        u = d0.xmat[w.robot.pelvis].reshape(3, 3)[:, 0]
        e1 = np.cross(u, [0.0, 0.0, 1.0])
        e1 /= np.linalg.norm(e1)
        e2 = np.cross(u, e1)
        polys, summed = [], 0.0
        for i in rob:
            b = self.drag_b[i]
            R = d0.xmat[b].reshape(3, 3)
            W_ = pts[i] @ R.T + d0.xpos[b]
            polys.append(_hull_poly(np.stack([W_ @ e1, W_ @ e2], 1)))
            loc = R.T @ u
            summed += float(np.sum(np.abs(loc) * self.drag_A[i]))
        allp = np.concatenate([p for p in polys if len(p) >= 3])
        lo, hi = allp.min(0) - res, allp.max(0) + res
        gx, gy = np.meshgrid(np.arange(lo[0], hi[0], res), np.arange(lo[1], hi[1], res))
        q = np.stack([gx.ravel(), gy.ravel()], 1)
        inside = np.zeros(len(q), bool)
        for h in polys:
            if len(h) < 3:
                continue
            ok = np.ones(len(q), bool)
            for k in range(len(h)):
                a0, a1 = h[k], h[(k + 1) % len(h)]
                ok &= ((a1[0] - a0[0]) * (q[:, 1] - a0[1]) - (a1[1] - a0[1]) * (q[:, 0] - a0[0])) >= 0
            inside |= ok
        return float(inside.sum() * res * res) / max(summed, 1e-9)

    def mu_k(self, v):
        v = np.abs(v)
        return self.mu_opt + 0.02 * np.clip((6.5 - v) / 3.2, 0.0, 1.0) + 0.003 * np.maximum(v - 11.5, 0.0)

    def apply(self, d) -> None:
        m = self.m
        xf = d.xfrc_applied
        # the air
        b = self.drag_b
        R = d.xmat[b].reshape(-1, 3, 3)
        cpos = d.xpos[b] + np.einsum("bij,bj->bi", R, self.drag_c)
        cv = d.cvel[b]
        vel = cv[:, 3:] + np.cross(cv[:, :3], cpos - d.subtree_com[self.drag_root])
        sp = np.linalg.norm(vel, axis=1)
        loc = np.einsum("bji,bj->bi", R, vel / np.maximum(sp, 1e-9)[:, None])
        A = np.sum(np.abs(loc) * self.drag_A, axis=1)
        F = -0.5 * AIR_RHO * self.cd * self.overlap * (A * sp)[:, None] * vel
        xf[b, :3] = F
        xf[b, 3:] = np.cross(cpos - d.xipos[b], F)
        self.last = {"drag_n": float(np.linalg.norm(F.sum(0))), "n": 0}
        # the snow
        nc = d.ncon
        if nc == 0:
            return
        con = d.contact
        g1, g2 = con.geom1[:nc], con.geom2[:nc]
        a = self.is_seg[g1] & self.is_snow[g2]
        bb = self.is_seg[g2] & self.is_snow[g1]
        sel = np.nonzero((a | bb) & (con.efc_address[:nc] >= 0))[0]
        if len(sel) == 0:
            return
        seg = np.where(a[sel], g1[sel], g2[sel])
        pos = con.pos[sel]
        nrm = con.frame[sel, :3] * np.where(a[sel], -1.0, 1.0)[:, None]
        fn = np.maximum(d.efc_force[con.efc_address[sel]], 0.0)
        body = self.seg_body[seg]
        Rs = d.xmat[body].reshape(-1, 3, 3)
        xs, ys = Rs[:, :, 0], Rs[:, :, 1]
        cvb = d.cvel[body]
        v = cvb[:, 3:] + np.cross(cvb[:, :3], pos - d.subtree_com[m.body_rootid[body]])

        def dot(u, q):
            return np.sum(u * q, axis=1)

        phi = np.arcsin(np.clip(dot(ys, nrm), -1.0, 1.0))      # + : rolled onto the right (-y) edge
        aphi = np.minimum(np.abs(phi), self.phi_max)
        t0 = xs - dot(xs, nrm)[:, None] * nrm
        t0 /= np.maximum(np.linalg.norm(t0, axis=1), 1e-9)[:, None]
        l0 = np.cross(nrm, t0)
        engage = np.clip(aphi / math.radians(5.0), 0.0, 1.0)    # the sidecut's arc engages from 0 to 5 degrees
        kappa = -np.sign(phi) * engage / (self.radius[body] * np.cos(aphi))
        al = kappa * self.seg_s[seg]
        t = np.cos(al)[:, None] * t0 + np.sin(al)[:, None] * l0
        lat = np.cross(nrm, t)
        vt = v - dot(v, nrm)[:, None] * nrm
        vl, vq = dot(vt, t), dot(vt, lat)
        mu = self.mu_k(np.linalg.norm(vt, axis=1))
        ln = self.seg_len[seg]
        V = fn / self.H_V                                        # the displaced volume (Mossner et al. 2013)
        tp = np.tan(aphi)
        bw = self.seg_w[seg]
        e_wedge = np.sqrt(2.0 * V * np.maximum(tp, 1e-6) / ln)
        e_full = V / (ln * bw) + 0.5 * bw * np.sin(aphi)
        e = np.where(e_wedge <= bw * np.sin(aphi), e_wedge, e_full)
        cap = mu * fn + np.minimum(fn * tp, self.S_f * e * ln)
        f_long = -mu * fn * vl / np.sqrt(vl * vl + self.v0 ** 2)
        f_lat = -cap * vq / np.sqrt(vq * vq + self.v0 ** 2)
        Fc = f_long[:, None] * t + f_lat[:, None] * lat
        Tc = np.cross(pos - d.xipos[body], Fc)
        np.add.at(xf, (body, slice(0, 3)), Fc)
        np.add.at(xf, (body, slice(3, 6)), Tc)
        self.last.update(n=len(sel), fn=float(fn.sum()), phi_deg=float(np.degrees(np.mean(phi))),
                         f_long=float(f_long.sum()), f_lat=float(f_lat.sum()))


class Digest:
    """The applied forces in the state hash (they carry the last physics step's contact forces into the next)."""

    def __init__(self, w, bodies):
        self.w, self.b = w, np.asarray(sorted(set(bodies)), int)

    def state(self):
        return [round(float(x), 9) for x in self.w.d.xfrc_applied[self.b].ravel()]


class Profile:
    """A snow surface's long profile z(x) of straight lines and circular arcs (freeski's profile.py), extruded across
    the course: built as thin tilted boxes, one per piece (a 1 m-thick slab under the surface)."""

    def __init__(self, x0=0.0, z0=0.0, angle_deg=0.0):
        self.pts = [(x0, z0)]
        self.ang = math.radians(angle_deg)
        self.marks = {}

    @property
    def end(self):
        return self.pts[-1]

    def mark(self, name):
        self.marks[name] = self.end
        return self

    def line(self, length, angle_deg=None):
        if angle_deg is not None:
            self.ang = math.radians(angle_deg)
        x, z = self.end
        self.pts.append((x + length * math.cos(self.ang), z + length * math.sin(self.ang)))
        return self

    def arc(self, radius, to_angle_deg, step=0.25):
        a1 = math.radians(to_angle_deg)
        da = a1 - self.ang
        if abs(da) < 1e-12:
            return self
        n = max(2, int(math.ceil(abs(da) * radius / step)))
        x, z = self.end
        sgn = 1.0 if da > 0 else -1.0
        cx, cz = x - sgn * radius * math.sin(self.ang), z + sgn * radius * math.cos(self.ang)
        for i in range(1, n + 1):
            a = self.ang + da * i / n
            self.pts.append((cx + sgn * radius * math.sin(a), cz - sgn * radius * math.cos(a)))
        self.ang = a1
        return self

    def arrays(self):
        p = np.array(self.pts)
        return p[:, 0], p[:, 1]

    def z_at(self, x):
        xs, zs = self.arrays()
        return np.interp(x, xs, zs)

    def slope_at(self, x):
        """The surface's angle (rad, + rising along +x) at x."""
        xs, zs = self.arrays()
        i = int(np.clip(np.searchsorted(xs, x) - 1, 0, len(xs) - 2))
        return math.atan2(zs[i + 1] - zs[i], xs[i + 1] - xs[i])

    def build(self, sc, prefix, half_width, rgba=SNOW_RGBA, thick=1.0, y=0.0):
        """The snow: one box per piece, overlapping its neighbours by 2 cm. Returns the geoms' names."""
        xs, zs = self.arrays()
        names = []
        for i in range(len(xs) - 1):
            dx, dz = xs[i + 1] - xs[i], zs[i + 1] - zs[i]
            L = math.hypot(dx, dz)
            a = math.atan2(dz, dx)
            ux, uz = dx / L, dz / L
            nx, nz = -uz, ux                       # the surface's normal
            cx = 0.5 * (xs[i] + xs[i + 1]) - nx * 0.5 * thick
            cz = 0.5 * (zs[i] + zs[i + 1]) - nz * 0.5 * thick
            nm = f"{prefix}{i}"
            sc.geom(sc.wb, nm, type="box", pos=(cx, y, cz), size=(0.5 * L + 0.02, half_width, 0.5 * thick),
                    quat=(math.cos(-a / 2), 0.0, math.sin(-a / 2), 0.0), rgba=rgba, contype=SNOW_BIT,
                    conaffinity=SNOW_CONAFF, friction=(0.35, 0.005, 0.0001), condim=3)
            names.append(nm)
        return names


def rot_y(a):
    return np.array([math.cos(a / 2), 0.0, math.sin(a / 2), 0.0])


def qmul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, np.asarray(a, float), np.asarray(b, float))
    return out


def place_on_slope(w, slope_rad, zfun, planks: dict, legs=None):
    """t = 0: the robot standing perpendicular to the snow (the surface z = zfun(x), falling at slope_rad along +x),
    its planks under its boots as the bindings hold them, resting on the snow. planks: name -> (foot body, relpos,
    relquat) of its binding. legs: the 12 leg joint angles to start in (default: as they are); the leg motors hold
    them (the arbiter's stand hold)."""
    m, d, r = w.m, w.d, w.robot
    a = r.arb
    if legs is not None:
        d.qpos[r.qadr[:12]] = legs
    fq = r.fq
    d.qpos[fq + 3:fq + 7] = qmul(rot_y(slope_rad), d.qpos[fq + 3:fq + 7])
    mujoco.mj_kinematics(m, d)
    for name, (foot, relpos, relquat) in planks.items():  # each plank where its binding holds it
        b = m.body(foot).id
        R = d.xmat[b].reshape(3, 3)
        w.put(name, d.xpos[b] + R @ np.asarray(relpos, float), qmul(d.xquat[b], relquat))
    mujoco.mj_kinematics(m, d)
    c = math.cos(slope_rad)
    gaps = [(float(d.geom_xpos[g][2]) - float(zfun(float(d.geom_xpos[g][0])))) * c - SPHERE_R
            for g in range(m.ngeom) if m.geom(g).name.rsplit("_e", 1)[0] in planks and "_e" in m.geom(g).name]
    dz = -(min(gaps) + 0.0003) / c                # the lowest edge sphere 0.3 mm into the snow
    d.qpos[fq + 2] += dz
    for name in planks:
        d.qpos[w.free[name][0] + 2] += dz
    L = a.GROUPS["legs"]
    a.q_a[L] = a.q_b[L] = d.qpos[a.qadr[L]]
    mujoco.mj_kinematics(m, d)
# ====================================================================================== END ski-snow block

G1_SOLE_Z, G1_FOOT_X = -0.035, 0.035         # the G1's sole below its ankle-roll link, its foot's middle ahead of it
# Jumping skis at the robot's size. FIS Specifications for Competition Equipment, Ski Jumping: a ski at most 145 % of
# the jumper's height, shorter for a body-mass index under 21 (the robot's is 19); the front of the boot at most 57 %
# of the ski's length from its tip; no sidecut (the base's grooves keep it straight). 1.70 m long (129 % of 1.32 m),
# 95 mm wide, 1.4 kg (estimate for a junior jumping ski), its binding plate 0.3 kg (estimate) with a 20 mm stand height;
# the boot's toe at 55 % of the length from the tip.
SJ_SKI = Plank("junior jumping ski 170 cm", 1.70, 0.095, 0.095, 0.095, 1.0e4, 1.40, -0.22, 0.30, 0.020,
               shovel=0.24, tail=0.04, tip_rise=0.10, tail_rise=0.005, thick=0.018)


def ski_relpos(pk):
    return (G1_FOOT_X - pk.mount, 0.0, G1_SOLE_Z - pk.stand_h)


def _hermite(x0, z0, s0, x1, z1, s1, n=24):
    """Points of the cubic from (x0, z0) with slope s0 to (x1, z1) with slope s1 (the knoll's cubic parabola)."""
    out = []
    h = x1 - x0
    for i in range(1, n + 1):
        t = i / n
        h00, h10, h01, h11 = 2 * t ** 3 - 3 * t ** 2 + 1, t ** 3 - 2 * t ** 2 + t, -2 * t ** 3 + 3 * t ** 2, t ** 3 - t ** 2
        out.append((x0 + t * h, h00 * z0 + h10 * h * s0 + h01 * z1 + h11 * h * s1))
    return out


class Task(TaskBase):
    name = "ski-jumping"
    title = "Ski jumping: small hill K 10"
    category = "track-and-jumps"
    robot = "unitree_g1"
    fall_rule = False                                # the snow's own fall rule (step)
    legal_ground = ("left_foot", "right_foot")
    rulebook = ("FIS International Competition Rules (ICR), Book III, Ski Jumping, edition 2024: Art. 411 (jumping hills "
                "to the FIS standards), 432 (the distance: where both feet are in full contact with the landing "
                "surface; in a fall, where a body part first touches it), the fall line (the end of the landing hill's "
                "transition, where the outrun starts) and a fall before it; FIS Jumping Hills Construction Norm 2018 "
                "(implementing Art. 411): table angle alpha = w/30 + 7.4 deg, h/n between w/800 + 0.4 and w/1000 + "
                "0.48, in-run angle gamma at most 32 deg on a beginners' hill (w < 30 m), HS = w / 0.9, beta_P = beta + "
                "alpha/2 - 2.5 deg, beta_0 = beta_P / 6, at most 0.7 g of centrifugal load at the end of the in-run's "
                "transition and 0.8 g in the landing's, distances w = 1.005 x the straight distance from the edge")
    deviations = [
        "Art. 411 and the Construction Norm 2018 (a hill certificate from FIS's JUMP program): the hill is built "
        "from the norm's published formulas for w = {w_m} m (K 10, HS 11) with h/n = {hn}, alpha = {alpha_deg} deg, "
        "gamma = {gamma_deg} deg; what JUMP computes from data it does not publish is our estimate: the take-off "
        "speed {v0_m_s} m/s that carries a point mass with the robot's drag and a take-off push of {v_perp_m_s} m/s "
        "across the table to K, beta = {beta_deg} deg from an equivalent landing height of {elh_K_m} m at K, the "
        "table's height {s_m} m, the knoll a cubic from beta_0 to beta_P, the landing circle's P {l1_m} m before K, "
        "the transitions as circles ({r1_m} m in the in-run, the norm's 0.7 g at {v0_m_s} m/s; {r2_m} m to the "
        "outrun, its 0.8 g), the table {table_m} m long (about a quarter second at take-off speed).",
        "In-run length: the start is where the robot, standing still, reaches about {v0_m_s} m/s at the edge in a "
        "crouch ({inrun_m} m of straight in-run above the transition); there is one start (no start gates, no "
        "gate changes, no wind), and no in-run track: the skis run on the groomed in-run, held straight by the "
        "snow model's edge grip.",
        "The start: no start bench and no push-off; the robot stands still on the in-run, perpendicular to it, "
        "and is released at t = 0.",
        "Equipment: jumping skis at the robot's size (see the module's constants); the binding holds the foot "
        "rigidly (a jumping binding lets the heel rise; the robot's ankle joint takes that role) and never "
        "releases; no ski boots (the feet are bound directly); rigid skis.",
        "Air and snow: still air at 1500 m (1.052 kg/m^3) with drag on every link but no lift (the ported snow-and-"
        "air model has no lift: a V-style flight gains nothing aerodynamically); the ski-snow contact is "
        "the freeski model of the CHIGUI0/roboolympics project (MIT, ported): kinetic friction after Wolfsperger et al. 2021, the edge's hold "
        "after Mossner et al. 2013, a hard prepared snow track.",
        "Judging: no style judges; the success line is the hill's construction point (a jump that reaches K, the "
        "distance a hill is built for) with a valid landing, judged by geometry at every 1 ms physics step: the "
        "landing is the first instant both skis touch the landing hill, its distance 1.005 x the straight distance "
        "from the edge to the profile point under the middle of the feet (the norm's approximation of Art. 432), "
        "in half metres; a fall (a body part other than the feet on the snow, or the pelvis less than {fall_h_m} m "
        "above the snow after the landing) before the fall line voids the jump; a telemark is not required.",
    ]
    time_limit_s = 20.0
    stages = ["take-off", "landing", "fall line"]
    params = dict(
        w_m=10.0, hn=0.451, alpha_deg=7.73, gamma_deg=30.0, v0_m_s=10.52, v_perp_m_s=1.0, elh_K_m=0.3,
        beta_deg=30.7, s_m=0.30, r1_m=17.0, table_m=2.6, inrun_m=12.3, l1_m=2.0, r2_m=26.0, outrun_m=40.0,
        inrun_width_m=3.0, hill_width_m=12.0, fall_h_m=0.35, tilt_max_deg=80.0, min_air_s=0.10, stop_speed_m_s=0.10, stop_s=1.5)

    sentence = ("Jump the K 10 hill: ski down the in-run, take off from the table, land on your skis at or beyond the "
                "K-point ({w_m} m) and ski through the transition past the fall line without falling.")
    rules = ("FIS ski jumping, one jump on a small hill (K {w_m} m) built to the FIS Construction Norm 2018. Your feet "
             "are bound to jumping skis. You stand still at the top of the in-run, which falls at {gamma_deg} degrees "
             "and bends (radius {r1_m} m) into the take-off table ({table_m} m long, falling at {alpha_deg} degrees); "
             "the table ends at its edge (x = 0) {s_m} m above the start of the landing hill. You are released at "
             "t = 0. The take-off: both skis leave the in-run with your feet past the edge. The landing: the first "
             "moment both skis touch the landing hill; the jump's distance is 1.005 times the straight distance from "
             "the edge to the point of the hill's profile under the middle of your feet, in half metres (rounded "
             "down). The landing hill: the knoll, the landing area from P to L (the K-point at {w_m} m, the hill size "
             "HS at {w_m} m / 0.9), the transition (radius {r2_m} m) to the flat outrun; the fall line is where the "
             "transition ends and the outrun starts. You succeed when, after a landing at a distance of at least "
             "{w_m} m, both feet pass the fall line without a fall, within {time_limit_s} s. You fail: on a fall (a "
             "part of you other than your feet on the snow at any time, or your pelvis less than {fall_h_m} m above "
             "the snow, measured along the hill's normal, outside the flight, or after the landing your torso tilted more than "
             "{tilt_max_deg} degrees from the hill's normal; a body part that touches the hill "
             "before both skis do is a fall, and the distance is measured there), if the flight lasted less than "
             "{min_air_s} s, if the distance is short of {w_m} m when your feet reach the fall line, if you stop "
             "(your pelvis slower than {stop_speed_m_s} m/s for {stop_s} s), or at {time_limit_s} s.")
    venue = ("You stand at the start gate near the top of the in-run, on your skis (1.70 m jumping skis), facing down "
             "the in-run (+x), perpendicular to it; the take-off's edge is at x = 0, about 20 m ahead of you along "
             "the in-run and about 8 m lower. The in-run ({gamma_deg} degrees, {inrun_width_m} m wide) bends into "
             "the table; the landing hill ({hill_width_m} m wide) starts {s_m} m below the edge, with its P (blue), K "
             "(red) and L (green) marks across it and the fall line (yellow) where the transition meets the "
             "outrun ({outrun_m} m long).")
    difficulty = ("The robot reaches about 10 m/s on the in-run and must jump at the right instant at the edge: too "
                  "late and it rotates forward, too early and it loses speed; with no lift from the air, only the "
                  "push at the edge carries it to K. In the air it must set its skis and body for a landing on a "
                  "slope of about 31 degrees at 13 m/s and hold its balance through the transition's g-load.")

    @property
    def start(self):
        """At the start gate: 1.2 m below the top of the in-run (x down the hill, the take-off's edge at x = 0)."""
        inrun, _ = self.profiles()
        return (float(inrun.pts[0][0]) + 1.2, 0.0, 0.0)

    # ------------------------------------------------------------------ the hill (the norm's construction)
    def hill(self):
        p = self.p
        w, hn = p["w_m"], p["hn"]
        n = w / math.sqrt(1 + hn * hn)
        h = n * hn
        a, g, b = (math.radians(p[k]) for k in ("alpha_deg", "gamma_deg", "beta_deg"))
        bP = b + math.radians(0.5 * p["alpha_deg"] - 2.5)
        b0 = bP / 6.0
        rL = p["l1_m"] / (bP - b)
        K = np.array([n, -h])
        C = K + rL * np.array([math.sin(b), math.cos(b)])      # the landing circle's centre (an upward open arc)
        on = lambda t: C - rL * np.array([math.sin(t), math.cos(t)])   # noqa: E731  the point where the slope is t
        P = on(bP)
        HS = w / 0.9
        # L: the point of the circle at HS (1.005 x straight distance from the edge)
        lo, hi = b - 0.5, b
        for _ in range(60):
            t = 0.5 * (lo + hi)
            if 1.005 * np.linalg.norm(on(t)) > HS:
                lo = t
            else:
                hi = t
        bL = 0.5 * (lo + hi)
        Lp = on(bL)
        return dict(n=n, h=h, a=a, g=g, b=b, bP=bP, b0=b0, rL=rL, K=K, P=P, L=Lp, bL=bL, HS=HS, C=C)

    def profiles(self):
        p, H = self.p, self.hill()
        # the in-run, built backwards from the edge T = (0, 0): the table, the transition of radius r1, the straight
        t_len, r1 = p["table_m"], p["r1_m"]
        a, g = H["a"], H["g"]
        E2 = np.array([-t_len * math.cos(a), t_len * math.sin(a)])
        # the arc from slope -g (at E1) to -a (at E2), concave up, radius r1: its centre above E2
        Cc = E2 + r1 * np.array([math.sin(a), math.cos(a)])
        E1 = Cc - r1 * np.array([math.sin(g), math.cos(g)])
        A = E1 + p["inrun_m"] * np.array([-math.cos(g), math.sin(g)])
        inrun = Profile(float(A[0]), float(A[1]), -p["gamma_deg"])
        inrun.line(p["inrun_m"]).mark("E1")
        inrun.arc(r1, -p["alpha_deg"]).mark("E2")
        inrun.line(float(np.hypot(*(np.array([0.0, 0.0]) - np.array(inrun.end)))))
        inrun.pts[-1] = (0.0, 0.0)
        inrun.mark("T")
        # the landing hill: the knoll's cubic from the base of the take-off to P, the landing circle P - K - L, the
        # transition (radius r2) to the outrun at U, the outrun
        land = Profile(0.0, -p["s_m"], -math.degrees(H["b0"]))
        land.pts += _hermite(0.0, -p["s_m"], -math.tan(H["b0"]), float(H["P"][0]), float(H["P"][1]), -math.tan(H["bP"]))
        land.mark("P")
        land.ang = -H["bP"]
        land.arc(H["rL"], -math.degrees(H["bL"]), step=0.1)
        land.mark("L")
        land.arc(p["r2_m"], 0.0).mark("U")
        land.line(p["outrun_m"], 0.0).mark("end")
        return inrun, land

    # ------------------------------------------------------------------ the venue
    def build(self, sc):
        p = self.p
        no_fluid(sc)
        self.inrun, self.land = self.profiles()
        zs = np.r_[self.inrun.arrays()[1], self.land.arrays()[1]]
        floor = next(g for g in sc.wb.findall("geom") if g.get("name") == "floor")
        floor.set("pos", f"0 0 {float(zs.min()) - 3.0:.4g}")
        self.inrun_geoms = self.inrun.build(sc, "inrun", p["inrun_width_m"] / 2)
        self.hill_geoms = self.land.build(sc, "hill", p["hill_width_m"] / 2)
        self.snow = self.inrun_geoms + self.hill_geoms
        sc.ground = ["floor"] + list(self.snow)
        # the take-off's front face (between the edge and the knoll's base): snow too
        sc.geom(sc.wb, "takeoff_face", type="box", pos=(-0.05, 0, -p["s_m"] / 2), size=(0.05, p["inrun_width_m"] / 2,
                p["s_m"] / 2), rgba=SNOW_RGBA, contype=SNOW_BIT, conaffinity=SNOW_CONAFF)
        self.snow.append("takeoff_face")
        H = self.hill()
        for nm, pt, rgba in (("P", H["P"], (0.1, 0.3, 0.85, 1)), ("K", H["K"], (0.85, 0.1, 0.1, 1)),
                             ("L", H["L"], (0.1, 0.6, 0.2, 1))):     # the hill's marks across it (painted)
            sc.geom(sc.wb, f"mark_{nm}", type="box", pos=(float(pt[0]), 0, float(pt[1]) + 0.004),
                    size=(0.05, p["hill_width_m"] / 2, 0.003), rgba=rgba, contype=0, conaffinity=0)
        ux, uz = self.land.marks["U"]
        sc.geom(sc.wb, "fall_line", type="box", pos=(ux, 0, uz + 0.004), size=(0.05, p["hill_width_m"] / 2, 0.003),
                rgba=(0.95, 0.75, 0.1, 1), contype=0, conaffinity=0)
        self.planks = {f"{s}_ski": (SJ_SKI, f"{s}_ankle_roll_link", ski_relpos(SJ_SKI), (1, 0, 0, 0))
                       for s in ("left", "right")}
        for name in self.planks:
            add_plank(sc, name, SJ_SKI, rgba=(0.95, 0.85, 0.1, 1))
        for name, (pk, foot, rp, rq) in self.planks.items():
            bind(sc, name, foot, rp, rq)
        ET.SubElement(sc.root.find("contact"), "pair", geom1="left_ski_plate", geom2="right_ski_plate", condim="3")
        ax, az = self.inrun.pts[0]
        sc.camera("venue", (6.0, -16.0, 4.0), (0.0, 0.0, -2.0), fovy=60)
        sc.camera("inrun", (ax - 3.0, 0.0, az + 2.0), (0.0, 0.0, -1.0), fovy=50)

    def surface_z(self, x):
        """The snow's height under x (the in-run for x < 0, the landing hill beyond the edge)."""
        if x < 0.0:
            return float(self.inrun.z_at(x))
        return float(self.land.z_at(x))

    def surface_slope(self, x):
        return self.inrun.slope_at(x) if x < 0.0 else self.land.slope_at(x)

    def reset(self, w):
        g = math.radians(self.p["gamma_deg"])
        # the robot stands at the start gate, perpendicular to the in-run
        place_on_slope(w, g, self.inrun.z_at, {n: (f, rp, rq) for n, (pk, f, rp, rq) in self.planks.items()})

    # ------------------------------------------------------------------ the referee
    def referee_reset(self, w, ref):
        m = w.m
        self.ss = SnowModel(w, {n: v[0] for n, v in self.planks.items()}, self.snow)
        self.plank_body = {n: m.body(n).id for n in self.planks}
        self.snow_mask = ref.geoms(names=self.snow)
        self.inrun_mask = ref.geoms(names=self.inrun_geoms + ["takeoff_face"])
        self.hill_mask = ref.geoms(names=self.hill_geoms)
        self.plank_mask = {n: ref.geoms(body=n) for n in self.planks}
        self.trackers = {"phase": "inrun", "air_k": 0, "t_off": None, "slow_k": 0, "distance_m": None,
                         "snow": Digest(w, list(self.plank_body.values()) + list(self.ss.drag_b))}
        self.ss.apply(w.d)

    def height(self, p) -> float:
        x = float(p[0])
        return (float(p[2]) - self.surface_z(x)) * math.cos(self.surface_slope(x))

    def landing_distance(self, w, x_land) -> float:
        """ICR 432 with the norm's approximation: 1.005 x the straight distance from the edge to the landing point on
        the profile, in half metres (rounded down)."""
        pt = np.array([x_land, self.surface_z(x_land)])
        return math.floor(2 * 1.005 * float(np.linalg.norm(pt))) / 2

    def substep(self, w, ref):
        self.ss.apply(w.d)
        tr, d = self.trackers, w.d
        on_run = [ref.touching(mk, self.inrun_mask) for mk in self.plank_mask.values()]
        on = {n: ref.touching(mk, self.hill_mask) for n, mk in self.plank_mask.items()}
        body_down = ref.parts_touching(self.snow_mask) - set(self.legal_ground)
        feet_x = [float(d.site_xpos[w.robot.foot_site[s]][0]) for s in ("left", "right")]
        if tr["phase"] == "inrun":
            if body_down and not (ref.failed or ref.success):    # the snow's fall rule, after every physics step
                return ref.fail("fell", on_the_snow=sorted(body_down))
            if not any(on_run) and min(feet_x) > 0.0:   # the skis have left the in-run, the feet past the edge
                ref.stage("take-off")
                tr["phase"], tr["t_off"] = "air", ref.time
        elif tr["phase"] == "air":
            if body_down:                                  # a fall: the distance where the body first touched
                pts = ref.contact_points(ref.part(*sorted(body_down)), self.snow_mask)
                tr["distance_m"] = self.landing_distance(w, float(pts[0][0])) if len(pts) else None
                ref.metrics["distance_m"] = tr["distance_m"]
                return ref.fail("fell", on_the_snow=sorted(body_down), distance_m=tr["distance_m"])
            if all(on.values()):
                air = ref.time - tr["t_off"]
                x = 0.5 * sum(feet_x)
                tr["distance_m"] = dist = self.landing_distance(w, x)
                ref.metrics["distance_m"] = dist
                ref.metrics["air_s"] = round(air, 3)
                fwd = [float(d.site_xpos[w.robot.foot_site[s]] @ np.array([1.0, 0, 0])) for s in ("left", "right")]
                ref.metrics["feet_apart_m"] = round(abs(fwd[0] - fwd[1]), 3)
                if air < self.p["min_air_s"]:
                    return ref.fail("no flight", air_s=round(air, 3))
                ref.stage("landing", distance_m=dist)
                tr["phase"] = "landed"
        elif tr["phase"] == "landed":
            if body_down and not (ref.failed or ref.success):
                return ref.fail("fell", on_the_snow=sorted(body_down))
            if min(feet_x) >= self.land.marks["U"][0]:
                if tr["distance_m"] < self.p["w_m"]:
                    return ref.fail("short of the K-point", distance_m=tr["distance_m"])
                if ref.stage("fall line"):
                    ref.succeed()

    def step(self, w, ref):
        p, tr = self.p, self.trackers
        if ref.failed or ref.success:
            return
        pel = w.d.xpos[w.robot.pelvis]
        h = self.height(pel)
        if h < p["fall_h_m"] and tr["phase"] != "air":
            return ref.fail("fell", pelvis_height_m=round(h, 3))
        if tr["phase"] == "landed":                     # tipped over on the skis (the torso more than tilt_max_deg
            a = self.surface_slope(float(pel[0]))        # from the hill's normal): a fall, though nothing touched yet
            nrm = np.array([-math.sin(a), 0.0, math.cos(a)])
            tilt = math.degrees(math.acos(float(np.clip(w.d.xmat[w.robot.torso].reshape(3, 3)[:, 2] @ nrm, -1, 1))))
            if tilt > p["tilt_max_deg"]:
                return ref.fail("fell", torso_tilt_deg=round(tilt, 1))
        v = float(np.linalg.norm(w.d.qvel[w.robot.fv:w.robot.fv + 3]))
        ref.metrics["speed_m_s"] = round(v, 3)
        tr["slow_k"] = tr["slow_k"] + 1 if v < p["stop_speed_m_s"] else 0
        if tr["slow_k"] * w.CONTROL_DT >= p["stop_s"] and w.t >= p["stop_s"]:
            return ref.fail("stopped", speed_m_s=round(v, 3))

    def privileged(self, w):
        H = self.hill()
        pel = w.d.xpos[w.robot.pelvis]
        return {"pelvis": w.pelvis().tolist(), "pelvis_height_m": self.height(pel), "phase": self.trackers.get("phase"),
                "edge_x": 0.0, "K": H["K"].tolist(), "P": H["P"].tolist(), "L": H["L"].tolist(),
                "fall_line_x": self.land.marks["U"][0], "beta_deg": math.degrees(H["b"]),
                "planks": {n: {"pos": w.d.xpos[b].tolist(), "xmat": w.d.xmat[b].tolist()}
                           for n, b in self.plank_body.items()},
                "snow_model": dict(self.ss.last)}
