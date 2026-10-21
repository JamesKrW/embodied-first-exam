"""Alpine skiing, giant slalom (FIS International Competition Rules, Alpine): one run down a shortened giant slalom
course of open gates on a groomed slope, on the robot's own skis: both ski tips and both feet through every gate (between
the inner poles of its two pole-and-panel pairs), then the finish."""
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

# ==================================================================================== BEGIN alpine-race block
# A race through gates on a groomed slope (the same block in alpine_slalom, giant_slalom and snowboard; its master
# copy is new-snow's block_alpine.py): the slope, the flex poles and the gates, the start, the referee of gate
# passing (FIS ICR 661.4), the finish and the falls.
POLE_R = 0.014                 # m: a 28 mm slalom pole (ICR 680: a round pole; FIS children rules: 25-27 mm, ICR
#                                803.2.1: light flex poles of 25-28.9 mm for U16-U14)
POLE_H = 1.80                  # m above the snow (ICR 680; the poles' FIS specification: at least 1.80 m out of the snow)
POLE_MASS = 0.60               # kg (a fibreglass pole and its flex head: estimate)
POLE_HINGE_Z = 0.03            # m: the flex head's pivot above the snow (the pole's base below it is screwed into the snow)
POLE_K = 20.0                  # N m / rad: the flex head's return spring (estimate: no published figure; a shin's
#                                shove of about 15 N at knee height folds it 20 degrees; it lies down under a body)
POLE_DAMP = 0.6                # N m s / rad (estimate)
POLE_FOLD = 1.45               # rad: how far it folds before its base stops it (nearly flat)
RED, BLUE = (0.85, 0.12, 0.12, 1), (0.10, 0.25, 0.80, 1)
FLAG_W, FLAG_H, FLAG_Z = 0.75, 0.50, 1.00    # m: a GS panel (ICR 901.2.2: about 75 x 50 cm, lower edge >= 1 m up)
TRACK = ("left_tip", "right_tip", "left_foot", "right_foot")   # what must cross every gate line (ICR 661.4.1)
TRACK_BOARD = ("nose", "tail", "left_foot", "right_foot")      # a snowboard's


class AlpineRace(TaskBase):
    """The race: the profile and the gates from params; the referee. A family sets params, the plank(s) and texts."""
    category = "track-and-jumps"
    robot = "unitree_g1"
    start = (0.0, 0.0, 0.0)
    fall_rule = False                                # the snow's own fall rule (step)
    legal_ground = ("left_foot", "right_foot", "left_hand", "right_hand")
    gate_kind = "sl"                                 # "sl": one flex pole per side; "gs": a pair with a panel; "psl":
    #                                                  snowboard parallel slalom's triangular panel on two poles
    planks: dict = {}                                # plank body -> (Plank, foot body, relpos, relquat)
    track = TRACK

    # ------------------------------------------------------------------ the geometry
    def slope(self):
        return math.radians(self.p["slope_deg"])

    def profile(self) -> Profile:
        p = self.p
        th = p["slope_deg"]
        pr = Profile(-8.0, 0.0, 0.0)
        pr.line(3.0, 0.0).mark("platform_end")
        pr.arc(p["brink_radius_m"], -th)
        # the straight slope: from the arc's end (just behind the start) to the finish and beyond
        x_end = p["course_m"] * math.cos(math.radians(th)) + 6.0
        pr.line((x_end - pr.end[0]) / math.cos(math.radians(th)), -th).mark("slope_end")
        pr.arc(p["outrun_radius_m"], 0.0).mark("outrun")
        pr.line(p["outrun_m"], 0.0).mark("end")
        xs, zs = pr.arrays()
        z0 = float(np.interp(0.0, xs, zs))            # the start: x = 0 on the surface, z = 0
        pr.pts = [(x, z - z0) for x, z in pr.pts]
        pr.marks = {k: (v[0], v[1] - z0) for k, v in pr.marks.items()}
        return pr

    def surf(self, s):
        """(x, z) of the point s metres down the slope from the start, along the surface's straight part."""
        th = self.slope()
        return s * math.cos(th), -s * math.sin(th)

    def gates(self) -> list[dict]:
        """Every gate: x of its gate line, its colour, the side of its turning pole (+1 left, -1 right) and the gate
        line's y interval (from the turning pole to the outside pole)."""
        p = self.p
        out = []
        for i in range(int(p["n_gates"])):
            s = p["first_gate_m"] + i * p["gate_spacing_m"]
            x, z = self.surf(s)
            side = 1.0 if (i % 2 == 0) == (p["first_side"] > 0) else -1.0
            yt = side * p["offset_m"]
            yo = yt + side * p["gate_width_m"]
            out.append(dict(i=i, s=s, x=x, z=z, side=side, y_turn=yt, y_out=yo, lo=min(yt, yo), hi=max(yt, yo),
                            colour="red" if i % 2 == 0 else "blue"))
        return out

    def finish(self):
        x, z = self.surf(self.p["course_m"])
        return x, z

    # ------------------------------------------------------------------ the venue
    def _pole(self, sc, name, x, y, z, rgba, flag=None):
        """A flex pole: a body on two crossed hinges at the snow (the flex head) with its return spring."""
        b = sc.body(name, (x, y, z + POLE_HINGE_Z))
        for ax, tag in (("1 0 0", "x"), ("0 1 0", "y")):
            ET.SubElement(b, "joint", name=f"{name}_flex{tag}", type="hinge", axis=ax, stiffness=f"{POLE_K:g}",
                          damping=f"{POLE_DAMP:g}", range=f"{-POLE_FOLD:g} {POLE_FOLD:g}", armature="0.002")
        m = POLE_MASS if flag is None else 2 * POLE_MASS + 0.15
        sc.geom(b, name, type="capsule", fromto=(0, 0, 0.03, 0, 0, POLE_H - POLE_HINGE_Z), size=(POLE_R,), mass=m,
                rgba=rgba, contype=1, conaffinity=1 | 4 | 8, condim=3, friction=(0.3, 0.005, 0.0001))
        if flag is not None:                          # a GS gate's pair: the second pole and the panel between them
            dy = flag
            sc.geom(b, f"{name}_b", type="capsule", fromto=(0, dy, 0.03, 0, dy, POLE_H - POLE_HINGE_Z),
                    size=(POLE_R,), mass=0, rgba=rgba, contype=1, conaffinity=1 | 4 | 8, condim=3,
                    friction=(0.3, 0.005, 0.0001))
            sc.geom(b, f"{name}_panel", type="box", pos=(0, dy / 2, FLAG_Z + FLAG_H / 2 - POLE_HINGE_Z),
                    size=(0.003, abs(dy) / 2 - POLE_R, FLAG_H / 2), mass=0, rgba=rgba, contype=1,
                    conaffinity=1 | 4 | 8, condim=3, friction=(0.3, 0.005, 0.0001))
        self._poles.append(name)
        return b

    def build_course(self, sc):
        p = self.p
        no_fluid(sc)
        pr = self.pr = self.profile()
        # the open ground (looks.py's floor) lowered under the lowest snow, so that it never meets the slope
        floor = next(g for g in sc.wb.findall("geom") if g.get("name") == "floor")
        floor.set("pos", f"0 0 {min(pr.arrays()[1]) - 3.0:.4g}")
        self.snow = pr.build(sc, "snow", p["course_width_m"] / 2)
        sc.ground = ["floor"] + list(self.snow)
        self._poles = []
        for g in self.gates():
            rgba = RED if g["colour"] == "red" else BLUE
            s = g["side"]
            if self.gate_kind == "sl":
                self._pole(sc, f"gate{g['i']}_turn", g["x"], g["y_turn"], g["z"], rgba)
                self._pole(sc, f"gate{g['i']}_out", g["x"], g["y_out"], g["z"], rgba)
            else:                                     # pairs: the inner poles at the gate line's ends
                self._pole(sc, f"gate{g['i']}_turn", g["x"], g["y_turn"], g["z"], rgba, flag=-s * FLAG_W)
                self._pole(sc, f"gate{g['i']}_out", g["x"], g["y_out"], g["z"], rgba, flag=s * FLAG_W)
        fx, fz = self.finish()
        hw = p["finish_width_m"] / 2
        for tag, y in (("L", hw), ("R", -hw)):        # the finish posts and the banner (looks)
            sc.geom(sc.wb, f"finish_post{tag}", type="cylinder", pos=(fx, y, fz + 1.5), size=(0.08, 1.5),
                    rgba=(0.2, 0.2, 0.22, 1), contype=0, conaffinity=0)
        sc.geom(sc.wb, "finish_banner", type="box", pos=(fx, 0, fz + 2.8), size=(0.04, hw, 0.2),
                rgba=(0.15, 0.3, 0.75, 1), contype=0, conaffinity=0)
        sc.geom(sc.wb, "finish_line", type="box", pos=(fx, 0, fz + 0.002), size=(0.04, hw, 0.002),
                quat=(math.cos(self.slope() / 2), 0, math.sin(self.slope() / 2), 0), rgba=(0.85, 0.1, 0.1, 1),
                contype=0, conaffinity=0)
        # the planks and their bindings
        for name, (pk, foot, relpos, relquat) in self.planks.items():
            add_plank(sc, name, pk, n_bind=2 if pk.stance else 1, rgba=self.plank_rgba)
        for name, (pk, foot, relpos, relquat) in self.planks.items():
            for f, rp in (foot if isinstance(foot, tuple) else ((foot, relpos),)):
                bind(sc, name, f, rp, relquat)
        cont = sc.root.find("contact")
        names = list(self.planks)
        for i, a in enumerate(names):                 # the planks hit each other and the poles
            for b2 in names[i + 1:]:
                ET.SubElement(cont, "pair", geom1=f"{a}_plate", geom2=f"{b2}_plate", condim="3")
            for pole in self._poles:
                for gn in (pole, f"{pole}_b", f"{pole}_panel") if self.gate_kind != "sl" else (pole,):
                    ET.SubElement(cont, "pair", geom1=f"{a}_plate", geom2=gn, condim="3")

    def plank_frames(self):
        """plank -> (foot body, relpos, relquat) of its (first) binding: where place_on_slope puts it."""
        out = {}
        for name, (pk, foot, relpos, relquat) in self.planks.items():
            if isinstance(foot, tuple):
                f, rp = foot[0]
                out[name] = (f, rp, relquat)
            else:
                out[name] = (foot, relpos, relquat)
        return out

    def reset(self, w):
        self.place(w)

    def place(self, w):
        place_on_slope(w, self.slope(), self.pr.z_at, self.plank_frames(), legs=self.start_legs(w))

    def start_legs(self, w):
        return None

    # ------------------------------------------------------------------ the referee
    def referee_reset(self, w, ref):
        m = w.m
        self.ss = SnowModel(w, {n: v[0] for n, v in self.planks.items()}, self.snow)
        self.plank_body = {n: m.body(n).id for n in self.planks}
        self.pole_mask = np.zeros(m.ngeom, bool)
        for pole in self._poles:
            self.pole_mask |= ref.geoms(body=pole)
        self.snow_mask = ref.geoms(names=self.snow)
        self.robot_mask = ref.part("robot")
        self.gl = self.gates()
        G = len(self.gl)
        self.trackers = {
            # per gate, per tracked point: None (not yet crossed downwards), True (its latest downward crossing was
            # inside the gate line), False (outside)
            "cross": [[None] * len(self.track) for _ in range(G)],
            "prev": None, "passed": 0, "slow_k": 0, "started": False, "snow": Digest(w, list(self.plank_body.values())
                                                                                    + list(self.ss.drag_b)),
        }
        self.ss.apply(w.d)                            # the forces of the first physics step, from the state at t = 0

    def points(self, w):
        """The tracked points now: the skis' tips (the front ends of their shovels) and the feet (ICR 661.4.1)."""
        d = w.d
        out = []
        for name, (pk, foot, relpos, relquat) in self.planks.items():
            b = self.plank_body[name]
            R = d.xmat[b].reshape(3, 3)
            out.append(d.xpos[b] + R[:, 0] * pk.tip_x())
        if len(out) == 1:                             # a snowboard: its nose and its tail
            name, (pk, *_) = next(iter(self.planks.items()))
            b = self.plank_body[name]
            R = d.xmat[b].reshape(3, 3)
            out.append(d.xpos[b] - R[:, 0] * pk.tip_x())
        for s in ("left", "right"):
            out.append(d.site_xpos[w.robot.foot_site[s]].copy())
        return np.array(out)

    def height(self, w, p) -> float:
        """Height of a point above the snow, along the surface's normal."""
        a = self.pr.slope_at(float(p[0]))
        return (float(p[2]) - float(self.pr.z_at(float(p[0])))) * math.cos(a)

    def substep(self, w, ref):
        self.ss.apply(w.d)
        tr = self.trackers
        P = self.points(w)
        prev, tr["prev"] = tr["prev"], P[:, :2].round(9).tolist()
        if prev is None or ref.failed:
            return
        prev = np.asarray(prev)
        fx, _ = self.finish()
        n_passed = tr["passed"]
        for k in range(len(self.gl) + 1):             # the gate lines, then the finish line (x = const planes)
            x = self.gl[k]["x"] if k < len(self.gl) else fx
            for j in range(len(P)):
                x0, x1 = prev[j, 0], P[j, 0]
                if (x0 < x) == (x1 < x):
                    continue
                f = (x - x0) / (x1 - x0)
                y = prev[j, 1] + f * (P[j, 1] - prev[j, 1])
                down = x1 >= x
                if k == len(self.gl):                 # the finish line
                    continue
                g = self.gl[k]
                if not down:                          # back up through it: that point's crossing no longer counts
                    tr["cross"][k][j] = None
                    continue
                if k > n_passed:                      # beyond a gate that has not been passed (ICR 614.2.2)
                    return ref.fail("missed a gate", gate=n_passed + 1,
                                    points=[self.track[i] for i, c in enumerate(tr["cross"][n_passed]) if c is not True])
                tr["cross"][k][j] = bool(g["lo"] <= y <= g["hi"])
        while n_passed < len(self.gl) and all(c is True for c in tr["cross"][n_passed]):
            n_passed += 1
            tr["passed"] = n_passed
            ref.stage(f"gate {n_passed}")
        ref.metrics["gates"] = n_passed
        # the finish: both feet over the finish line between its posts, every gate passed
        feet = P[-2:]
        if all(f[0] >= fx for f in feet):
            if n_passed < len(self.gl):
                return ref.fail("missed a gate", gate=n_passed + 1,
                                points=[self.track[i] for i, c in enumerate(tr["cross"][n_passed]) if c is not True])
            if all(abs(f[1]) <= self.p["finish_width_m"] / 2 for f in feet):
                ref.metrics["time_s"] = round(ref.time, 3)
                if ref.stage("finish", time_s=round(ref.time, 3)):
                    ref.succeed()
            else:
                ref.fail("missed the finish", y=[round(float(f[1]), 2) for f in feet])

    def step(self, w, ref):
        p, tr = self.p, self.trackers
        if ref.failed or ref.success:
            return
        bad = ref.parts_touching(self.snow_mask) - set(self.legal_ground)
        pel = w.d.xpos[w.robot.pelvis]
        h = self.height(w, pel)
        if bad:
            return ref.fail("fell", on_the_snow=sorted(bad))
        if h < p["fall_h_m"]:
            return ref.fail("fell", pelvis_height_m=round(h, 3))
        v = float(np.linalg.norm(w.d.qvel[w.robot.fv:w.robot.fv + 3]))
        ref.metrics["speed_m_s"] = round(v, 3)
        if v > p["stop_speed_m_s"]:
            tr["started"] = True
        tr["slow_k"] = tr["slow_k"] + 1 if v < p["stop_speed_m_s"] else 0
        if tr["slow_k"] * w.CONTROL_DT >= p["stop_s"] and w.t >= p["stop_s"]:
            return ref.fail("stopped", speed_m_s=round(v, 3))

    def privileged(self, w):
        pel = w.d.xpos[w.robot.pelvis]
        return {"pelvis": w.pelvis().tolist(), "pelvis_height_m": self.height(w, pel), "slope_deg": self.p["slope_deg"],
                "gates": [{k: g[k] for k in ("i", "x", "z", "colour", "y_turn", "y_out")} for g in self.gl],
                "gates_passed": self.trackers.get("passed", 0), "finish_x": self.finish()[0],
                "planks": {n: {"pos": w.d.xpos[b].tolist(), "xmat": w.d.xmat[b].tolist()}
                           for n, b in self.plank_body.items()},
                "points": dict(zip(self.track, self.points(w).tolist())), "snow_model": dict(self.ss.last)}
# ====================================================================================== END alpine-race block

G1_SOLE_Z, G1_FOOT_X = -0.035, 0.035         # the G1's sole below its ankle-roll link, its foot's middle ahead of it
# Atomic Redster G9 FIS J-RP2 138 (a junior FIS giant-slalom race ski; the maker's data: 138 cm, 100 / 65 / 83.5 mm,
# sidecut radius 14.5 m), the length a 1.32 m junior racer takes for GS (head height); its mass with the race plate
# (1.75 kg) and its binding's (0.85 kg), the shovel and tail and the mounting point are estimates (after RoboOlympics'
# freeski "race" ski); the stand height is within the 50 mm of the FIS Specifications for Competition Equipment.
GS_SKI = Plank("Atomic Redster G9 FIS J-RP2 138 cm", 1.38, 0.100, 0.065, 0.0835, 14.5, 1.75, -0.06, 0.85, 0.045,
               shovel=0.15, tail=0.05, tip_rise=0.055, tail_rise=0.012)


def ski_relpos(pk):
    """The ski's frame in the ankle-roll link's frame (the binding)."""
    return (G1_FOOT_X - pk.mount, 0.0, G1_SOLE_Z - pk.stand_h)


class Task(AlpineRace):
    name = "giant-slalom"
    title = "Alpine skiing: giant slalom"
    gate_kind = "gs"
    plank_rgba = (0.10, 0.10, 0.12, 1)
    planks = {f"{s}_ski": (GS_SKI, f"{s}_ankle_roll_link", ski_relpos(GS_SKI), (1, 0, 0, 0)) for s in ("left", "right")}
    rulebook = ("FIS International Competition Rules (ICR), Book IV, Alpine, edition 2025: Art. 614.2 (gates, no "
                "continuing after a missed gate or a stop), 661.4.1 and 661.4.1.1 (a gate is passed when both ski "
                "tips and both feet cross the gate line, the shortest line on the snow between the inner poles of "
                "its two pole pairs), 680 (flex turning poles), 690 and 901.2.2 (panels of about 75 x 50 cm, their "
                "lower edge at least 1 m above the snow), 901 (giant slalom technical data: a gate of four poles and "
                "two panels, alternately red and blue, 4-8 m wide, at least 10 m between the nearest poles of "
                "consecutive gates, direction changes 11-15 % of the vertical drop), 902-903 (the course)")
    deviations = [
        "ICR 901.1 (vertical drop 250-450 m, U16-U14 200-350 m): the course is {course_m} m long on a slope of "
        "{slope_deg} degrees with {n_gates} gates, a small fraction of the official drop, so that the robot's run "
        "fits the episode; 901.2.4's direction changes (11-15 % of the vertical drop) are not kept: they would "
        "allow only one change of direction on this drop, so the course keeps the gate spacing and widths instead.",
        "ICR 902 (the terrain, a course about 40 m wide): a uniform groomed slope of {slope_deg} degrees, "
        "{course_width_m} m wide.",
        "ICR 903 (the course setter's free choice of long, medium and short turns): open gates at a regular "
        "{gate_spacing_m} m, their turning poles alternately {offset_m} m left and right of the centre line.",
        "ICR 805 / 905 (the start: the racer pushes out of the start gate, the start wand starts the clock): the "
        "robot stands still on the slope at the start and is released at t = 0; the run's time is the episode's.",
        "ICR 614.2.4 and 628 (a racer who falls may get up and go on): a fall ends the run here (the families' fall "
        "rule): a part of the robot other than its feet and its hands on the snow, or its pelvis less than "
        "{fall_h_m} m above the snow.",
        "Snow and air: the ski-snow contact is the freeski model of the CHIGUI0/roboolympics project (MIT, ported): kinetic friction along the "
        "plank after Wolfsperger et al. 2021, the edge's hold across it after Mossner et al. 2013 with race-piste "
        "snow, the sidecut's arc after Howe 2001; the snow keeps no ruts. Still air at 1500 m (1.052 kg/m^3), "
        "drag on every link; no lift.",
        "Equipment: no ski poles (the robot's hands are bare pads that cannot hold one), no ski boots (the feet are "
        "bound directly); the binding is rigid and never releases; the skis are rigid.",
        "Gates: each pole pair and its panel is one body on a sprung two-axis hinge at the snow (a real pair has a "
        "flex hinge per pole and a panel that tears off in a collision; ICR 690.1); stiffness and mass are "
        "estimates.",
    ]
    time_limit_s = 60.0
    stages = [f"gate {i + 1}" for i in range(4)] + ["finish"]
    params = dict(slope_deg=8.0, brink_radius_m=6.0, outrun_radius_m=30.0, outrun_m=50.0, course_width_m=40.0,
                  n_gates=4, first_gate_m=12.0, gate_spacing_m=22.0, offset_m=2.0, gate_width_m=5.0, first_side=1.0,
                  course_m=90.0, finish_width_m=14.0, fall_h_m=0.35, stop_speed_m_s=0.10, stop_s=1.5)
    sentence = ("Ski the giant slalom: one run down the course on your skis, both ski tips and both feet through each "
                "of the {n_gates} gates in order, then over the finish line.")
    rules = ("FIS alpine giant slalom, one run (a shortened course). The course is a groomed slope falling at "
             "{slope_deg} degrees along +x (the fall line), {course_width_m} m wide, centred on y = 0. Each of your "
             "feet is bound to a giant-slalom ski. You stand still at the start at t = 0 and are released. There are "
             "{n_gates} open gates: gate 1's line is {first_gate_m} m down the slope from your start (measured along "
             "the slope), each next one {gate_spacing_m} m further. A gate is two pairs of flexible poles across the "
             "slope, each pair joined by a panel, red and blue in turn: the inner pole of its turning pair stands "
             "{offset_m} m from the centre line, on the left (y > 0) for gate 1 and then alternately right and left; "
             "the inner pole of its outside pair stands {gate_width_m} m further out on the same side. Its gate line "
             "is the line on the snow between the two inner poles. A gate is passed when both ski tips (the front "
             "ends of the skis) and both feet have crossed its gate line going down the slope; a point that comes "
             "back up across a gate line must cross it again. Gates are passed in order: if any of those four points "
             "goes down past the next gate's line, or the finish line, while a gate is not passed, you missed it and "
             "the run fails. Hitting a pole or a panel is allowed. The finish line is {course_m} m down the slope "
             "from the start, {finish_width_m} m wide and centred on y = 0: you succeed when, after every gate, both "
             "feet cross it between its posts, within {time_limit_s} s. The run fails if you fall (a part of you "
             "other than your feet and your hands touches the snow, or your pelvis is less than {fall_h_m} m above "
             "the snow, measured along the slope's normal), if you stop (your pelvis slower than {stop_speed_m_s} "
             "m/s for {stop_s} s; ICR 614.2.3), if a foot crosses the finish line outside its posts, or at "
             "{time_limit_s} s.")
    venue = ("You stand at the start, x = 0, y = 0, facing down the fall line (+x), perpendicular to the slope, on "
             "your skis (138 cm junior giant-slalom skis, sidecut radius 14.5 m). Behind you is a short flat "
             "platform; the slope falls at {slope_deg} degrees for the whole course and flattens out after the "
             "finish into an outrun. The gates' poles and panels are red and blue, the poles 1.8 m out of the snow; "
             "the finish has two posts and a banner.")
    difficulty = ("Giant-slalom skis carve long arcs: to turn, the robot must roll them onto their edges and lean its "
                  "body into the turn so that the snow's force passes through its centre of mass, with no walking or "
                  "running policy to help. The gates are far apart, so speed builds between them, and a lean that "
                  "does not match the turn throws it on the snow.")

    def build(self, sc):
        self.build_course(sc)
        sc.camera("venue", (-8.0, -20.0, 12.0), (30.0, 0.0, -8.0), fovy=60)
