"""Self-contained runtime for one IKEA MuJoCo scene (copied verbatim into every scene dir).

Only depends on mujoco + numpy. Provides:
  * Scene: model/data loading at the 'home' keyframe, step(), mechanism + success hooks
  * RetentionMechanism: the source's post-seat retention joints (MuJoCo weld equalities that are
    switched on at runtime once the source's geometric/contact/dwell gates hold)
  * small pose / contact helpers
  * cylinder_prisms: every cylinder a robot geom can touch is compiled as a 32-facet convex prism (at every build)
  * the welded-part contact switch-off (RetentionMechanism, from the scene's sw_cutover.json; no file = off)
"""
import json, os, math, hashlib
import numpy as np
import mujoco

HERE = os.path.dirname(os.path.abspath(__file__))

# ---------------------------------------------------------------- math
def qmul(a, b):
    w, x, y, z = a; W, X, Y, Z = b
    return np.array([w*W-x*X-y*Y-z*Z, w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X, w*Z+x*Y-y*X+z*W])

def qinv(q):
    return np.array([q[0], -q[1], -q[2], -q[3]]) / np.dot(q, q)

def _cross(a, b):
    # the numpy cross product of two 3-vectors, bit-identical (the same products and differences), without its axis handling (6x
    # faster; rotate() runs ~100 times per control tick in the success monitors)
    return np.array([a[1]*b[2] - a[2]*b[1], a[2]*b[0] - a[0]*b[2], a[0]*b[1] - a[1]*b[0]])

def rotate(q, v):
    q = np.asarray(q, float); v = np.asarray(v, float)
    return v + 2*_cross(q[1:], _cross(q[1:], v) + q[0]*v)

def quat_angle_deg(a, b):
    e = qmul(np.asarray(a)/np.linalg.norm(a), qinv(np.asarray(b)/np.linalg.norm(b)))
    return math.degrees(2*math.atan2(np.linalg.norm(e[1:]), abs(e[0])))

def axis_angle_deg(u, v):
    u = np.asarray(u, float); v = np.asarray(v, float)
    return math.degrees(math.atan2(np.linalg.norm(_cross(u, v)), float(np.dot(u, v))))

# ---------------------------------------------------------------- model helpers
class Helpers:
    def __init__(self, model, data):
        self.m, self.d = model, data
        self.geom_body = model.geom_bodyid.copy()

    def bid(self, name):
        return self.m.body(name).id

    def pose(self, name):
        b = self.bid(name)
        return self.d.xpos[b].copy(), self.d.xquat[b].copy()

    def state13(self, name):
        """[pos, quat(wxyz), linear vel (world), angular vel (world)] of the body frame."""
        b = self.bid(name)
        v = np.zeros(6)
        mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_BODY, b, v, 0)
        return np.r_[self.d.xpos[b], self.d.xquat[b], v[3:], v[:3]]

    def rel_pose(self, host, part):
        ph, qh = self.pose(host); pp, qp = self.pose(part)
        qi = qinv(qh)
        return rotate(qi, pp - ph), qmul(qi, qp)

    def contact_normal_forces(self):
        """(body1, body2, normal force N) arrays for all active contacts."""
        n = self.d.ncon
        if n == 0:
            return np.zeros(0, int), np.zeros(0, int), np.zeros(0)
        c = self.d.contact
        g1 = c.geom1[:n]; g2 = c.geom2[:n]; adr = c.efc_address[:n]
        f = np.where(adr >= 0, self.d.efc_force[np.maximum(adr, 0)], 0.)
        return self.geom_body[g1], self.geom_body[g2], f

    def contact_force_between(self, a, b_list):
        """Sum of contact normal force magnitudes between body a and any body in b_list."""
        b1, b2, f = self.contact_normal_forces()
        ia = self.bid(a); ib = np.array([self.bid(x) for x in b_list])
        sel = ((b1 == ia) & np.isin(b2, ib)) | ((b2 == ia) & np.isin(b1, ib))
        return float(np.abs(f[sel]).sum())

    def contact_force_vector(self, a, b_list):
        """Net contact force (world) applied to body a by bodies in b_list."""
        n = self.d.ncon; total = np.zeros(3)
        ia = self.bid(a); ib = set(self.bid(x) for x in b_list)
        f6 = np.zeros(6)
        for i in range(n):
            c = self.d.contact[i]
            x1, x2 = self.geom_body[c.geom1], self.geom_body[c.geom2]
            if (x1 == ia and x2 in ib) or (x2 == ia and x1 in ib):
                mujoco.mj_contactForce(self.m, self.d, i, f6)
                fw = c.frame.reshape(3, 3).T @ f6[:3]   # force on geom2 from geom1, world
                total += fw if x2 == ia else -fw
        return total

# ---------------------------------------------------------------- welded-part contact switch-off
WELD_GROUP_BIT0 = 16              # collision bits 16.. mark the welded groups (no scene geom uses them)


def contact_cutover(scene_dir, ops):
    """The scene's sw_cutover.json {"from_op": <1-based op number or op key, or null>, "from_t": <s or null>}: the welded-part
    contact switch-off is on from the start of that operation (every earlier operation latched and, if from_t is given, sim
    time >= from_t). No file, or from_op null: off, and the mechanism never touches the model's collision bits. The scene
    reads nothing from the environment. Returns (0-based op index or None, time or None)."""
    p = os.path.join(scene_dir, 'sw_cutover.json')
    if not os.path.exists(p):
        return None, None
    cfg = json.load(open(p))
    op, t = cfg.get('from_op'), cfg.get('from_t')
    if op in (None, '', 'none', 'off'):
        return None, None
    keys = [o['key'] for o in ops]
    i = keys.index(op) if op in keys else int(op) - 1
    assert 0 <= i < len(ops), op
    return i, (None if t in (None, '') else float(t))


# ---------------------------------------------------------------- retention mechanism
class RetentionMechanism:
    """Source-defined post-seat retention joints, one weld equality per operation.

    Each operation from task.json['operations'] with a 'weld' entry owns an (initially inactive)
    weld equality named weld['eq']. The weld is switched on with the CURRENT relative pose once the
    operation's gates (pose error to the assembled target, relative speed, counterpart contact,
    optional thread turns and extra source conditions) hold continuously for the dwell time.
    Welds are never switched off by the mechanism (the source has no disassembly); reset() clears.

    Welded-part contact switch-off (speed; on only where the scene has a sw_cutover.json, contact_cutover()): from the
    cut-over on, the bodies joined by latched welds (each operation's part and host, transitively) form one rigid group,
    and contacts between two bodies of the same group are off: the group's geoms get contype = the group's own bit and
    conaffinity = their own bits plus the other groups' bits, so they still collide with everything else exactly as
    before (checked at the build: the scene's collision bits must make that exact, or the build fails). A scene may switch
    more geoms off (contact_off_geoms(), e.g. a seated screw). The bits follow the weld state (engage, reset, set_state),
    so every replay and every restored state reproduces them. (A subclass that does not call this __init__ has no
    cut-over: the class defaults below keep the switch-off off.)
    """
    sw_from = None
    sw_from_t = None
    sw_active = False

    def __init__(self, model, data, task, period=4, scene_dir=HERE):
        self.m, self.d, self.task = model, data, task
        self.h = Helpers(model, data)
        self.period = period
        self.ops = [o for o in task['operations'] if o.get('weld')]
        self.eq = {o['key']: model.equality(o['weld']['eq']).id for o in self.ops}
        self.sw_from, self.sw_from_t = contact_cutover(scene_dir, self.ops)
        self._prepare_weld_contacts()
        self.reset()

    # -- welded-part contact switch-off
    def _prepare_weld_contacts(self):
        m = self.m
        self.sw_active = False
        if self.sw_from is None:
            return
        self.sw_orig = (m.geom_contype.copy(), m.geom_conaffinity.copy())
        self.sw_body_orig = (m.body_contype.copy(), m.body_conaffinity.copy())
        self.sw_pairs = [(m.body(o['part']).id, m.body(o['host']).id, self.eq[o['key']]) for o in self.ops]
        ct, ca = self.sw_orig
        live = (ct | ca) != 0
        if ((ct | ca) >> WELD_GROUP_BIT0).any():
            raise ValueError('welded-part contact switch-off: a geom uses collision bits >= %d' % WELD_GROUP_BIT0)
        members = {b for p in self.sw_pairs for b in p[:2]}
        cls = {(int(ct[g]), int(ca[g])) for g in np.flatnonzero(live)}
        own = {(int(ct[g]), int(ca[g])) for g in np.flatnonzero(live) if int(m.geom_bodyid[g]) in members}
        hit = lambda a, b: bool((a[0] & b[1]) | (b[0] & a[1]))
        for e in own:      # exactness: a group geom vs a non-group geom collides iff (its ct & our ca); two groups always
            if any(hit(e, o) != bool(o[0] & e[1]) for o in cls) or not all(hit(e, f) for f in own):
                raise ValueError('welded-part contact switch-off: the collision bits %s do not allow an exact switch-off' % (e,))
        self.sw_geoms = {b: np.flatnonzero(live & (m.geom_bodyid == b)) for b in members}

    def sw_check(self):
        """Cut-over reached: every operation before sw_cutover's from_op latched and sim time >= its from_t (if given)."""
        if self.sw_from is None:
            return False
        if self.sw_from_t is not None and self.d.time < self.sw_from_t - 1e-7:
            return False
        return bool(all(self.d.eq_active[self.eq[o['key']]] for o in self.ops[:self.sw_from]))

    def sw_poll(self):
        if self.sw_from is not None and not self.sw_active and self.sw_check():
            self.sync_contacts()

    def contact_off_geoms(self):
        """Geoms the scene switches off entirely once the cut-over is reached (none here; e.g. kyrre's seated screws)."""
        return ()

    def weld_groups(self):
        """The rigid groups of latched welds: sorted lists of body ids (2 bodies or more), ordered by their first body."""
        root = {}
        def find(x):
            while root.get(x, x) != x:
                x = root[x]
            return x
        for p, h, e in self.sw_pairs:
            if self.d.eq_active[e]:
                a, b = find(p), find(h)
                if a != b:
                    root[max(a, b)] = min(a, b)
        comp = {}
        for p, h, e in self.sw_pairs:
            for b in (p, h):
                comp.setdefault(find(b), set()).add(b)
        return sorted((sorted(c) for c in comp.values() if len(c) > 1), key=lambda c: c[0])

    def sync_contacts(self):
        """The model's collision bits from the weld state (a no-op where the scene has no cut-over)."""
        if self.sw_from is None or not hasattr(self, 'sw_orig'):
            return
        m = self.m; ct, ca = self.sw_orig
        self.sw_active = self.sw_check()
        nct, nca = ct.copy(), ca.copy()
        if self.sw_active:
            groups = self.weld_groups()
            if len(groups) > 31 - WELD_GROUP_BIT0:
                raise ValueError('welded-part contact switch-off: more than %d groups' % (31 - WELD_GROUP_BIT0))
            bits = [1 << (WELD_GROUP_BIT0 + k) for k in range(len(groups))]
            allbits = sum(bits)
            for grp, bit in zip(groups, bits):
                g = np.concatenate([self.sw_geoms[b] for b in grp])
                nct[g] = bit
                nca[g] = ca[g] | (allbits & ~bit)
            off = np.asarray(list(self.contact_off_geoms()), dtype=np.int64)
            nct[off] = 0
            nca[off] = 0
        m.geom_contype[:] = nct
        m.geom_conaffinity[:] = nca
        # MuJoCo's body-pair pre-filter reads body_contype / body_conaffinity (OR over the body's geoms): keep a superset
        bct, bca = self.sw_body_orig[0].copy(), self.sw_body_orig[1].copy()
        np.bitwise_or.at(bct, m.geom_bodyid, nct)
        np.bitwise_or.at(bca, m.geom_bodyid, nca)
        m.body_contype[:] = bct
        m.body_conaffinity[:] = bca

    def reset(self):
        self.dwell = {o['key']: 0. for o in self.ops}
        self.turns = {o['key']: 0. for o in self.ops}
        self.prev_q = {}
        self.engaged_at = {}
        self.metrics = {}
        self.counter = 0
        for o in self.ops:
            self.d.eq_active[self.eq[o['key']]] = 0
        self.sync_contacts()

    def active(self, key):
        return bool(self.d.eq_active[self.eq[key]])

    def state(self):
        return dict(dwell=dict(self.dwell), turns=dict(self.turns), engaged_at=dict(self.engaged_at),
                    prev_q={k: v.tolist() for k, v in self.prev_q.items()}, counter=self.counter,
                    eq_data={k: self.m.eq_data[i].tolist() for k, i in self.eq.items()},
                    eq_active={k: bool(self.d.eq_active[i]) for k, i in self.eq.items()})

    def set_state(self, s):
        self.dwell = dict(s['dwell']); self.turns = dict(s['turns']); self.engaged_at = dict(s['engaged_at'])
        self.prev_q = {k: np.array(v) for k, v in s['prev_q'].items()}; self.counter = s['counter']
        for k, i in self.eq.items():
            self.m.eq_data[i] = s['eq_data'][k]; self.d.eq_active[i] = int(s['eq_active'][k])
        self.sync_contacts()

    # -- geometry of one operation (all in world frame)
    def measure(self, o):
        h = self.h
        sp = h.state13(o['part']); sh = h.state13(o['host'])
        qh = sh[3:7]
        tgt_p = sh[:3] + rotate(qh, o['target_pos'])          # part frame origin at assembled pose
        tgt_q = qmul(qh, o['target_quat'])
        axis = rotate(qh, o['axis'])                            # outward / insertion axis (world)
        # error measured at reference points on the part (connectors) if given, else at the origin
        pts = o.get('points') or [[0, 0, 0]]
        gaps, lats = [], []
        for p in pts:
            a = sp[:3] + rotate(sp[3:7], p); b = tgt_p + rotate(tgt_q, p)
            dl = a - b; g = float(dl @ axis); gaps.append(g); lats.append(float(np.linalg.norm(dl - g*axis)))
        gap = max(gaps, key=abs); lat = max(lats)
        if o.get('tilt', 'full') == 'axis':
            ang = axis_angle_deg(rotate(sp[3:7], o['part_axis']), rotate(tgt_q, o['part_axis']))
        else:
            ang = quat_angle_deg(sp[3:7], tgt_q)
        # relative linear speed of the part w.r.t. the host material point at the part origin
        vh = sh[7:10] + _cross(sh[10:13], sp[:3] - sh[:3])
        speed = float(np.linalg.norm(sp[7:10] - vh))
        # relative spin about the axis (for thread turns)
        qrel = qmul(qinv(qh), sp[3:7])
        return dict(gap=gap, lateral=lat, angle=ang, speed=speed, qrel=qrel, axis_host=np.asarray(o['axis'], float))

    def update(self):
        self.sw_poll()
        self._update_welds()
        self.sw_poll()

    def _update_welds(self):
        self.counter += 1
        if self.counter % self.period:
            return
        dt = self.m.opt.timestep * self.period
        for o in self.ops:
            k = o['key']
            if self.active(k):
                continue
            g = o['gates']; me = self.measure(o)
            if o.get('count_turns'):
                q = me['qrel']
                if k in self.prev_q:
                    dq = qmul(q, qinv(self.prev_q[k]))
                    if dq[0] < 0: dq = -dq
                    # rotation increment about the host axis (sign: o['turn_sign'] tightening direction)
                    self.turns[k] += 2*math.atan2(float(dq[1:] @ me['axis_host']), dq[0]) * o.get('turn_sign', 1) / (2*math.pi)
                self.prev_q[k] = q
            contact = self.h.contact_force_between(o['part'], o['counterparts']) if o.get('counterparts') else 1e9
            deps_ok = all(self.active(x) for x in o.get('dependencies', []) if x in self.eq)
            ok = (deps_ok and abs(me['gap']) < g['axial_m'] and me['lateral'] < g['lateral_m'] and me['angle'] < g['axis_deg']
                  and me['speed'] < g['speed_m_s'] and contact > g.get('min_contact_N', -1))
            if o.get('min_turns') is not None:
                ok = ok and self.turns[k] >= o['min_turns']
            extra = getattr(self, 'extra_' + o.get('extra', 'none'), None)
            if ok and extra is not None:
                ok = extra(o, me)
            self.dwell[k] = self.dwell[k] + dt if ok else 0.
            self.metrics[k] = dict(gap_m=me['gap'], lateral_m=me['lateral'], angle_deg=me['angle'], speed_m_s=me['speed'],
                                   contact_N=contact if contact < 1e8 else None, turns=self.turns[k], dwell_s=self.dwell[k])
            if self.dwell[k] >= g['dwell_s']:
                self.engage(o)

    def engage(self, o):
        i = self.eq[o['key']]
        r, q = self.h.rel_pose(o['host'], o['part'])
        self.m.eq_data[i, 0:3] = 0; self.m.eq_data[i, 3:6] = r; self.m.eq_data[i, 6:10] = q; self.m.eq_data[i, 10] = 1
        self.d.eq_active[i] = 1
        self.engaged_at[o['key']] = float(self.d.time)
        self.sync_contacts()

# ---------------------------------------------------------------- cylinder -> prism
PRISM_FACETS = 32

def cylinder_prisms(spec):
    """Every cylinder geom outside the robot that a robot geom can collide with (collision bits, default classes resolved)
    becomes a convex prism of PRISM_FACETS facets with its vertices on the radius (vertex k at angle 2 pi k / PRISM_FACETS
    from the geom's local x axis, the axis its local z; a fromto cylinder takes the shortest rotation of z onto its axis,
    MuJoCo's own fromto frame), the same pose, and the cylinder's mass (density x pi r^2 x length) unless the geom states
    its own. MuJoCo has no analytic cylinder test against a convex hull: its GJK / EPA can return, near a flat cap's rim,
    the right depth with a sign-flipped normal, and the row pushes nothing; a prism collides as a polytope. The same rule
    as the benchmark's other MuJoCo scenes, applied here to the MjSpec at every build, so scene.xml (and its keyframe)
    stay as they are. The robot: every body tree under the world that carries an actuated joint (rcb_ikea.robot_bodies).
    Edits spec in place; returns the audit [{geom, body, r_m, half_length_m}]."""
    def root(body):
        while body.parent is not None and body.parent.name != spec.worldbody.name:
            body = body.parent
        return body.name

    roots = set()
    for a in spec.actuators:
        if a.trntype in (mujoco.mjtTrn.mjTRN_JOINT, mujoco.mjtTrn.mjTRN_JOINTINPARENT):
            roots.add(root(spec.joint(a.target).parent))
        elif a.trntype == mujoco.mjtTrn.mjTRN_TENDON:
            for w in spec.tendon(a.target).path:
                if w.type == mujoco.mjtWrap.mjWRAP_JOINT:
                    roots.add(root(spec.joint(w.target.name).parent))
    world = spec.worldbody.name
    in_robot = lambda g: g.parent.name != world and root(g.parent) in roots
    robot_bits = {(int(g.contype), int(g.conaffinity)) for g in spec.geoms if in_robot(g)}
    n = PRISM_FACETS
    ang = 2 * np.pi * np.arange(n) / n
    faces = [[k, (k + 1) % n, n + k] for k in range(n)] + [[(k + 1) % n, n + (k + 1) % n, n + k] for k in range(n)]
    faces += [[0, k + 1, k] for k in range(1, n - 1)] + [[n, n + k, n + k + 1] for k in range(1, n - 1)]
    faces = np.asarray(faces, dtype=np.int64)
    meshes, audit = {}, []
    for g in list(spec.geoms):
        if g.type != mujoco.mjtGeom.mjGEOM_CYLINDER or in_robot(g):
            continue
        ct, ca = int(g.contype), int(g.conaffinity)
        if not any((ct & ra) or (rt & ca) for rt, ra in robot_bits):
            continue
        r = float(g.size[0])
        if not np.isnan(g.fromto[0]):
            p0, p1 = np.asarray(g.fromto[:3], float), np.asarray(g.fromto[3:], float)
            pos, axis, h = (p0 + p1) / 2, p1 - p0, float(np.linalg.norm(p1 - p0)) / 2
            q = np.zeros(4)
            mujoco.mju_quatZ2Vec(q, axis / np.linalg.norm(axis))
        else:
            if g.alt.type != mujoco.mjtOrientation.mjORIENTATION_QUAT:
                raise NotImplementedError(f"cylinder prism: {g.name} states an alternative orientation")
            h = float(g.size[1])
            pos, q = np.array(g.pos, dtype=float), np.array(g.quat, dtype=float)
        v = np.array([[r * np.cos(t), r * np.sin(t), z] for z in (-h, h) for t in ang])
        if np.isnan(g.mass):
            g.mass = float(g.density) * np.pi * r * r * 2 * h
        key = hashlib.sha256(v.tobytes() + faces.tobytes()).hexdigest()[:16]
        if key not in meshes:
            mesh = spec.add_mesh()
            mesh.name = f"{g.name or 'cyl%d' % len(audit)}_prism{n}_{key}"
            mesh.uservert = v.reshape(-1)
            mesh.userface = faces.reshape(-1)
            meshes[key] = mesh.name
        g.type = mujoco.mjtGeom.mjGEOM_MESH
        g.meshname = meshes[key]
        g.fromto = np.full(6, np.nan)
        g.size = np.zeros(3)
        g.pos, g.quat = pos, q
        audit.append(dict(geom=g.name, body=g.parent.name, r_m=r, half_length_m=h))
    return audit

# ---------------------------------------------------------------- scene wrapper
class Scene:
    """Generic per-scene wrapper (scene.py in each directory calls load()).

    scene.model / scene.data : MuJoCo model and data (arms: left_*/right_*, parts: task.json['parts'])
    scene.step(n)            : n physics steps; the source mechanism and the success monitor update
    scene.success()          : dict with 'success' (bool) and per-criterion details (success.py)
    scene.get_state()/set_state(s) : full integration state incl. mechanism and monitor state
    scene.cylinder_prisms    : the cylinder geoms compiled as prisms (cylinder_prisms(); [] when none)
    """
    MECHANISM = RetentionMechanism

    def __init__(self, scene_dir=HERE, keyframe='home', mechanism=None, monitor=None):
        self.dir = scene_dir
        self.task = json.load(open(os.path.join(scene_dir, 'task.json')))
        spec = mujoco.MjSpec.from_file(os.path.join(scene_dir, 'scene.xml'))
        self.cylinder_prisms = cylinder_prisms(spec)
        self.model = spec.compile()
        self.data = mujoco.MjData(self.model)
        self.keyframe = keyframe
        mech_cls = mechanism if mechanism is not None else self.MECHANISM
        self.mech = mech_cls(self.model, self.data, self.task) if mech_cls else None
        self.monitor = monitor(self.model, self.data, self.task, self) if monitor else None
        self.reset()

    def reset(self):
        k = self.model.key(self.keyframe).id
        mujoco.mj_resetDataKeyframe(self.model, self.data, k)
        if self.mech: self.mech.reset()
        mujoco.mj_forward(self.model, self.data)
        if self.monitor: self.monitor.reset()

    def step(self, n=1):
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)
            if self.mech: self.mech.update()
            if self.monitor: self.monitor.update()

    def success(self):
        return self.monitor.report() if self.monitor else {}

    def get_state(self):
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        s = np.zeros(mujoco.mj_stateSize(self.model, spec))
        mujoco.mj_getState(self.model, self.data, s, spec)
        return dict(physics=s, mechanism=self.mech.state() if self.mech else None,
                    monitor=self.monitor.state() if self.monitor else None)

    def set_state(self, st):
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        mujoco.mj_setState(self.model, self.data, st['physics'], spec)
        if self.mech and st['mechanism'] is not None: self.mech.set_state(st['mechanism'])
        mujoco.mj_forward(self.model, self.data)
        if self.monitor and st.get('monitor') is not None: self.monitor.set_state(st['monitor'])
