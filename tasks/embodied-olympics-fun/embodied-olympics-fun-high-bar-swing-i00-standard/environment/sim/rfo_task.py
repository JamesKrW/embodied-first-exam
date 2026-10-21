"""High bar (FIG): from standing on a mounting block under the FIG horizontal bar (2.80 m high, fixed in its
uprights), jump up and catch the bar, then one giant swing: a full circle of the extended body around the bar, the
hands on the bar throughout. Nothing is welded: both hands are Unitree's Dex1-1 two-finger grippers (rfo.realhand),
whose jaws close fore and aft across the bar during the jump and hold it by friction alone; the bar can slide along
the jaws and out of them.

THE GRIP FORCE (declared deviation; measured in the simulator image): the device's 120 N per jaw holds 477 N with both
grippers, 1.41 times the weight of the G1 with its grippers (34.4 kg, 338 N). A giant swing pulls at least 1420 N at
the bottom (4.2 body weights: the extended G1, centre of mass 0.83 m below the bar, 29.6 kg m^2 about it, over the top
at rest); a magic-action giant swing measured 4.75 to 5.8 body weights (100 ms means), and gymnasts measure 3.5 to 3.7
(Kopp & Reid 1980, strain gauges on the bar). Pure friction (two hands, two pads, friction 1.0) needs 355 N per jaw at
4.2 body weights, 533 N with a margin of 1.5: this family raises the Dex1-1's clamp force 4.4 times (528 N per jaw;
the jaw servo's force limit, kp and kd x 4.4 in the compiled model at reset: apply_grip_force); nothing else about the
hand changes, and the core's hand model is the same for every family. With the device's force the bar tears out of the jaws at the bottom of a swing from the
horizontal (0.18 s).

The hand on the fixed bar: the bar does not turn (as the norm's does not); a gripper turns on it only by its pads
sliding round the bar or by the wrist's joints (5 N m in pitch and yaw). The body's extension over the top is judged
from the wrists (declared), so that a gripper folding at the wrist on the bar is not a tuck.

The bar's and the venue's block (build_bar, hang, clamp_bar, jaws_on, bar_in_jaws, swing_state, BarGrip) is
toes-to-bar's, copied byte for byte (a task directory holds one module); hang() is not used here (the robot starts on
the block)."""
import math
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from rfo import realhand
from rfo.task import TaskBase

# ------------------------------------------------------------------------------------------- the bar and the hang
# (shared, identically, by high-bar-swing and toes-to-bar)
# The hanging pose of the G1's left arm (shoulder pitch, roll, yaw, elbow, wrist roll, pitch, yaw; the right arm
# mirrors the roll and the yaws): high-bar-swing's, straight up, with the wrist rolled 0.97 rad so that the gripper's
# closing axis lies along x (across the bar, horizontal) and the fingers point up along the forearm.
HANG_ARM = np.array([-3.06, 0.57, 1.0, 1.4, 0.97, 0.0, 0.0])
MIRROR = np.array([1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0])
ARMS = slice(15, 29)
MAT = 0.20                    # FIG landing mats under the bar (m)
SIDES = ("left", "right")


def build_bar(sc, height, length, diameter):
    """The horizontal bar along y at (0, 0, height) between two uprights `length` apart, over the landing mats; a
    Dex1-1 gripper on each wrist. The bar is fixed in the world, as the real one is fixed in its uprights: it does not
    turn (a hand turns round it only by its pads sliding round the bar, or by the wrist's joints). Nothing is grippable
    (no weld): the bar meets every collision geom of the robot, the grippers' jaws and bodies included. Steel on the
    pads: the pads' contact parameters win (priority 2, realhand.py); the bar's own friction 0.8 meets the hulls."""
    half, r = length / 2, diameter / 2
    grey = (0.3, 0.3, 0.33, 1)
    for s, tag in ((1, "L"), (-1, "R")):
        sc.geom(sc.wb, f"post{tag}", type="cylinder", pos=(0, s * (half + 0.03), (height + 0.06) / 2),
                size=(0.03, (height + 0.06) / 2), rgba=grey, contype="1", conaffinity="13")
        sc.box(f"post_foot{tag}", (0, s * (half + 0.03), 0.01), (0.60, 0.05, 0.01), rgba=grey)
    sc.platform("mat", (0.0, 0.0), (3.0, 1.0, MAT / 2), rgba=(0.20, 0.35, 0.60, 1), ground=True)
    sc.geom(sc.wb, "bar", type="capsule", fromto=(0, -half, height, 0, half, height), size=(r,),
            rgba=(0.85, 0.85, 0.88, 1), contype="0", conaffinity="12", friction=(0.8, 0.005, 0.0001))
    for s in SIDES:
        sc.real_hands[s] = "dex1"


def hang(w, height, bar_depth):
    """Put the robot under the bar, arms straight up, the bar between each gripper's open jaws with its axis
    `bar_depth` m from the gripper's body along the fingers; then both grippers close on it."""
    r, d, m = w.robot, w.d, w.m
    pose = np.r_[HANG_ARM, HANG_ARM * MIRROR]
    d.qpos[r.qadr[ARMS]] = pose
    r.arb.q_a[ARMS] = r.arb.q_b[ARMS] = pose
    mujoco.mj_kinematics(m, d)
    off = (2 * realhand.HOUSING[2] + bar_depth) - realhand.CENTRE_Z      # from the jaws' centre along the fingers
    pts = []
    for s in SIDES:
        k = m.site(f"{s}_jaw_centre").id
        pts.append(d.site_xpos[k] + off * d.site_xmat[k].reshape(3, 3)[:, 2])
    mid = 0.5 * (pts[0] + pts[1])
    d.qpos[r.fq] += 0.0 - mid[0]
    d.qpos[r.fq + 2] += height - mid[2]
    clamp_bar(w)


def clamp_bar(w, closure=1.0, extra_s=0.05):
    """Close both grippers on the bar with the rest of the world held still: realhand.close_on's procedure (which
    takes free bodies only; the bar is fixed in the world): the jaws close at their speed limit until both jaws of
    both hands touch the bar and the servos' setpoints are at the command, then extra_s more for the pads to take the
    clamp. The robot is let go at t = 0 (deterministic: a function of the state). Raises if a hand does not hold it."""
    hands, m, d = w.hands, w.m, w.d
    keep_q = np.ones(m.nq, dtype=bool)
    keep_v = np.ones(m.nv, dtype=bool)
    for s in SIDES:
        hands.cmd[s] = closure
        keep_q[hands.jq[s]] = keep_v[hands.jv[s]] = False
    mujoco.mj_forward(m, d)
    q0 = d.qpos.copy()
    arm0 = m.dof_armature.copy()
    m.dof_armature[keep_v] = realhand.FREEZE_ARMATURE
    bar = m.geom("bar").id
    n_close = int(math.ceil((realhand.HALF / (realhand.SPEED / 2) + 0.05) / m.opt.timestep))
    n_extra = int(round(extra_s / m.opt.timestep))
    on_since = None
    try:
        for k in range(n_close + n_extra):
            hands.substep()
            mujoco.mj_step(m, d)
            d.qpos[keep_q] = q0[keep_q]
            d.qvel[keep_v] = 0.0
            if on_since is None and all(jaws_on(w, s, bar) == (True, True) for s in SIDES) and all(
                    abs(hands.setpoint[s] - realhand.HALF * closure) < 1e-12 for s in SIDES):
                on_since = k
            if on_since is not None and k - on_since >= n_extra:
                break
    finally:
        m.dof_armature[:] = arm0
    for s in SIDES:
        d.qvel[hands.jv[s]] = 0.0
    mujoco.mj_forward(m, d)
    if on_since is None:
        raise RuntimeError("the grippers did not close on the bar")


def jaws_on(w, side, bar):
    """(jaw a touches the bar, jaw b touches the bar) now, by contact (penetrating: dist <= 0)."""
    m, d, hands = w.m, w.d, w.hands
    n = d.ncon
    if not n:
        return (False, False)
    g = d.contact.geom[:n]
    keep = d.contact.dist[:n] <= 0.0
    i = 1 + 2 * hands.sides.index(side)
    out = [False, False]
    for a, b in g[keep]:
        o = b if a == bar else a if b == bar else None
        if o is None:
            continue
        c = int(hands.jaw_code[o])
        if c in (i, i + 1):
            out[c - i] = True
    return tuple(out)


def bar_in_jaws(w, height):
    """Per hand: where the bar's axis is in the gripper (m): along the fingers from the face of the gripper's body (the
    jaws' pads run from 0 to 0.08), and off the jaws' centre line along the closing axis."""
    m, d = w.m, w.d
    out = {}
    for s in SIDES:
        k = m.site(f"{s}_jaw_centre").id
        c, R = d.site_xpos[k], d.site_xmat[k].reshape(3, 3)
        rel = np.array([0.0, c[1], height]) - c
        out[s] = (float(R[:, 2] @ rel) + realhand.CENTRE_Z - 2 * realhand.HOUSING[2], float(R[:, 0] @ rel))
    return out


def swing_state(w, height):
    """(angle of the line bar -> centre of mass from straight down, rad, positive forwards, in (-pi, pi]; its length, m)."""
    c = w.com()
    dx, dz = float(c[0]), height - float(c[2])
    return math.atan2(dx, dz), math.hypot(dx, dz)


class BarGrip:
    """The grip rules: a hand holds the bar while the bar touches both of its gripper's jaws; a break longer than
    params["off_bar_s"] is letting go (no re-grip). Only the grippers (their jaws and bodies) and the parts in
    `bar_ok` may touch the bar."""
    bar_ok: tuple = ()

    def grip_reset(self, w, ref):
        self.bar_g = ref.geoms(names=["bar"])
        self.jaws = {s: (ref.geoms(body=f"{s}_hand_jaw_a"), ref.geoms(body=f"{s}_hand_jaw_b")) for s in SIDES}
        allowed = ref.geoms(body="left_mount") | ref.geoms(body="right_mount")      # the grippers: body and jaws
        if self.bar_ok:
            allowed |= ref.part(*self.bar_ok)
        self.not_on_bar = ref.part("robot") & ~allowed
        self.trackers["on_t"] = {s: 0.0 for s in SIDES}

    def holding(self, ref):
        return [s for s in SIDES if ref.touching(self.bar_g, self.jaws[s][0]) and ref.touching(self.bar_g, self.jaws[s][1])]

    def grip_substep(self, w, ref) -> bool:
        """The grip rules on this physics step; True if one was broken (the attempt has failed)."""
        p, on = self.p, self.trackers["on_t"]
        held = self.holding(ref)
        for s in SIDES:
            if s in held:
                on[s] = round(ref.time, 4)
                continue
            gap = round(ref.time - on[s], 4)
            if gap > ref.metrics.get("grip_break_max_s", 0.0):
                ref.metrics["grip_break_max_s"] = gap   # (reported: the longest break of a hand's two-jaw contact)
            if gap > p["off_bar_s"] + 1e-9:
                ref.fail("let go of the bar", hand=s, off_s=gap)
                return True
        if ref.touching(self.not_on_bar, self.bar_g):
            ref.fail("held the bar with a part other than the grippers", parts=sorted(ref.parts_touching(self.bar_g)))
            return True
        return False
# ------------------------------------------------------------------------------- (end of the shared bar block)

def apply_grip_force(w, factor):
    """Raise the Dex1-1 grippers' clamp force by `factor` in the compiled model (the core's files unchanged): each jaw
    servo (rfo.realhand: force = clip(kp (setpoint - q) - kd dq, -F, F), a MuJoCo `general` actuator) gets kp, kd and
    the force limit F x factor, so the clamp on the bar is min(F x factor, 10 N x factor per mm that the commanded
    opening is below the bar's width): a stronger drive of the same speed. Nothing else in the hand changes (the
    setpoint's speed, the stroke, the pads). The grippers' facts (spec()["robot_facts"]) report the raised values.
    Idempotent (always from the device's values, kept at the first call); call first in every reset."""
    m, h = w.m, w.hands
    aid = np.concatenate([h.act[s] for s in h.sides])
    if not hasattr(h, "_grip0"):
        h._grip0 = (m.actuator_gainprm[aid].copy(), m.actuator_biasprm[aid].copy(), m.actuator_forcerange[aid].copy())
        facts0 = h.facts

        def facts():
            f = facts0()
            k = h._grip_factor
            f["force_limit_n"] = round(realhand.F_MAX * k, 3)
            f["servo"] = dict(f["servo"], kp_n_per_m=realhand.KP * k, kd_n_s_per_m=realhand.KV * k,
                              rule=f["servo"]["rule"].replace("10 N per mm", f"{10 * k:g} N per mm"))
            f["force_note"] = (f"this family's grippers clamp with {k:g} times the Dex1-1's 120 N (a declared "
                               "deviation of the task)")
            return f
        h.facts = facts
    g0, b0, f0 = h._grip0
    h._grip_factor = float(factor)
    m.actuator_gainprm[aid, 0] = g0[:, 0] * factor
    m.actuator_biasprm[aid, 1:3] = b0[:, 1:3] * factor
    m.actuator_forcerange[aid] = f0 * factor


def wrist_radius(w):
    """The distance from the midpoint of the two wrists (the wrist pitch joints) to the centre of mass: the body's
    extension, whatever the grippers and the wrists do on the bar."""
    d, m = w.d, w.m
    mid = 0.5 * (d.xpos[m.body("left_wrist_pitch_link").id] + d.xpos[m.body("right_wrist_pitch_link").id])
    return float(np.linalg.norm(w.com() - mid))


class Task(BarGrip, TaskBase):
    name = "high-bar-swing"
    title = "High bar: jump to the bar and giant swing"
    category = "parkour-apparatus-balance"
    robot = "unitree_g1"
    rulebook = ("FIG Apparatus Norms, men's artistic gymnastics, horizontal bar: the bar 2.80 m above the floor (2.60 m "
                "above the 0.20 m landing mats), 2.40 m between the uprights, 28 mm in diameter, fixed in the "
                "uprights; FIG Code of Points (men's artistic gymnastics), horizontal bar: the giant swing (a full "
                "circle round the bar with the body extended, in hang)")
    deviations = [
        "the grip (the hands on the bar): each hand is a two-finger gripper, Unitree's Dex1-1 (90 mm stroke, 80 mm "
        "jaws with rubber pads; sold for the G1), closed across the bar; it holds the bar by the jaws' friction "
        "alone, nothing is welded, and the bar can slide along the jaws and out of them; the bar is fixed, as the "
        "norm's is, and a hand turns on it only by its pads sliding round it or by the wrist's joints",
        "the grip force: each jaw clamps with up to {clamp_force_n} N, 4.4 times the Dex1-1's 120 N, because the "
        "device's force holds only 1.41 body weights with both hands (measured) while the bottom of a giant swing "
        "pulls 4.2 body weights or more (4.75 to 5.8 measured in this venue; gymnasts 3.5 to 3.7, Kopp & Reid 1980): "
        "533 N per jaw is that load's need with a margin of 1.5. The device's own rating is a 5 kg friction-grasp "
        "load: this hand is far stronger than any Unitree hand",
        "the grip (the hands stay on the bar): defined by contact: a hand holds the bar while the bar touches both of "
        "its gripper's jaws; after the catch a break of up to {off_bar_s} s (the pads' contact flickering under a "
        "load) is not letting go",
        "the mount: the robot (1.32 m) starts standing on a {block_top_m} m mounting block under the bar and jumps "
        "to it (a rise of about 0.15 m), because it cannot reach a 2.80 m bar from the mats (a gymnast is lifted to "
        "the bar or jumps from a springboard)",
        "FIG Code of Points (the body extended, judged by eye): the extension is measured from the wrists: whenever "
        "the centre of mass is above the bar it must be at least {min_radius_m} m from the midpoint of the wrists "
        "(nine tenths of the dead hang's 0.628 m), because the referee has positions, not a judge's eye, and the "
        "grippers may fold at the wrists on the fixed bar",
        "FIG Apparatus Norms, horizontal bar (elasticity): the bar is rigid, not the norm's elastic steel bar on "
        "tensioned cables, because the simulator has rigid bodies only (the real bar's spring helps the gymnast: this "
        "is not easier)",
        "FIG Code of Points (a routine is scored): one element is judged pass or fail, because the task is a single "
        "action",
    ]
    time_limit_s = 60.0
    start = (-0.008, 0.0, 0.0)
    stages = ["jumped", "caught the bar", "over the top", "full circle"]
    fall_rule = False                # the task's own rules judge the ground, the block and the grip
    bar_ok = ()                      # only the grippers may touch the bar
    params = dict(bar_height_m=2.80, bar_length_m=2.40, bar_diameter_m=0.028, circle_deg=360.0, min_radius_m=0.565,
                  reversal_deg=5.0, off_bar_s=0.05, block_top_m=1.0, clamp_force_n=528.0)

    sentence = ("Standing on the block under the high bar, jump up and catch the bar with both grippers, then do one "
                "giant swing: take your extended body in a full circle around the bar, without letting go.")
    rules = ("The bar is a competition horizontal bar: {bar_diameter_m} m thick, {bar_height_m} m above the floor, "
             "{bar_length_m} m long between its two uprights, along the y axis; it is fixed and does not turn. Your "
             "hands are two-finger grippers (each jaw clamps with up to {clamp_force_n} N); they hold the bar by the "
             "friction of their jaws only, so the bar can slide along the jaws and out of them. A hand holds the bar "
             "while the bar touches both of that gripper's jaws. You start standing on the block and must jump to the "
             "bar: the catch is the moment both hands hold the bar after you have left the block (holding it before "
             "leaving the block fails). Before the catch you fail if you touch the mats or the floor; you may land "
             "back on the block and jump again. After the catch, when for more than {off_bar_s} s a hand has not held "
             "the bar you have let go (whether you opened the gripper or the bar slipped out), and there is no "
             "regrip; only the grippers may touch the bar; touching the block, the uprights, the mats or the floor "
             "fails. Your swing angle is the angle between straight down and the line from the bar to your centre of "
             "mass, seen from the side. The giant swing is a full circle of that line after the catch: the angle "
             "must travel {circle_deg} degrees in one direction (forwards or backwards) without turning back; a turn "
             "back of more than {reversal_deg} degrees starts the count again from where you turned. The body must "
             "be extended over the top: whenever your centre of mass is higher than the bar it must be at least "
             "{min_radius_m} m from the midpoint of your two wrists (nine tenths of the dead hang's 0.628 m: a slight "
             "pike or arch passes, a tuck or bent arms do not); if it comes closer there, the count starts again from "
             "that point. You succeed the moment the count reaches {circle_deg} degrees. Below the bar you may bend "
             "as you like to build up the swing. You fail after {time_limit_s} s.")
    venue = ("At t = 0 you stand on a {block_top_m} m block (0.40 m along x, 0.80 m along y) under the middle of the "
             "bar, facing +x: both arms straight up, the wrists turned so that each gripper's jaws open fore and aft, "
             "the fingers pointing up, the grippers fully open (grip values 0), the bar about 0.15 m above the "
             "grippers' bodies and right above the gaps between the jaws (the jaws' centres at y = +0.207 and "
             "-0.207 m). A grip value is that gripper's closure command (0 open, 1 closed): on the 28 mm bar, about "
             "0.82 or more clamps it at the full force, below about 0.69 the jaws no longer clamp it; a full close "
             "takes 0.24 s. The bar is not a free body: the `held` flags of your state stay empty for it; the grip is "
             "judged as the rules say. The `stand` action keeps the legs in the stance; `joints` or `lowcmd` rows "
             "drive every joint. The uprights stand at the two ends of the bar, 1.2 m to either side of you; the "
             "landing mats (0.20 m) cover the floor under the bar around the block.")
    difficulty = ("A real jump: the legs must lift the whole robot about 0.15 m so that the bar enters the open jaws, "
                  "and the jaws must close in flight. Then nothing pushes you: the swing has to be pumped up from a "
                  "hang by the legs, the waist and the shoulders, against motors of 88 N m at the hips and 25 N m in "
                  "the arms, until the body goes over the top of the bar extended; the bottom of a giant swing pulls "
                  "more than four times your weight through the grippers and the arms, and the grippers slide round "
                  "the fixed bar against their own friction.")

    def build(self, sc):
        p = self.p
        build_bar(sc, p["bar_height_m"], p["bar_length_m"], p["bar_diameter_m"])
        sc.platform("block", (-0.008, 0.0), (0.20, 0.40, p["block_top_m"] / 2), rgba=(0.75, 0.55, 0.25, 1))
        sc.camera("venue", (3.6, -4.6, 2.3), (0.0, 0.0, 1.75), fovy=50)
        sc.camera("side", (1.6, -3.9, 2.3), (0.0, 0.0, 2.2), fovy=60)

    def reset(self, w):
        """Standing on the block, arms straight up in the hang pose, grippers open; the grippers' clamp force raised."""
        apply_grip_force(w, self.p["clamp_force_n"] / realhand.F_MAX)
        r, d = w.robot, w.d
        pose = np.r_[HANG_ARM, HANG_ARM * MIRROR]
        d.qpos[r.qadr[ARMS]] = pose
        r.arb.q_a[ARMS] = r.arb.q_b[ARMS] = pose
        d.qpos[r.fq + 2] += self.p["block_top_m"]

    def referee_reset(self, w, ref):
        self.robot = ref.part("robot")
        self.posts = ref.geoms(prefix="post")
        self.block = ref.geoms(names=["block"])
        ang, _ = swing_state(w, self.p["bar_height_m"])
        self.trackers = {"prev": ang, "s": 0.0, "anchor": 0.0, "ext": 0.0, "sense": 0, "best_deg": 0.0,
                         "max_deg": 0.0, "min_deg": 0.0, "on_block": False, "flown": False, "caught": False}
        self.grip_reset(w, ref)

    def substep(self, w, ref):
        tr = self.trackers
        if not tr["caught"]:
            if ref.touching(self.not_on_bar, self.bar_g):
                return ref.fail("touched the bar with a part other than the grippers",
                                parts=sorted(ref.parts_touching(self.bar_g)))
            if ref.touching(self.robot, ref.ground):
                return ref.fail("touched the ground", parts=sorted(ref.parts_touching(ref.ground)))
            held = self.holding(ref)
            on_block = ref.touching(self.robot, self.block)
            tr["on_block"] = tr["on_block"] or on_block
            if tr["on_block"] and not on_block and not held and not tr["flown"]:
                tr["flown"] = True
                ref.stage("jumped")
            if len(held) == 2:
                if not tr["flown"] or on_block:
                    return ref.fail("held the bar without jumping to it")
                tr["caught"] = True
                self.trackers["on_t"] = {s: round(ref.time, 4) for s in SIDES}
                tr["prev"] = swing_state(w, self.p["bar_height_m"])[0]
                ref.stage("caught the bar")
            return
        if self.grip_substep(w, ref):
            return
        if ref.touching(self.robot, self.posts):
            return ref.fail("touched the uprights", parts=sorted(ref.parts_touching(self.posts)))
        if ref.touching(self.robot, self.block):
            return ref.fail("touched the block after the catch", parts=sorted(ref.parts_touching(self.block)))
        if ref.touching(self.robot, ref.ground):
            return ref.fail("touched the ground", parts=sorted(ref.parts_touching(ref.ground)))

    def step(self, w, ref):
        p, tr = self.p, self.trackers
        if ref.failed or not tr["caught"]:
            return
        ang, radius = swing_state(w, p["bar_height_m"])
        step = (ang - tr["prev"] + math.pi) % (2 * math.pi) - math.pi
        tr["prev"] = ang
        s = tr["s"] = tr["s"] + step
        tr["max_deg"], tr["min_deg"] = max(tr["max_deg"], math.degrees(s)), min(tr["min_deg"], math.degrees(s))
        rev = math.radians(p["reversal_deg"])
        if tr["sense"] == 0:
            if abs(s - tr["anchor"]) > rev:
                tr["sense"] = 1 if s > tr["anchor"] else -1
                tr["ext"] = s
        elif (s - tr["ext"]) * tr["sense"] > 0:
            tr["ext"] = s
        elif (tr["ext"] - s) * tr["sense"] > rev:                 # turned back: the travel starts again there
            tr["anchor"], tr["ext"], tr["sense"] = tr["ext"], s, -tr["sense"]
        above = w.com()[2] > p["bar_height_m"]
        if above and wrist_radius(w) < p["min_radius_m"]:         # not extended over the top: the count starts again
            if abs(tr["ext"] - tr["anchor"]) > rev:
                ref.event("not extended over the bar", wrist_radius_m=round(wrist_radius(w), 3))
            tr["anchor"] = tr["ext"] = s
            tr["sense"] = 0
        travel = abs(tr["ext"] - tr["anchor"])
        tr["best_deg"] = max(tr["best_deg"], round(math.degrees(travel), 1))
        ref.metrics["travel_deg"] = tr["best_deg"]
        ref.metrics["max_forward_deg"] = round(tr["max_deg"], 1)
        ref.metrics["max_backward_deg"] = round(-tr["min_deg"], 1)
        k0, k1 = sorted((tr["anchor"], tr["ext"]))
        n = math.ceil((k0 - math.pi) / (2 * math.pi))
        if math.pi + 2 * math.pi * n <= k1 and travel > rev:
            ref.stage("over the top", wrist_radius_m=round(wrist_radius(w), 3))
        if travel >= math.radians(p["circle_deg"]) and ref.stage("full circle"):
            ref.metrics["time_s"] = round(w.t, 2)
            ref.succeed()

    def privileged(self, w):
        ang, radius = swing_state(w, self.p["bar_height_m"])
        return {"swing_deg": math.degrees(ang), "com_radius_m": radius, "wrist_radius_m": wrist_radius(w),
                "bar": [0.0, 0.0, self.p["bar_height_m"]], "travel_deg": self.trackers.get("best_deg", 0.0),
                "holding": self.holding(w.ref), "caught": self.trackers.get("caught", False),
                "bar_in_jaws_m": {k: v[0] for k, v in bar_in_jaws(w, self.p["bar_height_m"]).items()}}
