"""Women's 50 m hurdles (World Athletics TR 22, the indoor 50 m hurdles: four 0.838 m hurdles at 13.00 m + 8.50 m x i,
11.50 m to the finish), run by the Unitree H1-2; every hurdle is gone over, knocking one down with the legs is allowed
(TR 22.7), going beside it or under its top is not, nor knocking it by hand or body (TR 22.6.2).

Each hurdle is a free rigid body standing on the track, as a real one does (TR 22.4-22.5): an L-shaped base on each
side (its foot pointing towards the approaching athlete, the upright at its far end), a rectangular frame (the top
bar and a cross bar), two counterweights on the bases set for the official toppling force at this height, rubber feet
on the polyurethane track. It touches everything (the track, the robot, the other hurdles): it can be pushed, slide,
tip over forwards or backwards, bounce, and lies where it comes to rest. Its construction is the one of the
`hurdles-freestanding` family (the men's 50 m), with the counterweights moved for the women's height, as on a real
adjustable hurdle. The two families differ in the event (women's 0.838 m hurdles on the women's marks here; the men's
1.067 m there) and in the referee: here TR 22.7 as written (a hurdle knocked or pushed by the legs is no offence, in
any way), there a stricter reading (only a leg striking the top bar in the air, never a hurdle shoved along the track
first, no climbing). A hurdle is judged at its standing pose on its mark (the plane of its top bar there, its top's
height), wherever it is now."""
import math

import numpy as np

from rfo.referee import Crossing
from rfo.task import TaskBase

TRACK = (0.72, 0.25, 0.18, 1)
H1_FOOT_FRONT = 0.158            # m: the front end of the standing H1-2's feet ahead of its pelvis (measured)
UPPER = ("hands", "forearms", "upper_arms", "torso", "pelvis")     # "hand or body" of TR 22.6.2
ORD = ("first", "second", "third", "fourth", "fifth", "sixth", "seventh", "eighth", "ninth", "tenth")
# The hurdle's construction (TR 22.4: metal, the top bar of a non-metallic material; two bases and two uprights at the
# extreme end of each base, a rectangular frame reinforced by a cross bar; adjustable counterweights). The rule book
# fixes the width, the top bar's height and thickness, the base's length, the total mass and the toppling force; the
# rest is a typical competition hurdle's (steel 30 mm tubes, a wooden top bar, steel counterweights): the numbers of
# the hurdles-freestanding family's hurdle.
BAR_DEPTH = 0.02                 # m: the top bar's thickness (TR 22.4: 10 to 25 mm)
TUBE = 0.03                      # m: the uprights' and the bases' square steel tube
CROSS_R = 0.0125                 # m: the frame's cross bar (a 25 mm tube), cross_below_m below the hurdle's top
CW_HALF = (0.05, 0.03, 0.0292)   # m: a counterweight's half sizes (steel, 2.75 kg)
PAD_R, CLEAR = 0.010, 0.002      # m: the rubber feet (one under each end of each base: what stands on the track) and
#                                  the base tube's clearance above the track
MASS = dict(bar=1.0, upright=0.9, cross=0.8, base=0.85, pad=0.025, counterweight=2.75)    # kg (10.9 kg in all)
TRACK_FRICTION = 0.6             # the hurdle's rubber feet on a polyurethane track (a hard rubber on a synthetic
#                                  surface, dry: 0.5 to 0.7); World Athletics does not specify it
G = 9.80665
# The hurdle's contacts (they win over the other geom's: priority): nearly hard, so that a push below the toppling
# force holds it on the track by friction instead of letting it creep (with MuJoCo's default soft contacts a push of
# 3.6 kgf at the top bar walks it 2.5 mm/s; with these 0.007 mm/s; measured in hurdles-freestanding)
CONTACT = dict(solref="0.003 1", solimp="0.998 0.9995 0.001")


def parts(p) -> list:
    """The hurdle's solid parts: (name, type, centre, size, mass), in the hurdle's frame: on the track under the
    front rubber feet (the line it tips over when pushed forwards), x towards the finish, y across the lane."""
    H, bw, bh, L = p["hurdle_height_m"], p["hurdle_width_m"], p["bar_height_m"], p["base_length_m"]
    fy = bw / 2 - TUBE / 2                       # the uprights and the bases, under the ends of the top bar
    up_h = (H - bh - CLEAR - TUBE) / 2           # an upright: from the base's top to the top bar's lower edge
    cw_x = -counterweight_d(p)
    out = [("bar", "box", (PAD_R - TUBE - BAR_DEPTH / 2, 0.0, H - bh / 2), (BAR_DEPTH / 2, bw / 2, bh / 2),
            MASS["bar"]),
           ("cross", "capsule", (PAD_R - TUBE / 2, 0.0, H - p["cross_below_m"]), (CROSS_R, fy - TUBE / 2),
            MASS["cross"])]
    for s, tag in ((1, "L"), (-1, "R")):
        out += [(f"up{tag}", "box", (PAD_R - TUBE / 2, s * fy, CLEAR + TUBE + up_h), (TUBE / 2, TUBE / 2, up_h),
                 MASS["upright"]),
                (f"base{tag}", "box", (PAD_R - L / 2, s * fy, CLEAR + TUBE / 2), (L / 2, TUBE / 2, TUBE / 2),
                 MASS["base"]),
                (f"cw{tag}", "box", (cw_x, s * fy, CLEAR + TUBE + CW_HALF[2]), CW_HALF, MASS["counterweight"]),
                (f"pad{tag}0", "sphere", (0.0, s * fy, PAD_R), (PAD_R,), MASS["pad"]),
                (f"pad{tag}1", "sphere", (2 * PAD_R - L, s * fy, PAD_R), (PAD_R,), MASS["pad"])]
    return out


def hull_points(m, g, n=400) -> np.ndarray:
    """The support points of mesh geom g (geom frame) along n fixed directions (a Fibonacci sphere)."""
    mid = int(m.geom_dataid[g])
    v = m.mesh_vert[m.mesh_vertadr[mid]:m.mesh_vertadr[mid] + m.mesh_vertnum[mid]].astype(float)
    i = np.arange(n) + 0.5
    phi, th = np.arccos(1 - 2 * i / n), math.pi * (1 + 5 ** 0.5) * i
    dirs = np.c_[np.cos(th) * np.sin(phi), np.sin(th) * np.sin(phi), np.cos(phi)]
    return v[np.unique(np.argmax(v @ dirs.T, axis=0))]


def fixed_moment(p) -> float:
    """sum(m d) (kg m) of the hurdle's parts other than the counterweights, d = each part's distance behind the front
    rubber feet (the line it tips over when pushed forwards)."""
    L = p["base_length_m"]
    return (MASS["bar"] * (TUBE + BAR_DEPTH / 2 - PAD_R) + (2 * MASS["upright"] + MASS["cross"]) * (TUBE / 2 - PAD_R)
            + 2 * MASS["base"] * (L / 2 - PAD_R) + 2 * MASS["pad"] * (L - 2 * PAD_R))


def counterweight_d(p) -> float:
    """Where the counterweights sit on the bases (their centres' distance behind the front rubber feet, m) for the
    toppling force of the rules: a horizontal force F at the centre of the top bar's top edge tips the hurdle over
    its front feet when F h > g sum(m d) (the official hurdle's adjustable counterweights, set for this height)."""
    return (p["toppling_force_kgf"] * p["hurdle_height_m"] - fixed_moment(p)) / (2 * MASS["counterweight"])


def balance_deg(p) -> float:
    """The forward tilt past which the hurdle falls on by itself (its centre of mass over the front feet)."""
    ps = parts(p)
    M = sum(q[4] for q in ps)
    cx = sum(q[4] * q[2][0] for q in ps) / M
    cz = sum(q[4] * q[2][2] for q in ps) / M
    return math.degrees(math.atan2(-cx, cz))


class Task(TaskBase):
    name = "hurdles"
    title = "Women's 50 m hurdles (indoor)"
    category = "track-and-jumps"
    robot = "unitree_h1"
    rulebook = ("World Athletics Competition and Technical Rules, 2024 edition: TR 22 (22.1-22.3 the races, heights "
                "and distances: the women's 50 m hurdles indoors, four hurdles 0.838 m high, 13.00 m from the start "
                "to the first, 8.50 m between, 11.50 m from the last to the finish; 22.4-22.5 the hurdle: metal with a "
                "non-metallic top bar, two bases and two uprights at the extreme end of each base supporting a "
                "rectangular frame reinforced by a cross bar, 1.18 to 1.20 m wide, a top bar 70 mm high and 10 to 25 "
                "mm thick, a base at most 0.70 m long, at least 10 kg, counterweights adjustable so that at every "
                "height a horizontal force of 3.6 to 4.0 kg at the centre of the top bar's top edge topples it; "
                "placed with the feet towards the approaching athlete; 22.6 each hurdle is gone over in the "
                "athlete's own lane; 22.6.1 a foot or leg beside the hurdle below the plane of its top; 22.6.2 "
                "knocking down or displacing a hurdle by hand or body; 22.7 knocking hurdles down otherwise is no "
                "offence), TR 17.3 (lanes), TR 18.2 and TR 19 (the finish)")
    deviations = [
        "TR 22.1 (the event): the women's 50 m hurdles is run by the Unitree H1-2 (1.78 m, 67 kg), a robot without a "
        "category; the event, not the athlete, sets the heights and marks (the men's 50 m with 1.067 m hurdles is "
        "the hurdles-freestanding family)",
        "TR 16 (the start): no starter's commands, no blocks and no reaction time: the robot stands with the front "
        "ends of its feet {toe_gap_m} m behind the start line and the clock runs from t = 0",
        "TR 22.6 ('goes over each hurdle') is judged geometrically at the hurdle's standing pose on its mark, "
        "wherever the hurdle is now: the pelvis must pass above the height of the hurdle's top, each foot (its "
        "ankle joint) above the lower edge of the top bar, all three between the bar's ends, through the plane "
        "where the top bar stood (a hurdle that has been knocked down or pushed away does not lower or move what "
        "is to be gone over)",
        "TR 22.6.2 (knocking down or displacing a hurdle by hand, body or the upper side of the lead leg): intent "
        "and the side of the leg cannot be judged: a hurdle that is displaced (tipped more than {displaced_deg} "
        "degrees, or a point of its top bar moved more than {displaced_m} m) within {upper_touch_s} s of a touch, "
        "while it still stood, by a hand, an arm, the trunk or the pelvis is a failure; the legs may knock it down "
        "or push it in any way (TR 22.7)",
        "TR 17.3 (lanes): the lane is judged by the pelvis (its centre more than half a lane width from the "
        "lane's centre line); TR 22.6.1 is judged on the centres of the legs' collision bodies within "
        "{beside_m} m of the hurdle's plane",
        "TR 17 (a fallen athlete): a robot whose pelvis is below {down_z_m} m is down; it may get up and go on, "
        "but {down_limit_s} s down in a row end the race (did not finish)",
        "TR 19 (photo finish): the time is the instant the leading point of the torso link's collision mesh "
        "reaches the finish line's plane, interpolated between 1 ms physics steps (the H1-2's trunk and head are "
        "one link, so its torso includes the head); no wind; the robot alone on a straight of three lanes",
        "TR 22.4-22.5 (the hurdle): a rigid body (no deflection of the top bar); the rule book does not give the "
        "friction of a hurdle on the track: {track_friction} (rubber feet on a polyurethane track); the "
        "counterweights are set once for the toppling force {toppling_force_kgf} kg at this height; the indoor "
        "distance is run on an outdoor straight; the time limit, {time_limit_s} s, is this benchmark's "
        "(FAMILIES.md)",
    ]
    time_limit_s = 140.0
    start = (-(0.05 + 0.02 + H1_FOOT_FRONT), 0.0, 0.0)
    stages = [f"hurdle {i + 1}" for i in range(4)] + ["finish"]
    fall_rule = False                # the race's own: a fallen runner may get up (the "down" rule)
    params = dict(distance_m=50.0, hurdles=4, first_hurdle_m=13.00, spacing_m=8.50, last_to_finish_m=11.50,
                  hurdle_height_m=0.838, hurdle_width_m=1.19, bar_height_m=0.07, base_length_m=0.70,
                  cross_below_m=0.35, toppling_force_kgf=3.8, hurdle_mass_kg=10.9, balance_deg=0.0,
                  track_friction=TRACK_FRICTION, lane_width_m=1.22, start_line_m=0.05, toe_gap_m=0.02,
                  knocked_deg=45.0, displaced_deg=10.0, displaced_m=0.10, upper_touch_s=0.5, beside_m=0.10,
                  down_z_m=0.40, down_limit_s=10.0)

    sentence = ("Run the women's 50 m hurdles: go over all four 0.838 m hurdles in your lane, never beside one or "
                "under its top, and bring your torso to the finish line.")
    rules = ("World Athletics rules for the hurdles: the women's 50 m hurdles (indoor). The lane runs along +x, "
             "{lane_width_m} m wide, centred on y = 0; the start line ({start_line_m} m wide) has its far edge at "
             "x = 0, the finish line is x = {distance_m} m. {hurdles} hurdles stand across the lane, the face of "
             "the first one's top bar {first_hurdle_m} m from the start line and one every {spacing_m} m after it "
             "(the last {last_to_finish_m} m before the finish). A hurdle is a loose object standing on the track: "
             "{hurdle_width_m} m wide and {hurdle_height_m} m high, a top bar {bar_height_m} m high, two uprights "
             "and a cross bar {cross_below_m} m below its top; its two bases, {base_length_m} m long, lie on the "
             "track on your side of it, with counterweights on them; it weighs {hurdle_mass_kg} kg and tips over "
             "forwards when a horizontal force of more than {toppling_force_kgf} kg pushes the middle of its top "
             "bar (tipped further than about {balance_deg} degrees it falls on by itself); it can slide on the "
             "track (friction {track_friction}), fall, bounce and lie where it comes to rest, and you can trip on "
             "it. Each hurdle is judged where it stands at the start (its standing pose on its mark), wherever it "
             "is later. The clock starts at t = 0. You go over a hurdle when your pelvis and both your feet (the "
             "ankle joints) have crossed the plane of its top bar in its standing pose towards +x, each between "
             "the bar's two ends, the pelvis higher than the hurdle's top ({hurdle_height_m} m above the track) "
             "and each foot higher than the lower edge of its top bar ({hurdle_height_m} m less {bar_height_m} m), "
             "whether the hurdle still stands there or not; a pelvis or a foot that comes back over the plane has "
             "not crossed it and must cross it again. The hurdles are gone over in order. Knocking a hurdle down "
             "or pushing it with your legs is allowed (a hurdle tipped more than {knocked_deg} degrees is counted "
             "as down, without a fault). You finish at the moment the leading point of your torso (the trunk and "
             "head hull; not the arms, legs or feet) reaches the finish line, with all hurdles gone over. You "
             "succeed with a finish within {time_limit_s} s of robot time. A fall does not end the race: you are "
             "down while your pelvis is below {down_z_m} m, you may get up and go on, but {down_limit_s} s down in "
             "a row end it. You fail if your pelvis or a foot crosses the next hurdle's plane beside the hurdle, or "
             "lower than said above (going under its top); if a part of one of your legs is beside the next hurdle "
             "(beyond the bar's ends, within {beside_m} m of its plane) lower than the hurdle's top; if a hurdle is "
             "displaced (tipped more than {displaced_deg} degrees, or a point of its top bar moved more than "
             "{displaced_m} m from where it stood) within {upper_touch_s} s of being touched, while it was not yet "
             "displaced, by a hand, an arm, your trunk or your pelvis (knocked by hand or body; only hurdles within "
             "3 m of your pelvis are judged); if your pelvis is more than half the lane's width from y = 0; if "
             "your torso reaches the finish line before every hurdle is gone over; when down for {down_limit_s} s; "
             "or after {time_limit_s} s.")
    venue = ("You stand in your lane facing the finish (+x), the front ends of your feet {toe_gap_m} m behind the "
             "start line, your pelvis 0.23 m behind it. The lane lines are white on a red track; the hurdles' top "
             "bars are white with black stripes, their bases and counterweights grey on the track, reaching "
             "{base_length_m} m towards you along the lane's edges; two white posts stand beside the finish line. "
             "In your rest pose your forearms point forward a little above the height of a hurdle's top.")
    difficulty = ("The robot's standing pelvis is above a women's hurdle, but each foot has to be lifted 0.77 m "
                  "over every one of the four, on a body of 67 kg whose only built-in gait walks at 0.7 m/s, drifts "
                  "in heading and lifts a foot a few centimetres. The hurdles are loose and touch everything: one "
                  "walked into with the arms or trunk is a failure, and one knocked down by a clipped foot lies on "
                  "the track as an obstacle.")

    def configure(self, params):
        super().configure(params)
        self.p["balance_deg"] = round(balance_deg(self.p))               # derived (texts)
        self.p["last_to_finish_m"] = round(self.p["distance_m"] - self.hurdle_x()[-1], 2)

    def __init__(self):
        super().__init__()
        self.configure({})

    # ------------------------------------------------------------------ the venue
    def hurdle_x(self):
        """The marks: the x of each hurdle's top bar face on the approach side, standing."""
        p = self.p
        return [round(p["first_hurdle_m"] + i * p["spacing_m"], 2) for i in range(int(p["hurdles"]))]

    def plane_x(self, i) -> float:
        """The plane of hurdle i's top bar (its middle) in the standing pose: what is gone over."""
        return self.hurdle_x()[i] + BAR_DEPTH / 2

    def hurdle_mass(self) -> float:
        return sum(q[4] for q in parts(self.p))

    def toppling_force_kgf(self) -> float:
        """The horizontal force at the centre of the top bar's top edge that just tips a hurdle over (kgf), from
        its parts (the statics of a rigid body on its front feet)."""
        ps = parts(self.p)
        return sum(q[4] * -q[2][0] for q in ps) / self.p["hurdle_height_m"]

    def _hurdle(self, sc, name, mark):
        """One hurdle, a free body (frame: parts())."""
        p = self.p
        H, bh = p["hurdle_height_m"], p["bar_height_m"]
        b = sc.body(name, (mark + BAR_DEPTH + TUBE - PAD_R, 0.0, 0.0), free=True)
        kw = dict(contype="1", conaffinity="13", condim="3", priority="1",
                  friction=(p["track_friction"], 0.005, 0.0001), **CONTACT)
        colour = {"bar": (0.97, 0.97, 0.97, 1), "cw": (0.18, 0.18, 0.2, 1), "pad": (0.1, 0.1, 0.1, 1)}
        for part, typ, pos, size, mass in parts(p):
            extra = {}
            if typ == "capsule":                         # along y
                extra = dict(fromto=(pos[0], -size[1], pos[2], pos[0], size[1], pos[2]))
                size, pos = size[:1], None
            sc.geom(b, f"{name}_{part}", type=typ, pos=pos, size=size, mass=mass,
                    rgba=colour.get(part.rstrip("LR01"), (0.3, 0.3, 0.32, 1)), **extra, **kw)
        for i, yy in enumerate((-0.4, 0.0, 0.4)):        # the stripes (visual only)
            sc.geom(b, f"{name}_stripe{i}", type="box", pos=(PAD_R - TUBE - BAR_DEPTH - 0.0005, yy, H - bh / 2),
                    size=(0.0005, 0.1, bh / 2 - 0.002), rgba=(0.08, 0.08, 0.08, 1), contype="0", conaffinity="0",
                    mass=0)

    def build(self, sc):
        p = self.p
        assert counterweight_d(p) + CW_HALF[0] <= p["base_length_m"] - PAD_R, "the counterweights are off the bases"
        assert counterweight_d(p) - CW_HALF[0] >= TUBE, "the counterweights are under the uprights"
        assert abs(self.hurdle_mass() - p["hurdle_mass_kg"]) < 1e-9, "hurdle_mass_kg is what the construction gives"
        D, W, S = p["distance_m"], p["lane_width_m"], p["start_line_m"]
        floor = next(g for g in sc.wb.iter("geom") if g.get("name") == "floor")
        floor.set("pos", f"{D / 2:g} 0 0")               # the grass is drawn around the whole straight
        floor.set("size", f"{D / 2 + 40:g} 30 0.05")
        x0, x1 = -12.0, D + 25.0
        sc.box("track", ((x0 + x1) / 2, 0, 0.0003), ((x1 - x0) / 2, W * 1.5 + 0.3, 0.0003), rgba=TRACK)
        for i, y in enumerate((-1.5 * W, -0.5 * W, 0.5 * W, 1.5 * W)):
            sc.line(f"lane{i}", (x0, y), (x1, y))
        sc.line("start_line", (-S / 2, -1.5 * W), (-S / 2, 1.5 * W), width=S, z=0.0009)
        sc.line("finish_line", (D + S / 2, -1.5 * W), (D + S / 2, 1.5 * W), width=S, z=0.0009)
        for s, tag in ((1, "L"), (-1, "R")):
            sc.box(f"finish_post{tag}", (D, s * (1.5 * W + 0.5), 1.0), (0.05, 0.05, 1.0))
        for i, x in enumerate(self.hurdle_x()):
            self._hurdle(sc, f"hurdle{i + 1}", x)
        sc.camera("venue", (D / 2, -(6.0 + 0.36 * D), 3.0 + 0.14 * D), (D / 2, 0.0, 0.0), fovy=62)
        x = self.hurdle_x()[0]
        sc.camera("first_hurdle", (x - 2.0, -3.2, 1.5), (x + 0.2, 0.0, 0.45), fovy=55)

    def reset(self, w):
        for i, x in enumerate(self.hurdle_x()):          # each hurdle on its mark, standing, at rest
            w.put(f"hurdle{i + 1}", (x + BAR_DEPTH + TUBE - PAD_R, 0.0, 0.0))

    # ------------------------------------------------------------------ the referee
    def referee_reset(self, w, ref):
        m = w.m
        xs = self.hurdle_x()
        self.n_hurdles = len(xs)
        self.hbody = [m.body(f"hurdle{i + 1}").id for i in range(self.n_hurdles)]
        self.bar_geom = [m.geom(f"hurdle{i + 1}_bar").id for i in range(self.n_hurdles)]
        self.hurdle_mask = [ref.geoms(body=f"hurdle{i + 1}") for i in range(self.n_hurdles)]
        self.points = {"pelvis": w.robot.pelvis, "left foot": m.body("left_ankle_roll_link").id,
                       "right foot": m.body("right_ankle_roll_link").id}
        self.leg_ids = np.flatnonzero(ref.part("legs"))
        self.upper = ref.part(*UPPER)
        self.torso_geom = next(g for g in range(m.ngeom) if m.body(int(m.geom_bodyid[g])).name == "torso_link"
                               and m.geom_contype[g] and int(m.geom_dataid[g]) >= 0
                               and m.mesh(int(m.geom_dataid[g])).name.endswith("torso_link"))
        self.torso_pts = hull_points(m, self.torso_geom)
        self.bar_rest = [self.bar_points(w, i) for i in range(self.n_hurdles)]
        # over: who (pelvis, left foot, right foot) is over the next hurdle; upper_t: when each hurdle was last
        # touched, while it still stood, by a hand, an arm or the trunk; down: the hurdles that have been knocked
        # down; front: the torso's leading x one physics step ago; down_k: since when the robot is down
        self.trackers = {"over": [], "upper_t": [None] * self.n_hurdles, "down": [], "front": None, "down_k": None}
        for i in range(self.n_hurdles):
            for k in self.points:
                self.trackers[f"bar{i + 1} {k}"] = Crossing((1, 0, 0), self.plane_x(i), direction=1)

    def bar_points(self, w, i) -> np.ndarray:
        """The top bar's centre and its two ends (the middle of each end face), world (3, 3)."""
        g = self.bar_geom[i]
        R = w.d.geom_xmat[g].reshape(3, 3)
        h = self.p["hurdle_width_m"] / 2
        return w.d.geom_xpos[g] + np.array([[0.0, 0.0, 0.0], [0.0, h, 0.0], [0.0, -h, 0.0]]) @ R.T

    def displacement(self, w, i) -> float:
        """How far a point of hurdle i's top bar is from where it stood (m)."""
        return float(np.max(np.linalg.norm(self.bar_points(w, i) - self.bar_rest[i], axis=1)))

    def displaced(self, w, i) -> bool:
        return self.tilt_deg(w, i) > self.p["displaced_deg"] or self.displacement(w, i) > self.p["displaced_m"]

    def torso_front(self, w) -> float:
        g = self.torso_geom
        R = w.d.geom_xmat[g].reshape(3, 3)
        return float(w.d.geom_xpos[g][0] + np.max(self.torso_pts @ R[0]))

    def tilt_deg(self, w, i) -> float:
        """The angle of the hurdle's uprights from the vertical, degrees (0 standing, about 90 lying)."""
        return math.degrees(math.acos(max(-1.0, min(1.0, float(w.d.xmat[self.hbody[i]][8])))))

    def substep(self, w, ref):
        tr, p = self.trackers, self.p
        half, H, D = p["hurdle_width_m"] / 2, p["hurdle_height_m"], p["distance_m"]
        n = len(ref.stages)                              # hurdles gone over so far
        for i in range(self.n_hurdles):
            for k, b in self.points.items():
                x = tr[f"bar{i + 1} {k}"].update(w.d.xpos[b])
                if x is None or i != n:                  # only the next hurdle counts
                    continue
                y, z = float(x[1]), float(x[2])
                if abs(y) >= half:
                    return ref.fail(f"went beside the {ORD[i]} hurdle", what=k, y=round(y, 3))
                low = H if k == "pelvis" else H - p["bar_height_m"]
                if z <= low:
                    return ref.fail(f"went under the top of the {ORD[i]} hurdle", what=k, z=round(z, 3),
                                    must_be_above=round(low, 3))
                if k not in tr["over"]:
                    tr["over"] = sorted(tr["over"] + [k])
            if i == n and tr["over"]:                    # a point that comes back has not gone over the hurdle
                back = [k for k in tr["over"] if float(w.d.xpos[self.points[k]][0]) < self.plane_x(i)]
                if back:
                    tr["over"] = [k for k in tr["over"] if k not in back]
            if i == n and len(tr["over"]) == len(self.points):
                ref.stage(f"hurdle {i + 1}", tilt_deg=round(self.tilt_deg(w, i), 1))
                tr["over"] = []
                n += 1
        if n < self.n_hurdles:                           # TR 22.6.1: a leg beside the next hurdle below its top
            c = w.d.geom_xpos[self.leg_ids]
            bad = (np.abs(c[:, 0] - self.plane_x(n)) < p["beside_m"]) & (np.abs(c[:, 1]) > half) & (c[:, 2] < H)
            if bool(np.any(bad)):
                g = int(self.leg_ids[np.flatnonzero(bad)[0]])
                return ref.fail(f"a leg beside the {ORD[n]} hurdle below its top",
                                part=ref.part_names[int(ref.part_code[g])], y=round(float(w.d.geom_xpos[g][1]), 3))
        px = float(w.d.xpos[w.robot.pelvis][0])
        for i in range(self.n_hurdles):                  # TR 22.6.2: by hand or body
            if abs(px - float(w.d.xpos[self.hbody[i]][0])) > 3.0:     # (where the hurdle is now)
                continue
            displaced = self.displaced(w, i)
            if not displaced and ref.touching(self.upper, self.hurdle_mask[i]):
                tr["upper_t"][i] = ref.time               # a touch of a hurdle that still stands
            if self.tilt_deg(w, i) > p["knocked_deg"] and i not in tr["down"]:
                tr["down"] = tr["down"] + [i]
                ref.event("hurdle_down", hurdle=i + 1)
                ref.metrics["hurdles_down"] = len(tr["down"])
            t = tr["upper_t"][i]
            if displaced and t is not None and ref.time - t <= p["upper_touch_s"]:
                return ref.fail(f"knocked the {ORD[i]} hurdle with a hand, an arm or the trunk",
                                tilt_deg=round(self.tilt_deg(w, i), 1), moved_m=round(self.displacement(w, i), 3),
                                parts=sorted(ref.parts_touching(self.hurdle_mask[i])))
        f0, f1 = tr["front"], self.torso_front(w)        # MuJoCo's positions after a step are one physics step old
        tr["front"] = f1
        if f0 is not None and f0 < D <= f1:
            if n < self.n_hurdles:
                return ref.fail("reached the finish line before going over every hurdle", hurdles=n)
            dt = w.m.opt.timestep
            t = ref.time - 2 * dt + (D - f0) / (f1 - f0) * dt
            ref.metrics["time_s"] = round(t, 3)
            if ref.stage("finish", time_s=round(t, 3)):
                ref.succeed()

    def step(self, w, ref):
        tr, p = self.trackers, self.p
        x, y, z, _ = (float(v) for v in w.pelvis())
        ref.metrics["hurdles_cleared"] = min(len(ref.stages), self.n_hurdles)
        ref.metrics["distance_m"] = round(max(ref.metrics.get("distance_m", 0.0), min(x, p["distance_m"])), 2)
        if abs(y) > p["lane_width_m"] / 2:
            return ref.fail("left the lane", y=round(y, 3))
        if z < p["down_z_m"]:
            if tr["down_k"] is None:
                tr["down_k"] = w.k
                ref.event("down")
            elif (w.k - tr["down_k"]) * w.CONTROL_DT >= p["down_limit_s"]:
                return ref.fail("did not finish: down for too long", down_s=p["down_limit_s"])
        elif tr["down_k"] is not None:
            tr["down_k"] = None
            ref.event("up")

    def privileged(self, w):
        n = min(len(w.ref.stages), self.n_hurdles)
        hs = []
        for i in range(self.n_hurdles):
            qa, _ = w.free[f"hurdle{i + 1}"]
            hs.append({"mark_x": self.hurdle_x()[i], "plane_x": self.plane_x(i),
                       "pos": [round(float(v), 4) for v in w.d.qpos[qa:qa + 3]],
                       "quat": [round(float(v), 5) for v in w.d.qpos[qa + 3:qa + 7]],
                       "tilt_deg": round(self.tilt_deg(w, i), 2), "displaced_m": round(self.displacement(w, i), 4)})
        return {"pelvis": w.pelvis().tolist(), "hurdle_x": self.hurdle_x(), "next_hurdle": n + 1,
                "hurdle_tilt_deg": [round(self.tilt_deg(w, i), 2) for i in range(self.n_hurdles)], "hurdles": hs,
                "hurdle_top_m": self.p["hurdle_height_m"], "torso_front_x": self.torso_front(w)}
