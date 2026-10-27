"""Scrub an EmbodiedSWE checkout down to what the agent's image may hold (docs/protocol/leakage.md, PROTOCOL H.2).

    python -I scrub_embodiedswe.py /opt/embodiedswe [--check]

Run in the image build, on a fresh checkout of EmbodiedSWE at the pinned commit, after its `.git` is gone (its history
holds solve-calibrated configurations). Idempotent: a second run changes nothing, and `--check` fails when any rule
would still change something or a banned pointer is left.

What it removes, and why:
- Everything but `robobench/`, `LICENSE` and `pyproject.toml`: `eval/` (prompts, hints, skills, example solutions,
  their Harbor adapter), `data_engine/`, `vla/` (names the public demonstration datasets, e.g. the bulb dataset),
  `rl/`, `sim_gen/`, `real_to_sim/`, `experiments/`, `scripts/`, `tests/`, `docs/`.
- Every suite's `grader/` (the hidden rubric; the verifier gets its task's grader back from the task's `tests/`) and
  `smokes/` (scripted teleport probes and their calibration printouts), and the `deformable` suite (Newton, VLM judge;
  not shipped).
- Every comment and every docstring but the module's in each suite's `configs/envs.py` (the bindings): they are the
  developers' calibration notes (reach envelopes measured with an IK probe, calibrated grasp and release stances,
  grasp-approach azimuth bands, "load-bearing calibrated constants"). A stripped docstring becomes a neutral one.
- In the shipped tasks' scene modules and their text assets (`.usda` layers, provenance notes), the comments that hand
  the agent solve-calibrated numbers or recipes (the calibrated grasp and placing stances, the measured grasp windows,
  release recipes, pointers to removed smokes and solutions): replaced by a neutral line. A `.usda` edit may change
  only `#` comment lines.
Code is never changed: the scrub asserts that every module's AST without docstrings is unchanged.
"""
from __future__ import annotations

import argparse
import ast
import shutil
import sys
from pathlib import Path

KEEP_TOP = {"robobench", "LICENSE", "pyproject.toml"}
DROP_SUITE_DIRS = ("grader", "smokes")
DROP_SUITES = ("deformable",)

# (file, old text, new text): each old text must occur exactly once on a fresh checkout and never after the scrub.
EDITS: list[tuple[str, str, str]] = [
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "distance is now the hard part: pick-side geometry reproduces the fixed-base tier's measured\n"
     "grasp cell, so the NEW skill this task grades on top of it is the carry.\n",
     "distance is now the hard part: the NEW skill this task grades on top of it is the carry.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "  * PICK ZONE (robot's side, y < 0): the fruit at the calibrated stance spot, the produce\n",
     "  * PICK ZONE (robot's side, y < 0): the fruit, the produce\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "  * FAR SIDE (y > 0): the clay plate, centred at `plate_pos`, inset from the far edge so a\n"
     "    placing stance on the far side reproduces the fixed-base tier's plate reach.\n",
     "  * FAR SIDE (y > 0): the clay plate, centred at `plate_pos`, inset from the far edge.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    The G1-tier manifest keeps the fruits this hand measurably grasps (the fixed-base tier's\n"
     "    findings carry over: the lemon is dependable; near-spheres roll out of the three-finger\n"
     "    pinch) and ALL SEVEN non-fruit distractors, because the crowd is what makes the scene read\n"
     "    like the original — and what makes the identification non-trivial.\n",
     "    The G1-tier manifest keeps one fruit and ALL SEVEN non-fruit distractors, because the crowd\n"
     "    is what makes the scene read like the original — and what makes the identification non-trivial.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: separation from the closest pick-side shoulder is 0.93 m (asserted below). The inset\n"
     "    #: is chosen so a FAR-side stance with the pelvis 0.13 m off the far edge (torso clear)\n"
     "    #: sees the dish centre ~0.38 m from its shoulder, comfortably inside the arm's\n"
     "    #: measured 0.20-0.46 m release annulus; at 0.24 the same stance was 0.44 m out.\n",
     "    #: separation from the closest pick-side shoulder is 0.93 m (asserted below).\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: pick zone on the NEAR side: the fruit line sits 0.28 m ahead of a shoulder whose\n"
     "    #: pelvis stands clear of the near edge — the fixed-base tier's calibrated grasp\n"
     "    #: distance, reproduced at the walking stance.\n",
     "    #: pick zone on the NEAR side.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: G1 tier: ONE fruit — the dependable lemon — and the full distractor set. The task\n"
     "    #: is one complete delivery: pick, carry around the table, place from the far side.\n"
     "    #: Near-spherical fruit stays excluded — measured to roll out of the pinch — and the\n"
     "    #: small lemon (40 mm, under the hand's ~50 mm minimum) with it.\n",
     "    #: G1 tier: ONE fruit (a lemon) and the full distractor set. The task is one complete\n"
     "    #: delivery: pick, carry around the table, place from the far side.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: for the one-lemon tier; kept as the hook for re-admitting the small lemon (x1.3 puts\n"
     "    #: its 40 mm short axis at ~52 mm, inside the hand's measured 50-64 mm window).\n",
     "    #: for the one-lemon tier.\n"),
    ("robobench/suites/locomanip/scenes/fruit_delivery.py",
     "    #: Reset regime of the fixed-base G1 tier (`_fruits_on_plate_g1_cfg`), whose measured grasp\n"
     "    #: this task inherits: the fruit's long axis along +y (narrow axis at the hand's azimuth\n"
     "    #: 180, dead centre of its reachable band) with +/-30 deg of yaw and +/-15 mm of xy\n"
     "    #: jitter, spawned essentially at rest so the commanded orientation survives settling.\n",
     "    #: Reset regime: the fruit's long axis along +y with +/-30 deg of yaw and +/-15 mm of xy\n"
     "    #: jitter, spawned essentially at rest so the commanded orientation survives settling.\n"),
    # BEGIN generated scene edits (leakage review)
    ('robobench/suites/assembly/scenes/allen_bolt_assembly.py',
     "  # the force-driven key smoke sets this True (no hand to bear the handle's weight)",
     '  # True = the key ignores gravity'),
    ('robobench/suites/assembly/scenes/allen_bolt_assembly.py',
     "    # Contact offsets add PER PAIR: the key<->socket clearance is 0.75 mm/side, so the key's and the\n    # bolt's authored offsets together must stay well under that, or speculative contacts seal the\n    # socket mouth — a key teleported INTO the recess still works (the force smoke), but a carried\n    # key can never ENTER from outside. The bolt previously had no authored offset at all, and the\n    # engine default on its head SDF sealed the mouth outright.\n",
     "    # Contact offsets add PER PAIR: the key<->socket clearance is 0.75 mm/side, so the key's and the\n    # bolt's authored offsets together must stay well under it.\n"),
    ('robobench/suites/assembly/scenes/allen_bolt_assembly.py',
     "        # Legacy surface: the grasp contract's state used to live directly on the scene\n        # (inline machinery era) and existing solutions read it there — forward to the\n        # composed contract. `__getattr__` only fires for attributes not found normally.\n",
     '        # Legacy surface: forward _gw_* reads to the composed grasp contract.\n'),
    ('robobench/suites/assembly/scenes/ikea_table_assembly.py',
     'Embodiment-agnostic — it knows nothing about who turns the legs. A Franka grips and screws them; a\nfloating-leg debug robot drives them by force; either way the legs are *scene* objects the robot\nreaches through `env.scene`.\n',
     'Embodiment-agnostic: the legs are *scene* objects the robot reaches through `env.scene`.\n'),
    ('robobench/suites/assembly/scenes/nut_thread_assembly.py',
     "    # Calibrated to the asset (the nut USD's origin sits 10 mm below its bottom face): 12 mm puts the\n    # nut's top face at/below the bolt's thread top — fully threaded on. Resting on the bolt top is\n    # ~25 mm, true bottom-out ~1 mm; gripper pads graze the bolt head below ~4 mm.\n",
     "    # The nut USD's origin sits 10 mm below its bottom face: 12 mm puts the nut's top face\n    # at/below the bolt's thread top (fully threaded on). Resting on the bolt top is ~25 mm.\n"),
    ('robobench/suites/assembly/scenes/pc_gpu_assembly.py',
     "        # Legacy surface: the grasp contract's state used to live directly on the scene\n        # (inline machinery era) and existing solutions read it there — forward to the\n        # composed contract. `__getattr__` only fires for attributes not found normally.\n",
     '        # Legacy surface: forward _gw_* reads to the composed grasp contract.\n'),
    ('robobench/suites/assembly/scenes/pc_gpu_ram_assembly.py',
     'here, no task layer): install the card the way `pc_gpu` proved — place it inside the case\nforward of the rear panel, slide it rearward so the bracket/ports pass through the cutout, press\nit straight down to seat — and press each stick straight down into its DIMM slot the way\n`pc_ram` proved, into the outermost and second-from-socket slots (the alternating pair a 2-stick\ndual-channel kit populates).\n',
     'here, no task layer): install the card as in `pc_gpu` (place it inside the case forward of the\nrear panel, slide it rearward so the bracket/ports pass through the cutout, press it straight\ndown to seat) and press each stick straight down into its DIMM slot as in `pc_ram`, into the\noutermost and second-from-socket slots (the alternating pair a 2-stick dual-channel kit populates).\n'),
    ('robobench/suites/assembly/scenes/pc_gpu_ram_assembly.py',
     'the union of the two proven invisible fixtures',
     'the union of the two invisible fixtures'),
    ('robobench/suites/assembly/scenes/pc_gpu_ram_assembly.py',
     "single-task scenes' proven values verbatim",
     "single-task scenes' values verbatim"),
    ('robobench/suites/assembly/scenes/pc_gpu_ram_assembly.py',
     '  # keeps the press PD/solver in the proven stability class',
     '  # stick mass (kg)'),
    ('robobench/suites/assembly/scenes/pc_gpu_ram_assembly.py',
     "        # Legacy surface: the grasp contract's state used to live directly on the scene\n        # (inline machinery era) and existing solutions read it there — forward to the\n        # composed contract. `__getattr__` only fires for attributes not found normally.\n",
     '        # Legacy surface: forward _gw_* reads to the composed grasp contract.\n'),
    ('robobench/suites/assembly/scenes/pc_motherboard_assembly.py',
     '(xy on the z=0 board face), serpentine drive order.',
     '(xy on the z=0 board face).'),
    ('robobench/suites/assembly/scenes/pc_motherboard_assembly.py',
     "  # the force-driven key smoke sets this True (no hand to bear the handle's weight)",
     '  # True = the key ignores gravity'),
    ('robobench/suites/assembly/scenes/pc_motherboard_assembly.py',
     "        # Legacy surface: the grasp contract's state used to live directly on the scene\n        # (inline machinery era) and existing solutions read it there — forward to the\n        # composed contract. `__getattr__` only fires for attributes not found normally.\n",
     '        # Legacy surface: forward _gw_* reads to the composed grasp contract.\n'),
    ('robobench/suites/assembly/scenes/pc_ram_assembly.py',
     'The stick asset authors an inflated rotational inertia (1e-3 kg m^2 diagonal vs the real\n~2.8e-5): the smoke\'s force-only PD "hand" reuses the pc_gpu rotation gains, which are unstable\nat dt=1/120 on the true inertia; the inflation is invisible on camera and irrelevant to the\npress regime.\n',
     'The stick asset authors an inflated rotational inertia (1e-3 kg m^2 diagonal vs the real\n~2.8e-5); the inflation is invisible on camera and irrelevant to the press regime.\n'),
    ('robobench/suites/assembly/scenes/pc_ram_assembly.py',
     "    # Uniform +/- XY jitter of the WHOLE CASE at reset (m; 0 = pinned at spawn, the historical\n    # behavior). Translation only — solves and the grader read seat positions as case_pos +\n    # seat_pos[k] with the case's IDENTITY orientation, so yaw must stay 0. Nonzero values give\n    # image->press-location covariance (the grounding VLA distillation needs); the demos' solve\n    # tracks the shifted case automatically (its waypoints derive from the live case pose).\n",
     "    # Uniform +/- XY jitter of the WHOLE CASE at reset (m; 0 = pinned at spawn). Translation only:\n    # seat positions are case_pos + seat_pos[k] with the case's identity orientation, so yaw must stay 0.\n"),
    ('robobench/suites/assembly/scenes/pc_ram_assembly.py',
     '    ram_mass: float = 0.25  # a real stick is ~45 g; 0.25 kg keeps the PD/solver in the\n    # proven stability class (the asset also authors an inflated rotational inertia — see module\n    # docstring)\n',
     '    ram_mass: float = 0.25  # a real stick is ~45 g; the asset also authors an inflated rotational\n    # inertia (see the module docstring)\n'),
    ('robobench/suites/assembly/scenes/pc_ram_assembly.py',
     "    #: L4 per-env physics bands (data_engine sampler grammar; nominal = the cfg default, slot 0\n    #: of every batch keeps it). The friction pair brackets the designed slick-stick / grippy-case\n    #: ratio the second stick's gravity-seat rides on (stock 0.3 / 0.75 — the pc_ram IK campaign's\n    #: robustness probe); mass ±20% around the 0.25 kg the PD/solver stability class was tuned at.\n",
     '    #: L4 per-env physics bands (data_engine sampler grammar; nominal = the cfg default, slot 0\n    #: of every batch keeps it). The friction pair brackets the slick-stick / grippy-case ratio\n    #: (stock 0.3 / 0.75); mass +/-20% around the 0.25 kg nominal.\n'),
    ('robobench/suites/assembly/scenes/pc_ram_assembly.py',
     "        # Legacy surface: the grasp contract's state used to live directly on the scene\n        # (inline machinery era) and existing solutions read it there — forward to the\n        # composed contract. `__getattr__` only fires for attributes not found normally.\n",
     '        # Legacy surface: forward _gw_* reads to the composed grasp contract.\n'),
    ('robobench/suites/packing/scenes/pen_holder.py',
     'This task is deliberately SOLVABLE — a graded 0-100 rubric so weak agents rank\ninstead of flatlining — while still honest bimanual manipulation (the source robot\npicks the holder up with one hand and fills it with the other; every insertion\ntargets a compliant, moving cup).\n',
     'This task is scored 0-100 by the staged rubric in score().\n'),
    ('robobench/suites/packing/scenes/pen_holder.py',
     'Grasp the ~1.5 mm rim with any jaw, or palm the ~69 mm body.',
     'Rim ~1.5 mm thick, body ~69 mm across flats.'),
    ('robobench/suites/packing/scenes/tool_packing.py',
     '`scripts/vendor_tool_packing_assets.py` in cap-x:',
     '`scripts/vendor_tool_packing_assets.py`:'),
    ('robobench/suites/packing/scenes/tool_packing.py',
     'Embodiment-agnostic: items and joints are scene objects the robot reaches through\n`env.scene`; a NullRobot smoke drives drawers/doors by joint writes and items by kinematic\nstaging. Heavy imports (isaaclab, pxr) are deferred so importing this module stays app-free.\n',
     'Embodiment-agnostic: items and joints are scene objects the robot reaches through\n`env.scene`. Heavy imports (isaaclab, pxr) are deferred so importing this module stays app-free.\n'),
    ('robobench/suites/packing/scenes/egg_carton.py',
     "    # Eggs scatter around the scatter frame's own yaw, lying roughly parallel.  The\n    # full-circle (180) jitter made the tableau unsolvable for a fixed-base arm on many reset\n    # seeds: a side-lying egg whose long axis points at the robot presents only its blunt end\n    # to a top-down pinch, and no wrist attitude reachable from this shoulder can cage such an\n    # egg.  +/-30 degrees keeps visible per-seed variety (combined with frame yaw, carton\n    # pose, and egg position jitter) while every egg stays physically pinchable from above.\n",
     "    # Eggs scatter around the scatter frame's own yaw, lying roughly parallel (+/-30 degrees).\n"),
    ('robobench/suites/packing/scenes/egg_carton.py',
     'to the deliberate two-stage closing push.',
     'to a deliberate closing push.'),
    ('robobench/suites/packing/scenes/clear_organic_objects.py',
     '`packing.clear_organic_objects.g1.{joint,pink_ik}` (the G1 humanoid works a resized cell — a\nsmaller bin and the four smallest organics — because its pinch hand and fixed pelvis are not an\n80 mm jaw on a 0.8 m arm; see `_clear_organic_objects_g1_cfg`).\n',
     '`packing.clear_organic_objects.g1.{joint,pink_ik}` (the G1 humanoid works its own resized cell).\n'),
    ('robobench/suites/packing/scenes/clear_organic_objects.py',
     '        # lemon_02 uses the SAME mesh as lemon_01 (was the lemon2 mesh at 1.2): the lemon1 mesh\n        # is the one object the G1 hand grasps AND releases reliably, so every sort target is it.\n',
     '        # lemon_02 uses the SAME mesh as lemon_01 (was the lemon2 mesh at 1.2).\n'),
    ('robobench/suites/packing/scenes/clear_organic_objects.py',
     "        # Two more lemon-mesh produce, coloured lime-green (their own asset, below): the\n        # sort tier's TRAY targets. The lemon mesh is the ONE object class the G1 hand grasps\n        # and releases reliably (76x50 / 72x48 mm, ~700-point faceted collider); every other\n        # candidate was measured out -- oranges crushed at the cage, hull avocado/onion/lime\n        # seated well and slid out on the lift, faceted limes launched, bottle and pen holder\n        # geometrically impossible. A lime is a green lemon-shaped fruit, so the sort still\n        # reads naturally: yellow into the crate, green onto the tray.\n        # `lemon_g` and `lime_g` are the lemon1 mesh in their OWN usds, the latter tinted\n        # lime-green (scripts/make_lime_from_lemon.py). Own files for two measured reasons:\n        # extra instances of the SAME usd were launched hundreds of metres whenever a sibling\n        # instance was lifted 8-10 cm away (shared cooked collision data), and this tier's\n        # grasp needs a coarser lemon collider than `fruits_on_plate` -- which reads produce\n        # from this same directory -- was calibrated against.\n",
     "        # Two more lemon-mesh produce, coloured lime-green (their own asset, below): the sort\n        # tier's TRAY targets. `lemon_g` and `lime_g` are the lemon1 mesh in their OWN usds, the\n        # latter tinted lime-green (scripts/make_lime_from_lemon.py).\n"),
    ('robobench/suites/packing/scenes/clear_organic_objects.py',
     "        # oranges: the sort tier's TRAY targets, at their shipped 63 mm. Their collider is a\n        # single convex hull (scripts/fix_clear_organic_colliders.py): with the source\n        # decomposition of the 9.9k-point scan they blew up whenever the G1's fingers pressed a\n        # seam (9.5 m, then a numerical explosion), and a 45 mm version slipped out of the hand\n        # (lift 24-39 mm) -- the two stacked fingers need something they can both bear on.\n",
     "        # oranges: the sort tier's TRAY targets, at their shipped 63 mm. Their collider is a\n        # single convex hull (scripts/fix_clear_organic_colliders.py).\n"),
    ('robobench/suites/packing/scenes/clear_organic_objects.py',
     'the lemons are pinched across their 50/40 mm short axis.',
     'the lemons are 50/40 mm across their short axis.'),
    ('robobench/suites/packing/scenes/clear_organic_objects.py',
     "        # 0.85 / 0.8 and LAID ON THEIR SIDE (fixed_layout roll): 50x51x77 and 49x49x74 mm --\n        # the lemon's shape, which is the one shape the G1 hand holds (a ~50 mm cross-section\n        # with length along the fingers). Their old explosive behaviour was the sliver-hull\n        # decomposition of their scans, replaced by one convex hull (see\n        # scripts/fix_clear_organic_colliders.py). These are the sort tier's TRAY targets.\n",
     "        # 0.85 / 0.8 and LAID ON THEIR SIDE (fixed_layout roll): 50x51x77 and 49x49x74 mm; single\n        # convex-hull colliders (scripts/fix_clear_organic_colliders.py). The sort tier's TRAY targets.\n"),
    ('robobench/suites/packing/scenes/clear_organic_objects.py',
     '    # (The G1 sort tier raises these to 2.0 / 1.8 in its own cfg: its palm CAGE holds a fruit by\n    # friction against the palm rather than by squeezing it, and at 1.1 a hulled lemon slid out\n    # of the cage during the lift-and-roll. That is a property of THAT grasp, not of the produce,\n    # so it lives in the binding — the franka tier keeps the pads it was measured with.)\n',
     '    # (The G1 sort tier raises these to 2.0 / 1.8 in its own cfg.)\n'),
    ('robobench/suites/packing/scenes/clear_organic_objects.py',
     "            # HALF the angle — a z-yaw quaternion is (cos(yaw/2), 0, 0, sin(yaw/2)). This wrote\n            # (cos yaw, 0, 0, sin yaw), which silently DOUBLED every commanded orientation. Free\n            # +/-180 hid it (a doubled uniform circle is still a uniform circle) and it only bit\n            # once a binding pinned the yaw: `reset_yaw_center_deg=90` presented items at 180 deg,\n            # putting a lemon's 50 mm narrow axis at azimuth 90 rather than the 180 the G1 binding\n            # was steering for. Found while porting the same reset into the fruits_on_plate scene,\n            # by tabulating the settled OBB spans instead of trusting the commanded pose.\n",
     '            # HALF the angle -- a z-yaw quaternion is (cos(yaw/2), 0, 0, sin(yaw/2)).\n'),
    ('robobench/suites/packing/scenes/fruits_on_plate.py',
     "            # HALF the angle — a z-yaw quaternion is (cos(yaw/2), 0, 0, sin(yaw/2)). Writing\n            # (cos yaw, 0, 0, sin yaw) silently DOUBLES every commanded orientation, which is\n            # invisible for a free +/-180 range (it still covers the circle) and wrong the moment\n            # a binding pins the yaw: `reset_yaw_center_deg=90` then lands the item at 180 deg, so\n            # the axis a solver was told to expect at azimuth 180 actually presents at 90. Caught\n            # here by measuring the settled OBB spans — the lemon's 50 mm narrow axis showed up at\n            # az 90 with a 76 mm span at az 180, exactly the doubled pose.\n",
     '            # HALF the angle -- a z-yaw quaternion is (cos(yaw/2), 0, 0, sin(yaw/2)).\n'),
    ('robobench/suites/packing/__init__.py',
     'Structure mirrors the assembly suite: `scenes/` (each scene IS one task), `configs/`\n(named ENVS bindings), `smokes/` (NullRobot smoke/oracle runs).\n',
     'Structure mirrors the assembly suite: `scenes/` (each scene IS one task), `configs/`\n(named ENVS bindings).\n'),
    ('robobench/robots/g1.py',
     '    # the work area, resting the hands among (or into) the props; solves then have to\n    # special-case their first move away from home ("re-homing rakes the hand through the\n    # produce", the clear_organics lesson). A binding can instead park the arms bent and\n    # retracted here.\n',
     '    # the work area, resting the hands among (or into) the props. A binding can instead park\n    # the arms bent and retracted here.\n'),
    ('robobench/suites/packing/assets/egg_carton/g1_rubber_fingers.usda',
     "# Task-specific pads for the stock G1 three-finger hand.  Distal links get the\n# high-friction grip compound.  The two distal pads sit 1 mm further apart than in the\n# author's first calibration (45 mm between the inner faces vs the egg's 43 mm): a 43 mm\n# egg then wedges lightly enough to slide out under gravity when the palm turns down\n# over a pocket, instead of having to be pried out by retracting a finger, which flicked it.  Index/middle proximal links use a release\n# compound with high static but much lower dynamic friction: they help establish\n# the pinch, then slide once finger retraction starts instead of carrying the egg\n# out of its pocket.  Binding at each instanceable collision root makes every\n# referenced convex collider inherit the material without modifying the asset.\n",
     '# Task-specific pads for the stock G1 three-finger hand. Distal links get a high-friction\n# grip compound (distal pad inner faces 45 mm apart); index/middle proximal links get a\n# compound with high static but lower dynamic friction. Binding at each instanceable\n# collision root makes every referenced convex collider inherit the material.\n'),
    ('robobench/suites/packing/assets/egg_carton/egg_holder/g1_guided.usda',
     "# slim rails restore the visible separator as collision geometry.  A local upper-row guard is\n# taller because an upright egg protrudes above the scanned base and otherwise lets a withdrawing\n# fingertip tip its occupied neighbour over the visual separator.  Keeping the high section away\n# from the lower-row-to-upper-row chute preserves the first placement's gravity settling path.\n# They are children\n",
     '# slim rails restore the visible separator as collision geometry; a local upper-row guard is\n# taller. They are children\n'),
    ('robobench/suites/packing/assets/egg_carton/SOURCE.md',
     "These are the original authored/scanned assets used by RoboDojo's\n`fill_egg_holder` task. They come from the official\n[`RoboDojo-Benchmark/RoboDojo`](https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo)\ndataset, whose dataset card declares the assets under Apache-2.0. CoSiGen itself also ships\nunder Apache-2.0; see the repository-root `LICENSE`.\n\nTask provenance: upstream\n[`fill_egg_holder.py`](https://github.com/RoboDojo-Benchmark/RoboDojo/blob/main/task/RoboDojo/tasks/fill_egg_holder.py)\nand\n[`fill_egg_holder.yml`](https://github.com/RoboDojo-Benchmark/RoboDojo/blob/main/task/RoboDojo/config/fill_egg_holder.yml).\n",
     "These are the original authored/scanned assets of RoboDojo's `fill_egg_holder` task (RoboDojo\ndataset, Apache-2.0); see the repository-root `LICENSE`.\n"),
    ('robobench/suites/puzzle/scenes/push_shapes.py',
     "    # Layout. Blocks sit nearer the robot than their stations so every piece is pushed away\n    # from the body (+y), the stroke the G1's bench-contact workspace actually affords.\n",
     '    # Layout. Blocks sit nearer the robot than their stations; every piece is pushed +y.\n'),
    ('robobench/suites/puzzle/scenes/push_shapes.py',
     '    # Station centres (pad poses), one per piece in PIECES order. MUST mirror STATIONS in\n    # scripts/author_push_shapes_assets.py: the recessed board is authored with its cutouts\n    # at exactly these poses, and that script also VALIDATES the layout (cutout separation,\n    # spawn clearance, push corridors) before it will write assets. The x placement is\n    # reach-derived -- the right hand completes at x in [0, +0.12], the leftmost station is\n    # finished by the two-hand relay -- and depth is bound by RUNWAY: a piece must spawn\n    # clear of its own cutout, so each lane needs a push of at least a piece depth plus\n    # margins inside a workspace only ~150 mm deep.\n',
     '    # Station centres (pad poses), one per piece in PIECES order; the recessed board is\n    # authored with its cutouts at exactly these poses.\n'),
    ('robobench/suites/puzzle/scenes/push_shapes.py',
     "    # Blocks spawn this far behind their station, straight down-lane (+y push). Per piece:\n    # the T's and L's runways are trimmed to keep spawns inside the arms' proven contact\n    # range (their stations sit in the workspace's deep corners).\n",
     '    # Blocks spawn this far behind their station, straight down-lane (+y push).\n'),
    ('robobench/suites/puzzle/scenes/push_shapes.py',
     '    # The reorientation the task demands. 18 degrees proved to sit past what a single-contact\n    # push can correct while also translating ~8 cm: the steering has authority to hold a\n    # small error but the push induces yaw faster than it can shed a large one. 10 degrees\n    # still exceeds the 7 degree success gate, so correcting it remains mandatory rather than\n    # incidental, while leaving the task solvable. Raise it once a stronger controller exists.\n    # 15 +/- 3 degrees: the worst-case start (12) must sit OUTSIDE yaw_stage_tolerance_deg\n    # (10) or a yaw stage can latch at reset (caught by the smoke as a nonzero reset score\n    # when 10 +/- 4 could jitter down to 6.7), and the best case (18) stays inside the range\n    # the reference policy demonstrably corrects (it recovered 20-degree starts).\n',
     "    # The reorientation the task demands: each block starts this far off its pad's yaw\n    # (sign alternating per lane, plus reset_yaw_jitter_deg).\n"),
    ('robobench/suites/puzzle/scenes/coffee_service.py',
     '    scene crashing (found by two datagen vectorize agents independently). Reuse the\n',
     '    scene crashing. Reuse the\n'),
    ('robobench/suites/puzzle/scenes/coffee_service.py',
     "    cup_r_tol: float = 0.050  # cup centre within this of the spout axis.\n    # 5 cm (not 3): the spout axis sits only 13 mm in front of the nook's back wall\n    # (measured 2026-08-09), so a mug's centre CANNOT reach the axis — machines pour\n    # into a mug's back third. The staged target is cup_stage_y_off south of the axis.\n    cup_stage_y_off: float = -0.040  # staged cup centre, y-offset from the spout\n",
     '    cup_r_tol: float = 0.050  # cup centre within this of the spout axis.\n    cup_stage_y_off: float = -0.040\n'),
    ('robobench/suites/puzzle/scenes/coffee_service.py',
     "    # 0.125 (was 0.105): fully open, the plate's north edge sat only 10 mm south\n    # of the pod pocket and dropped pods FLANGE-CAUGHT on it (v9 telemetry\n    # 2026-08-10: settle z 0.411 = resting on the open cover's rim, never in the\n    # pocket). 2 cm more travel puts the edge 3 cm clear; the cover overhangs\n    # open air to the south by design, nothing to collide with.\n",
     '    # the cover overhangs open air to the south by design, nothing to collide with.\n'),
    ('robobench/suites/puzzle/scenes/coffee_service.py',
     '    # 0.030 with 6 mm walls (was 0.026 with 12 mm): drop scatter (~13-15 mm\n    # measured) caught the pod flange on the wide wall rims and the old z-band\n    # counted rim-caught pods as seated (2026-08-10). Outer span 72 mm still\n    # fits the real 75 mm top opening.\n',
     '    # 6 mm walls; outer span 72 mm fits the real 75 mm top opening.\n'),
    ('robobench/suites/puzzle/scenes/coffee_service.py',
     "    # 0.345 (scan said 0.352): a seated pod's dome ended flush with the cover's\n    # underside (2 mm), and the CLOSING COVER SCOOPED THE POD onto its lid in 3\n    # of 5 chain runs (gate probe 2026-08-10). The 7 mm deeper physics floor\n    # sinks the pod visual slightly into the scanned pocket floor — hidden\n    # inside the dark bay — and buys a 9 mm dome-to-cover margin.\n",
     "    # physics floor 7 mm below the scan's (a 9 mm dome-to-cover margin).\n"),
    ('robobench/suites/puzzle/scenes/coffee_service.py',
     '    # 64 (was 8): each weld/release consumes one pre-authored joint and the pool\n    # never rewinds — 8 ran dry mid-session on the persistent coffee driver\n    # (2026-08-09, "pool dry for cup"). This is the per-boot weld budget.\n',
     '    # each weld/release consumes one pre-authored joint and the pool never\n    # rewinds. This is the per-boot weld budget.\n'),
    ('robobench/suites/puzzle/scenes/spatula_flip_serve.py',
     "Metrics for the brief: `max_carry_tilt_deg` (the blade's worst tilt while loaded — the\nfinesse number; the smoke's calibration sweep publishes the tilt budget the carry\ntolerates) and `spills` (payload-drop events: the bread at rest on the bare bench after\nhaving been loaded).\n",
     "Metrics for the brief: `max_carry_tilt_deg` (the blade's worst tilt while loaded) and\n`spills` (payload-drop events: the bread at rest on the bare bench after having been loaded).\n"),
    ('robobench/suites/puzzle/scenes/spatula_flip_serve.py',
     '    rubric and any oracle work directly in blade coordinates. Colliders: three thin\n',
     '    rubric works directly in blade coordinates. Colliders: three thin\n'),
    ('robobench/suites/puzzle/scenes/spatula_flip_serve.py',
     '    board: its RIM (5.3 cm) walls the payload in, so the wedge becomes a pitched\n    over-the-rim entry and the wall itself is the anchor a jab pins the bread against\n    (a real pan-corner scoop), and every lift-out must clear the rim.\n',
     '    board: its RIM (5.3 cm) walls the payload in, and every lift-out must clear the rim.\n'),
    ('robobench/suites/puzzle/scenes/spatula_flip_serve.py',
     '    """Config for `SpatulaFlipServeScene`. Friction/mass are the brief\'s feasibility-spike\n    knobs; the smoke\'s calibration sweep publishes the carry tilt budget they produce.\n',
     '    """Config for `SpatulaFlipServeScene`. Friction/mass are the brief\'s feasibility-spike\n    knobs.\n'),
    ('robobench/suites/puzzle/scenes/spatula_flip_serve.py',
     '    # loaded step (5 s at 120 Hz — the no-toss/no-shove contact-history clause; covers the\n    # lower-and-tip end game, where the payload dips under the lift gate). Sized for THIS\n    # plate: the bamboo dish is a slope, and a slice tipped off the blade can slide/settle\n    # for 3-4 s before it rests flat (measured) — a real toss still never counts, because\n    # a thrown slice was never loaded near the plate at all.\n',
     '    # loaded step (5 s at 120 Hz — the no-toss/no-shove contact-history clause).\n'),
    ('robobench/suites/puzzle/scenes/spatula_flip_serve.py',
     "    # Friction pair sized from BOTH ends (round-2 GPU lesson): PhysX combines by AVERAGE, so\n    # blade-bread ~ (0.33 static / 0.29 dynamic). Low enough that the wedge SLIPS under the\n    # payload instead of sticking to it and bulldozing; high enough that the carry has a real\n    # tilt budget: atan(0.33) ~ 18 deg. Bread-pan stays grippier (the pan rig bakes the old\n    # board's 0.6/0.55) so the pan anchors the payload while the blade slides beneath — and\n    # the pan WALL is now a hard anchor no jab can shove the payload past.\n",
     '    # Friction pair (PhysX combines by AVERAGE).\n'),
    ('robobench/suites/puzzle/scenes/spatula_flip_serve.py',
     "        # Legacy surface: the grasp contract's state used to live directly on the scene\n        # (inline machinery era) and existing solutions read it there — forward to the\n        # composed contract. `__getattr__` only fires for attributes not found normally.\n",
     '        # Legacy surface: forward `_gw_*` reads to the composed grasp contract.\n        # `__getattr__` only fires for attributes not found normally.\n'),
    ('robobench/suites/puzzle/scenes/spatula_flip_serve.py',
     "    # Grasp-weld contract constants — the scene's public knobs (solutions read these off the\n    # scene, e.g. `scene.GRASP_PINCH_OFFSET`); the composed GraspWeldContract consumes them.\n",
     "    # Grasp-weld contract constants — the scene's public knobs (e.g.\n    # `scene.GRASP_PINCH_OFFSET`); the composed GraspWeldContract consumes them.\n"),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     '    # of capacity (relaxed from +/-5% — the tight band priced in kinematic-oracle\n    # metering precision, not an arm pressing the plunger through o-ring friction)\n',
     '    # of capacity\n'),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     "    # O-ring feel: the prismatic joint's Coulomb friction + viscous damping (the\n    # plunger is a REAL body now — an arm hooks the thumb plate and pulls). The\n    # friction holds the 0.06 kg plunger wherever it lands, including vertical.\n",
     "    # O-ring feel: the prismatic joint's Coulomb friction + viscous damping. The\n    # friction holds the 0.06 kg plunger wherever it lands, including vertical.\n"),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     '    home_pos: tuple = (-0.06, -0.08)  # lying syringe centre on the surface.\n    # The syringe lies ALONG X (tip west, plate east): a y-lying syringe forces\n    # its plate to y>=+0.09 where every wrist column hits the north guard rail —\n    # unreachable by both arms in all orientation families (jobs 19-27,\n    # 2026-08-11). Lying along x keeps the whole syringe south of the rail wall.\n',
     '    home_pos: tuple = (-0.06, -0.08)  # lying syringe centre on the surface (along x).\n'),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     "    rack_pos: tuple = (0.0, 0.14)  # tube-rack centre (north strip, east\n    # of the home column; the 1.02 m glass tubes must sit outside both arms'\n    # LOW-reach sweeps — park carries dogleg south of them)\n",
     '    rack_pos: tuple = (0.0, 0.14)  # tube-rack centre (north strip)\n'),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     '    res_pos: tuple = (0.14, -0.05)  # reservoir beaker centre: EAST of\n    # the home footprint (17 mm clear of the lying plate end — at x=0.02 the\n    # beaker overlapped the boot pose) at the PROVEN south latitude (y=+0.03\n    # sat in the rack shadow that kills the two-hand seat, job_0038/pod-8;\n    # x=-0.16 jammed the left hand past its own base, job_0034). Warm-pod\n    # validated: boot rest clean, grasp 8.0 mm, seat corridor 2.8 mm.\n',
     '    res_pos: tuple = (0.14, -0.05)  # reservoir beaker centre (east of the home footprint)\n'),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     '        # The plunger is a real body with a rod + thumb-plate collider: a second\n        # arm hooks under the plate and PULLS to draw (the earlier kinematic\n        # plunger metered through a scene scalar, which made the pressing arm\n        # cosmetic — user directive 2026-08-10: the fill must be physical).\n',
     '        # The plunger is a real body with a rod + thumb-plate collider.\n'),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     '    # setting; the prismatic joint IS the metering mechanism now: a second arm\n    # hooks the plate and pulls, joint friction holds it wherever it lands.)\n',
     '    # setting; the prismatic joint IS the metering mechanism.)\n'),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     '        # lying home: tip west, thumb east; joint 0 = plunger fully pushed (its\n        # rod still stands proud of the rim — the hook-grasp handle). Spawned a\n',
     '        # lying home: tip west, thumb east; joint 0 = plunger fully pushed. Spawned a\n'),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     '        """Plunger travel (m): + = drawn out, 0 = fully pushed. MEASURED — the\n        prismatic joint position (a second arm pulling the plate moves it; joint\n        friction holds it wherever it lands)."""\n',
     '        """Plunger travel (m): + = drawn out, 0 = fully pushed: the prismatic joint\n        position (joint friction holds it wherever it lands)."""\n'),
    ('robobench/suites/puzzle/scenes/syringe_dosing.py',
     '        """Thumb-plate centre (world) — the hook-grasp handle on the plunger link."""\n',
     '        """Thumb-plate centre (world) on the plunger link."""\n'),
    ('robobench/suites/puzzle/configs/envs.py',
     '  - syringe:    draw / triple-dose / re-park. Defining embodiment GR1-T2 (bimanual:\n                one hand fixtures the barrel, the other meters the plunger); G1 for\n                curriculum; franka / piper / wxai / bimanual_franka as arm bindings.\n',
     '  - syringe:    draw / triple-dose / re-park (GR1-T2, G1, franka, piper, wxai and\n                bimanual_franka bindings).\n'),
    ('robobench/suites/cutting/scenes/slice_food.py',
     'and the edge level — a top-down pinch on the handle lifts it straight off). **Goal: pick up\n',
     'and the edge level). **Goal: pick up\n'),
    ('robobench/suites/cutting/scenes/slice_food.py',
     'of pieces** — graded by `grader.SliceFoodGrader`. Embodiment-agnostic: the scene knows\n',
     'of pieces**. Embodiment-agnostic: the scene knows\n'),
    ('robobench/suites/cutting/scenes/slice_food.py',
     '    knife_mass: float = 0.18  # a 22 cm chef knife; heavier knives creep in a friction pinch\n',
     '    knife_mass: float = 0.18  # a 22 cm chef knife\n'),
    ('robobench/suites/cutting/scenes/slice_food.py',
     '    # tip (0.22,-0.134)): rest/hold it rotated by this about the blade normal so the EDGE\n    # is level, else the tip grounds 6 cm before the mid-blade\n',
     '    # tip (0.22,-0.134)): the rest lays it rotated by this about the blade normal\n'),
    ('robobench/suites/cutting/scenes/slice_food.py',
     "    # island height — an arm's reach wants its base ~0.55 m behind the cut, past the island's\n    # y=-0.38 edge. None -> no stand.\n",
     '    # island height. None -> no stand.\n'),
    ('robobench/suites/cutting/scenes/slice_food.py',
     '            # firm, straight, 3.2 cm: release as the edge reaches the flesh centre — the\n            # split forms mid-stroke and the knife need not drive on to the board\n',
     '            # firm, straight, 3.2 cm\n'),
    ('robobench/suites/cutting/scenes/slice_food.py',
     '        # RIGID knife (no compliant contact — a strong pinch squashes through a compliant\n        # plate; softness lives in the FOOD), lying edge-level on the rest\n',
     '        # RIGID knife, lying edge-level on the rest\n'),
    ('robobench/suites/cutting/scenes/dice_food.py',
     "  against (all of a plane's pairs released). `plane_aim_w` / `plane_axis_w` give drivers the\n  live aim point and blade yaw for a plane.\n",
     "  against (all of a plane's pairs released).\n"),
    ('robobench/suites/locomanip/scenes/box_to_bin.py',
     "stand-off, and the bin sits to the robot's right where Arena's navigation waypoints send it. See\n",
     "stand-off, and the bin sits to the robot's right. See\n"),
    ('robobench/suites/locomanip/scenes/box_to_bin.py',
     "the MIDDLE board (top 0.763) with 0.69 m of clear air to the board above, so the pick is a\nwaist-height front approach. The bin's",
     "the MIDDLE board (top 0.763) with 0.69 m of clear air to the board above. The bin's"),
    ('robobench/suites/locomanip/scenes/box_to_bin.py',
     '        constraints — the pick pose is where the robot already starts (facing +y, the Arena\n        stand-off), and the place pose is Arena\'s own navigation goal, rotated into this frame."""\n',
     '        constraints — the pick pose is where the robot already starts (facing +y)."""\n'),
    ('robobench/suites/locomanip/scenes/box_to_bin.py',
     '    #: Stage-wide look knobs for visual replay (data_engine render.py --visual_draw).\n',
     '    #: Stage-wide look knobs for visual replay.\n'),
    ('robobench/suites/locomanip/scenes/wheel_carry.py',
     "substrate. Each station reproduces `wheel_pick_place`'s validated standing geometry exactly — a\nrobot at the station's `stand_xy` sees precisely the layout it saw there — so any failure here is\nattributable to the transit, not to a new reach problem.\n",
     'substrate.\n'),
    ('robobench/suites/locomanip/scenes/wheel_carry.py',
     'their own feet instead — the work surfaces do not move, but they become a 0.994 m chest-height pick\nand the inherited reach calibration no longer applies.\n',
     'their own feet instead — the work surfaces do not move, but they become a 0.994 m chest-height pick.\n'),
    ('robobench/suites/locomanip/scenes/wheel_carry.py',
     '    # interior, so it is a genuine "in the bin", not "near it". Inherited verbatim from\n    # `assembly.wheel_pick_place`, where they were measured.\n',
     '    # interior, so it is a genuine "in the bin", not "near it".\n'),
    ('robobench/suites/locomanip/scenes/wheel_carry.py',
     '    # flattens them to one value, and the nominal is the LEATHER rim, so the contact an end effector\n    # meets on the outside of the wheel is unchanged from the asset.\n',
     "    # flattens them to one value (the nominal is the leather rim's).\n"),
    ('robobench/suites/locomanip/scenes/wheel_carry.py',
     "    #: Where the wheel lies, in the PICK TABLE's frame — the same spot it occupies in\n    #: `assembly.wheel_pick_place`, so the grasp transfers.\n",
     "    #: Where the wheel lies, in the PICK TABLE's frame.\n"),
    ('robobench/suites/locomanip/scenes/wheel_carry.py',
     '        These are not constraints — nothing checks them — but they are the two spots at which this\n        scene reproduces `assembly.wheel_pick_place`\'s validated geometry, which is what\n        `describe()` hands the agent as a starting guess."""\n',
     '        These are not constraints — nothing checks them — and `describe()` hands them to the agent\n        as a starting guess."""\n'),
    ('robobench/suites/locomanip/scenes/wheel_carry.py',
     '    #: Stage-wide look knobs for visual replay (data_engine render.py --visual_draw).\n',
     '    #: Stage-wide look knobs for visual replay.\n'),
    ('robobench/suites/locomanip/__init__.py',
     '`assembly.wheel_pick_place` (same assets, same measured reach geometry at both ends, one basket\nmoved out of range)',
     '`assembly.wheel_pick_place` (same assets, one basket\nmoved out of range)'),
    ('robobench/suites/cutting/configs/envs.py',
     '  - "cutting.<scene>.franka.joint"     — direct joint position targets (the first slice\n                                          solution ran its own IK on top of this)\n',
     '  - "cutting.<scene>.franka.joint"     — direct joint position targets\n'),
    ('robobench/core/grader.py',
     '            # would poison a "once" milestone for the rest of the run (seen 2026-09-04, box_to_bin\n            # `carried` when scoring reached-state logs).\n',
     '            # would poison a "once" milestone for the rest of the run.\n'),
    # END generated scene edits
]
# Code edits, each confined by its kind: "describe" may change only the scene's describe() method (the task text the
# agent reads: it states the goal and the API, never working poses), "string" only string constants.
B2B, WHC = "robobench/suites/locomanip/scenes/box_to_bin.py", "robobench/suites/locomanip/scenes/wheel_carry.py"
CODE_EDITS: list[tuple[str, str, str, str]] = [
    (B2B, "describe",
     "            f\"robot itself does — and the table stands off the shelf's axis, so the route turns. \"\n"
     "            f\"Working poses (x, y, heading): about ({px:.2f}, {py:.2f}, {ph:+.2f}) for the pick — \"\n"
     "            f\"the start pose already faces the shelf squarely — and ({tx:.2f}, {ty:.2f}, {th:+.2f}) \"\n"
     "            f\"for the place; they are a starting guess, not a requirement.\"\n",
     "            f\"robot itself does — and the table stands off the shelf's axis, so the route turns.\"\n"),
    (B2B, "string",
     "corrugated board's friction varies \"\n"
     "                                   \"down from that with surface wear/dust, not up; the low end is \"\n"
     "                                   \"where the two-palm grip actually has to work\"},\n",
     "corrugated board's friction varies \"\n"
     "                                   \"down from that with surface wear/dust, not up\"},\n"),
    (WHC, "describe",
     "            f\"there if the robot itself does. Working positions that face each table squarely are \"\n"
     "            f\"about ({sx:.2f}, {sy:.2f}) for the pick and ({tx:.2f}, {ty:.2f}) for the place; they \"\n"
     "            f\"are a starting guess, not a requirement.\"\n",
     "            f\"there if the robot itself does.\"\n"),
]
# Shipped files that are not task code and hand out a working controller (a reach driver whose example is a task).
DROP_FILES = ("robobench/scripts/robot_binding_smoke.py",)
# Phrases that must not survive in the files the scrub edits (a pointer to a solve-calibrated number).
STRIP_GLOB = "robobench/suites/*/configs/envs.py"
NEUTRAL_DOC = '"""(binding notes removed from this image)"""'
BANNED = ("calibrated grasp", "calibrated stance", "release annulus", "pink_probe", "reach envelope", "reach map",
          "azimuth\n    #: 180", "pick radius", "grasp cell", "REORIENT_STUCK", "robot_binding_smoke before",
          "existing solutions", "demos' solve", "gravity-seat", "IK campaign", "solves and the grader",
          "grasps and releases reliably", "grasps AND releases reliably", "palm CAGE", "steering for",
          "first calibration", "force smoke", "key smoke")


# every scene module the shipped tasks build (the check fails when one is missing)
SCENE_MODULES = tuple(f"robobench/suites/{m}.py" for m in (
    "assembly/scenes/bulb_assembly", "puzzle/scenes/stack_blocks", "locomanip/scenes/fruit_delivery",
    "packing/scenes/fruits_on_plate", "assembly/scenes/allen_bolt_assembly", "assembly/scenes/ikea_table_assembly",
    "assembly/scenes/nut_thread_assembly", "assembly/scenes/pc_gpu_assembly", "assembly/scenes/pc_gpu_ram_assembly",
    "assembly/scenes/pc_motherboard_assembly", "assembly/scenes/pc_ram_assembly", "packing/scenes/pen_holder",
    "packing/scenes/tool_packing", "packing/scenes/egg_carton", "packing/scenes/clear_organic_objects",
    "puzzle/scenes/classify_objects", "puzzle/scenes/push_shapes", "puzzle/scenes/coffee_service",
    "puzzle/scenes/syringe_dosing", "cutting/scenes/slice_food", "cutting/scenes/dice_food",
    "locomanip/scenes/box_to_bin", "locomanip/scenes/wheel_carry"))


def code_fingerprint(src: str) -> str:
    """The module's AST without docstrings: a comment or docstring edit leaves it unchanged."""
    tree = ast.parse(src)
    for node in ast.walk(tree):
        body = getattr(node, "body", None)
        if isinstance(body, list) and body and isinstance(body[0], ast.Expr) \
                and isinstance(getattr(body[0], "value", None), ast.Constant) and isinstance(body[0].value.value, str):
            body[0] = ast.Pass()
    return ast.dump(tree)


def _outside_describe(src: str) -> str:
    """The module's AST with every describe() method's body emptied (an edit inside describe() leaves it unchanged)."""
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and node.name == "describe":
            node.body = [ast.Pass()]
    return ast.dump(tree)


def _without_strings(src: str) -> str:
    """The module's AST with every string constant blanked (an edit of string text only leaves it unchanged)."""
    tree = ast.parse(src)
    for node in ast.walk(tree):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            node.value = ""
    return ast.dump(tree)


def _usd_body(src: str) -> list[str]:
    """A .usda layer without its `#` comment lines (a comment edit leaves it unchanged)."""
    return [x for x in src.splitlines() if not x.lstrip().startswith("#")]


def strip_comments(src: str) -> str:
    """Remove every comment and replace every docstring but the module's with NEUTRAL_DOC; code unchanged."""
    import io
    import tokenize

    lines = src.splitlines(keepends=True)
    cut: dict[int, int] = {}                       # 1-based row -> column where its comment starts
    for tok in tokenize.generate_tokens(io.StringIO(src).readline):
        if tok.type == tokenize.COMMENT:
            cut[tok.start[0]] = tok.start[1]
    docs = []                                      # (first row, last row, indent) of each non-module docstring
    for node in ast.walk(ast.parse(src)):
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) and node.body:
            b = node.body[0]
            if isinstance(b, ast.Expr) and isinstance(b.value, ast.Constant) and isinstance(b.value.value, str) \
                    and b.value.value != NEUTRAL_DOC.strip('"'):
                docs.append((b.lineno, b.end_lineno, b.col_offset))
    out, skip = [], set()
    for first, last, col in docs:
        skip.update(range(first + 1, last + 1))
    doc_first = {first: col for first, _last, col in docs}
    for i, line in enumerate(lines, start=1):
        if i in skip:
            continue
        if i in doc_first:
            out.append(" " * doc_first[i] + NEUTRAL_DOC + "\n")
            continue
        if i in cut:
            head = line[:cut[i]].rstrip()
            if not head:
                continue                           # a comment-only line goes entirely
            line = head + "\n"
        out.append(line)
    return "".join(out)


def plan(root: Path) -> list[str]:
    """What a run would still change (empty after a scrub)."""
    todo = [f"remove {p.name}" for p in sorted(root.iterdir()) if p.name not in KEEP_TOP]
    suites = root / "robobench" / "suites"
    for s in sorted(suites.iterdir()) if suites.is_dir() else []:
        if s.is_dir() and s.name in DROP_SUITES:
            todo.append(f"remove suites/{s.name}")
        for d in DROP_SUITE_DIRS:
            if (s / d).exists():
                todo.append(f"remove suites/{s.name}/{d}")
    for rel, old, _new in EDITS:
        if old in (root / rel).read_text():
            todo.append(f"edit {rel}")
    for rel, _kind, old, _new in CODE_EDITS:
        if old in (root / rel).read_text():
            todo.append(f"code edit {rel}")
    todo += [f"remove {rel}" for rel in DROP_FILES if (root / rel).exists()]
    for path in sorted(root.glob(STRIP_GLOB)):
        src = path.read_text()
        if strip_comments(src) != src:
            todo.append(f"strip {path.relative_to(root)}")
    todo += [f"pycache {p}" for p in root.rglob("__pycache__")]
    return todo


def scrub(root: Path) -> None:
    for p in root.rglob("__pycache__"):           # bytecode built before the removal keeps the old code
        shutil.rmtree(p, ignore_errors=True)
    for p in sorted(root.iterdir()):
        if p.name not in KEEP_TOP:
            shutil.rmtree(p) if p.is_dir() and not p.is_symlink() else p.unlink()
    suites = root / "robobench" / "suites"
    for s in sorted(suites.iterdir()):
        if not s.is_dir():
            continue
        if s.name in DROP_SUITES:
            shutil.rmtree(s)
            continue
        for d in DROP_SUITE_DIRS:
            if (s / d).exists():
                shutil.rmtree(s / d)
    for rel, old, new in EDITS:
        path = root / rel
        src = path.read_text()
        n = src.count(old)
        if n == 0:
            continue                               # already scrubbed (idempotent)
        if n != 1:
            raise SystemExit(f"scrub: {rel}: expected the block once, found {n}")
        out = src.replace(old, new)
        if rel.endswith(".py") and code_fingerprint(out) != code_fingerprint(src):
            raise SystemExit(f"scrub: {rel}: an edit changed code, not only comments")
        if rel.endswith(".usda") and _usd_body(out) != _usd_body(src):
            raise SystemExit(f"scrub: {rel}: an edit changed USD content, not only `#` comments")
        path.write_text(out)
    for rel, kind, old, new in CODE_EDITS:
        path = root / rel
        src = path.read_text()
        n = src.count(old)
        if n == 0:
            continue
        if n != 1:
            raise SystemExit(f"scrub: {rel}: expected the code block once, found {n}")
        out = src.replace(old, new)
        same = _outside_describe if kind == "describe" else _without_strings
        if same(out) != same(src):
            raise SystemExit(f"scrub: {rel}: a {kind} edit changed more than its {kind}")
        path.write_text(out)
    for rel in DROP_FILES:
        if (root / rel).exists():
            (root / rel).unlink()
    for path in sorted(root.glob(STRIP_GLOB)):
        src = path.read_text()
        out = strip_comments(src)
        if code_fingerprint(out) != code_fingerprint(src):
            raise SystemExit(f"scrub: {path}: stripping changed code")
        path.write_text(out)


def check(root: Path) -> list[str]:
    bad = plan(root)
    for rel in sorted({e[0] for e in EDITS} | {e[0] for e in CODE_EDITS}
                      | {str(p.relative_to(root)) for p in root.glob(STRIP_GLOB)}):
        txt = (root / rel).read_text()
        bad += [f"banned phrase {b!r} in {rel}" for b in BANNED if b in txt]
    # the suites still import (registration only; no simulator): every scene module the tasks need is present
    for rel in SCENE_MODULES:
        if not (root / rel).is_file():
            bad.append(f"missing {rel}")
    return bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", type=Path)
    ap.add_argument("--check", action="store_true", help="change nothing; fail if a rule would still apply")
    a = ap.parse_args()
    root = a.root.resolve()
    if not a.check:
        # a fresh checkout must still carry every block (else upstream moved and the rules need a look)
        for rel, old, _new in EDITS + [(e[0], e[2], e[3]) for e in CODE_EDITS]:
            if old not in (root / rel).read_text() and _new not in (root / rel).read_text():
                raise SystemExit(f"scrub: {rel}: neither the original nor the scrubbed block is present")
        scrub(root)
    bad = check(root)
    for b in bad:
        print(f"scrub check: {b}", file=sys.stderr)
    print(f"scrub: {'FAILED' if bad else 'ok'} ({len(EDITS)} scene-comment edits, {len(CODE_EDITS)} describe/string "
          f"edits, "
          f"{len(list(root.glob(STRIP_GLOB)))} binding files stripped, graders/smokes removed)")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
