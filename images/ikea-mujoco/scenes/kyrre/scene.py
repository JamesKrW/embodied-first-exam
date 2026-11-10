"""KYRRE stool (AA-2118871-4, step 1: three legs, six screws) in MuJoCo 3.14 with two Franka Panda arms.

Load:   import scene; S = scene.load()      # run with this directory on sys.path (or importlib)
        S.model, S.data                     # MuJoCo model/data, reset to keyframe 'home'
        S.data.ctrl[...] = ...              # arms: left_actuator1..7 (joint position targets, rad),
                                            #       left_actuator8 gripper (0 closed .. 255 open), same for right_
        S.step(n)                           # n x 1 ms physics steps (+ retention mechanism + success bookkeeping)
        S.success()                         # dict, 'success' True when the source criteria hold
        S.mech.metrics / S.mech.active(key) # per-operation gate values / weld state
Parts (free bodies): seat, leg_1..3, screws 130915_<leg>_<k> (2 per leg).
Each leg still carries the idle spring jaw of the removed press-in pin (jaw_s2_press_in_<leg>_0, slide joint).
The seat lies upside-down; operations (task.json, strictly in order): place leg 1, screw 1-1, screw 1-2,
leg 2, ..., screw 3-2; then release it (the stool stays upside-down). The press-in pins and the turn-over
were removed by user decision on 2026-10-16.
Non-contact mechanisms: one retention weld per operation, switched on by the source acceptance gates
(kyrre_mechanism.py); the jaw springs. Screws thread by SDF contact (0.5 mm pitch, right-hand).
Arm TCP sites: left_tcp / right_tcp (fingertip centre). Task details: task.json.
The one exception to the prism rule: native MuJoCo cylinders (the 32-facet prism rule off for this scene; NATIVE_CYLINDERS).
"""
import os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
if HERE not in sys.path:
    sys.path.insert(0, HERE)
import ikea_runtime as _rt
from ikea_runtime import Scene
from kyrre_mechanism import KyrreMechanism
import success as _success

# EXCEPTION (kyrre only; approved by the owner 2026-10-19): this scene keeps MuJoCo's native cylinders. The benchmark's
# rule compiles every robot-touchable cylinder as a 32-facet prism (ikea_runtime.cylinder_prisms); for kyrre the rule is
# switched off here, at this scene's build only (the shared runtime and every other scene unchanged). The reason: a
# 203-episode stress A/B on the same seeds and states (scripts/ikea_mujoco/REVIEW.md): non-jerk BREAKDOWN 1.15 % native vs
# 2.3 % with prisms, no breakdown on a cylinder geom, and the author's demo assembles the stool only without the rule.
# scene.cylinder_prisms is then [] and the welded-part contact switch-off (ikea_runtime) works as everywhere else.
NATIVE_CYLINDERS = True

def load(keyframe='home'):
    if not NATIVE_CYLINDERS:
        return Scene(HERE, keyframe=keyframe, mechanism=KyrreMechanism, monitor=_success.Monitor)
    rule = _rt.cylinder_prisms
    _rt.cylinder_prisms = lambda spec: []
    try:
        return Scene(HERE, keyframe=keyframe, mechanism=KyrreMechanism, monitor=_success.Monitor)
    finally:
        _rt.cylinder_prisms = rule
