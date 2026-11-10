# RoboARC (simulator package `topogym`) · clock_assembling — frozen instance
You control a simulated robot (**two 6-DoF ALOHA (ViperX) arms with parallel-jaw grippers on a fixed base**; RoboARC on MuJoCo 3.14.0 physics, Nyx rendering) via Python.
**Task:** Assemble the watch movement's gear train: the four wheels on their pivots in the main plate, the train bridge over them on its pillars with every upper pivot in its hole, the two bridge screws tight; then wind the barrel through its ratchet, let go and watch the fourth wheel run.
## Success requirements
- Seat the barrel, the center wheel, the third wheel and the fourth wheel on their lower pivots in the main plate (each wheel no more than 1 mm above its seat); a wheel seats once its lower pivot is down in its hole: the wheel at most 0.8 mm above its seat, within 0.6 mm of the hole's axis and tilted less than 3 degrees. Each wheel's pinion must mesh with the previous wheel's teeth (barrel, center, third, fourth); a wheel lowered with its pinion's teeth tip to tip on the other wheel's teeth stops until it is turned a little.
- Every upper pivot (the barrel: its arbor) inside its hole in the bridge, at least 1 mm deep and within the hole's radius of its axis.
- The bridge seated on its pillars: within 0.5 mm of its seat height, 0.5 mm of its seat position and tilted less than 0.5 degrees.
- Both screws in the bridge's two screw holes (either screw in either hole, one to a hole) and turned clockwise by 2 full turns there (no thread is simulated: the turns are counted only while the screw's head sits on the bridge over its hole, its underside between 0.3 mm below and 0.8 mm above the bridge's top and its axis within 0.6 mm of the hole's; counterclockwise turns count back; a screw moved to the other hole starts again). Once both are tight, every wheel is seated, every upper pivot is in its bridge hole and the bridge is seated (height and tilt), the bridge is fixed: screw the bridge down after the wheels, not before.
- After the bridge is fixed, wind the barrel: turn its square (the ratchet under it turns with it) counterclockwise seen from above; the click lets it turn that way only and holds it one ratchet tooth (18 degrees) behind the furthest point reached. Wind it at least 120 degrees (as held by the click), then stop winding and let go of the square: the mainspring drives the barrel and the train, and any further advance of the square counts as winding again. Half a second after the last winding the rules time a 5 s run: the fourth wheel must turn at least 3.5 times in it, spend at most 0.2 s slower than a tenth of its free-run speed (the speed the mainspring's present windup gives the train against its own damping and friction), and turn 450 times as much as the barrel in the opposite direction, within 2 %. The run counts only for the mainspring alone: from the last winding to the end of the run neither the hands (any part of the robot) nor the probe may touch the barrel or a wheel, and the barrel must turn the way the mainspring drives it; otherwise the run does not count until the barrel is wound again. Winding again restarts the run.
- Holding parts is allowed, apart from the run above.
- Keep all final conditions satisfied together for at least 0.5 s of simulation time. If any final condition stops holding, the confirmation interval starts again. A brief qualifying pose is not enough; let the simulation advance through the full interval.
## Task failure conditions
- A task piece falling more than 100 mm below the tabletop ends the episode in failure. Contact penetration never ends the episode by itself: a success with a task contact deeper than 3 mm, or deeper than 2 mm for at least 0.25 s (100 consecutive physics ticks), is not credited automatically but reviewed, with the recorded evidence and frames (for a finger contact with a held movable piece: 4 mm, or 2 mm for 0.25 s; holding requires both fingers of the same hand to contact the same piece, or an active assisted grasp by that hand); a clean success is credited automatically. A task contact deeper than 10 mm is a simulator breakdown (Grader details).
## Grader details (in addition to the requirements above)
- Contact penetration is read from the contacts at the end of every physics tick (2.5 ms; 10 ticks per control step). "Held" is decided anew at every tick: a zero-force touch counts, and if one finger loses contact (with no active assisted grasp), the other finger is back at the non-held limits in the same tick.
- Robot safety (this benchmark's tooling): no gripper target goes lower than 15 mm below the tabletop; a commanded target that would bring any part of a gripper lower is stopped there, on the commanded path. The fingertips reach about 16 mm beyond the tool frame (TCP) along its approach axis, open or closed. Arm drive torques are capped (the effort limits in robot_spec), so an arm pushing against an obstacle stalls at that torque. An arm with any part more than 5 mm deep in a fixed object (a fixture, the table) is stopped where it is: its joint targets become its measured joints for that control step, so it stops pushing (the safety readings report these stops as static_guard).
- Forcing is recorded, not judged: an arm drive (not a gripper drive) at 95% of its torque cap or more while that arm's gripper, or a part it holds, presses on anything else with 20 N or more in total is forcing; 0.5 s of it on every physics tick is a forcing event in the evidence (`s.safety()`, `s.status()["forcing"]`), shown to the reviewer when a success is reviewed. Forcing does not end the episode and does not change the grade by itself.
- Simulator breakdown (a non-finite state or a task contact deeper than 10 mm): the episode ends as an infrastructure error, not your failure, unless an arm was forcing (even for one tick) within the 0.25 s before it: then it is your failure (forced_breakdown: what broke and what forced it).
- The rules (seating, the tooth-on-tooth gates, meshing, the screws' turns, the bridge's fixing, the click and the run test's timing) are applied after every physics tick (2.5 ms); the success conditions are checked at the end of every tick. The mainspring is a spring between the barrel's arbor and drum; the click is a limit on the arbor's turn that the rules move up as it is wound.
- Grasp regions (assisted grasping, see Grippers above; one sphere per graspable part): barrel: radius 65.8 mm; center: radius 42 mm; third: radius 33.8 mm; fourth: radius 33.8 mm; bridge: radius 120.6 mm; screw1: radius 20.9 mm; screw2: radius 20.9 mm; probe: radius 56.1 mm; barrel_arbor: radius 30.6 mm. Both finger contacts must lie inside the part's sphere; a part touched elsewhere is grasped physically. Each region's centre (in its link's frame) is in `/app/instance.json` under `simulator.grasp_assist.targets`.
- A failure is reported in `s.status()["episode_end"]` as "A task body fell below the table" or "forced_breakdown: <what>" (<what>: the readable reason); `s.safety()` reports forcing, contact depths, volume ratios and table-guard stops against these limits, and `s.status()["review"]` the deep task contacts that would hold a success for review.
The scene objects are the task's entities (`s.objects()`: `clock`). The movement, its stands and every part are one entity, `clock`: `s.objects()` and `s.object_pose("clock")` report its base link, the barrel (its drum). Each part is a link of it, under these link keys: `"barrel"`, `"center"`, `"third"`, `"fourth"`, `"bridge"`, `"screw1"`, `"screw2"`, `"probe"`, `"barrel_arbor"` (`"barrel_arbor"`: the barrel's arbor with the ratchet and the square, on the mainspring inside the drum); a part's pose is `s.env._world.read_state()["articulations"]["clock"]["links"][key]` (`position_m`, `quaternion_wxyz`, `linear_velocity_m_s`, `angular_velocity_rad_s`), e.g. `["links"]["bridge"]`, and the rules' state (seated wheels, gates, meshes, screw turns, the bridge fixed) is `s.env._world.read_state()["mechanism"]`.
Implementation reference: `ClockAssemblingGearTrainTask.evaluate_state` (module `topogym.tasks.clock_assembling.common`, class `ClockAssemblingTask`).
This is a partial implementation excerpt; delegated helpers, passage history and the shared release/confirmation
checks also apply. The complete public goal requirements are stated above.
```python
    def evaluate_state(self, state):
        rules = state.get("mechanism")
        if rules is None:
            raise RuntimeError("clock_assembling runs on MuJoCo: the world reads no mechanism")
        _, _, _, failure = self.rigid_measurements(state)
        if failure:
            return False, failure, {}
        links = state["articulations"][geo.ENTITY]["links"]
        poses = {n: self._part_pose(links, n) for n in geo.PARTS}
        for n, (p, R) in poses.items():
            if not np.isfinite(np.r_[p, R.ravel()]).all():
                return False, "Non-finite physical state", {}
            if (p + geo.part_points(n) @ R.T)[:, 2].min() < self.table_height_m - .1:
                return False, "A task body fell below the table", {}
        t = self.config["terminal"]
        o = self.movement_origin()
        bp, BR = poses["bridge"]
        bp = bp - o
        metrics = dict(evaluation_status="available", holding_allowed=True)
        seated = in_bridge = True
        for n in geo.WHEELS:
            p, R = poses[n]
            p = p - o
            seated_n = bool(rules[f"{n}_seated"] and p[2] <= t["seat_height_m"])
            z = geo.SHOULDER_Z + geo.UP_PIVOT_L if n != "barrel" else geo.H_BRIDGE + geo.T_BRIDGE
            v = p + R @ np.array([0., 0., z]) - (bp + BR @ np.array([geo.POS[n][0], geo.POS[n][1], 0.]))
            h = float(v @ BR[:, 2])
            lateral = float(np.linalg.norm(v - h * BR[:, 2]))
            radius = geo.PIVOT_HOLE_R if n != "barrel" else geo.BARREL_ARBOR_R + .0005
            in_n = bool(lateral <= radius and h >= t["pivot_depth_m"])
            metrics |= {f"{n}_seated": seated_n, f"{n}_in_bridge": in_n, f"{n}_pivot_lateral_m": lateral,
                        f"{n}_pivot_depth_m": h}
            seated, in_bridge = seated and seated_n, in_bridge and in_n
        tilt = float(np.arccos(np.clip(BR[2, 2], -1., 1.)))
        bridge_seated = bool(abs(bp[2] - geo.H_BRIDGE) <= t["bridge_height_m"] and np.hypot(bp[0], bp[1]) <= t["bridge_lateral_m"]
                             and tilt < t["bridge_tilt_rad"])
        fixed = bool(rules["fixed"] and rules["screw1_tight"] and rules["screw2_tight"])
        meshed = all(rules[f"{n}_meshed"] for n in geo.PARENT)
        wound = float(rules["run_wound_rad"])
        turns, stall, ratio = float(rules["run_fourth_turns"]), float(rules["run_stall_s"]), float(rules["run_ratio"])
        ran = bool(rules["run_done"] and rules["run_ratio_measured"] and wound >= t["wind_min_rad"]
                   and turns >= t["run_turns_min"] and stall <= t["run_stall_max_s"]
                   and abs(ratio / geo.TOTAL_RATIO - 1.) <= t["ratio_tolerance"])
        metrics |= dict(bridge_seated=bridge_seated, bridge_height_error_m=float(bp[2] - geo.H_BRIDGE),
                        bridge_tilt_rad=tilt, bridge_fixed=fixed, meshed=meshed, wound_rad=wound, run_done=bool(rules["run_done"]),
                        run_fourth_turns=turns, run_stall_s=stall, run_ratio=ratio, run_test=ran,
                        run_void=bool(rules["run_void"]),
                        screw1_turns=float(rules["screw1_turns"]), screw2_turns=float(rules["screw2_turns"]))
        return bool(seated and in_bridge and bridge_seated and fixed and meshed and ran), None, metrics
```
This is a *fixed scene instance*: object poses and the robot's start pose never change between resets. You may reset
and retry as many times as you want. Your deliverable is **one replayable trajectory**, not a general policy.
## Deliverable
Write the final trajectory to `/app/output/trajectory.npz` with `Session.save_trajectory(...)` (below).
It is replayed open-loop once from the same initial state in a fresh process and scored 1 if the task is
completed at the end of the replay, else 0. Also leave the script that produced it at `/app/output/gen.py`.
## Tooling (already installed; run `python` from any directory)
```python
from rcb_topogym import Session
s = Session("clock_assembling", instance="/app/instance.json")   # builds the frozen scene (instance.json) and compiles kernels
s.reset()                          # back to the frozen initial state (unlimited; a restore of the first recorded state)
p = s.primitives()                 # thin wrappers of the low-level action; each has a docstring: help(p["move_ee"])
s.objects()                        # objects: name -> position, orientation_wxyz, euler_xyz, bounds, size, velocity
p["move_ee"]("left", pos, quat_wxyz, linear=3)   # IK on the robot's model to a TCP pose, then joint interpolation
p["move_joints"]("left", q_arm); p["set_gripper"]("left", 0.0); p["open_gripper"]("left"); p["hold"](10)
s.ee_pose("left"); s.joint_positions(); s.ik("left", pos, quat_wxyz)    # kinematics helpers (arms: ['left', 'right'])
s.render_frame("cam_high")         # RGB uint8 (480, 848, 3) from one camera (s.VIDEO_CAMERAS); s.render_grid()
s.render_depth("cam_high")         # float32 (480, 848) z-depth in metres (0 = no reading), pixel-aligned with render_frame
s.camera_info()                    # per camera: intrinsic_cv, extrinsic_cv (world -> camera, OpenCV axes), cam2world_gl, mount
s.robot_spec()                     # joints (limits), grippers (the grasping rule), PD gains, tool frames ['left_tcp', 'right_tcp']
s.kinematics({"op": "fk", "q": q})   # stateless FK / IK on the robot's model; {"op": "ik", "targets": {"left_tcp": pose7}, "seed": q}
s.joint_velocities(); s.joint_efforts(); s.gripper_width("left")   # measured state in action layout (efforts: the PD drives')
s.object_pose(name); s.object_bounds(name); s.state_sha()          # one object's pose / AABB; the state hash that replays compare
p["object_in_hand"]("left"); p["set_joint_targets"](row, steps=1); p["gripper_quat"](yaw, approach)   # help(p[...])
s.step(action)                     # one low-level action (14 entries, below) = one control step (1/40 s)
s.record_start(); ...; actions = s.record_stop()   # (T, 14) actions, one row per control step
s.status()                         # {'success', 'progress', 'sim_steps', 'resets', 'state_sha', 'episode_end', 'review', 'forcing'}
s.safety()                         # since the last call: forcing / deepest contacts / rope volume / table-guard stops vs the limits
s.replay(actions)                  # resets, replays open-loop, returns status  -> use this to self-check
s.save_trajectory(actions, "/app/output/trajectory.npz")
s.render_video(actions, "/app/output/check.mp4")   # replays from the frozen state and writes a tiled camera video
```
The primitives are deliberately thin (IK moves, joint moves, gripper, hold): grasping, placing and every other skill
are yours to build from them.
CLI equivalent of the grader:
`python /opt/tools/rcb_topogym.py replay --task clock_assembling --instance /app/instance.json --traj /app/output/trajectory.npz`
## Conventions
- World frame: +z up. Quaternions are **WXYZ**. The ee_pose action and the proprio TCP pose are given in this world frame.
- Action (14): arm `left`: 6 joint targets (rad), then `left_native_gripper` = native drive target (m, limits 0.002 .. 0.037; open/closed targets in robot_spec); arm `right`: 6 joint targets (rad), then `right_native_gripper` = native drive target (m, limits 0.002 .. 0.037; open/closed targets in robot_spec); absolute position targets tracked by PD controllers, out-of-limit
  values clipped. `s.hold_action()` is the no-op.
- Control runs at 40 Hz (10 physics step(s) per control step). There is no step budget, but a trajectory may have at most
  180000 rows (75.0 min of simulated time): the verifier has to replay it within its own time limit, so `s.step` stops
  after that many control steps since the last reset, and a longer trajectory is rejected as invalid (reward 0). The time limit
  (`## Time limit`) is your only budget. One control step takes about 0.08 s of wall clock here, so budget your experiments.
- Grippers: assisted grasping (contact_weld_v1): a gripper whose command moves towards closed takes hold of a graspable part (barrel, barrel_arbor, bridge, center, fourth, probe, screw1, screw2, third) once both fingers touch the same part inside its grasp region, the tool frame within 15 cm of it and the penetration at most 2 mm, for 0.02 s; the part then follows the gripper rigidly. Any command that opens the gripper (by more than 0.2 % of its travel) lets it go. A hold that breaks (the fingers open more than 15 %, the part drifts 1 cm / 0.15 rad from its held pose, or the hold has to push or pull its part with more than 60 N averaged over 20 ms) is lost, and the gripper must open before it can take hold again; that force is reached when, for example, a held part is pressed hard into another object. Two grippers never hold the same part; other objects are grasped physically (friction); the grippers start empty and nearly closed: open a gripper, then close it on a part to take hold.
- Tool frame (TCP, the ee pose and IK frame): a fixed point near the closed finger pads; approach axis (hand -> fingertips) = local [1.0, 0.0, -0.023], finger-closing axis = local [0.0, 1.0, 0.004]. The ee_pose action, the proprio tcp pose and the IK use this frame.
  `p["top_down_quat"](yaw)` builds a vertical approach whose fingers close along `yaw`; `p["gripper_quat"](yaw, approach)` any other.
- The task succeeds once its predicate has held continuously for 0.5 s; that, or one of its failure predicates,
  ends the simulated episode: later steps do not change the execution record or move the scene (`s.status()["episode_end"]`), and
  `s.reset()` starts over.
- One `Session` per process (one simulator per process). Building the scene and compiling its kernels can take minutes;
  every `Session`, including the one a replay or render CLI command starts, holds its own scene in GPU memory, so keep
  one alive at a time. Do not call `s.close()` before writing every output.

## External material
You may install packages you need (pip, apt), and you may read anything installed in your container, the simulator's and this benchmark's own source code included. Please solve the task yourself: do not search the web for, or download, a ready-made solution to it, such as published or reference solutions, recorded demonstrations, or policies trained for it. This is an honour rule; trials are reviewed. The network reaches your model's API and the package registries (PyPI, Debian / Ubuntu archives) only.

## Resources
Your container: **16 CPUs**, **30 GiB of memory**, **8192 processes and threads**, **10 GiB** of disk (new files under `/app`, `/tmp` and `/root`) and **one GPU, 12 GiB of its memory**. CPU only slows you down; a process that goes over any other cap can be killed. Usage and caps: `/run/rcb-limits/usage.json`.

## Time limit
You have 10800 seconds to complete this task. Do not cheat by using online solutions or hints specific to this task.

This is wall-clock time from the start of your session; the time a simulator takes to start counts. When it is up you are stopped without warning and only what is in `/app/output/` at that moment is graded, so save a deliverable as soon as you have one and overwrite it whenever you have a better one. Your session started when you received this task. `/run/rcb-limits/usage.json`, rewritten every few seconds, gives the time left on this limit (`time_left_s`) and its end (`deadline_utc`), counted from a little before your session started, so it never gives you more time than you have: read it (or run `date`) before you decide that time is short, never guess. Waiting counts too: a tool call that sends input to a process that keeps running (an interactive interpreter, a simulator loop, a server) can wait for the whole wait or yield time you give it even when the answer is printed at once, so give such calls a short wait and poll again if the output is not complete, or run one-shot scripts that exit when they are done. Installing packages is not cheating (`## External material` above).
