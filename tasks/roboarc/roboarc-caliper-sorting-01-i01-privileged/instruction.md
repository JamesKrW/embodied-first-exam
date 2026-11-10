# RoboARC (simulator package `topogym`) · caliper_sorting — frozen instance
You control a simulated robot (**two 6-DoF ALOHA (ViperX) arms with parallel-jaw grippers on a fixed base**; RoboARC on MuJoCo 3.14.0 physics, Nyx rendering) via Python.
**Task:** Measure the sleeves with the caliper and put every sleeve into the bin whose label gives its two caliper readings, then briefly hold everything still.
## Success requirements
- Four sleeves stand in the rack. They look alike and differ only in their inside and outside diameters. Each of the four bins carries a label 'ID r1 / OD r2' with two readings of this caliper: r1 with its knife-edge inside jaws spread in a sleeve's bore, r2 with its outside jaws closed on the sleeve's outside. A sleeve belongs in the bin whose two readings both match its own (within 0.1). A bin may stay empty; each bin holds two sleeves standing.
- The caliper is fixed face up on a stand, its jaws out over the table: it cannot be picked up. Bring each sleeve to its jaws, set the jaws with the slider and read the vernier.
- The caliper is not graduated in millimetres. Its scale is in its own unit (one main-scale division; the vernier reads tenths of it); the size of that unit and the reading with the jaws closed are this caliper's own and are not given, so a size measured in millimetres any other way cannot be matched to a label: the readings must come from the scale.
- The slider is locked at the start. Its locking screw is the knurled knob on the slider's front face, left of the vernier on the jaws' side; a white line on its top shows its angle. It turns between two stops 45 degrees apart, and clockwise (seen from the scale side) tightens. Within 10 degrees of the loose stop the slider moves freely (about 1.5 N of friction); from 20 degrees on it is locked where it stands and does not move however hard it is pushed. Turn the knob by pinching it and rotating the gripper about its axis: it turns by friction only and is not a grasp target.
- Every sleeve's centre must end inside the bin of its class: within 40 mm of the bin's centre in both horizontal directions (the bin's 80 mm x 80 mm cavity) and 3 mm to 40 mm above the tabletop.
- Every sleeve must be released: no sleeve may still be held by an assisted grasp, and no part of the robot (arm, gripper or finger) may press on a sleeve with more than 0.05 N. Whether the caliper is locked at the end does not matter.
- Keep every sleeve's linear speed at most 1 mm/s and angular speed at most 0.02 rad/s.
- Keep all final conditions satisfied together for at least 0.5 s of simulation time. If any final condition stops holding, the confirmation interval starts again. A brief qualifying pose is not enough; let the simulation advance through the full interval.
## Task failure conditions
- A task piece falling more than 100 mm below the tabletop ends the episode in failure. Contact penetration never ends the episode by itself: a success with a task contact deeper than 3 mm, or deeper than 2 mm for at least 0.25 s (100 consecutive physics ticks), is not credited automatically but reviewed, with the recorded evidence and frames (for a finger contact with a held movable piece: 4 mm, or 2 mm for 0.25 s; holding requires both fingers of the same hand to contact the same piece, or an active assisted grasp by that hand); a clean success is credited automatically. A task contact deeper than 10 mm is a simulator breakdown (Grader details).
- A sleeve left in a bin that is not its own (its centre inside that bin as above, released as above and still at the speeds above) for 0.5 s ends the episode in failure. When you set a sleeve down to take it again, set it on the table, not in another bin.
## Grader details (in addition to the requirements above)
- Contact penetration is read from the contacts at the end of every physics tick (2.5 ms; 10 ticks per control step). "Held" is decided anew at every tick: a zero-force touch counts, and if one finger loses contact (with no active assisted grasp), the other finger is back at the non-held limits in the same tick.
- Robot safety (this benchmark's tooling): no gripper target goes lower than 15 mm below the tabletop; a commanded target that would bring any part of a gripper lower is stopped there, on the commanded path. The fingertips reach about 16 mm beyond the tool frame (TCP) along its approach axis, open or closed. Arm drive torques are capped (the effort limits in robot_spec), so an arm pushing against an obstacle stalls at that torque. An arm with any part more than 5 mm deep in a fixed object (a fixture, the table) is stopped where it is: its joint targets become its measured joints for that control step, so it stops pushing (the safety readings report these stops as static_guard).
- Forcing is recorded, not judged: an arm drive (not a gripper drive) at 95% of its torque cap or more while that arm's gripper, or a part it holds, presses on anything else with 20 N or more in total is forcing; 0.5 s of it on every physics tick is a forcing event in the evidence (`s.safety()`, `s.status()["forcing"]`), shown to the reviewer when a success is reviewed. Forcing does not end the episode and does not change the grade by itself.
- Simulator breakdown (a non-finite state or a task contact deeper than 10 mm): the episode ends as an infrastructure error, not your failure, unless an arm was forcing (even for one tick) within the 0.25 s before it: then it is your failure (forced_breakdown: what broke and what forced it).
- Grasp regions (assisted grasping, see Grippers above; one sphere per graspable part): caliper: radius 20 mm; sleeve_0: radius 40 mm; sleeve_1: radius 40 mm; sleeve_2: radius 40 mm; sleeve_3: radius 40 mm. Both finger contacts must lie inside the part's sphere; a part touched elsewhere is grasped physically. Each region's centre (in its link's frame) is in `/app/instance.json` under `simulator.grasp_assist.targets`.
- A failure is reported in `s.status()["episode_end"]` as "A task body fell below the table" or "forced_breakdown: <what>" (<what>: the readable reason); `s.safety()` reports forcing, contact depths, volume ratios and table-guard stops against these limits, and `s.status()["review"]` the deep task contacts that would hold a success for review.
The scene objects are the task's entities (`s.objects()`: `bin_0` (fixed), `bin_1` (fixed), `bin_2` (fixed), `bin_3` (fixed), `caliper` (fixed), `caliper_stand` (fixed), `sleeve_0`, `sleeve_1`, `sleeve_2`, `sleeve_3`, `sleeve_rack` (fixed)).
`caliper` is one object of four links, fixed on its stand (`caliper_stand`): its pose in `s.objects()` is its root link's (link `mount`, the stand's clamp), whose frame is the beam's: origin on the closed outside measuring faces, x along the beam toward its tail, y toward the inside jaws, z out of the scale face. The beam (link `beam`) sits in the clamp on the joint `mount`, which its limits hold within 0.001 mm. The slider (link `slider`, the moving jaws) slides along the beam's x on the prismatic joint `slider`, whose value is the jaw opening in metres: `s.env._world.read_state()["articulations"]["caliper"]["links"]["slider"]` (`position_m`, `quaternion_wxyz`) and `s.env._world.read_state()["articulations"]["caliper"]["joints"]["slider"]["position"]`. The locking screw's knob (link `screw`) turns on the revolute joint `screw` (radians off the loose stop, 0 to 0.785; its axis is the slider frame's -z): `s.env._world.read_state()["articulations"]["caliper"]["joints"]["screw"]["position"]`; the lock itself: `s.env._world.read_state()["mechanism"]` (`screw_angle_deg`, `locked`, `lock_position_m`). The jaw opening in metres is not a reading: the scale's unit and zero are the scene's own. Each sleeve is one object of one link `sleeve` (its axis is the link's z).
Implementation reference: `CaliperSortingVernierTask.evaluate_state` and `evaluate_transition` (defined on `CaliperSortingTask`, module `topogym.tasks.caliper_sorting.common`).
This is a partial implementation excerpt; delegated helpers (`rigid_measurements`: fall and contact checks; `released`: no robot contact above 0.05 N) and the shared confirmation checks also apply. The complete public goal requirements are stated above.
```python
    def evaluate_state(self, state):
        """Success now: every sleeve in the bin labelled with its class, every sleeve still, none held by an assisted
        grasp and none touched by the robot (released). Also reports the sleeves left in another class's bin (still, not
        held, not touched): evaluate_transition times them."""
        _, _, _, failure = self.rigid_measurements(state)
        if failure:
            return False, failure, {}
        t = self.config["terminal"]
        held = sorted({h["binding"]["target"][0] for h in ((state.get("grasp") or {}).get("hands") or {}).values()
                       if h.get("binding") is not None} & set(self.sleeves))
        placed, speeds, bins, wrong = [], [], [], []
        for k, name in enumerate(self.sleeves):
            body = state["bodies"][name]
            cls = self.scenario["sleeves"][k]["cls"]
            kb = self.scenario["bins"].index(cls)
            v = float(np.linalg.norm(body["linear_velocity_m_s"]))
            w = float(np.linalg.norm(body["angular_velocity_rad_s"]))
            still = v <= t["max_linear_speed_m_s"] and w <= t["max_angular_speed_rad_s"]
            placed.append(self._in_bin(body["position_m"], kb))
            wrong.append(bool(still and name not in held and self.released(state, [name])
                              and any(self._in_bin(body["position_m"], j) for j in range(len(self.bin_centres)) if j != kb)))
            speeds.append((v, w))
            bins.append(kb)
        quiet = all(v <= t["max_linear_speed_m_s"] and w <= t["max_angular_speed_rad_s"] for v, w in speeds)
        released = self.released(state, self.sleeves)
        reached = bool(all(placed) and quiet and not held and released)
        return reached, None, dict(sleeves_in_their_bins=int(sum(placed)), sleeve_count=len(self.sleeves),
                                   sleeve_in_its_bin=placed, sleeve_target_bin=bins, held_sleeves=held,
                                   sleeves_released=bool(released), sleeve_left_in_wrong_bin=wrong,
                                   maximum_sleeve_speed_m_s=max(v for v, _ in speeds),
                                   maximum_sleeve_angular_speed_rad_s=max(w for _, w in speeds), sleeves_quiet=bool(quiet))
    def evaluate_transition(self, state, task_state):
        """A sleeve left still in another class's bin (no assisted grasp on it, no robot touching it) for the goal's
        confirmation time (stable_duration_s) ends the episode in failure: the episode ends at the first success, so
        without this a sorter could learn the right bins by trying placements one after the other."""
        reached, failure, metrics = self.evaluate_state(state)
        history = task_state or self.initial_task_state(state)
        if failure:
            return reached, failure, metrics, history
        now = float(state["sim_time_s"])
        since = [(s if s >= 0 else now) if w else -1. for s, w in zip(history["wrong_bin_since_s"],
                                                                         metrics["sleeve_left_in_wrong_bin"])]
        limit = float(self.config.get("stable_duration_s", .5))
        late = [k for k, s in enumerate(since) if s >= 0 and now - s + 1e-9 >= limit]
        metrics = {**metrics, "wrong_bin_time_s": [round(now - s, 6) if s >= 0 else 0. for s in since]}
        if late:
            return False, "A sleeve was left in a bin that is not its own", metrics, dict(wrong_bin_since_s=since)
        return reached, None, metrics, dict(wrong_bin_since_s=since)
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
s = Session("caliper_sorting", instance="/app/instance.json")   # builds the frozen scene (instance.json) and compiles kernels
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
`python /opt/tools/rcb_topogym.py replay --task caliper_sorting --instance /app/instance.json --traj /app/output/trajectory.npz`
## Conventions
- World frame: +z up. Quaternions are **WXYZ**. The ee_pose action and the proprio TCP pose are given in this world frame.
- Action (14): arm `left`: 6 joint targets (rad), then `left_native_gripper` = native drive target (m, limits 0.002 .. 0.037; open/closed targets in robot_spec); arm `right`: 6 joint targets (rad), then `right_native_gripper` = native drive target (m, limits 0.002 .. 0.037; open/closed targets in robot_spec); absolute position targets tracked by PD controllers, out-of-limit
  values clipped. `s.hold_action()` is the no-op.
- Control runs at 40 Hz (10 physics step(s) per control step). There is no step budget, but a trajectory may have at most
  200000 rows (83.3 min of simulated time): the verifier has to replay it within its own time limit, so `s.step` stops
  after that many control steps since the last reset, and a longer trajectory is rejected as invalid (reward 0). The time limit
  (`## Time limit`) is your only budget. One control step takes about 0.066 s of wall clock here, so budget your experiments.
- Grippers: assisted grasping (contact_weld_v1): a gripper whose command moves towards closed takes hold of a graspable part (the slider link of caliper, sleeve_0, sleeve_1, sleeve_2, sleeve_3) once both fingers touch the same part inside its grasp region, the tool frame within 15 cm of it and the penetration at most 2 mm, for 0.02 s; the part then follows the gripper rigidly. Any command that opens the gripper (by more than 0.2 % of its travel) lets it go. A hold that breaks (the fingers open more than 15 %, the part drifts 1 cm / 0.15 rad from its held pose, or the hold has to push or pull its part with more than 60 N averaged over 20 ms) is lost, and the gripper must open before it can take hold again; that force is reached when, for example, two held parts that hook into each other are pulled apart, or a held part is pressed into another object. Two grippers never hold the same part; other objects are grasped physically (friction).
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
