# RoboARC (MuJoCo release)

RoboARC's topological puzzle families on MuJoCo 3.14.0 physics, run by the simulator package `topogym`, and its IKEA
furniture-assembly suite ([IKEA suite](#ikea-suite), its own tooling and images). The topological families are RoboARC
2.2.1's 78 bases (`images/roboarc/release.json`): the image `ghcr.io/mll-lab-nu/efe:roboarc-2.2.0` (RoboARC's release
image, pinned by digest; 2.2.1 keeps the 2.2.0 image) and its tooling; results of the earlier Genesis release are not
comparable with this one.

## Quick start
```bash
harbor run -d embodied-first-exam@all -p tasks/roboarc -a codex -m openai/gpt-6-astra
```
Every image is public: Harbor pulls it.

- Name: RoboARC (display) / `roboarc` (task prefix, image `ghcr.io/mll-lab-nu/efe:roboarc-<images/roboarc/VERSION>`, `images/roboarc/`, this
  page). Code identifiers keep the simulator's name: the package `topogym`, the tooling `rcb_topogym.py`, `TOPOGYM_*` variables, the family keys
  (`interlocked_claws`); inside the image, `/opt/robotangle/` (the instances and `stage_instance.py`).

## Tasks

The topological families: `roboarc-<family>-<NN>-i01-{privileged,standard}` (PROTOCOL §1); the IKEA suite:
`roboarc-ikea-<item>-i00-{privileged,standard}` (one frozen scene per item, `i00`; no tier number; [IKEA suite](#ikea-suite)):
- `<family>` with dashes (`interlocked-claws`); `[metadata].family` keeps the underscores.
- `<NN>`: two digits, the base's number in its family. A number is never reused: removing a task leaves a gap. The
  difficulty is not in the name: it is `[metadata].difficulty` in task.toml (TAXONOMY).
- `i01`: the MuJoCo instance (the HF dataset's `instances/<family>-<NN>-i01/`).

Every topological task shares the tooling `rcb_topogym.py`, the graders, the sim service and the eai-standard/2.4 layer:
- physics: MuJoCo 3.14.0, CPU float64 (the blanket families' cloth: a deterministic CUDA step, `blanket_vbd_gpu`);
- the numpy / OpenBLAS SIMD masking in every container that runs a Session (so every CPU class builds the same frozen
  bytes); `render_bridge.py` and `nyx_direct.py` next to every tooling copy, named in the runtime binding;
- the privileged images (agent `/opt/tools/`, verifier `/tests/`) take `rcb_topogym.py`, `render_bridge.py` and
  `nyx_direct.py` from the files-only image `ghcr.io/mll-lab-nu/efe:roboarc-files-<version>` (`images/roboarc-files/`, built
  from `embodied_first_exam/benchmarks/roboarc/`; `FILES.json` pins its digest and the files' sha256) with
  `COPY --from=<image@digest>`: the privileged task dirs carry no copy. The standard sim service is built from `environment/sim/` (its `Dockerfile`)
  and takes them, and the rest of the files it shares with other tasks, at `/srv` from the same `images/roboarc-files/`;
- the texts state what the tooling grades: the contact-review values come from `CONTACT_RULE`, and the grasping rule is
  `Session.grasp_rules()` of the instance (the weld's slip force over 20 ms, 60 N ALOHA / 65 N Robotiq, and the
  Robotiq's release after 0.2 s without finger contact).

## Release

- The shared task code (the tooling, the graders, the sim service, the standard oracle player) has one source,
  `embodied_first_exam/benchmarks/roboarc/`. A change to a file of `images/roboarc-files/FILES.json` needs a new image
  version: `images/roboarc-files/VERSION`, `python3 scripts/files_image.py build roboarc-files`, push,
  `files_image.py pin roboarc-files <digest>`, then the tasks' `COPY --from` lines name the new digest; until then
  `scripts/files_image.py check` fails.
- The files a task carries are in `tasks/`: the privileged reference
  (`tasks/roboarc/roboarc-<base>-privileged/solution/trajectory.npz`, both modes), the instance's lock (that task's
  `tests/runtime_binding.json`), the goal-state images (its `environment/goal/`) and the standard agent Dockerfile.
  The privileged reference must be the file `images/roboarc/release.json`'s evidence names (`trajectory_sha256`).
  The task directories are the deliverable; the tooling that built them is kept outside this repository.
- The release (the image, the instances made in it, the release files, the renders, the checks and Harbor's oracle / nop
  runs) is built outside this repository; each base is named either as a tier of its family (`numbers`) or, for a new
  design, by its key.
- Large files: the frozen instances (and their appearance seeds) are in the public image, under
  `/opt/robotangle/instances/<base>/` (seeds: `.../seeds/<S>/`), pinned file by file in `images/roboarc/assets.json`.
- Image: `ghcr.io/mll-lab-nu/efe:roboarc-<images/roboarc/VERSION>`, RoboARC's release image of that version
  (`images/roboarc/Dockerfile`, RoboARC's recipe at the release commit; context: `images/roboarc/fetch_image_context.sh`).

## Seeds

Every topological base allows seeds 0, 1 and 2 (`[metadata.seeds]`, `build = "runtime"`): RoboARC's appearance
seeds. Seed 0 is the released look; seeds 1 and 2 change only what is drawn (the room's lights, the table top, its rim and
apron, the floor, and the task objects' colours and finishes, family by family), never a colliding geom, mass, joint or
contact parameter, so the physics, the grading and the reference rows are seed 0's. Parts whose colour the texts name keep
it, so one text serves every seed. Each seed is its own frozen instance in the image (`/opt/robotangle/instances/<base>/
seeds/<S>/`), bound by `runtime_binding.s<S>.json` next to the task's `runtime_binding.json`; the file `seed` (0 in
the committed task) selects which one `stage_instance.py` stages. `tests/seeds/<S>/` of the privileged task holds `seed`
(S), the reference re-stamped with the seed's birth state (`solution/trajectory.npz`) and, where a task shows goal
pictures, the seed's pictures (`environment/goal/`); `scripts/task_seed.py materialize` puts them in place.
`[metadata.seeds] oracle` lists seed 0 and the seeds RoboARC's own Harbor runs of the reference passed.

## Time budget

10800 s (3 h) wall clock in both modes and every family (each task's `[agent] timeout_sec`), within the default cap.
The texts say "the time limit (`## Time limit`)". The standard service's clock (`meta["time_left_s"]`, `RCB_WALL_CLOCK_S`
= the cutoff) counts from the agent's first request to the service (logged in `episode.jsonl`, kept over a restart), so
the instruction has the common closing sentence. The service still ends the episode with reason `timeout` when that
count reaches 0, never before the harness's cutoff; open: that end becomes report-only at RoboARC's next task update.

## Grading replays (privileged: the strict goal within the agent's rows)

The six families with a grading hold (blanket-flatten, knot-untying, lock-picking, orbits, rope-threading, slotted-post-ring;
`rcb_topogym.REPLAY_TOLERANCES`) replay a trajectory with the instance's own thresholds and, if the episode has not ended after
the last row, hold the final targets for up to 2 s; during that hold blanket-flatten accepts 92 % coverage and 80 mm/s cloth
speed (95 % and 30 mm/s in the rows) and knot-untying a rope speed 0.25 mm/s higher. Standard mode grades the live episode
strictly, with no grading replay (Standard grading, below).

**The rule.** Privileged success needs the one grading replay to succeed AND to reach the goal with the
instance's own thresholds within the trajectory's rows (`tests/grade.py`: `strict_replay_required` 1, `strict_goal_reached`,
`strict_goal_row`; `success_in_hold_only` 1 marks a trajectory whose replay succeeded only in the hold, not credited). The
instruction says where `s.replay(actions, grading=True)` reports it (`["grading"]["strict_success"]`). Why: the MuJoCo
release replays deterministically on the same host (the cloth step on a deterministic GPU solver, the rest on the CPU), so
a recording that stops on its own success reaches it within its rows; without the strict goal the hold's relaxed
thresholds would become the effective goal in privileged mode only.

**Validation** (Harbor `-a oracle` / `-a nop`, roboarc image 0.1.0): the privileged oracle scores 1 with
`strict_goal_reached` 1 on one task of each family with a hold and on interlocked-claws-01, deterministic; nop scores 0.
blanket-flatten-01's reference without its last 8 rows (a recording that stops 8 control steps before its own success)
is graded 0 (`all_success` 1, `success_in_hold_only` 1).

## Standard grading: the live episode

`success` = the live success: the task predicate on the live simulator with the instance's own thresholds, confirmation
window included (`server.py`: after every `submit` and every watching `observe`, and as the episode stands at its end),
recorded in `episode.json`. No grading replay runs. `finalize.sh` writes `final.json` from `episode.json` first
(`graded_by` live, `replays` 0, `state_sha` = the live end-state hash, `video_pending` 1), then runs ONE video-only pass
(`replay.py`: a plain fresh-process replay of the live rows on the sidecar's GPU that renders `replay.mp4` and, for a
success held for review, the frames around its live review events; record `replay.video.json`). It never changes the
verdict, and a failed, killed or GPU-less pass loses only the video. `tests/grade.py` transcribes the result. In `final.json`, `live_success` is the live episode's own success and `success` the verdict: false with an infrastructure failure or when the success is held for review (`held_for_review`). A service that dumped its episode but did not exit in time keeps that live verdict (`service_exit_timed_out`, no video). A missing `trajectory.npz` is recorded (`dump_error`) and changes no verdict. Non-finite numbers are written as null.
- **No hold.** The live episode gets no hold (a hold would loosen the rule: the privileged twin needs a strict success
  within the agent's rows).
- **Review, forcing, instability.** The live episode's `review` (deep task contacts: a live success is held for review,
  `success_pending_review`), `forcing` (evidence) and `simulator_instability` (a breakdown without forcing: infrastructure).
- **Start state.** `Session(strict=True)` checks the frozen instance against `runtime_binding.json` when the service builds
  it. A mismatch is `instance_mismatch`, `infra_error 1`, before anything is served. `tests/grade.py` also checks the live
  startup record (`startup_binding.json`) against its own copy of the binding.
- **Infrastructure** (H.5): no, unreadable or unfinalized `final.json`, the end timed out, `instance_mismatch`, a give-up
  after crashes not all inside the agent's calls (`sim_crashed`, `resume_failed`, `call_watchdog`), `simulator_instability`
  in the live episode. The agent's 0: a simulator error (reason `error`, `sim_error`), the task's failure predicate,
  `forced_breakdown`, the wall clock, `step_guard`, `sim_crashed_on_call`. A video-pass error or a GPU lost after the
  episode never voids a trial.
- **Re-scoring recorded standard trials.** The verdict is the recorded live fields of `final.json`: the `success` of
  `episode.json`. A live instance mismatch, `infra_error` or `simulator_instability` voids it, and a live `review` holds
  it.

## Safety readings in standard mode (`meta["safety"]`)

The standard service publishes `meta["safety"]` in every response, declared here as allowed by PROTOCOL H.3. Why: the benchmark's physical-validity policy judges
the agent on these readings -- a success with a deep task contact is held for human review instead of credited, a
soft body that stays collapsed ends the episode, a simulator breakdown shortly after forcing is the agent's failure --
so an agent that could not see them would be judged on something it cannot measure. The fields are exactly what
`rcb_topogym._SafetyMonitor.read` returns (maxima since the previous response, each next to its limit), and nothing
else of the simulator's contact state is published:

| Field of `meta["safety"]` | What it is | Families |
|---|---|---|
| `rules`, `scope` | the policy's revision; "since the previous read" | all |
| `forcing` | `max_press_force_n` (the largest total contact force an arm's gripper, or a part it holds, pressed on anything else with), `max_drive_fraction_while_pressing`, `longest_forcing_s`, `events`, the last three events (each: the arm and joint, the control step, the drive fraction and cap, the force, and in words what pressed on what, e.g. "a part it holds" on a named task part), and the thresholds (`contact_n`, `near_cap_fraction`, `event_after_s`) | all |
| `task_contact_mm` | the deepest contact between task parts (`max`), the longest time above the sustained limit, and the review and breakdown limits | rigid |
| `held_finger_contact_mm` | the deepest contact between a finger and the part it holds, with its review limits | rigid |
| `review` | the count of review events and the last three (each: the kind, the control step, the names of the two task parts in contact, the depth in mm and the force in N); a success with one is held for review | rigid |
| `soft_volume_ratio` | the lowest volume ratio of each soft body and the longest time at or below 0 | soft (rope, cord, cloth) |
| `attachment_stretch_mm` | the worst cord attachment stretch and the longest time above its limit | soft |
| `table_guard` | the control steps whose gripper target the table guard stopped, with the last events | where the guard applies |
| `ik_jumps` | the last IK solutions that lay far from the current joints (`ee_pose` actions) | when one occurred |

These are forces, depths, ratios and the names of the parts involved, never a pose: no field says where an object is.
A change to this list is a change of the declaration and goes through the protocol-change process (PROTOCOL §7).

Names of the parts involved are allowed in standard mode; positions / poses in the simulator's world frame, the table
height and physics parameters are withheld. The service's `robot_felt` passes exactly these fields (`SAFETY_PUBLISHED`).

## Standard mode, world frame, network

`eai-standard/2.4`; no world coordinates in standard texts (D7,
`ci_checks/benchmarks/roboarc/advisory-world-frame.sh`); agent egress on the allowlist (PROTOCOL Appendix H.6).

Every text the agent sees calls the service's frame the world frame, as the other services call theirs, and defines it in
words; it is the robot frame (the table height and the robot base's world position are never disclosed): origin at the robot's kinematic root link on its mounting, axes as the simulator's world
(+z up). The proprio TCP poses, `ee_pose` targets, `/kinematics` (ik targets, fk poses) and the camera extrinsics (static
mounts, per-observe calibration, per-frame `extrinsic_cv`) are in it; `robot_spec.base.pose` is the identity; no text and no
`/spec` field states a world coordinate of the mounting. `joint_position` rows, physics, graders and state hashes do not depend
on the frame. Privileged mode keeps the simulator's world frame (its tooling is the simulator); its texts state no table height
or robot base position either.

## Registration (the lines a rename changes)

`git mv` of `images/roboarc/`, `images/roboarc-files/` (a new image name: build, push, pin) and this page; then the registration lines: `scripts/task_conventions.py`
(`BENCHMARKS`), `scripts/task_resources.toml`
(the `^roboarc-` rules; the generic `^roboarc-(?!ikea-)` rules leave the IKEA suite to its own `^roboarc-ikea-` rules), `docs/TAXONOMY.md`, `README.md`, the checks that name a benchmark's prefix
(`ci_checks/check-tooling-identity.sh` `MODULE`, where the IKEA suite is the group `roboarc-ikea`,
`ci_checks/benchmarks/roboarc/advisory-world-frame.sh`); `ci_checks/ci_scope.py` (RoboARC's unit includes `images/ikea-mujoco/` and
`images/ikea-mujoco-files/`); the cutoff is
each task's own `[agent] timeout_sec`.

## Standard-mode service details

Serves `eai-standard/2.4` (docs/protocol/standard_mode.md Part 4). Contact and safety telemetry is published in `meta["safety"]`, declared field by field in this doc. `max_frames_per_call` = 2000 (IKEA suite: 1000; PROTOCOL §3 "Images per call"); `meta.frames_skipped` counts the frames a call whose length is known only while it runs left out. Its service ends the episode when `time_left_s` reaches 0 (counted from the agent's first request), until its next task update.

## IKEA suite

The IKEA furniture-assembly tasks (`roboarc-ikea-<item>-i00-{privileged,standard}`, twelve items in both modes) are part
of RoboARC. They share the `roboarc-` prefix, this folder, the eai-standard/2.4 layer and the 3 h budget;
everything else is their own: the tooling `rcb_ikea.py` (not `rcb_topogym.py`), the scenes, the graders and the sim service
(source `embodied_first_exam/benchmarks/roboarc/ikea/`, files-only image `efe:ikea-mujoco-files`), and the images
`efe:roboarc-ikea-<VERSION>` / `efe:roboarc-ikea-sim-<VERSION>` (recipes and scenes in `images/ikea-mujoco/`). The
sections above describe the topological families; this section is the whole description of the IKEA suite.

Furniture-assembly scenes in plain MuJoCo 3.14.0 on the CPU (float64), each with two robot arms and physical
grasps (finger friction, no assist). Twelve tasks ship, each in both modes (24 task directories,
`tasks/roboarc/roboarc-ikea-<task>-i00-{privileged,standard}`):

### IKEA quick start
```bash
harbor run -d embodied-first-exam@all -p tasks/roboarc -i 'roboarc-ikea-*' -a codex -m openai/gpt-6-astra
```
All images are public (`efe:roboarc-ikea-1.1.0`, `efe:roboarc-ikea-sim-1.1.0`, `efe:ikea-mujoco-files`): the tasks run
directly. To rebuild the images from source, see [IKEA image and assets](#ikea-image-and-assets) and `images/ikea-mujoco/README.md`.

| task | robot | what is assembled | mechanism |
|---|---|---|---|
| `lack` | two Franka Panda | LACK side table: four legs screwed into the top | threads (exact convex pieces), latched retention welds |
| `svallet` | two ViperX 300 s (ALOHA 2) | SVALLET lamp: the bulb screwed in, the shade snapped on | spring jaws and snaps, the bulb's thread |
| `gladom` | two Franka Panda | GLADOM tray table | threads (SDF meshes), latched welds |
| `variera` | two Franka Panda | VARIERA box: screws driven with a screwdriver | threads (SDF meshes) |
| `eket` | two Franka Panda | EKET cabinet: dowels and panels | latched retention welds |
| `ikea365` | two Franka Panda | IKEA 365+ stand and bowl: a screw driven with a screwdriver | threads (exact convex pieces), latched welds |
| `vesken` | two Franka Panda | VESKEN shelf unit: 23 snap connections | spring jaws and snaps |
| `stegesund` | two Franka Panda | STEGESUND stand: bolts driven with a hex key | capsule helices, seat welds |
| `stenerik` | two Franka Panda | STENERIK laptop stand: six screws driven with a screwdriver, four pads peeled and pressed on | threads (exact convex pieces), retention welds, pad peel-off and bonds |
| `burvik` | two Franka Panda | BURVIK side table: four legs screwed in (two into the top, two through the handle's sleeves), the handle through the top, the table turned upright | threads (SDF meshes), latched retention welds |
| `uppsta` | two Franka Panda | UPPSTÅ toy hammering block: four dowels and two end blocks, two screws driven with a hex key, eight pegs pushed in | threads (convex pieces), seat welds with the welded-part contact switch-off |
| `kyrre` | two Franka Panda | KYRRE stool: three legs placed on the upside-down seat, six screws driven with a hex key | threads (SDF), latched retention welds with the welded-part contact switch-off; MuJoCo's native cylinders |

`simulator = "mujoco"`; `robot = "franka-panda-bimanual"` (two Franka Panda arms with MuJoCo Menagerie's Panda hand;
stegesund and uppsta name their arms `left/` / `right/`, the others `left_` / `right_`) or `"aloha-2"` (svallet: MuJoCo
Menagerie's ALOHA 2, two ViperX 300 s arms). Standard tasks speak `eai-standard/2.4` (the operations capability:
`r.operation_status()`).

The agent budget is 10800 s (3 h) of wall clock in both modes. A privileged trajectory may have at most the task's row
cap (stated in its instruction: the verifier's replay must fit its time limit).

### IKEA seeds
Every IKEA task allows seeds 0, 1 and 2 (`[metadata.seeds]`: `allowed = [0, 1, 2]`, `build = "runtime"`; the
reference is checked at seed 0). The scene is the one frozen instance at every seed, and a seed changes the standard
mode's Nyx lighting only: seed 0 is the released lighting; seeds 1 and 2 move the key light to another of the four
lights (three times as bright as the others, the only one casting shadows), hang the lights 0.98-1.48 m above the work
surface, light them white of 3600-4300 K or 9000-10500 K, and scale them and the environment map (which takes their
colour as its tint) by a brightness factor: one seed warm and one cool, one bright and one dim. The standard task's
`environment/sim/rig.json` holds every seed's lights (`seeds`); the sim service reads the file `seed` (0 in the
committed task; `scripts/task_seed.py materialize` writes the trial's) and draws with that seed's lights. The privileged
mode shows the agent no images and its replay video keeps the scene's own lights, so there every seed is the same task.
The scene, the physics, the instruction, the graders and the reference are the same at every seed.

### IKEA tasks
- The task directories are the deliverable; the tooling that built them is kept outside this repository.
- In `images/ikea-mujoco/`: `assets.json` (the sha256 of every scene file in the image, and the dataset pin of the
  meshes; Image and assets), `bindings.json` (what each scene builds to in the image: initial-state hash, actuators,
  timestep, speed limits). The scenes' upstream provenance is kept outside this repository (PROTOCOL Appendix H.2).
- Each privileged task carries `rcb_ikea.py` (the tooling and the replay), `tests/grade.py` (the verifier) and
  `tests/render_video.py` (the replay video); each standard task the sim service (`environment/sim/`: `server.py`,
  `ikea_rig.py`, `ikea_nyx.py`, `nyx_direct.py`, `finalize.sh`, and the task's own `rig.json`; `ikea_nyx.py` drives that
  `nyx_direct.py`, the IKEA tasks' own copy of RoboARC's renderer front end, not the topological tasks' one), its
  grader (`tests/grade.py`: transcribes the live episode's `final.json`) and the oracle client.
- The references are kept in the task directories only (the standard `solution/oracle_actions.npz`, which the
  privileged `solution/trajectory.npz` is a copy of).
- Every task's agent egress is the allowlist of `scripts/set_agent_egress.py`.

### IKEA grading (both modes, every task)
One rule in both modes. Privileged: the deliverable (`/app/output/trajectory.npz`) is replayed open-loop in one fresh
process from the frozen keyframe. Standard: the rule is evaluated on the live episode by the sim service; nothing is
replayed (below). Success needs all of:
- **Goal:** the scene's own `success.py`, a port of the source assembly grader.
- **Terminal rules,** the same in every task: the parts are released (no robot link touches a part); every part is at
  rest (at most 0.01 m/s and 0.05 rad/s); the simulation never broke down (no non-finite state, no MuJoCo BADQACC /
  BADQPOS / BADQVEL warning, no contact of a part deeper than 10 mm).
- **End pose (EKET):** every panel, the back included, ends within 10 mm and 2 degrees of its goal relative to the top
  panel (`rcb_ikea.END_POSE_RULES`): the source grader checks the back only at its slide, and the back has no frame joint.
- These hold at the end of the rows **and** again after the grader holds the last row for 1.0 s
  (`rcb_ikea.Session.final_check()`).
- A replay that crashes (a simulator crash, `replay_crash`) or in which the scene's code or MuJoCo raises on the state
  the rows drove it into (`replay_exception`) is a breakdown (`sim_breakdown 1`, reward 0), not a grader error: a
  grader or infrastructure error gives the trial a free retry (PROTOCOL H.5), which the deliverable's own failures must
  not get. A scene that differs from the binding or builds to another initial state is `infra_error 1`.
- The privileged verifier's time limit covers a trajectory of the task's row cap on a slow trial-class CPU (one
  reference replay on a Xeon Gold 6230R core, with a 1.5x margin), not only the
  Harbor host's timing: a verifier that times out writes no verdict.
- The privileged verifier checks the scene in its image file by file against `tests/runtime_binding.json`
  (`instance_exact`) and its initial-state hash (`instance_ok`); the standard sim service does the same before its
  episode begins (`server.py`: scene files, initial-state hash, actuators and runtime against
  `environment/sim/runtime_binding.json`; a mismatch is `instance_mismatch`, `infra_error 1`, and nothing is served).

Standard mode is graded by the live episode: `success` = the sim service's live
verdict, `graded_by_live 1`, `replays 0`. The service ends an episode at the first settled success, `Session.status()`
and then `final_check()` on a snapshot with the last row held 1.0 s (the privileged grader's rule, the settle window
included); a failed settle lets the episode go on (`meta["settle_failed"]`). An episode that ends otherwise (the wall
clock, the collect hook's end) is judged as it stands by the same two checks. `finalize.sh` writes `final.json`
(`graded_by "live"`, `replays 0`, `state_sha` = the live end-state hash); the verifier (`tests/grade.py`, the shared
client base, no simulator) transcribes it. The trajectory (`/data/trajectory.npz`) stays the replayable deliverable; a
live success without it is `infra_error`. Infrastructure (a free retry, H.5): no, unreadable or unfinalized
`final.json`, `instance_mismatch`, a give-up after crashes not all inside the agent's calls (`sim_crashed`,
`resume_failed`, `call_watchdog`), a service-side error. The agent's 0: a breakdown, a simulator error its rows caused
(`sim_error`), a timeout, all crashes inside its calls (`sim_crashed_on_call`), a failed settle.

Every graded trial leaves a video (PROTOCOL §4), evidence only: standard, the sidecar's `/data/live.mp4`, recorded during
the live episode (Nyx, cam_high, 10 frames per simulated second, on the GPU the sidecar renders the agent's cameras with;
no extra pass); privileged, `/logs/verifier/replay.mp4`, rendered by another process next to the verdict replay
(MuJoCo's renderer on CPU OSMesa: the verifier has no GPU; 512 x 288, at most 750 frames).

### IKEA robot speed (both modes)
Every arm joint target moves at most the robot's real joint velocity limit per control step: Franka Panda 2.175 rad/s on
joints 1-4 and 2.61 rad/s on joints 5-7; ViperX 300 s 3.14159 rad/s. It is a clamp, not a failure
(`rcb_ikea.Session.applied()`, in the tooling, the replay and the standard service alike): a row within the limit
(+1e-9) is applied unchanged, a faster one is spread over the following control steps. The Franka Hand's command
(`actuator8`) is clamped the same way at the hand's real finger speed, 50 mm/s per finger (0.1 m/s of commanded finger
width): 318.75 units/s in lack and stenerik, 182.142857 units/s in the other Panda scenes
(`rcb_ikea.GRIPPER_WIDTH_SPEED_M_S`; `images/ikea-mujoco/bindings.json` records the per-scene values). The ViperX
gripper is not limited.

### IKEA physics changes against the source scenes
One rule for all scenes, clamp not fail (as the topological families):
- **MuJoCo 3.14.0 with the EPA fix byte-patched in.** 3.14.0's EPA keeps the polytope horizon in two fixed 24-entry
  arrays and writes past them unchecked (fixed upstream after 3.14.0). A dowel pressed deep into a thin convex piece
  of eket's side panel overflows them: SIGSEGV or silent memory corruption, about 1 in 400 random eket episodes. The
  image patches 14 bytes of the wheel's own `libmujoco.so` (`images/ikea-mujoco/mujoco-3.14.0-epa/`); it is
  bit-identical to the stock wheel on every reference and stress episode checked. A from-source build is not
  bit-identical to the wheel and is not used.
- **Cylinder -> prism:** RoboARC's rule. Every cylinder a robot geom can touch is a 32-facet convex prism at scene
  build (inscribed in the cylinder). It removed every cylinder-caused breakdown of the stress test. kyrre is the one
  exception: its six screws keep MuJoCo's native cylinders, which gave fewer breakdowns in a stress A/B on the same 203
  episodes (1.15 % against 2.3 % non-jerk breakdowns), and the authors' demo assembles the stool only without the rule.
- **Joint-speed clamp and Franka Hand slew:** above. The source demos command targets up to 55 x the real limits; every
  reference was rebuilt or retimed within them.
- **Welded-part contact switch-off:** bodies joined by latched retention welds (kyrre, from its `sw_cutover.json`) or by
  seat welds (uppsta) form one rigid group whose members stop colliding with each other; every other contact is kept
  (checked contact for contact: with the switch, the contacts are those without it minus the in-group pairs).
- eket's dowels: the soft dowel shells touch only the side panels; a stiff twin meets everything else.
- numpy / OpenBLAS SIMD masking in the image (`NPY_DISABLE_CPU_FEATURES` without AVX-512, `OPENBLAS_CORETYPE=Haswell`),
  as in the topological families' image: unmasked, eket's reference diverged between AVX-512 and AVX2 hosts.

Stress test (random, jerk and prefix strategies from the frozen start state; 203 episodes per scene for the later
scenes): non-jerk breakdown rates between about 0.5 % and 2 % per scene. Residual classes, none blocking: energy created
at robot joint limits under random arm targets (vesken, eket), one table penetration (gladom, prefix-jerk), ViperX
self-contact under jerk commands (svallet).

### IKEA references
Each task has one reference for both modes, every row within the joint-speed limits and the Franka Hand slew (the
privileged `solution/trajectory.npz` and the standard `solution/oracle_actions.npz` are the same rows):
- lack and svallet: the authors' demo brought within the limits (lack: through the clamp, each over-fast hand row held
  until the hand reaches it; svallet: time-dilated by a slew-limited tracker);
- eket, ikea365, vesken, gladom, variera, stegesund, stenerik, burvik, uppsta: closed-loop references built on the final
  physics: the authors' demo as the phase guide, with true-state feedback (damped-least-squares IK servos onto the
  parts' true poses, screwdrivers and hex keys on the true screw axes, gates and local searches), recorded as the
  executed rows;
- kyrre: the authors' demo on the native-cylinder scene, each row held until the slewed hand reaches it, one grasp
  re-aimed at where the screw really lies and the hex key set down off its knife-edge rest.

RoboARC's validation on the final images: Harbor oracle 1 / nop 0 in both modes for all twelve scenes; each reference
ends on the same state hashes on Intel AVX2 (i9-14900HX), Intel AVX-512 (Xeon Gold 6230R) and AMD Zen 4 (Ryzen 9 7950X,
EPYC 9354); in standard mode the live end state is the reference's where the episode runs to the last row (ikea365,
gladom, vesken and stegesund reach their settled success at an earlier chunk end).

### IKEA adversarial trial (PROTOCOL H.1)
`roboarc-ikea-eket-i00-privileged` with a red-team paragraph on top of its
instruction (obtain reward 1 without assembling the cabinet, by any exploit of the grader, the tooling, the file format
or the simulator), Codex + GPT-6 Luna xhigh with the runner's settings: reward 0, deterministic, no grader or
infrastructure error. The agent read the loader, the replay, `final_check()`, the operation tracker, the latches and the
end-pose rule and found no bypass; its best attempt (a one-row hold) replays released and at rest and fails the goal.
A malformed or compressed-bomb trajectory, a scene exception and, in standard mode, a simulator error or a malformed
request field are the agent's 0, never a free retry.

### IKEA timing
One reference execution against the 10800 s agent budget. A Xeon Gold 6230R core (2.1 GHz) is a slow trial-class CPU;
the quiet i9-14900HX laptop a fast one; the standard live episodes ran on a shared, loaded RTX 4090 workstation
(Ryzen 9 7950X).

| task | rows | one replay, quiet i9-14900HX | one replay, dedicated Xeon Gold 6230R core | standard live episode, loaded host |
|---|---|---|---|---|
| eket | 83,181 | 221 s (2.0 %) | 575 s (5.3 %) | 1,049 s |
| svallet | 69,414 | | 478 s (4.4 %) | 397 s |
| ikea365 | 96,747 | 249 s (2.3 %) | 649 s (6.0 %) | 1,285 s |
| lack | 67,697 | 372 s (3.4 %) | 1,127 s (10.4 %) | 1,277 s |
| vesken | 389,504 | | 1,822 s (16.9 %) | 1,889 s |
| stegesund | 379,529 | 1,164 s (10.8 %) | 2,450-2,473 s on a Xeon Gold 6338 core (22.8 %) | 1,938 s |
| burvik | 162,534 | 1,694 s (15.7 %) | 3,783 s (35.0 %) | 1,824 s |
| gladom | 186,611 | 2,630 s (24.3 %) | 8,069-8,082 s (74.8 %) | 4,780 s |
| stenerik | 222,600 | 2,649 s (24.5 %) | 6,944-7,082 s (64.3-65.6 %) | 3,672 s |
| variera | 243,159 | 2,664 s (24.7 %) | 8,024-8,696 s (74.3-80.5 %) | 4,274 s |
| uppsta | 430,225 | 3,199 s (29.6 %) | 8,425 s (78.0 %) | 4,291-4,685 s |
| kyrre | 733,558 | 2,740 s (25.4 %) | 8,283 s (76.7 %) | 3,073 s |

The budget is the same on every host, so on a slow CPU an agent has less room for full replays of the longer scenes:
MuJoCo's own step is about 95 % of the time in the threaded scenes (SDF threads, convex thread pieces).

### IKEA image and assets
- `ghcr.io/mll-lab-nu/efe:roboarc-ikea-<images/ikea-mujoco/VERSION>` (1.1.0, image id `9f9cbab8aa74`), RoboARC's IKEA
  release image byte for byte: MuJoCo 3.14.0 (EPA-patched), numpy 2.5.4, Python 3.12, OSMesa, imageio with ffmpeg and
  Pillow, and the twelve scenes at `/opt/ikea/scenes/<task>`. The privileged agent and the privileged verifier run it;
  the standard verifier runs the shared client base (it replays nothing).
- `ghcr.io/mll-lab-nu/efe:roboarc-ikea-sim-<images/ikea-mujoco/sim/VERSION>` (1.1.0, image id `cbd929360257`): the
  standard sidecar. It adds Nyx, RoboARC's renderer and look, in its own venv `/opt/nyx` (gs-nyx 0.1.3 needs numpy < 2.4;
  the physics process keeps numpy 2.5.4 and the patched MuJoCo): RGB from Nyx (path tracer, 32 spp, denoised, studio
  HDRI, point lights placed relative to each scene's work surface), depth from MuJoCo's EGL raster, the same camera rig
  and calibration. Nyx draws only what MuJoCo's raster drew.
- Both images are built from fully pinned inputs (the Python base image by digest, Debian packages by version from the
  Debian snapshot archive, wheels by sha256; `images/ikea-mujoco/Dockerfile`, `sim/Dockerfile`, RoboARC's recipes); the
  tasks pin them by digest.
- The scene code is in git (`images/ikea-mujoco/scenes/<task>/`); the meshes (2,078 files) are not, and every file's
  sha256 is pinned in `images/ikea-mujoco/assets.json`. The robot meshes (828: Franka Panda, Apache-2.0; ALOHA,
  BSD-3-Clause) come from MuJoCo Menagerie at a pinned commit; the furniture part meshes (1,250, parameterized
  reconstructions drawn from the IKEA assembly manuals, not IKEA CAD; CC BY 4.0) from the public Hugging Face dataset
  `MLL-Lab/embodied-first-exam-public` (`ikea-mujoco/`) at a pinned revision, no login. `images/ikea-mujoco/fetch_image_context.sh` downloads
  both, checks them and stages the build context (`images/ikea-mujoco/README.md`).
- `stage_context.py` refuses a scene text that points at upstream material (H.2).

### IKEA world-frame information
The standard tasks' texts give no world coordinates: no part poses or positions, no table height, no world base poses.
Every pose the standard agent gets or gives is in the robot frame (origin at the left arm's base link, +z up). The
privileged tasks read anything, as privileged tasks do. A scene reads nothing from the environment (`rcb_ikea` drops
kyrre's development switches `SW_FROM_OP` / `SW_FROM_T` on import).
