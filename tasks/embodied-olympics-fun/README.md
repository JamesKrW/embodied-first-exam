# Embodied Olympics - Fun

Seventy-nine single-robot sports for a humanoid: one Unitree G1 or H1-2 sprints the indoor 50 m, jumps rope, takes a
penalty against a keeper, returns a table-tennis serve, dunks, swings a giant on the high bar, skis a slalom, flies a
big air or dives from the 10 m platform. A referee decides from the simulator's state whether it did the sport. The
physics is real and the robot is its own, so most tasks have no known solution with the robot's own controls: the
benchmark is meant to find the ceiling of coding agents, which may use the robot's built-in walking (and, on the G1,
running) policy, drive every joint, and search for and download public tools and pretrained controllers.

## Quick start
```bash
harbor run -d embodied-first-exam@all -p tasks/embodied-olympics-fun -a codex -m openai/gpt-6-astra
```
Every image is public: Harbor pulls it. Each task dir's README carries its family's rule book, deviations, evidence
and numbers.

| | |
|---|---|
| tasks | 79 families: 79 `-privileged`, 79 `-standard` (`eai-standard/2.1`) and 21 `-standard-realtime` (`eai-standard/2.3`, the dynamic families), 179 dirs, `embodied-olympics-fun-<family>-i00-<mode>`; core set 56 (45 default standard, 11 default standard-realtime) |
| simulator | MuJoCo 3.3.7 on the CPU (1 ms physics, 10 ms control), on [Embodied Olympics](https://github.com/CHIGUI0/roboolympics)' robot library and running policy (MIT), unchanged in the image |
| image | `ghcr.io/mll-lab-nu/efe:embodied-olympics-fun-<images/embodied-olympics-fun/VERSION>`; shared files (tooling `rcb_rfo.py`, the `rfo` core, graders, the standard service, the dynamic families' modules) from `efe:embodied-olympics-fun-files-<VERSION>` (`images/embodied-olympics-fun-files/FILES.json`); source `embodied_first_exam/benchmarks/embodied-olympics-fun/` |
| actions | one sim row per 10 ms: `stand`, `walk`, `joints`, `lowcmd` (Unitree's LowCmd), on the G1 also `walk_falcon`, `walk_agile`, `run`, `run_arms`; `grip_left`, `grip_right` |
| score | the referee's binary verdict (every stage in order, no rule broken, within the time limit), with `progress` and the family's metrics beside it |
| agent budget | 7200 s in every mode (each task's `[agent] timeout_sec`) |
| network | open (below) |

## Robots and hands

- **Unitree G1** (1.32 m, 33 kg, 29 joints; three walking policies and a running policy) with its **Dex3-1**
  three-finger hands, and the **Unitree H1-2** (1.78 m, 67 kg, 27 joints; a walking policy) where the venue needs an
  adult-size body, with its **Inspire** five-finger hands (20 N per fingertip, 30 N the thumb).
- Unitree's measured actuator data (torque-speed curves, rotor inertia, friction); the H1-2's legs and arms are
  H2-class actuators from Unitree's H2 data (hips and knees 360 N m), its torso keeps 200 N m and its wrists 19 N m.
  Real self-collision; contact with the ground stays MuJoCo's soft default by design.
- Nothing a hand holds is welded and nothing is grippable. In 23 families the finger hands hold rackets, balls, bars,
  poles and handles by contact and friction (one closure command per hand); in 5 ball families (`bowling`,
  `juggling`, `juggling-feet`, `shot-put`, `volleyball-serve`) the ball is pressed between or carried on the hands' flat
  palm pads; elsewhere the hands hold nothing.

## Design rules

- **Real physics.** No welds of held objects, no pins, no mocap bodies, no velocities written after the start: balls
  come from two-wheel ball machines, keepers and rope turners are force-limited mechanisms, material numbers are sourced.
- **The native body.** Each robot with its own hands and actuators.
- **Raised parameters at most 1.5-2 times native**, declared with the measured need; the one exception is
  `high-bar-swing`'s grip (5 times: a giant swing pulls four or more body weights at the bottom).
- **Official venues where the hardware can reach them**, else the demand lowered only to near the measured physical
  limit, and declared. For example:
  - `dunk`, `dunk-between-legs`: the ring at 2.39 m, where an agent's own-motor dunk is recorded;
  - `clean-and-jerk`: a 5 kg aluminium technique bar lifted from 0.35 m blocks;
  - `javelin`: the women's 600 g javelin;
  - `sprint`, `hurdles-freestanding`: the indoor 50 m.

Every family names its rule book and lists each deviation with its reason in its task's instruction and README.

## Modes

- **privileged**: the world in the agent's container (full state, the referee's live record, the rules as code,
  unlimited resets); the deliverable is `trajectory.npz` (sim rows).
- **standard**: a client only (head cameras with depth, joint encoders, torques, IMUs, the hands' state, the venue's
  camera; action chunks); the rule book and the venue as an athlete knows them, never the robot's or an object's pose
  during an attempt or the referee's state; each attempt ends with the referee's call.
- **standard-realtime** (21 dynamic families: a ball, keeper, rope, boom or opponent that moves on the venue's clock):
  standard on a clock that does not stop, 100 Hz control, slowdown S = 1 in every family (real time); rendered with EGL on the
  sidecar's GPU.

**Reset budget 5** in standard and standard-realtime: a sport is attempts, and 5 is about the trials a field event
gives an athlete, enough to correct from the referee's call and too few to search for a plan by trial and error. The
last attempt is graded and a success ends the trial. With resets the first attempt is kept (`trajectory.first.npz`,
`replay.first.mp4`; `first_success` in `final.json` and `reward.json`): the no-reset view.

## Grading

- **The referee** (`rfo/referee.py`) judges from the simulator's state after every 1 ms physics step: stages that
  latch in order, contact events and legal body parts, crossings found between physics steps, flight phases; a foul,
  a fall, the time limit or a physics blow-up ends the attempt. Its record is part of the state hash.
- **Privileged**: `tests/grade.py` replays the trajectory in two fresh processes; both must succeed and end in the
  same state hash.
- **Standard and standard-realtime**: the sidecar's referee calls the live attempt (`final.json`, `graded_by` live,
  no grading replay); the start is checked against the instance's state hash, and a crash resumes the attempt from the
  sidecar's journal.

## Reference solutions

19 families have a real controller that succeeds in the task's own setting (`oracle = "full"`, marked below); the
other 60 have none (`oracle = "none"`, human review). Every family has a referee test with a declared positive (a real
controller or a magic action that makes the sport's events happen in physics) and negative controls.

## Network: open

The tasks declare `[metadata] network = "open"`: the instruction encourages the agent to find, vet and adapt public
tools, robot models and pretrained controllers (fair equipment, as a published walking controller is for a real G1)
and asks only that it not look up a ready-made solution to the task, such as this repository's reference solutions
(an honour rule; trials are reviewed). Results are open-network results; the harness's web search stays off in
official runs; trial rows stay private (a passing trajectory is a complete solution).

## Compute

CPU only; one world steps about 0.8 to 3.6 times real time on one core, depending on the host (the finger-hand families slowest) and holds about 0.4 GB.
Privileged: agent 8 CPUs / 16 GiB, verifier 4 CPUs / 8 GiB; standard: sidecar 4 CPUs / 8 GiB, agent 4 CPUs / 10 GiB
(author estimates, `scripts/task_resources.toml`).

## Run

```bash
harbor run -p tasks/embodied-olympics-fun/embodied-olympics-fun-sprint-i00-privileged -a oracle        # or -a nop: scores 0
harbor run -p tasks/embodied-olympics-fun/embodied-olympics-fun-penalty-kick-i00-standard -a codex -m openai/gpt-6-astra --env-file .env
```
Build the image with `scripts/build_images.sh` (context `images/embodied-olympics-fun/`); after a change to a shared
file, rebuild and re-pin the files image (`scripts/files_image.py build|pin embodied-olympics-fun-files`).

## The 79 families

One row per family; "ref." marks the 19 with a reference solution. The robot is the family's `Task.robot`.

| family | robot | event | ref. | family | robot | event | ref. |
|---|---|---|---|---|---|---|---|
| `sprint` | G1 | 50 m | yes | `tennis-return` | H1-2 | Tennis: return of serve |  |
| `hurdles-freestanding` | H1-2 | 50 m hurdles, free-standing hurdles |  | `tennis-serve` | H1-2 | Tennis: serve | yes |
| `shuttle-run` | G1 | 4 x 10 m shuttle run |  | `badminton-serve` | G1 | Badminton: singles service |  |
| `slalom` | G1 | Illinois agility test |  | `table-tennis-return` | G1 | Table tennis: return a service |  |
| `high-jump` | H1-2 | High jump |  | `paddle-bounce` | G1 | Table tennis: keep the ball bouncing on a paddle | yes |
| `long-jump` | G1 | Long jump |  | `golf-putt` | G1 | Golf: a 3 m putt | yes |
| `standing-long-jump` | G1 | Standing long jump |  | `javelin` | H1-2 | Athletics: javelin throw |  |
| `triple-jump` | G1 | Triple jump |  | `shot-put` | H1-2 | Athletics: shot put |  |
| `vertical-jump` | G1 | Vertical jump (jump and reach) |  | `discus` | H1-2 | Athletics: discus throw |  |
| `jump-rope` | G1 | Rope skipping (one minute) |  | `pitch-target` | H1-2 | Baseball: pitch a strike |  |
| `long-rope` | Reference: a periodic hop from a balanced crouch (joints/lowcmd rows) timed to the rope's angle, closed loop on privileged rope and pelvis state: 10 consecutive jumps, the rope 4-9 cm under the soles, lands at 10.17 s and stands 0.5 s before the rope comes round (success 10.85 s); the crank torque peaks at 3.0 of 10 N m and the rope collides with itself; 4 of 4 with the tempo +-2 % and the phase +-3 deg, 10 of 10 under small perturbations of the rope's phase and tempo and a push at the start, 48 of 50 with its own lead and push gain varied (push gain +3 %: 8 of 10). Negatives: standing still, a late or low hop, a squat, leaving the centre zone, lying down, a tipping jump, a bad last landing, no stand after the landing; two hops per turn count once. Measured (make_oracles.py): success after 10.8 s of the 20 s limit, live and in a fresh replay, ending in the same state. |  |
| `pole-vault` | G1 | Pole vault |  | `catch` | H1-2 | Baseball: catch a fly ball |  |
| `alpine-slalom` | G1 | Alpine skiing: slalom |  | `juggle-three` | G1 | Juggling: a three-ball cascade |  |
| `giant-slalom` | G1 | Alpine skiing: giant slalom |  | `wall-ball` | H1-2 | Wall ball: throw, rebound, catch |  |
| `freeski-360` | G1 | Freeski big air: a 360 |  | `boxing` | H1-2 | Boxing: one clean punch on the heavy bag | yes |
| `snowboard` | G1 | Snowboard: parallel slalom |  | `kick-target` | H1-2 | Kick the heavy bag at trunk height |  |
| `freeski-straight-air` | G1 | Freeski big air: straight air |  | `taekwondo-kick` | G1 | Taekwondo: a scoring kick to the trunk protector |  |
| `double-dutch` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_double_dutch.py: positive by injection (a teleported entry and ten 1.5 m/s kicks timed to the ropes: 10 jumps, a landing, success). No controller: entering needs the whole body across 1.0 m of rope sweep in a 0.58 s clear window (1.7 m/s; the G1 walks 0.6 m/s), then hops at 2.2 Hz; the best G1 hop controller found makes 1.7 Hz, 19 hops in 11 s with 10-15 cm clearance; human review in the pull request that adds the benchmark (PROTOCOL §2.7) | yes |
| `penalty-kick` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_penalty_kick.py: positive by injection (a real walk-up and foot contact, the ball's velocity set at the contact to 20 m/s into the corner; also a chip over the keeper); the keeper is a board on a rail driven by a force-limited servo (at most 800 N, 3 m/s), never moved by state writes, and a 15 m/s ball pushes it 3 mm; negatives: an 8 m/s corner shot saved by the keeper, a weak kick, wide, over the bar, a second touch, in without a kick, a body contact. Beating the keeper takes about 12 m/s to a corner; a G1 foot running into the ball at 4 m/s gives 3 to 5 m/s; human review in the pull request that adds the benchmark (PROTOCOL §2.7) | yes |
| `dribble-slalom` | G1 | Football: slalom dribble |  | `balance-beam` | G1 | Balance beam |  |
| `trap-ball` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_trap_ball.py: positive by injection (the robot teleported at t = 0 to stand in the real pass, which reaches its foot; the ball set to 0.3 m/s as it comes off the foot, under control 1.45 s after the touch); the pass is played by a football machine 15 m away (exit 9.6 m/s, -4 deg, 34 rad/s topspin), skids and hops, and arrives 1.97 s later at 5.9 m/s, 0.5 m to the right; negatives: a planted foot (the ball ends 4.3 m away or in an arm), rolling away, the first touch on the shin, a handball, too close to the machine, touching the machine; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |  |
| `volley-shot` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_volley_shot.py: positive by injection (the robot teleported 0.3 m forward and the ball's velocity set at the foot contact, in 11.9, out 19.4 m/s, then into the goal); the cross comes from a football machine on the left wing (exit 14.2 m/s with backspin, apex 4.5 m) and comes down knee-high 0.6 m in front at 11.7 m/s; the keeper is the physical board of header; negatives: a still leg (at most 0.89 of the speed back), a body contact, carried, wide, at the keeper (saved), a second touch, too close to the machine, touching the machine, a goal without a volley, a fall; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |  |
| `juggling` | G1 | Football: keepie-uppie |  | `vault` | H1-2 | Vault |  |
| `juggling-feet` | G1 | Football: keepie-uppie with the feet |  | `backflip` | G1 | Backflip |  |
| `juggling-head` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_juggling_head.py: positive by injection (after each touch the ball reset to 3.5 m/s up over the head: ten headers); a two-wheel football machine 1 m in front lobs the ball (exit 4.8 m/s, 76 deg) onto the top of the head, 3.5 m/s down and 1.1 m/s towards the robot; a real controller (a squat timed to the ball) makes the first header, 3.55 to 3.45 m/s (0.97, the bar is 0.95); negatives: nop (one passive bounce at 0.85, then the ground), a still head, weak touches, carried, hand, chest, shoulder, nine then the ground, touching the machine before the feed, the ball onto the machine, out of the grid, a fall; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |  |
| `header` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_header.py: positive by injection (the robot teleported 0.40 m forward and the ball's velocity set at the head contact, in 11.6, out 12.5 m/s, then into the goal); the cross comes from a football machine on the left wing 20 m away (exit 15.3 m/s with backspin, apex 4.4 m) and passes 0.45 m in front of the head, at 1.65 m, at 11.7 m/s; the keeper is a physical 80 kg board on a rail, servo-held; negatives: a still head (at most 0.85 of the speed back), chest, hand, carry, wide, headed at the keeper (saved), a second touch, too close to the machine, touching the machine, a goal without a header, a fall; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |  |
| `volleyball-bump` | Reference, on Unitree's actuator data (H2-class legs, rotor inertia, joint friction): a walk forward and to the left, then a forearm platform held through the contact; the walk, key times and the platform's joint offsets from CMA-ES, with penalties for the arms coming within 5 cm of each other and for any arm passing into the body. The serve comes from the ball machine behind the far end line; the ball meets the right forearm (it also brushes the wrist-pitch link by 2 mm), comes in at 11.3 m/s, leaves at 8.0 m/s, rises to 3.30 m and lands 0.13 m from the target centre; success after 5.85 s. The arms stay at least 2.9 cm apart and no arm passes into the body (at most 0.4 mm). It is open loop on the one serve: 4 of 6 perturbations succeed (servo gains x0.97 / x1.03 and the timing 10 ms earlier / later pass; a machine launch speed 0.5 % lower or higher fails, because the ball misses the platform). Measured (make_oracles.py): success after 5.84 s of the 10 s limit, live and in a fresh replay, ending in the same state. | yes |
| `volleyball-set` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_volleyball_set.py: No reference solution: a CMA-ES search over the walk command, the key times and the arms' key offsets (about 150 generations) finds legal sets (out 7.0-7.5 m/s against 6.6 in, apex 4.3-4.6 m, 0.5-0.8 m from the target's centre) only with the hands 5.7-6.4 cm into each other as they rise above the forehead, which the robot's self-collision rules out; with a penalty for that, no clean set was found in 20 generations. Positive by a declared magic action. The real parts: the ball machine's toss, the walking policy at zero command with the right arm raised straight up (the hands far apart; at most 0.2 mm of self-overlap), the contact with the right hand, the flight and the landing. Injected: (1) once, 0.35 m above the palm, the descending ball is moved sideways, its height and velocity kept, onto a path that meets the palm; (2) at the first contact with the hand, its velocity is set to rise to 4.6 m and come down on the target's centre. It comes in at 6.85 m/s, leaves at 8.13 m/s, peaks at 4.74 m and lands 0.82 m from the centre; the same state hash twice. Negatives: nop (the floor before the set), the same at half speed (not a set), and the referee test's other negatives.; human review in the pull request that adds the benchmark (PROTOCOL §2.7) | yes |
| `volleyball-serve` | H1-2 | Volleyball: serve | yes | `balance-board` | G1 | Balance board |  |
| `volleyball-spike` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_volleyball_spike.py: positive by injection (a 3.2 m/s ghost jump, the ball's velocity set on the fist at 2.70 m, the landing softened); the set comes from a volleyball machine on a stand at the setter's place (exit 7.7 m/s, 55 deg, at 2.3 m), peaks at 4.49 m and comes down 0.5 m from the net at 2.8 m height, 1.21 s after it leaves; negatives: a leg tuck, a lift without a jump, a still fist in a jump (0.87 back), from the floor, below the net, before the apex, upward, into the net, long, the antenna, outside the antenna, a second touch, held, the net, the centre line, a fall, touching the machine. No jump or landing controller exists for the H1-2; human review in the pull request that adds the benchmark (PROTOCOL §2.7) | yes |
| `rugby-conversion` | G1 | Rugby: conversion kick over the crossbar | yes | `springboard-3m` | G1 | 3 m springboard: forward dive straight (101A) | yes |
| `free-kick-wall` | G1 | Football: direct free kick over or around the wall |  | `trampoline` | G1 | Trampoline: tuck jump and stop | yes |
| `goalkeeper-save` | Reference: the H1-2 walks along the goal line into each shot's line (walking policy, closed loop on the pelvis), feet on the line at each kick; three 20 m/s machine shots hit the body and stay out. It learns each machine's real crossing (scatter up to 0.4 m) and side offsets over privileged resets (2-6 attempts); 10 of 10 perturbations (aims, speeds 19 and 21 m/s, kick time) succeed. Measured (make_oracles.py): success after 26.7 s of the 27 s limit, live and in a fresh replay, ending in the same state. |  |
| `first-time-shot` | Reference: a scripted standing side-foot kick of the G1 (joint targets, centre-of-mass feedback ankle balance on the left foot, the right hip turned out, the swing start timed within a control step on the rolling ball), then the walking policy recovers: the ball is met at 3.1 m/s and shot at 5.0 m/s, a goal at 9.16 s; 9 of 10 perturbations (gains, release time, ramp height) succeed. Measured (make_oracles.py): success after 9.22 s of the 15 s limit, live and in a fresh replay, ending in the same state. |  |
| `free-throw` | H1-2 | Basketball: free throw |  | `breaking-windmill` | G1 | Breaking: windmills |  |
| `catch-and-shoot` | no reference solution; the referee is tested by scripts/embodied-olympics/tests/test_catch_and_shoot.py: The chest pass is thrown by a physical two-wheel passing machine on the left wing, where a passer would stand, 4.0 m from the aim point: the shared ball machine block, copied byte for byte. It uses the football preset's 0.40 m pneumatic wheels on 15 N m speed-controlled motors with a size-7 ball: contact stiffness 42 kN/m from K = 2 pi r p at 0.56 bar, the pressure printed on match balls; restitution 0.85. A force-limited gate and ram feed the ball into the wheels. Nothing is pinned and no velocity is written after reset; the test checks the Task source and that the world has no pins or mocap bodies. The machine is trimmed in this world: the ball leaves the wheels at 2.00 s at 9.00 m/s, 12.7 degrees up, and passes 2 mm from the aim point 0.44 s later at 8.4 m/s. The test's positive is a declared magic action: real: the open two-hand catch pose before the pass; injected: the passed ball put at rest between the palms within 0.6 m; real: the arms squeeze and both hands close (the catch); injected at the shot: the ball's 52-degree shot velocity; the flight, the ring and the referee are real; the same state hash twice. Nop: not caught (deterministic). The real pass at the open hands is not caught. Negatives: hands opened, a shot 1 m wide, held to time, walking with the ball, a dragged pivot foot, a third step, a touch before the pass, the pass at the thigh, retouched after the release, up through the ring, a fall. In the ball-held arm stress the hands are penetrated by at most 0.9 mm. Collision stress scan: the machine raises no flags in the nop, torque, sweep and walk episodes. Flags appear only when the robot is placed in the machine's throat and drives its shoulder into it. The ram is pushed 5 cm past its stop, and the pinched ball gains 50 J. Header shows the same issue in the shared block. RTF (physics only, start command held, 4 x 10 s): 0.91 on the cluster, about 2.0-2.5 on the workstation (free-throw 0.84 in the same run).; human review in the pull request that adds the benchmark (PROTOCOL §2.7) |  |
| `layup` | H1-2 | Basketball: layup |  | `skateboard-ollie` | G1 | Skateboard ollie |  |
| `dunk` | H1-2 | Basketball: two-hand dunk, lowered ring |  | `inline-slalom` | G1 | Inline speed slalom |  |
| `dunk-between-legs` | H1-2 | Basketball: between-the-legs dribble and dunk |  | `sweeper-jump` | G1 | Sweeper jump (jumping the boom of a Meltdown sweeper) | yes |
| `basketball-dribble` | H1-2 | Basketball: dribble to the frontcourt |  |  |  |  |  |

## Tasks exported by Embodied Olympics

The matches Embodied Olympics exports with its own service (PROTOCOL §3), listed in
`ci_checks/benchmarks/embodied-olympics-match/exported.toml`, and their privileged twins are the benchmark
[Embodied Olympics - Match](../embodied-olympics-match/README.md) (task prefix `embodied-olympics-match-`).

## Licences

This benchmark's code: MIT. Embodied Olympics (robot library, running policy port, freeski course and ski-snow model):
MIT. Unitree G1 model with the Dex3-1 hands: MuJoCo Menagerie, BSD-3-Clause. H1-2 model and Unitree's walking policy
(unitree_rl_gym): BSD-3-Clause. Running policy (C1801SYQ/g1_running): Apache-2.0. FALCON (LeCAR-Lab): MIT. WBC-AGILE
(NVIDIA): Apache-2.0. Licence files sit next to their files in the image (`images/embodied-olympics-fun/roboolympics/SOURCE.md`).
