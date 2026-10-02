# Progress log

## M0 — Toolchain + spike (2026-10-01) ✅
- Works: Jazzy/24.04; `mujoco_ros2_control` 0.1.1 (packages.ros.org) with MuJoCo 3.12.0; venv pinned to
  `mujoco==3.12.0`; NVIDIA renderer verified for GLX offload, EGL, MuJoCo offscreen, and the sim
  process itself (GUI and headless); package demo runs; spike answers (a)–(e) all YES. RTF 1.00.
- Numbers: RTF 1.000 (headless & GUI, 1 ms step); spike JTC end error 0.011 rad (no gravcomp yet);
  gripper 0.6 → 0.590 rad in 0.40 s.
- Decisions: effort actuators + JTC PD + MuJoCo gravcomp; gripper actuator re-expressed in radians and
  named after the URDF knuckle joint; MJCF generated with MjSpec from pinned Menagerie with DH patched to
  ur_description values; Menagerie as git submodule.
- Open issues: 0.1.1 plugin-param bug (workaround in yaml); leftover Isaac `robotiq_description` /
  `ur_*` binaries (downgrade command in `docs/m0_toolchain.md`); demos package not installed.
- Details: `docs/m0_toolchain.md`.

## M1 — Feasibility study (2026-10-01) ✅ (with a scene change — please review)
- Works: `catch_ballistics` (15 tests), `catch_kinematics` (4 tests incl. DH == ur_description URDF to 1e-6),
  `catch_throw` profile + generator (8 tests), `tools/feasibility.py` sweep (10 800 candidates) with MuJoCo
  inverse-dynamics torque and clearance checks, plots in `docs/feasibility/`.
- Numbers: brief's geometry needs up to 2.7x the 180 deg/s limit (constant orientation) and ≥ 0.92x even fully
  optimised for a 0.7 m flight. Chosen: pedestals 0.35 m, release (-0.05, 0.25, 1.25), catch (0.35, 0.25, 1.15),
  65 deg, 2.14 m/s, flight 0.442 s, spin -1.93 rad/s; peak joint speed 0.80, torque 0.64 of limits,
  position margin 1.61 rad, clearance 4.8 cm.
- Decisions: robots on 0.35 m box pedestals; 0.4 m flight; palm pitch during the stroke (κ = -0.9 rad/m);
  primitive palm insert. Limits unchanged; acceleration bounded via torque (UR publishes none).
- Open issues: perfect tracking assumed (M4 measures); no self-collision check in the sweep.
- Details: `docs/m1_feasibility.md`.

## M2 — Scene and bringup (2026-10-01) ✅
- Works: one-command bringup (sim + controllers + move_group + RViz, all on the NVIDIA GPU); arms hold their
  start pose (gravcomp); MoveIt jogs both arms and both grippers (12/12); scene/ground truth/reset/spawn
  services; RTF and GPU monitors.
- Numbers: FK URDF vs MJCF worst 1e-9 mm / 4e-6 deg (50 configs x 3 frames x 2 arms); static grasp slip
  1.61 mm (A) / 1.46 mm (B) over 10 s at up to 2.1 rad/s (< 2 mm); grip ~46 N per finger; RTF 1.000.
- Decisions: palm insert 40x12 mm and gripper command cap 0.70; palm_up pose by IK-branch clearance search;
  SRDF generated headless; rclpy MoveIt client (no moveit_py binary).
- Open issues: /joint_states lags ground truth by ~2.5 ms (measured; relevant to M5 timing); deprecated
  GripperActionController (warning only).
- Details: `docs/m2_scene_bringup.md`.

## M3 — Perception and pick/place (2026-10-01) ✅
- Works: overhead HSV detection with confidence; colour check from B's wrist camera; pick (A, from detection)
  and place (A and B) skills on MoveIt; isolated B-place test with a spawned cube.
- Numbers: detection 1.74 mm mean / 4.66 mm max, yaw 0.4° mean (60 cubes, 0 misses); colour check 3/3;
  B place error 0.39-0.63 mm; A pick 3/3, place-back 0.11-3.82 mm, all in slot.
- Decisions: top-grasp TCP offset 17 mm (pad kinematics), pallets moved to x = ∓0.45, URDF pallet with real
  borders, IK-seeded MoveIt joint goals, collision checking off only for short in-slot vertical moves,
  lighting reduced + overhead fov 32°.
- Open issues: the URDF gripper model differs from the MJCF one (fingertips ~4 mm longer); B's wrist camera is a
  bracket camera with no collision geometry.
- Details: `docs/m3_perception_pick_place.md`.

## M4 — The throw (2026-10-01) ✅
- Works: throw action server (plan from YAML → JTC with torque feed-forward, future start stamp, gripper loosen
  on the sim clock), ThrowPlan latched on /throw_plan before release, full chain pick → re-seat → throw.
- Numbers (20 throws): release speed error +6.6 ± 2.5 mm/s, direction 0.28 ± 0.03°, spin −1.890 ± 0.007 (plan
  −1.928) rad/s, catch-height crossing (3.9, 1.6) ± (0.6, 0.05) mm from the catch point, table landing
  (9.6, 2.0) ± (2.1, 0.05) mm from prediction. Without feed-forward: +0.264 m/s, 55 mm long.
- Decisions: inverse-dynamics feed-forward table (hashed against its inputs); loosen to 0.30 at t = 0.12 s
  (sweep); cube re-seated on the palm before the throw.
- Open issues: separation time only resolvable to ±16 ms from GT (stamp jitter); the feed-forward table must be
  regenerated (venv) when the throw geometry changes — the node warns and runs without it otherwise.
- Details: `docs/m4_throw.md`.

## Pre-M5 review items (2026-10-01/02)
- URDF gripper collision geometry = Menagerie (worst vertex 0.84 mm, pads 0.31 mm / 0.81°), FK test covers pads +
  all gripper collision geoms; MoveIt collision checking back on for in-slot moves (commit 75e6ba6).
- `apt-cache policy`: the Isaac `ros-jazzy-robotiq-description 9.0.1-0noble` and `ros-jazzy-ur-* 99.2.0-0noble` are
  **still installed** (the downgrade did not take). Only robotiq_description's visual meshes are used; nothing
  changed. Rebuild + 40 tests (system python) + 5 venv tests: all pass after regenerating the stale throw
  feed-forward table (gripper/sensor change; max difference 0.09 N·m).

## M5 — The catch (2026-10-02) ✅
- Works: B pre-positions from the throw-plan preview (elbow-up cup pose, 58 mm from the flight path), fingers close
  on the palm force sensor (contact trigger, no weld — tested), color from the wrist camera, in-hand pose from the
  wrist camera (square fit along the closing axis), corrected top-down place; `catch_eval` judges with ground truth.
- Numbers (30 throws, 10/color): **30/30 caught**, color 30/30, **30/30 placed in slot**, place error x −0.70 ± 0.07
  mm (max 0.83), **yaw 0.00°**, trigger −2.7 ± 0.7 ms vs predicted arrival. In-hand estimate error ≤ 0.75° / 0.3 mm.
  Baselines: time trigger 30/30 (its close instant scatters +6 ± 12 ms, up to +50 ms); no in-hand correction
  15/15 in slot but x error 1.67 ± 1.19 mm (max 3.39). Off-centre catches (experiment knob, up to 18° in the
  hand): corrected 9/9 in slot, uncorrected 0/9 (8–14 mm off, resting tilted on a border).
  Regression: pick/place 6/6, 20 throws +8.2 ± 1.8 mm/s, static grasp A one 2.35 mm outlier in 8 runs (others
  ≤ 1.63 mm), B ≤ 1.60 mm.
- Decisions: grip loosen timed by the controller (gripper A → JointTrajectoryController with a time-stamped
  loosen; a Python sim-clock sleep was up to 97 ms late and caused a miss); palm-force queue 2 s deep, subscribed
  at goal start (an rclpy lag dropped an 82 N spike once); carry via home before placing; /tf_static-only TF in
  perception; URDF wrist-camera frame.
- Open issues: the clean demo sits ~25 mm/s inside the catch envelope's across-path tolerance (±25–50 mm/s); no
  flight tracking (B trusts the plan); every Python node burns a core on the 1 kHz /clock (fixed in v1.0).
- Details: `docs/m5_catch.md`.

## M6 — Full loop and demo (2026-10-02) ✅
- Works: `run_demo` state machine (detect → random pick → seat → B pre-positions → throw → contact catch → wrist-camera
  color → in-hand pose → corrected place → overhead verify → repeat), failure recovery (log → open grippers → arms
  home → sim reset of that cube → retry, ≤ 3 attempts per cube), opt-in fault injection for the recovery path,
  rosbag2 recording + ground-truth check of every run, replay renderer → demo mp4.
- Numbers: seeds 1–3 (orders R-G-B, G-B-R, B-R-G) complete in 96–99 s with 3 throws each; seed 4 with an injected
  late grip loosen: miss → recovery → re-throw → complete in 124 s. VERIFY agrees with ground truth 12/12, final
  cubes 0.6–0.9 mm from their slot centres. Demo: `docs/media/demo_seed1.mp4`, `demo_recovery_seed4.mp4`.
- Decisions: rosbag2 instead of an rclpy recorder (it captured ~5 % of the samples); hashed string seeds (integer
  seeds 1–3 gave the same cube order); FastDDS stale shared-memory cleanup in `sim_ctl.sh start` (killed processes
  left segments that made a new process' discovery fail).
- Open issues: drops only via fault injection in this deterministic sim; the reset is a teleport; no flight
  tracking; videos are replays of recorded state (not the live sim window).
- Details: `docs/m6_full_loop.md`, `README.md`.

## v1.0 housekeeping (2026-10-02)
- Floating object near B's gripper in `demo_seed1.mp4` (t ≈ 19.6 s): identified by segmentation rendering of the
  replayed frame as `b_robotiq_wrist_cam_housing` (body `b_robotiq_base`; every other geom there belongs to B's arm,
  gripper or the caught `cube_red`). It was attached, but 13 cm beside and 15 cm above the gripper base with
  nothing visible holding it. Added an L-bracket (`scene.yaml: cameras.b_wrist_bracket`): MJCF visual-only boxes
  (body masses/inertias bit-identical), URDF visual + collision (SRDF regenerated), new test that the URDF housing
  and bracket boxes match the MJCF geoms (≤ 0.001 mm). Clearances: ≥ 12 mm below the lowest finger point, ≥ 62 mm
  outside the open fingers. Never in the wrist camera's view; its shadow reaches the cube only outside the color /
  in-hand windows (0 changed cube pixels in those windows, seeds 1–4). Demo videos and poster re-rendered.
- README architecture diagram: top-to-bottom (sensing → ROS 2 nodes → controllers → physics, every arrow down),
  exported to `docs/media/architecture.svg` (plain SVG text, no foreignObject) by `tools/export_architecture.sh`
  from the README's Mermaid block.
- Isaac downgrade: **not applied** (`apt-cache policy`: `ros-jazzy-robotiq-description 9.0.1-0noble` and
  `ros-jazzy-ur-*` `99.2.0-0noble` from isaac.download.nvidia.com still installed; no apt log ever downgraded
  them; the `isaac-ros-lowprio` pin alone cannot downgrade). Dry run against the ros.org
  `robotiq-description 0.0.1` .deb: same 9 visual meshes (identical vertices) under `meshes/visual/` instead of
  `meshes/visual/2f_85/`; `tools/gen_gripper_urdf.py` then changes only those 9 mesh paths (kinematics, inertias,
  collisions identical) — re-run it after the downgrade. Rebuild + all tests pass: 59 = the 56 before + the
  housing/bracket test + 2 sim_clock tests (52 system python, 7 venv).
  Feed-forward table regenerated (scene.yaml changed): torques identical (max |Δ| 0.0), new input hash.
- /clock CPU: with use_sim_time rclpy subscribes /clock (1 kHz) on the node; under a MultiThreadedExecutor that
  alone costs 108 % of a core (bare node; SingleThreadedExecutor 21 %, EventsExecutor 7 %).
  `catch_sim.sim_clock.offload_sim_clock` moves it to a helper node on an EventsExecutor thread (node keeps its
  executor and use_sim_time; tested incl. sim-time timers); the monitors spin on an EventsExecutor.
  Idle, % of one core (`tools/node_cpu.py`, 30 s): scene_manager 107.5 → 5.8, color_check 106.1 → 18.1,
  throw_node 106.7 → 95.2, catcher 80.7 → 51.3, pallet_detector 141.7 → 110.3, monitors 22–32 → 6–10;
  all Python nodes 643 → 309 %. Regression run_demo seed 1: complete R-G-B, 3 throws, contact trigger −2.3 ms ×3,
  verify offsets identical to v1; the color step 5.9 → 0.4 s (color_check is no longer saturated), run 95.7 → 85.8 s.
- Open issues: throw_node, catcher (MoveItClient) and run_demo keep a permanent /joint_states (≈ 470 Hz)
  subscription on their MultiThreadedExecutor (≈ 95 / 50 / 50 % of a core); pallet_detector processes every
  overhead image (~12 Hz, ~110 %). The m5 clips in docs/media still show the old housing without the bracket.

## v1.0.1 cleanup (2026-10-02)
- Robot descriptions pinned in the workspace, system untouched (no apt changes): `src/external/robotiq_description`
  and `src/external/ur_description` are git submodules at the jazzy release tags packages.ros.org builds from
  (`release/jazzy/robotiq_description/0.0.1-3` = `4131e61`, `release/jazzy/ur_description/3.5.1-1` = `6e45dd1`).
  `ros2 pkg prefix` → `install/` for both; the Isaac `robotiq_description 9.0.1` / `ur_* 99.2.0` in /opt/ros are no
  longer used (`ur_moveit_config` and the other `ur_*` never were).
  - Why submodules (option 1) and not copying files into catch_description (option 2): option 2 is fewer bytes
    (~10 MB of ur5e + 2F-85 meshes vs a 73 MB ur_description checkout, kept out of our git history by the
    submodule) but is a fork: `ur_macro.xacro` hard-codes `$(find ur_description)` includes and
    `visual_parameters.yaml` the mesh package, the robotiq macro its ros2_control include, and the DH test and
    `gen_gripper_urdf.py` read the upstream xacros — all would need edits, and every upstream bump a re-copy.
    The release-tag submodules are byte-identical to the official .debs (ur_description: identical to the
    installed 3.5.1; robotiq: the 9 2F-85 visual meshes identical to the Isaac ones), add 2 small entries to
    `.gitmodules`, and a bump is one checkout.
  - `tools/gen_gripper_urdf.py` rerun: only the 9 visual mesh paths changed (`meshes/visual/2f_85/x.dae` →
    `meshes/visual/x.dae`). robotiq 0.0.1 writes absolute `file://$(find ...)` mesh paths; the generator now
    stores `package://robotiq_description/...` (the URDF must not embed an install path). Expanded scene URDF:
    identical to v1.0 except those 9 × 2 filenames → FK identical. New test: every URDF mesh resolves into the
    workspace, never /opt/ros. Clean rebuild (`rm -rf build install`): 60/60 tests pass (52 + the new one with
    system python, 7 venv). Feed-forward table not regenerated: its inputs (throw/scene/physics yaml, MJCF) are
    unchanged and its hash matches.
  - colcon warns that the overlay overrides underlay packages; harmless for description-only packages;
    `--allow-overriding robotiq_description ur_description` silences it (README).
- m5 clips re-rendered with the bracket (same recordings, views, windows and frame counts; commands in
  `docs/m5_catch.md`).
- Still open for v2 Phase 2: permanent /joint_states subscriptions in throw_node, catcher and run_demo.

## v2 Phase 1 — offline feasibility (2026-10-02) ✅
- Scope: random throws, vision-only B, chopsticks vs cup; offline only (`tools/v2_*.py`, `tools/v2_study.yaml`,
  `docs/v2/`), ROS pipeline unchanged. `gripper_profile: realistic | fast_idealized` (default realistic) in every
  result; fast_idealized is not a real 2F-85.
- Ready pose: TCP (0.30, 0.25, 1.10), approach 60° up towards A, fingers across the flight; min joint margin
  1.25 rad, 84 mm clearance; 58 % of a broad throw family catchable at a 0.20 s decision.
- Perception: 2 orthogonal cameras required (1 camera: ~10 cm depth error); measured stereo noise ~0.8/0.8/1.9 mm
  + 2 mm z bias, correlated (end-to-end 1.7× the white-noise model); 120 Hz/2 mm/25 ms → 10 mm at 0.21 s;
  rendering 1.4 ms per 320×240 camera, physics RTF unaffected (plugin renders on its own thread).
- Throws: 259 of 1080 (target, angle) pairs feasible for A, 213 also catchable; square-arrival palm pitch → 150 /
  121. Library recommended over online ID (pinocchio = MuJoCo exactly with the same parameters; the URDF's
  inertias differ by up to 14 N·m; ros-jazzy-pinocchio 4.1.0 exists but is not installed).
- Catch (120 throws per row): realistic 2F-85 speed — natural pinch 86–87 % (close 10 ms after nominal), cup
  pre-closed 90 % on square throws (40 % with open fingers on random throws: tilted cubes slide out before the
  150 mm/s fingers close); fast_idealized — 98–100 %. Side chopsticks: 0 % within limits (plans 2.5–51× over);
  velocity matching: ≤ 40 % deliverable, no benefit.
- Finding for v1: the v1 sim gripper closes 3–5× faster than the real 2F-85 (M5's 30/30 cup used it).
- Details and Phase 2 plan: `docs/v2_phase1_feasibility.md`.

## v2 Phase 2 — step 0: corrections from the Phase 1 review (2026-10-02)
- Pinch numbers reconciled (docs/v2_phase1_feasibility.md, "Corrections"): the 86–87 % headline is the realistic pinch
  closing 10 ms after the nominal instant (56 mm pre-gap, measured perception); the plots showed nominal timing
  (76 / 71 %), 0.25 matching (79 / 72 %) and a 5x-force actuator at 150 mm/s (55–71 %). Plots relabelled, panel (f) =
  headline configurations, new `catch_corrections.png`.
- Speed vs force: new profile `fast_close` (10x speed, realistic force): pinch 100 % at nominal timing (fast_idealized
  100 %, realistic 87 % at +10 ms) -> the gain is speed, not force. Realistic-force speed sweep: 75/150/300/600 mm/s
  -> 43/81/96/100 % (best timing).
- Gripper command latency 10 ms (Robotiq manual 4.7.1: 200 Hz command rate, >= 5 ms between commands, 115.2 kbps;
  estimate, not measured), offline and in the sim (catch_sim/gripper_model.py: rate-limited 150 mm/s ramp, delayed;
  both grippers are JTCs). Sweep (B assumes 10 ms): pinch 76/87/88/82 %, pre-closed cup 90/89/88/86 % at 0/10/20/30 ms.
- v1 honesty: the v1 sim gripper closed 3–5x faster than a 2F-85. Re-run with the realistic model (150 mm/s, 10 ms,
  both arms): **v1 catch eval 30/30 caught, 30/30 placed** (trigger -2.8 ± 0.6 ms; original 30/30); **M4 20/20, speed
  error +5.4 ± 1.8 mm/s** (v1 +6.6 ± 2.5), direction 0.27°, catch-height crossing +2.3 mm. The M1 throw lands square on
  a static, plan-informed cup, so the slow close does not matter there (Phase 1: it does for random throws).

## v2 Phase 2 — step 1: /joint_states CPU fix (2026-10-02)
- `catch_sim.joint_state_cache.JointStateCache`: latest /joint_states on a helper node + EventsExecutor thread (same
  pattern as the /clock offload); used by MoveItClient (catcher, run_demo, evals, skills) and throw_node.
- Idle CPU, % of one core (tools/node_cpu.py, 30 s, same sim): **throw_node 93.2 -> 9.8, catcher 53.2 -> 9.8,
  run_demo's node 46.6 -> 11.8** (docs/v2/phase2/cpu_idle_{before,after}.json).
- Regression with the v1 gripper (MJCF variant + profile v1, so only the CPU fix differs): **M4 20/20, +7.1 ± 2.2 mm/s,
  0.28 ± 0.03°, crossing (+4.1, +1.6) mm** (v1: +6.6 ± 2.5, 0.28 ± 0.03, (+3.9, +1.6)); **run_demo seed 1 complete R-G-B,
  3 throws, trigger -2.3 ms x3, verify offsets identical to v1 to 0.1 mm**, 95.7 -> 88.3 s.
- Found on the way: move_group also published /robot_description (from the default MJCF) and raced
  robot_state_publisher; it no longer publishes (`mujoco_model:=` variants now load). Short gripper ramps stamped
  "now" could arrive after their end (rejected): immediate commands use a zero stamp.

