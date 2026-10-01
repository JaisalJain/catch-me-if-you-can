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
  flight tracking (B trusts the plan); every Python node burns a core on the 1 kHz /clock.
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
