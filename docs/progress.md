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
