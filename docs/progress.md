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
