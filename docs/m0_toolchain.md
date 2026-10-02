# M0 — Toolchain verification and spike

Date: 2026-10-01. All evidence below was produced on the target laptop; the spike files are in
`tools/m0_spike/` (re-runnable).

## Versions

| Item | Value | Evidence |
|---|---|---|
| OS | Ubuntu 24.04.5 LTS (noble) | `lsb_release -a` |
| ROS | Jazzy (`$ROS_DISTRO=jazzy`) | env |
| `ros-jazzy-mujoco-ros2-control` | **0.1.1-1noble.20260905** from packages.ros.org (Isaac 999.0.1 at prio 100, not installed) | `apt-cache policy ros-jazzy-mujoco-ros2-control` |
| `ros-jazzy-mujoco-vendor` | 0.1.0 → **MuJoCo 3.12.0** | `mjVERSION_HEADER 3012000`, `libmujoco.so.3.12.0` |
| venv `mujoco` | **3.12.0** (pinned in `tools/requirements.txt`) | `pip freeze` |
| JTC / ros2_controllers | 4.42.1 | dpkg |
| MoveIt | 2.12.4 | dpkg |
| `ur_description` | 3.5.1, workspace overlay (`src/external`, since v1.0.1) | `ros2 pkg prefix` |
| `robotiq_description` | 0.0.1, workspace overlay (`src/external`, since v1.0.1) | `ros2 pkg prefix` |
| Menagerie | commit `4d038b3feae26ec82b46a4d586379114012a8ac7` (2026-09-29), git submodule | `git submodule status` |

### Robot descriptions are pinned in the workspace (v1.0.1)
No special apt versions are needed. The workspace builds its own `robotiq_description` and `ur_description`
from git submodules in `src/external/`, pinned to the jazzy release tags that packages.ros.org builds its
binaries from (`ros2-gbp/*-release`, see `src/external/README.md`):

| Package | Tag | Commit |
|---|---|---|
| `robotiq_description` | `release/jazzy/robotiq_description/0.0.1-3` | `4131e61` |
| `ur_description` | `release/jazzy/ur_description/3.5.1-1` | `6e45dd1` |

The overlay takes precedence over whatever is in `/opt/ros/jazzy` (`ros2 pkg prefix` → `install/...`), so it
does not matter that this laptop still has NVIDIA Isaac builds installed (`ros-jazzy-robotiq-description
9.0.1-0noble`, `ros-jazzy-ur-{robot-driver,moveit-config,controllers,...} 99.2.0-0noble`). The system is left as it
is (a past downgrade attempt is not worth the risk). `ur_moveit_config` and the other `ur_*` packages are not used
(MoveIt config is `catch_moveit_config`). `colcon build` warns that the two packages override the underlay; they
are description-only (no headers, no libraries), so `--allow-overriding robotiq_description ur_description`
just silences the warning. The test `test_urdf.py::test_meshes_resolve_into_the_workspace` fails if any mesh of
the expanded URDF resolves into `/opt/ros`.

## GPU / rendering

| Path | Result |
|---|---|
| `glxinfo -B` default | Mesa Intel (ARL) — the desktop default, as expected for PRIME on-demand |
| `glxinfo -B` with `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` | **NVIDIA GeForce RTX 5060 Laptop GPU**, driver 595.91.07 |
| `eglinfo -B` with `__EGL_VENDOR_LIBRARY_FILENAMES=.../10_nvidia.json` | **NVIDIA** (device, surfaceless platforms) |
| MuJoCo Python `Renderer` with `MUJOCO_GL=egl` (venv) | `GL_RENDERER = NVIDIA GeForce RTX 5060 Laptop GPU/PCIe/SSE2` |
| `ros2_control_node` (GUI) with `tools/nv_env.sh` | listed by `nvidia-smi` as a **G** process, 668 MiB |
| `ros2_control_node` (headless) with `tools/nv_env.sh` | listed by `nvidia-smi` as a **G** process, 123 MiB |

Important: the camera plugin uses **GLFW whenever a display exists, even in headless mode**; without the
PRIME variables it would silently render on Intel. Therefore the bringup launch sets the variables
itself and a GPU check node (M2) verifies via `nvidia-smi` that `ros2_control_node` is on the dGPU
and errors out otherwise.

Gotcha: with `MUJOCO_GL=egl` the Python `Renderer` raises an `EGLError` at interpreter exit if it is
garbage-collected after EGL teardown; tools call `renderer.close()` explicitly.

## Package demo
`ros2 launch mujoco_ros2_control_demos 01_basic_robot.launch.py headless:=true`: 4 controllers active,
`position_controller` command `[0.5, -0.5]` → `joint1=0.505, joint2=-0.500`, 0 errors in the log.

## Spike: answers with evidence

Spike = `tools/m0_spike/gen_spike.py` (MjSpec composition of 2× Menagerie UR5e + 2F-85, prefixes
`a_`/`b_`, a free cube, an overhead camera) + `spike.urdf.xacro` (`ur_description` macro ×2 +
`robotiq_description` macro, one `ros2_control` system) + `spike_client.py`.

**(a) Two prefixed arms in one model — YES.** One `MujocoSystemInterface` system with 14 joints;
`arm_a_controller` and `arm_b_controller` (JTC) and two gripper controllers all active. Both arms
reached a 3 s trajectory target (`error_code=0`). Joint mapping is by name: the ros2_control joint
name must equal the MJCF joint name, so the MJCF uses the URDF names (`a_shoulder_pan_joint`, …).

**(b) 2F-85 through ros2_control — YES, one position interface, after one change.** The source
(`register_mujoco_actuators`) maps a *tendon* actuator to the joint that has the **same name as the
actuator**, and (`getActuatorType`) classifies Menagerie's `general` actuator (affine bias,
`biasprm[1]=-100`) as POSITION, writing `ctrl = command` directly. Menagerie's ctrl is 0–255 while
the reported state is the driver-joint angle in rad, so a GripperActionController would compare
mismatched units. Fix in the generated MJCF: the driver joint is renamed to the URDF name
`<p>robotiq_85_left_knuckle_joint`, and the actuator (same name) becomes
`<position tendon="split" kp="100" kv="10" ctrlrange="0 0.8" forcerange="-5 5">` — mathematically the
same force law as Menagerie (`0.3137·ctrl − 100·L − 10·L̇` with ctrl rescaled), but in radians. Result:
`position_controllers/GripperActionController`, command 0.6 → state 0.590, `reached_goal=True` in 0.40 s
sim time. The tendon + equality coupling is left untouched.

**(c) Cameras — YES** via `mujoco_ros2_control_plugins/CameraPlugin`: every MJCF `<camera>` publishes
`color` (rgb8), `depth`, `camera_info`; 640×480 frame verified (`docs/media/m0_spike_overhead.png`).
Limits: one publish rate for all cameras (matters for the later 60–120 Hz side camera); a per-camera
`polled` policy with a trigger service exists (useful for B's color check). Gotcha: the plugin
hard-codes the parameter prefix `mujoco_plugins.mujoco_camera_plugin.`, so the yaml key must be
exactly `mujoco_camera_plugin`.

**(d) Ground-truth free-body poses — YES** via `FreeJointStatePublisherPlugin` on
`/free_joint_state_publisher/free_joint_states` (`FreeJointStateArray`, pose + twist, world frame,
sim-time stamps). **Bug in 0.1.1:** the plugin reads its parameters *un-prefixed* (rclpy/rclcpp
sub-nodes do not prefix parameter names), so `mujoco_plugins.free_joint_state_publisher.publish_rate`
is ignored. Fixed upstream in 0.1.2 ("Fix plugin parameter namespaces (#291)"). Workaround (no source
build): set `publish_rate`/`body_names`/`frame_id` as top-level params of `mujoco_ros2_control_node`;
verified: "publishing 1 free-joint body … at 100.0 Hz".

**(e) Reset / respawn — YES** via services on `/mujoco_ros2_control_node/`:
`set_free_joint_state` (atomic multi-body teleport incl. twist; verified: cube teleported with
`vx=1.0` and flew), `reset_world` (startup state or a keyframe, with joint/free-joint overrides; doc
warns to deactivate controllers first), `set_pause`, `step_simulation`. 0.1.2 additionally restores
`eq_active` on reset (only relevant for the optional debug weld).

**Real-time factor:** 1.000 headless and 1.000 with GUI (2 arms, 1 ms timestep); `/clock` ≈ 925 msg/s.

## Findings that shape the design

1. **Arm actuation: effort + JTC PD + MuJoCo gravity compensation.** Menagerie's UR5e actuators are
   position servos `kp·(q*−q) − kv·q̇` with kv/kp = 0.2 s: the damping pulls towards *zero* velocity,
   so at 3 rad/s the steady lag is ≈0.6 rad — useless for a throw, and ros2_control can drive only
   one actuator per joint. JTC 4.42.1 with an `effort` command interface computes
   `u = ff_vel·q̇_d + effort_ff + PID(e_q, e_q̇)` sampled one cycle ahead (source read:
   `joint_trajectory_controller.cpp` L417–425), i.e. a proper PD on *velocity error* with optional
   per-point feed-forward torque. So the MJCF uses `motor` actuators clamped to the published UR5e
   torques (150/150/150/28/28/28 N·m), and gravity is compensated in MuJoCo (`gravcomp` on arm/gripper
   bodies, `actuatorgravcomp` on arm joints so the compensation counts against the torque limits —
   like the real UR controller's model-based compensation). Without it the arms fall until the JTC
   activates (seen in the spike GUI). Tracking quality is measured in M2/M4.
2. **Determinism:** physics runs in its own thread paced to wall-clock; the controller loop sleeps on
   `/clock`. Runs are therefore repeatable but not bit-identical; M4 reports the spread.
3. **Kinematics mismatch Menagerie vs ur_description:** Menagerie rounds the UR5e DH values
   (0.163 vs 0.1625, 0.392 vs 0.3922, 0.134 vs 0.1333 lateral, 0.1 vs 0.0997/0.0996). Accumulated, this
   exceeds the 1 mm FK budget, so the generator patches the MJCF body offsets to the exact
   `ur_description` values (visual meshes shift < 1 mm).
4. **MjSpec composition gotchas:** renaming elements inside a spec silently drops tendons/equalities/
   actuators that reference them by name; attaching with an empty prefix breaks `to_xml()` (duplicate
   `main` default class). Working recipe: rename in the XML text (attribute-aware) before parsing and
   attach the gripper with prefix `robotiq_`, which yields the URDF joint name for free.

No blocker: `mujoco_ros2_control` 0.1.1 covers everything essential; no source build.
