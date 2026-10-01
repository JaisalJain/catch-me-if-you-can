# catch — two UR5e arms that throw and catch colored cubes (ROS 2 Jazzy + MuJoCo)

## Status
- M0 toolchain ✅ (see `docs/m0_toolchain.md`)
- M1 feasibility ✅ (scene changed: 0.35 m pedestals, 0.4 m flight — docs/m1_feasibility.md)
- M2 scene/bringup — in progress
- M3 perception + pick/place, M4 throw — pending
- Progress log: `docs/progress.md`

## Environment rules
- ROS nodes: system Python + apt only. Never `pip --break-system-packages`, `sudo pip`, `pip --user`.
- Offline tools (`tools/`): venv `.venv` (isolated). Run with `env -u PYTHONPATH .venv/bin/python ...`
  (ROS setup puts `/opt/ros/.../site-packages` on PYTHONPATH, which leaks into the venv otherwise).
  Install with `.venv/bin/pip`, then refresh `tools/requirements.txt` (pinned).
- venv `mujoco` must equal the `mujoco_vendor` version (3.12.0).
- `catch_ballistics` may only depend on numpy/scipy (system numpy 1.26 / scipy 1.11 and venv versions).
- GPU: `source tools/nv_env.sh` (PRIME offload for GLX + NVIDIA EGL). Bringup launch sets these itself.
- Kill a running sim safely (don't `pkill -f "ros2 launch"` — it matches your own shell):
  `pkill -f "^/opt/ros/jazzy/lib/mujoco_ros2_control/ros2_control_node"`

## mujoco_ros2_control 0.1.1 gotchas
- ros2_control joint names must equal MJCF joint names. Tendon actuators bind to the joint with the
  actuator's name.
- Camera plugin yaml key must be exactly `mujoco_camera_plugin`.
- FreeJointStatePublisherPlugin reads un-prefixed params (`publish_rate`, `body_names`, `frame_id`) on
  node `mujoco_ros2_control_node` (bug fixed in 0.1.2).
- Services: `/mujoco_ros2_control_node/{set_free_joint_state,reset_world,set_pause,step_simulation}`.
- JTC gains must be floats (`3000.0`), ints are rejected.

## Conventions
- World frame at table center, x from A to B, z up, table top z=0.75.
- Prefixes `a_` (thrower, x=-0.8) and `b_` (catcher, x=+0.8).
- Parameters (geometry, contact, timings) live in YAML, not code.
- Commit after each working step; don't push.

## Offline tools / tests
- Regenerate MJCF after editing `src/catch_description/config/{scene,physics}.yaml`:
  `env -u PYTHONPATH .venv/bin/python src/catch_description/scripts/build_mjcf.py`
- Feasibility sweep + plots: `env -u PYTHONPATH .venv/bin/python tools/feasibility.py && ... tools/plot_feasibility.py`
- Pure-python unit tests (system python, no build needed):
  `PYTHONPATH=src/catch_ballistics:src/catch_kinematics:src/catch_throw:$PYTHONPATH python3 -m pytest -q src/catch_{ballistics,kinematics,throw}/test`
