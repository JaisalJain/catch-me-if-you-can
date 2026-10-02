# catch — two UR5e arms that throw and catch colored cubes (ROS 2 Jazzy + MuJoCo)

## Status

- M0 toolchain ✅ (see `docs/m0_toolchain.md`)
- M1 feasibility ✅ (scene changed: 0.35 m pedestals, 0.4 m flight — docs/m1_feasibility.md)
- M2 scene/bringup ✅ (docs/m2_scene_bringup.md)
- M3 perception + pick/place ✅ (docs/m3_perception_pick_place.md)
- M4 throw ✅ (docs/m4_throw.md)
- M5 catch ✅ (docs/m5_catch.md)
- M6 full loop + demo ✅ (docs/m6_full_loop.md, README.md)

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

## Build / run

- Build: `source /opt/ros/jazzy/setup.bash && colcon build && source install/setup.bash`
- Everything (GUI): `ros2 launch catch_bringup bringup.launch.py` (args: `headless:=true rviz:=false`)
- Scripted runs: `tools/sim_ctl.sh start catch_bringup bringup.launch.py headless:=true rviz:=false`,
  `tools/sim_ctl.sh wait 5 90`, ..., `tools/sim_ctl.sh stop` (log: `log/sim_latest.log`)
- M2 checks: `ros2 run catch_tasks jog_test`, `ros2 run catch_tasks grasp_test --robot a`
- M3 checks: `ros2 run catch_perception eval_detection`, `ros2 run catch_tasks pickplace_test`
- M4: `ros2 run catch_tasks throw_eval --throws 20 [--from-pallet] [--out docs/m4/throws_20.json]`; plots `tools/plot_throws.py`
- M5: `ros2 run catch_tasks catch_eval --throws 30 [--trigger time] [--no-place] [--no-correction] [--cup-offset 0.015]
[--out docs/m5/x.json] [--rec-dir log/m5/x]`; plots `tools/plot_catch.py`; offline `tools/catch_study.py [--sweep|--envelope]`
- M6: `ros2 run catch_tasks run_demo --seed 1 [--fault-throw N] [--out docs/m6/run.json]`; recorded batch
  `log/run_m6.sh 1 2 3`-style (record with `install/catch_tasks/lib/catch_tasks/record_run --out x.npz`, stop with SIGINT)
- Clips / video (venv): `tools/render_replay.py rec.npz --out x.mp4 [--view catch|throw|overview]`,
  `tools/render_demo.py rec.npz run.json --out docs/media/demo.mp4`
- After changing throw.yaml (except `release:`), scene.yaml or physics.yaml: `env -u PYTHONPATH .venv/bin/python tools/throw_feedforward.py`
- Venv integration tests (ROS sourced): `env -u PYTHONPATH .venv/bin/python -m pytest src/catch_description/test/test_fk_agreement.py src/catch_description/test/test_mjcf_up_to_date.py`
- After changing the URDF/SRDF-relevant geometry: `env -u PYTHONPATH .venv/bin/python tools/gen_srdf.py`

## Known gotchas (project)

- Never command the gripper beyond 0.70 (palm insert clearance). Cubes must be spawned with the palm facing up.
- /joint_states lags the free-joint ground truth by ~2.5 ms; the GT topic itself has ~1-2 ms stamp jitter.
- Don't activate .venv in ROS terminals (colcon/ROS would pick up the venv python); call .venv/bin/python explicitly.
- Temporary files: use log/ (gitignored).
- rclpy is slow: a MultiThreadedExecutor costs ~1 core per 1 kHz topic. /clock (1 kHz) is taken off the node's
  executor by `catch_sim.sim_clock.offload_sim_clock(node)` (call it after `ex.add_node` in every new sim-time node;
  simple non-blocking nodes use `spin_events`). Measure with `python3 tools/node_cpu.py`. Subscribe to 500 Hz topics
  (/joint_states, wrench) only while needed, with deep queues (the catcher's palm-force queue is 1000: a lagging
  executor once dropped the 2 ms impact spike). Never time actuation with Python sleeps on the sim clock: the grip
  loosen is a time-stamped trajectory on gripper_a_controller (a JTC); a Python-timed loosen was up to 97 ms late.
- Gripper A is a JointTrajectoryController (position, interpolate_from_desired_state), gripper B a
  GripperActionController. MoveItClient.gripper() hides the difference.
- New subscriptions need DDS discovery time: subscribe before you need the data (catcher: at goal start).
- Don't `pkill -f`/`pgrep -f` with a pattern that also appears in your own command line (kills your shell);
  kill by PID.
- B's wrist camera frame `b_wrist_cam_optical` is in the URDF (tested against the MJCF camera); perception nodes
  use a /tf_static-only buffer (a full TransformListener drowns rclpy in 500 Hz /tf).

## Offline tools / tests

- Regenerate MJCF after editing `src/catch_description/config/{scene,physics}.yaml`:
  `env -u PYTHONPATH .venv/bin/python src/catch_description/scripts/build_mjcf.py`
- Feasibility sweep + plots: `env -u PYTHONPATH .venv/bin/python tools/feasibility.py && ... tools/plot_feasibility.py`
- Pure-python unit tests (system python, no build needed):
  `PYTHONPATH=src/catch_ballistics:src/catch_kinematics:src/catch_throw:$PYTHONPATH python3 -m pytest -q src/catch_{ballistics,kinematics,throw}/test`

Do not add claude as co author in git pushes
