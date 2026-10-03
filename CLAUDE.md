# catch — two UR5e arms that throw and catch colored cubes (ROS 2 Jazzy + MuJoCo)

## Status

- M0 toolchain ✅ (see `docs/m0_toolchain.md`)
- M1 feasibility ✅ (scene changed: 0.35 m pedestals, 0.4 m flight — docs/m1_feasibility.md)
- M2 scene/bringup ✅ (docs/m2_scene_bringup.md)
- M3 perception + pick/place ✅ (docs/m3_perception_pick_place.md)
- M4 throw ✅ (docs/m4_throw.md)
- M5 catch ✅ (docs/m5_catch.md)
- M6 full loop + demo ✅ (docs/m6_full_loop.md, README.md)
- v1.0.1 ✅ robot descriptions pinned in src/external (submodules), m5 clips with the bracket
- v2 Phase 1 ✅ offline feasibility (docs/v2_phase1_feasibility.md, tools/v2_*.py, docs/v2/)
- v2 Phase 2 ✅ random throws, vision-only reactive pinch, live benchmark, demo (docs/v2_phase2.md, docs/v2/phase2/; tag v2.0)
- v2.1 ✅ review fixes: benchmark re-run with every cell alone, v1 30/30 contradiction resolved, fast_close vs offline bounded
- v2.2 ✅ gripper delay probe (100 % at 0-30 ms true latency), trajectory sample-time fix, MoveIt-invalid library entries skipped

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
- Camera plugin: ONE rate for all streaming cameras, checked on its own ticks and restarted at "now": the setting is
  not the rate. Measured: 150 -> 128.7 Hz, 70 -> 63.9 Hz (125 -> 115, 130-140 -> 116). Measure with tools/v2_live/rate_probe.py
  (stamps). Non-streaming cameras are `polled` (std_srvs/Trigger at /cameras/<name>/trigger); depth is always
  rendered and published too (nobody subscribes -> not transported). Never use `policy: disabled`: buffers skip
  disabled cameras but triggers index by MJCF camera order, so triggering a polled camera listed after a disabled
  one aborts the whole sim (out_of_range in CameraPlugin::handle_trigger); make unused cameras `polled` instead.
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

- Build: `source /opt/ros/jazzy/setup.bash && colcon build --allow-overriding robotiq_description ur_description && source install/setup.bash`
  (the two descriptions are pinned submodules in src/external that overlay /opt/ros; `git submodule update --init`)
- Everything (GUI): `ros2 launch catch_bringup bringup.launch.py` (args: `headless:=true rviz:=false`,
  v2: `camera_rate:=150` (default, 128.7 Hz), `tracker:=true` (default), `plan_catcher:=false`, `mujoco_model:=<abs path>`)
- Scripted runs: `tools/sim_ctl.sh start catch_bringup bringup.launch.py headless:=true rviz:=false`,
  `tools/sim_ctl.sh wait 5 90`, ..., `tools/sim_ctl.sh stop` (log: `log/sim_latest.log`)
- M2 checks: `ros2 run catch_tasks jog_test`, `ros2 run catch_tasks grasp_test --robot a`
- M3 checks: `ros2 run catch_perception eval_detection`, `ros2 run catch_tasks pickplace_test`
- M4: `ros2 run catch_tasks throw_eval --throws 20 [--from-pallet] [--out docs/m4/throws_20.json]`; plots `tools/plot_throws.py`
- M5: `ros2 run catch_tasks catch_eval --throws 30 [--trigger time] [--no-place] [--no-correction] [--cup-offset 0.015]
[--out docs/m5/x.json] [--rec-dir log/m5/x]`; plots `tools/plot_catch.py`; offline `tools/catch_study.py [--sweep|--envelope]`
- M6: `ros2 run catch_tasks run_demo --v1 --seed 1 [--fault-throw N] [--out docs/m6/run.json]`; recorded batch
  `tools/run_m6.sh 1 2 3` (rosbag2 + `ros2 run catch_tasks bag_to_npz`); v2: `tools/v2_live/run_demo_v2.sh 1 2 3`
- Clips / video (venv): `tools/render_replay.py rec.npz --out x.mp4 [--view catch|throw|overview]`,
  `tools/render_demo.py rec.npz run.json --out docs/media/demo.mp4`
- After changing throw.yaml (except `release:`), scene.yaml or physics.yaml: `env -u PYTHONPATH .venv/bin/python tools/throw_feedforward.py`
- Venv integration tests (ROS sourced): `env -u PYTHONPATH .venv/bin/python -m pytest src/catch_description/test/test_fk_agreement.py src/catch_description/test/test_mjcf_up_to_date.py`
- After changing the URDF/SRDF-relevant geometry: `env -u PYTHONPATH .venv/bin/python tools/gen_srdf.py`

- v2 live: `ros2 run catch_tasks tracker_eval --throws 50 --seed 1 [--calibrate]` (tracker vs ground truth),
  `ros2 run catch_tasks reactive_eval --mode pinch|cup --source vision|plan --noise none|low|medium|high --throws 60`,
  benchmark `tools/v2_live/bench_seq.sh` (v2.1: every cell ALONE, sequentially, resumable; reactive_eval records RTF
  + load average per trial; nothing else heavy may run — parallel load cost up to 24 points in v2.0, whose 3-domain
  runner is `tools/v2_live/bench.sh`; one cell: `tools/v2_live/bench_cell.sh`, OUT_DIR=...), offline counterpart of
  every cell on the same throws + noise draws `tools/v2_live/offline_cells.py` (venv, < 1 min per cell),
  fast_close / same-throw diagnostics `tools/v2_live/fastclose_diag.py`, v1 close analysis
  `tools/v2_live/v1_close_analysis.py`, table/plots `tools/v2_live/plot_bench.py` (venv),
  re-label stored cells `tools/v2_live/reclassify.py`, close timing `tools/v2_live/close_timing.py`, clips
  `tools/v2_live/render_failures.py` (venv), demo GT check `tools/analyze_runs.py run.json:rec.npz`,
  `ros2 run catch_tasks run_demo --seed 1` (v2: random throws, vision-only pinch; `--v1` for the v1 loop).
- v2 offline (venv): `tools/v2_perception.py`, `tools/v2_ready_envelope.py`, `tools/v2_throws.py [--square|--id]`,
  `tools/v2_catch_sim.py --cup|--pinch|--corrections|--live-match [--throws square]` (resumable), `tools/v2_plots.py`.
- After changing scene.yaml / physics.yaml / throw.yaml: `tools/throw_feedforward.py` AND `tools/build_throw_library.py`
  (both hash those files; throw_node refuses stale tables / library entries). Robot model exports for the ROS nodes:
  `tools/export_dynamics.py` (numpy RNEA parameters, test fixture), `tools/export_collision.py` (capsule model,
  validated against MuJoCo). MJCF variants with another gripper actuator: `tools/v2_live/mjcf_variant.py PROFILE`.
- Gripper model (catch_sim/gripper_model.py, config/gripper.yaml): every gripper command is a rate-limited (150 mm/s)
  ramp starting one command latency (10 ms) later, sent to the gripper JTC; `CATCH_GRIPPER_PROFILE` /
  `CATCH_GRIPPER_LATENCY` env vars select variants (benchmark). Both grippers are JointTrajectoryControllers now.

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
- Both grippers are JointTrajectoryControllers (position, interpolate_from_desired_state); command them only through
  MoveItClient.gripper()/gripper_async() or catch_sim.gripper_model (2F-85 speed + latency). Immediate commands use a
  ZERO header stamp (a short ramp stamped "now" can arrive after its end and is rejected); timed commands stamp the
  sim time. Never send GripperCommand goals (no GripperActionController any more).
- /joint_states: use catch_sim.joint_state_cache.JointStateCache (EventsExecutor helper thread), never a subscription
  on a MultiThreadedExecutor node (cost 50-95 % of a core). MoveItClient shares one cache per node.
- move_group must not publish /robot_description (it would race robot_state_publisher's, e.g. with an MJCF variant).
- catch_tracker: gate stereo pairs in 3D (rays meet within 10 mm inside the flight box); a 2D region of interest lets
  objects far below the flight volume (pallet slot borders) into the top view. Bias calibrated with tracker_eval.
- Reactive catcher in vision mode must never subscribe /throw_plan or call /throw/preview (test_reactive_source.py).
- JointTrajectory points must be strictly increasing in time: sample plans with catch_catcher.reactive.sample_times
  (np.arange(0, T, dt) + [T] duplicated T when T was a hair above a multiple of dt; the JTC rejected 3-4 % of B's plans).
- Throw library: after changing scene/URDF/SRDF geometry or rebuilding the library, re-run
  `python3 tools/v2_live/check_library_moveit.py` (bringup running) — MoveIt's URDF meshes reject some stroke starts that
  MuJoCo accepts; Library.draw skips entries flagged moveit_start_valid=false.
- The reactive catcher probes its gripper delay (reactive.yaml delay_probe) before its first catch; benchmark cells
  that vary the gripper latency rely on it.
- Parallel sims: one ROS_DOMAIN_ID each, stop them by process group (tools/sim_ctl.sh stop kills ALL sims).
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
