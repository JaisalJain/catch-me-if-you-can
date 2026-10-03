# v2 Phase 2 — random throws, vision-only B, reactive catch from the ready pose

A throws random entries of a precomputed throw library with release noise; B starts from the Phase 1 ready pose,
sees the cube only through two cameras (C++ tracker), commits to an intercept as soon as the prediction is good
enough, keeps refining it by replacing its trajectory at ~40 Hz, and catches with the "pinch" (fingers pre-opened
to 56 mm, close timed on the predicted arrival) — on the live ROS pipeline, with the 2F-85's real closing speed and
command latency modelled. Phase 1 (offline study): `docs/v2_phase1_feasibility.md`.

Defaults everywhere: `gripper_profile: realistic` (2F-85 150 mm/s gap speed, 10 ms command latency). `fast_close`
and `fast_idealized` (NOT a real 2F-85) appear only in the benchmark and are labelled there.

**Key numbers** (v2.1: live ROS pipeline, every benchmark cell run alone, 60 throws per cell, ground truth):
* Default — camera-only pinch, medium release noise, realistic 2F-85: **59/59 = 100 % [94–100]**; offline on the same throws 98 % [91–100].
* Camera-only barely notices the release noise (100 / 100 / 100 / 95 % for none / low / medium / high, although
  the noise moves the arrival by up to 15 cm); plan-informed B (A's published plan) collapses: 100 → 100 → 39 → 22 %.
* Perception: **5 ms** render → estimate (median), 128.7 Hz, first estimate 47 ms after release, 9.7 mm (95 %)
  intercept error 0.20 s ahead. B commits ~75 ms after the first estimate and re-plans at 39 Hz (4 ms per plan).
* The gripper is the limit: a 2F-85 that reacts 10 / 20 ms later than B assumes costs ~30 / ~35 points (71 / 64 %).
* Full loop, 3 seeds, medium noise: **9/9 caught first time, 9/9 placed (≤ 1.1 mm), VERIFY = ground truth 9/9**.
* v2.0's parallel benchmark (three sims at once) lost up to 24 points to CPU load: "Under load" in step 6.

<!-- STEPS -->

## Step 0 — corrections from the Phase 1 review
Details: `docs/v2_phase1_feasibility.md` ("Corrections after the Phase 1 review"), `docs/v2/catch_corrections.png`.
* **Pinch numbers reconciled.** 86–87 % = realistic pinch closing 10 ms after the nominal instant; the plots showed
  nominal timing (76 / 71 %), 0.25 velocity matching and a 5×-force actuator sweep (55–71 % at 150 mm/s). Every panel
  now names its configuration; panel (f) shows the headline configurations.
* **Speed vs grip force.** `fast_close` = 10× speed with the realistic force: 100 % (pinch, nominal close) — the gain
  is closing speed, not force. Realistic-force speed sweep: 75 / 150 / 300 / 600 mm/s → 43 / 81 / 96 / 100 %.
* **Gripper command latency** 10 ms (Robotiq 2F-85/2F-140 manual 4.7.1: "200 Hz is the usual speed when commanding /
  reading … minimum delay of 5 ms between [commands]", RS-485 115.2 kbps → ≤ 5 ms slot + ~2 ms transaction + gripper
  cycle; an estimate, not a measurement), modelled offline and in the sim (`catch_sim/gripper_model.py`). Sweep with
  B assuming 10 ms: pinch 76 / 87 / 88 / 82 %, pre-closed cup 90 / 89 / 88 / 86 % at 0 / 10 / 20 / 30 ms.
* **v1 honesty.** The v1 sim gripper closed ~3× faster than a 2F-85. With the realistic model on both arms the v1
  evaluation still catches **30/30 (placed 30/30)** and M4 throws **20/20, +5.4 ± 1.8 mm/s** — the v1 demo throw lands
  square on a static, plan-informed cup, where the closing speed does not matter (it does for random throws, Phase 1).
  Measured in both runs' recordings (v2.1, `tools/v2_live/v1_close_analysis.py`): 480 vs 150 mm/s reached, pads on
  the cube 108 vs 340 ms after the impact, the cube at rest in the palm (≤ 6 mm drift) until they arrive.

## Step 1 — /joint_states CPU fix
`catch_sim.joint_state_cache.JointStateCache` keeps the latest /joint_states on a helper node spun by an
EventsExecutor thread (the /clock pattern), used by MoveItClient (catcher, run_demo, evals) and throw_node.

| idle, % of one core | before | after |
|---|---|---|
| throw_node | 93.2 | **9.8** |
| catcher | 53.2 | **9.8** |
| run_demo's node | 46.6 | **11.8** |

Regression with the v1 gripper (MJCF variant + profile `v1`, so only the CPU fix differs): M4 20/20, +7.1 ± 2.2 mm/s,
0.28 ± 0.03° (v1: +6.6 ± 2.5, 0.28 ± 0.03); run_demo seed 1 complete R-G-B in 3 throws, verify offsets identical
to v1 to 0.1 mm, 95.7 → 88.3 s.

## Step 2 — random throws
* **Library** (`tools/build_throw_library.py` → `catch_throw/config/library_{pinch,cup}.json`): pinch = all 241
  A-feasible throws B can catch with the natural pinch; cup = the 121 square-arrival throws. Each entry: generation
  parameters, its own MuJoCo feed-forward table, a hash of everything it depends on (throw_node refuses stale entries).
  A picks an entry at random (seeded) via `/throw/preview`; the plan message carries library, entry and stroke-start
  joints so the caller can move A there.
* **Release noise** (`catch_throw/config/v2.yaml`), Gaussian per throw on speed, direction and release time, applied to
  the executed throw; the published plan stays nominal:

| level | speed (1σ) | direction (1σ per axis) | release time (1σ) | basis |
|---|---|---|---|---|
| none | 0 | 0 | 0 | |
| low | 0.12 % | 0.03° | 1 ms | M4's measured scatter |
| medium | 0.6 % | 0.36° | 5 ms | half the plan-informed tolerance across the path (±25 mm/s) |
| high | 1.2 % | 0.72° | 10 ms | that tolerance |

  The realised release error (ground-truth ballistic fit vs the published plan) is recorded in every benchmark trial.
* **Vision-only is enforced.** The reactive catcher's source is fixed per node: with `source=vision` none of its nodes
  subscribes `/throw_plan` or has a `/throw/preview` client (`test_reactive_source.py`); the plan-informed baseline is
  a separate node (`source=plan`, `/reactive_catch_plan`), launched only for the benchmark.

## Step 3 — live vision tracking
* **Cameras:** `track_side` + `track_top` (320×240, colour, orthogonal views), streaming; overheads, side and wrist
  cameras on-demand (polled: pallet_detector triggers at 4 Hz, color_check per request). The camera plugin checks its
  single rate on its own ticks and restarts the timer at "now": a setting of 150 gives **128.7 Hz** (125 → 115 Hz).
* **Detection outside Python:** `catch_tracker` (C++ composable node): HSV components per view; every cross-view pair
  is triangulated and kept only if the two rays meet within 10 mm inside the 3D flight box (the first version used
  a 2D region of interest and picked up pallet slot borders far below the flight volume in the top view: −75 mm
  errors); bias calibrated on 20 separate throws (0.9, 0.4, −0.4 mm); ballistic least squares with known g and
  covariance; predicted intercept (closest point to B's ready TCP) with covariance. The images come from the MuJoCo
  process (camera plugin), so they always cross a process boundary (FastDDS shared memory); intra-process transport is
  not possible without moving detection into the plugin — not needed at the measured latency.
* **50 throws vs ground truth** (`ros2 run catch_tasks tracker_eval`, `docs/v2/phase2/tracker_eval_50.json`):

| | live | Phase 1 offline |
|---|---|---|
| end-to-end latency render stamp → estimate published | **5 ms median, 6 p95, 10 max** | 25 ms assumed |
| — of which render + transport / processing | 5 / 1 ms | |
| camera rate | 128.7 Hz | 120 Hz |
| per-fix error (bias-calibrated) | (0.0, −0.4, 0.0) ± (1.0, 1.0, 2.0) mm | 0.7/0.8/1.8 mm + 1.9 mm z bias (tumbling cube) |
| first fix after release | 47 ms | 30 ms assumed |
| 95 % intercept error at 0.10 / 0.15 / 0.20 / 0.25 / 0.30 s | 47 / 19 / **9.7** / 6.3 / 4.8 mm | – / 26 / 18 / 13 / 11 mm |

![tracker](v2/phase2/tracker_live_vs_offline.png)

The live curve sits on the Phase 1 white-noise model (2 mm, 120 Hz, 25 ms): the lower latency compensates the later
first fix; the Phase 1 rendered-detection estimate (a cube tumbling about all three axes) was pessimistic.

## Step 4 — reactive intercept from the ready pose (pinch)
`catch_catcher/reactive_node.py` (+ `reactive.py`, numpy only), action `/reactive_catch`:
* B waits at the Phase 1 ready pose (TCP (0.30, 0.25, 1.10), approach 60° up towards A) with the fingers at 56 mm.
* **Commit** when the tracker's 95 % intercept error is ≤ 30 mm (~74 ms after the first fix, ~120 ms after release):
  search the predicted descending path for the intercept with the most limit margin — IK + batched inverse dynamics for
  every candidate, collision checks on the feasible ones best first (~20 ms).
* **Refine**: on each new estimate, at most every 20 ms, re-plan the same intercept time from the current desired state
  (quintic, ~4 ms) and replace `arm_b_controller`'s trajectory (JointTrajectory topic, points every 10 ms with
  inverse-dynamics feed-forward; `interpolate_from_desired_state` for continuity) — **~39 Hz** in practice, until
  40 ms before the intercept. A replan that fails late keeps the current plan; B aborts (back to the ready pose) only
  when the predicted intercept leaves the catch envelope.
* **Limits** on every plan: joint speed ≤ 85 %, torque ≤ 80 % (numpy RNEA on the MuJoCo-derived parameters,
  `catch_kinematics.dynamics`, equal to mj_inverse to 1e-6 N·m, test), joint margins.
* **Collisions** on every plan, every 15 ms of the trajectory: capsule model of B (MuJoCo's UR5e capsule geoms
  one-to-one + two fitted gripper capsules) against itself, table / pedestals / pallets and arm A (from A's measured
  joints), `catch_kinematics.collision`. Validated against MuJoCo's collision detection on 4000 random configurations:
  **0 missed collisions**, conservative (21 % of free configurations flagged), 0.2 ms per configuration — fast
  enough, no precomputed safe region needed.
* **Close**: the timed close is scheduled in the gripper controller in advance (sim-time stamp, 2F-85 model ramp; gap
  reaches 40 mm on the predicted arrival + 10 ms, B compensates its assumed 10 ms latency); a palm contact before that
  instant closes immediately. With a 150 mm/s gripper the timed close has to start ~110 ms before the arrival, so in
  the pinch it fires first (the contact trigger only guards an early arrival); in the cup the contact fires first.
  The result says which one fired.
* Smoke test (12 throws, no noise): **12/12**, commit 74 ms after the first fix, 39 Hz replans of 4.4 ms, limit ratio
  ≤ 0.94, plan clearance ≥ 12 mm.

## Step 5 — cup baseline
Same framework, `mode: cup`: palm against the predicted velocity within a 10° cone around the ready axis, fingers
pre-closed to 56 mm, B at rest 30 ms before the arrival, contact close (timed fallback 30 ms after the predicted
arrival), square-arrival throws (cup library). Smoke test (8 throws, no noise): **8/8, contact trigger every time**.

## Step 6 — live benchmark (v2.1: every cell alone)
**Method.** `tools/v2_live/bench_seq.sh`: all 15 cells **sequentially, one sim at a time, nothing else heavy running**
(`tools/v2_live/bench_cell.sh` per cell, ROS domain 41), `ros2 run catch_tasks reactive_eval`, 60 throws per cell, the
same seeded library entries and release-noise draws in every pinch cell, judged on ground truth (held = cube at rest
in the hand < 25 mm from the pad centre, as offline). Recorded per trial: the real-time factor (sim/wall over the whole
trial, and the minimum 2 s window of rtf_monitor) and the 1 min load average. Over all cells: **RTF ≥ 0.9979
per trial, ≥ 0.9956 in any 2 s window; load average median 2.4–5.5, max 33.4 on
24 cores; camera-only re-planning 35.7–39.3 Hz (median per cell)**. One sim alone loads the machine
to ~3–7; the few trials above that (latency 20 ms trials 21–43: up to 26, latency 30 ms trials 56–59: up to 33,
fast_close trials 0–3: up to 16, at 10:51–10:57 and 11:14–11:15) coincide with no job of the benchmark or of
mine — load from elsewhere on the desktop machine; RTF and re-planning stayed at ≥ 0.997 / 38–39 Hz in those
trials and their failure rate is the cell's (latency 20 ms: 6 of 16 vs 17 of 59), so the cells were kept. The first
default run overlapped light single-core offline diagnostics; it was re-run strictly alone (59/59 both times, load
max 7.7 vs 7.6). Offline counterparts on the
**same throws** (same library entries + the same release-noise draws, measured live perception):
`tools/v2_live/offline_cells.py`. Table + plots: `env -u PYTHONPATH .venv/bin/python tools/v2_live/plot_bench.py`.

![benchmark](v2/phase2/bench.png)

| cell | n | caught | rate [95 % CI] | offline, same throws | outcomes | trigger | commit after 1st fix | re-plan rate median (p10–p90) | RTF trial median / min 2 s window | load avg 1 min median / max (24 cores) | realised release error (speed sd, direction rms, time sd) |
|---|---|---|---|---|---|---|---|---|---|---|---|
| vision-only, pinch, noise none | 59 | 59 | **100 %** [94–100] | 100 % [94–100] | held 59, setup 1 | timed 59 | 74 ms | 39 Hz (39–39) | 1.000 / 1.000 | 3.8 / 7.0 | 0.29 %, 0.28°, 0.0 ms |
| vision-only, pinch, noise low | 59 | 59 | **100 %** [94–100] | 100 % [94–100] | held 59, setup 1 | timed 59 | 74 ms | 39 Hz (39–39) | 1.000 / 0.999 | 3.2 / 7.2 | 0.32 %, 0.28°, 1.0 ms |
| **vision-only, pinch, noise medium (default)** | 59 | 59 | **100 %** [94–100] | 98 % [91–100] | held 59, setup 1 | timed 59 | 74 ms | 39 Hz (39–39) | 1.000 / 1.000 | 2.8 / 7.6 | 1.15 %, 0.68°, 4.9 ms |
| vision-only, pinch, noise high | 59 | 56 | **95 %** [86–98] | 100 % [94–100] | held 56, abort 2, missed intercept 1, setup 1 | timed 57, none 2 | 74 ms | 39 Hz (39–39) | 1.000 / 0.999 | 3.2 / 7.3 | 2.02 %, 1.23°, 9.9 ms |
| plan-informed, pinch, noise none | 59 | 59 | **100 %** [94–100] | — | held 59, setup 1 | timed 59 | before the throw (plan) | — | 1.000 / 1.000 | 3.1 / 8.1 | 0.26 %, 0.26°, 0.0 ms |
| plan-informed, pinch, noise low | 59 | 59 | **100 %** [94–100] | — | held 59, setup 1 | timed 59 | before the throw (plan) | — | 1.000 / 1.000 | 2.9 / 5.7 | 0.32 %, 0.29°, 1.0 ms |
| plan-informed, pinch, noise medium | 59 | 23 | **39 %** [28–52] | — | missed intercept 25, held 23, fingertip 10, late close 1, setup 1 | timed 59 | before the throw (plan) | — | 1.000 / 1.000 | 2.4 / 8.3 | 1.20 %, 0.69°, 4.9 ms |
| plan-informed, pinch, noise high | 58 | 13 | **22 %** [14–35] | — | missed intercept 38, held 13, fingertip 6, setup 2, late close 1 | timed 58 | before the throw (plan) | — | 1.000 / 1.000 | 4.1 / 8.8 | 2.04 %, 1.23°, 9.6 ms |
| vision-only, pre-closed cup (square throws), medium | 58 | 52 | **90 %** [79–95] | 87 % [76–93] | held 52, late close 3, missed intercept 2, setup 2, fingertip 1 | contact 56, timed 2 | 74 ms | 36 Hz (32–36) | 1.000 / 1.000 | 3.7 / 7.6 | 1.21 %, 0.67°, 4.9 ms |
| vision-only, pinch, 64 Hz cameras, medium | 59 | 57 | **97 %** [88–99] | 97 % [89–99] | held 57, setup 1, abort 1, fingertip 1 | timed 58, none 1 | 85 ms | 39 Hz (39–39) | 1.000 / 1.000 | 2.9 / 5.0 | 1.09 %, 0.69°, 4.9 ms |
| vision-only, pinch, gripper latency 0 ms (B assumes 10), medium | 59 | 57 | **97 %** [88–99] | 100 % [94–100] | held 57, fingertip 2, setup 1 | timed 59 | 73 ms | 39 Hz (39–39) | 1.000 / 0.999 | 2.9 / 6.4 | 1.13 %, 0.69°, 4.9 ms |
| ... latency 20 ms | 59 | 42 | **71 %** [59–81] | 67 % [54–77] | held 42, fingertip 10, late close 7, setup 1 | timed 59 | 73 ms | 39 Hz (39–39) | 1.000 / 0.998 | 5.5 / 25.7 | 1.27 %, 0.69°, 4.9 ms |
| ... latency 30 ms | 59 | 38 | **64 %** [52–75] | 50 % [38–62] | held 38, late close 13, fingertip 8, setup 1 | timed 59 | 74 ms | 39 Hz (38–39) | 1.000 / 0.997 | 4.9 / 33.4 | 1.33 %, 1.46°, 4.9 ms |
| B's gripper fast_close (NOT a 2F-85; 10x speed, realistic force), medium  | 59 | 52 | **88 %** [77–94] | 80 % [68–88] | held 52, fingertip 7, setup 1 | timed 59 | 74 ms | 39 Hz (39–39) | 1.000 / 0.998 | 4.0 / 15.6 | 1.13 %, 0.69°, 4.9 ms |
| B's gripper fast_idealized (NOT a 2F-85; 10x speed, 5x force), medium | 59 | 59 | **100 %** [94–100] | 100 % [94–100] | held 59, setup 1 | timed 59 | 74 ms | 39 Hz (39–39) | 1.000 / 0.996 | 5.5 / 8.7 | 1.15 %, 0.70°, 4.9 ms |

"setup" = A could not start the throw (library entry 79: MoveIt fails to reach its stroke start from the load
pose, in every cell; excluded, not a catch attempt).

### Does the camera-only catch rate drop as the noise rises?
* vision: caught 59/59, 59/59, 59/59, 56/59 (none → high); Cochran–Armitage trend z = -2.34, p = 0.0194; slope -1.5 points per level
* plan: caught 59/59, 59/59, 23/59, 13/58 (none → high); Cochran–Armitage trend z = -10.58, p = 0; slope -29.4 points per level

| noise level | arrival shift median [mm] | p95 | max |
|---|---|---|---|
| none | 4 | 7 | 8 |
| low | 7 | 15 | 18 |
| medium | 26 | 61 | 78 |
| high | 46 | 118 | 151 |

Barely, and only at the highest level: camera-only 59/59, 59/59, 59/59,
56/59 (none → high; the trend test is significant at p ≈ 0.02 only because of the high cell's 3
failures, −1.5 points per level), plan-informed 59/59, 59/59, 23/59,
13/58 (−29 points per level). Evidence for why:
* **The noise is real and large.** The realised release error grows as configured (table: speed sd, direction rms,
  timing sd per level), and it moves the cube's arrival by a median 4 / 7 / 26 / 46 mm, up to 151 mm (table above).
* **Camera-only does not use the release.** B estimates the flight from the cameras after the release and plans to
  where the cube *will be*; the arrival shift only moves the intercept inside B's envelope. Offline, on the same
  throws and noise draws, the result is just as flat: 100 % [94–100], 100 % [94–100], 98 % [91–100],
  100 % [94–100].
* **What high noise does cost:** the 3 high-noise failures are 2 aborts (the moved intercept needs > 85 % joint speed
  / 80 % torque everywhere: "over limits") and 1 missed intercept — throws pushed to the edge of the envelope, not
  tracking errors.
* **Plan-informed B** commits to A's *published* (nominal) plan before the throw: from medium noise on, the arrival
  shift exceeds the pinch's tolerance (fingers at 56 mm around a 40 mm cube: ±8 mm) and it misses.

### Live vs offline (same throws)
* **Default (camera-only pinch, medium noise, realistic 2F-85): 100 % [94–100] live, 98 % [91–100] offline.**
* Pre-closed cup 90 % [79–95] (offline 87 % [76–93]): the contact-triggered close of a 150 mm/s gripper
  fires on the arrival but needs ~117 ms (10 ms latency + 16 mm) — the cube sometimes rebounds out first (late close).
* 64 Hz cameras 97 % [88–99] (offline 97 % [89–99]): half the camera rate commits later but costs little.
* Gripper latency (true latency varied, B assumes 10 ms): 97 % [88–99] / 100 % [94–100] /
  71 % [59–81] / 64 % [52–75] live for 0 / 10 / 20 / 30 ms; offline 100 % [94–100] /
  98 % [91–100] / 67 % [54–77] / 50 % [38–62]. A close that comes later than B assumes
  loses the cube (late close, rebound → fingertip); an earlier one is tolerated with good aim.
* B's gripper made faster (NOT a real 2F-85): fast_close 88 % [77–94] (offline 80 % [68–88]),
  fast_idealized 100 % [94–100] (offline 100 % [94–100]). The realistic gripper's timed pinch
  close already hides the 2F-85's slowness (it starts ~110 ms early).
* **Timing check.** In the recorded failures the gap reaches 40 mm ~10 ms later than B schedules (latency 0 / 10 / 20
  / 30 → +10 / +19 / +30 / +39 ms after the arrival, v2.0 recordings, `tools/v2_live/close_timing.py`): the MuJoCo
  finger actuator lags its reference ramp by kv/kp = 10 ms, offline and live alike.

### Under load (v2.0: three sims in parallel)
The v2.0 benchmark ran three cells at a time (three ROS domains on 24 cores); `docs/v2/phase2/bench_v2.0/`. Load
slowed B's planner (re-plans 11–12 ms at ~29 Hz instead of 4 ms at 39 Hz; commit ~100 ms instead of 73 ms after the
first fix) and cost up to 24 points (latency 0 ms: 76 % under load, 97–100 % alone — with the fingers closing 10 ms
earlier than B assumes, the few millimetres of extra aim error left by the slower re-planning are enough for the
fingertips to catch the cube). Cells without re-planning (plan-informed) are unaffected (plan medium 23/59 in both).
Under load the camera-only rates also scattered without a noise trend (93 / 86 / 95 / 90 %), which v2.0 reported as
"insensitive to noise"; alone they are 100 / 100 / 100 / 95 %.

| cell | v2.0 under load (3 sims in parallel) | re-plan rate | v2.1 alone | re-plan rate |
|---|---|---|---|---|
| vision-only, pinch, noise none | 93 % [84–97] (55/59) | 29.5 Hz | 100 % [94–100] (59/59) | 39.3 Hz |
| vision-only, pinch, noise low | 86 % [75–93] (51/59) | 29.5 Hz | 100 % [94–100] (59/59) | 39.3 Hz |
| **vision-only, pinch, noise medium (default)** | 95 % [86–98] (56/59) | 29.3 Hz | 100 % [94–100] (59/59) | 39.3 Hz |
| vision-only, pinch, noise high | 90 % [80–95] (53/59) | 31.4 Hz | 95 % [86–98] (56/59) | 39.3 Hz |
| plan-informed, pinch, noise none | 98 % [91–100] (57/58) | 0.0 Hz | 100 % [94–100] (59/59) | 0.0 Hz |
| plan-informed, pinch, noise low | 98 % [91–100] (58/59) | 0.0 Hz | 100 % [94–100] (59/59) | 0.0 Hz |
| plan-informed, pinch, noise medium | 39 % [28–52] (23/59) | 0.0 Hz | 39 % [28–52] (23/59) | 0.0 Hz |
| plan-informed, pinch, noise high | 27 % [17–40] (16/59) | 0.0 Hz | 22 % [14–35] (13/58) | 0.0 Hz |
| vision-only, pre-closed cup (square throws), medium | 79 % [67–88] (45/57) | 27.3 Hz | 90 % [79–95] (52/58) | 35.7 Hz |
| vision-only, pinch, 64 Hz cameras, medium | 95 % [86–98] (56/59) | 31.4 Hz | 97 % [88–99] (57/59) | 39.2 Hz |
| vision-only, pinch, gripper latency 0 ms (B assumes 10), medium | 76 % [63–85] (44/58) | 28.6 Hz | 97 % [88–99] (57/59) | 39.3 Hz |
| ... latency 20 ms | 79 % [67–88] (46/58) | 29.5 Hz | 71 % [59–81] (42/59) | 39.3 Hz |
| ... latency 30 ms | 58 % [45–69] (34/59) | 29.5 Hz | 64 % [52–75] (38/59) | 39.3 Hz |
| B's gripper fast_close (NOT a 2F-85; 10x speed, realistic force), medium  | 90 % [80–95] (53/59) | 39.3 Hz | 88 % [77–94] (52/59) | 39.3 Hz |
| B's gripper fast_idealized (NOT a 2F-85; 10x speed, 5x force), medium | 97 % [88–99] (57/59) | 29.0 Hz | 100 % [94–100] (59/59) | 39.3 Hz |

Earlier versions of three v2.0 cells were wrong and were re-run then: high-noise throws whose perturbed stroke
started > 0.05 rad from the nominal start were refused by throw_node (now blended in, ≤ 6 rad/s², A's refusals are
"throw failed", excluded); the fast_close / fast_idealized cells had also given A's gripper the fast actuator — now
only B's gripper varies (`mjcf_variant.py --arms b`, `CATCH_GRIPPER_ARMS=b`).

### Failure taxonomy (ground truth, `reactive_eval.classify`)
| class | meaning | where it happens |
|---|---|---|
| fingertip | gripped, 25–50 mm out of the hand | early close (latency 0 under load), late close after a rebound (latency 20/30) |
| late close | came within 30 mm, fingers closed on nothing | true latency > assumed (20/30 ms), contact-triggered cup |
| slide-off | gripped, then lost | rare (latency 20/30) |
| missed intercept | never within 30 mm of the TCP | plan-informed with noise (the realised flight differs from the plan) |
| abort | no feasible intercept in the envelope | rare (≤ 2 per cell): throws whose intercept lies at the envelope edge |
| collision | B's capsule clearance < 0 on the recorded motion | **never** (planner checks every plan) |

Clips (0.25×, catch view; `tools/v2_live/render_failures.py`): see `docs/v2/phase2/failure_clips.json`.

### Latency budget (vision-only pinch, sim alone, medians)
| | time |
|---|---|
| release → first estimate (cube in both views, 5 consistent fixes) | 47 ms |
| render stamp → estimate published (included above) | 5 ms |
| first estimate → commit (95 % intercept error ≤ 30 mm) | 73 ms (≈ 120 ms after release) |
| commit → first trajectory to arm_b_controller (search + commit lead) | ≈ 20 + 30 ms |
| re-plans until 40 ms before the arrival | 8 (6–10) × 4.3 ms, 39 Hz |
| timed close command → gap 40 mm | 108.7 ms (10 ms latency + 2 ms + 16 mm at 150 mm/s) + ~10 ms actuator lag |
| arrival after release | 0.29–0.49 s (median 0.41 s) |
So the close is commanded ~0.30 s after release, ~180 ms after the commit — the 2F-85 closing time (~120 ms) is the
largest single item; perception is 5 ms.

## Step 7 — full loop with random throws, vision-only pinch
`ros2 run catch_tasks run_demo --seed N` is v2 by default (`--v1` = the v1 loop): DETECT → PICK → LOAD (A picks a
random library entry, seeded, and moves to its stroke start) → PREPOSITION (B to the ready pose, fingers at 56 mm,
watching the cameras only) → THROW (medium release noise) + reactive pinch catch → COLOR (wrist camera) → IN_HAND →
PLACE → VERIFY. Recorded batch: `tools/v2_live/run_demo_v2.sh 1 2 3` (fresh sim per seed, rosbag2 →
`bag_to_npz`); ground-truth check `tools/analyze_runs.py` → `docs/v2/phase2/demo/verify_vs_ground_truth.jsonl`.

| seed | order (random) | throws | caught first time | recoveries | duration (sim) | VERIFY vs ground truth | final cube offset in slot (ground truth) |
|---|---|---|---|---|---|---|---|
| 1 | red, green, blue | 3 | 3/3 | 0 | 86.2 s | 3/3 | ≤ 0.7 mm |
| 2 | green, blue, red | 3 | 3/3 | 0 | 91.9 s | 3/3 | ≤ 1.1 mm |
| 3 | blue, red, green | 3 | 3/3 | 0 | 85.8 s | 3/3 | ≤ 0.8 mm |

All 9 catches: timed pinch close, commit 64–78 ms after the first estimate, 8–12 re-plans at 39 Hz of 3.6–4.6 ms;
B's wrist camera named the right color 9/9; RTF 1.000. Cubes held up to 21° tilted in the hand were placed square
(the in-hand correction). VERIFY's own offsets (−4.5…+1.4 mm) are overhead-detection error; the ground-truth
placement error is ≤ 1.1 mm.

Two bugs surfaced only here (the benchmark never triggers the wrist camera): mujoco_ros2_control 0.1.1 aborted the
whole sim on the first wrist-camera trigger (a polled camera listed after a `disabled` one — the plugin's buffers
skip disabled cameras but triggers index by MJCF order; the demo camera is now polled), and color_check rejected the
triggered frame because the plugin stamps it with the rendered physics time, a few ms before the request (freshness
is now "arrived after the trigger").

Demo video: [`media/demo_v2_seed1.mp4`](media/demo_v2_seed1.mp4) (seed 1: B starts from the ready pose and moves to
each cube; real time around each throw, 0.25× close-up of each catch). Failure clips from the benchmark:
`media/v2_fail_*.mp4` (`v2/phase2/failure_clips.json`).

## v2.1 review fixes

### The v1 30/30 contradiction (resolved)
Phase 1 wrote that "the M5 cup's 30/30 was obtained with the v1 sim gripper, i.e. faster than the real hardware can
close", and the README added that the headline "was measured with the faster v1 sim gripper" — implying the result
depended on it. Step 0 then re-ran the v1 evaluation with the realistic model and got 30/30 again. The recordings of
both runs (`python3 tools/v2_live/v1_close_analysis.py log/m5/force log/p2/v1_realistic`) settle which claim is
wrong:

| | M5 (v1 sim gripper) | step 0 re-run (realistic) |
|---|---|---|
| gripper | MJCF actuator kp 100 / kv 10 / ±5 N, step command (GripperActionController) | kp 1000 / kv 10 / ±5 N (default MJCF), 2F-85 command model: 10 ms latency + 150 mm/s ramp on a JTC |
| gap speed reached (80 → 45 mm) | 480 mm/s | **150 mm/s** |
| palm impact → pads on the cube (gap 41 mm) | 108 ms | **340 ms** |
| contact trigger vs predicted arrival | −2.8 ms | −3.3 ms |
| cube movement in the palm between 20 ms after the impact and the grip | ≤ 6.1 mm (median 3.0) | ≤ 5.3 mm (median 2.7) |
| caught / placed | 30/30 / 30/30 | 30/30 / 30/30 |

The re-run really used the slow gripper, and the catch did not need speed: in v1, B is pre-positioned from A's plan
with the palm square against the arrival, the cube comes to rest in the palm within ~20 ms and waits there for the
fingers. **The implication "v1 depended on the fast gripper" was wrong**; the statement that the v1 gripper was
faster (~3×, 480 vs 150 mm/s) is right. Closing speed matters for the catches from the ready pose (Phase 1 and v2),
where the cube is not cupped. Corrected in `docs/v2_phase1_feasibility.md`, the README and `docs/progress.md`.

### fast_close: live 90 % vs offline 69 % (resolved to 88 % vs 80 %, residual bounded; ~1 h)
`tools/v2_live/fastclose_diag.py`, `tools/v2_live/offline_cells.py`, `tools/v2_live/close_timing.py`:
1. **Throw population (~9 points).** The v2.0 offline number came from the Phase 1 throw distribution
   (`docs/v2/throw_distribution.json`, 213 throws), not from the library A actually throws. On the same 60 library
   entries as the live cell: 78 % (47/60); with the same release-noise draws as live (throw_node's sample_noise,
   seed·1000+i): **80 % (48/60) [68–88]**. Every other cell's offline counterpart is now computed the same way.
2. **Ruled out** (identical or measured equal): MJCF / contact parameters (the offline model is compiled from the
   same scene + physics YAML), timestep (1 ms), B's actuator (fast_close: kp 2000 / kv 10 / ±5 N both), B's arm
   gains (controllers.yaml in both), close timing (gap 40 mm reached +27.6 ms after the arrival offline vs +26.9 ms
   live, median), closing speed reached (706 vs 667–706 mm/s — the ±5 N force limit caps the "1500 mm/s" profile at
   ~700 mm/s in both), intercept time (offline − live: median −4 ms, sd 12 ms), release noise (78 → 80 %), cube spin
   (live 0.96–2.01 rad/s vs planned 1.05–2.09, same axis) and release orientation (2–8° from the offline model).
3. **What decides a fast_close failure, in both**: the cube's rotation relative to the pads at the arrival. Offline,
   failures arrive misaligned by 16.5° (median, 10.8–24.8) vs 7.3° for catches; live, every v2.0 failure arrived at
   7.5–26.5° (median 21°). 5 of the 6 live failures are offline failures too (same throws). A 10× faster close
   catches a misaligned cube between the fingertips before it can settle against the palm (fingertip grasp); the
   realistic 150 mm/s close leaves it the time to square up (1 fingertip grasp in the same 60 throws offline).
4. **Per throw, clean run** (v2.1 cell, every trial recorded; `python3 tools/v2_live/fastclose_compare.py`): live
   **52/59 = 88 % [77–94]**, offline on the same throws 80 % [68–88] — CIs overlap. 52 of 59 throws end the same way.
   Misalignment at the arrival correlates r = 0.92 between live and offline; failures arrive at 16.6° (live) /
   16.5° (offline), catches at 3.1° / 7.5°. The residual 8 points come from the live cube arriving slightly
   squarer (median 2.8° less misaligned: consistent with the 2–8° release-orientation difference between the
   physical throw and the offline model's toss orientation) and the live fingers reaching 40 mm ~1.6 ms earlier
   (median; the live gripper JTC runs at 500 Hz, the offline reference at 1 kHz — cause not isolated): of the 6 throws only live catches, 4 arrive clearly squarer
   (e.g. #15 6° vs 15°) and all 6 close 1.5–3 ms earlier. Not a model bug; the offline model would need A's
   measured release orientation to close it.

## What is still idealized
* **Cameras**: noise-free renders, fixed lighting, ideal calibration (the tracker's 1–2 mm per-fix error is pixel
  quantisation and colour segmentation only); 128.7 Hz with a 5 ms render-to-estimate latency — a real global-shutter
  pair at 120 Hz with exposure, transport and synchronisation would add tens of ms (Phase 1 assumed 25 ms and still
  reached 87 %).
* **Gripper**: the 2F-85 is modelled (150 mm/s, 10 ms command latency from the manual's command rate — not measured on
  hardware); the MuJoCo position actuator adds ~10 ms of ramp lag. Contact (soft MuJoCo contacts, rigid pads) decides
  rebounds; real pads and cube materials differ.
* **Arms**: torque-controlled JTCs at 500 Hz with exact-model feed-forward (a real UR5e takes position/velocity
  targets through its own controller); joint states are exact; B's plan limits (85 % speed, 80 % torque) are checked
  against the same model that simulates it.
* **Release noise** is Gaussian and independent per throw (speed, direction, timing), sized from M4's scatter and
  the plan-informed tolerance — not measured on a real thrower.
* **Compute**: one machine, sim alone for the headline; catch rates drop with load (95 % with three sims).
* Still from v1: pedestals, scene reset after a drop (teleport), ideal palm force sensor.

## Next steps
1. Camera realism: exposure blur, latency and jitter injected into the tracker input (the offline study's 25 ms
   assumption costs ~10 points); then a real stereo pair.
2. Make the close robust to latency error: measure the gripper's latency online (command → finger motion) and
   adapt the schedule; the benchmark shows a 10 ms error in either direction costs 20–40 points under load.
3. Run B's planner and the tracker on a real-time budget (separate cores / C++ planner) so the parallel-load
   degradation disappears.
4. Cup on random throws (today: square-arrival library only) and a learned or tuned contact trigger.
5. Hardware: the UR5e's position interface instead of torque control, re-identified dynamics, the real 2F-85.


