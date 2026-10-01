# M6 — Full loop and demo

## The state machine (`catch_tasks/state_machine.py`, `ros2 run catch_tasks run_demo --seed N`)
One plain-Python loop over explicit states; every transition is logged with sim time (`docs/m6/run_seed*.json`).

```
DETECT ──► PICK ──► LOAD ──► PREPOSITION ──► THROW ──► COLOR ──► IN_HAND ──► PLACE ──► VERIFY ──► DETECT …
  │ A's pallet empty                │ B not ready   │ not caught  │ no color             │ not in slot
  ▼                                 ▼               ▼             ▼                      ▼
 DONE                    RECOVER: log it, open both grippers, both arms home, reset that cube to its slot on
                         A's pallet (sim teleport), back to DETECT (≤ 3 attempts per cube)
```

| State | What happens | Sensing used |
|---|---|---|
| DETECT | fresh overhead-A frame; cubes on A's pallet; random choice (seeded, `random.Random("catch-demo-<seed>")`) | overhead camera A |
| PICK | top-down MoveIt pick at the detected pose/yaw (2 tries) | detection, knuckle angle |
| LOAD | palm-up, loosen so the cube drops onto the palm insert, re-grip, MoveIt to the stroke start | knuckle angle |
| PREPOSITION | `/catch` goal with the throw-plan preview; wait for "ready" (B in its cup pose, palm-force stream live) | joint states |
| THROW | `/throw` (plan latched on `/throw_plan`), wait for both results; A home | — / palm force (B) |
| COLOR | B's wrist camera (A's color logged as a cross-check only) | wrist camera |
| IN_HAND | wrist-camera in-hand pose (rotation about the closing axis + offset) | wrist camera |
| PLACE | carry via home, corrected top-down place into that color's slot on B's pallet | knuckle angle |
| VERIFY | overhead-B frame (up to 3): cube of that color within 6 mm of its slot | overhead camera B |
| RECOVER | see diagram | — |

The state machine never reads ground truth. Ground truth is used afterwards to check it
(`tools/analyze_runs.py`, on rosbag2 recordings of joint states, cube ground truth and palm force).

Fault injection for the recovery path (off by default, logged as `FAULT INJECTED` in the run log):
`--fault-throw N` makes the throw node loosen the grip late (0.26 s instead of 0.12 s) on throw N — the real
failure mode found in M5 — so the cube misses B's palm.

## Results
Three clean runs with different seeds plus one run with an injected fault (`tools/run_m6.sh 1 2 3`, `FAULT="--fault-throw 1" tools/run_m6.sh 4`); logs `docs/m6/run_seed*.json`, ground-truth check `docs/m6/gt_check.jsonl`.

| Seed | Throw order (random) | Throws | Drops → recoveries | Duration (sim) | All 3 sorted | VERIFY vs ground truth | Final cube offset in slot (x, ground truth) |
|---|---|---|---|---|---|---|---|
| 1 | red → green → blue | 3 | 0 | 96 s | yes | 3/3 agree | R -0.79, G -0.67, B -0.61 mm |
| 2 | green → blue → red | 3 | 0 | 98 s | yes | 3/3 agree | R -0.75, G -0.70, B -0.67 mm |
| 3 | blue → red → green | 3 | 0 | 99 s | yes | 3/3 agree | R -0.87, G -0.66, B -0.59 mm |
| 4 (fault) | blue → green → blue → red | 4 | 1 (injected fault: missed palm) → reset, re-thrown, caught | 124 s | yes | 3/3 agree | R -0.84, G -0.70, B -0.60 mm |

All 12 catches were contact-triggered (-3.3 … -2.3 ms vs predicted arrival); B's color agreed with A's label every time (cross-check only); RTF 1.000 in every run (rtf_monitor, 30 s intervals; one reading 0.999, one 1.001).

## Demo video
[`media/demo_seed1.mp4`](media/demo_seed1.mp4) — seed 1, the full loop in 66 s of video (96 s of sim time)  
[`media/demo_recovery_seed4.mp4`](media/demo_recovery_seed4.mp4) — the run with the injected fault: the miss, the
recovery and the re-throw.

Both are offscreen MuJoCo renders (NVIDIA EGL) of the recorded run (`tools/render_demo.py`): arm joints and cube
poses replayed from the rosbag2 recording, gripper linkages re-simulated from the recorded knuckle angle, the
state machine's log as captions. Overview at 2x between throws, real time around each throw, every catch
repeated as a 0.25x close-up. Re-encoded with `ffmpeg -c:v libx264 -crf 27 -preset slow` for size.

## Notes
* Recording: an rclpy recorder could not keep up (≈ 5 % of the samples, 0.5 s gaps, data ending seconds early);
  `ros2 bag record` (C++, mcap) loses 0–9 messages per run. Stop it with SIGTERM: background jobs of a
  non-interactive shell ignore SIGINT.
* With integer seeds 1, 2, 3 Python's `random.Random` picked the same order (blue, green, red); the seed is now a
  hashed string.
* RTF 1.000 in all runs (rtf_monitor every 30 s).
