# M3 — Perception, pick and place

## Overhead detection (`catch_perception/pallet_detector`)
HSV segmentation per colour (`config/perception.yaml`) → distance-transform core (pixels > 7 px inside the
mask: a cube top is ~48 px wide, its same-coloured slot border ~10 px, so borders and side-face slivers drop
out) → centroid + min-area-rect yaw → ray from the camera (intrinsics from `camera_info`, extrinsics from the
`camera_tf` static TFs) intersected with the plane of the cube tops (pallet plane if the hit lies on a pallet,
table plane otherwise). Confidence = visible core area / core area of an unoccluded top at that depth.
Published as `CubeDetectionArray` on `/perception/overhead_{a,b}/cubes` (world frame, 10 Hz).

Verified chain: camera_info K = fovy formula (f = 579.4 px at 45°, 836 px at 32°); ground-truth cubes
projected through TF + K land on pixels of their own colour.

**Accuracy vs MuJoCo ground truth** (`ros2 run catch_perception eval_detection`, 20 layouts = 60 cubes,
pallets at x = −0.45): 10 layouts with cubes in their slots (±5 mm offset, ±15° yaw), 10 with cubes loose on
the table (any yaw):

| | n | xy mean | xy p95 | xy max | yaw mean | yaw max |
|---|---|---|---|---|---|---|
| all | 60 | **1.74 mm** | 3.79 mm | 4.66 mm | 0.40° | 2.74° |
| in slots | 30 | 1.46 mm | 3.74 mm | 4.66 mm | 0.62° | 2.74° |
| loose | 30 | 2.01 mm | 3.78 mm | 3.96 mm | 0.17° | 0.65° |

0 misses, 0 false detections. Two bugs found on the way: a fixed wall-clock wait read stale frames (now the
evaluation waits for a detection stamped ≥ 1 s sim time after the teleport), and loose cubes were projected
on the pallet plane (3–7 mm error, fixed by the table-plane fallback).

Visual-only change: the scene was over-exposed (cube red rendered as (255,101,102)); lights reduced and made
configurable (`scene.yaml: lighting`), overhead cameras narrowed to 32° → red (223,26,26), green (30,198,43),
blue (53,121,255).

## Colour check at B (`/perception/get_color`)
Dominant-colour fraction in a central ROI of B's wrist camera (`b_robotiq_wrist_cam`, on a bracket on the
gripper looking at the palm). Uses only that image; A's colour is never passed to B. With a cube held palm-up:
**3/3 correct**, winning fraction 0.40, other colours 0.00.

## Pick / place (`catch_tasks/skills.py`, `config/skills.yaml`)
* pick: closed-form IK joint goal (collision-checked by MoveIt) to 10 cm above the detected cube, gripper down,
  finger axis aligned with a cube face (the 90°-equivalent yaw closest to the current wrist) → straight descent
  → close (command 0.70, force-limited) → "in hand" iff knuckle ∈ [0.38, 0.55] → straight lift → the cube is
  attached to `<p>_tcp` in MoveIt's planning scene.
* place: IK joint goal above the slot → straight descent → open → retreat. Release height from geometry: the
  closed pads reach 33 mm past the TCP; cube bottom ≥ max(10 mm, fingertip overhang + border 6 mm + 2 mm).

**Test** (`ros2 run catch_tasks pickplace_test`):

| | red | green | blue |
|---|---|---|---|
| B: colour seen (wrist cam) | red ✓ | green ✓ | blue ✓ |
| B: place error / in slot | 0.63 mm ✓ | 0.39 mm ✓ | 0.44 mm ✓ |
| A: detection error | 2.24 mm | 0.55 mm | 3.37 mm |
| A: pick (knuckle) | ✓ 0.450 | ✓ 0.450 | ✓ 0.450 |
| A: place back error / in slot | 2.04 mm ✓ | 0.11 mm ✓ | 3.82 mm ✓ |

(B: cube spawned into B's palm-up gripper first, as the brief asks. Tilt after placing 0.0° in all cases.)

## Problems found and fixed (in order)
1. Top grasp at 5 mm TCP offset: the 2F-85's pads move ~11 mm *towards the fingertips* while closing, so they
   hit the slot borders (knuckle stalled at 0.17). `tools/pick_geometry.py` (real MuJoCo gripper) → **17 mm**.
2. With the 0.35 m pedestals the pallets were almost under the bases; moved to x = ∓0.45 (joint margin over all
   slots 0.57 → 0.72 rad).
3. The URDF pallet was one solid 18 mm slab; now base + 4 border bars per slot (same boxes as the MJCF).
4. MoveIt pose goals (KDL IK sampling) failed intermittently near the pedestals → joint goals from the
   closed-form IK (8 branches, closest first), still collision-checked by MoveIt.
5. The URDF (robotiq_description) fingertips reach ~4 mm further than the Menagerie pads, so MoveIt reported
   finger/border contacts that do not exist in the sim. The short vertical in-slot moves run with MoveIt
   collision checking off; their clearance is guaranteed by the release-height formula and checked in MuJoCo.
6. Detection with the arm hovering over the pallet partially occludes cubes (14 mm error once): the arm goes
   home before detecting, and low-confidence detections (< 0.7) are rejected.
