# What is idealized

Everything runs in simulation. Ground truth (MuJoCo's free-joint state) is used only to *evaluate*; no decision reads
it. These are the places where the simulation is kinder than a real cell, so read them before trusting the numbers.

## Sensing
* **Cameras are ideal renders.** Noise-free, fixed lighting, perfect calibration; the tracker's 1–2 mm per-fix error
  is pixel quantisation and color segmentation only. 128.7 Hz with a 5 ms render-to-estimate latency: a real
  global-shutter stereo pair with exposure, transport and synchronisation would add tens of ms (the offline study
  assumed 25 ms and still caught 87 %). Detection, color and in-hand pose errors here are lower bounds.
* **Palm force sensor** (v1 cup catch only) is a perfect MuJoCo force sensor on a welded palm-insert body; a real
  palm would need a load cell or the gripper's motor current.
* **Joint states are exact** (no encoder noise, ~2.5 ms lag behind the physics).

## Actuation
* **Exact-model torque feed-forward.** A's throw feed-forward torques come from inverse dynamics of *the same*
  MuJoCo model that simulates the arm. Without feed-forward the release was 0.26 m/s (12 %) too fast and 55 mm long;
  on a real robot the model error would show up as release-velocity error.
* **Torque-controlled arms, unlike a real UR.** The joint trajectory controllers command joint *torques* (PD +
  feed-forward) at 500 Hz. A real UR5e takes position/velocity targets through its own controller (or torque via
  the newer direct torque interface) with its own latency and tracking; the throw would need re-tuning on hardware.
  B's plan limits (85 % speed, 80 % torque) are checked against the same model that simulates it.
* **Gripper.** Both grippers are commanded through a 2F-85 model: 150 mm/s gap speed and a 10 ms command latency
  taken from the Robotiq manual's command rate — an estimate, not a hardware measurement. The MuJoCo position
  actuator adds ~10 ms of lag. v2.2's delay probe measures the total from the finger motion, as it would on
  hardware. `fast_close` / `fast_idealized` (NOT a 2F-85) exist only as labelled benchmark cells.

## Throws and contacts
* **Release noise** is Gaussian and independent per throw (speed, direction, timing), sized from the measured v1
  release scatter and the plan-informed tolerance — not measured on a real thrower.
* **Soft contacts.** Tuned MuJoCo contacts (solref/solimp, 1 ms step, elliptic cones, rigid pads) decide whether
  the cube bounces off the fingers or palm; real pads, compliance and cube materials differ.
* **v1's known release state.** v1's B pre-positions from A's published *plan*, which is why v1 needs no vision
  (release scatter ~7 mm/s). v2's B uses the cameras only (enforced by a test).

## Scene and compute
* **Pedestals.** Both arms stand on 0.35 m pedestals: the original geometry (arms on the table, 0.7 m flight)
  needs up to 2.7× the UR5e joint-speed limit.
* **Scene reset.** Recovering from a drop teleports the cube back to A's pallet (sim service); it is not picked up
  from the floor.
* **One machine, sim alone.** Catch rates drop under CPU load: the planner, the tracker and the controllers share
  the machine with the physics. Three sims in parallel cost up to 24 points (v2.0); the headline numbers are with one
  sim alone (real-time factor ≥ 0.996), and a real robot would need the same headroom.
* **Demo noise.** v1 has no injected noise (the seeded randomness is only the cube order); v2 draws random library
  throws with the medium release-noise level (seeded). Drops are rare, so the recovery path is exercised with an
  opt-in, logged fault injection (late grip loosen).
