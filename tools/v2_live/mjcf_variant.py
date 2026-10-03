#!/usr/bin/env python3
"""MJCF variants of the scene with another gripper actuator (benchmark profiles, v1 comparison).

    env -u PYTHONPATH .venv/bin/python tools/v2_live/mjcf_variant.py {v1|fast_close|fast_idealized} [--arms ab|b] [--out log/mjcf]
-> log/mjcf/<profile>[_b]/scene.xml (+ assets); launch with  mujoco_model:=<abs path>  and CATCH_GRIPPER_PROFILE=<profile>
(+ CATCH_GRIPPER_ARMS=b for --arms b: only B's gripper differs, A throws with the default gripper - the benchmark).
Actuator values (v1 base kp 100, kv 10, force +-5; tools/v2_study.yaml profiles):
  v1             kp 100,  kv 10, force x1  (the v1 sim gripper; command model: step, 0 latency)
  realistic      kp 1000, kv 10, force x1  (= physics.yaml, the default scene.xml)
  fast_close     kp 2000, kv 10, force x1  (NOT a 2F-85: fast, realistic force)
  fast_idealized kp 2000, kv 10, force x5  (NOT a 2F-85: fast and 5x force)
"""
import argparse
import copy
import os
import sys

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "src/catch_description/scripts"))
import build_mjcf  # noqa: E402

PROFILES = {"v1": (100.0, 10.0, 1.0), "realistic": (1000.0, 10.0, 1.0), "fast_close": (2000.0, 10.0, 1.0),
            "fast_idealized": (2000.0, 10.0, 5.0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("profile", choices=list(PROFILES))
    ap.add_argument("--arms", default="ab", choices=["ab", "b"])
    ap.add_argument("--out", default=os.path.join(REPO, "log", "mjcf"))
    a = ap.parse_args()
    scene, phys = build_mjcf.load_cfg()
    phys = copy.deepcopy(phys)
    kp, kv, fs = PROFILES[a.profile]
    phys["gripper"] = dict(kp=kp, kv=kv, force_scale=fs)
    out = os.path.join(a.out, a.profile + ("_b" if a.arms == "b" else ""))
    build_mjcf.generate(out_dir=out, verbose=True, scene=scene, phys=phys)
    if a.arms == "b":
        # A's gripper actuator back to the default scene's (only the catching gripper varies)
        default = open(os.path.join(REPO, "src/catch_description/mjcf/scene.xml")).read().splitlines()
        line_a = [ln for ln in default if 'name="a_robotiq_85_left_knuckle_joint"' in ln and "<general" in ln]
        assert len(line_a) == 1, line_a
        p = os.path.join(out, "scene.xml")
        lines = open(p).read().splitlines()
        idx = [i for i, ln in enumerate(lines) if 'name="a_robotiq_85_left_knuckle_joint"' in ln and "<general" in ln]
        assert len(idx) == 1, idx
        lines[idx[0]] = line_a[0]
        open(p, "w").write("\n".join(lines) + "\n")
    print(os.path.join(out, "scene.xml"))


if __name__ == "__main__":
    main()
