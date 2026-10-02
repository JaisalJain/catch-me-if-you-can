#!/usr/bin/env python3
"""MJCF variants of the scene with another gripper actuator (benchmark profiles, v1 comparison).

    env -u PYTHONPATH .venv/bin/python tools/v2_live/mjcf_variant.py {v1|fast_close|fast_idealized} [--out log/mjcf]
-> log/mjcf/<profile>/scene.xml (+ assets); launch with  mujoco_model:=<abs path>  and CATCH_GRIPPER_PROFILE=<profile>.
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
    ap.add_argument("--out", default=os.path.join(REPO, "log", "mjcf"))
    a = ap.parse_args()
    scene, phys = build_mjcf.load_cfg()
    phys = copy.deepcopy(phys)
    kp, kv, fs = PROFILES[a.profile]
    phys["gripper"] = dict(kp=kp, kv=kv, force_scale=fs)
    out = os.path.join(a.out, a.profile)
    build_mjcf.generate(out_dir=out, verbose=True, scene=scene, phys=phys)
    print(os.path.join(out, "scene.xml"))


if __name__ == "__main__":
    main()
