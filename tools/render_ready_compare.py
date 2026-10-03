#!/usr/bin/env python3
"""Side-by-side render of B's ready poses (A at home): one column per pose, rows = views (side, front-left, top-ish),
labelled with the IK branch and the elbow height above the shoulder. Arm-configuration review (docs/v2/arm_config/).

    env -u PYTHONPATH .venv/bin/python tools/render_ready_compare.py [--out docs/v2/arm_config/ready_poses.png]
Poses: every entry of src/catch_catcher/config/reactive.yaml ready_poses (+ its wait_q as a last column).
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import yaml  # noqa: E402

import v2_common as V  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=os.path.join(V.OUT, "arm_config", "ready_poses.png"))
    a = ap.parse_args()
    import cv2
    import mujoco
    rc = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_catcher/config/reactive.yaml")))
    poses = yaml.safe_load(open(os.path.join(V.REPO, "src/catch_description/config/poses.yaml")))
    m, scene, _ = V.build_model()
    d = mujoco.MjData(m)
    robot = V.F.make_robots(scene)["b"]
    jid = lambda n: m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
    cols = [(f"{name} ready ({'default' if name == rc['ready_pose'] else 'variant'})", np.array(p["q"]))
            for name, p in rc["ready_poses"].items()]
    cols.append(("B waiting pose (home)", np.array(rc["ready_poses"][rc["ready_pose"]]["wait_q"])))
    views = (([0.55, 0.1, 1.15], 1.35, 90, -5), ([0.55, 0.1, 1.15], 1.25, 40, -22), ([0.55, 0.05, 1.15], 1.4, 150, -30))
    ren = mujoco.Renderer(m, 420, 520)
    out_cols = []
    for label, q in cols:
        for j, n in enumerate(V.JN):
            d.qpos[jid(f"a_{n}")] = poses["a"]["home"][j]
            d.qpos[jid(f"b_{n}")] = q[j]
        mujoco.mj_forward(m, d)
        fr = robot.frames(q)
        rise = fr[2][2, 3] - fr[1][2, 3]
        tiles = []
        for look, dist, az, el in views:
            cam = mujoco.MjvCamera()
            cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = look, dist, az, el
            ren.update_scene(d, camera=cam)
            tiles.append(ren.render().copy())
        col = np.concatenate(tiles, axis=0)
        bar = np.full((70, col.shape[1], 3), 30, np.uint8)
        cv2.putText(bar, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(bar, f"branch {robot.branch(q)}, elbow {100 * rise:+.0f} cm vs shoulder", (10, 58),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 220, 255), 1, cv2.LINE_AA)
        out_cols.append(np.concatenate([bar, col], axis=0))
    ren.close()
    img = np.concatenate(out_cols, axis=1)
    os.makedirs(os.path.dirname(a.out), exist_ok=True)
    cv2.imwrite(a.out, img[:, :, ::-1])
    print("wrote", a.out, img.shape)


if __name__ == "__main__":
    main()
