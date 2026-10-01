#!/usr/bin/env python3
"""Render a recorded sim run (catch_tasks.recorder .npz: /joint_states + cube ground truth) to mp4, offscreen
on the NVIDIA GPU (EGL), with the same MJCF the ROS sim loads.

The arm joints and the cubes are pinned to the recorded states every physics step; only the gripper linkages
are simulated (their actuators follow the recorded knuckle angle), so the fingers sit on the cube exactly as
the linkage/contact model puts them. Nothing here changes what happened in the run — it is a replay.

    env -u PYTHONPATH .venv/bin/python tools/render_replay.py log/m5/force/trial_07.npz --out docs/media/x.mp4 \
        [--view catch|side|overview] [--window catch|all|T0,T1] [--speed 0.25] [--fps 30] [--size 960x544] [--label ...]
"""
import argparse
import json
import os
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
os.environ.setdefault("__EGL_VENDOR_LIBRARY_FILENAMES", "/usr/share/glvnd/egl_vendor.d/10_nvidia.json")

import cv2  # noqa: E402
import imageio.v2 as imageio  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CUBES = ("cube_red", "cube_green", "cube_blue")
VIEWS = {   # free cameras: lookat, distance, azimuth [deg] (camera looks along (cos az, sin az) in xy), elevation
    "catch": ([0.33, 0.22, 1.12], 0.95, 62.0, -14.0),        # close-up of B's cup, seen from the -y side, A behind
    "overview": ([0.0, -0.08, 0.98], 2.6, 80.0, -22.0),      # whole table from the -y side (A left, B right)
    "throw": ([0.12, 0.2, 1.15], 1.45, 90.0, -8.0),          # flight path from the -y side
}


def interp_rows(t, ts, rows):
    i = int(np.clip(np.searchsorted(ts, t), 1, len(ts) - 1))
    a = (t - ts[i - 1]) / max(ts[i] - ts[i - 1], 1e-9)
    a = float(np.clip(a, 0.0, 1.0))
    return (1 - a) * rows[i - 1] + a * rows[i], i, a


class Replay:
    def __init__(self, rec, width, height):
        self.m = mujoco.MjModel.from_xml_path(os.path.join(REPO, "src/catch_description/mjcf/scene.xml"))
        m = self.m
        m.vis.global_.offwidth, m.vis.global_.offheight = max(width, 640), max(height, 480)
        self.d = mujoco.MjData(m)
        self.ren = mujoco.Renderer(m, height, width)
        self.t_js, self.q, self.names = rec["t_js"], rec["q"], [str(n) for n in rec["names"]]
        self.cubes = rec["cubes"]
        self.jq, self.act = [], []
        for k, n in enumerate(self.names):
            jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
            if "knuckle" in n:
                self.act.append((k, mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)))
            elif jid >= 0:
                self.jq.append((k, m.jnt_qposadr[jid], m.jnt_dofadr[jid]))
        self.cq = []
        for c in CUBES:
            j = m.body_jntadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, c)]
            self.cq.append((m.jnt_qposadr[j], m.jnt_dofadr[j]))
        self.started = False

    def pin(self, t):
        d = self.d
        q, _, _ = interp_rows(t, self.t_js, self.q)
        for k, qa, da in self.jq:
            d.qpos[qa], d.qvel[da] = q[k], 0.0
        for k, a in self.act:
            d.ctrl[a] = q[k]
        if len(self.cubes):
            row, i, a = interp_rows(t, self.cubes[:, 0], self.cubes[:, 1:])
            for n, (qa, da) in enumerate(self.cq):
                p = row[7 * n:7 * n + 3]
                q0, q1 = self.cubes[i - 1, 4 + 7 * n:8 + 7 * n], self.cubes[i, 4 + 7 * n:8 + 7 * n]
                if np.dot(q0, q1) < 0:
                    q1 = -q1
                qq = (1 - a) * q0 + a * q1
                d.qpos[qa:qa + 3], d.qpos[qa + 3:qa + 7] = p, qq / np.linalg.norm(qq)
                d.qvel[da:da + 6] = 0.0

    def advance(self, t_prev, t):
        m, d = self.m, self.d
        if not self.started:
            self.pin(t)
            mujoco.mj_forward(m, d)
            for _ in range(400):                        # let the gripper linkages settle at the first frame
                self.pin(t)
                mujoco.mj_step(m, d)
            self.started = True
        n = max(2, int(round((t - t_prev) / m.opt.timestep)))
        for s in range(1, n + 1):
            self.pin(t_prev + (t - t_prev) * s / n)
            mujoco.mj_step(m, d)
        self.pin(t)
        mujoco.mj_forward(m, d)

    def render(self, view):
        if view in VIEWS:
            cam = mujoco.MjvCamera()
            cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            cam.lookat[:], cam.distance, cam.azimuth, cam.elevation = VIEWS[view]
            self.ren.update_scene(self.d, camera=cam)
        else:
            self.ren.update_scene(self.d, camera=view)
        return self.ren.render()


def catch_time(rec):
    """Time of the first palm-force spike (cube hits B's palm) or, without one, of the cube's closest approach
    to the catch point."""
    ft = rec["ft"]
    if len(ft):
        f = np.linalg.norm(ft[:, 1:], axis=1)
        f0 = np.median(f[:50])
        hit = np.where(np.abs(f - f0) > 8.0)[0]
        if len(hit):
            return float(ft[hit[0], 0])
    c = rec["cubes"]
    best = None
    for n in range(3):
        p = c[:, 1 + 7 * n:4 + 7 * n]
        dist = np.linalg.norm(p - [0.35, 0.25, 1.15], axis=1)
        k = int(np.argmin(dist))
        if best is None or dist[k] < best[0]:
            best = (dist[k], float(c[k, 0]))
    return best[1]


def render(rec, out, view="catch", window="catch", speed=0.25, fps=30, size=(960, 544), label="", t_marks=()):
    w, h = size
    rp = Replay(rec, w, h)
    t_first, t_last = float(rec["t_js"][0]), float(rec["t_js"][-1])
    if window == "catch":
        tc = catch_time(rec)
        t0, t1 = max(t_first, tc - 0.55), min(t_last, tc + 0.9)
    elif window == "all":
        t0, t1 = t_first, t_last
    else:
        a, b = [float(v) for v in window.split(",")]
        t0, t1 = t_first + a, t_first + b
    dt = speed / fps
    ts = np.arange(t0, t1, dt)
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    wr = imageio.get_writer(out, fps=fps, codec="libx264", quality=8, macro_block_size=16, pixelformat="yuv420p")
    t_prev = ts[0]
    for t in ts:
        rp.advance(t_prev, t)
        t_prev = t
        img = np.ascontiguousarray(rp.render(view))
        txt = f"{label}  t={t - t0:5.2f} s" + (f"  ({speed:g}x)" if speed != 1 else "")
        for mt, msg in t_marks:
            if t >= mt:
                txt += f"  {msg}"
        (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        img[6:16 + th, 6:18 + tw] = (img[6:16 + th, 6:18 + tw] * 0.35).astype(np.uint8)
        cv2.putText(img, txt, (12, 10 + th), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        wr.append_data(img)
    wr.close()
    return out, len(ts)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("--out", required=True)
    ap.add_argument("--view", default="catch")
    ap.add_argument("--window", default="catch")
    ap.add_argument("--speed", type=float, default=0.25)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--size", default="960x544")       # multiple of 8/16 for H.264
    ap.add_argument("--label", default="")
    a = ap.parse_args()
    rec = dict(np.load(a.rec, allow_pickle=True))
    label = a.label
    if not label and "meta" in rec:
        meta = json.loads(str(rec["meta"]))
        label = f"trial {meta.get('i')} {meta.get('color')}"
    w, h = [int(v) for v in a.size.split("x")]
    out, n = render(rec, a.out, a.view, a.window, a.speed, a.fps, (w, h), label)
    print(f"wrote {out}: {n} frames")


if __name__ == "__main__":
    sys.exit(main())
