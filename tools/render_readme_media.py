#!/usr/bin/env python3
"""README media from one recorded demo run (catch_tasks.recorder .npz + run_demo's .json event log), offscreen on the
NVIDIA GPU, replayed with tools/render_replay.py (nothing is re-simulated):

  hero.gif              one throw -> catch -> place (A throws at 0.5x, close-up of the flight and catch at 0.25x,
                        color check / in-hand pose / place at 4x); palette GIF via ffmpeg
  catch_strip.png       release -> tracking -> fingers closing -> held
  wrist_view.png        B's wrist camera (the color check's view, rendered at 2x its 320x240) with the color ROI
  pallet_detection.png  overhead cameras over A's pallet (start) and B's pallet (end), with catch_perception's
                        detector (vision.cube_blobs, perception.yaml thresholds) drawn on top

    env -u PYTHONPATH .venv/bin/python tools/render_readme_media.py log/v2demo/run_seed1.npz run_seed1.json \
        [--cycle 0] [--out docs/media] [--gif-size 960x540] [--gif-fps 15]
"""
import argparse
import json
import os
import subprocess
import sys
import tempfile

import cv2
import imageio.v2 as imageio
import mujoco
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from render_replay import REPO, VIEWS, Replay  # noqa: E402

sys.path.insert(0, os.path.join(REPO, "src/catch_perception"))
from catch_perception.vision import cube_blobs  # noqa: E402

VIEWS.update({
    "hero": ([0.0, -0.05, 1.12], 1.72, 90.0, -14.0),     # both arms, both pallets, the flight, from the -y side
    "flight": ([0.16, 0.19, 1.24], 0.8, 70.0, -8.0),     # A's hand to B's hand
    "hand": ([0.40, 0.18, 1.10], 0.42, 60.0, -10.0),     # B's fingers at the catch point
})
DRAW = {"red": (255, 90, 90), "green": (90, 230, 110), "blue": (120, 160, 255)}   # label text on dark


def body_pos(rp, name):
    return rp.d.xpos[mujoco.mj_name2id(rp.m, mujoco.mjtObj.mjOBJ_BODY, name)].copy()


def cycle_times(rec, ev, cycle):
    """(release, cube between B's fingers, place done) of the cycle-th throw, from the ground truth and the log."""
    throws = [e for e in ev if e["state"] == "throw" and "caught" in e]
    seated = [e for e in ev if e["state"] == "load" and "seated" in e["msg"]]
    t_seat, t_res = seated[cycle]["t_sim"], throws[cycle]["t_sim"]
    t_place = next(e["t_sim"] for e in ev if e["state"] == "place" and e["t_sim"] > t_res)
    rp = Replay(rec, 64, 64)
    t_rel = t_in = None
    for t in np.arange(t_seat, t_res, 0.002):
        rp.pin(t)
        mujoco.mj_kinematics(rp.m, rp.d)
        cubes = [body_pos(rp, c) for c in ("cube_red", "cube_green", "cube_blue")]
        a = 0.5 * (body_pos(rp, "a_robotiq_left_pad") + body_pos(rp, "a_robotiq_right_pad"))
        b = 0.5 * (body_pos(rp, "b_robotiq_left_pad") + body_pos(rp, "b_robotiq_right_pad"))
        if t_rel is None:
            k = int(np.argmin([np.linalg.norm(p - a) for p in cubes]))     # the cube in A's hand
        cube = cubes[k]
        if t_rel is None and np.linalg.norm(cube - a) > 0.03:
            t_rel = t
        if t_rel is not None and np.linalg.norm(cube - b) < 0.025:
            t_in = t
            break
    return t_rel, t_in, t_place


def label(img, txt, where="top", scale=0.55):
    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, scale, 1)
    y0 = 8 if where == "top" else img.shape[0] - th - 20
    img[y0:y0 + th + 12, 8:20 + tw] = (img[y0:y0 + th + 12, 8:20 + tw] * 0.3).astype(np.uint8)
    cv2.putText(img, txt, (14, y0 + th + 5), cv2.FONT_HERSHEY_SIMPLEX, scale, (255, 255, 255), 1, cv2.LINE_AA)
    return img


def hero(rec, t_rel, t_in, t_place, out, size, fps):
    w, h = size
    segs = [("hero", t_rel - 0.6, t_rel + 0.12, 0.5, "A throws (0.5x)"),
            ("flight", t_rel + 0.12, t_in + 0.45, 0.25, "B tracks it with two cameras, closes on time (0.25x)"),
            ("hero", t_in + 0.45, t_place + 0.6, 4.0, "color check, in-hand pose, place (4x)")]
    rp = Replay(rec, w, h)
    frames, tp = [], segs[0][1]
    for view, t0, t1, speed, txt in segs:
        for t in np.arange(t0, t1, speed / fps):
            rp.advance(tp, t)
            tp = t
            frames.append(label(np.ascontiguousarray(rp.render(view)), txt))
    frames += [frames[-1]] * int(0.8 * fps)
    with tempfile.TemporaryDirectory(dir=os.path.join(REPO, "log")) as tmp:
        for k, f in enumerate(frames):
            imageio.imwrite(os.path.join(tmp, f"f{k:04d}.png"), f)
        src = ["-framerate", str(fps), "-i", os.path.join(tmp, "f%04d.png")]
        pal = os.path.join(tmp, "palette.png")
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *src, "-vf", "palettegen=max_colors=192:stats_mode=diff",
                        pal], check=True)
        subprocess.run(["ffmpeg", "-y", "-loglevel", "error", *src, "-i", pal, "-lavfi",
                        "paletteuse=dither=bayer:bayer_scale=4:diff_mode=rectangle", "-loop", "0", out], check=True)
    return len(frames) / fps


def strip(rec, t_rel, t_in, out):
    shots = [("flight", t_rel + 0.01, "1  release"), ("flight", t_rel + 0.6 * (t_in - t_rel), "2  tracking (cameras only)"),
             ("hand", t_in - 0.01, "3  fingers closing"), ("hand", t_in + 0.5, "4  held")]
    rp = Replay(rec, 480, 400)
    tiles, tp = [], shots[0][1] - 0.5
    for view, t, txt in shots:
        rp.advance(tp, t)
        tp = t
        tiles.append(label(np.ascontiguousarray(rp.render(view)), txt, "bottom", 0.6))
    gap = np.full((400, 6, 3), 255, np.uint8)
    imageio.imwrite(out, np.concatenate([x for t in tiles for x in (t, gap)][:-1], 1))


def wrist(rec, t, out):
    rp = Replay(rec, 640, 480)
    rp.advance(t - 1.0, t)
    img = np.ascontiguousarray(rp.render("b_robotiq_wrist_cam"))
    u0, v0, u1, v1 = yaml.safe_load(open(os.path.join(REPO, "src/catch_perception/config/perception.yaml")))["color_check"]["roi"]
    cv2.rectangle(img, (int(u0 * 640), int(v0 * 480)), (int(u1 * 640), int(v1 * 480)), (255, 255, 255), 1, cv2.LINE_AA)
    imageio.imwrite(out, label(img, "B's wrist camera, cube in hand (box: color-check region)", "bottom"))


def pallets(rec, t_start, t_end, out):
    """The detector runs on the camera's own 640x480 render; the result is drawn on a 2x render for display."""
    cfg = yaml.safe_load(open(os.path.join(REPO, "src/catch_perception/config/perception.yaml")))
    pd = cfg["pallet_detector"]
    tiles = []
    for cam, t, txt in (("overhead_a", t_start, "overhead camera A: A's pallet at the start"),
                        ("overhead_b", t_end, "overhead camera B: B's pallet at the end")):
        shots = []
        for w, h in ((640, 480), (1280, 960)):
            rp = Replay(rec, w, h)
            rp.advance(t - 1.0, t)
            shots.append(np.ascontiguousarray(rp.render(cam)))
        img = shots[1]
        for b in cube_blobs(shots[0], cfg["colors"], pd["core_px"], pd["min_core_area"]):
            u, v, half = 2 * b.u, 2 * b.v, 2 * (np.sqrt(b.area) / 2 + pd["core_px"])   # core + eroded rim = cube top
            box = cv2.boxPoints(((u, v), (2 * half, 2 * half), np.degrees(b.yaw))).astype(np.int32)
            cv2.polylines(img, [box], True, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.drawMarker(img, (int(u), int(v)), (255, 255, 255), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)
            (tw, th), _ = cv2.getTextSize(b.color, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
            x0, y0 = int(u - tw / 2), int(v + half + 34)
            img[y0 - th - 8:y0 + 8, x0 - 8:x0 + tw + 8] = (30, 30, 30)
            cv2.putText(img, b.color, (x0, y0), cv2.FONT_HERSHEY_SIMPLEX, 0.8, DRAW[b.color], 2, cv2.LINE_AA)
        tiles.append(label(img[290:690, 220:1060].copy(), txt, "top", 0.8))   # the pallet, not the table edge
    gap = np.full((tiles[0].shape[0], 8, 3), 255, np.uint8)
    imageio.imwrite(out, np.concatenate([tiles[0], gap, tiles[1]], 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("rec")
    ap.add_argument("events")
    ap.add_argument("--cycle", type=int, default=0)
    ap.add_argument("--out", default=os.path.join(REPO, "docs/media"))
    ap.add_argument("--gif-size", default="960x540")
    ap.add_argument("--gif-fps", type=int, default=15)
    ap.add_argument("--only", default="hero,strip,wrist,pallets")
    a = ap.parse_args()
    rec = dict(np.load(a.rec, allow_pickle=True))
    ev = json.load(open(a.events))["events"]
    os.makedirs(a.out, exist_ok=True)
    t_rel, t_in, t_place = cycle_times(rec, ev, a.cycle)
    print(f"cycle {a.cycle}: release {t_rel:.3f}, cube between B's fingers {t_in:.3f}, place done {t_place:.3f}")
    only = a.only.split(",")
    if "hero" in only:
        d = hero(rec, t_rel, t_in, t_place, os.path.join(a.out, "hero.gif"),
                 tuple(int(v) for v in a.gif_size.split("x")), a.gif_fps)
        print(f"hero.gif: {d:.1f} s")
    if "strip" in only:
        strip(rec, t_rel, t_in, os.path.join(a.out, "catch_strip.png"))
    if "wrist" in only:
        t_color = next(e["t_sim"] for e in ev if e["state"] == "color" and e["t_sim"] > t_in)
        wrist(rec, t_color, os.path.join(a.out, "wrist_view.png"))
    if "pallets" in only:
        pallets(rec, ev[0]["t_sim"], ev[-1]["t_sim"], os.path.join(a.out, "pallet_detection.png"))


if __name__ == "__main__":
    sys.exit(main())
