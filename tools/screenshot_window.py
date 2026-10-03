#!/usr/bin/env python3
"""Screenshot of an X window by name (xdotool + xwd, no ImageMagick) -> PNG (venv: numpy + cv2).

    env -u PYTHONPATH .venv/bin/python tools/screenshot_window.py "RViz" docs/media/rviz.png [--size 1600x900]
"""
import argparse
import struct
import subprocess
import sys

import cv2
import numpy as np


def xwd_to_array(data):
    h = struct.unpack(">25I", data[:100])
    header_size, ncolors = h[0], h[19]
    width, height, byte_order, bpp, bpl = h[4], h[5], h[7], h[11], h[12]
    nb = bpp // 8
    if nb not in (3, 4) or bpl < nb * width:
        raise SystemExit(f"unsupported xwd: {bpp} bits per pixel, {bpl} bytes per line for width {width}")
    off = header_size + 12 * ncolors
    rows = np.frombuffer(data[off:off + bpl * height], np.uint8).reshape(height, bpl)
    px = rows[:, :width * nb].reshape(height, width, nb)    # packed pixels, lines padded to bytes_per_line
    if byte_order == 1:                 # MSBFirst: (x)RGB
        px = px[:, :, ::-1]
    return px[:, :, :3].copy()          # LSBFirst: B, G, R(, x) in memory = cv2 order


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("out")
    ap.add_argument("--size", default="")
    a = ap.parse_args()
    wids = subprocess.check_output(["xdotool", "search", "--onlyvisible", "--name", a.name]).decode().split()
    if not wids:
        sys.exit(f"no visible window named {a.name!r}")
    wid = wids[-1]
    if a.size:
        w, h = a.size.split("x")
        subprocess.call(["xdotool", "windowsize", "--sync", wid, w, h])
    subprocess.call(["xdotool", "windowmove", wid, "0", "0"])     # fully on screen if the WM allows (no --sync: Wayland ignores moves)
    subprocess.call(["xdotool", "windowactivate", "--sync", wid])
    import time
    time.sleep(1.5)                     # let the application repaint after a resize / raise
    data = subprocess.check_output(["xwd", "-silent", "-id", wid])
    img = xwd_to_array(data)
    cv2.imwrite(a.out, img)
    print(f"{a.out}: {img.shape[1]}x{img.shape[0]} (window {wid})")


if __name__ == "__main__":
    main()
